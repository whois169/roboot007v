// v0.5 渲染：node shots_v5.mjs <viewer.html> <输出目录>   （需要 node_modules 里有 three 和 playwright）
import { chromium } from "playwright";
import fs from "fs";
const NM = process.cwd() + "/node_modules/three/";
const [page_path, outdir] = process.argv.slice(2);
const b = await chromium.launch({ executablePath: "/opt/pw-browsers/chromium-1194/chrome-linux/chrome", args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"] });
const ctx = await b.newContext({ viewport: { width: 1280, height: 900 }, deviceScaleFactor: 1.5 });
await ctx.route(/fonts\.(googleapis|gstatic)\.com/, r => r.abort());
await ctx.route(/cdn\.jsdelivr\.net\/npm\/three@0\.160\.0\/(.*)$/, r => {
  r.fulfill({ body: fs.readFileSync(NM + r.request().url().match(/three@0\.160\.0\/(.*)$/)[1]), contentType: "application/javascript" });
});
const page = await ctx.newPage();
page.on("pageerror", e => console.log("pageerror:", e.message));
await page.goto("file://" + page_path);
await page.waitForFunction(() => window.__ready === true, null, { timeout: 180000 });

// 标签：[文字, 刚体组, 机身坐标(mm, 零位)]
const X = 188, YO = 90;   // 左前腿 HFE 轴 x、HFE 输出面 y
const SHOTS = [
  { name: "v5_stand", title: "站立（前 45°）", pose: "stand", cam: [0.95, 0.60, 0.95], tgt: [0, 0.25, 0], shell: true },
  { name: "v5_side", title: "侧面", pose: "stand", cam: [1.75, 0.32, 0.02], tgt: [0, 0.26, 0.0], shell: true },
  { name: "v5_top", title: "俯视", pose: "stand", cam: [0.02, 1.85, 0.12], tgt: [0, 0.28, 0.0], shell: true },
  { name: "v5_rear", title: "后 45°", pose: "stand", cam: [0.95, 0.66, -0.95], tgt: [0, 0.27, -0.02], shell: true },
  { name: "v5_look", title: "转头 35°、抬头 12°", pose: "look", cam: [0.55, 0.62, 1.05], tgt: [0.02, 0.33, 0.1], shell: true },
  { name: "v5_sit", title: "坐下", pose: "sit", cam: [1.25, 0.42, 0.55], tgt: [0, 0.22, 0], shell: true },
  { name: "v5_lie", title: "趴下", pose: "lie", cam: [1.05, 0.62, 0.95], tgt: [0, 0.08, 0], shell: true },
  { name: "v5_face", title: "前脸：银色面板 + 黑色传感器面板（热成像、红外都直接开孔）", pose: "stand", focus: ["neck_pitch", [252, 0, 102]], off: [0.12, 0.04, 0.25], shell: true,
    labels: [["2.25 寸长条屏（青色 HUD 眼睛）", "neck_pitch", [250, -10, 108]], ["银色前脸面板", "neck_pitch", [252, 41, 110]],
             ["MLX90640 热成像（开孔，不能挡亚克力）", "neck_pitch", [252, 26, 88]], ["红外发射（遥控空调电视）", "neck_pitch", [252, 35, 88]],
             ["相机镜头", "neck_pitch", [252, 0, 88]], ["VL53L5CX 测距", "neck_pitch", [252, -16, 88]],
             ["红外接收（学习遥控器）", "neck_pitch", [252, -34, 88]], ["隐私指示灯", "neck_pitch", [252, 12, 88]],
             ["毫米波藏在这条缝后面", "neck_pitch", [252, 10, 120]]] },
  { name: "v5_leg", title: "腿：深灰装甲 + 银色端盖 + 银色小腿护甲 + 髋部状态灯", pose: "stand", focus: ["FL_hfe", [188, 150, -130]], off: [0.46, 0.06, 0.30], shell: true,
    labels: [["髋罩状态灯（着地 / 故障 / 方向）", "FL_haa", [228, 89, 0]], ["银色髋端盖（盖住轴承）", "FL_hfe", [188, 186, 10]],
             ["大腿装甲：凸起中板 + 大斜切 + 缝线", "FL_hfe", [182, 185, -95]], ["张紧轮长孔：不拆装甲也能调皮带", "FL_hfe", [163, 185, -70]],
             ["银色膝端盖", "FL_hfe", [188, 186, -189]], ["银色小腿护甲（加宽到和小腿一样宽）", "FL_knee", [188, 141, -290]]] },
  { name: "v5_back", title: "背部：任务导轨（兼提手）+ 激光雷达模块 + 示范任务箱", pose: "stand", cam: [0.62, 0.92, -0.42], tgt: [0.0, 0.42, 0.0], shell: true,
    labels: [["单线激光雷达（Φ60 占位，扫描面高过任务箱）", "body", [46, 0, 134]], ["琥珀橙灯环", "body", [46, 26, 99]],
             ["任务导轨：立柱锁进 2020 型材，兼提手", "body", [-40, 72, 89]], ["示范任务箱（4 个快拆夹）", "body", [-36, -56, 108]],
             ["任务接口 GX16（24V / 5V / CAN）", "body", [-78, -34, 72]], ["机脊状态灯（分段）", "body", [-47, 75, 58]],
             ["任务箱相机窗", "body", [8, 10, 96]]] },
  { name: "v5_leg_inside", title: "膝关节皮带（拆下装甲和外壳）：v0.5 修正了皮带长度", pose: "stand", cam: [0.72, 0.30, 0.55], tgt: [0.12, 0.19, 0.13], shell: false, thigh: true,
    labels: [["20T 小带轮", "FL_hfe", [X, YO + 70, 0]], ["104 齿 HTD-5M 皮带（520mm）", "FL_hfe", [X - 20, YO + 70, -120]],
             ["张紧轮：名义压进 6.9mm，长槽 ±6mm", "FL_hfe", [X - 25, YO + 70, -70]], ["36T 大带轮（1:1.8）", "FL_knee", [X, YO + 70, -189]],
             ["大腿轴距 189.3mm（v0.4 是 190）", "FL_hfe", [X, YO + 45, -138]]] },
  { name: "v5_head", title: "头部内部（拆下头壳）", pose: "stand", cam: [0.50, 0.60, 0.48], tgt: [0, 0.40, 0.17], shell: false,
    labels: [["USB 相机镜头模组", "neck_pitch", [244, 0, 93]], ["MLX90640 + 转接板", "neck_pitch", [245, 26, 90]],
             ["红外发射管", "neck_pitch", [244, 35, 88]], ["红外接收头", "neck_pitch", [246, -34, 91]],
             ["VL53L5CX 测距", "neck_pitch", [246, -16, 94]], ["LD2410 毫米波（屏幕上方）", "neck_pitch", [246, 8, 123]],
             ["2.25 寸长条彩屏", "neck_pitch", [246, 0, 115]], ["ESP32-S3 + USB 集线器", "neck_pitch", [130, 10, 108]],
             ["俯仰电机 GM4108", "neck_yaw", [184, 22, 96]], ["偏航电机 GM3506", "body", [140, 0, 62]]] },
];
const STUDIO = true;   // 浅灰摄影棚背景（和参考图一致），关掉网格
for (const s of SHOTS) {
  await page.evaluate(s => {
    const v = window.__v;
    v.setPoseNow(s.pose);
    v.vis.shell = s.shell; v.vis.elec = s.elec !== false; v.vis.thigh = !s.thigh; v.refresh();
    v.setGhost(!!s.ghost);
    if (s.focus) {
      const t = v.worldOf(...s.focus);
      v.controls.target.copy(t); v.camera.position.set(t.x + s.off[0], t.y + s.off[1], t.z + s.off[2]);
    } else { v.camera.position.set(...s.cam); v.controls.target.set(...s.tgt); }
    v.controls.update();
    document.querySelector(".toolbar").style.display = "none";
    if (s.studio !== false) {
      document.querySelector(".stage").style.background = "radial-gradient(ellipse at 50% 35%, #e4e6e9 0%, #c3c6ca 70%, #b3b6ba 100%)";
      v.scene.traverse(o => { if (o.type === "GridHelper") o.visible = false; });
    }
    document.querySelector(".hint").style.display = "none";
    document.getElementById("ov")?.remove();
  }, s);
  await page.waitForTimeout(1500);
  await page.evaluate(s => {
    const v = window.__v, st = document.querySelector(".stage"), cv = document.getElementById("cv");
    const W = cv.clientWidth, H = cv.clientHeight;
    v.controls.update(); v.camera.updateMatrixWorld(true); v.renderer.render(v.scene, v.camera);
    const ns = "http://www.w3.org/2000/svg";
    const svg = document.createElementNS(ns, "svg");
    svg.id = "ov"; svg.setAttribute("width", W); svg.setAttribute("height", H);
    svg.style.cssText = "position:absolute;left:0;top:0;pointer-events:none;font-family:'Noto Sans SC',sans-serif";
    const add = (tag, attrs) => { const e = document.createElementNS(ns, tag); Object.entries(attrs).forEach(([k, val]) => e.setAttribute(k, val)); svg.appendChild(e); return e; };
    add("text", { x: 22, y: 40, "font-size": 20, "font-weight": 700, fill: s.studio !== false ? "#1d2024" : "#e9edf1" }).textContent = "RoboDog v0.5 · " + s.title;
    const items = (s.labels || []).map(([txt, g, p]) => {
      const q = v.worldOf(g, p).project(v.camera);
      return { txt, x: (q.x + 1) / 2 * W, y: (1 - q.y) / 2 * H, side: q.x < 0 ? "L" : "R",
               w: [...txt].reduce((a, c) => a + (c.charCodeAt(0) > 255 ? 14 : 8), 20) };
    });
    for (const side of ["L", "R"]) {
      const col = items.filter(i => i.side === side).sort((a, b) => a.y - b.y);
      let prev = 60;
      for (const i of col) { i.ly = Math.max(i.y, prev + 38); prev = i.ly; }
      const over = prev - (H - 24);
      if (over > 0) col.forEach(i => i.ly -= over);
      for (const i of col) {
        const bx = side === "L" ? 24 : W - 24 - i.w, ex = side === "L" ? bx + i.w : bx, knee = side === "L" ? ex + 26 : ex - 26;
        add("polyline", { points: `${ex},${i.ly} ${knee},${i.ly} ${i.x},${i.y}`, fill: "none", stroke: "#e85d12", "stroke-width": 1.5 });
        add("circle", { cx: i.x, cy: i.y, r: 4.5, fill: "#e85d12", stroke: "#fff", "stroke-width": 1.5 });
        add("rect", { x: bx, y: i.ly - 14, width: i.w, height: 28, rx: 3, fill: "#f2f4f6" });
        add("text", { x: bx + 10, y: i.ly + 5, "font-size": 13.5, fill: "#15181c" }).textContent = i.txt;
      }
    }
    st.appendChild(svg);
  }, s);
  await (await page.$(".stage")).screenshot({ path: `${outdir}/${s.name}.png` });
  console.log("shot", s.name);
}
await b.close();
