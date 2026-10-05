// v0.3 渲染：node shots_v3.mjs <viewer.html> <输出目录>   （需要 node_modules 里有 three 和 playwright）
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
  { name: "v3_stand", title: "站立", pose: "stand", cam: [0.95, 0.60, 0.95], tgt: [0, 0.25, 0], shell: true },
  { name: "v3_look", title: "转头 35°、抬头 12°", pose: "look", cam: [0.55, 0.62, 1.05], tgt: [0.02, 0.33, 0.1], shell: true },
  { name: "v3_sit", title: "坐下", pose: "sit", cam: [1.25, 0.42, 0.55], tgt: [0, 0.22, 0], shell: true },
  { name: "v3_lie", title: "趴下（前腿前伸、后腿后伸，可断电休息）", pose: "lie", cam: [1.05, 0.62, 0.95], tgt: [0, 0.08, 0], shell: true },
  { name: "v3_foot", title: "加粗小腿 + 防滑足端", pose: "stand", focus: ["FL_knee", [X, YO + 32, -330]], off: [0.30, 0.06, 0.22], shell: true,
    labels: [["小腿（38→24mm 宽、32mm 厚）", "FL_knee", [X, YO + 32, -250]], ["白色小腿护甲", "FL_knee", [X, YO + 51, -200]],
             ["TPU 95A 内芯（M4 横销）", "FL_knee", [X + 12, YO + 32, -374]], ["防滑外套 TPU 85A / 硅胶（可换）", "FL_knee", [X + 10, YO + 45, -390]],
             ["36T 大带轮", "FL_knee", [X, YO + 70, -190]]] },
  { name: "v3_motor", title: "电机安装（按 DM-J4310 V1.1 图纸）", pose: "stand", focus: ["FL_hfe", [176, 110, 5]], off: [0.30, 0.16, 0.26], shell: false, thigh: true, ghost: true,
    labels: [["膝电机前安装环（6×M3 / Φ50）", "FL_hfe", [163, 145, 25]], ["KFE 插头（沿大腿向下）", "FL_hfe", [188, 106, -41]],
             ["HFE 插头（朝上）", "FL_haa", [188, 52, 41]], ["HAA 插头（斜向上外侧，在髋座里）", "body", [112, 55, 34]],
             ["髋支架（6×M3 / Φ27 + 3×Φ4 销）", "FL_haa", [154, 20, 22]], ["HFE 背面 4×M3 / Φ38", "FL_haa", [188, 41, 18]]] },
  { name: "v3_face", title: "v0.3.3 前脸：长条彩屏眼睛，传感器收进脸里", pose: "stand", focus: ["neck_pitch", [252, 0, 100]], off: [0.13, 0.05, 0.24], shell: true,
    labels: [["2.25 寸长条彩屏（深色亚克力后面）", "neck_pitch", [249.6, 22, 110]], ["USB 相机镜头（分体式模组）", "neck_pitch", [250, 0, 88]],
             ["VL53L5CX 测距窗", "neck_pitch", [250, -16, 88]], ["隐私指示灯", "neck_pitch", [250, 12, 88]],
             ["毫米波藏在这条白边后面", "neck_pitch", [250, -8, 120]]] },
  { name: "v4_leg", title: "封闭大腿：皮带和带轮全包在里面，小腿实心", pose: "stand", focus: ["FL_hfe", [176, 128, -120]], off: [0.42, 0.06, -0.42], shell: true,
    labels: [["围边：从外壳一直包到内板", "FL_hfe", [147, 135, -120]], ["张紧轮罩（拆外壳调皮带）", "FL_hfe", [136, 135, -70]],
             ["小腿槽口：里面是实心的膝部圆盘", "FL_hfe", [190, 118, -228]], ["实心小腿（去掉减重孔）", "FL_knee", [188, 122, -300]]] },
  { name: "v4_hip", title: "腿部全包裹：髋罩 + 膝部毛刷条", pose: "stand", cam: [0.48, 0.52, 0.62], tgt: [0.11, 0.26, 0.15], shell: true,
    labels: [["髋罩：包住髋电机、插头和支架", "FL_haa", [205, 88, 22]], ["膝部毛刷条：挡住小腿槽口", "FL_hfe", [222, 118, -175]],
             ["封闭大腿", "FL_hfe", [188, 179, -90]], ["电机线从髋罩后面直接进机身", "FL_haa", [156, 56, 36]]] },
  { name: "v3_rear", title: "后 45°", pose: "stand", cam: [0.9, 0.62, -0.95], tgt: [0, 0.27, -0.02], shell: true },
  { name: "v3_leg", title: "左前腿：同步带膝关节（拆下装甲和外壳）", pose: "stand", cam: [0.72, 0.30, 0.55], tgt: [0.12, 0.19, 0.13], shell: false, thigh: true,
    labels: [["膝电机 KFE（随大腿转）", "FL_hfe", [X, YO + 31, 0]], ["HFE 髋俯仰电机", "FL_haa", [X, 67, 0]],
             ["20T 小带轮", "FL_hfe", [X, YO + 70, 0]], ["HTD-5M 20mm 皮带", "FL_hfe", [X - 22, YO + 70, -95]],
             ["张紧轮（长槽可调）", "FL_hfe", [X - 31, YO + 70, -70]], ["36T 大带轮（1:1.8）", "FL_knee", [X, YO + 70, -190]],
             ["中段立柱（在皮带环内）", "FL_hfe", [X, YO + 45, -138]], ["小腿", "FL_knee", [X, YO + 32, -290]],
             ["TPU 足端", "FL_knee", [X, YO + 32, -380]]] },
  { name: "v3_head", title: "头部内部（拆下头壳）", pose: "stand", cam: [0.50, 0.60, 0.48], tgt: [0, 0.40, 0.17], shell: false,
    labels: [["USB 相机镜头模组", "neck_pitch", [244, 0, 93]], ["VL53L5CX 测距", "neck_pitch", [246, -16, 94]],
             ["LD2410 毫米波（屏幕上方）", "neck_pitch", [246, 8, 123]], ["相机主板（贴右侧内壁）", "neck_pitch", [225, -41, 102]], ["2.25 寸长条彩屏（眼睛）", "neck_pitch", [246, 0, 115]],
             ["屏幕压框", "neck_pitch", [242, -22, 122]], ["I2S 麦克风（左）", "neck_pitch", [202, 46, 112]],
             ["功放 MAX98357A", "neck_pitch", [175, 47, 109]], ["ESP32-S3 + USB 集线器", "neck_pitch", [130, 10, 108]],
             ["隐私开关", "neck_pitch", [140, 47, 101]], ["俯仰电机 GM4108", "neck_yaw", [184, 22, 96]],
             ["偏航电机 GM3506", "body", [140, 0, 62]], ["轭架", "neck_yaw", [202, 36, 85]],
             ["头顶触摸铜箔", "neck_pitch", [165, 0, 129]], ["光照 BH1750", "neck_pitch", [200, 0, 127]]] },
  { name: "v3_body", title: "机身内部（拆下装甲，髋座半透明）", pose: "stand", cam: [0.70, 0.88, 0.55], tgt: [0, 0.30, 0], shell: false, ghost: true,
    labels: [["树莓派 4B", "body", [2, 0, 38]], ["MC02 主控（3 路 CAN）", "body", [-8, 20, 12]], ["配电板 + INA228", "body", [-66, 0, 15]],
             ["DC-DC 5V", "body", [-65, 39, 17]], ["DC-DC 12V（脖子）", "body", [-75, -39, 9]], ["4010 排风扇", "body", [-68, 0, 49]],
             ["MPR121 触摸芯片", "body", [58, -38, 28]], ["6S2P 电池（配平后前移 23mm）", "body", [23, 31, -24]],
             ["HAA 外摆电机", "body", [125, 32, 0]], ["线束口", "body", [145, 56, 36]]] },
  { name: "v3_sense", title: "全身感知布局（拆下外壳）", pose: "stand", cam: [0.85, 0.75, 0.75], tgt: [0, 0.28, 0.02], shell: false,
    labels: [["头顶触摸", "neck_pitch", [165, 0, 129]], ["下巴触摸", "neck_pitch", [228, 0, 79]],
             ["背部触摸（前）", "body", [47, 37, 62]], ["背部触摸（后）", "body", [-47, 37, 62]],
             ["前防跌落 ToF", "body", [131, 0, -38]], ["后防跌落 ToF", "body", [-131, 0, -38]],
             ["前向测距", "neck_pitch", [250, -16, 88]], ["毫米波人体存在", "neck_pitch", [246, 8, 123]],
             ["相机", "neck_pitch", [250, 0, 88]], ["麦克风", "neck_pitch", [202, 46, 112]]] },
];

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
    add("text", { x: 22, y: 40, "font-size": 20, "font-weight": 700, fill: "#e9edf1" }).textContent = "RoboDog v0.4 · " + s.title;
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
