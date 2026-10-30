"""007A 第 4 步：3D 打印件（工程版）——外形用 v19 外观，做成能打印、能装的壳，按 256 × 256 × 256 的打印机拆块。

外观模型（body_v10.py）里外壳是实心块，这里：
  - 上层甲板壳 / 黑腰 / 下层底盘：减掉铝骨架占的空间（骨架外面留 0.5）+ 往里缩 WALL 的内腔 → 剩一层壳，包在铝骨架外面。
    上层的货台中间是空的（货台底就是 A07 顶板 + 防滑垫），下层底下是 2 mm 的皮（底板下面）。
  - 拆块：上层 488 × 264 太大 → x = 0、y = 0 各拆一刀成 4 块；下层、黑腰 x = 0 拆成 2 块。接缝面上打 Φ4.2 × 6 定位销孔（买 Φ4 × 10 钢销，或打印销），再用胶粘。
  - 螺丝：上层壳底面在 A07 顶板的外壳孔（eng_body.SHELL_SCREWS_TOP）对上的地方打 Φ4.0 × 5 热熔螺母孔（M3），从顶板下面往上拧；
    下层底下皮薄（2 mm），在 SHELL_SCREWS_BOT 开 Φ3.4 通孔，M3 圆头螺丝从下面拧进底板的螺纹孔。
  - 头：照 eng_head.py 做成 2 mm 壁、后面敞开的壳。
  - 腿上打印的：大腿外层壳、小腿、绕线环（工程件，eng_leg.py），外观盖板（大腿外盖板、小腿盖板、髋盖、膝盖帽、脚垫 TPU）。
  - 其他小件：侧装甲板、前防撞杠、深度相机罩、热像仪座、雷达护圈、护角（TPU）、急停座、麦克风座。
输出：007a/out/print/*.stl（每块一个，按装配位置，切片软件里自己摆平）+ print_list.json；docs/机械狗007A/打印件清单.md
用法：python 007a/print_parts.py
"""
import json
import os

import cadquery as cq

import body_v10 as BV
import eng_body as B
import eng_leg as E
import model as M
from model import V, box, cyl_y

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "print")
DOC = os.path.join(os.path.dirname(HERE), "docs", "机械狗007A")
BED = 256.0
WALL = 2.5
L2, W2 = M.L2, M.W2

# 骨架占的空间（外面留 0.5）
FRAME = box(-B.X_END - B.T_END - 0.5, B.X_END + B.T_END + 0.5, -B.Y_SR - B.T_SIDE - 0.5, B.Y_SR + B.T_SIDE + 0.5,
            B.Z_BOT - 0.3, B.Z_TOPP + B.T_TOP + 0.3)

# (外观里的名字, 材料, 颜色说明, 数量, 拆块方式)
BIG = [("上层甲板壳", "ASA 浅灰", "xy"), ("黑腰", "ASA 深灰", "x"), ("下层底盘", "ASA 浅灰", "x")]
SMALL = [("侧装甲板1", "ASA 深灰", "左"), ("侧装甲板-1", "ASA 深灰", "右"), ("前防撞杠", "ASA 深灰", ""), ("深度相机罩", "ASA 深灰", ""),
         ("热像仪座", "ASA 深灰", ""), ("雷达护圈", "ASA 深灰", ""), ("急停座", "ASA 深灰", ""), ("麦克风座", "ASA 深灰", "")]
CORNERS = ("护角11", "护角1-1", "护角-11", "护角-1-1")
LEG_APP = ("大腿外盖板", "小腿盖板", "髋盖", "膝盖帽外", "脚垫")


def bbox(s):
    b = s.BoundingBox()
    return b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax


def inner_scaled(s, t):
    """往里缩 t 的内腔（按包围盒非均匀缩放，外形是方盒 + 倒角，近似等厚）。"""
    x0, x1, y0, y1, z0, z1 = bbox(s)
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
    kx, ky, kz = [(d - 2 * t) / d for d in (x1 - x0, y1 - y0, z1 - z0)]
    m = cq.Matrix([[kx, 0, 0, cx * (1 - kx)], [0, ky, 0, cy * (1 - ky)], [0, 0, kz, cz * (1 - kz)]])
    return s.transformGeometry(m)


def hollow(name, s):
    h = s.cut(FRAME)
    if name != "黑腰":                                                 # 黑腰只有 10 高，减掉骨架就是一圈环
        h = h.cut(inner_scaled(s, WALL).intersect(box(-1e3, 1e3, -1e3, 1e3, B.Z_BOT - 50, 1e3) if name == "上层甲板壳" else
                                                  box(-1e3, 1e3, -1e3, 1e3, M.Z0 + 2.0, 1e3)))
    return h


def fits(s):
    x0, x1, y0, y1, z0, z1 = bbox(s)
    d = sorted((x1 - x0, y1 - y0, z1 - z0))
    return d[1] <= BED and d[2] <= BED, (x1 - x0, y1 - y0, z1 - z0)


def pins(piece_a, piece_b, axis, at, cands):
    """接缝面上打定位销孔：候选点两边都有料（实心）才打。"""
    hole_a, hole_b = [], []
    for u, w in cands:
        p = V(at, u, w) if axis == "x" else V(u, at, w)
        da = V(-3, 0, 0) if axis == "x" else V(0, -3, 0)
        ok_a = piece_a.isInside(p + da) or piece_b.isInside(p + da)
        ok_b = piece_a.isInside(p - da) or piece_b.isInside(p - da)
        if ok_a and ok_b:
            d = V(1, 0, 0) if axis == "x" else V(0, 1, 0)
            c = cq.Solid.makeCylinder(2.1, 12, p - d * 6, d)
            hole_a.append(c)
    for c in hole_a:
        piece_a, piece_b = piece_a.cut(c), piece_b.cut(c)
    return piece_a, piece_b, len(hole_a)


def split(name, s, how):
    """按 x = 0（和 y = 0）拆；接缝 0.2（打印公差），每条缝打销孔。"""
    zmid = (bbox(s)[4] + bbox(s)[5]) / 2
    pieces = [("", s)]
    n_pin = 0
    if "x" in how:
        a = s.intersect(box(0.1, 1e3, -1e3, 1e3, -1e3, 1e3))
        b = s.intersect(box(-1e3, -0.1, -1e3, 1e3, -1e3, 1e3))
        side = W2 - 6.0                                                # 侧壁中间
        cands = [(sy * side, z) for sy in (1, -1) for z in (zmid - 15, zmid + 15)] + [(sy * 60, bbox(s)[5] - 3) for sy in (1, -1)] + \
                [(sy * 60, bbox(s)[4] + 1.2) for sy in (1, -1)]
        a, b, k = pins(a, b, "x", 0.0, cands)
        n_pin += k
        pieces = [("前", a), ("后", b)]
    if "y" in how:
        out = []
        for tag, p in pieces:
            l = p.intersect(box(-1e3, 1e3, 0.1, 1e3, -1e3, 1e3))
            r = p.intersect(box(-1e3, 1e3, -1e3, -0.1, -1e3, 1e3))
            sx = 1 if tag == "前" else -1
            cands = [(sx * x, z) for x in (L2 - 6.0, 120.0) for z in (zmid - 15, zmid + 15)] + [(sx * x, bbox(s)[5] - 3) for x in (60.0, 180.0)]
            l, r, k = pins(l, r, "y", 0.0, cands)
            n_pin += k
            out += [(tag + "左", l), (tag + "右", r)]
        pieces = out
    return pieces, n_pin


def screw_holes(name, s):
    if name == "上层甲板壳":
        z = B.Z_TOPP + B.T_TOP
        for x, y in B.SHELL_SCREWS_TOP:
            s = s.cut(cq.Solid.makeCylinder(2.0, 5.5, V(x, y, z - 0.5)))   # M3 热熔螺母 Φ4.0 × 5
    if name == "下层底盘":
        for x, y in B.SHELL_SCREWS_BOT:
            s = s.cut(cq.Solid.makeCylinder(1.7, 10, V(x, y, M.Z0 - 1)))    # M3 通孔
    return s


def main():
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        os.remove(os.path.join(OUT, f))
    look = {n: s for n, s, c in BV.all_parts()}
    rows = []

    def save(code, name, s, mat, qty, note=""):
        fn = f"{code}_{name}.stl"
        cq.exporters.export(cq.Workplane().add(s), os.path.join(OUT, fn), tolerance=0.05, angularTolerance=0.1)
        ok, d = fits(s)
        rows.append(dict(code=code, name=name, file=fn, mat=mat, qty=qty, size=[round(v) for v in d], fits=ok,
                         vol=round(s.Volume() / 1000, 1), note=note))
        print(f"  {code} {name} ×{qty}  {d[0]:.0f} × {d[1]:.0f} × {d[2]:.0f}  {'✓' if ok else '超出打印机'}  {s.Volume() / 1000:.0f} cm³")
    k = 1
    for name, mat, how in BIG:
        h = screw_holes(name, hollow(name, look[name]))
        parts, n_pin = split(name, h, how)
        for tag, p in parts:
            save(f"S{k:02d}", f"{name}{tag}", p, mat, 1, f"接缝 {n_pin} 个 Φ4.2×6 销孔，配 Φ4×10 钢销 + 胶")
            k += 1
    hx = BV.L2
    head = look["头"]
    hb = bbox(head)
    head = head.cut(box(L2 - 20, hb[1] - 2, -150 / 2 + 2, 150 / 2 - 2, hb[4] + 2, hb[5] - 2))   # 同 eng_head.py：壁 2，后面敞开
    save(f"S{k:02d}", "头壳", head, "ASA 深灰", 1, "屏幕玻璃（亚克力 2 mm 烟黑）从前面嵌进去；后面敞开，头支架从影线缝伸进来")
    k += 1
    for nm in ("头鳍1", "头鳍-1"):
        save(f"S{k:02d}", nm, look[nm], "ASA 浅灰", 1)
        k += 1
    for name, mat, note in SMALL:
        save(f"S{k:02d}", name, look[name], mat, 1, note)
        k += 1
    save(f"S{k:02d}", "护角", look[CORNERS[0]], "TPU 95A 黑", 4, "四个角一样（左右镜像打：切片里镜像）")
    k += 1
    # 腿：工程件（大腿外层壳、小腿、绕线环）+ 外观盖板
    leg, _, _ = E.leg_module()
    for nm, mat in (("大腿外层壳（打印，完整外形）", "PA12-CF / ASA"), ("小腿（铝 + 打印壳）", "PA12-CF"), ("绕线环（打印 PETG）", "PETG")):
        s = next(s for n, s, c, g in leg if n == nm)
        save(f"S{k:02d}", nm.split("（")[0], s, mat, 4, "左右腿镜像（切片里镜像）")
        k += 1
    for key in LEG_APP:
        nm = next(n for n in look if n.startswith("前左") and key in n)
        mat = "TPU 95A 黑" if key == "脚垫" else ("ASA 浅灰" if "盖板" in key else "ASA 深灰")
        qty = 8 if key == "小腿盖板" else 4
        save(f"S{k:02d}", key, look[nm], mat, qty, "小腿两面各一块" if key == "小腿盖板" else "")
        k += 1
    json.dump(rows, open(os.path.join(OUT, "print_list.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    too_big = [r["name"] for r in rows if not r["fits"]]
    with open(os.path.join(DOC, "打印件清单.md"), "w", encoding="utf-8") as f:
        f.write("# 机械狗007A 3D 打印件清单（第 4 步，v19 省钱版）\n\n")
        f.write("> 由 `007a/print_parts.py` 生成，STL 在 `007a/out/print/`（按装配位置导出，切片时自己摆平、加支撑）。打印机按 256 × 256 × 256 算。\n")
        f.write("> 外壳是 2.5 mm 的壳，包在铝骨架外面；大块按 x = 0 / y = 0 拆开，接缝打 Φ4.2 × 6 销孔（Φ4 × 10 钢销）再用胶粘。\n")
        f.write("> 上层甲板壳从 A07 顶板下面用 M3 拧进热熔螺母；下层底盘从下面用 M3 圆头螺丝拧进 A01 底板。\n\n")
        f.write("| 编号 | 名字 | 数量 | 材料 | 外形 mm | 体积 cm³ | 能不能打 | 说明 |\n|---|---|---|---|---|---|---|---|\n")
        for r in rows:
            f.write(f"| {r['code']} | {r['name']} | {r['qty']} | {r['mat']} | {' × '.join(map(str, r['size']))} | {r['vol']} | {'✓' if r['fits'] else '超出'} | {r['note']} |\n")
        tot = sum(r["vol"] * r["qty"] for r in rows)
        f.write(f"\n合计约 {tot:.0f} cm³（实心体积；按 15–20 % 填充、3 圈壁，耗材约 {tot * 0.45 * 1.2 / 1000:.1f} kg）。超出打印机的：{', '.join(too_big) or '没有'}\n")
    print("超出 256 的：", too_big or "没有")


if __name__ == "__main__":
    main()
