"""机械狗007v 制造件：出图（SVG 尺寸图 + 孔位表）、DXF（激光切割）、STEP（CNC）、STL（3D 打印）。

用法：python 007v/mfg.py
输出：007v/out/mfg/<编号>_<名字>.(dxf|step|stl)，007v/out/mfg/parts.json（给图纸网页用）

所有尺寸都从 config.py 来，和 3D 模型、碰撞检查、仿真用的是同一套数。
标 ★ 的孔位网上查不到（卖家没公开），先按常见规格画：下单时问卖家要安装尺寸图，到货实测后改 config.py 再跑一遍。
坐标：板件图纸原点标在图上（⊕），孔位表里的 X / Y 都相对这个原点，单位 mm。
"""
import json
import math
import os
import sys

import cadquery as cq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import *  # noqa: E402,F401,F403
import model as M  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "mfg")
os.makedirs(OUT, exist_ok=True)
V = cq.Vector
W2 = BODY_W / 2
YIN = W2 - PLATE_SIDE
PY0 = YIN - PULLEY_W - 1                      # 大带轮内侧（机身坐标 y）
HUB0 = W2 + BEARING_HOUSING_W + 1             # 轴毂内侧
SHAFT_Y0 = PY0 - 6                            # 髋轴内端
SHAFT_Y1 = HUB0 + HUB["boss_l"] + HUB["flange_t"] - 1
DECK_FIX_X = [-125.0, -5.0, 85.0]             # 机身和甲板 / 底板之间的 2020 角码位置（侧板、甲板、底板三处对齐）
RAIL_HOLES_X = [-185.0, -125.0, -65.0, -5.0, 35.0]


# ================================================================ 基本几何
def arc_pts(cx, cy, r, a0, a1, step=3.0):
    n = max(2, int(abs(a1 - a0) / step) + 1)
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / (n - 1))),
             cy + r * math.sin(math.radians(a0 + (a1 - a0) * i / (n - 1)))) for i in range(n)]


def rect_pts(x0, y0, x1, y1, ch=0.0):
    if ch <= 0:
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(x0 + ch, y0), (x1 - ch, y0), (x1, y0 + ch), (x1, y1 - ch), (x1 - ch, y1), (x0 + ch, y1), (x0, y1 - ch), (x0, y0 + ch)]


def stadium_pts(L, r0, r1):
    """圆心 (0,0) 半径 r0 和 (L,0) 半径 r1 两个圆的外公切线轮廓。"""
    a = math.degrees(math.asin((r0 - r1) / L))
    return arc_pts(L, 0, r1, -90 + a, 90 - a) + arc_pts(0, 0, r0, 90 - a, 270 + a)


def hole(x, y, d, note="", star=False, kind="通孔"):
    return dict(x=round(x, 2), y=round(y, 2), d=d, note=note, star=star, kind=kind)


def slot(x, y, d, dx, dy, note="", star=False):
    return dict(x=round(x, 2), y=round(y, 2), d=d, dx=round(dx, 2), dy=round(dy, 2), note=note, star=star, kind="长孔")


def ring(cx, cy, n, pcd, d, a0=45.0, **kw):
    return [hole(cx + pcd / 2 * math.cos(math.radians(a0 + 360 * i / n)), cy + pcd / 2 * math.sin(math.radians(a0 + 360 * i / n)), d, **kw)
            for i in range(n)]


# ================================================================ 板件
class Plate:
    kind = "plate"

    def __init__(self, code, name, qty, material, process, t, outline, holes, notes=(), origin_note="", cutouts=()):
        self.code, self.name, self.qty, self.material, self.process, self.t = code, name, qty, material, process, t
        self.outline, self.holes, self.notes, self.origin_note = outline, holes, list(notes), origin_note
        self.cutouts = list(cutouts)              # 方孔：(x0, y0, x1, y1, 说明)

    def solid(self):
        s = cq.Workplane("XY").polyline(self.outline).close().extrude(self.t).val()
        for h in self.holes:
            if h["kind"] == "长孔":
                L = 2 * math.hypot(h["dx"], h["dy"]) + h["d"]
                ang = math.degrees(math.atan2(h["dy"], h["dx"]))
                c = cq.Workplane("XY").center(h["x"], h["y"]).slot2D(L, h["d"], ang).extrude(self.t + 2).translate((0, 0, -1)).val()
            else:
                c = cq.Solid.makeCylinder(h["d"] / 2, self.t + 2, V(h["x"], h["y"], -1))
            s = s.cut(c)
        for x0, y0, x1, y1, _ in self.cutouts:
            s = s.cut(cq.Solid.makeBox(x1 - x0, y1 - y0, self.t + 2, V(x0, y0, -1)))
        return s

    def export(self):
        s = self.solid()
        base = os.path.join(OUT, f"{self.code}_{self.name}")
        cq.exporters.export(cq.Workplane().add(s), base + ".step")
        face = cq.Workplane().add(s).faces("<Z").val()
        cq.exporters.exportDXF(cq.Workplane().add(face), base + ".dxf")
        files = [f"{self.code}_{self.name}.dxf", f"{self.code}_{self.name}.step"]
        if "打印" in self.process:
            cq.exporters.export(cq.Workplane().add(s), base + ".stl", tolerance=0.05, angularTolerance=0.1)
            files.append(f"{self.code}_{self.name}.stl")
        return files

    def svg(self, W=660):
        xs = [p[0] for p in self.outline]
        ys = [p[1] for p in self.outline]
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        m = 64
        s = min((W - 2 * m) / (x1 - x0), 420 / (y1 - y0))
        Wp = (x1 - x0) * s + 2 * m
        Hp = (y1 - y0) * s + 2 * m
        X = lambda x: m + (x - x0) * s  # noqa: E731
        Y = lambda y: Hp - m - (y - y0) * s  # noqa: E731
        out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {Wp:.0f} {Hp:.0f}" class="dwg">']
        out.append('<path class="ol" d="M' + " L".join(f"{X(px):.1f},{Y(py):.1f}" for px, py in self.outline) + ' Z"/>')
        for cx0, cy0, cx1, cy1, _ in self.cutouts:
            out.append(f'<rect class="ol" x="{X(cx0):.1f}" y="{Y(cy1):.1f}" width="{(cx1 - cx0) * s:.1f}" height="{(cy1 - cy0) * s:.1f}"/>')
        for i, h in enumerate(self.holes):
            cls = "hs" if h["star"] else "ho"
            r = max(h["d"] / 2 * s, 1.2)
            if h["kind"] == "长孔":
                ex, ey = h["dx"] * s, -h["dy"] * s
                L = math.hypot(ex, ey)
                ux, uy = (ex / L, ey / L) if L else (1, 0)
                px, py = -uy * r, ux * r
                cx, cy = X(h["x"]), Y(h["y"])
                out.append(f'<path class="{cls}" d="M{cx - ex + px:.1f},{cy - ey + py:.1f} L{cx + ex + px:.1f},{cy + ey + py:.1f} '
                           f'A{r:.1f},{r:.1f} 0 0 0 {cx + ex - px:.1f},{cy + ey - py:.1f} L{cx - ex - px:.1f},{cy - ey - py:.1f} '
                           f'A{r:.1f},{r:.1f} 0 0 0 {cx - ex + px:.1f},{cy - ey + py:.1f} Z"/>')
            else:
                out.append(f'<circle class="{cls}" cx="{X(h["x"]):.1f}" cy="{Y(h["y"]):.1f}" r="{r:.1f}"/>')
            c = 3 + r
            out.append(f'<path class="cl" d="M{X(h["x"]) - c:.1f},{Y(h["y"]):.1f} h{2 * c:.1f} M{X(h["x"]):.1f},{Y(h["y"]) - c:.1f} v{2 * c:.1f}"/>')
            out.append(f'<text class="hl" x="{X(h["x"]) + r + 2:.1f}" y="{Y(h["y"]) - r - 2:.1f}">{i + 1}</text>')
        # 原点
        out.append(f'<g class="org"><circle cx="{X(0):.1f}" cy="{Y(0):.1f}" r="5"/><path d="M{X(0) - 8:.1f},{Y(0):.1f} h16 M{X(0):.1f},{Y(0) - 8:.1f} v16"/></g>')
        # 总长、总高
        yb = Hp - m + 26
        out.append(dim_h(X(x0), X(x1), yb, num(x1 - x0)))
        xl = m - 26
        out.append(dim_v(Y(y0), Y(y1), xl, num(y1 - y0)))
        out.append("</svg>")
        return "".join(out)

    def table(self):
        rows = []
        for i, h in enumerate(self.holes):
            size = f"Φ{h['d']:g}" if h["kind"] != "长孔" else f"{h['d']:g} 宽、长 {2 * math.hypot(h['dx'], h['dy']) + h['d']:.1f}"
            rows.append(dict(no=i + 1, x=h["x"], y=h["y"], size=size, kind=h["kind"], note=h["note"], star=h["star"]))
        for x0, y0, x1, y1, note in self.cutouts:
            rows.append(dict(no="方孔", x=round((x0 + x1) / 2, 2), y=round((y0 + y1) / 2, 2), size=f"{x1 - x0:g} × {y1 - y0:g}", kind="方孔", note=note, star=False))
        return rows

    def info(self):
        xs = [p[0] for p in self.outline]
        ys = [p[1] for p in self.outline]
        return dict(code=self.code, name=self.name, qty=self.qty, material=self.material, process=self.process,
                    size=f"{num(max(xs) - min(xs))} × {num(max(ys) - min(ys))} × {self.t:g}", notes=self.notes,
                    origin=self.origin_note, holes=self.table(), svg=self.svg(), kind="plate")


def num(v):
    """尺寸文字：保留到 0.1，去掉多余的 0。"""
    return f"{round(v, 1):g}"


def dim_h(xa, xb, y, text):
    return (f'<g class="dim"><path d="M{xa:.1f},{y - 18:.1f} V{y + 4:.1f} M{xb:.1f},{y - 18:.1f} V{y + 4:.1f} M{xa:.1f},{y:.1f} H{xb:.1f}"/>'
            f'<path class="ar" d="M{xa:.1f},{y:.1f} l7,-3 v6 z M{xb:.1f},{y:.1f} l-7,-3 v6 z"/>'
            f'<text x="{(xa + xb) / 2:.1f}" y="{y - 5:.1f}" text-anchor="middle">{text}</text></g>')


def dim_v(ya, yb, x, text):
    return (f'<g class="dim"><path d="M{x - 4:.1f},{ya:.1f} H{x + 18:.1f} M{x - 4:.1f},{yb:.1f} H{x + 18:.1f} M{x:.1f},{ya:.1f} V{yb:.1f}"/>'
            f'<path class="ar" d="M{x:.1f},{ya:.1f} l-3,-7 h6 z M{x:.1f},{yb:.1f} l-3,7 h6 z"/>'
            f'<text x="{x - 6:.1f}" y="{(ya + yb) / 2:.1f}" text-anchor="middle" transform="rotate(-90 {x - 6:.1f} {(ya + yb) / 2:.1f})">{text}</text></g>')


# ================================================================ 车削件（回转体）
class Turned:
    kind = "turned"

    def __init__(self, code, name, qty, material, process, steps, bores, faces, notes=(), extra=None, marks=()):
        """steps：外形台阶 [(直径, 长度), …]，从 A 面往 B 面；bores：内孔 [(直径, 起点, 终点)]（从 A 面量）；
        faces：端面孔 [(面 'A'/'B', 个数, 分布圆, 孔径, 说明, ★)]。"""
        self.code, self.name, self.qty, self.material, self.process = code, name, qty, material, process
        self.steps, self.bores, self.faces, self.notes, self.extra = steps, bores, faces, list(notes), extra
        self.marks = list(marks)              # 侧视图上标出来的槽：(起点, 终点, 深, 文字)

    def length(self):
        return sum(L for _, L in self.steps)

    def solid(self):
        z = 0.0
        s = None
        for d, L in self.steps:
            c = cq.Solid.makeCylinder(d / 2, L, V(0, 0, z))
            s = c if s is None else s.fuse(c)
            z += L
        for d, a, b in self.bores:
            s = s.cut(cq.Solid.makeCylinder(d / 2, b - a, V(0, 0, a)))
        Lt = self.length()
        for face, n, pcd, d, _note, _star, *dep in self.faces:
            depth = (dep[0] if dep and dep[0] else Lt) + 0.2
            for h in ring(0, 0, n, pcd, d):
                z0 = -0.1 if face == "A" else Lt - depth + 0.1
                s = s.cut(cq.Solid.makeCylinder(d / 2, depth, V(h["x"], h["y"], z0)))
        if self.extra:
            s = self.extra(s)
        return s

    def export(self):
        s = self.solid()
        base = os.path.join(OUT, f"{self.code}_{self.name}")
        cq.exporters.export(cq.Workplane().add(s), base + ".step")
        return [f"{self.code}_{self.name}.step"]

    def svg(self):
        """左：A 面视图（端面孔）；右：侧面半剖（台阶、内孔）。"""
        D = max(d for d, _ in self.steps)
        Lt = self.length()
        s = min(4.0, 240 / D, 300 / Lt)
        m = 46
        fw = D * s
        Wp = m + fw + 70 + Lt * s + m + 120
        Hp = D * s + 2 * m + 20 + (30 if self.marks else 0)
        cy = m + D * s / 2 + (30 if self.marks else 0)
        cx = m + fw / 2
        o = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {Wp:.0f} {Hp:.0f}" class="dwg">']
        for d, _ in self.steps:
            o.append(f'<circle class="ol" cx="{cx:.1f}" cy="{cy:.1f}" r="{d / 2 * s:.1f}"/>')
        for d, a, b in self.bores:
            o.append(f'<circle class="hid" cx="{cx:.1f}" cy="{cy:.1f}" r="{d / 2 * s:.1f}"/>')
        for face, n, pcd, d, note, star, *_ in self.faces:
            o.append(f'<circle class="cl" cx="{cx:.1f}" cy="{cy:.1f}" r="{pcd / 2 * s:.1f}"/>')
            for h in ring(0, 0, n, pcd, d):
                o.append(f'<circle class="{"hs" if star else "ho"}" cx="{cx + h["x"] * s:.1f}" cy="{cy - h["y"] * s:.1f}" r="{d / 2 * s:.1f}"/>')
        o.append(f'<path class="cl" d="M{cx - D * s / 2 - 8:.1f},{cy:.1f} h{D * s + 16:.1f} M{cx:.1f},{cy - D * s / 2 - 8:.1f} v{D * s + 16:.1f}"/>')
        o.append(f'<text class="vl" x="{cx:.1f}" y="{Hp - 8:.1f}" text-anchor="middle">A 面</text>')
        # 侧面：上半外形，下半剖开看内孔
        x0 = m + fw + 70
        z = 0.0
        prof = []
        for d, L in self.steps:
            prof += [(z, d / 2), (z + L, d / 2)]
            z += L
        top = f'M{x0:.1f},{cy:.1f} ' + " ".join(f"L{x0 + a * s:.1f},{cy - r * s:.1f}" for a, r in prof) + f" L{x0 + Lt * s:.1f},{cy:.1f}"
        o.append(f'<path class="ol" d="{top}"/>')
        bot = f'M{x0:.1f},{cy:.1f} ' + " ".join(f"L{x0 + a * s:.1f},{cy + r * s:.1f}" for a, r in prof) + f" L{x0 + Lt * s:.1f},{cy:.1f} Z"
        o.append(f'<path class="sec" d="{bot}"/>')                       # 下半剖面：灰色 = 材料
        for d, a, b in self.bores:                                        # 内孔：白色挖掉
            o.append(f'<rect class="bore" x="{x0 + a * s:.1f}" y="{cy:.1f}" width="{(b - a) * s:.1f}" height="{d / 2 * s:.1f}"/>')
            o.append(f'<path class="hid" d="M{x0 + a * s:.1f},{cy - d / 2 * s:.1f} h{(b - a) * s:.1f}"/>')
        o.append(f'<path class="ol" d="{bot}"/>')
        for i, (d, a, b) in enumerate(self.bores):
            o.append(f'<text class="vl" x="{x0 + Lt * s + 8:.1f}" y="{cy + 14 + i * 14:.1f}">内孔 Φ{d:g}：{a:g}–{b:g}</text>')
        o.append(f'<path class="cl" d="M{x0 - 8:.1f},{cy:.1f} h{Lt * s + 16:.1f}"/>')
        r0 = max(d for d, _ in self.steps) / 2
        for a, b, dep, text, row in self.marks:
            o.append(f'<rect class="bore" x="{x0 + a * s:.1f}" y="{cy - r0 * s:.1f}" width="{max((b - a) * s, 1.5):.1f}" height="{dep * s:.1f}"/>')
            o.append(f'<path class="cl" d="M{x0 + (a + b) / 2 * s:.1f},{cy - r0 * s:.1f} V{cy - r0 * s - 6 - row * 14:.1f}"/>')
            o.append(f'<text class="vl" x="{x0 + (a + b) / 2 * s + 3:.1f}" y="{cy - r0 * s - 8 - row * 14:.1f}">{text}</text>')
        o.append(dim_h(x0, x0 + Lt * s, Hp - m + 18, f"{Lt:g}"))
        z = 0.0
        for d, L in self.steps:
            o.append(f'<text class="vl" x="{x0 + Lt * s + 8:.1f}" y="{cy - d / 2 * s + 4:.1f}">Φ{d:g} × {L:g}</text>')
            z += L
        o.append(f'<text class="vl" x="{x0 + Lt * s / 2:.1f}" y="{Hp - 8:.1f}" text-anchor="middle">侧面（下半剖开）</text>')
        o.append("</svg>")
        return "".join(o)

    def info(self):
        rows = []
        for face, n, pcd, d, note, star, *dep in self.faces:
            how = f"深 {dep[0]:g}" if dep and dep[0] else "通孔"
            rows.append(dict(no=f"{face} 面", x="", y="", size=f"{n} × Φ{d:g}，分布圆 {pcd:g}，{how}", kind="端面孔", note=note, star=star))
        for d, a, b in self.bores:
            rows.append(dict(no="内孔", x="", y="", size=f"Φ{d:g}，从 A 面 {a:g} 到 {b:g}", kind="内孔", note="", star=False))
        D = max(d for d, _ in self.steps)
        return dict(code=self.code, name=self.name, qty=self.qty, material=self.material, process=self.process,
                    size=f"Φ{D:g} × {self.length():g}", notes=self.notes, origin="", holes=rows, svg=self.svg(), kind="turned")


# ================================================================ 打印件（直接用 cadquery 实体，图纸用轴测投影）
class Printed:
    kind = "printed"

    def __init__(self, code, name, qty, material, solid, notes=(), size=""):
        self.code, self.name, self.qty, self.material, self.s, self.notes, self.size = code, name, qty, material, solid, list(notes), size
        self.process = "3D 打印"

    def export(self):
        base = os.path.join(OUT, f"{self.code}_{self.name}")
        cq.exporters.export(cq.Workplane().add(self.s), base + ".stl", tolerance=0.05, angularTolerance=0.1)
        cq.exporters.export(cq.Workplane().add(self.s), base + ".step")
        return [f"{self.code}_{self.name}.stl", f"{self.code}_{self.name}.step"]

    def svg(self):
        tmp = os.path.join(OUT, "_tmp.svg")
        cq.exporters.export(cq.Workplane().add(self.s), tmp, "SVG",
                            opt=dict(width=560, height=340, marginLeft=20, marginTop=20, showAxes=False,
                                     projectionDir=(1.0, -1.4, 0.9), strokeWidth=-1.0, strokeColor=(30, 34, 40),
                                     hiddenColor=(150, 160, 170), showHidden=False))
        with open(tmp, encoding="utf-8") as f:
            t = f.read()
        os.remove(tmp)
        i = t.find("<svg")
        return t[i:].replace("<svg", '<svg class="dwg iso"', 1)

    def info(self):
        bb = self.s.BoundingBox()
        return dict(code=self.code, name=self.name, qty=self.qty, material=self.material, process=self.process,
                    size=self.size or f"{bb.xlen:.0f} × {bb.ylen:.0f} × {bb.zlen:.0f}", notes=self.notes, origin="",
                    holes=[], svg=self.svg(), kind="printed")


# ================================================================ 零件定义
def side_plate():
    H = []
    for sx in (1, -1):
        hx = sx * HIP_X
        H.append(hole(hx, HIP_Z, 22.0, "髋轴穿过；同时挡住两个 6001 的外圈"))
        H += ring(hx, HIP_Z, 4, HOUSING_PCD, HOUSING_HOLE, note="轴承座 M4×25 螺栓 + 防松螺母")
        gx, gz = sx * (HIP_X + WORM_OUT_DX), HIP_Z + WORM_OUT_DZ
        e = (HIP_X - (HIP_X + WORM_OUT_DX), HIP_Z - gz)
        el = math.hypot(*e)
        for u, v in WORM_FACE["holes"]:
            H.append(slot(gx + sx * u, gz + v, WORM_FACE["d"], sx * e[0] / el * WORM_FACE["slot"], e[1] / el * WORM_FACE["slot"],
                          "蜗轮箱 M4 六角柱（长 16）；NFP-5840 厂家孔位，电机朝上装；长孔用来调皮带松紧", star=True))
        H.append(hole(hx + sx * 25.0, 75.0, 12.0, "轮子电机线过线孔（装 Φ12 橡胶护线圈；在黑腰铝带上，避开灯带）"))
        for dx in (-9.0, 9.0):
            H.append(hole(hx + dx, HIP_Z + 34, 3.4, "编码器支架 P26：M3×8 沉头从外面拧（外面要沉头孔）"))
    zp = FRAME_RAIL[2] + FRAME_GAP
    for x in (-88.0, 88.0):
        for z in (zp + 9.5, 67.0 - 9.5):
            H.append(hole(x, z, 4.2, "侧挂板 M5 螺纹孔（底孔 4.2 攻 M5）"))
        for z in (13.0, 140.0):
            H.append(hole(sx * 185.0, z, 5.5, "2020 横梁端面 M5"))
    for x in DECK_FIX_X:
        H.append(hole(x, 140.0, 4.5, "上边角码 M4（和甲板对齐）"))
        H.append(hole(x, 10.0, 4.5, "下边角码 M4（和底板对齐）"))
    return Plate("A01", "机身侧板", 2, "6061-T6 铝板 4 mm", "激光切割（或 CNC）", PLATE_SIDE,
                 rect_pts(-BODY_L / 2, 0, BODY_L / 2, BODY_H, 5.0), H,
                 ["左右两块一样（翻过来用）", "蜗轮箱 4 个孔按 NFP-5840-31ZY 厂家图纸（孔距 28 × 40）", "长孔方向沿着皮带：往外推电机就是张紧"],
                 "原点：板底边中点（机身中心正下方）；X 向前，Y 向上")


def deck():
    H = []
    for x in RAIL_HOLES_X:
        for sy in (1, -1):
            H.append(hole(x, sy * RAIL_Y, 5.5, "侧导轨：M5 从下往上拧进导轨 T 型螺母" + ("；同时固定下面的角码" if x in DECK_FIX_X else "")))
    for sy in (1, -1):
        H.append(hole(DECK_FIX_X[2], sy * RAIL_Y, 4.5, "角码 M4，拧进脖子底座的热熔螺母"))
    for y in (-60.0, 0.0, 60.0):
        H.append(hole(BED_X0 + PROFILE / 2, y, 5.5, "后导轨 M5（从下往上拧进导轨）"))
        H.append(hole(BED_X1 - PROFILE / 2, y, 5.5, "前导轨 M5（从下往上拧进导轨）"))
        H.append(hole(185.0, y, 5.5, "前上横梁 M5 + T 型螺母（从上往下）"))
    for y in (-30.0, 30.0):
        H.append(hole(-185.0, y, 5.5, "后上横梁 M5 + T 型螺母（后导轨正好压在后横梁上，螺丝错开）"))
    for x in (NECK_X - 42.0, NECK_X + 55.0):
        for sy in (1, -1):
            H.append(hole(x, sy * 96.0, 5.5, "脖子底座 M5 沉头（从上往下穿过头部平台盖，下面 M5 防松螺母）"))
    for sy in (1, -1):
        H.append(hole(NECK_X + CABLE_X, sy * (POD_IN + POD_W / 2), 16.0, "过线孔：对着肩座走线孔（左：头部 USB + 供电、雷达；右：热成像、红外）"))
    ax, ay = ARM_PORT_XY
    H += ring(ax, ay, 4, 50.0 * math.sqrt(2), 5.5, note="机械臂接口板 M5")
    H.append(hole(ax, ay, 16.0, "机械臂接口过线孔（航空插座）"))
    return Plate("A02", "甲板", 1, "6061-T6 铝板 3 mm", "激光切割", PLATE, rect_pts(-BODY_L / 2, -W2, BODY_L / 2, W2, 5.0), H,
                 ["货斗导轨（2020）、脖子底座、机械臂接口板都装在上面", "四周用 2020 角码和侧板连接"],
                 "原点：板中心；X 向前，Y 向左")


def bottom():
    L2 = BODY_L / 2 - PLATE
    H = []
    for x in DECK_FIX_X:
        for sy in (1, -1):
            H.append(hole(x, sy * 106.0, 4.5, "下边角码 M4"))
    for y in (-60.0, 0.0, 60.0):
        for sx in (1, -1):
            H.append(hole(sx * 185.0, y, 5.5, "前后下横梁 M5 + T 型螺母"))
    for x in (-130.0, -45.0, 45.0, 130.0):
        for sy in (1, -1):
            H.append(hole(x, sy * 80.0, 4.5, "滑撬 M4 沉头（从下面拧）"))
    for x in (-130.0, -60.0, 60.0, 130.0):
        for sy in (1, -1):
            H.append(hole(x, sy * 56.0, 3.4, "电子托板 M3×65 铜柱"))
    import body_electronics as BE
    H += be_holes(BE.BASE)
    cut = []
    for sx in (1, -1):
        for sy in (1, -1):
            yb = BATTERY[1] / 2 + 4.0
            cut.append((sx * 20 - 13, sy * yb - 2, sx * 20 + 13, sy * yb + 2, "电池绑带孔 26 × 4（电池 75 × 135 × 47 两边各让 2）"))
    return Plate("A03", "底板", 1, "6061-T6 铝板 3 mm", "激光切割", PLATE, rect_pts(-L2, -YIN, L2, YIN, 3.0), H,
                 ["夹在两块侧板中间，用角码和下横梁固定", "电池用两根魔术贴绑带穿过绑带孔"], "原点：板中心；X 向前，Y 向左", cut)


def end_plate(front):
    H = []
    for y in (-80.0, 0.0, 80.0):
        for z in (13.0, 140.0):
            H.append(hole(y, z, 5.5, ("前" if front else "后") + "横梁 M5 + T 型螺母"))
    cut = []
    if front:
        dx, dy, dz = DEPTH_CAM
        cut.append((-dy / 2 - 1, 24.0, dy / 2 + 1, 24.0 + dz + 2, "深度相机窗口"))
        for y in (-88.0, 88.0):
            for z in (25.0, 55.0):
                H.append(hole(y, z, 3.4, "前保险杠 M3×10（从里面拧进保险杠的热熔螺母）"))
        for y in (-62.0, 62.0):
            H.append(hole(y, 32.0, 8.0, "台阶测距过线孔"))
    else:
        cut.append((-70.0, 60.0, -54.0, 68.0, "XT60 充电口（面板座）"))
        H.append(hole(0.0, 70.0, 22.0, "急停按钮 Φ22"))
        H.append(hole(55.0, 70.0, 16.0, "总开关 Φ16（带灯自锁）"))
        for y in (-70.0, 70.0):
            for z in (24.0, 46.0):
                H.append(hole(y, z, 3.4, "后格栅 M3×10（从里面拧进格栅的热熔螺母）"))
        for y in (-48.0, 48.0):
            H.append(hole(y, 33.0, 8.0, "台阶测距过线孔"))
    return Plate("A04" if front else "A05", "前板" if front else "后板", 1, "6061-T6 铝板 3 mm", "激光切割", PLATE,
                 rect_pts(-YIN, PLATE, YIN, BODY_H, 3.0), H,
                 ["外面以后还要盖打印外壳"], "原点：机身中线 × 机身底面；X 向左，Y 向上", cut)


def outer_arm():
    """外板（4 mm 铝）：腿外侧的盖面板，轮子那头挖 R52.5 圆弧缺口让麦轮嵌进去。"""
    H = [hole(u, v, 4.5, "M4 沉头：拧进箱形臂座外壳的热熔螺母") for u, v in ((20, 14), (20, -14), (95, 14), (95, -14))]
    rc = WHEEL_D / 2 + ARM_WHEEL_CLEAR
    r0, r1 = ARM_ROOT_R, ARM_END_R
    # 外轮廓：髋端半圆 + 上下两条边 + 轮子那头凹进去的 R(rc) 圆弧（麦轮嵌进去）
    def hit(sign):                                        # 上 / 下边和缺口圆的交点
        lo, hi = 0.0, ARM
        for _ in range(60):
            m = (lo + hi) / 2
            y = sign * (r0 + (r1 - r0) * m / ARM)
            if math.hypot(m - ARM, y) > rc:
                lo = m
            else:
                hi = m
        return lo, sign * (r0 + (r1 - r0) * lo / ARM)
    (xt, yt), (xb, yb) = hit(1), hit(-1)
    pts = [(r0 * math.cos(math.radians(a)), r0 * math.sin(math.radians(a))) for a in range(270, 89, -10)]
    pts.append((xt, yt))
    at, ab = math.atan2(yt, xt - ARM), math.atan2(yb, xb - ARM) + 2 * math.pi
    for i in range(1, 24):
        a = at + (ab - at) * i / 24
        pts.append((ARM + rc * math.cos(a), rc * math.sin(a)))
    pts.append((xb, yb))
    return Plate("A06", "摆臂外板", 4, "6061-T6 铝板 4 mm（深灰阳极）", "激光切割 + 沉头孔", 4.0, pts, H,
                 ["四块一样；只是外面的盖板，受力的是 A07 腿板", f"轮子那头凹进去一道 R{rc:g} 圆弧，麦轮嵌进去，外侧面和外板外侧齐平"],
                 "原点：髋轴中心；X 沿摆臂指向轮子")


def root_plate():
    """腿板（6 mm 铝，承力）：髋端拧轴毂法兰，一直伸到轮轴，轮端拧轮轴承座和电机护罩（兼电机支架）。"""
    H = [hole(0, 0, 10.0, "髋轴端螺丝 M4 + 垫片从这里拧")]
    H += ring(0, 0, 4, HUB["pcd"], 4.5, note="轴毂法兰 M4 沉头")
    H.append(hole(ARM, 0, 13.0, "轮轴穿过（Φ12 轴，留 0.5 间隙）"))
    H += ring(ARM, 0, 4, 30.0, 4.5, a0=45.0, note="轮轴承座 M4（从外面拧）")
    for X, Z in ((-24, 6), (24, 6), (-24, -20), (24, -20)):
        H.append(hole(ARM + X * 0.574 - Z * 0.819, X * 0.819 + Z * 0.574, 4.5, "电机护罩 M4（护罩兼电机支架）"))
    for u in (30, 70, 110):
        for v in (17, -17):
            H.append(hole(u, v, 3.4, "M3：箱形臂座外壳固定"))
    return Plate("A07", "腿板", 4, "6061-T6 铝板 6 mm", "激光切割 + 沉头孔", ARM_ROOT_PLATE_T,
                 stadium_pts(ARM, ARM_ROOT_R - 2, ARM_END_R), H,
                 ["四块一样；腿的主要受力件：髋端拧轴毂（B02），轮端拧轮轴承座（B07）和电机护罩（D29）",
                  "护罩孔位按站立时的方向画的（摆臂往下 55°），左右腿镜像时孔位跟着镜像（同一块板翻面用）"],
                 "原点：髋轴中心；X 沿摆臂指向轮子")


def be_holes(items):
    """body_electronics 里的零件 → 激光切割孔：圆孔 / 沿 x 长孔（±3）/ 扎带孔（3 × 7）。"""
    import body_electronics as BE  # noqa: F401
    H = []
    for name, x0, y0, sx, sy, sz, hs in items:
        for hx, hy, d, kind in hs:
            x, y = x0 + hx, y0 + hy
            if kind == "hole":
                H.append(hole(x, y, d, f"{name}（官方孔位）"))
            elif kind == "slotx":
                H.append(slot(x, y, d, 3.0, 0.0, f"{name}（通用件，长孔 ±3）"))
            else:
                H.append(slot(x, y, 3.0, 2.0, 0.0, f"{name} 扎带孔"))
    return H


def tray():
    import body_electronics as BE
    H = []
    for x in (-130.0, -60.0, 60.0, 130.0):
        for sy in (1, -1):
            H.append(hole(x, sy * 56.0, 3.4, "铜柱 M3"))
    H += be_holes(BE.TRAY)
    cut = [(-140.0 + 4, -10.0, -140.0 + 12, 10.0, "后侧过线口"), (128.0, -10.0, 136.0, 10.0, "前侧过线口")]
    return Plate("C01", "电子托板", 1, "黑色亚克力 3 mm", "激光切割", 3.0, rect_pts(-140, -66, 140, 66, 4.0), H,
                 ["8 根 M3×65 铜柱立在底板上；小板下面垫 6 mm 尼龙柱（M2 / M2.5 / M3）",
                  "树莓派、ESP32、CAN Pal、TCA9548A、两块 Pololu 按官方孔位；BTS7960、12V 降压、INA226 是通用件，开长孔"],
                 "原点：机身中心（和底板同一个原点）；X 向前，Y 向左", cut)


def housing():
    b = BEARING
    return Turned("B01", "髋轴承座", 4, "6061-T6 铝棒 Φ50", "数控车", [(BEARING_HOUSING_D, BEARING_HOUSING_W)],
                  [(b[1], 0.0, 2 * b[2]), (20.0, 2 * b[2], BEARING_HOUSING_W)],
                  [("A", 4, HOUSING_PCD, HOUSING_HOLE, "M4 穿螺栓到侧板", False)],
                  ["A 面贴侧板外面；两个 6001-2RS 从 A 面压进 Φ28 孔（H7），外圈被 B 端 Φ20 挡边和侧板 Φ22 孔夹住",
                   "Φ28 孔公差 H7（+0 ～ +0.021），A 面倒角 0.5"])


def hub():
    h = HUB

    def keyway(s):
        return s.cut(cq.Solid.makeBox(KEY[0], 3.0, h["boss_l"] + h["flange_t"] + 2, V(-KEY[0] / 2, HIP_SHAFT_D / 2 - 1.0, -1)))
    return Turned("B02", "臂座轴毂", 4, "6061-T6 铝棒 Φ48", "数控车 + 插键槽", [(h["boss_d"], h["boss_l"]), (h["flange_d"], h["flange_t"])],
                  [(HIP_SHAFT_D, 0.0, h["boss_l"] + h["flange_t"])],
                  [("B", 4, h["pcd"], 3.3, "M4 螺纹孔（底孔 3.3）：臂根内板沉头螺丝拧进来", False, h["flange_t"])],
                  ["Φ12 孔公差 H7，键槽宽 4（JS9）、深到孔心 7.8", "A 面朝机身，靠 B 隔圈顶住轴承内圈"], extra=keyway)


def axle_housing():
    f = WHEEL_MOTOR_FRONT
    return Turned("B07", "轮轴承座（兼电机座）", 4, "6061-T6 铝棒 Φ55", "数控车 + 钻孔 + 攻丝", [(53.0, 5.5), (40.0, AXLE_HOUSING_L - 5.5)],
                  [(37.1, 0.0, 5.5), (21.0, 5.5, 15.5), (14.0, 15.5, AXLE_HOUSING_L)],
                  [("A", f["n"], f["pcd"], 2.7, "M2.5×8：从 Φ53 台阶背面穿过来，拧进 GIM4310 前面机壳（螺纹深 5）", False),
                   ("B", 4, 30.0, 3.3, "M4 螺纹孔（底孔 3.3）：从腿板外面拧进来", False, 8.0)],
                  ["A 面贴 GIM4310 输出那面的机壳；Φ37.1 孔（H8）套住电机 Φ37 凸台定心，里面转着 B08 的 Φ34 法兰",
                   "两个 6801-2RS（12×21×5）从 Φ37.1 那头压进 Φ21 孔（H7），B 端 Φ14 挡边顶住外圈",
                   "电机的反扭矩：电机机壳 → 3 颗 M2.5 → B07 → 4 颗 M4 → 腿板；轮子重量由两个 6801 扛，不压电机轴",
                   "按 GIM4310-10 官方安装图（2025-12-03）：前面机壳 3-M2.5 深 5、分布圆 47.5；输出凸台 Φ37 × 1"])


def axle():
    L = M.HUB_IN + 6 - (LP0 - AXLE_HOUSING_L + 1 + AXLE_FLANGE_T)
    o = WHEEL_MOTOR_OUT
    return Turned("B08", "轮轴（带法兰）", 4, "45# 钢 Φ38（调质）", "数控车 + 铣双 D", [(34.0, AXLE_FLANGE_T), (12.0, L)],
                  [], [("A", o["n"], o["pcd"], o["d"], "M3×6 沉头：从轴那面沉头，拧进 GIM4310 输出盘（螺纹深 4）", True)],
                  ["A 端法兰贴电机输出盘，在 B07 的 Φ37.1 孔里转；3 颗 M3 沉头分布圆 27（官方图纸）",
                   f"Φ12 段总长 {L:g}；轴承段 Φ12 k6，轴承外侧一道 GB894 挡圈 12 的槽",
                   "B 端最后 13 mm 铣双 D（两边各铣到 10，对边 10），插进 B09 轮毂法兰传扭矩；端面 M5 螺纹孔深 10，M5×12 + 垫片压住 B09"])


def hub_adapter():
    h = WHEEL_HUB
    return Turned("B09", "轮毂法兰（接 goBILDA 麦轮）", 4, "45# 钢 Φ35", "数控车 + 线切割双 D 孔 + 攻丝", [(32.0, 7.0), (13.95, 6.0)],
                  [(12.0, 0.0, 13.0)],
                  [("A", h["n"], h["pcd"], 3.3, "M4 螺纹孔（底孔 3.3）：goBILDA 轮毂 16 mm 孔型（±8, ±8），M4×16 低头螺丝从轮子外面拧进来", False, 7.0)],
                  ["孔是双 D（对边 10，和 B08 轴端配合，H7 / g6）；Φ13.95 凸台插进 goBILDA 轮毂的 Φ14 孔定心",
                   "按 goBILDA 96 mm 麦轮（3213-3606-0002）官方 3D 模型：轮毂厚 12、在轮子正中、中心孔 Φ14、16 mm 孔型 4 × M4"])


def shaft():
    L = SHAFT_Y1 - SHAFT_Y0
    k1 = (PY0 + 1 - SHAFT_Y0, PY0 + 1 + KEY[2] - SHAFT_Y0)
    k2 = (HUB0 + 2 - SHAFT_Y0, HUB0 + 2 + KEY[2] - SHAFT_Y0)
    g = PY0 - 1.6 - SHAFT_Y0

    def feats(s):
        for a, b in (k1, k2):
            s = s.cut(cq.Solid.makeBox(KEY[0], 3.0, b - a, V(-KEY[0] / 2, HIP_SHAFT_D / 2 - 2.5, a)))
        s = s.cut(cq.Solid.makeCylinder(6.1 / 2, 2.6, V(0, 0, -0.1)))
        s = s.cut(cq.Solid.makeCylinder(7.0, 1.1, V(0, 0, g)).cut(cq.Solid.makeCylinder(11.5 / 2, 1.1, V(0, 0, g))))
        s = s.cut(cq.Solid.makeCylinder(3.3 / 2, 10.0, V(0, 0, L - 10.0 + 0.1)))
        return s
    marks = [(k1[0], k1[1], 2.5, f"键槽 {k1[0]:g}–{k1[1]:g}", 0), (k2[0], k2[1], 2.5, f"键槽 {k2[0]:g}–{k2[1]:g}", 0),
             (g, g + 1.1, 0.25, f"挡圈槽 {g:g}", 1), (0, 2.6, 2.95, "磁铁坑 Φ6.1×2.6", 2), (L - 10, L, 1.35, "M4 深 10", 1)]
    T = Turned("B04", "髋轴", 4, "45# 钢 Φ14（调质）", "数控车 + 铣键槽", [(HIP_SHAFT_D, L)], [], [], extra=feats, marks=marks, notes=[
        f"总长 {L:g}；外径 Φ12 k6（轴承、带轮、轴毂都装在上面）",
        f"键槽 1：距 A 端 {k1[0]:g}–{k1[1]:g}（大带轮）；键槽 2：{k2[0]:g}–{k2[1]:g}（轴毂）；宽 4 N9、深 2.5",
        f"挡圈槽：距 A 端 {g:g}，槽径 Φ11.5、宽 1.1（GB894 轴用挡圈 12）",
        "A 端面：Φ6.1 × 2.6 磁铁坑（AS5600 用的 Φ6×2.5 径向磁铁，用 AB 胶粘）",
        "B 端面：M4 螺纹孔深 10（底孔 3.3），拧螺丝 + Φ14 垫片把轴毂、隔圈、轴承内圈、带轮一起压紧"])
    return T


def spacer(code, name, L, note):
    return Turned(code, name, 4, "6061 铝管或黄铜", "车", [(16.0, L)], [(12.2, 0.0, L)], [], [note])


def arm_shell():
    t = (WHEEL_Y0 + WHEEL_W - 6) - (LP0 + 6)
    s = M.arm_box_solid(0, t)
    return Printed("D01", "箱形臂座外壳", 4, "ASA / PETG（深灰）", s.rotate(V(0, 0, 0), V(1, 0, 0), -90),
                   [f"夹在腿板（A07）和外板（A06）之间（高 {t:g} = 两板间距），不受力，只是外壳",
                    "从髋一直包到轮子跟前，尽头是贴着轮子的弧面（离轮子 4 mm），中间不留空档",
                    "靠机身那面：臂根内板以外的部分封 2 mm 墙；从髋那头装进去"])


def lidar_mount():
    L = LD19
    plate = cq.Workplane("XY").box(58, POD_W, 5, centered=(True, True, False)).val()
    for u, v in L["holes"]:
        plate = plate.cut(cq.Solid.makeCylinder(3.6 / 2, 6, V(u, v, -0.5)))
    plate = plate.cut(cq.Solid.makeBox(14, 8, 7, V(-7, -L["ear_d"] / 2 + 2, -1)))
    return Printed("D02", "雷达底座", 1, "PETG（黑）", plate,
                   ["坐在左肩座顶上；3 个 M2.5 热熔螺母（外径 3.6）对 LD19 的 3 个安装孔", "中间后侧 14 × 8 过线口"])


def fence(name, length, h):
    t = 5.8
    s = cq.Workplane("XY").box(length, t, h, centered=(True, True, False)).val()
    # 六边形镂空
    r = 9.0
    row = 0
    z = 14.0
    while z < h - 10:
        x = -length / 2 + 14 + (row % 2) * r * 0.9
        while x < length / 2 - 12:
            s = s.cut(cq.Workplane("XZ").polygon(6, 2 * r * 0.82).extrude(t + 2).translate((x, t / 2 + 1, z)).val())
            x += r * 1.8
        z += r * 1.56
        row += 1
    return s


def fences():
    out = []
    side_len = BED_X1 - BED_X0 - 10
    inner = 2 * (RAIL_Y - PROFILE / 2) - 4
    out.append(Printed("D03", "侧栏板", 2, "PETG / ASA", fence("侧栏板", side_len, FENCE_H),
                       [f"长 {side_len:g}、高 {FENCE_H:g}、厚 5.8：下边插进 2020 导轨上面的槽（槽口 6.2），插深 {FENCE_INSERT:g}"]))
    out.append(Printed("D04", "后栏板", 1, "PETG / ASA", fence("后栏板", inner, FENCE_H), ["同上，插后导轨"]))
    out.append(Printed("D05", "前栏板", 1, "PETG / ASA", fence("前栏板", inner, 50.0), ["比两侧矮（高 50），不挡头往后看"]))
    return out


def skid():
    s = cq.Workplane("XY").box(300, 20, SKID_H, centered=(True, True, False)).edges("|Z").fillet(4).val()
    for x in (-130.0, -45.0, 45.0, 130.0):
        s = s.cut(cq.Solid.makeCylinder(2.2, SKID_H + 2, V(x, 0, -1)))
    return Printed("D06", "滑撬", 2, "PETG 或 TPU 95A", s, ["趴平时整机坐在上面；M4 从底板里面往下拧（自攻）"])


def from_model(code, name, qty, material, key, notes, group="neck"):
    parts = {"neck": M.neck_parts, "head": M.head_parts, "body": M.body_parts, "leg": M.leg_parts_FL}[group]()
    s = [p for n, p, c in parts if n == key][0]
    return Printed(code, name, qty, material, s, notes)


def all_parts():
    P = [side_plate(), deck(), bottom(), end_plate(True), end_plate(False), outer_arm(), root_plate(), tray(),
         housing(), hub(), axle_housing(), axle(), hub_adapter(), shaft(),
         spacer("B05", "隔圈A", 5.0, "带轮和轴承内圈之间，穿过侧板 Φ22 孔"),
         spacer("B06", "隔圈B", 3.0, "轴承内圈和轴毂之间"),
         arm_shell(), *fences(), skid(),
         from_model("D02", "雷达法兰", 1, "PETG（黑）或铝", "雷达法兰", ["电机耳顶上，高 15，宽不超过耳朵；4 颗 M3；LD19 三个 M2.5 热熔螺母"]),
         from_model("D07", "屏幕模块外壳", 1, "ASA（浅灰白，哑光喷涂）", "屏幕头",
                    ["空心壳，壁厚 2.5；前框 / 顶盖 / 侧板之间 1.5 mm 分缝（打印时就是分开的几块）；前脸 118 × 50 窗；下巴相机、测距、隐私灯孔；"
                     "屏幕下一排 9 个 Φ2.4 麦克风孔；两侧 4×4 扬声器孔、4 条散热槽；左边 Φ10 过舵盘、右边 Φ8.2 过轴销"], "head"),
         from_model("D08", "屏板卡槽（左）", 1, "PETG", "屏板卡槽左", ["屏板侧边插进槽里；右边那个镜像打印"], "head"),
         from_model("D09", "屏板压条", 1, "PETG", "屏板压条", ["从后面压住屏板，两头用 M2.5 拧在卡槽上"], "head"),
         from_model("D10", "电机耳骨架", 1, "ASA（浅灰白，哑光喷涂）", "左耳骨架", ["面对机器人时的右耳：里面是 STS3215 舵机仓、舵盘过孔"]),
         from_model("D11", "方窗耳骨架", 1, "ASA（浅灰白，哑光喷涂）", "右耳骨架", ["面对机器人时的左耳：铜套孔、热成像 / 红外小板仓"]),
         from_model("D12", "耳朵中板", 2, "ASA（浅灰白，哑光喷涂）", "左耳中板", ["三层叠板的中间一层（比骨架缩 4）；左右镜像各一块"]),
         from_model("D13", "耳朵外板", 2, "ASA（浅灰白，哑光喷涂）", "左耳外板", ["最外层（缩 8）；上面切出可拆侧盖（1.5 mm 缝），电机耳那块带 4 条散热槽"]),
         from_model("D14", "脖子底座", 1, "PETG / 铝", "脖子底座", ["U 形，两只耳朵拧在上面；后沿让开货斗"]),
         from_model("D18", "转轴壳", 2, "PETG（深灰）", "左转轴壳", ["露在屏幕模块和耳朵之间的缝里；中间 Φ12 过轴"]),
         from_model("D15", "前保险杠", 1, "ASA（深灰）", "前保险杠", ["中间开深度相机窗，两边雾灯槽"], "body"),
         from_model("D16", "后格栅", 1, "PETG（黑）", "后格栅", ["6 片银色鳍可以用铝条或打印后喷银"], "body"),
         from_model("D17", "侧挂板", 2, "ASA（深灰）", "侧挂板左", ["独立挂在机身两侧，四角 M5 沉头；和下层外壳之间一圈 2 mm 缝；左右镜像"], "body"),
         from_model("D19", "下层外壳", 1, "ASA（浅灰白，哑光喷涂）", "外壳·下层外壳", ["架在底盘边梁上面（中间留 6 mm 缝）；前后各一道 2 mm 缝分成 4 块打印"], "body"),
         from_model("D24", "上层甲板壳", 1, "ASA（浅灰白，哑光喷涂）", "上层甲板壳", ["机身上半截一整圈，前后各两道斜面；前后各一道 2 mm 缝分成 4 块打印"], "body"),
         from_model("D25", "黑腰", 1, "PETG（黑）", "外壳·黑腰", ["上下两层中间凹进去的黑色圈，里面贴灯条"], "body"),
         from_model("D26", "头部平台盖", 1, "ASA（浅灰白，哑光喷涂）", "头部平台盖", ["顶面前段，厚 7，比货斗底板高 4，往前悬挑 9；下面带缩进 3 的垫条"], "body"),
         from_model("D27", "货斗底板", 1, "ASA（浅灰白，哑光喷涂）", "货斗底板", ["顶面中段，厚 7，货放在这里；导轨坐在上面"], "body"),
         from_model("D28", "后检修盖", 1, "ASA（浅灰白，哑光喷涂）", "后检修盖", ["顶面后段，厚 7，比货斗底板高 4，往后悬挑 9"], "body"),
         from_model("D31", "髋支座", 4, "6061-T6 铝板 7 mm（激光切割，深灰阳极）", "髋支座前左",
                    ["下面两颗 M5 拧在底盘边梁上、上面两颗拧在侧板上；中间 Φ47 孔让髋轴承座穿出来；四块一样（翻面用）"], "body"),
         from_model("D22", "摆臂外盖板", 4, "ASA（深灰）", "摆臂外盖板", ["贴在外板外面（2 mm），中间长槽；轮子那头圆弧缺口；黄条用反光贴纸"], "leg"),
         from_model("D23", "髋部大圆盖", 4, "ASA（深灰）", "髋部大圆盖", ["Φ46，盖住髋轴端头和 AS5600 编码器；外圈贴黄色反光圈"], "leg"),
         from_model("D29", "电机护罩（兼电机支架）", 4, "PA12-CF 尼龙碳纤（或 PETG-CF）", "电机护罩",
                    ["切角八边形“靴子”，背面 4 颗 M4 拧在腿板上，电机背面 4 颗 M3 拧在护罩里（它是电机的支架，要扛电机反扭矩，别用普通 PLA）",
                     "朝机身那头敞开露出电机端面；底面 4 道散热槽；后上方出线口"], "leg"),
         from_model("D32", "编码器支架", 4, "PETG", "前左编码器支架", ["托住 Adafruit AS5600 板（4 × M2.5 自攻），板上芯片正对髋轴端头的磁铁（间隙 1 mm）",
                                                                   "从大带轮上面绕过去，2 颗 M3 拧在侧板上（热熔螺母）；左右腿镜像打印"], "body")]
    return P


def profile_cuts():
    inner = 2 * (RAIL_Y - PROFILE / 2)
    return [dict(name="机身横梁 2020", qty=4, length=BODY_W - 2 * PLATE_SIDE, note="两端攻 M5（型材中心孔）"),
            dict(name="货斗侧导轨 2020", qty=2, length=BED_X1 - BED_X0, note="平放在甲板上，栏板插上面的槽"),
            dict(name="货斗前后导轨 2020", qty=2, length=inner, note="夹在两根侧导轨中间"),
            dict(name="底盘边梁（6063 铝方管 26×10×1.5）", qty=2, length=BODY_L + 20, note="贴在侧板外面下沿，两端斜切 5，深灰阳极"),
            dict(name="底盘横管（6063 铝管 Φ16×1.5）", qty=2, length=BODY_W, note="前后各一根，两端塞银色端盖，M5 拧在边梁上")]


def main():
    data = []
    for p in all_parts():
        files = p.export()
        info = p.info()
        info["files"] = files
        data.append(info)
        print(f"{p.code} {p.name} ×{p.qty}：{', '.join(files)}")
    with open(os.path.join(OUT, "parts.json"), "w", encoding="utf-8") as f:
        json.dump(dict(parts=data, profiles=profile_cuts()), f, ensure_ascii=False)
    print(f"共 {len(data)} 种零件，输出在 007v/out/mfg/")


if __name__ == "__main__":
    main()
