"""机械狗007v 外壳和头部内部结构（照外观效果图 v10 做到真实骨架上）。

名字以“外壳·”开头的是装饰件（灯带、螺栓、分缝、镜头），只查它和别的运动部件撞不撞，不查它和同一部件里的零件。
其余（屏幕头外壳、肩座三层、屏板卡槽、舵盘、轴销）是要打印 / 加工的真零件。

间隙规矩（碰撞检查和折叠扫描会验证）：
  - 机身侧面 z < 95 的下半截：折叠的轮子扫不到，侧装甲可以凸出 8 mm；
  - z 92–100 的腰线灯槽、甲板边线只凸出 1–2 mm（上半截离折叠的轮子只有 5 mm）；
  - 肩座所有层都在 y 72–120 以内，头在 ±68 以内。
"""
import math

import cadquery as cq

from config import *  # noqa: F401,F403

V = cq.Vector


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl_y(cx, cz, d, y0, y1):
    return cq.Solid.makeCylinder(d / 2, y1 - y0, V(cx, y0, cz), V(0, 1, 0))


def cyl_dir(p, n, d, h):
    """以 p 为底面中心、沿 n 方向的圆柱。"""
    L = math.sqrt(sum(c * c for c in n))
    return cq.Solid.makeCylinder(d / 2, h, V(*p), V(n[0] / L, n[1] / L, n[2] / L))


def prism_xz(points, y0, y1, offset=0.0):
    """侧面轮廓（x, z 点）沿 Y 拉伸到 y0..y1；offset < 0 往里缩。"""
    wp = cq.Workplane("XY").polyline(points).close()
    if offset:
        wp = wp.offset2D(offset, "intersection")
    wp = wp.extrude(y1 - y0)
    return wp.val().rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(0, y1, 0))


def ring_y(cx, cz, d_out, d_in, y0, y1):
    return cyl_y(cx, cz, d_out, y0, y1).cut(cyl_y(cx, cz, d_in, y0 - 1, y1 + 1))


def strip(p0, p1, w, y0, y1, inward=(0.0, 0.0)):
    """侧面上从 p0 到 p1 的一条细带（宽 w，x-z 平面里），可以沿法线方向挪 inward。"""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dz)
    nx, nz = -dz / L * w / 2, dx / L * w / 2
    ox, oz = inward
    pts = [(p0[0] + nx + ox, p0[1] + nz + oz), (p1[0] + nx + ox, p1[1] + nz + oz),
           (p1[0] - nx + ox, p1[1] - nz + oz), (p0[0] - nx + ox, p0[1] - nz + oz)]
    return prism_xz(pts, y0, y1)


def chamfer_rect(x0, x1, z0, z1, c):
    return [(x0 + c, z0), (x1 - c, z0), (x1, z0 + c), (x1, z1 - c), (x1 - c, z1), (x0 + c, z1), (x0, z1 - c), (x0, z0 + c)]


def soft(s, r):
    """把一个零件的棱边倒圆（半径 r）；倒不了就退成倒角，再不行原样返回。打印件也按这个出，所以实物也是圆边。"""
    for fn in (lambda: cq.Workplane().add(s).edges().fillet(r).val(),
               lambda: cq.Workplane().add(s).edges().chamfer(r * 0.8).val(),
               lambda: cq.Workplane().add(s).edges().fillet(r * 0.5).val()):
        try:
            out = fn()
            if out.isValid() and out.Volume() > 0.5 * s.Volume():
                return out
        except Exception:
            pass
    return s


def mirror_y(s):
    return s.mirror("XZ")


# ================================================================ 机身外壳（机身坐标）
def rrect_ring(hx, hy, r, z0, z1, ix=None, iy=None):
    """圆角矩形外框（半长 hx、半宽 hy、圆角 r），挖掉中间的机身盒子（半长 ix、半宽 iy）。"""
    ix = BODY_L / 2 if ix is None else ix
    iy = BODY_W / 2 if iy is None else iy
    o = cq.Workplane("XY").box(2 * hx, 2 * hy, z1 - z0, centered=(True, True, False)).edges("|Z").fillet(r).val().translate(V(0, 0, z0))
    return o.cut(box(-ix, ix, -iy, iy, z0 - 1, z1 + 1))


TUB_TOP, WAIST_TOP = 67.0, 83.0          # 照效果图比例：下层底盘 67、黑腰 16、上层甲板 67（机身 153 高）


def body_shell():
    P = []
    W2, L2 = BODY_W / 2, BODY_L / 2
    deck = BODY_H + PLATE
    # ---- 下层底盘：一整圈，四角 R16，底边斜切；凸出机身 6（只在下半截，折叠的轮子扫不到）
    tub = rrect_ring(L2 + 4, W2 + 6, 16, 12, TUB_TOP).fuse(rrect_ring(L2 - 2, W2, 10, 4, 12.01, L2 - 2, W2 - 0.01))
    tub = tub.fuse(prism_xz([(-L2 + 2, 4), (L2 - 2, 4), (L2 + 4, 12), (-L2 - 4, 12)], W2 - 1, W2 + 6))
    tub = tub.fuse(prism_xz([(-L2 + 2, 4), (L2 - 2, 4), (L2 + 4, 12), (-L2 - 4, 12)], -W2 - 6, -W2 + 1))
    tub = tub.cut(box(-L2 + 0.01, L2 - 0.01, -W2 + 0.01, W2 - 0.01, 0, TUB_TOP + 1))
    for hx in (-HIP_X, HIP_X):
        tub = tub.cut(cyl_y(hx, HIP_Z, 62, -W2 - 8, W2 + 8))                    # 髋部像轮拱一样让开
    dx, dy, dz = DEPTH_CAM
    tub = tub.cut(box(L2 - 1, L2 + 8, -dy / 2 - 1, dy / 2 + 1, 23, 24 + dz + 2))   # 深度相机窗
    tub = tub.cut(cq.Solid.makeCylinder(13, 10, V(-L2 - 8, 0, 70), V(1, 0, 0)))    # 急停
    tub = tub.cut(cq.Solid.makeCylinder(9, 10, V(-L2 - 8, 55, 70), V(1, 0, 0)))    # 总开关
    tub = tub.cut(box(-L2 - 8, -L2 + 1, -71, -53, 59, 69))                         # 充电口
    P.append(("外壳·下层底盘", soft(tub, 1.5), "shell"))
    # ---- 黑腰：凹进去的一圈 + 两侧琥珀灯、前后青灯
    P.append(("外壳·黑腰", rrect_ring(L2 + 1.5, W2 + 1.5, 13, TUB_TOP, WAIST_TOP), "dark"))
    for sy in (1, -1):
        for x in (-130, 130):
            P.append((f"外壳·腰线灯 y{sy} x{x}", box(x - 35, x + 35, *sorted((sy * (W2 + 1.5), sy * (W2 + 2.1))), 73.5, 76.5), "amber"))
    for sx in (1, -1):
        P.append((f"外壳·腰线青灯 x{sx}", box(*sorted((sx * (L2 + 1.5), sx * (L2 + 2.1))), -60, 60, 73.5, 76.5), "cyan"))
    # ---- 上层甲板：薄皮（侧面只凸 1.5，上半截离折叠的轮子只有 5 mm）+ 前后大斜面
    P.append(("外壳·上层甲板皮", soft(rrect_ring(L2 + 1.5, W2 + 1.5, 12, WAIST_TOP, BODY_H), 0.6), "shell"))
    for sx in (1, -1):
        nose = prism_xz([(0, WAIST_TOP), (16, WAIST_TOP + 14), (16, BODY_H - 14), (0, BODY_H)], -W2 + 6, W2 - 6)
        if sx < 0:
            nose = nose.mirror("YZ")
        P.append((f"{'前' if sx > 0 else '后'}鼻子", soft(nose, 4.0).translate(V(sx * (L2 + 1.5), 0, 0)), "deck"))
    for sy, tag in ((1, "左"), (-1, "右")):
        out = []
        # 侧装甲模块：贴在下层底盘外面
        frame = prism_xz(chamfer_rect(-100, 100, 16, 62, 6), W2 + 6, W2 + 12).cut(prism_xz(chamfer_rect(-90, 90, 24, 54, 4), W2 + 5, W2 + 13))
        out.append((f"外壳·{tag}侧装甲框", soft(frame, 2.5), "dark"))
        out.append((f"外壳·{tag}侧装甲面板", prism_xz(chamfer_rect(-90, 90, 24, 54, 4), W2 + 6, W2 + 9), "dark"))
        for i in range(7):
            out.append((f"外壳·{tag}散热槽{i + 1}", box(30 + i * 8, 33 + i * 8, W2 + 9, W2 + 9.6, 29, 49), "seam"))
        out.append((f"外壳·{tag}警示条", box(-75, -15, W2 + 9, W2 + 9.6, 44, 50), "amber"))
        for i in range(6):
            x0 = -75 + i * 10
            out.append((f"外壳·{tag}斜纹{i + 1}", prism_xz([(x0, 44), (x0 + 5, 44), (x0 + 9, 50), (x0 + 4, 50)], W2 + 9, W2 + 9.8), "black"))
        for x, z in ((-94, 22), (94, 22), (-94, 56), (94, 56)):
            out.append((f"外壳·{tag}螺栓 x{x}z{z}", cyl_y(x, z, 6, W2 + 12, W2 + 13.5), "silver"))
        out.append((f"外壳·{tag}甲板边线", box(-170, 170, W2 + 1.5, W2 + 1.8, BODY_H - 6, BODY_H - 4), "silver"))
        for n, s_, c in out:
            P.append((n, s_ if sy > 0 else mirror_y(s_), c))
        txt = cq.Workplane("XZ").text("007V", 34, 0.6, halign="center", valign="center").val()
        small = cq.Workplane("XZ").text("CARGO 10KG  ·  MAX STEP 0.10M", 6, 0.4, halign="left", valign="center").val()
        if sy > 0:
            txt = txt.translate(V(-40, 0, 0)).rotate(V(0, 0, 0), V(0, 0, 1), 180).translate(V(0, W2 + 1.5, 118))
            small = small.rotate(V(0, 0, 0), V(0, 0, 1), 180).translate(V(5, W2 + 1.5, 122))
        else:
            txt = txt.translate(V(-40, -W2 - 1.5, 118))
            small = small.translate(V(-5, -W2 - 1.5, 122))
        P.append((f"外壳·{tag}字样007V", txt, "silver"))
        P.append((f"外壳·{tag}小字", small, "silver"))
    # 前保险杠：上下斜切，中间给深度相机开窗；两边琥珀雾灯
    L2 = BODY_L / 2
    bump = prism_xz([(L2, 15), (L2 + 12, 21), (L2 + 12, 59), (L2, 65)], -100, 100)
    dx, dy, dz = DEPTH_CAM
    bump = bump.cut(box(L2 - 1, L2 + 13, -dy / 2 - 1, dy / 2 + 1, 24, 24 + dz + 2))
    P.append(("前保险杠", soft(bump, 3.0), "dark"))
    P.append(("外壳·深度相机窗", box(L2 + 9, L2 + 10.5, -dy / 2, dy / 2, 25, 24 + dz + 1), "gloss"))
    for sy in (1, -1):
        P.append((f"外壳·雾灯{'左' if sy > 0 else '右'}", box(L2 + 12, L2 + 13.5, *sorted((sy * 64, sy * 88)), 36, 44), "amber"))
    # 尾部：散热格栅 + 6 片银色鳍 + 两条琥珀尾灯
    P.append(("后格栅", box(-L2 - 8, -L2, -80, 80, 18, 52), "black"))
    for i in range(6):
        y = -60 + i * 24
        P.append((f"外壳·格栅鳍{i + 1}", box(-L2 - 10, -L2 - 8, y - 1, y + 1, 20, 50), "silver"))
    for sy in (1, -1):
        P.append((f"外壳·尾灯{'左' if sy > 0 else '右'}", box(-L2 - 18.1, -L2 - 17.4, *sorted((sy * 40, sy * 100)), 118, 126), "amber"))
    # 底光（青色，夜里照地面）
    for sy in (1, -1):
        P.append((f"外壳·底光{'左' if sy > 0 else '右'}", box(-140, 140, *sorted((sy * 30, sy * 36)), -1.5, 0), "cyan"))
    # 栏板顶边的琥珀反光条
    rail_top = deck + PROFILE
    ftop = rail_top - FENCE_INSERT + FENCE_H
    for sy in (1, -1):
        P.append((f"外壳·栏板反光条{'左' if sy > 0 else '右'}", box(BED_X0 + 15, BED_X1 - 15, sy * RAIL_Y - 3.5, sy * RAIL_Y + 3.5, ftop - 6, ftop + 0.5), "amber"))
    return P


# ================================================================ 屏幕头（俯仰轴坐标）
def head():
    hw = HEAD_W / 2
    wall = 2.5
    front = max(x for x, _ in HEAD_PROFILE)
    gw, gh = SCREEN_GLASS
    t, w, h = SCREEN_BOARD
    shell = prism_xz(HEAD_PROFILE, -hw, hw).cut(prism_xz(HEAD_PROFILE, -hw + wall, hw - wall, offset=-wall))
    shell = shell.cut(box(front - wall - 1, front + 1, -gw / 2, gw / 2, SCREEN_Z - gh / 2, SCREEN_Z + gh / 2))     # 前脸开窗
    shell = shell.cut(cyl_y(0, 0, 10, hw - wall - 1, hw + 1))                                                      # 左边：舵盘法兰穿出去
    shell = shell.cut(cyl_y(0, 0, 8.2, -hw - 1, -hw + wall + 1))                                                   # 右边：俯仰轴销
    # 下巴斜面：相机、测距、隐私灯开孔
    c0, c1 = (56, -42), (84, -24)
    L = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
    n = ((c1[1] - c0[1]) / L, -(c1[0] - c0[0]) / L)            # 朝外（朝前下）
    mid = ((c0[0] + c1[0]) / 2, (c0[1] + c1[1]) / 2)
    sensors = [("相机镜头", 0.0, 12.0, "black"), ("测距窗 VL53L5CX", -30.0, 8.0, "gloss"), ("隐私指示灯", 30.0, 4.0, "amber")]
    for _, y, d, _ in sensors:
        shell = shell.cut(cyl_dir((mid[0] - n[0] * 4, y, mid[1] - n[1] * 4), (n[0], 0, n[1]), d + 1, 8))
    P = [("屏幕头", soft(shell, 4.0), "deck")]
    board_x0 = SCREEN_X - t
    P.append(("4.3寸屏板", box(board_x0, SCREEN_X, -w / 2, w / 2, SCREEN_Z - h / 2, SCREEN_Z + h / 2), "pcb"))
    P.append(("屏幕玻璃", box(front - 1.5, front + 0.5, -gw / 2, gw / 2, SCREEN_Z - gh / 2, SCREEN_Z + gh / 2), "glass"))
    # 屏板卡槽：两侧各一条导轨（屏板从后面推进去顶住前墙），后面一根压条压住
    for sy in (1, -1):
        g = box(board_x0 - 4, SCREEN_X, w / 2 + 0.3, hw - wall, SCREEN_Z - h / 2, SCREEN_Z + h / 2)
        g = g.cut(box(board_x0 - 0.3, SCREEN_X + 0.3, w / 2 - 3, w / 2 + 0.6, SCREEN_Z - h / 2 - 0.3, SCREEN_Z + h / 2 + 0.3))
        P.append((f"屏板卡槽{'左' if sy > 0 else '右'}", g if sy > 0 else mirror_y(g), "printed"))
    P.append(("屏板压条", box(board_x0 - 4, board_x0, -w / 2 + 6, w / 2 - 6, SCREEN_Z - 6, SCREEN_Z + 6), "printed"))
    # 左边舵盘法兰（接舵机，跟着头转）、右边轴销（插进右肩座的铜套）
    P.append(("舵盘法兰", cyl_y(0, 0, 9.5, hw - wall - 2, POD_IN + POD_W / 2 - SERVO[1] / 2 - 0.5), "silver"))
    P.append(("俯仰轴销", cyl_y(0, 0, 8.0, -POD_IN - 12, -hw + wall + 2), "steel"))
    # 外面：银色屏框、深色眉、下巴传感器、脑后散热片
    frame = box(front, front + 2, -gw / 2 - 6, gw / 2 + 6, SCREEN_Z - gh / 2 - 6, SCREEN_Z + gh / 2 + 6).cut(
        box(front - 1, front + 3, -gw / 2, gw / 2, SCREEN_Z - gh / 2, SCREEN_Z + gh / 2))
    P.append(("外壳·屏框", frame, "silver"))
    b0, b1 = (72, 66), (84, 54)
    P.append(("外壳·眉", prism_xz([b0, b1, (b1[0] + 1.8, b1[1] + 1.8), (b0[0] + 1.8, b0[1] + 1.8)], -hw + 2, hw - 2), "dark"))
    for name, y, d, col in sensors:
        P.append((f"外壳·{name}", cyl_dir((mid[0] - n[0] * 3, y, mid[1] - n[1] * 3), (n[0], 0, n[1]), d, 4.5), col))
        if name == "相机镜头":
            P.append(("外壳·镜头银圈", cyl_dir((mid[0] - n[0] * 0.5, y, mid[1] - n[1] * 0.5), (n[0], 0, n[1]), d + 4, 1.0), "silver"))
    x_back = min(x for x, _ in HEAD_PROFILE)
    for i in range(5):
        y = -40 + i * 20
        P.append((f"外壳·脑后散热片{i + 1}", box(x_back - 5, x_back + 0.5, y - 1.5, y + 1.5, -12, 36), "silver"))
    return P


# ================================================================ 两块肩座（脖子坐标：原点在甲板上表面、俯仰轴正下方）
def shrink(pts, k, c=(16.0, PIVOT_UP)):
    return [(c[0] + (x - c[0]) * k, c[1] + (z - c[1]) * k) for x, z in pts]


def pods():
    P = []
    hexp = [(x, z) for x, z in POD_HEX]
    y_in, y_out = POD_IN, POD_IN + POD_W            # 72 … 120
    sx, sw, sh = SERVO
    yc = (y_in + y_out) / 2
    up0, up1 = (78, 62), (50, 122)                   # 上前斜边
    lo0, lo1 = (48, 16), (78, 62)                    # 下前斜边
    lerp = lambda a, b, t: (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)  # noqa: E731
    for sy, tag in ((1, "左"), (-1, "右")):
        out = []
        seal = prism_xz(hexp, y_in, y_in + 2).cut(cyl_y(0, PIVOT_UP, 14 if sy > 0 else 10, y_in - 1, y_in + 3))
        out.append((f"{tag}肩座封板", soft(seal, 0.8), "shell"))
        core = prism_xz(hexp, y_in + 2, y_out - 14)
        if sy > 0:   # 左：舵机仓 + 舵盘过孔 + 往下的走线孔
            core = core.cut(box(-sx / 2 - 0.5, sx / 2 + 0.5, yc - sw / 2 - 0.5, y_out - 13, PIVOT_UP - sh / 2 - 0.5, PIVOT_UP + sh / 2 + 0.5))
            core = core.cut(cyl_y(0, PIVOT_UP, 14, y_in, yc))
            core = core.cut(cq.Solid.makeCylinder(6, PIVOT_UP, V(CABLE_X, yc, 0)))
        else:        # 右：轴销铜套孔 + 传感器小板仓
            core = core.cut(cyl_y(0, PIVOT_UP, 10, y_in, y_in + 20))
            core = core.cut(box(30, 62, y_in + 6, y_out - 15, 70, 104))
        out.append((f"{tag}肩座骨架", soft(core, 4.0), "dark"))
        armor = prism_xz(shrink(hexp, 0.86), y_out - 10, y_out - 4).cut(cyl_y(0, PIVOT_UP, 38, y_out - 11, y_out - 3))
        out.append((f"{tag}肩座外装甲", soft(armor, 2.8), "deck"))
        for i, (x, z) in enumerate(((50, 30), (42, 106), (-30, 62))):
            out.append((f"外壳·{tag}肩座支柱{i + 1}", cyl_y(x, z, 6, y_out - 14, y_out - 10), "silver"))
        out.append((f"外壳·{tag}电机端盖", cyl_y(0, PIVOT_UP, 24, y_out - 14, y_out - 6), "silver"))
        out.append((f"外壳·{tag}端盖六角", cyl_y(0, PIVOT_UP, 9, y_out - 6, y_out - 5), "dark"))
        out.append((f"外壳·{tag}端盖青光圈", ring_y(0, PIVOT_UP, 40, 38, y_out - 4, y_out - 3.4), "cyan"))
        for i in range(8):
            a = 2 * math.pi * i / 8
            out.append((f"外壳·{tag}端盖螺栓{i + 1}", cyl_y(25 * math.cos(a), PIVOT_UP + 25 * math.sin(a), 3.2, y_out - 4, y_out - 3), "silver"))
        out.append((f"外壳·{tag}肩座分缝", strip((8.3, 80.5), (-32.3, 109.5), 1.4, y_out - 4, y_out - 3.5), "seam"))
        a0, a1 = shrink([lerp(up0, up1, 0.18), lerp(up0, up1, 0.82)], 0.86)
        out.append((f"外壳·{tag}肩座光槽", strip(a0, a1, 2.2, y_out - 4, y_out - 3.5, inward=(-3.5, -1.6)), "cyan"))
        for p0, p1 in ((lerp(lo0, lo1, 0.1), lerp(lo0, lo1, 0.95)), (lerp(up0, up1, 0.05), lerp(up0, up1, 0.9))):
            out.append((f"外壳·{tag}缝灯 {p0[1]:.0f}", strip(p0, p1, 2.6, y_in - 0.5, y_in, inward=(-2.0, 0.0)), "cyan"))
        lamp0, lamp1 = lerp(lo0, lo1, 0.35), lerp(lo0, lo1, 0.65)
        out.append((f"外壳·{tag}肩座琥珀灯", strip(lamp0, lamp1, 3.0, yc - 5, yc + 5, inward=(1.4, -0.9)), "amber"))
        if sy < 0:   # 右肩座：热成像窗、两颗红外管、短天线
            Ln = math.hypot(up1[0] - up0[0], up1[1] - up0[1])
            nrm = ((up1[1] - up0[1]) / Ln, -(up1[0] - up0[0]) / Ln)      # 上前斜面朝外
            k = 18 / Ln
            out.append(("外壳·热成像窗 MLX90640", strip(lerp(up0, up1, 0.22), lerp(up0, up1, 0.22 + k), 2.0, yc - 9, yc + 9,
                                                       inward=(nrm[0] * 0.5, nrm[1] * 0.5)), "gloss"))
            out.append(("外壳·热成像银框", strip(lerp(up0, up1, 0.22 - 2 / Ln), lerp(up0, up1, 0.22 + k + 2 / Ln), 1.0, yc - 11, yc + 11,
                                              inward=(nrm[0] * 0.2, nrm[1] * 0.2)), "silver"))
            for dy in (-8, 8):
                p = lerp(up0, up1, 0.72)
                out.append((f"外壳·红外管{'甲' if dy < 0 else '乙'}", cyl_dir((p[0] - nrm[0] * 2, yc + dy, p[1] - nrm[1] * 2), (nrm[0], 0, nrm[1]), 5, 3.5), "black"))
            top = max(z for _, z in POD_HEX)
            out.append(("外壳·天线", box(-4, 1, yc - 5, yc + 5, top, top + 30), "dark"))
            out.append(("外壳·天线灯", box(-4.5, 1.5, yc - 5.5, yc + 5.5, top + 30, top + 35), "amber"))
        for n, s, col in out:
            P.append((n, s if sy > 0 else mirror_y(s), col))
    # 头部俯仰舵机（左肩座里），右肩座里一个铜套
    P.append(("俯仰舵机", box(-sx / 2, sx / 2, yc - sw / 2, yc + sw / 2, PIVOT_UP - sh / 2, PIVOT_UP + sh / 2), "motor"))
    P.append(("俯仰铜套", ring_y(0, PIVOT_UP, 10, 8.1, -y_in - 20, -y_in - 2), "trim"))
    return P


# ================================================================ 摆臂外板上的琥珀反光条（左前腿坐标）
def stadium_pts(x0, x1, r0, r1, n=24):
    """两头圆（半径 r0 在 x0、r1 在 x1）的外公切线轮廓（x, z）。"""
    L = x1 - x0
    a = math.asin((r0 - r1) / L)
    pts = []
    for i in range(n + 1):
        t = -math.pi / 2 + a + (math.pi - 2 * a) * i / n
        pts.append((x1 + r1 * math.cos(t), r1 * math.sin(t)))
    for i in range(n + 1):
        t = math.pi / 2 - a + (math.pi + 2 * a) * i / n
        pts.append((x0 + r0 * math.cos(t), r0 * math.sin(t)))
    return pts


def arm_marks(oy1, root1, my1):
    """摆臂外面（左前腿坐标，髋轴在原点，轮轴在 (ARM, 0)）：外盖板 + 长槽 + 黄条、髋部大圆盖、轮端青光圈、臂座上的黄色箍。"""
    P = []
    cover = prism_xz(stadium_pts(0, ARM, ARM_ROOT_R - 1, ARM_END_R - 1), oy1, oy1 + 3)
    cover = cover.cut(prism_xz(stadium_pts(48, 128, 6, 6), oy1 - 1, oy1 + 4))
    P.append(("摆臂外盖板", soft(cover, 1.2), "dark"))
    P.append(("外壳·长槽底", prism_xz(stadium_pts(48, 128, 6, 6), oy1, oy1 + 1), "black"))
    for i, u in enumerate((34, 41, 48)):                                   # 髋边上三道斜黄条
        P.append((f"外壳·摆臂黄条{i + 1}", strip((u, -14), (u + 10, 14), 2.4, oy1 + 3, oy1 + 3.6), "amber"))
    P.append(("外壳·摆臂边黄线", strip((60, 16), (135, 19), 1.6, oy1 + 3, oy1 + 3.6), "amber"))
    P.append(("髋部大圆盖", soft(cyl_y(0, 0, 50, oy1 + 3, oy1 + 6), 1.2), "dark"))
    P.append(("外壳·髋盖黄圈", ring_y(0, 0, 50, 40, oy1 + 6, oy1 + 6.6), "amber"))
    P.append(("外壳·髋盖六角", prism_xz([(9 * math.cos(math.pi / 6 + k * math.pi / 3), 9 * math.sin(math.pi / 6 + k * math.pi / 3)) for k in range(6)], oy1 + 6, oy1 + 8), "silver"))
    P.append(("外壳·轮端青光圈", ring_y(ARM, 0, 26, 20, oy1 + 3, oy1 + 3.6), "cyan"))
    for i, y in enumerate((root1 + 8, root1 + 16, root1 + 24, root1 + 32)):    # 臂座上的黄色箍（像减震弹簧）
        P.append((f"外壳·臂座黄箍{i + 1}", prism_xz(stadium_pts(0, ARM_BOX_LEN - ARM_ROOT_R, ARM_ROOT_R + 1.2, ARM_ROOT_R + 1.2), y, y + 2.5)
                  .cut(prism_xz(stadium_pts(0, ARM_BOX_LEN - ARM_ROOT_R, ARM_ROOT_R - 0.5, ARM_ROOT_R - 0.5), y - 1, y + 4)), "amber"))
    return P
