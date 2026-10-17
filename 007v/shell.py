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

BOLT_D = 10.0                       # 全机统一的沉头螺栓头直径（机身、头、腿都用这个）

V = cq.Vector


def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl_z(cx, cy, d, z0, z1):
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(cx, cy, z0), V(0, 0, 1))


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
    """棱边倒小角（硬表面风格，2026-10-18 起不再倒大圆角）：r ≥ 2 倒 1.5 mm，否则 1 mm；倒不了退一半，再不行原样返回。"""
    c = 1.5 if r >= 2 else 1.0
    for k in (1.0, 0.5):
        try:
            out = cq.Workplane().add(s).edges().chamfer(c * k).val()
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
    o = cq.Workplane("XY").box(2 * hx, 2 * hy, z1 - z0, centered=(True, True, False)).edges("|Z").chamfer(min(r, 2.0)).val().translate(V(0, 0, z0))
    return o.cut(box(-ix, ix, -iy, iy, z0 - 1, z1 + 1))


TUB_TOP, WAIST_TOP = 67.0, 83.0          # 照效果图比例：下层底盘 67、黑腰 16、上层甲板 67（机身 153 高）


def body_shell():
    P = []
    W2, L2 = BODY_W / 2, BODY_L / 2
    deck = BODY_H + PLATE
    # ---- 底盘框架（外露的金属骨架）：两根矩形边梁贴在侧板外面 + 前后两根横管；外壳架在它上面，中间留一道看得见的缝
    rw, rz0, rz1 = FRAME_RAIL
    for sy, tag in ((1, "左"), (-1, "右")):
        rail = soft(prism_xz(chamfer_rect(-L2 - 10, L2 + 10, rz0, rz1, 5), *sorted((sy * W2, sy * (W2 + rw)))), 1.0)
        P.append((f"底盘边梁{tag}", rail, "anod"))
    for sx, tag in ((1, "前"), (-1, "后")):
        P.append((f"底盘横管{tag}", cyl_y(sx * (L2 + 10), 10, 16, -W2, W2), "anod"))
        P.append((f"外壳·横管端盖{tag}左", cyl_y(sx * (L2 + 10), 10, 12, W2 + rw, W2 + rw + 1.5), "silver"))
        P.append((f"外壳·横管端盖{tag}右", cyl_y(sx * (L2 + 10), 10, 12, -W2 - rw - 1.5, -W2 - rw), "silver"))
    # 髋支座：独立的铝件，下面两颗螺栓拧在边梁上、上面两颗拧在侧板上；轴承座从中间的孔穿出来
    for hx in (-HIP_X, HIP_X):
        for sy, tag in ((1, "左"), (-1, "右")):
            y0, y1 = sorted((sy * (W2 + rw), sy * (W2 + rw + 7)))
            sup = prism_xz(chamfer_rect(hx - 40, hx + 40, 12, 66, 8), y0, y1).cut(cyl_y(hx, HIP_Z, 47, y0 - 1, y1 + 1))
            name = f"髋支座{'前' if hx > 0 else '后'}{tag}"
            P.append((name, soft(sup, 1.0), "anod"))
            for bx, bz in ((hx - 32, 21), (hx + 32, 21), (hx - 32, 57), (hx + 32, 57)):
                yb = sorted((sy * (W2 + rw + 7), sy * (W2 + rw + 8.5)))
                P.append((f"外壳·{name}螺栓 x{bx:+.0f}z{bz}", cyl_y(bx, bz, BOLT_D, *yb), "silver"))
    # ---- 下层外壳：架在边梁上面（下沿 = 边梁上沿 + 缝），一整圈，四角 R16
    z0 = rz1 + FRAME_GAP
    tub = rrect_ring(L2 + 2, W2 + TUB_OUT, 16, z0, TUB_TOP)
    for hx in (-HIP_X, HIP_X):
        tub = tub.cut(cyl_y(hx, HIP_Z, 70, -W2 - 8, W2 + 8))                    # 髋支座的位置让开
    dx, dy, dz = DEPTH_CAM
    tub = tub.cut(box(L2 - 1, L2 + 8, -dy / 2 - 1, dy / 2 + 1, 23, 24 + dz + 2))   # 深度相机窗
    tub = tub.cut(cq.Solid.makeCylinder(13, 10, V(-L2 - 8, 0, 70), V(1, 0, 0)))    # 急停
    tub = tub.cut(cq.Solid.makeCylinder(9, 10, V(-L2 - 8, 55, 70), V(1, 0, 0)))    # 总开关
    tub = tub.cut(box(-L2 - 8, -L2 + 1, -71, -53, 59, 69))                         # 充电口
    for sy in (1, -1):                                                            # 侧挂板的位置：外壳开窗，挂板四周留 1 mm 缝
        tub = tub.cut(box(-100 - SEAM, 100 + SEAM, *sorted((sy * (W2 - 1), sy * (W2 + 8))), z0 + 3 - SEAM, TUB_TOP - 3 + SEAM))
    for x in (L2 - 30, -(L2 - 30)):                                               # 拆成前、后、左、右几块面板，中间 2 mm 缝
        tub = tub.cut(box(x - SEAM / 2, x + SEAM / 2, -W2 - 20, W2 + 20, z0 - 1, TUB_TOP + 1))
    P.append(("外壳·下层外壳", soft(tub, 1.5), "shell"))
    # ---- 黑腰：凹进去的一圈 + 两侧琥珀灯、前后青灯
    P.append(("外壳·黑腰", rrect_ring(L2 + WAIST_IN, W2 + WAIST_IN, 2, TUB_TOP, WAIST_TOP), "dark"))
    for sy in (1, -1):
        for x in (-130, 130):
            P.append((f"外壳·腰线灯 y{sy} x{x}", box(x - 35, x + 35, *sorted((sy * (W2 + WAIST_IN), sy * (W2 + WAIST_IN + 0.6))), 73.5, 76.5), "amber"))
    for sx in (1, -1):
        P.append((f"外壳·腰线青灯 x{sx}", box(*sorted((sx * (L2 + WAIST_IN), sx * (L2 + WAIST_IN + 0.6))), -60, 60, 73.5, 76.5), "cyan"))
    # ---- 上层甲板：薄皮（侧面只凸 1.5，上半截离折叠的轮子只有 5 mm）+ 前后大斜面
    # ---- 上层甲板：整块带前后两道斜面的外壳（侧面看就是效果图那种往回收的装甲板）；侧面只有 1.5 mm 皮
    prof = [(-L2 + 14, WAIST_TOP), (L2 - 14, WAIST_TOP), (L2 + 16, WAIST_TOP + 20), (L2 + 16, BODY_H - 20), (L2 + 1, BODY_H + 0.5),
            (-L2 - 1, BODY_H + 0.5), (-L2 - 16, BODY_H - 22), (-L2 - 16, WAIST_TOP + 22)]
    wp = cq.Workplane("XY").polyline(prof).close().extrude(BODY_W + 2 * SLAB_OUT)       # 硬表面：转角不倒圆
    slab = wp.val().rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(0, W2 + SLAB_OUT, 0))
    slab = slab.cut(box(-L2, L2, -W2, W2, WAIST_TOP - 1, BODY_H + PLATE + 1))
    for x in (L2 - 30, -(L2 - 30)):                                               # 前鼻、后鼻、左右侧板分开，中间 2 mm 缝
        slab = slab.cut(box(x - SEAM / 2, x + SEAM / 2, -W2 - 20, W2 + 20, WAIST_TOP - 1, BODY_H + 2))
    P.append(("上层甲板壳", slab, "shell"))
    for sy, tag in ((1, "左"), (-1, "右")):
        out = []
        # 侧挂板：独立的一块，四角螺栓固定，和外壳之间一圈 1 mm 缝（拆下来就能看到里面的电池、驱动）
        z0 = FRAME_RAIL[2] + FRAME_GAP
        out.append((f"侧挂板{tag}", soft(prism_xz(chamfer_rect(-100, 100, z0 + 3, TUB_TOP - 3, 5), W2 + 0.5, W2 + 7.5), 1.0), "dark"))
        for x, z in ((-88, z0 + 9.5), (88, z0 + 9.5), (-88, TUB_TOP - 9.5), (88, TUB_TOP - 9.5)):
            out.append((f"外壳·{tag}挂板螺栓 x{x}z{z:.1f}", cyl_y(x, z, BOLT_D, W2 + 7.5, W2 + 9.5), "silver"))
        out.append((f"外壳·{tag}甲板边线", box(-170, 170, W2 + SLAB_OUT, W2 + SLAB_OUT + 0.3, BODY_H - 6, BODY_H - 4), "silver"))
        for n, s_, c in out:
            P.append((n, s_ if sy > 0 else mirror_y(s_), c))
        txt = cq.Workplane("XZ").text("007V", 34, 0.6, halign="center", valign="center").val()
        small = cq.Workplane("XZ").text("CARGO 10KG  ·  MAX STEP 0.10M", 6, 0.4, halign="left", valign="center").val()
        if sy > 0:
            txt = txt.translate(V(-40, 0, 0)).rotate(V(0, 0, 0), V(0, 0, 1), 180).translate(V(0, W2 + SLAB_OUT, 118))
            small = small.rotate(V(0, 0, 0), V(0, 0, 1), 180).translate(V(5, W2 + SLAB_OUT, 122))
        else:
            txt = txt.translate(V(-40, -W2 - SLAB_OUT, 118))
            small = small.translate(V(-5, -W2 - SLAB_OUT, 122))
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
        P.append((f"外壳·尾灯{'左' if sy > 0 else '右'}", box(-L2 - 16.6, -L2 - 15.9, *sorted((sy * 40, sy * 100)), 114, 122), "amber"))
    # 底光（青色，夜里照地面）
    for sy in (1, -1):
        P.append((f"外壳·底光{'左' if sy > 0 else '右'}", box(-140, 140, *sorted((sy * 30, sy * 36)), -1.5, 0), "cyan"))
    # 栏板顶边的琥珀反光条
    rail_top = deck + TOP_PLATE_T + PROFILE
    ftop = rail_top - FENCE_INSERT + FENCE_H
    for sy in (1, -1):
        P.append((f"外壳·栏板反光条{'左' if sy > 0 else '右'}", box(BED_X0 + 15, BED_X1 - 15, sy * RAIL_Y - 3.5, sy * RAIL_Y + 3.5, ftop - 6, ftop + 0.5), "amber"))
    return P


# ================================================================ 屏幕模块（俯仰轴坐标：原点在俯仰轴上）
SEAM_HEAD = 1.5                     # 头部面板分缝


def skin_groove(profile, hw, cutter, depth=1.2):
    """在外表面刻一道略凹的分缝 / 槽：只切外面 depth 深的一层皮。"""
    skin = prism_xz(profile, -hw, hw).cut(prism_xz(profile, -hw + depth, hw - depth, offset=-depth))
    return skin.intersect(cutter)


def head():
    """屏幕模块：夹在两只耳朵中间上下俯仰。电机耳那边（+Y）舵盘法兰插进舵机，方窗耳那边（-Y）钢轴销插进铜套。
    外壳分前框、顶盖、侧板（1.5 mm 略凹分缝），四角沉头螺栓；侧面散热槽 + 扬声器格栅；屏幕下一排麦克风孔；侧面小字。"""
    hw = HEAD_W / 2
    wall = 2.5
    front = max(x for x, _ in HEAD_PROFILE)
    gw, gh = SCREEN_GLASS
    t, w, h = SCREEN_BOARD
    shell = prism_xz(HEAD_PROFILE, -hw, hw).cut(prism_xz(HEAD_PROFILE, -hw + wall, hw - wall, offset=-wall))
    shell = shell.cut(box(front - wall - 1, front + 1, -gw / 2, gw / 2, SCREEN_Z - gh / 2, SCREEN_Z + gh / 2))
    shell = shell.cut(cyl_y(0, 0, 10, hw - wall - 1, hw + 1))
    shell = shell.cut(cyl_y(0, 0, 8.2, -hw - 1, -hw + wall + 1))
    c0, c1 = (56, -42), (84, -24)
    L = math.hypot(c1[0] - c0[0], c1[1] - c0[1])
    n = ((c1[1] - c0[1]) / L, -(c1[0] - c0[0]) / L)
    mid = ((c0[0] + c1[0]) / 2, (c0[1] + c1[1]) / 2)
    sensors = [("相机镜头", 0.0, 12.0, "black"), ("测距窗 VL53L5CX", -30.0, 8.0, "gloss"), ("隐私指示灯", 30.0, 4.0, "amber")]
    for _, y, d, _ in sensors:
        shell = shell.cut(cyl_dir((mid[0] - n[0] * 4, y, mid[1] - n[1] * 4), (n[0], 0, n[1]), d + 1, 8))
    s2 = SEAM_HEAD / 2
    grooves = [box(front - 14 - s2, front - 14 + s2, -hw - 1, hw + 1, -60, 80),        # 前框
               box(-40, front - 14, -hw - 1, hw + 1, 44 - s2, 44 + s2)]               # 顶盖 / 侧板
    for i in range(4):                                                                 # 侧面 4 条横向散热槽
        z = 6 + i * 7
        grooves.append(box(-22, 6, -hw - 1, hw + 1, z - 1, z + 1))
    for g in grooves:
        shell = shell.cut(skin_groove(HEAD_PROFILE, hw, g))
    for iy in range(9):                                                                # 屏幕下一排麦克风孔
        shell = shell.cut(cyl_dir((front + 1, -32 + iy * 8, -17), (-1, 0, 0), 2.4, 3))
    for sy in (1, -1):                                                                 # 两侧扬声器格栅 4 × 4
        for ix in range(4):
            for iz in range(4):
                shell = shell.cut(cyl_y(32 + ix * 6, -30 + iz * 6, 2.6, *sorted((sy * (hw - 1.5), sy * (hw + 1)))))
    P = [("屏幕头", soft(shell, 2.0), "deck")]
    board_x0 = SCREEN_X - t
    P.append(("4.3寸屏板", box(board_x0, SCREEN_X, -w / 2, w / 2, SCREEN_Z - h / 2, SCREEN_Z + h / 2), "pcb"))
    P.append(("屏幕玻璃", box(front - 1.5, front + 0.5, -gw / 2, gw / 2, SCREEN_Z - gh / 2, SCREEN_Z + gh / 2), "glass"))
    for sy in (1, -1):
        g = box(board_x0 - 4, SCREEN_X, w / 2 + 0.3, hw - wall, SCREEN_Z - h / 2, SCREEN_Z + h / 2)
        g = g.cut(box(board_x0 - 0.3, SCREEN_X + 0.3, w / 2 - 3, w / 2 + 0.6, SCREEN_Z - h / 2 - 0.3, SCREEN_Z + h / 2 + 0.3))
        P.append((f"屏板卡槽{'左' if sy > 0 else '右'}", g if sy > 0 else mirror_y(g), "printed"))
    P.append(("屏板压条", box(board_x0 - 4, board_x0, -w / 2 + 6, w / 2 - 6, SCREEN_Z - 6, SCREEN_Z + 6), "printed"))
    yc = POD_IN + POD_W / 2
    P.append(("舵盘法兰", cyl_y(0, 0, 9.5, hw - wall - 2, yc - SERVO[1] / 2 - 0.5), "silver"))
    P.append(("俯仰轴销", cyl_y(0, 0, 8.0, -POD_IN - 12, -hw + wall + 2), "steel"))
    frame = box(front, front + 2, -gw / 2 - 6, gw / 2 + 6, SCREEN_Z - gh / 2 - 6, SCREEN_Z + gh / 2 + 6).cut(
        box(front - 1, front + 3, -gw / 2, gw / 2, SCREEN_Z - gh / 2, SCREEN_Z + gh / 2))
    P.append(("外壳·屏框", frame, "dark"))
    for name, y, d, col in sensors:
        P.append((f"外壳·{name}", cyl_dir((mid[0] - n[0] * 3, y, mid[1] - n[1] * 3), (n[0], 0, n[1]), d, 4.5), col))
    for y, z in ((-hw + 8, 47), (hw - 8, 47), (-hw + 8, -15), (hw - 8, -15)):          # 前框四角
        P.append((f"外壳·前框螺栓 y{y:+.0f}z{z}", cyl_dir((front, y, z), (1, 0, 0), BOLT_D, 0.6), "dark"))
    for sy, tag in ((1, "左"), (-1, "右")):                                              # 侧板四角
        for x, z in ((-21, 35), (62, 35), (-17, -31), (50, -35)):
            P.append((f"外壳·{tag}侧板螺栓 x{x}z{z}", cyl_y(x, z, BOLT_D, *sorted((sy * hw, sy * (hw + 0.6)))), "dark"))
    txt = cq.Workplane("XZ").text("VISION UNIT · 4.3 HUD", 5, 0.4, halign="left", valign="center").val()
    P.append(("外壳·屏幕模块小字", txt.translate(V(-20, -hw - 0.4, 51)), "dark"))
    return P


def cable():
    """电机耳到屏幕模块的黑色编织护套线缆（俯仰轴坐标，跟屏幕模块一起动；耳朵里那一头离转轴近，转头时只挪几毫米）。"""
    hw = HEAD_W / 2
    pts = [(-31, hw - 14, 22), (-46, hw - 4, 30), (-46, hw + 6, 18), (-26, POD_IN + 8, 4), (-14, POD_IN + 12, 2)]
    out = None
    for (p0, p1) in zip(pts, pts[1:]):
        d = V(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
        seg = cq.Solid.makeCylinder(4, d.Length, V(*p0), d.normalized())
        out = seg if out is None else out.fuse(seg)
        out = out.fuse(cq.Solid.makeSphere(4, V(*p1)))
    return out


# ================================================================ 两只耳朵 + 脖子底座（脖子坐标：原点在头部平台盖上表面、俯仰轴正下方）
def ears():
    """两只耳朵都固定在脖子底座上，屏幕模块夹在中间俯仰。
    +Y 电机耳（主动侧）：骨架里装舵机；侧面 4 条散热槽；顶上低矮法兰 + 雷达 + 六边形护环。
    -Y 方窗耳（随动侧）：骨架里装铜套；上前斜面一个内凹烟黑方窗（热成像 + 红外管在后面），四周窄边框。
    每只：三层板叠起来（外层比内层缩 4 mm，露出台阶和板厚），外板上切一块可拆侧盖（1.5 mm 分缝），四角沉头螺栓；
    内侧一个圆柱转轴壳（法兰一圈螺栓），电机耳的转轴壳外圈一道青色灯环。"""
    P = []
    hexp = POD_HEX
    y_in, y_out = POD_IN, POD_IN + POD_W
    yc = (y_in + y_out) / 2
    sx, sw, sh = SERVO
    hw = HEAD_W / 2
    up0, up1 = (78, 62), (50, 122)
    Ln = math.hypot(up1[0] - up0[0], up1[1] - up0[1])
    nrm = ((up1[1] - up0[1]) / Ln, -(up1[0] - up0[0]) / Ln)
    pt = lambda t: (up0[0] + (up1[0] - up0[0]) * t, up0[1] + (up1[1] - up0[1]) * t)  # noqa: E731
    for sy, tag in ((1, "左"), (-1, "右")):
        out = []
        (a0, a1, _), (b0, b1, kb), (c0, c1, kc) = EAR_LAYERS
        core = prism_xz(hexp, a0, a1)
        if sy > 0:
            core = core.cut(box(-sx / 2 - 0.5, sx / 2 + 0.5, yc - sw / 2 - 0.5, yc + sw / 2 + 0.5, PIVOT_UP - sh / 2 - 0.5, PIVOT_UP + sh / 2 + 0.5))
            core = core.cut(cyl_y(0, PIVOT_UP, 14, y_in - 1, yc))
        else:
            core = core.cut(cyl_y(0, PIVOT_UP, 10, y_in - 1, y_in + 20))
            core = core.cut(box(30, 62, y_in + 6, a1 - 4, 70, 104))                    # 热成像 / 红外小板仓
        out.append((f"{tag}耳骨架", soft(core, 2.0), "shell"))
        mid = prism_xz(hexp, b0, b1, offset=kb)
        if sy > 0:
            mid = mid.cut(box(-sx / 2 - 0.5, sx / 2 + 0.5, yc - sw / 2 - 0.5, yc + sw / 2 + 0.5, PIVOT_UP - sh / 2 - 0.5, PIVOT_UP + sh / 2 + 0.5))
        out.append((f"{tag}耳中板", soft(mid, 1.0), "shell"))
        outer = prism_xz(hexp, c0, c1, offset=kc)
        cover = box(-30, 24, c1 - 1.2, c1 + 1, 24, 52)                                 # 可拆侧盖：四边 1.5 mm 分缝
        ring = cover.cut(box(-30 + SEAM_HEAD, 24 - SEAM_HEAD, c1 - 2, c1 + 2, 24 + SEAM_HEAD, 52 - SEAM_HEAD))
        outer = outer.cut(ring)
        if sy > 0:
            for i in range(4):                                                          # 电机耳：4 条散热槽
                z = 80 + i * 7
                outer = outer.cut(box(-14, 28, c1 - 1.2, c1 + 1, z - 1, z + 1))
        out.append((f"{tag}耳外板", soft(outer, 1.0), "shell"))
        for x, z in ((-12, 30), (36, 30), (-2, 108), (34, 108)):                        # 外板四角沉头螺栓
            out.append((f"外壳·{tag}耳螺栓 x{x}z{z}", cyl_y(x, z, BOLT_D, c1, c1 + 0.6), "dark"))
        for x, z in ((-25, 29), (19, 29), (-25, 47), (19, 47)):                         # 侧盖四角小螺栓
            out.append((f"外壳·{tag}侧盖螺栓 x{x}z{z}", cyl_y(x, z, 4, c1, c1 + 0.5), "dark"))
        # 内侧转轴壳：露在屏幕模块和耳朵之间那条缝里
        out.append((f"{tag}转轴壳", cyl_y(0, PIVOT_UP, 30, hw + 1.0, y_in).cut(cyl_y(0, PIVOT_UP, 12, hw, y_in + 1)), "anod"))
        out.append((f"外壳·{tag}转轴法兰", cyl_y(0, PIVOT_UP, 44, y_in - 2.5, y_in).cut(cyl_y(0, PIVOT_UP, 12, y_in - 3, y_in + 1)), "anod"))
        for i in range(6):
            a = 2 * math.pi * i / 6
            out.append((f"外壳·{tag}转轴螺栓{i + 1}", cyl_y(18 * math.cos(a), PIVOT_UP + 18 * math.sin(a), 4, y_in - 3.5, y_in - 2.5), "dark"))
        if sy > 0:
            out.append(("外壳·转轴青光环", ring_y(0, PIVOT_UP, 34, 30, hw + 2.5, hw + 4.5), "cyan"))
        if sy < 0:                                                                      # 方窗：内凹烟黑窗 + 窄边框
            p = pt(0.42)
            k = 26 / Ln
            win = strip(pt(0.42 - k / 2), pt(0.42 + k / 2), 2.0, y_in + 7, a1 - 5, inward=(nrm[0] * 0.3, nrm[1] * 0.3))
            out.append(("外壳·方窗（热成像 / 红外）", win, "visor"))
            fr = strip(pt(0.42 - k / 2 - 3 / Ln), pt(0.42 + k / 2 + 3 / Ln), 2.4, y_in + 4, a1 - 2, inward=(nrm[0] * 1.2, nrm[1] * 1.2))
            fr = fr.cut(strip(pt(0.42 - k / 2), pt(0.42 + k / 2), 4, y_in + 7, a1 - 5, inward=(nrm[0] * 1.2, nrm[1] * 1.2)))
            out.append(("外壳·方窗边框", fr, "dark"))
        for n_, s_, c in out:
            P.append((n_, s_ if sy > 0 else mirror_y(s_), c))
    P.append(("俯仰舵机", box(-sx / 2, sx / 2, yc - sw / 2, yc + sw / 2, PIVOT_UP - sh / 2, PIVOT_UP + sh / 2), "motor"))
    P.append(("俯仰铜套", ring_y(0, PIVOT_UP, 10, 8.1, -y_in - 20, -hw - 1.5), "trim"))
    # 电机耳顶上：低矮法兰（15 mm，不超出耳朵）+ 4 颗小螺栓 + 雷达底座 + 六边形护环
    top = max(z for _, z in hexp)
    top_x = sum(x for x, z in hexp if z == top) / 2
    fz = top + LIDAR_FLANGE
    fl = prism_xz([(top_x - 28, top), (top_x + 28, top), (top_x + 25, fz), (top_x - 25, fz)], y_in, y_out)
    P.append(("雷达法兰", soft(fl, 1.5), "anod"))
    for x in (top_x - 20, top_x + 20):
        for yy in (y_in + 6, y_out - 6):
            P.append((f"外壳·法兰螺栓 x{x:.0f}y{yy:.0f}", cyl_z(x, yy, 5, fz, fz + 1), "dark"))
    L = LD19
    zr = fz + L["h"] - L["head_h"]
    hexring = lambda r: [(top_x + r * math.cos(k * math.pi / 3), yc + r * math.sin(k * math.pi / 3)) for k in range(6)]  # noqa: E731
    ring = cq.Workplane("XY").polyline(hexring(25)).close().extrude(3.8).val().cut(
        cq.Workplane("XY").polyline(hexring(21.5)).close().extrude(5).val())
    P.append(("雷达护环", ring.translate(V(0, 0, zr)), "anod"))
    for k, (dx, dy) in enumerate(((24.5, 0), (-24.5, 0), (0, 20.8), (0, -20.8))):
        P.append((f"外壳·护环支柱{k + 1}", cyl_z(top_x + dx, yc + dy, 2.4, fz + 2.5, zr + 0.5), "dark"))
    return P, (top_x, yc, fz)


def neck():
    """脖子底座：U 形（后面一根横梁 + 两条垫在耳朵下面），屏幕模块下面挖空；后沿让开货斗。"""
    nx, nh, ny = NECK_BASE
    x0, x1 = BED_X1 + 3 - NECK_X, nx / 2
    u = box(x0, x1, -ny / 2, ny / 2, 0, nh).cut(box(x0 + 26, x1 + 1, -POD_IN, POD_IN, -1, nh + 1))
    P = [("脖子底座", soft(u, 1.5), "anod")]
    for x in (x0 + 10, x1 - 10):
        for sy in (1, -1):
            P.append((f"外壳·底座螺栓 x{x:.0f}y{sy}", cyl_z(x, sy * (POD_IN + POD_W / 2), BOLT_D, nh, nh + 0.6), "dark"))
    e, lid = ears()
    return P + e, lid


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


def waist_xz(x0, x1, r0, r1, y0, y1, k=LINK_WAIST):
    """变截面连杆（侧面看）：两端是半径 r0 / r1 的圆（关节），中段收窄到两端平均高度的 k 倍。"""
    hm = (r0 + r1) / 2 * k
    xm = (x0 + x1) / 2
    poly = prism_xz([(x0, r0), (xm, hm), (x1, r1), (x1, -r1), (xm, -hm), (x0, -r0)], y0, y1)
    return poly.fuse(cyl_y(x0, 0, 2 * r0, y0, y1)).fuse(cyl_y(x1, 0, 2 * r1, y0, y1))


def w2a(X, Z):
    """站立姿势下“以轮轴为原点、X 朝前、Z 朝上”的点 → 左前腿的摆臂坐标（髋轴原点，轮轴在 (ARM, 0)）。"""
    c, s_ = math.cos(math.radians(STANCE_DEG)), math.sin(math.radians(STANCE_DEG))
    return (ARM + X * c - Z * s_, X * s_ + Z * c)


def wprism(pts, y0, y1):
    return prism_xz([w2a(*p) for p in pts], y0, y1)


def arc_pts(r, a0, a1, n=16):
    return [(r * math.cos(math.radians(a0 + (a1 - a0) * i / n)), r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def fork_profile(k=0.0):
    """叉板侧面（站立时的方向，轮轴为原点）：轮轴处一个半径 FORK_LOW 的圆（最低点就在轴下 15），往上收成一根臂接到挡泥横梁；k = 往里缩多少。"""
    r = FORK_LOW - k
    poly = [(-FORK_LOW + k, 0), (FORK_LOW - k, 0), (24 - k, FENDER_R[1] - k), (-24 + k, FENDER_R[1] - k)]
    return poly, r


def fork_plate(y0, y1, k=0.0):
    poly, r = fork_profile(k)
    return wprism(poly, y0, y1).fuse(cyl_y(ARM, 0, 2 * r, y0, y1))              # 实心（用户 2026-10-18：不要镂空）


def leg_skin(oy, ly0, ly1, fy0, hub0):
    """腿和叉的外观件（左前腿坐标）。oy = 腿板外表面；ly0 / ly1 = 电机两端；fy0 = 叉板内侧；hub0 = 臂座外壳内侧。"""
    P = []
    # 臂座上的黄色箍（截图那版的样子，跟着臂座收窄）+ 腿板外面髋边上三道斜黄条
    for i, y in enumerate((hub0 + 5, hub0 + 11, hub0 + 17)):
        hoop = waist_xz(0, ARM, ARM_ROOT_R + 1.2, ARM_END_R + 1.2, y, y + 2.5).cut(waist_xz(0, ARM, ARM_ROOT_R - 0.5, ARM_END_R - 0.5, y - 1, y + 4))
        P.append((f"外壳·臂座黄箍{i + 1}", hoop.intersect(box(-40, ARM_BOX_LEN, y - 1, y + 4, -40, 40)), "amber"))
    for i, u in enumerate((34, 41, 48)):
        P.append((f"外壳·腿板黄条{i + 1}", strip((u, -14), (u + 10, 14), 2.4, oy, oy + 0.6), "amber"))
    f1 = fy0 + sum(FORK_PLATE_T)
    # 挡泥横梁：轮子上方一段圆弧，把腿板和叉板连成 U 形叉；顶上一道凹槽线，两头各一道 1.5 mm 分缝
    r0, r1 = FENDER_R
    band = arc_pts(r1, 35, 145) + arc_pts(r0, 145, 35)
    fender = wprism(band, oy, fy0)
    groove = wprism([(r1 * math.cos(math.radians(a)), r1 * math.sin(math.radians(a))) for a in (89.3, 90.7)] +
                    [((r1 - 1) * math.cos(math.radians(a)), (r1 - 1) * math.sin(math.radians(a))) for a in (90.7, 89.3)], oy - 1, fy0 + 4)
    fender = fender.cut(groove)
    for yy in (oy + 3, fy0 - 3):
        fender = fender.cut(wprism(arc_pts(r1 + 1, 30, 150) + arc_pts(r1 - 1, 150, 30), yy - 0.75, yy + 0.75))
    P.append(("挡泥横梁", soft(fender, 1.0), "deck"))
    # 叉板两层：内层深、外层浅（缩进 2.5，露出台阶）
    P.append(("叉板内层", soft(fork_plate(fy0, fy0 + FORK_PLATE_T[0]), 0.8), "anod"))
    P.append(("叉板外层", soft(fork_plate(fy0 + FORK_PLATE_T[0], f1, PLATE_INSET), 0.8), "deck"))
    # 叉板端盖：盖住轮轴外端，一圈 6 颗螺栓 + 青色光环（半径不超过 15，不低于轴下 15）
    P.append(("叉板端盖", soft(cyl_y(ARM, 0, 26, f1, f1 + 4), 0.8), "anod"))
    for i in range(6):
        a = math.pi / 6 + i * math.pi / 3
        P.append((f"外壳·端盖钉{i + 1}", cyl_y(ARM + 9 * math.cos(a), 9 * math.sin(a), 3, f1 + 4, f1 + 4.6), "dark"))
    P.append(("外壳·端盖青光环", ring_y(ARM, 0, 29, 26, f1 + 0.5, f1 + 2.0), "cyan"))
    P.append(("外壳·端盖六角", prism_xz([(ARM + 4 * math.cos(math.pi / 6 + k * math.pi / 3), 4 * math.sin(math.pi / 6 + k * math.pi / 3)) for k in range(6)],
                                         f1 + 4, f1 + 5.2), "dark"))
    # 叉板顶上两颗沉头螺栓（和横梁拧在一起）
    for X in (-15, 15):
        u, v = w2a(X, FENDER_R[1] - 9)
        P.append((f"外壳·叉板螺栓{X:+d}", cyl_y(u, v, BOLT_D, f1, f1 + 0.6), "dark"))
    # 金线：叉板一条（从横梁走到端盖），腿板一条（从髋关节走到轴心）
    P.append(("外壳·叉板金线", strip(w2a(15, 58), w2a(12, 17), 1.6, f1, f1 + 0.6), "amber"))
    P.append(("外壳·腿板金线", strip((62, 16), (ARM - 30, 17.5), 1.6, oy, oy + 0.6), "amber"))
    # 电机护罩（“靴子”）：从腿末端长出来——浅色臂座在轮子前收成一道 R59 的弧，深色护罩从 R58 接着往下包住电机（1 mm 分缝）；
    # 切角八边形，背面封住拧在腿板上，朝机身那头敞开露出电机端面；底面 4 道散热槽，两侧各两颗沉头螺栓；后上方出线口，线沿臂座上沿走到髋
    def oct_w(h, ch):
        return [(-h + ch, -h), (h - ch, -h), (h, -h + ch), (h, h - ch), (h - ch, h), (-h + ch, h), (-h, h - ch), (-h, -h + ch)]
    tail = [(ARM - 75, 24), (ARM, 31), (ARM, -31), (ARM - 75, -22)]
    gy0, gy1 = ly0 + 2, ly1

    def boot(k, y0, y1):
        body = wprism(oct_w(31 - k, 10 - k * 0.4), y0, y1)
        pts = [(x + (k if x < ARM else 0), v - math.copysign(k, v)) for x, v in tail]
        t = prism_xz(pts, y0, y1).intersect(cyl_y(ARM, 0, 2 * (58 - k), y0 - 1, y1 + 1))
        return body.fuse(t)
    guard = boot(0, gy0, gy1).cut(boot(3, gy0 - 1, gy1 - 3))                       # 3 mm 壳，靠腿板那面封住
    guard = guard.cut(cyl_y(ARM, 0, WHEEL_MOTOR_D + 1, gy1 - 4, gy1 + 1))           # 封住那面中间开孔，电机从这里拧到腿板上
    for X in (-12, -4, 4, 12):                                                      # 底面散热槽
        guard = guard.cut(wprism([(X - 1, -31.5), (X + 1, -31.5), (X + 1, -30), (X - 1, -30)], gy0 + 4, gy1 - 5))
    P.append(("电机护罩", soft(guard, 0.8), "anod"))
    c55, s55 = math.cos(math.radians(STANCE_DEG)), math.sin(math.radians(STANCE_DEG))
    for X in (-31, 31):
        for yb in (gy0 + 9, gy1 - 9):
            u, v = w2a(X, -14)
            sg = math.copysign(1, X)
            P.append((f"外壳·护罩螺栓{X:+d}y{yb:.0f}", cyl_dir((u, yb, v), (sg * c55, 0, sg * s55), BOLT_D, 0.6), "dark"))
    P.append(("外壳·电机端面青光环", ring_y(ARM, 0, 40, 36, ly0 - 0.6, ly0), "cyan"))
    # 出线口（线缆接头）+ 电机线缆：沿臂座上沿走到髋，三个小线夹
    yy = oy - sum(LEG_PLATE_T) - 5
    P.append(("外壳·出线口", cyl_dir((ARM - 46, yy, 25), (0, 0, 1), 10, 5), "dark"))
    pts = [(ARM - 46, 30), (ARM - 62, 30), (95, 27), (45, 29)]
    cab = None
    for (p0, p1) in zip(pts, pts[1:]):
        d = V(p1[0] - p0[0], 0, p1[1] - p0[1])
        seg = cq.Solid.makeCylinder(3.5, d.Length, V(p0[0], yy, p0[1]), d.normalized())
        cab = seg if cab is None else cab.fuse(seg)
        cab = cab.fuse(cq.Solid.makeSphere(3.5, V(p1[0], yy, p1[1])))
    P.append(("外壳·电机线缆", cab, "black"))
    for i, (u, v) in enumerate(((ARM - 62, 30), (95, 27), (60, 28.5))):
        P.append((f"外壳·线夹{i + 1}", box(u - 3, u + 3, yy - 4.5, yy + 4.5, v - 4.5, v + 4.5), "dark"))
    # 髋端编码器罩（AS5600）
    P.append(("编码器罩", soft(cyl_y(0, 0, 44, oy, oy + 4), 1.0), "anod"))
    for i in range(4):
        a = math.pi / 4 + i * math.pi / 2
        P.append((f"外壳·编码器罩钉{i + 1}", cyl_y(15 * math.cos(a), 15 * math.sin(a), 4, oy + 4, oy + 4.8), "dark"))
    P.append(("外壳·编码器罩六角", prism_xz([(6 * math.cos(math.pi / 6 + k * math.pi / 3), 6 * math.sin(math.pi / 6 + k * math.pi / 3)) for k in range(6)],
                                         oy + 4, oy + 5.5), "dark"))
    # 腿板沉头螺栓：髋那头两颗、靠轮那头两颗
    for u, v in ((24, 17), (24, -17), (ARM - 100, 17), (ARM - 100, -17)):
        P.append((f"外壳·腿板螺栓 u{u}v{v}", cyl_y(u, v, BOLT_D, oy, oy + 0.6), "dark"))
    # 台阶测距（站立时朝前下方 30°）
    a = math.radians(25)
    u_, n_ = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
    c = (ARM - 80, 13)
    corner = lambda su, sn: (c[0] + u_[0] * 9 * su + n_[0] * 6 * sn, c[1] + u_[1] * 9 * su + n_[1] * 6 * sn)  # noqa: E731
    P.append(("台阶测距", soft(prism_xz([corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)], oy, oy + 8), 0.8), "anod"))
    f0_ = (c[0] + u_[0] * 9 - n_[0] * 4, c[1] + u_[1] * 9 - n_[1] * 4)
    f1_ = (c[0] + u_[0] * 9 + n_[0] * 4, c[1] + u_[1] * 9 + n_[1] * 4)
    P.append(("外壳·台阶测距窗", strip(f0_, f1_, 1.2, oy + 1.5, oy + 6.5, inward=(u_[0] * 0.5, u_[1] * 0.5)), "visor"))
    return P


def leg_label(key, fx, sy):
    """叉板外侧的小刻字（每条腿单独做，不跟着镜像，免得变成反字）：编号 · 轮径 · 麦轮旋向（FL/RR 用 A 型，FR/RL 用 B 型）。返回该腿局部坐标里的零件。"""
    f1 = FORK_Y0 + sum(FORK_PLATE_T)
    kind = "A" if key in ("FL", "RR") else "B"
    t = cq.Workplane("XZ").text(f"{key} · Φ110 · {kind}", 3.2, 0.4, halign="center", valign="center").val()
    if sy > 0:
        t = t.rotate(V(0, 0, 0), V(0, 0, 1), 180)
    t = t.rotate(V(0, 0, 0), V(0, 1, 0), -STANCE_DEG if fx > 0 else STANCE_DEG)
    c, s_ = math.cos(math.radians(STANCE_DEG)), math.sin(math.radians(STANCE_DEG))
    X, Z = -4.0, 33.0                                                             # 叉板中段，金线左边
    u, v = (ARM + X * c - Z * s_, X * s_ + Z * c)
    if fx < 0:                                                                    # 后腿是前腿前后镜像过去的
        u = -u
    return [(f"外壳·叉板刻字{key}", t.translate(V(u, sy * (f1 + 0.4), v)), "dark")]
