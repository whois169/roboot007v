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
                yb = sorted((sy * (W2 + rw + 7), sy * (W2 + rw + 10)))
                P.append((f"外壳·{name}螺栓 x{bx:+.0f}z{bz}", cyl_y(bx, bz, 12, *yb), "silver"))
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
            out.append((f"外壳·{tag}挂板螺栓 x{x}z{z:.1f}", cyl_y(x, z, 10, W2 + 7.5, W2 + 9.5), "silver"))
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


# ================================================================ 传感器头（俯仰轴坐标：原点在俯仰轴上，轴在头的底部）
def head():
    """紧凑的传感器头：空心壳 + 前脸一圈环绕的烟黑面罩（屏幕和传感器都在面罩后面）+ 两侧伸进耳轴座的短轴。"""
    hw = HEAD_W / 2
    wall = 2.5
    front = max(x for x, _ in HEAD_PROFILE)
    gw, gh = SCREEN_GLASS
    t, w, h = SCREEN_BOARD
    shell = prism_xz(HEAD_PROFILE, -hw, hw).cut(prism_xz(HEAD_PROFILE, -hw + wall, hw - wall, offset=-wall))
    shell = shell.cut(box(front - wall - 1, front + 1, -gw / 2, gw / 2, SCREEN_Z - gh / 2, SCREEN_Z + gh / 2))     # 屏幕窗
    shell = shell.cut(cyl_y(0, 0, 10, hw - wall - 1, hw + 1))                                                      # 左：舵盘法兰
    shell = shell.cut(cyl_y(0, 0, 8.2, -hw - 1, -hw + wall + 1))                                                   # 右：轴销
    P = [("屏幕头", soft(shell, 2.0), "deck")]
    board_x0 = SCREEN_X - t
    P.append(("4.3寸屏板", box(board_x0, SCREEN_X, -w / 2, w / 2, SCREEN_Z - h / 2, SCREEN_Z + h / 2), "pcb"))
    P.append(("屏幕玻璃", box(front - 1.5, front + 0.5, -gw / 2, gw / 2, SCREEN_Z - gh / 2, SCREEN_Z + gh / 2), "glass"))
    for sy in (1, -1):
        g = box(board_x0 - 4, SCREEN_X, w / 2 + 0.3, hw - wall, SCREEN_Z - h / 2, SCREEN_Z + h / 2)
        g = g.cut(box(board_x0 - 0.3, SCREEN_X + 0.3, w / 2 - 3, w / 2 + 0.6, SCREEN_Z - h / 2 - 0.3, SCREEN_Z + h / 2 + 0.3))
        P.append((f"屏板卡槽{'左' if sy > 0 else '右'}", g if sy > 0 else mirror_y(g), "printed"))
    P.append(("屏板压条", box(board_x0 - 4, board_x0, -w / 2 + 6, w / 2 - 6, SCREEN_Z - 6, SCREEN_Z + 6), "printed"))
    # 两侧短轴：左边舵盘法兰插进左耳轴座里的舵机，右边钢轴销插进右耳轴座的铜套
    P.append(("舵盘法兰", cyl_y(0, 0, 9.5, hw - wall - 2, (TRUNNION_Y0 + TRUNNION_Y1) / 2 - SERVO[1] / 2 - 0.5), "silver"))
    P.append(("俯仰轴销", cyl_y(0, 0, 8.0, -TRUNNION_Y0 - 16, -hw + wall + 2), "steel"))
    # 环绕面罩：前脸一整条 + 两侧各包回 30 mm，烟黑色，盖住屏幕窗和以后的传感器窗
    z0, z1 = 6.0, 76.0
    band = prism_xz(HEAD_PROFILE, -hw - 1.5, hw + 1.5, offset=1.5).cut(prism_xz(HEAD_PROFILE, -hw, hw))
    band = band.intersect(box(front - 38, front + 3, -hw - 3, hw + 3, z0, z1))
    P.append(("外壳·烟黑面罩", band, "visor"))
    return P


def neck():
    """固定颈座（脖子坐标：原点在头部平台盖上表面、俯仰轴正下方）：
    后横梁 + 两边立板，立板顶上是两个圆柱形耳轴座（左边装舵机、右边装铜套），端盖带螺栓——一看就知道头只能上下点；
    左耳轴座上立一根短桅杆，顶上是激光雷达（固定，不跟头动）。"""
    P = []
    d, y0, y1 = TRUNNION_D, TRUNNION_Y0, TRUNNION_Y1
    r = d / 2
    xa, xb = YOKE_X
    yc = (y0 + y1) / 2
    beam = box(-44, -26, -y1, y1, 0, 16)
    for sy in (1, -1):
        cheek = prism_xz([(xa, 0), (xb, 0), (xb, PIVOT_UP), (xa, PIVOT_UP)], *sorted((sy * (y0 + 2), sy * y1)))
        beam = beam.fuse(cheek)
    beam = beam.cut(cyl_y(0, PIVOT_UP, d + 0.6, -y1 - 1, y1 + 1))            # 立板顶上挖出圆弧，耳轴座坐进去
    P.append(("颈座", soft(beam, 2.0), "anod"))
    sx, sw, sh = SERVO
    for sy, tag in ((1, "左"), (-1, "右")):
        out = []
        tr = cyl_y(0, PIVOT_UP, d, y0, y1)
        if sy > 0:   # 左：舵机仓 + 舵盘过孔
            tr = tr.cut(box(-sx / 2 - 0.5, sx / 2 + 0.5, yc - sw / 2 - 0.5, yc + sw / 2 + 0.5, PIVOT_UP - sh / 2 - 0.5, PIVOT_UP + sh / 2 + 0.5))
            tr = tr.cut(cyl_y(0, PIVOT_UP, 14, y0 - 1, yc))
        else:        # 右：铜套孔
            tr = tr.cut(cyl_y(0, PIVOT_UP, 10, y0 - 1, y0 + 20))
        out.append((f"{tag}耳轴座", soft(tr, 2.0), "anod"))
        out.append((f"{tag}耳轴端盖", cyl_y(0, PIVOT_UP, d - 18, y1, y1 + 4), "anod"))
        out.append((f"外壳·{tag}端盖六角", prism_xz([(9 * math.cos(math.pi / 6 + k * math.pi / 3), PIVOT_UP + 9 * math.sin(math.pi / 6 + k * math.pi / 3))
                                                   for k in range(6)], y1 + 4, y1 + 8), "silver"))
        for i in range(6):
            a = 2 * math.pi * i / 6 + math.pi / 6
            out.append((f"外壳·{tag}耳轴螺栓{i + 1}", cyl_y((r - 4) * math.cos(a), PIVOT_UP + (r - 4) * math.sin(a), 6, y1, y1 + 2.5), "silver"))
        for n, s_, c in out:
            P.append((n, s_ if sy > 0 else mirror_y(s_), c))
    P.append(("俯仰舵机", box(-sx / 2, sx / 2, yc - sw / 2, yc + sw / 2, PIVOT_UP - sh / 2, PIVOT_UP + sh / 2), "motor"))
    P.append(("俯仰铜套", ring_y(0, PIVOT_UP, 10, 8.1, -y0 - 20, -y0 + 1), "trim"))
    # 雷达桅杆：从左耳轴座顶上立起来，顶上一块安装板
    top = PIVOT_UP + LIDAR_UP
    my = min(LIDAR_Y, y1 - 9)
    P.append(("雷达桅杆", cyl_z(0, my, 18, PIVOT_UP + r + 0.2, top - 4), "anod"))
    P.append(("雷达底座", box(-29, 29, LIDAR_Y - LD19["ear_d"] / 2, LIDAR_Y + LD19["ear_d"] / 2, top - 4, top), "trim"))
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
    cover = prism_xz(stadium_pts(0, ARM, ARM_ROOT_R - 1, ARM_END_R - 1), oy1, oy1 + 2)
    cover = cover.cut(prism_xz(stadium_pts(48, 120, 5, 5), oy1 - 1, oy1 + 3))
    P.append(("摆臂外盖板", soft(cover, 1.2), "dark"))
    P.append(("外壳·长槽底", prism_xz(stadium_pts(48, 120, 5, 5), oy1, oy1 + 0.8), "black"))
    for i, u in enumerate((34, 41, 48)):                                   # 髋边上三道斜黄条
        P.append((f"外壳·摆臂黄条{i + 1}", strip((u, -14), (u + 10, 14), 2.4, oy1 + 2, oy1 + 2.6), "amber"))
    P.append(("外壳·摆臂边黄线", strip((60, 16), (135, 19), 1.6, oy1 + 2, oy1 + 2.6), "amber"))
    P.append(("髋部大圆盖", soft(cyl_y(0, 0, 46, oy1 + 2, oy1 + 4.5), 1.2), "dark"))
    P.append(("外壳·髋盖黄圈", ring_y(0, 0, 46, 37, oy1 + 4.5, oy1 + 5.1), "amber"))
    P.append(("外壳·髋盖六角", prism_xz([(9 * math.cos(math.pi / 6 + k * math.pi / 3), 9 * math.sin(math.pi / 6 + k * math.pi / 3)) for k in range(6)], oy1 + 4.5, oy1 + 6), "silver"))
    P.append(("外壳·轮端青光圈", ring_y(ARM, 0, 24, 18, oy1 + 2, oy1 + 2.6), "cyan"))
    for i, y in enumerate((root1 + 8, root1 + 16, root1 + 24, root1 + 32)):    # 臂座上的黄色箍（像减震弹簧）
        P.append((f"外壳·臂座黄箍{i + 1}", prism_xz(stadium_pts(0, ARM_BOX_LEN - ARM_ROOT_R, ARM_ROOT_R + 1.2, ARM_ROOT_R + 1.2), y, y + 2.5)
                  .cut(prism_xz(stadium_pts(0, ARM_BOX_LEN - ARM_ROOT_R, ARM_ROOT_R - 0.5, ARM_ROOT_R - 0.5), y - 1, y + 4)), "amber"))
    return P
