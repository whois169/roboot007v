"""
RoboDog v0.6 腿：同步带膝关节，大腿全封闭（腿坐标，零位 = 大腿、小腿都竖直向下）
  腿坐标：原点 = HFE 轴线与 HFE 输出面交点，x 向前，y 向外，z 向上
  膝坐标：原点 = 膝轴与 y=0 平面交点（零位时位于腿坐标 (0, 0, -L_THIGH)）
传动：膝电机(KFE) -> 转接盘 -> 20T 小带轮 -> HTD-5M 皮带 -> 36T 大带轮(固定在小腿上)
"""
import math
import cadquery as cq
from config import *
from parts import (V, box, cyl_x, cyl_y, cyl_z, xz_prism, tapered_slot_xz, stadium_y, offset_poly,
                   bolt_circle, chamfer_edges, motor_y, motor_connector_y, dowel_holes_y)

PD_S = PULLEY_T_SMALL * BELT_PITCH / math.pi      # 小轮节圆直径 31.83
PD_B = PULLEY_T_BIG * BELT_PITCH / math.pi        # 大轮节圆直径 57.30
R_S, R_B = PD_S / 2, PD_B / 2
OD_GAP = 0.57                                      # HTD-5M 节线到齿顶的距离（单边）
BELT_IN, BELT_OUT = -1.14, BELT_THICK - 1.14       # 皮带相对节线的内外表面
YB0, YB1 = Y_PULLEY[0] + 1.2, Y_PULLEY[1] - 1.2    # 齿面宽 20.6mm
KNEE_Z = -L_THIGH

# 大腿内外板之间的立柱（零位 x,z）：髋部 3 根在皮带外圈后上方；中段 2 根在皮带环内侧中线上
_RS = 35.0
STANDOFF_POS = [(-_RS * math.sin(math.radians(a)), -_RS * math.cos(math.radians(a))) for a in (90.0, 135.0, 180.0)]
STANDOFF_POS += [(0.0, -78.0), (0.0, -138.0)]
N_HIP_STANDOFF = 3                                # 前 3 根在髋部，同时托住膝电机前安装环
KFE_FRONT = Y_KFE[1] - MOTOR["flange_h"]          # 膝电机外壳前端面 y=53
Y_KFE_RING = (KFE_FRONT, KFE_FRONT + 4.0)         # 膝电机前安装环
KFE_CONN_ANG = -90.0                              # 膝电机插头朝向：沿大腿向下（大腿板之间，不碰立柱）


BELT_TEETH = 104                                   # 买的环形带：HTD-5M 104 齿，节线长 520mm
BELT_LEN = BELT_TEETH * BELT_PITCH
IDLER_R = 9.0
IDLER_Z = -70.0                                    # 张紧轮压在后侧直段 z=-70 处
IDLER_ADJ = 6.0                                    # 张紧轮长槽 ±6mm（v0.4 是 ±4）


def _span_x(z, off):
    """后侧（x<0）皮带直段在高度 z 处的 x，off = 相对节线的径向偏移（不压张紧轮时）"""
    t = -z / L_THIGH
    return -((R_S + off) + (R_B - R_S) * t)


def idler_center(press):
    """张紧轮中心：press = 把皮带从直线位置往里压进去多少 mm"""
    return (_span_x(IDLER_Z, BELT_OUT) - IDLER_R + press, IDLER_Z)


def _belt_loop(press, off=0.0, n_arc=28):
    """皮带走线（小轮 -> 张紧轮 -> 大轮，逆时针）。off = 相对节线的径向偏移。
    返回 (长度, 折线点)。张紧轮在皮带外侧（压皮带背面），所以按负半径处理。"""
    circles = [((0.0, 0.0), R_S + off), ((0.0, KNEE_Z), R_B + off)]
    if press is not None:
        circles.insert(1, (idler_center(press), -(IDLER_R + BELT_OUT) + off))
    n = len(circles)
    normals = []                                   # 每段直线的法向（从 i 到 i+1）
    for i in range(n):
        (c1, r1), (c2, r2) = circles[i], circles[(i + 1) % n]
        dx, dz = c2[0] - c1[0], c2[1] - c1[1]
        d = math.hypot(dx, dz)
        ux, uz = dx / d, dz / d
        px, pz = -uz, ux
        k = -(r2 - r1) / d
        h = math.sqrt(1 - k * k)
        normals.append((k * ux - h * px, k * uz - h * pz))
    length, pts = 0.0, []
    for i in range(n):
        (c, r) = circles[i]
        n_in, n_out = normals[i - 1], normals[i]
        a_in, a_out = math.atan2(n_in[1], n_in[0]), math.atan2(n_out[1], n_out[0])
        da = (a_out - a_in) % (2 * math.pi) if r > 0 else -((a_in - a_out) % (2 * math.pi))
        length += abs(r * da)
        for k in range(n_arc + 1):
            a = a_in + da * k / n_arc
            pts.append((c[0] + r * math.cos(a), c[1] + r * math.sin(a)))
        (c2, r2) = circles[(i + 1) % n]
        p1 = (c[0] + r * n_out[0], c[1] + r * n_out[1])
        p2 = (c2[0] + r2 * n_out[0], c2[1] + r2 * n_out[1])
        length += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    return length, pts


def belt_length(press=None):
    """节线长度；press=None 表示不装张紧轮的开口带"""
    return _belt_loop(press)[0]


def _solve_press(target):
    lo, hi = 0.0, 2 * IDLER_ADJ
    if belt_length(lo) > target or belt_length(hi) < target:
        return None
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if belt_length(mid) < target else (lo, mid)
    return (lo + hi) / 2


IDLER_PRESS = _solve_press(BELT_LEN)               # 名义位置：刚好把 520mm 皮带张紧（约 6mm）
if IDLER_PRESS is None:
    raise ValueError(f"104 齿皮带和张紧轮调节范围对不上：不压 {belt_length(0):.2f}mm，压满 {belt_length(2 * IDLER_ADJ):.2f}mm")
IDLER_PRESS = round(IDLER_PRESS, 1)
IDLER_POS = idler_center(IDLER_PRESS)
IDLER_RANGE = (belt_length(IDLER_PRESS - IDLER_ADJ), belt_length(IDLER_PRESS + IDLER_ADJ))   # 长槽两端能适应的带长


KFE_CABLE = (-35.0 * math.sin(math.radians(112.5)), -35.0 * math.cos(math.radians(112.5)))   # 膝电机出线孔（内板，两根髋部立柱之间）


def _thigh_plate(y0, y1):
    p = tapered_slot_xz(R_HIP_PLATE, R_KNEE_PLATE, L_THIGH, y0, y1)
    # v0.4：去掉减重孔，内外板都是实心的，从外面看不到皮带
    for (x, z) in STANDOFF_POS:
        p = p.cut(cyl_y(2.2, y0 - 1, y1 + 1, x, z))   # M4
    p = p.cut(cyl_y(KNEE_SHAFT_D / 2 + 0.1, y0 - 1, y1 + 1, 0, KNEE_Z))
    # 张紧轮长槽（M5，±6mm 调节）
    ix, iz = IDLER_POS
    p = p.cut(stadium_y((ix - IDLER_ADJ, iz), (ix + IDLER_ADJ, iz), 2.7, y0 - 1, y1 + 1))
    return p


def thigh_inner():
    M = MOTOR
    y0, y1 = Y_INNER
    p = _thigh_plate(y0, y1)
    p = p.cut(cyl_y(7, y0 - 1, y1 + 1))  # 中心走线孔
    for (dx, dz) in bolt_circle(M["out_pcd"], M["out_n"], 0):           # HFE 输出端 6×M3，沉头在 y1 侧
        p = p.cut(cyl_y(M["out_hole"] / 2, y0 - 1, y1 + 1, dx, dz))
        p = p.cut(cyl_y(3.2, y1 - 3.5, y1 + 1, dx, dz))
    p = dowel_holes_y(p, y0 - 1, y0 + 5, 0, 0, M["out_dowel_pcd"], M["out_dowel_n"], M["out_dowel_d"], M["out_dowel_phase"])
    # 膝电机不再用背面固定（背面 Φ38 孔会被 HFE 法兰挡住），改由前安装环 + 髋部立柱固定
    # v0.4：去掉后缘翻边（外壳围边接管了刚度）；膝电机的线从这个孔穿到内侧，再进机身
    p = p.cut(cyl_y(5.0, y0 - 1, y1 + 1, *KFE_CABLE))
    return p.clean()


def kfe_front_ring():
    """膝电机前安装环：6×M3 锁在电机外壳前端（Φ50），3 个耳朵套在髋部立柱上"""
    M = MOTOR
    y0, y1 = Y_KFE_RING
    r = cyl_y(31, y0, y1).cut(cyl_y(20.5, y0 - 1, y1 + 1))
    for (x, z) in STANDOFF_POS[:N_HIP_STANDOFF]:
        d = math.hypot(x, z)
        ux, uz = x / d, z / d
        r = r.fuse(stadium_y((ux * 26, uz * 26), (x, z), 6.0, y0, y1))
    r = r.cut(cyl_y(20.5, y0 - 1, y1 + 1))
    for (x, z) in STANDOFF_POS[:N_HIP_STANDOFF]:
        r = r.cut(cyl_y(2.2, y0 - 1, y1 + 1, x, z))
    for (dx, dz) in bolt_circle(M["front_pcd"], M["front_n"], 0):
        r = r.cut(cyl_y(M["out_hole"] / 2, y0 - 1, y1 + 1, dx, dz))
        r = r.cut(cyl_y(2.9, y1 - 2.2, y1 + 1, dx, dz))
    for a in M["front_dowel_angles"]:
        t = math.radians(a)
        r = r.cut(cyl_y(M["front_dowel_d"] / 2 + 0.05, y0 - 1, y0 + 3, M["front_pcd"] / 2 * math.cos(t), M["front_pcd"] / 2 * math.sin(t)))
    return r.clean()


SHANK_ZT = -L_SHANK + 14


def shank_profile(style=None):
    """小腿杆身外形（膝坐标 x 前、z 上，膝轴在原点）。弯折都放在"后侧"（-x）：
    膝关节折叠时小腿的前侧贴向大腿，后侧朝外，所以往后拱不会碰到大腿。足端球心始终在 (0, -L_SHANK)"""
    style = style or LEG_STYLE
    zt = SHANK_ZT
    if style == "A":                                   # 直小腿（v0.5 原样）
        return [(-19, -8), (19, -8), (12, zt), (-12, zt)]
    if style == "B":                                   # 后弓小腿：膝下 60mm 处往后鼓出"小腿肚"，前沿保持直线
        return [(19, -8), (12, zt), (-12, zt), (-16, -126), (-38, -66), (-30, -34), (-19, -8)]
    if style == "C":                                   # 狗腿反弓：小腿在 90mm 处往后折约 30°，前沿跟着内凹
        return [(19, -8), (-5, -90), (12, zt), (-12, zt), (-43, -90), (-19, -8)]
    raise ValueError(style)


def _notch_sweep():
    """小腿在大腿围边上扫过的区域（腿坐标零位）：膝关节全行程 -12°..150°。
    v0.6：按小腿真实外形（外扩 2mm）扫，再并上原来 ±21 的直条，小腿怎么弯槽口都跟着变"""
    y0, y1 = Y_INNER[1] - 0.1, Y_PARTITION[0] - 1.0
    prof = xz_prism(offset_poly(shank_profile(), -2.0), y0, y1).fuse(box(-21, 21, y0, y1, -L_SHANK - 25, 4))
    sweep = None
    a = -KNEE_BETA_RANGE[1] - 4
    while a <= -KNEE_BETA_RANGE[0] + 4:
        b = prof.rotate(V(0, 0, 0), V(0, 1, 0), a)
        sweep = b if sweep is None else sweep.fuse(b)
        a += 4
    return sweep.translate(V(0, 0, KNEE_Z))


Z_FORK = -40.0                    # v0.6：这个高度以下，大腿围边只包皮带层（和隔板上沿对齐）


def thigh_outer():
    """大腿外壳：外板 + 一圈围边。皮带、带轮、张紧轮都包在里面；后侧鼓出一个小包罩住张紧轮。
    v0.6"叉形"大腿：髋部（z > Z_FORK，里面是膝电机）围边从外板一直包到内板；下半段围边只包皮带那一层
    （到隔板为止），小腿那一层不做围边，小腿夹在隔板和内板之间活动——从外面看没有槽口，是一整块完整的大腿"""
    y0, y1 = Y_OUTER
    t = THIGH_WALL
    p = _thigh_plate(y0, y1)
    ix, iz = IDLER_POS
    ya, yb = Y_INNER[1], y0 + 0.5
    outer = tapered_slot_xz(R_HIP_PLATE, R_KNEE_PLATE, L_THIGH, ya, yb).fuse(
        stadium_y((ix - IDLER_ADJ, iz), (ix + IDLER_ADJ, iz), IDLER_R + 1.0 + t, ya, yb))           # 张紧轮罩（鼓包）
    inner = tapered_slot_xz(R_HIP_PLATE - t, R_KNEE_PLATE - t, L_THIGH, ya - 1, yb + 1).fuse(
        stadium_y((ix - IDLER_ADJ, iz), (ix + IDLER_ADJ, iz), IDLER_R + 1.0, ya - 1, yb + 1))
    wall = outer.cut(inner)
    upper = wall.intersect(box(-80, 80, ya - 1, yb + 1, Z_FORK, 80)).cut(_notch_sweep())
    lower = wall.intersect(box(-80, 80, Y_PARTITION[0], yb + 1, -L_THIGH - 60, Z_FORK + 0.01))
    p = p.fuse(upper).fuse(lower)
    p = p.cut(stadium_y((ix - IDLER_ADJ, iz), (ix + IDLER_ADJ, iz), 4.9, y1 - 2.9, y1 + 0.1))   # 张紧轮 M5 沉头螺丝的长条沉孔
    # 小带轮外端支撑轴承 688ZZ（8×16×5）
    p = p.fuse(cyl_y(14, y1, y1 + 3))
    p = p.cut(cyl_y(8.05, y0 - 1, y0 + 5))
    p = p.cut(cyl_y(5.0, y0 - 1, y1 + 4))
    # 膝轴外端凸台（膝轴用 M5 螺丝从两端锁在大腿板上）
    p = p.fuse(cyl_y(11, y1, y1 + 2.5, 0, KNEE_Z)).cut(cyl_y(KNEE_SHAFT_D / 2 + 0.1, y0, y1 + 4, 0, KNEE_Z))
    for a in (30, 150, 270):                                  # v0.6 银色膝盖盖的 M2 自攻孔（膝盖露在装甲外面）
        t = math.radians(a)
        p = p.cut(cyl_y(0.8, y1 - 4, y1 + 0.1, 18 * math.cos(t), KNEE_Z + 18 * math.sin(t)))
    import armor                                              # v0.6 大腿装甲的 M3 热熔螺母孔
    for (x, z) in armor.THIGH_COVER_SCREWS:
        p = p.cut(cyl_y(2.0, y1 - 4.5, y1 + 0.1, x, z))
    return p.clean()


def thigh_partition():
    """大腿内隔板（3mm）：套在中段两根立柱上，把小腿活动的那一层和皮带层隔开"""
    y0, y1 = Y_PARTITION
    r = THIGH_WALL + 0.4
    p = tapered_slot_xz(R_HIP_PLATE - r, R_KNEE_PLATE - r, L_THIGH, y0, y1)
    p = p.cut(box(-60, 60, y0 - 1, y1 + 1, -40, 60))                           # 上面是膝电机，隔板从 z=-40 往下
    p = p.cut(cyl_y(25, y0 - 1, y1 + 1, 0, KNEE_Z))                            # 小腿轮毂法兰
    ix, iz = IDLER_POS
    p = p.cut(stadium_y((ix - IDLER_ADJ, iz), (ix + IDLER_ADJ, iz), 5.2, y0 - 1, y1 + 1))      # 张紧轮套筒
    for (x, z) in STANDOFF_POS[N_HIP_STANDOFF:]:
        p = p.cut(cyl_y(2.2, y0 - 1, y1 + 1, x, z))
    return p.clean()


def standoffs():
    """立柱（M4 通螺丝）。髋部 3 根分两段夹住膝电机前安装环，中段 2 根分两段夹住隔板"""
    out = []
    for i, (x, z) in enumerate(STANDOFF_POS):
        if i < N_HIP_STANDOFF:
            segs = [(Y_INNER[1], Y_KFE_RING[0]), (Y_KFE_RING[1], Y_OUTER[0])]
        else:                                                      # 中段 2 根分两段，中间夹住隔板
            segs = [(Y_INNER[1], Y_PARTITION[0]), (Y_PARTITION[1], Y_OUTER[0])]
        for (a, b) in segs:
            out.append(cyl_y(4.5, a, b, x, z).cut(cyl_y(2.2, a - 1, b + 1, x, z)))
    return out


def _toothed(r_pitch, teeth, y0, y1, yb0, yb1, flange_r):
    """带齿的带轮外形：齿顶圆 + 齿槽 + 两侧挡边"""
    r_od = r_pitch - OD_GAP
    p = cyl_y(r_od, yb0, yb1)
    for i in range(teeth):
        a = 2 * math.pi * i / teeth
        p = p.cut(cyl_y(1.55, yb0 - 0.1, yb1 + 0.1, (r_od + 0.35) * math.cos(a), (r_od + 0.35) * math.sin(a)))
    p = p.fuse(cyl_y(flange_r, y0, yb0)).fuse(cyl_y(flange_r, yb1, y1))
    return p.clean()


def pulley_adapter():
    """转接盘：6×M3 接 KFE 输出法兰；外面 3×M3 热熔螺母给小带轮"""
    M = MOTOR
    y0, y1 = Y_ADAPTER
    a = cyl_y(19, y0, y1)
    for (dx, dz) in bolt_circle(M["out_pcd"], M["out_n"], 0):
        a = a.cut(cyl_y(M["out_hole"] / 2, y0 - 1, y1 + 1, dx, dz))
        a = a.cut(cyl_y(3.2, y1 - 2.2, y1 + 1, dx, dz))
    for (dx, dz) in bolt_circle(18, 3, 30):
        a = a.cut(cyl_y(2.0, y0 - 1, y1 + 1, dx, dz))     # M3 热熔螺母孔
    a = a.cut(cyl_y(4.1, y0 - 1, y1 + 1))
    a = dowel_holes_y(a, y0 - 1, y0 + 3, 0, 0, M["out_dowel_pcd"], M["out_dowel_n"], M["out_dowel_d"], M["out_dowel_phase"])
    return a


def pulley_small():
    y0, y1 = Y_PULLEY
    p = _toothed(R_S, PULLEY_T_SMALL, y0, y1, YB0, YB1, 19.0)
    p = p.fuse(cyl_y(4.0, y1, Y_OUTER[0] + 5))          # Φ8 外支撑轴头 -> 688ZZ
    for (dx, dz) in bolt_circle(18, 3, 30):
        p = p.cut(cyl_y(1.7, y0 - 1, y1 + 1, dx, dz))
        p = p.cut(cyl_y(3.0, y1 - 3, y1 + 1, dx, dz))
    return p.clean()


def pulley_big():
    """36T 大带轮（膝坐标）：4×M4 锁在小腿轮毂法兰上，外端装一颗 6800ZZ"""
    y0, y1 = Y_PULLEY
    p = _toothed(R_B, PULLEY_T_BIG, y0, y1, YB0, YB1, R_B + 2.6)
    p = p.cut(cyl_y(6.0, y0 - 1, y1 + 1))
    p = p.cut(cyl_y(9.55, y1 - 5, y1 + 1))
    for (dx, dz) in bolt_circle(36, 4, 45):
        p = p.cut(cyl_y(2.2, y0 - 1, y1 + 1, dx, dz))
        p = p.cut(cyl_y(3.9, y1 - 4, y1 + 1, dx, dz))
    for (dx, dz) in bolt_circle(36, 4, 0):                # 减重孔
        p = p.cut(cyl_y(4.5, y0 + 2, y1 - 2, dx, dz))
    return p.clean()


def belt():
    """HTD-5M 皮带（腿坐标零位），按张紧轮名义位置压弯后的实际走线"""
    _, po = _belt_loop(IDLER_PRESS, BELT_OUT)
    _, pi = _belt_loop(IDLER_PRESS, BELT_IN)
    outer = xz_prism(po, Y_PULLEY[0] + 1.5, Y_PULLEY[1] - 1.5)
    inner = xz_prism(pi, Y_PULLEY[0], Y_PULLEY[1])
    return outer.cut(inner)


def idler():
    """张紧轮：打印滚轮 + 2×F625ZZ，M5 螺丝穿过内外板长槽"""
    ix, iz = IDLER_POS
    r = cyl_y(IDLER_R, YB0 + 0.3, YB1 - 0.3, ix, iz).cut(cyl_y(2.6, YB0, YB1, ix, iz))
    axle = cyl_y(2.5, Y_INNER[0] - 2, Y_OUTER[1], ix, iz)          # v0.6 外侧用 M5 沉头螺丝，头和外板平齐（大腿装甲直接盖在上面）
    sleeve = cyl_y(4.5, Y_INNER[1], Y_PULLEY[0], ix, iz).fuse(cyl_y(4.5, Y_PULLEY[1], Y_OUTER[0], ix, iz))
    sleeve = sleeve.cut(cyl_y(2.6, Y_INNER[0], Y_OUTER[1], ix, iz))
    return r, axle, sleeve


def knee_shaft():
    return cyl_y(KNEE_SHAFT_D / 2, Y_INNER[0], Y_OUTER[1] + 2.5, 0, KNEE_Z)


def shank():
    """小腿（膝坐标）：轮毂 + 膝部圆盘 + 带轮法兰 + 杆身。前后 38→24mm、厚 32mm。
    v0.4：杆身改实心（去掉减重长孔）；加膝部圆盘，从大腿槽口看进去是实心的"""
    hub = cyl_y(17, Y_SHANK_HUB[0], Y_SHANK_HUB[1] - 10)
    # 膝部"鼓"：外圈 3mm 壁 + 内侧端板，里面空着减重（从外面看是一整个实心圆盘）
    drum = chamfer_edges(cyl_y(KNEE_DISC_R, SHANK_Y[0], SHANK_Y[1] - 0.5), "%CIRCLE", 1.5)
    drum = drum.cut(cyl_y(KNEE_DISC_R - 3, SHANK_Y[0] + 2, SHANK_Y[1] + 1))
    hub = hub.fuse(drum)
    hub = hub.fuse(cyl_y(23, Y_SHANK_HUB[1] - 10, Y_SHANK_HUB[1]))        # 带轮法兰
    hub = chamfer_edges(hub, "%CIRCLE", 1.2)
    zt = SHANK_ZT
    bar = xz_prism(shank_profile(), *SHANK_Y)
    bar = chamfer_edges(bar, "|Y", 3.0)
    tip = box(-11, 11, SHANK_Y[0] + 3, SHANK_Y[1] - 3, -L_SHANK + 2, zt + 1)
    s = hub.fuse(bar).fuse(tip).clean()
    s = s.cut(cyl_y(6, Y_SHANK_HUB[0] - 1, Y_SHANK_HUB[1] + 1))
    s = s.cut(cyl_y(9.55, Y_SHANK_HUB[0] - 1, Y_SHANK_HUB[0] + 5))          # 6800ZZ
    for (dx, dz) in bolt_circle(36, 4, 45):
        s = s.cut(cyl_y(1.9, Y_SHANK_HUB[1] - 9, Y_SHANK_HUB[1] + 1, dx, dz))  # M4 热熔螺母
    s = s.cut(cyl_x(2.2, -20, 20, (SHANK_Y[0] + SHANK_Y[1]) / 2, -L_SHANK + 7))
    s = s.cut(cyl_x(1.9, -20, 20, (SHANK_Y[0] + SHANK_Y[1]) / 2, (-L_SHANK + 11.5 + SHIN_Z[1] - 0.2) / 2))   # 脚踝护圈 M3 热熔螺母
    for z in (-30.0, -100.0, -150.0):                                        # 护甲 M3 热熔螺母
        s = s.cut(cyl_y(1.9, SHANK_Y[1] - 6, SHANK_Y[1] + 1, 0, z if z != -100.0 else -137.0))
    return s


SHIN_Z = (-27.0, -172.0)          # 小腿护甲上下端（膝坐标）


def _shin_outline(inset):
    """护甲外形：小腿外形往里缩 inset，再截取 SHIN_Z 这一段"""
    return xz_prism(offset_poly(shank_profile(), inset), Y_SHIN[0] - 2, Y_SHIN[1] + 2).intersect(
        box(-60, 60, Y_SHIN[0] - 3, Y_SHIN[1] + 3, SHIN_Z[1], SHIN_Z[0]))


def shin_armor():
    """小腿外侧银色护甲（膝坐标）。v0.6：外形跟着小腿杆身走（A 直 / B 后弓 / C 反弓），大斜切折面；
    中间挖一块浅槽，贴深灰内嵌板，做出概念图那种"银框 + 深色板"的双色效果。宽度在大腿槽口以内，不影响膝关节活动"""
    y0, y1 = Y_SHIN
    a = _shin_outline(1.0).intersect(box(-60, 60, y0, y1, -400, 100))
    a = chamfer_edges(a, ">Y", 1.8)
    a = a.cut(_shin_outline(4.0).intersect(box(-60, 60, y1 - 1.0, y1 + 1, SHIN_Z[1] + 14, SHIN_Z[0] - 10)))   # 内嵌板浅槽
    for z in (-30.0, -137.0):
        a = a.cut(cyl_y(1.7, y0 - 1, y1 + 1, 0, z))
    return a.clean()


def shin_insert():
    """小腿护甲上的深灰内嵌板（1mm 厚，双面胶贴在浅槽里，顺便盖住护甲螺丝）"""
    y1 = Y_SHIN[1]
    return _shin_outline(4.3).intersect(box(-60, 60, y1 - 0.95, y1 - 0.1, SHIN_Z[1] + 14.3, SHIN_Z[0] - 10.3))


def ankle_collar():
    """银色脚踝护圈（膝坐标）：套在小腿末端、足端上面，像概念图里的金属脚踝；1 颗 M3 横向锁住"""
    z0, z1 = -L_SHANK + 11.5, SHIN_Z[1] - 0.2
    o = chamfer_edges(box(-15.5, 15.5, SHANK_Y[0] - 2.5, SHANK_Y[1] + 2.5, z0, z1), "|Z", 3.0)
    o = chamfer_edges(o, ">Z or <Z", 0.8)
    o = o.cut(box(-12.8, 12.8, SHANK_Y[0] - 0.3, SHANK_Y[1] + 0.3, z0 - 1, z1 + 1))
    o = o.cut(xz_prism(offset_poly(shank_profile(), -0.4), SHANK_Y[0] - 0.3, SHANK_Y[1] + 0.3))   # 跟着小腿外形让位（反弓小腿下端略宽）
    o = o.cut(box(-16, 16, SHANK_Y[1] + 1.9, SHANK_Y[1] + 3, (z0 + z1) / 2 - 0.4, (z0 + z1) / 2 + 0.4))   # 一道细线
    return o.cut(cyl_x(1.7, -17, 17, (SHANK_Y[0] + SHANK_Y[1]) / 2, (z0 + z1) / 2)).clean()


FOOT_CORE_R = FOOT_D / 2 - 2.5
FOOT_YC = (SHANK_Y[0] + SHANK_Y[1]) / 2


def foot_core():
    """足端内芯（TPU 95A）：插在小腿末端，M4 横销固定"""
    zc = -L_SHANK
    f = cq.Solid.makeSphere(FOOT_CORE_R, V(0, FOOT_YC, zc), angleDegrees1=-90, angleDegrees2=90)
    f = f.fuse(cyl_z(14.5, zc, zc + 11, 0, FOOT_YC))
    f = f.cut(box(-11.2, 11.2, SHANK_Y[0] + 2.8, SHANK_Y[1] - 2.8, zc + 1.8, zc + 20))
    f = f.cut(box(-20, 20, FOOT_YC - 20, FOOT_YC + 20, zc + 13.5, zc + 22))
    f = f.cut(cyl_x(2.2, -25, 25, FOOT_YC, zc + 7))
    return f


def foot_tread():
    """防滑外套（TPU 85A 或浇注硅胶 40A）：包住内芯下半部，表面浅横纹 + 竖纹，磨损后单独更换"""
    zc = -L_SHANK
    R = FOOT_D / 2
    t = cq.Solid.makeSphere(R, V(0, FOOT_YC, zc), angleDegrees1=-90, angleDegrees2=90)
    t = t.cut(cq.Solid.makeSphere(FOOT_CORE_R, V(0, FOOT_YC, zc), angleDegrees1=-90, angleDegrees2=90))
    t = t.cut(box(-30, 30, FOOT_YC - 30, FOOT_YC + 30, zc + 5, zc + 30))
    t = t.cut(box(-11.2, 11.2, SHANK_Y[0] + 2.8, SHANK_Y[1] - 2.8, zc + 1.8, zc + 30))   # 给小腿末端让位
    inner_keep = cq.Solid.makeSphere(R - 1.0, V(0, FOOT_YC, zc), angleDegrees1=-90, angleDegrees2=90)
    for k in range(8):                                                      # 竖纹
        g = box(-0.6, 0.6, -30, 30, zc - 30, zc + 6).rotate(V(0, 0, 0), V(0, 0, 1), 22.5 * k)
        t = t.cut(g.translate(V(0, FOOT_YC, 0)).cut(inner_keep))
    for dz in (-6.0, -11.5):                                                # 横纹
        ring = cyl_z(30, zc + dz - 0.6, zc + dz + 0.6, 0, FOOT_YC).cut(inner_keep)
        t = t.cut(ring)
    return t.clean()


def to_knee(shape):
    """膝坐标 -> 腿坐标零位"""
    return shape.translate(V(0, 0, KNEE_Z))
