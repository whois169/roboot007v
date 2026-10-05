"""
RoboDog v0.3 腿：同步带膝关节（腿坐标，零位 = 大腿、小腿都竖直向下）
  腿坐标：原点 = HFE 轴线与 HFE 输出面交点，x 向前，y 向外，z 向上
  膝坐标：原点 = 膝轴与 y=0 平面交点（零位时位于腿坐标 (0, 0, -L_THIGH)）
传动：膝电机(KFE) -> 转接盘 -> 20T 小带轮 -> HTD-5M 皮带 -> 36T 大带轮(固定在小腿上)
"""
import math
import cadquery as cq
from config import *
from parts import (V, box, cyl_x, cyl_y, cyl_z, xz_prism, tapered_slot_xz, stadium_y,
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


def belt_length():
    """开口皮带外切长度（节线）"""
    phi = math.asin((R_B - R_S) / L_THIGH)
    return 2 * L_THIGH * math.cos(phi) + math.pi * (R_B + R_S) + 2 * phi * (R_B - R_S)


def _span_x(z, off):
    """后侧（x<0）皮带直段在高度 z 处的 x，off = 相对节线的径向偏移"""
    t = -z / L_THIGH
    return -((R_S + off) + (R_B - R_S) * t)


IDLER_R = 9.0
IDLER_POS = (_span_x(-70.0, BELT_OUT) - IDLER_R + 1.0, -70.0)   # 张紧轮压住后侧直段 1mm


def _thigh_plate(y0, y1):
    p = tapered_slot_xz(R_HIP_PLATE, R_KNEE_PLATE, L_THIGH, y0, y1)
    # 减重腰形孔：放在前侧，透过去能看到皮带
    p = p.cut(stadium_y((15, -55), (17, -98), 5.5, y0 - 1, y1 + 1))
    p = p.cut(stadium_y((18, -114), (20, -150), 5.0, y0 - 1, y1 + 1))
    for (x, z) in STANDOFF_POS:
        p = p.cut(cyl_y(2.2, y0 - 1, y1 + 1, x, z))   # M4
    p = p.cut(cyl_y(KNEE_SHAFT_D / 2 + 0.1, y0 - 1, y1 + 1, 0, KNEE_Z))
    # 张紧轮长槽（M5，±4mm 调节）
    ix, iz = IDLER_POS
    p = p.cut(stadium_y((ix - 4, iz), (ix + 4, iz), 2.7, y0 - 1, y1 + 1))
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
    # 后缘加强翻边
    lip = tapered_slot_xz(R_HIP_PLATE, R_KNEE_PLATE, L_THIGH, y1, y1 + 6)
    inner = tapered_slot_xz(R_HIP_PLATE - 4, R_KNEE_PLATE - 4, L_THIGH, y1 - 1, y1 + 7)
    lip = lip.cut(inner).cut(box(-0.5, 60, y1 - 1, y1 + 7, -L_THIGH - 40, 60))
    lip = lip.cut(cyl_y(MOTOR["D"] / 2 + 3, y1 - 1, y1 + 7))
    lip = lip.cut(cyl_y(24, y1 - 1, y1 + 7, 0, KNEE_Z))
    ix, iz = IDLER_POS
    lip = lip.cut(cyl_y(7, y1 - 1, y1 + 7, ix, iz))
    for sh in motor_connector_y(Y_KFE[0], KFE_CONN_ANG):                  # 给膝电机插头让位
        bb = sh.BoundingBox()
        lip = lip.cut(box(bb.xmin - 1.5, bb.xmax + 1.5, y1 - 1, y1 + 7, bb.zmin - 1.5, bb.zmax + 1.5))
    return p.fuse(lip).clean()


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


def thigh_outer():
    y0, y1 = Y_OUTER
    p = _thigh_plate(y0, y1)
    # 小带轮外端支撑轴承 688ZZ（8×16×5）
    p = p.fuse(cyl_y(14, y1, y1 + 3))
    p = p.cut(cyl_y(8.05, y0 - 1, y0 + 5))
    p = p.cut(cyl_y(5.0, y0 - 1, y1 + 4))
    # 膝轴外端凸台（膝轴用 M5 螺丝从两端锁在大腿板上）
    p = p.fuse(cyl_y(11, y1, y1 + 2.5, 0, KNEE_Z)).cut(cyl_y(KNEE_SHAFT_D / 2 + 0.1, y0, y1 + 4, 0, KNEE_Z))
    return p.clean()


def standoffs():
    """立柱（M4 通螺丝）。髋部 3 根分两段，中间夹住膝电机前安装环"""
    out = []
    for i, (x, z) in enumerate(STANDOFF_POS):
        segs = [(Y_INNER[1], Y_KFE_RING[0]), (Y_KFE_RING[1], Y_OUTER[0])] if i < N_HIP_STANDOFF else [(Y_INNER[1], Y_OUTER[0])]
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
    """HTD-5M 皮带（腿坐标零位），张紧轮处的压弯忽略"""
    outer = tapered_slot_xz(R_S + BELT_OUT, R_B + BELT_OUT, L_THIGH, Y_PULLEY[0] + 1.5, Y_PULLEY[1] - 1.5)
    inner = tapered_slot_xz(R_S + BELT_IN, R_B + BELT_IN, L_THIGH, Y_PULLEY[0], Y_PULLEY[1])
    return outer.cut(inner)


def idler():
    """张紧轮：打印滚轮 + 2×F625ZZ，M5 螺丝穿过内外板长槽"""
    ix, iz = IDLER_POS
    r = cyl_y(IDLER_R, YB0 + 0.3, YB1 - 0.3, ix, iz).cut(cyl_y(2.6, YB0, YB1, ix, iz))
    axle = cyl_y(2.5, Y_INNER[0] - 2, Y_OUTER[1] + 1.5, ix, iz)
    sleeve = cyl_y(4.5, Y_INNER[1], Y_PULLEY[0], ix, iz).fuse(cyl_y(4.5, Y_PULLEY[1], Y_OUTER[0], ix, iz))
    sleeve = sleeve.cut(cyl_y(2.6, Y_INNER[0], Y_OUTER[1], ix, iz))
    return r, axle, sleeve


def knee_shaft():
    return cyl_y(KNEE_SHAFT_D / 2, Y_INNER[0], Y_OUTER[1] + 2.5, 0, KNEE_Z)


def shank():
    """小腿（膝坐标）：轮毂 + 带轮法兰 + 杆身。v0.3.1 加粗：前后 38→24mm、厚 32mm，中间保留减重长孔"""
    hub = cyl_y(17, Y_SHANK_HUB[0], Y_SHANK_HUB[1] - 10)
    hub = hub.fuse(cyl_y(23, Y_SHANK_HUB[1] - 10, Y_SHANK_HUB[1]))        # 带轮法兰
    hub = chamfer_edges(hub, "%CIRCLE", 1.2)
    zt = -L_SHANK + 14
    bar = xz_prism([(-19, -8), (19, -8), (12, zt), (-12, zt)], *SHANK_Y)
    bar = bar.cut(stadium_y((0, -46), (0, -128), 6.5, SHANK_Y[0] - 1, SHANK_Y[1] + 1))
    bar = chamfer_edges(bar, "|Y", 3.0)
    tip = box(-11, 11, SHANK_Y[0] + 3, SHANK_Y[1] - 3, -L_SHANK + 2, zt + 1)
    s = hub.fuse(bar).fuse(tip).clean()
    s = s.cut(cyl_y(6, Y_SHANK_HUB[0] - 1, Y_SHANK_HUB[1] + 1))
    s = s.cut(cyl_y(9.55, Y_SHANK_HUB[0] - 1, Y_SHANK_HUB[0] + 5))          # 6800ZZ
    for (dx, dz) in bolt_circle(36, 4, 45):
        s = s.cut(cyl_y(1.9, Y_SHANK_HUB[1] - 9, Y_SHANK_HUB[1] + 1, dx, dz))  # M4 热熔螺母
    s = s.cut(cyl_x(2.2, -20, 20, (SHANK_Y[0] + SHANK_Y[1]) / 2, -L_SHANK + 7))
    for z in (-30.0, -100.0, -150.0):                                        # 护甲 M3 热熔螺母
        s = s.cut(cyl_y(1.9, SHANK_Y[1] - 6, SHANK_Y[1] + 1, 0, z if z != -100.0 else -137.0))
    return s


def shin_armor():
    """小腿外侧白色护甲（膝坐标）"""
    y0, y1 = Y_SHIN
    a = xz_prism([(-15, -27), (15, -27), (10, -L_SHANK + 30), (-10, -L_SHANK + 30)], y0, y1)
    a = chamfer_edges(a, ">Y", 1.4)
    a = a.cut(xz_prism([(-2, -40), (2, -40), (1.5, -120), (-1.5, -120)], y1 - 1.2, y1 + 1))   # 中线凹槽
    for z in (-30.0, -137.0):
        a = a.cut(cyl_y(1.7, y0 - 1, y1 + 1, 0, z))
    return a


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
