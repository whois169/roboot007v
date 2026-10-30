"""007A 第 2 步：一条腿的工程结构（方案 B：髋、膝两个电机同轴藏进机身侧面的窝，皮带和大腿在机身外面）。

左前腿，腿坐标：原点 = 髋轴中心（机身坐标 x = HIP_X、z = HIP_Z），x 前、z 上；y 直接用机身坐标（向左为 +）。
从里到外（y）：
  外摆电机（轴朝前后，在髋后面，定子固定在机身）→ 髋座（跟外摆一起转：后板接外摆输出、侧板托髋电机）
  → 髋电机（定子在髋座侧板上，输出朝外；髋座只挂在外摆电机输出上，像宇树那样，上下两块三角筋加强）→ 大腿杯（杯底接髋电机输出；膝电机的定子也拧在杯底上，跟大腿一起转）
  → 膝电机（输出朝外）→ 转接轴 + 20 齿小带轮（在两块大腿板之间）→ 大腿外板（外侧轴承托住转接轴）
膝：30 齿大带轮和小腿根做成一体，套在膝轴上（两块大腿板各一个轴承），减速 30 / 20 = 1.5。
皮带：HTD5M 宽 20 PU 钢丝芯闭口带（旧四足 v0.4 同款），张紧轮在大腿后侧直段上。
电机尺寸按宇树 GO-M8010-6 官方尺寸图（007a/ref/go-m8010-6/尺寸图.png）：Φ96.5 × 42.3，输出盘 Φ59 凸出 6.6、6-M4 在 Φ34.5，背面 8-M3.5 在 R42.5。
用法：python 007a/eng_leg.py → 007a/out/leg_parts（STL + manifest，给 blender/render.py）和 docs/机械狗007A/腿_截面.png
"""
import json
import math
import os

import cadquery as cq

import model as M
from model import V, box, cyl_x, cyl_y, prism_xz, ring_y, soft

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- 电机（宇树 GO-M8010-6）
# v20：换回宇树 GO-M8010-6（用户定），按官方《GO-M8010-6 用户手册 V1.0》第 2 节尺寸图（007a/ref/go-m8010-6/尺寸图.png）：
#   壳 Φ96.5（两边削平，平面距 92.5）、壳长 27.7 ±0.05；总长 42.3
#   输出面：输出盘 Φ59 凸出 6.6，6-M4 在 Φ34.5（0°、60°…，0° 对着削平那个方向）；壳上 8 根筋各一个 M3.5（R44.5，削平方向两个在 ±42.5）
#   背面：8-M3.5 在 R42.5（0°、45°…）；中间凸台凸出 8（42.3 − 27.7 − 6.6）、直径图上没标，按比例量约 Φ61 → 取 Φ62（★ 到货量），板上开 Φ64 让它穿过去
#   螺纹深度图上没标（★ 到货量），先按 M3.5 拧 5、M4 拧 6 算
MD, MH = M.MOTOR_D, M.MOTOR_H                    # Φ96.5，总长 42.3
MBOSS_D, MBOSS_H = 59.0, 6.6                     # 输出盘
MBACK_D, MBACK_H = 62.0, 8.0                     # 背面凸台（★ 直径按图比例量的）
OUT_PCD, OUT_N, OUT_HOLE = 34.5, 6, 4.0          # 输出盘 6-M4 在 Φ34.5
OUT_CLR = 4.5                                    # M4 过孔
PIN_PCD, PIN_N, PIN_D = 0.0, 0, 0.0               # 宇树输出盘没有定位销
BACK_R, BACK_N = 42.5, 8                         # 背面 8-M3.5 在 R42.5
BACK_HOLE = 3.9                                  # M3.5 过孔

# ---------------------------------------------------------------- 皮带（HTD5M）
PITCH = 5.0
Z_S, Z_B = 20, 30                                # 小轮 20 齿（髋）、大轮 30 齿（膝）→ 1.5
PD_S, PD_B = Z_S * PITCH / math.pi, Z_B * PITCH / math.pi
BELT_W, BELT_T = 20.0, 3.6

# ---------------------------------------------------------------- y 分层（机身坐标，左腿）
W2 = M.W2                                        # 机身侧面 y = 130
Y_HAA = 70.0                                     # 外摆轴的 y（外摆电机 Φ96.5 要整个装在机身里：70 + 48 = 118 < 下层底盘内壁 121）
Y_HFE = (50.0, 50.0 + MH)                        # 髋电机（输出朝外）
Y_SIDE = (Y_HFE[0] + MBACK_H - 6.0, Y_HFE[0] + MBACK_H)   # 髋座侧板：贴在髋电机背面（背面凸台从中间 Φ64 孔穿过去）
Y_CUP_BOT = (Y_HFE[1], Y_HFE[1] + 5.0)           # 大腿杯底（夹在髋电机输出和膝电机背面之间）
Y_KFE = (Y_CUP_BOT[1], Y_CUP_BOT[1] + MH)        # 膝电机（跟大腿一起转，输出朝外）
Y_TIN = (Y_KFE[1] - 5.0, Y_KFE[1] - 1.0)         # 大腿内板（让开膝电机输出盘）
Y_ADP = (Y_KFE[1], Y_KFE[1] + 4.0)               # 转接盘（接膝电机输出）
Y_PUL = (Y_ADP[1] + 1.0, Y_ADP[1] + 1.0 + BELT_W + 3.0)   # 小带轮（含挡边）
Y_TOUT = (Y_PUL[1] + 1.0, Y_PUL[1] + 5.0)        # 大腿外板
Y_LEG = (Y_PUL[0] + Y_PUL[1]) / 2                # 腿平面（皮带中线）
CUP_OD = MD + 8.0                                # 大腿杯外径（包住膝电机）
CAV_R = CUP_OD / 2 + 6.0                         # 机身侧面窝的半径（杯外面留 6）
PIT_R, PIT_D = 62.0, 5.0                         # 外观上的髋窝（body_v10.py）

# 外观跟着改：膝圆头要装下 30 齿大轮，髋盘要包住大腿杯
RK_NEW = PD_B / 2 + BELT_T + 4.0                 # 大轮节圆半径 + 皮带厚 + 壁
HEX_A_NEW = CUP_OD / 2 + 2.0


def motor(name, y0, out_dir, color="skel_dk"):
    """宇树 GO-M8010-6：背面凸台 Φ62 × 8 → 壳 Φ96.5 × 27.7 → 输出盘 Φ59 × 6.6；out_dir = +1 输出朝 +y。"""
    segs = [(MBACK_D, MBACK_H), (MD, MH - MBACK_H - MBOSS_H), (MBOSS_D, MBOSS_H)]
    if out_dir < 0:
        segs = segs[::-1]
    y, m = y0, None
    for d, h in segs:
        c = cyl_y(0, 0, d, y, y + h)
        m = c if m is None else m.fuse(c)
        y += h
    yo = y0 + MH if out_dir > 0 else y0                                # 输出面
    for k in range(OUT_N):                                             # 输出盘 6-M4（深 ★ 按 6 画）
        a = 2 * math.pi * k / OUT_N
        y_a, y_b = (yo - 6, yo + 0.1) if out_dir > 0 else (yo - 0.1, yo + 6)
        m = m.cut(cyl_y(OUT_PCD / 2 * math.cos(a), OUT_PCD / 2 * math.sin(a), OUT_HOLE, y_a, y_b))
    return name, m, color


def out_holes(plate, y0, y1, cx=0.0, cz=0.0):
    """接电机输出盘的件：6-Φ4.5（M4）在 Φ34.5。"""
    plate = bolt_ring(plate, OUT_PCD / 2, OUT_N, OUT_CLR, y0, y1, cx=cx, cz=cz)
    return bolt_ring(plate, PIN_PCD / 2, PIN_N, PIN_D, y0, y1, a0=math.pi / 6, cx=cx, cz=cz)


def bolt_ring(plate, r, n, d, y0, y1, a0=0.0, cx=0.0, cz=0.0):
    for k in range(n):
        a = a0 + 2 * math.pi * k / n
        plate = plate.cut(cyl_y(cx + r * math.cos(a), cz + r * math.sin(a), d, y0 - 1, y1 + 1))
    return plate


def hull(pts):
    """凸包（逆时针）。"""
    pts = sorted(set(pts))
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and (lo[-1][0] - lo[-2][0]) * (p[1] - lo[-2][1]) - (lo[-1][1] - lo[-2][1]) * (p[0] - lo[-2][0]) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and (up[-1][0] - up[-2][0]) * (p[1] - up[-2][1]) - (up[-1][1] - up[-2][1]) * (p[0] - up[-2][0]) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def belt_loop(knee, y0, y1):
    """皮带：两个带轮节圆的外切闭环（= 两个圆的凸包）；节线往外 0.6、往里 3.0 当皮带厚。"""
    kx, kz = knee

    def ring(off):
        pts = []
        for i in range(72):
            a = 2 * math.pi * i / 72
            pts.append((round((PD_S / 2 + off) * math.cos(a), 4), round((PD_S / 2 + off) * math.sin(a), 4)))
            pts.append((round(kx + (PD_B / 2 + off) * math.cos(a), 4), round(kz + (PD_B / 2 + off) * math.sin(a), 4)))
        return hull(pts)
    return prism_xz(ring(0.6), y0, y1).cut(prism_xz(ring(-3.0), y0 - 1, y1 + 1))


def belt_length(c):
    d = PD_B / 2 - PD_S / 2
    return 2 * math.sqrt(c ** 2 - d ** 2) + math.pi * (PD_S / 2 + PD_B / 2) + 2 * d * math.asin(d / c)


def leg_module():
    """左前腿的所有工程零件（腿坐标），站立姿势。返回 [(名字, 实体, 颜色, 组)]，组 = 机身 / 外摆 / 大腿 / 小腿。"""
    M.RK, M.HEX_A, M.HUB_CUT = RK_NEW, HEX_A_NEW, HEX_A_NEW + 3.0
    thigh2d, shank2d, knee, foot = M.leg_2d()
    kx, kz = knee
    P = []
    # ---- 外摆电机（机身上），轴朝前后，在髋后面；输出朝前
    xa1 = -(CAV_R + 4.0)                                           # 外摆输出面
    haa = cyl_x(Y_HAA, 0, MBACK_D, xa1 - MH, xa1 - MH + MBACK_H).fuse(cyl_x(Y_HAA, 0, MD, xa1 - MH + MBACK_H, xa1 - MBOSS_H)).fuse(
        cyl_x(Y_HAA, 0, MBOSS_D, xa1 - MBOSS_H, xa1))
    xe = xa1 - MH + MBACK_H                                         # 外摆电机背面（拧在 A02 隔板上）
    P.append(("外摆电机", haa, "skel_dk", "机身"))
    seat = box(xe - 6, xe, Y_HAA - 55, Y_HAA + 45, -55, 55)
    seat = seat.cut(cyl_x(Y_HAA, 0, MBACK_D + 2, xe - 7, xe + 1))
    for k in range(BACK_N):
        a = 2 * math.pi * k / BACK_N
        seat = seat.cut(cyl_x(Y_HAA + BACK_R * math.cos(a), BACK_R * math.sin(a), BACK_HOLE, xe - 7, xe + 1))
    P.append(("外摆电机座（铝板 6）", seat, "metal", "机身"))
    # ---- 髋座（跟外摆转）：后板 + 侧板 + 前板（带短轴，前面进机身隔板的轴承 6001）
    rear = box(xa1, xa1 + 6, Y_SIDE[0], Y_HFE[1] + 6, -55, 55)
    for k in range(OUT_N):
        a = 2 * math.pi * k / OUT_N
        rear = rear.cut(cyl_x(Y_HAA + OUT_PCD / 2 * math.cos(a), OUT_PCD / 2 * math.sin(a), OUT_CLR, xa1 - 1, xa1 + 7))
    for k in range(PIN_N):
        a = math.pi / 6 + 2 * math.pi * k / PIN_N
        rear = rear.cut(cyl_x(Y_HAA + PIN_PCD / 2 * math.cos(a), PIN_PCD / 2 * math.sin(a), PIN_D, xa1 - 1, xa1 + 7))
    side = box(xa1, -xa1, *Y_SIDE, -55, 55).cut(cyl_y(0, 0, MBACK_D + 2, Y_SIDE[0] - 1, Y_SIDE[1] + 1))
    side = bolt_ring(side, BACK_R, BACK_N, BACK_HOLE, *Y_SIDE)
    ribs = None                                                     # 上下两块三角筋（后板和侧板之间）
    for zz in (49.0, -55.0):
        r = prism_xz([(xa1 + 6, 0), (xa1 + 6, 0)], 0, 1) if False else None
        tri = cq.Workplane("XY").polyline([(xa1 + 6, Y_SIDE[1]), (xa1 + 6, Y_HFE[1] + 6), (xa1 + 50, Y_SIDE[1])]).close().extrude(6).val().translate(V(0, 0, zz))
        ribs = tri if ribs is None else ribs.fuse(tri)
    hipseat = rear.fuse(side).fuse(ribs)
    P.append(("髋座（铝，CNC）", soft(hipseat, 1.0), "metal", "外摆"))
    # ---- 绕线环（打印）：髋电机外面那一圈空地（半径 51–56），髋 → 膝电机的线在里面绕 1.5 圈（像钟表发条），大腿转 ±100° 线不拉不磨
    P.append(("绕线环（打印 PETG）", ring_y(0, 0, 112, 102, Y_SIDE[1] + 1, Y_SIDE[1] + 31), "skel_dk", "外摆"))
    # ---- 髋电机（定子在髋座侧板上，输出朝外）
    P.append(motor("髋电机", Y_HFE[0], +1) + ("外摆",))
    # ---- 大腿杯：杯底（接髋电机输出 + 膝电机背面）+ 杯壁 + 内板 + 外板 + 立柱
    cup = cyl_y(0, 0, CUP_OD, *Y_CUP_BOT)
    cup = out_holes(cup, *Y_CUP_BOT)                                   # 杯底拧髋电机输出盘
    cup = cup.fuse(ring_y(0, 0, CUP_OD, CUP_OD - 6, Y_CUP_BOT[1], Y_TIN[0]))
    yk = Y_KFE[0] + MBACK_H                                            # 膝电机背面：杯里一圈法兰托住它（背面凸台从 Φ64 孔里过）
    flange = bolt_ring(ring_y(0, 0, CUP_OD - 5, MBACK_D + 2, yk - 5, yk), BACK_R, BACK_N, BACK_HOLE, yk - 5, yk)
    cup = cup.fuse(flange)
    tin = prism_xz(M.belt_case(knee), *Y_TIN)                          # 内板只包住皮带（外摆时不碰机身）.cut(cyl_y(0, 0, MBOSS_D + 4, Y_TIN[0] - 1, Y_TIN[1] + 1)).cut(cyl_y(kx, kz, 21, Y_TIN[0] - 1, Y_TIN[1] + 1))
    P.append(("大腿杯 + 内板（铝，CNC）", soft(cup.fuse(tin), 0.8), "skel", "大腿"))
    tout = prism_xz(thigh2d, *Y_TOUT).cut(cyl_y(0, 0, 21, Y_TOUT[0] - 1, Y_TOUT[1] + 1)).cut(cyl_y(kx, kz, 21, Y_TOUT[0] - 1, Y_TOUT[1] + 1))
    P.append(("大腿外板（铝 4）", soft(tout, 0.8), "shell", "大腿"))
    shell = prism_xz(thigh2d, M.Y_STEP, Y_TOUT[0]).cut(prism_xz(M.belt_case(knee, 4.0), M.Y_STEP - 1, Y_TOUT[0] + 1))
    P.append(("大腿外层壳（打印，完整外形）", soft(shell, 0.8), "skel", "大腿"))
    # 立柱：髋后上方 2 根 + 中段 2 根（避开皮带）
    ux, uz = kx / math.hypot(kx, kz), kz / math.hypot(kx, kz)
    nx, nz = -uz, ux
    so = []
    for t, off in ((0.35, 0), (0.65, 0)):
        so.append((kx * t + nx * off, kz * t + nz * off))
    for a in (100.0, 160.0):
        r = 44.0
        th = math.atan2(kz, kx) + math.radians(a)
        so.append((r * math.cos(th), r * math.sin(th)))
    for i, (x, z) in enumerate(so):
        P.append((f"立柱 M4 {i}", cyl_y(x, z, 8, Y_TIN[1], Y_TOUT[0]), "metal", "大腿"))
    # ---- 膝电机（定子拧在杯底上，跟大腿转）
    P.append(motor("膝电机", Y_KFE[0], +1) + ("大腿",))
    # ---- 转接盘 + 转接轴 + 小带轮
    adp = cyl_y(0, 0, MBOSS_D, *Y_ADP).fuse(cyl_y(0, 0, 12, Y_ADP[1], Y_TOUT[1]))
    P.append(("转接盘 + 轴（钢，车削）", out_holes(adp, *Y_ADP), "metal", "大腿"))
    pul_s = cyl_y(0, 0, PD_S - 1.1, Y_PUL[0] + 1.5, Y_PUL[1] - 1.5).fuse(cyl_y(0, 0, PD_S + 5, Y_PUL[0], Y_PUL[0] + 1.5)).fuse(
        cyl_y(0, 0, PD_S + 5, Y_PUL[1] - 1.5, Y_PUL[1]))
    P.append((f"小带轮 HTD5M {Z_S} 齿", pul_s, "metal", "大腿"))
    P.append(("外板轴承 6801", ring_y(0, 0, 21, 12, *Y_TOUT), "metal", "大腿"))
    # ---- 张紧轮（压在后侧直段上）
    t = 0.45
    ix, iz = kx * t - nx * (PD_S / 2 + (PD_B - PD_S) / 2 * t + 1.2 + 9), kz * t - nz * (PD_S / 2 + (PD_B - PD_S) / 2 * t + 1.2 + 9)
    P.append(("张紧轮 Φ18", cyl_y(ix, iz, 18, Y_PUL[0] + 1, Y_PUL[1] - 1), "metal", "大腿"))
    # ---- 皮带
    P.append((f"同步带 HTD5M 宽{BELT_W:.0f}", belt_loop(knee, Y_PUL[0] + 1.5, Y_PUL[1] - 1.5), "rubber", "大腿"))
    # ---- 膝：膝轴 + 两个轴承 + 大带轮（和小腿根一体）+ 小腿
    P.append(("膝轴 Φ12", cyl_y(kx, kz, 12, Y_TIN[0], Y_TOUT[1]), "metal", "大腿"))
    for nm, yy in (("膝轴承 6801 内", Y_TIN), ("膝轴承 6801 外", Y_TOUT)):
        P.append((nm, ring_y(kx, kz, 21, 12, *yy), "metal", "大腿"))
    pul_b = cyl_y(kx, kz, PD_B - 1.1, Y_PUL[0] + 1.5, Y_PUL[1] - 1.5).fuse(cyl_y(kx, kz, PD_B + 5, Y_PUL[0], Y_PUL[0] + 1.5)).fuse(
        cyl_y(kx, kz, PD_B + 5, Y_PUL[1] - 1.5, Y_PUL[1]))
    P.append((f"大带轮 HTD5M {Z_B} 齿（小腿根）", pul_b, "metal", "小腿"))
    sh = soft(prism_xz(shank2d, Y_LEG - 8, Y_LEG + 8), 1.5).cut(cyl_y(kx, kz, PD_B + 6, Y_LEG - 9, Y_LEG + 9))
    P.append(("小腿（铝 + 打印壳）", sh, "skel", "小腿"))
    return P, knee, foot


def body_half_width(z):
    """髋处机身横截面（相对髋高 z）的半宽：上层甲板壳（顶上斜肩 22）/ 黑腰 / 下层底盘（底下收腰 18）。"""
    zb, zw, zt = M.Z0 - M.HIP_Z, M.WAIST_TOP - M.HIP_Z, M.TOP - M.HIP_Z
    zt0 = M.TUB_TOP - M.HIP_Z
    if z > zt or z < zb:
        return None
    if z >= zw:
        return min(W2 + 2, W2 + 2 - (z - (zt - 22)))
    if z >= zt0:
        return W2 - 10
    return min(W2 - 6, W2 - 6 - (zb + 18 - z))


def abd_check(thigh2d, knee):
    """分两层查：内板（只包皮带）在 Y_TIN，外层完整外形从 M.Y_STEP 开始。"""
    return _abd(((Y_TIN[0], M.belt_case(knee)), (M.Y_STEP, thigh2d)))


def _abd(layers):
    """外摆转 θ（+ = 脚往外）时，大腿内板上每个点离机身侧面多远（窝里面的不算）。返回 [(θ, 最小间隙)]。"""
    lp = []
    for y, poly in layers:
        for i in range(len(poly)):
            (x0, z0), (x1, z1) = poly[i], poly[(i + 1) % len(poly)]
            for t in (0, 0.25, 0.5, 0.75):
                lp.append((y, x0 + (x1 - x0) * t, z0 + (z1 - z0) * t))
    res = []
    for deg in range(-30, 31, 5):
        th = math.radians(deg)
        gap = 1e9
        for y, x, z in lp:
            if True:
                dy, dz = y - Y_HAA, z
                y2 = Y_HAA + dy * math.cos(th) - dz * math.sin(th)
                z2 = dy * math.sin(th) + dz * math.cos(th)
                hw = body_half_width(z2)
                if hw is None or abs(x) > M.L2 - M.HIP_X + 4:
                    continue
                r = math.hypot(x, z2)
                if r < CAV_R and y2 > Y_SIDE[0]:
                    continue                                        # 在窝里
                if r < PIT_R:
                    hw = min(hw, W2 - PIT_D)                        # 机身侧面的髋窝
                gap = min(gap, y2 - hw)
        res.append((deg, gap))
    return res


def section_png(path, knee, foot):
    """髋处横截面（y–z，看向机身前面）+ 腿侧面（x–z）：分层、尺寸。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(16, 7.6), gridspec_kw=dict(width_ratios=[1.15, 1]))
    zb, zt = M.Z0 - M.HIP_Z, M.TOP - M.HIP_Z                       # 机身底 / 顶（相对髋）
    a1.add_patch(Rectangle((0, zb), W2, zt - zb, fc="#eceef0", ec="k", lw=1))
    a1.add_patch(Rectangle((Y_SIDE[0] - 4, -CAV_R), W2 - Y_SIDE[0] + 6, 2 * CAV_R, fc="white", ec="k", ls="--", lw=1))
    a1.text(W2 - 4, zt - 8, "机身侧面 y=130", ha="right", va="top", fontsize=9)
    a1.text(Y_SIDE[0], CAV_R + 4, f"侧面的窝：Φ{2 * CAV_R:.0f}，深 {W2 - Y_SIDE[0] + 4:.0f}", fontsize=9)
    layers = [("髋座侧板", Y_SIDE, 55, "#a3a9af"), ("髋电机", Y_HFE, MD / 2, "#3d4247"), ("杯底", Y_CUP_BOT, CUP_OD / 2, "#6b7680"),
              ("膝电机", Y_KFE, MD / 2, "#3d4247"), ("杯壁", (Y_CUP_BOT[1], Y_TIN[0]), CUP_OD / 2, None), ("转接盘", Y_ADP, MBOSS_D / 2, "#a3a9af"),
              (f"小带轮 {Z_S} 齿 + 皮带", Y_PUL, PD_S / 2 + 3, "#141516"), ("大腿内板", Y_TIN, 62, "#6b7680"), ("大腿外板", Y_TOUT, 62, "#d9dcdf")]
    for n, (y0, y1), r, c in layers:
        if c is None:
            for s in (1, -1):
                a1.add_patch(Rectangle((y0, s * (CUP_OD / 2 - 3) - 1.5), y1 - y0, 3, fc="#6b7680"))
            continue
        a1.add_patch(Rectangle((y0, -r), y1 - y0, 2 * r, fc=c, ec="k", lw=0.6, alpha=0.95))
        a1.text((y0 + y1) / 2, -r - 4, n, rotation=90, ha="center", va="top", fontsize=8)
    a1.add_patch(Circle((Y_HAA, 0), 6, fc="orange"))
    a1.text(Y_HAA, 9, "外摆轴", ha="center", fontsize=8, color="darkorange")
    a1.plot([Y_LEG, Y_LEG], [-120, 70], "b-.", lw=0.8)
    a1.text(Y_LEG + 2, 64, f"腿平面 y={Y_LEG:.0f}", color="b", fontsize=9)
    wid = 2 * (Y_TOUT[1] + 3)
    a1.set_title(f"髋处横截面（左前腿，朝前看）  整机宽约 {wid:.0f}", fontsize=12)
    a1.set_xlim(-5, Y_TOUT[1] + 30)
    a1.set_ylim(-200, zt + 10)
    a1.set_aspect("equal")
    a1.set_xlabel("y（mm，向外）")
    # 侧面：皮带走向
    kx, kz = knee
    a2.add_patch(Circle((0, 0), MD / 2, fc="#3d4247", alpha=0.35))
    a2.add_patch(Circle((0, 0), PD_S / 2, fc="none", ec="k"))
    a2.add_patch(Circle((kx, kz), PD_B / 2, fc="none", ec="k"))
    a2.add_patch(Circle((kx, kz), RK_NEW, fc="none", ec="gray", ls="--"))
    a2.add_patch(Circle((kx, kz), M.RK if False else 17, fc="none", ec="r", ls=":"))
    a2.plot([0, kx], [0, kz], "k-", lw=0.6)
    c = math.hypot(kx, kz)
    a2.text(kx / 2 + 8, kz / 2, f"中心距 {c:.0f}\n带长 {belt_length(c):.0f}（节线）", fontsize=9)
    a2.text(kx - 70, kz - RK_NEW - 18, f"膝圆头 R17 → R{RK_NEW:.0f}（红点线 = 原来）", fontsize=9)
    a2.add_patch(Rectangle((-260, M.Z0 - M.HIP_Z), 260 + M.L2 - M.HIP_X + 4, M.BODY_H, fc="#eceef0", ec="k", alpha=0.4))
    a2.plot([kx, foot[0]], [kz, foot[1]], "k-", lw=2)
    a2.plot([0, kx], [0, kz], "k-", lw=2)
    a2.set_title(f"侧面：小轮 {Z_S} 齿 Φ{PD_S:.1f} → 大轮 {Z_B} 齿 Φ{PD_B:.1f}，减速 {Z_B / Z_S:.1f}", fontsize=12)
    a2.set_aspect("equal")
    a2.set_xlim(-240, 120)
    a2.set_ylim(-300, 90)
    plt.tight_layout()
    plt.savefig(path, dpi=110)


if __name__ == "__main__":
    P, knee, foot = leg_module()
    out = os.path.join(HERE, "out", "leg_parts")
    os.makedirs(out, exist_ok=True)
    for f in os.listdir(out):
        os.remove(os.path.join(out, f))
    man = []
    for i, (n, s, c, g) in enumerate(P):
        fn = f"{i:03d}.stl"
        cq.exporters.export(cq.Workplane().add(s.translate(V(0, 0, M.HIP_Z))), os.path.join(out, fn), tolerance=0.08, angularTolerance=0.15)
        man.append(dict(file=fn, name=n, color=c, group=g))
    json.dump(man, open(os.path.join(out, "manifest.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=0)
    png = os.path.join(os.path.dirname(HERE), "docs", "机械狗007A", "腿_截面.png")
    section_png(png, knee, foot)
    c = math.hypot(*knee)
    print(f"{len(P)} 个零件；腿平面 y={Y_LEG:.1f}，外板外侧 y={Y_TOUT[1]:.1f} → 整机宽约 {2 * (Y_TOUT[1] + 3):.0f}")
    M.RK, M.HEX_A, M.HUB_CUT = RK_NEW, HEX_A_NEW, HEX_A_NEW + 3.0
    t2d = M.leg_2d()[0]
    print("外摆角 / 大腿离机身侧面的最小间隙（负 = 撞）：", ", ".join(f"{d:+d}°:{g:.0f}" for d, g in abd_check(t2d, knee)))
    print(f"窝 Φ{2 * CAV_R:.0f} 深 {W2 - Y_SIDE[0] + 4:.0f}；膝圆头 R{RK_NEW:.1f}；髋盘边心距 {HEX_A_NEW:.1f}；皮带节线长 {belt_length(c):.1f}（{belt_length(c) / PITCH:.1f} 齿）")
