"""007A 外观模型（cadquery）：机身 + 四条腿，站立姿势。只做外观（工业设计），工程细节（螺丝孔、电机座）下一步再做。

坐标：x 往前、y 往左、z 往上，mm，地面 z = 0。
设计语言延续 007v（配色 B：外壳浅灰白、骨架深灰）：
  机身：下层底盘往里收 → 一圈凹进去的黑腰（两侧青色灯条）→ 上层甲板壳前后大斜面；面板之间 2 mm 分缝；
        头嵌在机身前面：屏幕窗（可抬头低头的屏幕）+ 两边各两道凹槽 + 左上角激光雷达；
        背上两条货物导轨 + 防滑垫；尾部红色尾灯、急停、充电口
  腿：  髋盘（带尖角，照用户草图）和大腿一体，深灰骨架上贴一块浅色外盖板；髋盘外面一圈青色光环；
        膝盖两面盖帽；小腿带折弯；橡胶脚垫
用法：python 007a/model.py → 007a/out/parts/*.stl + manifest.json（给 blender/render.py 渲染）
"""
import json
import math
import os

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "parts")
V = cq.Vector

# ================================================================ 尺寸
BODY_L, BODY_W, BODY_H = 480.0, 260.0, 140.0          # v12：机身高 185 → 160 → 140
L2, W2 = BODY_L / 2, BODY_W / 2
HIP_X = 160.0                                   # 髋往后收：头要在最前面（v2，用户：腿不能超过头）
MOTOR_D = 92.0                                   # 关节电机（GIM8108-8 级别）
SEAM = 2.0

# 腿：比例照用户草图（concept_v3）
H0, K0, B0, T0 = (97, 122), (307, 262), (213, 295), (155, 345)
L1, LS = 200.0, 190.0                           # v13：大腿 230 → 200、小腿 158 → 190（stair_calc.py 算的：住宅楼梯余量 10 → 26 mm，站高不变）
S = LS / math.dist(K0, T0)                       # 小腿草图的缩放（大腿外形由 L1 直接生成，不用这个）
STAND_KNEE = 90.0
HIP_Z = math.sqrt(L1 ** 2 + LS ** 2 - 2 * L1 * LS * math.cos(math.radians(STAND_KNEE)))
Z0 = HIP_Z - 72.0                                # 机身底离地（v12：髋在机身高度一半附近）
TUB_TOP, WAIST_TOP = Z0 + 40, Z0 + 50            # 下层底盘顶、黑腰顶（v12：上层外壳 90 高）
TOP = Z0 + BODY_H
HUB_SK = [(-35, -33), (28, -33), (33, 38), (-4, 49), (-47, 8)]
RK = 17.0                                        # 大腿膝那头的圆头半径
HEX_A = MOTOR_D / 2 + 5.0                        # v6：髋盘每条边离电机中心都是 51（刚好包住电机，四周留 5）
# v6：髋盘照 v3（用户认可的造型：后面尖角、上沿斜边、下面缺口、楔形大腿），只把轮廓理顺、缩小。
# 在“大腿坐标”里定义（u 沿 髋→膝，v 朝大腿上沿那侧），每条边：(方向角, 离中心的距离)；v3 量出来是 34° / -129° / 170° / 106°，取整成：
HUB_EDGES = dict(AB=30.0, CD=-120.0, DE=180.0, EA=120.0)
HUB_CUT = 54.0                                    # 尖角 A 切掉以后，切口离电机中心多远
HUB_B_U, HUB_C_V = 8.0, -12.0                     # 大腿上沿从 B 出发（AB 边上 u = 8 的点），下沿从 C 出发（CD 边上 v = -12 的点）


def _hub_scale():
    """髋盘照草图的尖角外形，按“刚好包住电机（半径 + 5）”缩放：最小边心距 = 51。"""
    best = 1e9
    n = len(HUB_SK)
    for i in range(n):
        (x0, y0), (x1, y1) = HUB_SK[i], HUB_SK[(i + 1) % n]
        best = min(best, abs(x0 * y1 - x1 * y0) / math.hypot(x1 - x0, y1 - y0))   # 原点到边的距离
    return (MOTOR_D / 2 + 5) / best


HUB_F = _hub_scale()
KNEE_TOP = [(303, 234), (311, 245), (313, 262)]
KNEE_BOT = [(296, 266)]
SHANK_SK = [(310, 250), (317, 265), (311, 285), (218, 303), (160, 348), (149, 340), (208, 288), (294, 262)]
T_THIGH, T_SHANK = 34.0, 16.0                    # v3：小腿插在大腿膝盖的叉口里，两段在同一个面上
LEG_Y = W2 + 8 + T_THIGH / 2                     # 大腿中面离中线


# ================================================================ 小工具
def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl_y(x, z, d, y0, y1):
    return cq.Solid.makeCylinder(d / 2, y1 - y0, V(x, y0, z), V(0, 1, 0))


def cyl_x(y, z, d, x0, x1):
    return cq.Solid.makeCylinder(d / 2, x1 - x0, V(x0, y, z), V(1, 0, 0))


def ring_y(x, z, d0, d1, y0, y1):
    return cyl_y(x, z, d0, y0, y1).cut(cyl_y(x, z, d1, y0 - 1, y1 + 1))


def prism_xz(pts, y0, y1):
    """x-z 平面上的多边形，沿 y 从 y0 拉到 y1。"""
    w = cq.Workplane("XZ").polyline(pts).close().extrude(-(y1 - y0))
    return w.val().translate(V(0, y0, 0))


def prism_yz(pts, x0, x1):
    w = cq.Workplane("YZ").polyline(pts).close().extrude(x1 - x0)
    return w.val().translate(V(x0, 0, 0))


def soft(s, r):
    try:
        return cq.Workplane().add(s).edges().fillet(r).val()
    except Exception:
        return s


def prism_xz_offset(pts, d, y0, y1):
    """多边形往里缩（d<0）以后再沿 y 拉伸。"""
    w = cq.Workplane("XZ").polyline(pts).close().offset2D(d, "intersection").extrude(-(y1 - y0))
    return w.val().translate(V(0, y0, 0))


# ================================================================ 腿（腿平面里的 2D：髋在原点，x 往前，膝朝后）
def leg_ik(foot):
    dx, dz = foot
    d = math.hypot(dx, dz)
    a = math.acos(max(-1, min(1, (L1 ** 2 + d ** 2 - LS ** 2) / (2 * L1 * d))))
    ang = math.atan2(dz, dx) - a                                  # 膝在髋→脚连线的后面（x 负的一侧）
    return L1 * math.cos(ang), L1 * math.sin(ang)


def _rot(p, r):
    c, s = math.cos(r), math.sin(r)
    return p[0] * c - p[1] * s, p[0] * s + p[1] * c


def leg_2d(foot=(0.0, -HIP_Z)):
    """返回 (髋盘+大腿 多边形, 小腿 多边形, 膝, 脚)。草图里膝朝右，这里膝朝后（-x），所以先把草图左右镜像。"""
    knee = leg_ik(foot)
    sk = lambda p, o: (-(p[0] - o[0]), -(p[1] - o[1]))           # 草图 → 镜像 + y 朝上
    a_sk = math.atan2(*reversed(sk(K0, H0)))
    r = math.atan2(knee[1], knee[0]) - a_sk
    kx, kz = knee
    th = math.atan2(kz, kx)                                        # 髋 → 膝 的方向
    cu, su = math.cos(th), math.sin(th)
    W = lambda u, v: (u * cu + v * su, u * su - v * cu)            # 大腿坐标 → 腿平面（v 取反：v3 里大腿连续的长直边在后面、髋盘尖角朝前上）

    def line(ang):
        """方向角 ang（度）的边，离中心 HEX_A；返回 (点, 方向)。外法线 = 方向转 +90°（多边形按顺时针走：A→B→膝→C→D→E）。"""
        a = math.radians(ang)
        d = (math.cos(a), math.sin(a))
        nrm = (-d[1], d[0])
        return (nrm[0] * HEX_A, nrm[1] * HEX_A), d

    def meet(l1, l2):
        (p1, d1), (p2, d2) = l1, l2
        den = d1[0] * d2[1] - d1[1] * d2[0]
        t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den
        return p1[0] + d1[0] * t, p1[1] + d1[1] * t
    AB, CD, DE, EA = (line(HUB_EDGES[k]) for k in ("AB", "CD", "DE", "EA"))
    A, D, E = meet(EA, AB), meet(CD, DE), meet(DE, EA)
    B = (HUB_B_U, AB[0][1] + (HUB_B_U - AB[0][0]) * AB[1][1] / AB[1][0])
    C = (CD[0][0] + (HUB_C_V - CD[0][1]) * CD[1][0] / CD[1][1], HUB_C_V)
    L = math.hypot(kx, kz)                                          # 膝在大腿坐标里是 (L, 0)

    def tangent(pt, side):
        """从 pt 到膝盖圆（(L, 0)，半径 RK）的切点；side = +1 上沿 / -1 下沿。"""
        dx, dy = L - pt[0], -pt[1]
        dd = math.hypot(dx, dy)
        phi = math.atan2(dy, dx) + side * math.asin(RK / dd)
        return L - side * RK * math.sin(phi), side * RK * math.cos(phi)
    T1, T2 = tangent(B, 1), tangent(C, -1)
    a1, a2 = math.atan2(T1[1], T1[0] - L), math.atan2(T2[1], T2[0] - L)
    while a2 > a1:
        a2 -= 2 * math.pi
    arc = [(L + RK * math.cos(a1 + (a2 - a1) * i / 16), RK * math.sin(a1 + (a2 - a1) * i / 16)) for i in range(17)]
    # v6.1：A 角是两条边 90° 相交，太尖（用户：不协调）——垂直于它的方向切掉，切口离中心 54，变成一条短边
    ra = math.hypot(*A)
    ua = (A[0] / ra, A[1] / ra)

    def cut_on(p0, p1):                                             # 边 p0→p1 和切口线（离中心 54、垂直于 ua）的交点
        d0 = p0[0] * ua[0] + p0[1] * ua[1] - HUB_CUT
        d1 = p1[0] * ua[0] + p1[1] * ua[1] - HUB_CUT
        t = d0 / (d0 - d1)
        return p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t
    A1, A2 = cut_on(E, A), cut_on(A, B)
    hub_arc = [W(*A2), W(*B)]
    knee_arc = [W(*q) for q in arc] + [W(*C), W(*D), W(*E), W(*A1)]
    thigh = hub_arc + knee_arc

    def at_knee(pts, rr):
        return [tuple(k + d for k, d in zip(knee, _rot((sk(p, K0)[0] * S, sk(p, K0)[1] * S), rr))) for p in pts]
    a_sh = math.atan2(*reversed(sk(T0, K0)))
    a_now = math.atan2(foot[1] - knee[1], foot[0] - knee[0])
    shank = at_knee(SHANK_SK, a_now - a_sh)
    return thigh, shank, knee, foot


def leg_parts(fx, sy):
    """一条腿（fx = 1 前 / -1 后，sy = 1 左 / -1 右）的零件，已放到机身上。"""
    thigh, shank, knee, foot = leg_2d()
    hx, hz = fx * HIP_X, HIP_Z
    P = []

    def place(s, y0):
        s = s.translate(V(hx, 0, hz))
        return s if sy > 0 else s.mirror("XZ")

    yc = LEG_Y                                                     # 大腿中面（左边的腿；右边镜像）
    y_out = yc + T_THIGH / 2
    kx, kz = knee
    th = prism_xz(thigh, yc - T_THIGH / 2, y_out)
    slot = cq.Solid.makeCylinder(RK + 2, T_SHANK + 2, V(kx, yc - T_SHANK / 2 - 1, kz), V(0, 1, 0))   # 膝盖叉口：小腿插进去
    th = soft(th.cut(slot.fuse(prism_xz(shank, yc - T_SHANK / 2 - 1, yc + T_SHANK / 2 + 1))), 2.5)
    P.append(("大腿", place(th, 0), "skel"))
    # 浅色盖板只在大腿上半（沿 髋→膝 中线分开）：上浅下深，看着细、有动势
    ang = math.atan2(kz, kx)
    nx, nz = -math.sin(ang), math.cos(ang)
    up = 1 if nz > 0 else -1
    half = [(nx * up * 400 + math.cos(ang) * -400, nz * up * 400 + math.sin(ang) * -400),
            (nx * up * 400 + math.cos(ang) * 400, nz * up * 400 + math.sin(ang) * 400),
            (math.cos(ang) * 400 + nx * up * 2, math.sin(ang) * 400 + nz * up * 2),
            (-math.cos(ang) * 400 + nx * up * 2, -math.sin(ang) * 400 + nz * up * 2)]
    cover = prism_xz_offset(thigh, -5.0, y_out - 0.5, y_out + 2.0).intersect(prism_xz(half, y_out - 2, y_out + 4))
    ux, uz = math.cos(ang), math.sin(ang)
    keep = [(ux * (HEX_A + 8) + nx * 300, uz * (HEX_A + 8) + nz * 300), (ux * (HEX_A + 8) - nx * 300, uz * (HEX_A + 8) - nz * 300),
            (ux * 900 - nx * 300, uz * 900 - nz * 300), (ux * 900 + nx * 300, uz * 900 + nz * 300)]
    cover = cover.intersect(prism_xz(keep, y_out - 3, y_out + 5)).cut(cyl_y(kx, kz, 2 * RK + 8, y_out - 5, y_out + 6))
    P.append(("大腿外盖板", place(soft(cover, 1.2), 0), "shell"))
    # 髋关节：分层——外壳（大腿一体）→ 一圈台阶 → 电机端盖（深色）→ 青色光环（关节状态灯）→ 金属中心盖 + 6 颗螺栓
    P.append(("髋台阶", place(soft(ring_y(0, 0, 2 * HEX_A - 6, MOTOR_D + 2, y_out, y_out + 2.5), 0.8), 0), "metal"))
    P.append(("髋盖", place(cyl_y(0, 0, MOTOR_D + 2, y_out, y_out + 4), 0), "skel_dk"))
    P.append(("髋光环", place(ring_y(0, 0, MOTOR_D - 6, MOTOR_D - 12, y_out + 4, y_out + 4.6), 0), "cyan"))
    P.append(("髋中心盖", place(soft(cyl_y(0, 0, 34, y_out + 4, y_out + 8), 1.5), 0), "metal"))
    for k in range(6):
        a = math.radians(30 + 60 * k)
        P.append((f"髋螺栓{k}", place(cyl_y(30 * math.cos(a), 30 * math.sin(a), 6, y_out + 4, y_out + 5.5), 0), "metal"))
    # 小腿：和大腿同一个中面，插在叉口里；两面各贴一条浅色盖板
    ys0 = yc - T_SHANK / 2
    sh = soft(prism_xz(shank, ys0, ys0 + T_SHANK), 2.0)
    P.append(("小腿", place(sh, 0), "skel"))
    for y0, y1 in ((ys0 - 1.5, ys0 + 0.5), (ys0 + T_SHANK - 0.5, ys0 + T_SHANK + 1.5)):
        sc = prism_xz_offset(shank, -4.0, y0, y1).cut(cyl_y(kx, kz, 2 * RK + 8, y0 - 1, y1 + 1))
        P.append((f"小腿盖板{y0:.0f}", place(soft(sc, 0.6), 0), "shell"))
    P.append(("膝轴", place(cyl_y(kx, kz, 12, yc - T_THIGH / 2 - 2, y_out + 3), 0), "metal"))
    P.append(("膝盖帽外", place(soft(cyl_y(kx, kz, 30, y_out, y_out + 4), 1.2), 0), "skel_dk"))
    P.append(("膝光点", place(ring_y(kx, kz, 20, 15, y_out + 4, y_out + 4.6), 0), "cyan"))
    fxp, fzp = foot
    pad = cq.Solid.makeSphere(19, V(fxp, ys0 + T_SHANK / 2, fzp + 17)).cut(box(fxp - 30, fxp + 30, ys0 - 20, ys0 + 50, fzp - 10, fzp + 1))
    P.append(("脚垫", place(pad.translate(V(0, 0, -1)), 0), "rubber"))      # 加大、底下削平一点（站得稳）
    # 机身侧面的髋座（外摆电机的法兰，深色圆盘）
    P.append(("髋座", place(soft(cyl_y(0, 0, MOTOR_D + 18, W2 - 1, W2 + 7), 2.0), 0), "skel_dk"))
    tag = ("前" if fx > 0 else "后") + ("左" if sy > 0 else "右")
    return [(f"{tag}{n}", s, c) for n, s, c in P]


# ================================================================ 机身
# v8 做精：一套规则
#   切角：所有斜面都是 45°；大斜面 24（机身前后上下）、中斜面 18（头的上沿和下巴、下层底盘）、俯视转角 10；所有外棱倒 1.5
#   线：  头的正面上下沿 = 两边斜槽的外沿；灯条、字样、检修板都在两条分缝（x = ±120）之间居中
#   接缝：头四周留 4 mm 的影线缝（屏幕抬头低头要的间隙，不是装饰）
CH_BIG, CH_MID, CH_PLAN, EDGE = 18.0, 18.0, 10.0, 1.5
SEAM_X = 120.0


def plan_chamfer(s, ch=CH_PLAN):
    """俯视的四个转角切 45°（竖直的棱）。"""
    for sx in (1, -1):
        for sy in (1, -1):
            bb = s.BoundingBox()
            x, y = (bb.xmax if sx > 0 else bb.xmin), (bb.ymax if sy > 0 else bb.ymin)
            tri = cq.Workplane("XY").polyline([(x + sx, y + sy), (x - sx * ch, y + sy), (x + sx, y - sy * ch)]).close() \
                .extrude(bb.zlen + 2).val().translate(V(0, 0, bb.zmin - 1))
            s = s.cut(tri)
    return s


def body_parts():
    P = []
    # ---- 下层底盘：往里收，前后下角切大斜角
    tub_prof = [(-L2 + 4, Z0 + CH_MID), (-L2 + 4 + CH_MID, Z0), (L2 - 4 - CH_MID, Z0), (L2 - 4, Z0 + CH_MID), (L2 - 4, TUB_TOP),
                (-L2 + 4, TUB_TOP)]
    tub = prism_xz(tub_prof, -W2 + 6, W2 - 6)
    tub = soft(plan_chamfer(tub), EDGE)
    for x in (-SEAM_X, SEAM_X):                                    # 面板分缝（和上层甲板壳同一个 x，线从上到下连着）
        tub = tub.cut(box(x - SEAM / 2, x + SEAM / 2, -W2, W2, Z0 - 1, TUB_TOP + 1))
    for sy in (1, -1):                                             # 侧面检修板（凹 1.5 + 四颗螺栓）
        y = sy * (W2 - 6)
        tub = tub.cut(box(-90, 90, *sorted((y, y - sy * 1.5)), Z0 + 17, TUB_TOP - 3))
    P.append(("下层底盘", tub, "shell"))
    for sy in (1, -1):
        for x, z in ((-82, Z0 + 21), (82, Z0 + 21), (-82, TUB_TOP - 7), (82, TUB_TOP - 7)):
            P.append((f"检修板螺栓{sy}{x}{z:.0f}", cyl_y(x, z, 5, *sorted((sy * (W2 - 7.5), sy * (W2 - 6.5)))), "metal"))
    # ---- 黑腰：凹进去一圈 + 两侧青色灯条 + 前后端头
    waist = box(-L2 + 8, L2 - 8, -W2 + 10, W2 - 10, TUB_TOP, WAIST_TOP)
    P.append(("黑腰", waist, "skel_dk"))
    for sy in (1, -1):
        P.append((f"腰线灯{sy}", box(-SEAM_X + 14, SEAM_X - 14, *sorted((sy * (W2 - 10), sy * (W2 - 9.4))), TUB_TOP + 3, WAIST_TOP - 3), "cyan"))
    # ---- 上层甲板壳：前后大斜面（007v 那种往回收的装甲板），左右在 x=±120 分缝
    c = CH_BIG
    slab_prof = [(-L2 - 4 + c, WAIST_TOP), (L2 + 4 - c, WAIST_TOP), (L2 + 4, WAIST_TOP + c), (L2 + 4, TOP - c), (L2 + 4 - c, TOP),
                 (-L2 - 4 + c, TOP), (-L2 - 4, TOP - c), (-L2 - 4, WAIST_TOP + c)]
    slab = prism_xz(slab_prof, -W2 - 2, W2 + 2)
    slab = soft(plan_chamfer(slab), EDGE)
    for x in (-SEAM_X, SEAM_X):                                # 分缝只在两侧和前后，顶上留整块（货台是一体的）
        for sy in (1, -1):
            slab = slab.cut(box(x - SEAM / 2, x + SEAM / 2, *sorted((sy * (W2 - 30), sy * (W2 + 5))), WAIST_TOP - 1, TOP - 20))
    # 前脸：屏幕窗 + 两边各两道凹槽（照用户的前脸图）
    sw, sh = 124.0, 64.0                                           # 4.3 寸屏露出来的部分
    zc = WAIST_TOP + (TOP - WAIST_TOP) / 2 - 2
    HW, HH, HD = sw + 26, sh + 16, 30.0                              # 头：往前凸出的屏幕模块（宽、高、伸出来多少）
    grooves = []
    for sy in (1, -1):                                             # 两边各两道平行斜槽（45°，和机身切角统一）
        y_in, y_end = sy * (HW / 2 + 12), sy * (W2 - 14)
        ff = HH / 2 - CH_MID                                          # 头正面的半高
        for z0, z1 in ((zc + ff - 12, zc + ff), (zc - ff, zc - ff + 12)):
            h = z1 - z0
            pts = [(y_in, z0), (y_end - sy * h, z0), (y_end, z1), (y_in + sy * h, z1)]   # 平行四边形：都往外上方斜
            slab = slab.cut(prism_yz(pts, L2 - 1, L2 + 10))
            grooves.append((sy, z0, pts))
    BX0, BX1, BW, BD = -178.0, 128.0, 196.0, 14.0                    # 货台：凹下去 14
    slab = slab.cut(plan_chamfer(box(BX0, BX1, -BW / 2, BW / 2, TOP - BD, TOP + 5), 16.0))
    for x in (BX0 - 9, BX1 + 9):                                   # 前后沿的绑带孔
        for y in (-60, 0, 60):
            slab = slab.cut(soft(box(x - 3, x + 3, y - 14, y + 14, TOP - 8, TOP + 5), 1.5))
    gap = box(L2 - 6, L2 + 10, -HW / 2 - 4, HW / 2 + 4, zc - HH / 2 - 4, zc + HH / 2 + 4)
    slab = slab.cut(gap)
    P.append(("头影线缝底", box(L2 - 6, L2 - 5, -HW / 2 - 4, HW / 2 + 4, zc - HH / 2 - 4, zc + HH / 2 + 4), "skel_dk"))
    P.append(("上层甲板壳", slab, "shell"))
    P.append(("货台底垫", box(BX0 + 4, BX1 - 4, -BW / 2 + 4, BW / 2 - 4, TOP - BD, TOP - BD + 2), "rubber"))
    for k in range(9):                                             # 防滑筋
        y = -BW / 2 + 22 + k * (BW - 44) / 8
        P.append((f"货台筋{k}", soft(box(BX0 + 14, BX1 - 14, y - 2, y + 2, TOP - BD + 2, TOP - BD + 5), 0.8), "rubber"))
    for x in (BX0 - 9, BX1 + 9):
        for y in (-60, 0, 60):
            P.append((f"绑带孔底{x:.0f}{y}", box(x - 3, x + 3, y - 14, y + 14, TOP - 8, TOP - 7), "skel_dk"))
    for sy, z0, pts in grooves:                                    # 凹槽里的深色内衬
        P.append((f"凹槽内衬{sy}{z0:.0f}", prism_yz(pts, L2 - 1.5, L2 + 0.8), "skel_dk"))
    for sy in (1, -1):
        txt = cq.Workplane("XZ").text("007A", 30, 0.8, halign="center", valign="center", fontPath="C:/Windows/Fonts/bahnschrift.ttf").val()
        if sy > 0:
            txt = txt.rotate(V(0, 0, 0), V(0, 0, 1), 180).translate(V(0, W2 + 2, zc))
        else:
            txt = txt.translate(V(0, -W2 - 2, zc))
        P.append((f"字样{sy}", txt, "skel_dk"))
    # ---- 头：深色屏幕模块从前脸往前伸 30，前面切角（像头盔面罩）；两侧金属圆盖 = 抬头低头的转轴
    hx0, hx1 = L2 + 4, L2 + 4 + HD
    zb, zt = zc - HH / 2, zc + HH / 2
    prof = [(hx0 - 10, zb), (hx1 - CH_MID, zb), (hx1, zb + CH_MID), (hx1, zt - CH_MID), (hx1 - CH_MID, zt), (hx0 - 10, zt)]   # 上沿、下巴都是 45° × 18
    head = prism_xz(prof, -HW / 2, HW / 2)
    for sy in (1, -1):                                             # 前面两侧切掉一个斜角（收窄）
        tri = cq.Workplane("XY").polyline([(hx1 - 12, sy * (HW / 2 + 1)), (hx1 + 12, sy * (HW / 2 + 1)),
                                            (hx1 + 12, sy * (HW / 2 - 13))]).close().extrude(HH + 20).val()
        head = head.cut(tri.translate(V(0, 0, zb - 10)))
    head = soft(head, 1.5)
    head = head.cut(box(hx1 - 3, hx1 + 1, -sw / 2, sw / 2, zc - sh / 2, zc + sh / 2))
    P.append(("头", head, "skel_dk"))
    for sy in (1, -1):                                             # 头两侧的鳍（装转轴的“耳朵”）
        fin = prism_xz([(hx0 - 6, zc - 22), (hx0 + 14, zc - 22), (hx0 + 22, zc + 10), (hx0 + 4, zc + 26), (hx0 - 6, zc + 26)],
                       *sorted((sy * HW / 2, sy * (HW / 2 + 6))))
        P.append((f"头鳍{sy}", soft(fin, 1.2), "shell"))
    P.append(("屏幕", box(hx1 - 3, hx1 - 1.5, -sw / 2, sw / 2, zc - sh / 2, zc + sh / 2), "glass"))
    for ey in (-30, 30):
        eye = cq.Workplane("YZ").rect(30, 24).extrude(0.4).edges("|X").fillet(6).val().translate(V(hx1 - 1.4, ey, zc + 4))
        P.append((f"眼睛{ey}", eye, "eye"))
    P.append(("屏幕下沿灯", box(hx1 - 1.3, hx1 - 0.9, -40, 40, zc - sh / 2 + 8, zc - sh / 2 + 10), "cyan"))
    for sy in (1, -1):
        P.append((f"头转轴盖{sy}", soft(cyl_y(hx0 + 6, zc, 20, *sorted((sy * (HW / 2 + 6), sy * (HW / 2 + 9)))), 0.8), "metal"))
        P.append((f"头转轴光环{sy}", ring_y(hx0 + 6, zc, 28, 23, *sorted((sy * (HW / 2 + 6), sy * (HW / 2 + 6.6)))), "cyan"))
    # 激光雷达：人面对它看的左上角（= 机器人自己的右边，-y），照用户的前脸图
    lx, ly = L2 - 46, -(W2 - 50)
    P.append(("雷达座", soft(plan_chamfer(box(lx - 26, lx + 26, ly - 26, ly + 26, TOP, TOP + 10)), 1.0), "skel_dk"))
    P.append(("雷达", soft(cq.Solid.makeCylinder(20, 26, V(lx, ly, TOP + 10)), 1.5), "skel_dk"))
    P.append(("雷达窗", cq.Solid.makeCylinder(20.4, 8, V(lx, ly, TOP + 18)).cut(cq.Solid.makeCylinder(19, 8, V(lx, ly, TOP + 18))), "glass"))
    # ---- 背上：货台已经做进上层甲板壳里了（见上面）
    # ---- 尾部：尾灯、急停、充电口
    for sy in (1, -1):
        P.append((f"尾灯{sy}", box(-L2 - 4.6, -L2 - 3.9, *sorted((sy * 50, sy * 110)), zc + 14, zc + 22), "red"))
    P.append(("急停", cyl_x(0, zc, 36, -L2 - 18, -L2 - 4), "estop"))
    P.append(("急停座", cyl_x(0, zc, 44, -L2 - 6, -L2 - 3), "skel_dk"))
    P.append(("充电口", box(-L2 - 5, -L2 - 3, -52, -30, zc - 6, zc + 6), "rubber"))
    # ---- 前保险杠下方的台阶测距窗、底部腹板
    for sy in (1, -1):
        P.append((f"测距窗{sy}", box(L2 - 4, L2 - 2, sy * 70 - 8, sy * 70 + 8, Z0 + 30, Z0 + 38), "glass"))
    return P


def all_parts():
    P = body_parts()
    for fx in (1, -1):
        for sy in (1, -1):
            P += leg_parts(fx, sy)
    return P


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        os.remove(os.path.join(OUT, f))
    man = []
    for i, (n, s, c) in enumerate(all_parts()):
        fn = f"{i:03d}.stl"
        cq.exporters.export(cq.Workplane().add(s), os.path.join(OUT, fn), tolerance=0.08, angularTolerance=0.15)
        man.append(dict(file=fn, name=n, color=c))
    json.dump(man, open(os.path.join(OUT, "manifest.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=0)
    print(f"{len(man)} 个零件；站立髋离地 {HIP_Z:.0f}，机身底离地 {Z0:.0f}，整机宽 {2 * (LEG_Y + T_THIGH / 2 + 8):.0f}")
