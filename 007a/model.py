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
BODY_L, BODY_W, BODY_H = 480.0, 260.0, 160.0
L2, W2 = BODY_L / 2, BODY_W / 2
HIP_X = 185.0
MOTOR_D = 92.0                                   # 关节电机（GIM8108-8 级别）
SEAM = 2.0

# 腿：比例照用户草图（concept_v3）
H0, K0, B0, T0 = (97, 122), (307, 262), (213, 295), (155, 345)
L1 = 230.0
S = L1 / math.dist(H0, K0)
LS = math.dist(K0, T0) * S                       # 小腿（膝 → 脚尖）
STAND_KNEE = 90.0
HIP_Z = math.sqrt(L1 ** 2 + LS ** 2 - 2 * L1 * LS * math.cos(math.radians(STAND_KNEE)))
Z0 = HIP_Z - BODY_H / 2                          # 机身底离地
TUB_TOP, WAIST_TOP = Z0 + 46, Z0 + 60            # 下层底盘顶、黑腰顶（上层外壳 100 高，屏幕整块嵌进去）
TOP = Z0 + BODY_H
HUB_SK = [(-35, -33), (28, -33), (33, 38), (-4, 49), (-47, 8)]
HUB_F = (MOTOR_D / 2 + 8) / 30.0
KNEE_TOP = [(303, 234), (311, 245), (313, 262)]
KNEE_BOT = [(296, 266)]
SHANK_SK = [(310, 250), (317, 265), (311, 285), (218, 303), (160, 348), (149, 340), (208, 288), (294, 262)]
T_THIGH, T_SHANK = 38.0, 30.0
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
    hub = [_rot((-x * HUB_F, -y * HUB_F), r) for x, y in HUB_SK]

    def at_knee(pts, rr):
        return [tuple(k + d for k, d in zip(knee, _rot((sk(p, K0)[0] * S, sk(p, K0)[1] * S), rr))) for p in pts]
    thigh = hub[:2] + at_knee(KNEE_TOP, r) + at_knee(KNEE_BOT, r) + hub[2:]
    a_sh = math.atan2(*reversed(sk(T0, K0)))
    a_now = math.atan2(foot[1] - knee[1], foot[0] - knee[0])
    shank = at_knee(SHANK_SK, a_now - a_sh)
    return thigh[::-1], shank[::-1], knee, foot                  # 镜像后顺序反了，翻回逆时针


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
    th = prism_xz(thigh, yc - T_THIGH / 2, y_out)
    th = soft(th, 4.0)
    P.append(("大腿", place(th, 0), "skel"))
    cover = prism_xz_offset(thigh, -7.0, y_out - 0.5, y_out + 2.5)   # 外盖板：浅色，比骨架小一圈、凸出 2.5
    cover = cover.cut(cyl_y(0, 0, MOTOR_D + 6, y_out - 2, y_out + 4))   # 髋盘位置让出来（那里是光环和盖）
    P.append(("大腿外盖板", place(soft(cover, 1.2), 0), "shell"))
    # 髋盘外面：深色电机盖 + 青色光环 + 中心盖 + 6 颗螺栓
    P.append(("髋盖", place(cyl_y(0, 0, MOTOR_D + 2, y_out, y_out + 4), 0), "skel_dk"))
    P.append(("髋光环", place(ring_y(0, 0, MOTOR_D - 6, MOTOR_D - 12, y_out + 4, y_out + 4.6), 0), "cyan"))
    P.append(("髋中心盖", place(soft(cyl_y(0, 0, 34, y_out + 4, y_out + 8), 1.5), 0), "metal"))
    for k in range(6):
        a = math.radians(30 + 60 * k)
        P.append((f"髋螺栓{k}", place(cyl_y(30 * math.cos(a), 30 * math.sin(a), 6, y_out + 4, y_out + 5.5), 0), "metal"))
    # 小腿在大腿里侧；膝盖两面盖帽
    ys0 = yc - T_THIGH / 2 - T_SHANK - 1
    sh = soft(prism_xz(shank, ys0, ys0 + T_SHANK), 3.0)
    P.append(("小腿", place(sh, 0), "skel"))
    sc = prism_xz_offset(shank, -6.0, ys0 - 2.5, ys0 + 0.5)          # 小腿里侧（朝机身）也贴一条浅色盖板
    P.append(("小腿盖板", place(soft(sc, 1.0), 0), "shell"))
    kx, kz = knee
    P.append(("膝轴", place(cyl_y(kx, kz, 16, ys0 - 3, y_out + 3), 0), "metal"))
    P.append(("膝盖帽外", place(soft(cyl_y(kx, kz, 30, y_out, y_out + 4), 1.2), 0), "skel_dk"))
    P.append(("膝光点", place(ring_y(kx, kz, 20, 15, y_out + 4, y_out + 4.6), 0), "cyan"))
    fxp, fzp = foot
    P.append(("脚垫", place(cq.Solid.makeSphere(14, V(fxp, ys0 + T_SHANK / 2, fzp + 14)), 0), "rubber"))
    # 机身侧面的髋座（外摆电机的法兰，深色圆盘）
    P.append(("髋座", place(soft(cyl_y(0, 0, MOTOR_D + 18, W2 - 1, W2 + 7), 2.0), 0), "skel_dk"))
    tag = ("前" if fx > 0 else "后") + ("左" if sy > 0 else "右")
    return [(f"{tag}{n}", s, c) for n, s, c in P]


# ================================================================ 机身
def body_parts():
    P = []
    # ---- 下层底盘：往里收，前后下角切大斜角
    tub_prof = [(-L2 + 4, Z0 + 20), (-L2 + 24, Z0), (L2 - 24, Z0), (L2 - 4, Z0 + 20), (L2 - 4, TUB_TOP), (-L2 + 4, TUB_TOP)]
    tub = prism_xz(tub_prof, -W2 + 6, W2 - 6)
    tub = soft(tub, 3.0)
    for x in (-L2 + 120, L2 - 120):                                # 面板分缝
        tub = tub.cut(box(x - SEAM / 2, x + SEAM / 2, -W2, W2, Z0 - 1, TUB_TOP + 1))
    for sy in (1, -1):                                             # 侧面检修板（凹 1.5 + 四颗螺栓）
        y = sy * (W2 - 6)
        tub = tub.cut(box(-90, 90, *sorted((y, y - sy * 1.5)), Z0 + 10, TUB_TOP - 8))
    P.append(("下层底盘", tub, "shell"))
    for sy in (1, -1):
        for x, z in ((-82, Z0 + 15), (82, Z0 + 15), (-82, TUB_TOP - 13), (82, TUB_TOP - 13)):
            P.append((f"检修板螺栓{sy}{x}{z:.0f}", cyl_y(x, z, 5, *sorted((sy * (W2 - 7.5), sy * (W2 - 6.5)))), "metal"))
    # ---- 黑腰：凹进去一圈 + 两侧青色灯条 + 前后端头
    waist = box(-L2 + 8, L2 - 8, -W2 + 10, W2 - 10, TUB_TOP, WAIST_TOP)
    P.append(("黑腰", waist, "skel_dk"))
    for sy in (1, -1):
        for x in (-100, 100):
            P.append((f"腰线灯{sy}{x}", box(x - 60, x + 60, *sorted((sy * (W2 - 10), sy * (W2 - 9.4))), TUB_TOP + 5.5, WAIST_TOP - 5.5), "cyan"))
    # ---- 上层甲板壳：前后大斜面（007v 那种往回收的装甲板），左右在 x=±120 分缝
    slab_prof = [(-L2 + 26, WAIST_TOP), (L2 - 26, WAIST_TOP), (L2 + 4, WAIST_TOP + 24), (L2 + 4, TOP - 18), (L2 - 18, TOP),
                 (-L2 + 18, TOP), (-L2 - 4, TOP - 18), (-L2 - 4, WAIST_TOP + 24)]
    slab = prism_xz(slab_prof, -W2 - 2, W2 + 2)
    slab = soft(slab, 2.5)
    for x in (-L2 + 120, L2 - 120):
        slab = slab.cut(box(x - SEAM / 2, x + SEAM / 2, -W2 - 5, W2 + 5, WAIST_TOP - 1, TOP + 1))
    # 前脸：屏幕窗 + 两边各两道凹槽（照用户的前脸图）
    sw, sh = 124.0, 64.0                                           # 4.3 寸屏露出来的部分
    zc = WAIST_TOP + (TOP - WAIST_TOP) / 2 - 2
    slab = slab.cut(box(L2 - 14, L2 + 10, -sw / 2 - 8, sw / 2 + 8, zc - sh / 2 - 8, zc + sh / 2 + 8))
    for sy in (1, -1):
        for z0, z1 in ((zc + 12, zc + 26), (zc - 26, zc - 12)):
            slab = slab.cut(box(L2 - 4, L2 + 10, *sorted((sy * (sw / 2 + 16), sy * (W2 - 6))), z0, z1))
    P.append(("上层甲板壳", slab, "shell"))
    for sy in (1, -1):                                             # 凹槽里的深色内衬（看得出是陷进去的）
        for z0, z1 in ((zc + 12, zc + 26), (zc - 26, zc - 12)):
            P.append((f"凹槽内衬{sy}{z0:.0f}", box(L2 - 4.5, L2 - 3.5, *sorted((sy * (sw / 2 + 16), sy * (W2 - 6))), z0, z1), "skel_dk"))
    for sy in (1, -1):
        txt = cq.Workplane("XZ").text("007A", 30, 0.8, halign="center", valign="center", fontPath="C:/Windows/Fonts/bahnschrift.ttf").val()
        txt = txt.translate(V(-40, 0, 0))
        if sy > 0:
            txt = txt.rotate(V(0, 0, 0), V(0, 0, 1), 180).translate(V(0, W2 + 2, zc))
        else:
            txt = txt.translate(V(0, -W2 - 2, zc))
        P.append((f"字样{sy}", txt, "skel_dk"))
        P.append((f"甲板边线{sy}", box(-L2 + 30, L2 - 30, *sorted((sy * (W2 + 2), sy * (W2 + 2.4))), TOP - 7, TOP - 5), "metal"))
    # ---- 头：屏幕框（深色）+ 屏幕（玻璃）+ 眼睛（屏上的图案）
    bez = box(L2 - 14, L2 - 2, -sw / 2 - 7, sw / 2 + 7, zc - sh / 2 - 7, zc + sh / 2 + 7)
    bez = bez.cut(box(L2 - 15, L2 - 1, -sw / 2, sw / 2, zc - sh / 2, zc + sh / 2))
    P.append(("屏幕框", soft(bez, 1.5), "skel_dk"))
    P.append(("屏幕", box(L2 - 6, L2 - 4, -sw / 2, sw / 2, zc - sh / 2, zc + sh / 2), "glass"))
    for ey in (-30, 30):
        eye = cq.Workplane("YZ").rect(30, 24).extrude(0.4).edges("|X").fillet(6).val().translate(V(L2 - 3.9, ey, zc + 4))
        P.append((f"眼睛{ey}", eye, "eye"))
    P.append(("屏幕下沿灯", box(L2 - 3.8, L2 - 3.4, -40, 40, zc - sh / 2 + 8, zc - sh / 2 + 10), "cyan"))
    # 激光雷达：人面对它看的左上角（= 机器人自己的右边，-y），照用户的前脸图
    lx, ly = L2 - 46, -(W2 - 40)
    P.append(("雷达座", soft(box(lx - 26, lx + 26, ly - 26, ly + 26, TOP, TOP + 10), 2.0), "skel_dk"))
    P.append(("雷达", soft(cq.Solid.makeCylinder(20, 26, V(lx, ly, TOP + 10)), 1.5), "skel_dk"))
    P.append(("雷达窗", cq.Solid.makeCylinder(20.4, 8, V(lx, ly, TOP + 18)).cut(cq.Solid.makeCylinder(19, 8, V(lx, ly, TOP + 18))), "glass"))
    # ---- 背上：两条货物导轨 + 防滑垫
    for sy in (1, -1):
        P.append((f"货物导轨{sy}", soft(box(-170, 120, sy * 85 - 10, sy * 85 + 10, TOP, TOP + 12), 1.5), "metal"))
    P.append(("防滑垫", soft(box(-160, 110, -70, 70, TOP, TOP + 3), 1.0), "rubber"))
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
