"""007A 方案草图 v2 + 关节扭矩估算。

改动（用户 2026-10-28 的批注）：
- 大腿根部加宽，和髋盘做成一个整体（从髋盘两边直接收到膝）；
- 载重 5 kg；机身稍微宽一点；外形照 007v 的感觉（上层甲板壳前后大斜面、中间一圈凹进去的黑腰、下层底盘往里收），工业设计感不变；
- 站高：腿的比例照用户草图，站立时膝盖弯 90° 左右（草图那个姿势太蹲，机身底才离地 128）。

用法：python 007a/concept/concept_v2.py → 007a/concept/007A_方案草图_v2.png（并打印扭矩表）
"""
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle, Circle  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
plt.rcParams["axes.unicode_minus"] = False
HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81

# ---------------------------------------------------------------- 腿（比例照用户草图）
H0, K0, B0, T0 = (97, 122), (307, 262), (213, 295), (155, 345)     # 草图里的 髋 膝 折弯 脚尖（像素）
L1 = 230.0                                                          # 大腿
S = L1 / math.dist(H0, K0)
L2 = math.dist(K0, T0) * S                                          # 小腿（膝 → 脚尖的直线距离）
BEND = [((p[0] - K0[0]) * S, -(p[1] - K0[1]) * S) for p in (B0, T0)]   # 小腿形状（以膝为原点，草图方向）
MOTOR_D = 92.0                                                      # GIM8108-8
HUB_R = MOTOR_D / 2 + 14                                            # 髋盘半径


def leg(hip, foot):
    """髋、脚尖位置 → 膝的位置（膝朝后 = +x 一侧）。"""
    dx, dz = foot[0] - hip[0], foot[1] - hip[1]
    d = math.hypot(dx, dz)
    a = math.acos(max(-1, min(1, (L1 ** 2 + d ** 2 - L2 ** 2) / (2 * L1 * d))))
    base = math.atan2(dz, dx)
    ang = base + a                                                  # 膝在髋→脚连线的后面
    return hip[0] + L1 * math.cos(ang), hip[1] + L1 * math.sin(ang)


def shank_pts(knee, foot):
    """按草图的小腿形状（带折弯），转到 膝→脚尖 方向上。"""
    bx, bz = BEND[0]
    tx, tz = BEND[1]
    a0 = math.atan2(tz, tx)
    a1 = math.atan2(foot[1] - knee[1], foot[0] - knee[0])
    r = a1 - a0
    c, s = math.cos(r), math.sin(r)
    return [knee, (knee[0] + bx * c - bz * s, knee[1] + bx * s + bz * c), foot]


# ---------------------------------------------------------------- 站姿
STAND_KNEE = 90.0                                                   # 站立时膝盖夹角
D_STAND = math.sqrt(L1 ** 2 + L2 ** 2 - 2 * L1 * L2 * math.cos(math.radians(STAND_KNEE)))
HIP_Z = D_STAND                                                     # 脚尖在髋正下方
BODY_L, BODY_W, BODY_H = 480.0, 260.0, 160.0
HIP_X = 185.0
BODY_Z0 = HIP_Z - BODY_H / 2
LEG_OFF = BODY_W / 2 + 34                                           # 腿的平面离中线（外摆电机在机身里，腿在外面）

# ---------------------------------------------------------------- 重量（估算）
MASS = [("12 个关节电机（GIM8108-8，含驱动 0.40 kg）", 12 * 0.40), ("腿：大腿、小腿、髋盘、连杆、脚垫", 4 * 0.35),
        ("机身骨架（铝板、型材、外摆电机座）", 3.0), ("电池 6S2P", 1.4), ("头（屏幕、舵机、传感器）+ 雷达", 0.9),
        ("树莓派、主控、驱动、线", 0.9), ("外壳（打印）", 1.2)]
M_EMPTY = sum(m for _, m in MASS)
PAYLOAD = 5.0
M_FULL = M_EMPTY + PAYLOAD


def torques():
    """满载：① 站着（四条腿分担）② 小跑（两条腿撑地，动态系数 1.8）③ 从趴着站起来（膝最吃力）。"""
    out = []
    hip, foot = (0.0, HIP_Z), (0.0, 0.0)
    knee = leg(hip, foot)
    arm_knee = abs(knee[0] - foot[0]) / 1000
    F4 = M_FULL * G / 4
    F2 = M_FULL * G / 2 * 1.8
    out.append(("站着", F4 * arm_knee, F4 * 0.0, F4 * (LEG_OFF - BODY_W / 2 + 10) / 1000))
    out.append(("小跑（两腿撑地 ×1.8）", F2 * arm_knee, F2 * 0.08, F2 * 0.05))   # 髋：脚前后 ±8 cm；外摆：侧向 5 cm
    hc = (0.0, 150.0)                                                # 蹲低到髋离地 150 再站起来
    kc = leg(hc, (0.0, 0.0))
    out.append(("从趴着站起来（髋离地 150）", F4 * abs(kc[0]) / 1000, 0.0, 0.0))
    return out, knee, kc


def hexagon(cx, cz, r, rot=0):
    return [(cx + r * math.cos(math.radians(rot + 30 + 60 * k)), cz + r * math.sin(math.radians(rot + 30 + 60 * k))) for k in range(6)]


def draw_leg(ax, hip, foot, col="white", lw=1.6, fill="#15191f"):
    knee = leg(hip, foot)
    ang = math.atan2(knee[1] - hip[1], knee[0] - hip[0])
    nx, nz = -math.sin(ang), math.cos(ang)
    hexp = hexagon(*hip, HUB_R, math.degrees(ang))
    # 大腿和髋盘一体：从髋盘两侧（垂直于大腿方向）直接收到膝
    w1 = 22
    thigh = [(hip[0] + nx * HUB_R * 0.92, hip[1] + nz * HUB_R * 0.92), (knee[0] + nx * w1 / 2, knee[1] + nz * w1 / 2),
             (knee[0] - nx * w1 / 2, knee[1] - nz * w1 / 2), (hip[0] - nx * HUB_R * 0.92, hip[1] - nz * HUB_R * 0.92)]
    ax.add_patch(Polygon(thigh, closed=True, fc=fill, ec=col, lw=lw, zorder=2))
    ax.add_patch(Polygon(hexp, closed=True, fc=fill, ec=col, lw=lw, zorder=3))
    ax.add_patch(Circle(hip, MOTOR_D / 2, fill=False, ec="#8a96a8", lw=lw * 0.7, zorder=4))
    ax.add_patch(Circle(hip, 10, fill=False, ec="#7fd0ff", lw=1.2, zorder=4))
    ax.add_patch(Circle(knee, 13, fc=fill, ec=col, lw=lw, zorder=5))
    ax.add_patch(Circle(knee, 3.5, color=col, zorder=6))
    sp = shank_pts(knee, foot)
    xs, zs = [p[0] for p in sp], [p[1] for p in sp]
    ax.plot(xs, zs, color=col, lw=lw * 6, solid_capstyle="round", zorder=1)
    ax.plot(xs, zs, color=fill, lw=lw * 3.4, solid_capstyle="round", zorder=1)
    ax.add_patch(Circle(foot, 9, color="#222", ec=col, lw=1.2, zorder=6))    # 橡胶脚垫
    return knee


def body_side(ax):
    """007v 的感觉：下层底盘往里收、中间一圈凹进去的黑腰、上层甲板壳前后大斜面。"""
    L2, z0, h = BODY_L / 2, BODY_Z0, BODY_H
    tub_top, waist_top = z0 + h * 0.40, z0 + h * 0.52
    tub = [(-L2 + 14, z0), (L2 - 14, z0), (L2, z0 + 14), (L2, tub_top), (-L2, tub_top), (-L2, z0 + 14)]
    ax.add_patch(Polygon(tub, closed=True, fc="#d9dbdd", ec="white", lw=1.4, zorder=1))
    ax.add_patch(Rectangle((-L2 + 4, tub_top), BODY_L - 8, waist_top - tub_top, fc="#2a2e33", ec="white", lw=1.0, zorder=1))
    for x in (-150, 150):                                            # 腰线灯
        ax.add_patch(Rectangle((x - 50, (tub_top + waist_top) / 2 - 2), 100, 4, color="#00bfff", zorder=2))
    top = z0 + h
    slab = [(-L2 + 34, waist_top), (L2 - 30, waist_top), (L2 + 6, waist_top + 26), (L2 + 6, top - 22), (L2 - 16, top),
            (-L2 + 16, top), (-L2 - 6, top - 22), (-L2 - 6, waist_top + 26)]
    ax.add_patch(Polygon(slab, closed=True, fc="#d9dbdd", ec="white", lw=1.4, zorder=1))
    ax.text(30, (waist_top + top) / 2, "007A", color="#2a2e33", fontsize=22, ha="center", va="center", zorder=2, fontweight="bold")
    ax.add_patch(Rectangle((L2 - 4, waist_top + 34), 6, 16, color="#ff2a1a", zorder=2))      # 尾灯
    # 前脸：屏幕嵌在机身前面（侧面看是一道凹进去的窗）
    ax.add_patch(Rectangle((-L2 - 6, z0 + h * 0.46), 10, h * 0.42, fc="#0b0f14", ec="#7fd0ff", lw=1.2, zorder=2))
    ax.add_patch(Rectangle((-L2 + 20, top), 46, 38, fc="#2a2e33", ec="white", lw=1.2, zorder=1))
    ax.text(-L2 + 43, top + 19, "雷达", color="white", fontsize=8, ha="center", va="center")


def body_front(ax):
    W2, z0, h = BODY_W / 2, BODY_Z0, BODY_H
    tub_top, waist_top = z0 + h * 0.40, z0 + h * 0.52
    ax.add_patch(Polygon([(-W2 + 14, z0), (W2 - 14, z0), (W2, z0 + 14), (W2, tub_top), (-W2, tub_top), (-W2, z0 + 14)],
                         closed=True, fc="#d9dbdd", ec="white", lw=1.4))
    ax.add_patch(Rectangle((-W2 + 4, tub_top), BODY_W - 8, waist_top - tub_top, fc="#2a2e33", ec="white", lw=1.0))
    top = z0 + h
    ax.add_patch(Rectangle((-W2 - 4, waist_top), BODY_W + 8, top - waist_top, fc="#d9dbdd", ec="white", lw=1.4))
    sw, sh = 124.0, 80.0                                             # 屏幕窗（嵌进去）
    sz0 = waist_top + (top - waist_top - sh) / 2 - 18
    for sx in (-1, 1):                                               # 两边各两道凹槽
        x0, x1 = sx * (sw / 2 + 8), sx * (W2 + 4)
        for zz0, zz1 in ((sz0 + sh * 0.62, sz0 + sh + 4), (sz0 - 4, sz0 + sh * 0.32)):
            ax.add_patch(Rectangle((min(x0, x1), zz0), abs(x1 - x0), zz1 - zz0, fc="#2a2e33", ec="white", lw=0.8))
    ax.add_patch(Rectangle((-sw / 2 - 8, sz0 - 8), sw + 16, sh + 16, fc="#2a2e33", ec="white", lw=1.2))
    ax.add_patch(Rectangle((-sw / 2, sz0), sw, sh, fc="#0b0f14", ec="#7fd0ff", lw=1.0))
    for ex in (-28, 28):                                             # 屏幕上的眼睛
        ax.add_patch(Rectangle((ex - 14, sz0 + sh / 2 - 10), 28, 24, color="#78c8ff"))
    ax.add_patch(Rectangle((-W2 + 12, top), 46, 38, fc="#2a2e33", ec="white", lw=1.2))
    ax.text(-W2 + 35, top + 19, "雷达", color="white", fontsize=8, ha="center", va="center")
    for sx in (-1, 1):                                               # 腿（正面看）
        x = sx * LEG_OFF
        ax.add_patch(Rectangle((x - 22, HIP_Z - HUB_R), 44, 2 * HUB_R, fc="#15191f", ec="white", lw=1.2))
        ax.plot([x, x], [HIP_Z - HUB_R, 12], color="white", lw=8)
        ax.plot([x, x], [HIP_Z - HUB_R, 12], color="#15191f", lw=5)
        ax.add_patch(Circle((x, 9), 9, color="#222", ec="white", lw=1.0))
        ax.add_patch(Rectangle((sx * (W2 - 44) - 30, HIP_Z - 46), 60, 92, fill=False, ec="#8a96a8", lw=0.9, ls="--"))
    ax.text(W2 - 44 - 30, HIP_Z - 60, "外摆电机\n（在机身里）", color="#8a96a8", fontsize=7, ha="center", va="top")


def dim(ax, p0, p1, text, off=(0, 0), col="#7fd0ff"):
    ax.annotate("", xy=p1, xytext=p0, arrowprops=dict(arrowstyle="<->", color=col, lw=0.9))
    ax.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], text, color=col, fontsize=9, ha="center", va="center")


def main():
    rows, knee, kc = torques()
    fig = plt.figure(figsize=(17, 10.2), facecolor="black")
    fig.suptitle("007A 方案草图 v2（等你确认）", color="white", fontsize=16, x=0.02, ha="left", y=0.975)
    ax = fig.add_axes([0.01, 0.06, 0.58, 0.86], facecolor="black")
    ax.set_aspect("equal")
    ax.axis("off")
    ax.plot([-470, 470], [0, 0], color="#555", lw=1)
    for hx in (-HIP_X, HIP_X):                                       # 远侧的腿（暗）
        draw_leg(ax, (hx + 6, HIP_Z + 4), (hx + 6, 0), col="#555", fill="#0d0f12")
    body_side(ax)
    for hx in (-HIP_X, HIP_X):
        draw_leg(ax, (hx, HIP_Z), (hx, 0))
    ax.text(-BODY_L / 2 - 30, 20, "← 前", color="white", fontsize=11, ha="right")
    dim(ax, (-BODY_L / 2, BODY_Z0 + BODY_H + 70), (BODY_L / 2, BODY_Z0 + BODY_H + 70), f"机身长 {BODY_L:.0f}", off=(0, 11))
    dim(ax, (BODY_L / 2 + 50, 0), (BODY_L / 2 + 50, BODY_Z0), f"离地\n{BODY_Z0:.0f}", off=(32, 0))
    dim(ax, (BODY_L / 2 + 50, BODY_Z0), (BODY_L / 2 + 50, BODY_Z0 + BODY_H), f"高\n{BODY_H:.0f}", off=(32, 0))
    dim(ax, (-HIP_X, -22), (HIP_X, -22), f"前后髋间距 {2 * HIP_X:.0f}", off=(0, -10))
    ax.set_xlim(-470, 470)
    ax.set_ylim(-50, BODY_Z0 + BODY_H + 110)
    ax.set_title(f"侧视：站立（膝弯 {STAND_KNEE:.0f}°，髋离地 {HIP_Z:.0f}）", color="white", fontsize=12, loc="left")

    bx = fig.add_axes([0.60, 0.44, 0.39, 0.48], facecolor="black")
    bx.set_aspect("equal")
    bx.axis("off")
    body_front(bx)
    bx.plot([-LEG_OFF - 60, LEG_OFF + 60], [0, 0], color="#555", lw=1)
    dim(bx, (-BODY_W / 2, BODY_Z0 - 16), (BODY_W / 2, BODY_Z0 - 16), f"机身宽 {BODY_W:.0f}", off=(0, -9))
    dim(bx, (-LEG_OFF - 22, -18), (LEG_OFF + 22, -18), f"整机宽 {2 * LEG_OFF + 44:.0f}", off=(0, -9))
    bx.set_xlim(-LEG_OFF - 70, LEG_OFF + 70)
    bx.set_ylim(-40, BODY_Z0 + BODY_H + 50)
    bx.set_title("正视：头嵌在机身前面，屏幕抬头低头", color="white", fontsize=12, loc="left")

    lines = [f"重量（估算）：空车 {M_EMPTY:.1f} kg + 载重 {PAYLOAD:.0f} kg = {M_FULL:.1f} kg"]
    lines += [f"   {n}  {m:.1f}" for n, m in MASS]
    lines += ["", "关节扭矩（满载，每条腿）    膝 / 髋前后 / 髋外摆 N·m"]
    lines += [f"   {n}：{k:.1f} / {h:.1f} / {a:.1f}" for n, k, h, a in rows]
    peak = max(r[1] for r in rows)
    lines += ["", f"GIM8108-8：额定 7.5、堵转 22 N·m（经销商数据，待核对）",
              f"→ 膝最大要 {peak:.1f} N·m：直接用会顶到堵转。",
              "   建议膝用 1.5:1 的连杆（大腿里的连杆本来就要做），",
              f"   膝电机只要出 {peak / 1.5:.1f} N·m；髋和外摆直接用够。"]
    fig.text(0.61, 0.40, "\n".join(lines), color="white", fontsize=8.8, va="top", linespacing=1.38)
    out = os.path.join(HERE, "007A_方案草图_v2.png")
    fig.savefig(out, dpi=110, facecolor="black")
    print(out)
    print(f"大腿 {L1:.0f} 小腿 {L2:.0f}；站立膝 {STAND_KNEE:.0f}° 髋离地 {HIP_Z:.0f} 机身离地 {BODY_Z0:.0f}；空车 {M_EMPTY:.1f} 满载 {M_FULL:.1f}")
    for r in rows:
        print(r)


if __name__ == "__main__":
    main()
