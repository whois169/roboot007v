"""007A 方案草图 v1：照用户画的腿（侧视）和前脸（正视）定比例，出一张侧视 + 正视 + 腿的尺寸图。

腿：从用户草图量出来（髋 → 膝 → 脚踝折弯 → 脚尖），按大腿 230 mm 缩放；膝朝后，四条腿一样。
前脸：头嵌在机身前面：中间可以上下翻（抬头低头）的屏幕，两边各两道凹槽，左上角激光雷达。
用法：python 007a/concept/concept_v1.py  → 007a/concept/007A_方案草图_v1.png
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

# ---- 腿：用户草图里的关节点（像素），髋 H、膝 K、脚踝折弯 B、脚尖 T
H, K, B, T = (97, 122), (307, 262), (213, 295), (155, 345)
THIGH = 230.0
S = THIGH / math.dist(H, K)                         # 每像素多少 mm


def leg_pts(hx, hz):
    """草图坐标 → 侧视坐标（x 往前为负：机器人朝左走，膝朝后）。返回 髋 膝 折弯 脚尖。"""
    def f(p):
        return hx + (p[0] - H[0]) * S, hz - (p[1] - H[1]) * S
    return [f(H), f(K), f(B), f(T)]


SHANK1, SHANK2 = math.dist(K, B) * S, math.dist(B, T) * S
HIP_DROP = (T[1] - H[1]) * S                        # 髋到脚尖的高度（站立）

# ---- 机身
BODY_L, BODY_W, BODY_H = 460.0, 220.0, 150.0
HIP_X = 175.0                                       # 髋轴离机身中心（前后）
HIP_Z = HIP_DROP                                    # 髋轴离地 = 站立高度
BODY_Z0 = HIP_Z - BODY_H / 2                        # 机身底离地
MOTOR_D = 92.0                                      # 髋 / 膝电机（GIM8108-8 级别）直径
LEG_Y = BODY_W / 2 + 30                             # 腿的平面离中线


def hexagon(cx, cz, r):
    return [(cx + r * math.cos(math.radians(30 + 60 * k)), cz + r * math.sin(math.radians(30 + 60 * k))) for k in range(6)]


def draw_leg(ax, hx, hz, col="white", lw=1.6):
    h, k, b, t = leg_pts(hx, hz)
    ax.add_patch(Polygon(hexagon(*h, MOTOR_D / 2 + 12), closed=True, fill=False, ec=col, lw=lw))
    ax.add_patch(Circle(h, MOTOR_D / 2, fill=False, ec=col, lw=lw * 0.8))
    w0, w1 = 46, 18                                  # 大腿：髋这头宽、膝那头窄
    ang = math.atan2(k[1] - h[1], k[0] - h[0])
    nx, nz = -math.sin(ang), math.cos(ang)
    ax.add_patch(Polygon([(h[0] + nx * w0 / 2, h[1] + nz * w0 / 2), (k[0] + nx * w1 / 2, k[1] + nz * w1 / 2),
                          (k[0] - nx * w1 / 2, k[1] - nz * w1 / 2), (h[0] - nx * w0 / 2, h[1] - nz * w0 / 2)],
                         closed=True, fill=False, ec=col, lw=lw))
    ax.add_patch(Circle(k, 11, fill=False, ec=col, lw=lw))
    ax.add_patch(Circle(k, 3, fill=True, color=col))
    ax.plot([k[0], b[0], t[0]], [k[1], b[1], t[1]], color=col, lw=lw * 5, solid_capstyle="round")
    ax.plot([k[0], b[0], t[0]], [k[1], b[1], t[1]], color="black", lw=lw * 2.4, solid_capstyle="round")
    return h, k, b, t


def dim(ax, p0, p1, text, off=(0, 0), col="#7fd0ff"):
    ax.annotate("", xy=p1, xytext=p0, arrowprops=dict(arrowstyle="<->", color=col, lw=0.9))
    ax.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], text, color=col, fontsize=9, ha="center", va="center")


fig = plt.figure(figsize=(16, 9.2), facecolor="black")
fig.suptitle("007A 方案草图 v1（等你确认）", color="white", fontsize=16, x=0.02, ha="left", y=0.97)

# ================================================================ 侧视
ax = fig.add_axes([0.02, 0.08, 0.6, 0.82], facecolor="black")
ax.set_aspect("equal")
ax.axis("off")
ax.plot([-420, 420], [0, 0], color="#555", lw=1)
ax.add_patch(Rectangle((-BODY_L / 2, BODY_Z0), BODY_L, BODY_H, fill=False, ec="white", lw=1.8))
# 前脸（左边是前）：屏幕嵌在机身前端
ax.add_patch(Rectangle((-BODY_L / 2 - 2, BODY_Z0 + 40), 14, 70, fill=False, ec="#7fd0ff", lw=1.2))
ax.text(-BODY_L / 2 - 8, BODY_Z0 + 120, "屏幕（抬头低头）", color="#7fd0ff", fontsize=9, ha="right")
ax.add_patch(Rectangle((-BODY_L / 2 + 15, BODY_Z0 + BODY_H), 40, 42, fill=False, ec="white", lw=1.2))
ax.text(-BODY_L / 2 + 35, BODY_Z0 + BODY_H + 21, "雷达", color="white", fontsize=9, ha="center", va="center")
for hx in (-HIP_X, HIP_X):
    draw_leg(ax, hx, HIP_Z)
ax.text(-BODY_L / 2 - 30, 25, "← 前", color="white", fontsize=11, ha="right")
dim(ax, (-BODY_L / 2, BODY_Z0 + BODY_H + 70), (BODY_L / 2, BODY_Z0 + BODY_H + 70), f"机身长 {BODY_L:.0f}", off=(0, 10))
dim(ax, (BODY_L / 2 + 40, 0), (BODY_L / 2 + 40, BODY_Z0), f"离地\n{BODY_Z0:.0f}", off=(30, 0))
dim(ax, (BODY_L / 2 + 40, BODY_Z0), (BODY_L / 2 + 40, BODY_Z0 + BODY_H), f"高\n{BODY_H:.0f}", off=(30, 0))
dim(ax, (-HIP_X, BODY_Z0 - 25), (HIP_X, BODY_Z0 - 25), f"前后髋间距 {2 * HIP_X:.0f}", off=(0, -9))
ax.set_xlim(-430, 430)
ax.set_ylim(-40, 420)
ax.set_title("侧视（朝左走，膝朝后，四条腿一样）", color="white", fontsize=12, loc="left")

# ================================================================ 正视（照用户的前脸）
bx = fig.add_axes([0.63, 0.47, 0.36, 0.43], facecolor="black")
bx.set_aspect("equal")
bx.axis("off")
W2 = BODY_W / 2
bx.add_patch(Rectangle((-W2, BODY_Z0), BODY_W, BODY_H, fill=False, ec="white", lw=1.6))
sw, sh = 120.0, 76.0                                  # 4.3 寸屏的显示区
sz0 = BODY_Z0 + BODY_H - sh - 18
bx.add_patch(Rectangle((-sw / 2 - 6, sz0 - 6), sw + 12, sh + 12, fill=False, ec="white", lw=1.4))
bx.add_patch(Rectangle((-sw / 2, sz0), sw, sh, fill=True, color="#0e1a26"))
bx.text(0, sz0 + sh / 2, "屏幕\n（抬头低头）", color="#7fd0ff", fontsize=9, ha="center", va="center")
for sx in (-1, 1):                                    # 两边各两道凹槽（和原来机身差不多）
    x0 = sx * (sw / 2 + 6)
    x1 = sx * W2
    for z0, z1 in ((sz0 + sh * 0.62, sz0 + sh + 6), (sz0 - 6, sz0 + sh * 0.3)):
        bx.add_patch(Rectangle((min(x0, x1), z0), abs(x1 - x0), z1 - z0, fill=False, ec="white", hatch="////", lw=1.0))
bx.add_patch(Rectangle((-W2 + 10, BODY_Z0 + BODY_H), 40, 42, fill=False, ec="white", lw=1.2))
bx.text(-W2 + 30, BODY_Z0 + BODY_H + 21, "雷达", color="white", fontsize=9, ha="center", va="center")
for sx in (-1, 1):                                    # 正视里的腿（髋外摆电机在机身里，腿在两边）
    bx.add_patch(Rectangle((sx * W2 if sx > 0 else -W2 - 50, HIP_Z - 50), 50, 100, fill=False, ec="#999", lw=1.0))
    bx.plot([sx * (W2 + 25)] * 2, [HIP_Z - 50, 0], color="#999", lw=6)
    bx.plot([sx * (W2 + 25)] * 2, [HIP_Z - 50, 0], color="black", lw=3)
bx.plot([-W2 - 90, W2 + 90], [0, 0], color="#555", lw=1)
dim(bx, (-W2, BODY_Z0 - 20), (W2, BODY_Z0 - 20), f"机身宽 {BODY_W:.0f}", off=(0, -9))
bx.set_xlim(-W2 - 100, W2 + 100)
bx.set_ylim(-20, BODY_Z0 + BODY_H + 60)
bx.set_title("正视（前脸）", color="white", fontsize=12, loc="left")

# ================================================================ 腿的尺寸
cx = fig.add_axes([0.63, 0.06, 0.18, 0.38], facecolor="black")
cx.set_aspect("equal")
cx.axis("off")
h, k, b, t = draw_leg(cx, 0, 0)
dim(cx, h, k, f"大腿 {THIGH:.0f}", off=(18, 18))
dim(cx, k, b, f"{SHANK1:.0f}", off=(10, -14))
dim(cx, b, t, f"{SHANK2:.0f}", off=(-22, -2))
cx.set_xlim(-70, 230)
cx.set_ylim(-230, 70)
cx.set_title("腿（照你的草图量的）", color="white", fontsize=11, loc="left")

notes = [
    "初步数字（都能改）：",
    f"· 机身 {BODY_L:.0f} × {BODY_W:.0f} × {BODY_H:.0f} mm，站立时离地 {BODY_Z0:.0f}",
    f"· 大腿 {THIGH:.0f}，小腿 {SHANK1:.0f} + 折弯 {SHANK2:.0f}（比例照草图）",
    f"· 站立时髋轴离地 {HIP_Z:.0f}",
    "· 每条腿 3 个电机：髋外摆（在机身里）",
    "  + 髋前后摆、膝（两个都在六边形髋壳里，",
    "  膝用大腿里的连杆带动）",
    "· 电机候选：SteadyWin GIM8108-8",
    "  Φ92，约 0.4 kg，堵转 22 N·m",
    "  （和轮子电机同一套 CAN 协议）",
    "· 头嵌在机身前面，屏幕抬头低头",
]
fig.text(0.825, 0.42, "\n".join(notes), color="white", fontsize=10, va="top", linespacing=1.6)
out = os.path.join(HERE, "007A_方案草图_v1.png")
fig.savefig(out, dpi=110, facecolor="black")
print(out, f"大腿 {THIGH:.0f} 小腿 {SHANK1:.0f}+{SHANK2:.0f} 髋离地 {HIP_Z:.0f} 机身离地 {BODY_Z0:.0f}")
