"""007A 第 2 步：12 个电机怎么摆——两个方案的俯视 + 髋处横截面草图（真实电机尺寸）。

电机 宇树 GO-M8010-6：Φ96.5 × 42.3。每条腿 3 个：外摆（轴朝前后，在机身里）、髋前后摆（轴朝左右）、膝（轴朝左右，和髋同轴，同步带到膝）。
用法：python 007a/layout_options.py → docs/机械狗007A/电机布置方案.png
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "docs", "机械狗007A", "电机布置方案.png")

D, T = 96.5, 42.3          # 电机直径、厚
L2, W2 = 240.0, 130.0      # 机身半长、半宽
HX = 160.0                 # 髋 x
TH = 34.0                  # 大腿厚（同步带在里面）
C_SHELL, C_MOT, C_HAA, C_THIGH, C_BAT, C_EL = "#e8eaec", "#3d4247", "#6b7680", "#a3a9af", "#f2c14e", "#7fb3d5"


def body(ax, w2=W2, cavity=None):
    ax.add_patch(Rectangle((-L2, -w2), 2 * L2, 2 * w2, fc=C_SHELL, ec="k", lw=1.2))
    if cavity:
        for fx in (1, -1):
            for sy in (1, -1):
                x0, y0, sx, syy = cavity(fx, sy)
                ax.add_patch(Rectangle((x0, y0), sx, syy, fc="white", ec="k", lw=0.8, ls="--"))


def top(ax, title, y_knee, y_hip, y_thigh, haa_x, cavity=None, note=""):
    body(ax, cavity=cavity)
    for fx in (1, -1):
        for sy in (1, -1):
            x = fx * HX
            for yc, c, lab in ((y_knee, C_MOT, "膝"), (y_hip, C_MOT, "髋")):
                ax.add_patch(Rectangle((x - D / 2, sy * yc - T / 2), D, T, fc=c, ec="k", lw=0.8))
                ax.text(x, sy * yc, lab, color="w", ha="center", va="center", fontsize=8)
            ax.add_patch(Rectangle((x - 20, sy * y_thigh - TH / 2), 40, TH, fc=C_THIGH, ec="k", lw=0.8))
            hx = fx * haa_x
            ax.add_patch(Rectangle((hx - T / 2, sy * 70 - D / 2), T, D, fc=C_HAA, ec="k", lw=0.8))
            ax.text(hx, sy * 70, "外摆", color="w", ha="center", va="center", fontsize=8, rotation=90)
    xin = haa_x - T / 2
    ax.add_patch(Rectangle((-xin + 4, -W2 + 8), 2 * xin - 8, 2 * W2 - 16, fc=C_BAT, ec="k", lw=0.8, alpha=0.9))
    ax.text(0, 0, f"电池 + 电子件\n{2 * xin - 8:.0f} × {2 * W2 - 16:.0f}（可用）", ha="center", va="center", fontsize=9)
    w = y_thigh + TH / 2 + 8
    ax.annotate("", xy=(L2 + 40, w), xytext=(L2 + 40, -w), arrowprops=dict(arrowstyle="<->"))
    ax.text(L2 + 48, 0, f"整机宽\n约 {2 * w:.0f}", va="center", fontsize=10)
    ax.set_title(title, fontsize=12)
    ax.text(-L2, -w - 45, note, fontsize=9, va="top")
    ax.set_xlim(-L2 - 30, L2 + 120)
    ax.set_ylim(-w - 130, w + 20)
    ax.set_aspect("equal")
    ax.axis("off")


fig, axs = plt.subplots(1, 2, figsize=(16, 7.5))
# 方案 A：两个电机都在机身外面（宇树 Go2 那种），机身不动
yk = W2 + 6 + T / 2
yh = yk + T
yt = yh + T / 2 + TH / 2
top(axs[0], "方案 A：髋、膝两个电机都在机身外面（像宇树 Go2）", yk, yh, yt, HX - D / 2 - T / 2 - 4,
    note="· 机身里空间最大，外摆电机好放\n· 每个髋外面鼓出两个电机（约 85 厚），整机变宽很多，外观要大改\n· 腿离机身远，走路左右晃得厉害")
# 方案 B：膝电机藏进机身侧面的窝里，髋电机在髋盘里（现在的外观）
yt_b = 155.0
yh_b = yt_b - TH / 2 - T / 2 + 8           # 髋电机一半埋在髋盘（大腿根）里
yk_b = yh_b - T
top(axs[1], "方案 B：膝电机藏进机身侧面的窝（推荐）", yk_b, yh_b, yt_b, HX - D / 2 - T / 2 - 4,
    cavity=lambda fx, sy: (fx * HX - 60, (W2 - 72) if sy > 0 else (-W2 + 72), 120, 72 if sy > 0 else -72),
    note="· 外观基本不变：整机宽不变，髋盘里就是髋电机\n· 机身侧面四个髋的位置往里挖一个窝（虚线）放膝电机，外摆时有让位\n· 中间放电池和电子件的地方小一些")
plt.tight_layout()
os.makedirs(os.path.dirname(OUT), exist_ok=True)
plt.savefig(OUT, dpi=110)
print(OUT, f"A 宽 {2 * (yt + TH / 2 + 8):.0f}  B 宽 {2 * (yt_b + TH / 2 + 8):.0f}  B 膝电机 y {yk_b - T / 2:.0f}..{yk_b + T / 2:.0f}")
