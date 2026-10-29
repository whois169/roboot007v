"""007A 走线图（俯视示意）：每条线从哪来、到哪去、从哪个孔过、在哪里留活动余量。

用法：python 007a/routing.py → docs/机械狗007A/走线图.png（说明在 docs/机械狗007A/走线说明.md）
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

import eng_body as B
import model as M

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
L2, W2 = M.L2, M.W2
C = dict(power="#d62728", bus="#ff7f0e", usb="#1f77b4", sig="#2ca02c")

fig, ax = plt.subplots(figsize=(16, 9))
ax.add_patch(Rectangle((-L2, -W2), 2 * L2, 2 * W2, fc="#f4f5f6", ec="k"))
for fx in (1, -1):
    ax.add_patch(Rectangle((min(fx * B.XB, fx * (B.XB - B.T_BH)), -W2 + 14), B.T_BH, 2 * W2 - 28, fc="#6b7680"))
    for sy in (1, -1):
        hx, hy = fx * M.HIP_X, sy * 100
        ax.add_patch(Circle((hx, hy), 48, fc="#3d4247", alpha=0.25))
        ax.text(hx, hy + sy * 2, "髋 / 膝电机\n（绕线环）", ha="center", va="center", fontsize=8)
        ax.add_patch(Rectangle((fx * (B.XB + 21), sy * 70 - 48), 1, 96, fc="none"))
        ax.add_patch(Circle((fx * (B.XB + 21), sy * 70), 6, fc=C["bus"]))
        ax.text(fx * (B.XB + 30), sy * 70, "外摆电机", fontsize=7, ha="left" if fx > 0 else "right")
# 中间
ax.add_patch(Rectangle((-32.5, -64.5), 65, 129, fc="#f2c14e", alpha=0.6))
ax.text(0, -50, "电池", ha="center")
parts = {"485": (0, -40), "派": (0, 67), "集线器": (-15, -90), "保险丝": (0, -100), "继电器": (0, 100), "12V": (25, -90)}
for n, (x, y) in parts.items():
    ax.add_patch(Rectangle((x - 14, y - 9), 28, 18, fc="#7fb3d5", ec="k"))
    ax.text(x, y, n, ha="center", va="center", fontsize=8)


def line(pts, c, lw=2.0, ls="-"):
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color=c, lw=lw, ls=ls, solid_capstyle="round")


# 4 条腿：485 一路 + 电源，从隔板中间的孔过去 → 外摆电机 → 髋电机 → 绕线环 → 膝电机
for fx in (1, -1):
    for sy in (1, -1):
        xb = fx * B.XB
        line([(0, -40), (fx * 30, sy * 20), (xb, sy * 20), (fx * (B.XB + 21), sy * 70)], C["bus"])
        line([(0, -100), (fx * 40, -100 if sy < 0 else 100), (xb, sy * 40), (fx * (B.XB + 21), sy * 60)], C["power"], 1.5)
        line([(fx * (B.XB + 21), sy * 70), (fx * M.HIP_X - fx * 30, sy * 95), (fx * M.HIP_X, sy * 100)], C["bus"], 2.5)
# 前面：深度相机 USB3、热像仪 USB、灯 12V、台阶测距 I2C、头（屏幕板 + 舵机）、喇叭
front = [((L2 + 10, 0), C["usb"], "深度相机 USB3 → 派"), ((L2 + 10, -101), C["usb"], "热像仪 USB → 集线器"),
         ((L2 + 10, 101), C["power"], "探照灯 12V + PWM"), ((L2, 70), C["sig"], "台阶测距 ×2 I2C"), ((L2 + 15, 30), C["sig"], "头：屏幕板 USB + 舵机总线")]
for (x, y), c, t in front:
    line([(x, y), (B.X_END, y * 0.15), (60, y * 0.15), (0, 67) if "派" in t or "头" in t else (-15, -90) if "集线器" in t else (25, -90) if "12V" in t else (0, 67)], c, 1.5, "--")
    ax.text(x + 4, y, t, fontsize=8, va="center")
# 顶上：雷达、麦克风；尾部：气体、急停、开关、充电口
line([(M.L2 - 46, -80), (180, -10), (60, -10), (-15, -90)], C["usb"], 1.5, ":")
ax.text(M.L2 - 46, -88, "雷达 USB", fontsize=8)
line([(M.L2 - 46, 80), (180, 10), (60, 10), (0, 67)], C["sig"], 1.5, ":")
ax.text(M.L2 - 46, 88, "麦克风 I2S", fontsize=8)
for (x, y), c, t in [((-L2 - 4, -94), C["sig"], "气体 串口（电平转换）"), ((-L2 - 4, 0), C["power"], "急停 → 继电器线圈"),
                     ((-L2 - 4, 44), C["power"], "总开关"), ((-L2 - 4, -41), C["power"], "充电口 XT60 → 电池")]:
    line([(x, y), (-B.X_END, y * 0.15), (-60, y * 0.15), (0, -100) if "充电" in t else (0, 100) if "急停" in t or "开关" in t else (0, 67)], c, 1.5, "--")
    ax.text(x - 4, y, t, fontsize=8, va="center", ha="right")
for k, t in (("power", "电源"), ("bus", "电机 485 总线 + 电源（XT30 2+2，16 AWG）"), ("usb", "USB"), ("sig", "信号（I2C / I2S / 串口 / 舵机总线）")):
    ax.plot([], [], color=C[k], lw=2, label=t)
ax.legend(loc="lower center", ncol=4, fontsize=9, bbox_to_anchor=(0.5, -0.08))
ax.set_title("007A 走线（俯视示意）：每条腿一路 485（外摆 → 髋 → 膝 串起来），线从外摆隔板中间的孔过", fontsize=12)
ax.set_xlim(-L2 - 120, L2 + 160)
ax.set_ylim(-W2 - 20, W2 + 20)
ax.set_aspect("equal")
ax.axis("off")
plt.tight_layout()
out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "docs", "机械狗007A", "走线图.png")
plt.savefig(out, dpi=105)
print(out)
