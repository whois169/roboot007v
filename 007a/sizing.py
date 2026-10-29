"""007A 选型计算：整机重量估算 → 每个关节要多大扭矩 → 电机够不够。

腿：大腿 200 / 小腿 190（stair_calc.py 定的），膝朝后。静力学（侧面 2D）：脚底受竖直力 F，
  膝扭矩 = F × 脚到膝的水平距离，髋（前后摆）扭矩 = F × 脚到髋的水平距离，外摆扭矩 = F × 脚到外摆轴的横向距离。
工况（F 已经乘了动载系数）：
  站立        4 条腿着地              F = m g / 4
  对角小跑    2 条腿着地 × 动载 1.8   F = m g / 2 × 1.8
  爬楼梯      3 条腿着地 × 动载 1.3   F = m g / 3 × 1.3，姿势取 stair_calc.py 里出现过的最弯 / 最斜的
  趴下站起    4 条腿着地，膝最弯（40°） F = m g / 4
用法：python 007a/sizing.py
"""
import math

G = 9.81
L1, LS = 200.0, 190.0
ABD_Y = 60.0              # 外摆轴到脚的横向距离（腿平面离外摆轴 ≈ 髋盘厚度一半 + 间隙）

# ---- 重量估算（kg）：电机按候选里最重的 0.53 kg 算
MASS = [
    ("关节电机 ×12", 12 * 0.53),
    ("腿结构（铝 + 打印件 + 脚垫）×4", 4 * 0.35),
    ("机身骨架（铝板 + 型材）", 2.0),
    ("外壳（打印）", 1.5),
    ("电池 6S 约 220 Wh", 1.5),
    ("主控 + 驱动 + 线", 0.8),
    ("传感器（雷达、深度相机、热像仪、气体、屏幕、灯）", 0.8),
]
BODY = sum(m for _, m in MASS)
PAYLOAD = 5.0


def leg(knee_deg, foot_dx=0.0):
    """膝角（大腿和小腿夹角）→ 髋到脚的竖直高度、膝的位置（膝朝后）。foot_dx：脚在髋前面多少（mm）。"""
    d = math.sqrt(L1 ** 2 + LS ** 2 - 2 * L1 * LS * math.cos(math.radians(knee_deg)))
    h = math.sqrt(max(d ** 2 - foot_dx ** 2, 1))
    foot = (foot_dx, -h)
    a = math.acos((L1 ** 2 + d ** 2 - LS ** 2) / (2 * L1 * d))
    ang = math.atan2(foot[1], foot[0]) - a
    knee = (L1 * math.cos(ang), L1 * math.sin(ang))
    if knee[0] > foot[0]:                                        # 取膝在后面那个解
        ang = math.atan2(foot[1], foot[0]) + a
        knee = (L1 * math.cos(ang), L1 * math.sin(ang))
    return foot, knee


def torques(F, knee_deg, foot_dx=0.0):
    foot, knee = leg(knee_deg, foot_dx)
    t_knee = F * abs(foot[0] - knee[0]) / 1000
    t_hip = F * abs(foot[0]) / 1000
    t_abd = F * ABD_Y / 1000
    return t_hip, t_knee, t_abd, -foot[1]


CASES = [  # 名字, 着地腿数, 动载, 膝角, 脚在髋前面多少
    ("站立（膝 90°）", 4, 1.0, 90.0, 0.0),
    ("对角小跑", 2, 1.8, 90.0, 0.0),
    ("爬楼梯·后腿蹬直", 3, 1.3, 140.0, -100.0),
    ("爬楼梯·前腿收起", 3, 1.3, 60.0, 80.0),
    ("趴下站起（膝 40°）", 4, 1.0, 40.0, 0.0),
]

if __name__ == "__main__":
    print("重量估算：")
    for n, m in MASS:
        print(f"  {n:<28} {m:5.2f} kg")
    print(f"  整机 {BODY:.1f} kg，加货 {PAYLOAD:.0f} kg = {BODY + PAYLOAD:.1f} kg\n")
    worst = [0, 0, 0]
    print(f"{'工况':<16}{'髋高':>6}{'每腿力 N':>10}{'髋前后 N·m':>12}{'膝 N·m':>10}{'外摆 N·m':>10}")
    for name, n, k, kd, dx in CASES:
        F = (BODY + PAYLOAD) * G / n * k
        th, tk, ta, h = torques(F, kd, dx)
        worst = [max(worst[0], th), max(worst[1], tk), max(worst[2], ta)]
        print(f"{name:<16}{h:6.0f}{F:10.0f}{th:12.1f}{tk:10.1f}{ta:10.1f}")
    print(f"\n最大：髋前后 {worst[0]:.1f}、膝 {worst[1]:.1f}、外摆 {worst[2]:.1f} N·m（这是要电机“能持续一会儿”的扭矩，峰值再留 1.5 倍）")
