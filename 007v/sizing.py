"""机械狗007v选型计算：重量、轮子和髋关节受力、电机够不够、电池能跑多久。

用法：python 007v/sizing.py
所有输入都在最上面，零件到货实测以后改这里的数字，重新跑一遍。结果抄进 docs/机械狗007v/选型计算书.md。
单位：kg、m、N、N·m、W、Wh。
"""
import math

G = 9.81

# ---------------------------------------------------------------- 输入：整机重量（估算，零件到货后称重更新）
MASS = {
    "铝型材机身 + 铝板": 2.5,
    "髋驱动 ×4（5840 金属蜗轮电机 + 3:1 同步带 + 轴承座）": 4 * 0.55,
    "摆臂 ×4（铝板 + 打印箱形臂座）": 4 * 0.4,
    "轮子电机 ×4（扁平关节电机，约 0.23 kg）": 4 * 0.23,
    "152 mm 重载麦轮 ×4": 4 * 0.85,
    "电池 6S2P 21700 + 保护板": 1.0,
    "头部（屏幕板、肩座、舵机、雷达）": 1.2,
    "树莓派 5 + 底盘 ESP32 + 驱动板 + 线": 0.8,
    "打印外壳": 1.5,
    "货斗导轨 + 栏板": 0.5,
}
PAYLOAD = 10.0                 # 背上货物（设计上限）

# ---------------------------------------------------------------- 输入：几何
WHEEL_D = 0.152                # 轮子直径（6 寸麦轮）
ARM = 0.17                     # 摆臂长（髋到轮心）
STANCE_DEG = 55                # 行驶姿态：摆臂往下 55°
HIP_X = 0.15                   # 髋关节离机身中心（前后）
TRACK_HALF = 0.165             # 轮子离中线（左右）

# ---------------------------------------------------------------- 输入：工况
V_FOLLOW = 1.4                 # 跟人走需要的最高速度（人走路约 1.2–1.4 m/s）
V_CRUISE = 1.0                 # 平时巡航
SLOPE = 0.10                   # 要能爬的坡（10%，坡道、无障碍通道）
ACCEL = 0.5                    # 起步加速度
CRR = 0.08                     # 麦轮滚动阻力系数（比普通轮大，保守取 0.08）
MEC_EFF = 0.7                  # 麦轮传力效率（前进；横移时更低，约 0.5）
SAFETY = 1.5                   # 安全系数

# ---------------------------------------------------------------- 输入：候选电机（厂家标称，到货实测更新）
WHEEL_MOTORS = {
    "SteadyWin GIM4310-10（V1.1）": dict(rated=2.05, peak=5.6, rpm_rated=150, rpm_max=228, mass=0.227, size="Φ53×38 mm（含驱动）"),
    "达妙 DM-J4310-2EC（V1.2，24V）": dict(rated=3.5, peak=12.5, rpm_rated=120, rpm_max=200, mass=0.325, size="Φ57×46 mm"),
}
HIP_WORM_RATED = 6.9           # 5840 金属齿轮蜗轮电机额定扭矩（70 kg·cm），挑 24V、带霍尔编码器的
HIP_BELT = 3.0                 # 外接同步带减速比
HIP_WORM_RPM = 40              # 选 24V 40 转/分 那一档

# ---------------------------------------------------------------- 输入：电池和功耗
BATTERY_WH = 6 * 3.6 * 10.0    # 6S2P，单节 5 Ah（如三星 50E）→ 10 Ah
DOD = 0.85                     # 只用 85%，延长寿命
P_ELEC = 20                    # 树莓派 5、ESP32×2、屏幕、雷达、深度相机
P_JETSON = 15                  # 以后加机载电脑


def main():
    m_robot = sum(MASS.values())
    m_total = m_robot + PAYLOAD
    W = m_total * G
    r = WHEEL_D / 2
    print(f"整机自重 ≈ {m_robot:.1f} kg，加 {PAYLOAD:.0f} kg 货物 = {m_total:.1f} kg（{W:.0f} N）")

    # ---- 轮子受力
    n4 = W / 4
    n3 = W / 2            # 抬起一条腿：三个轮子里，对角那个大约扛一半（重心在对角线上时）
    print(f"\n【轮子】平时每个轮子 {n4 / G:.1f} kg；抬一条腿时最重的那个约 {n3 / G:.1f} kg")

    # ---- 轮子扭矩和转速
    f_roll = CRR * W
    f_slope = SLOPE * W
    f_acc = m_total * ACCEL
    f_cont = (f_roll + f_slope) / MEC_EFF
    f_peak = (f_roll + f_slope + f_acc) / MEC_EFF
    t_cont = f_cont * r / 4
    t_peak = f_peak * r / 4
    rpm_need = V_FOLLOW / (math.pi * WHEEL_D) * 60
    rpm_cruise = V_CRUISE / (math.pi * WHEEL_D) * 60
    print(f"  需要：持续 {t_cont:.2f} N·m（10% 坡匀速），峰值 {t_peak:.2f} N·m（坡上起步）；"
          f"巡航 {rpm_cruise:.0f} 转/分，跟人走 {rpm_need:.0f} 转/分")
    for name, mo in WHEEL_MOTORS.items():
        ok_t = mo["rated"] >= t_cont * SAFETY and mo["peak"] >= t_peak * SAFETY
        ok_v = mo["rpm_max"] * 0.85 >= rpm_need            # 带载大约掉 15% 转速
        print(f"  {name}：额定 {mo['rated']} / 峰值 {mo['peak']} N·m，最高 {mo['rpm_max']} 转/分（{mo['size']}）"
              f"→ 扭矩{'够' if ok_t else '不够'}（余量 {mo['rated'] / t_cont:.1f} 倍），"
              f"最高速度约 {mo['rpm_max'] * 0.85 * math.pi * WHEEL_D / 60:.2f} m/s，{'跟得上' if ok_v else '跟不上'}人走路")

    # ---- 髋关节扭矩
    lever = ARM * math.cos(math.radians(STANCE_DEG))      # 轮子着地点到髋的水平距离
    t_stand = n4 * lever
    t_lift = n3 * lever
    t_rise = n4 * ARM                                      # 从趴着撑起来：摆臂接近水平时力臂最大
    hip_rated = HIP_WORM_RATED * HIP_BELT
    hip_rpm = HIP_WORM_RPM / HIP_BELT
    print(f"\n【髋关节】行驶时撑住车身 {t_stand:.1f} N·m；抬一条腿时最重的 {t_lift:.1f} N·m；"
          f"从趴着撑起来（摆臂接近水平）最大 {t_rise:.1f} N·m")
    print(f"  5840 金属蜗轮（额定 {HIP_WORM_RATED} N·m）× 3:1 同步带 = 额定 {hip_rated:.1f} N·m，"
          f"髋转速 {hip_rpm:.1f} 转/分（{hip_rpm * 6:.0f}°/秒）")
    print(f"  → 行驶/抬腿余量 {hip_rated / t_lift:.1f} 倍；满载从趴着撑起来余量 {hip_rated / t_rise:.1f} 倍"
          f"（{'够' if hip_rated >= t_rise else '不够：满载时先卸货或者靠轮子往前推一下再起身'}）")
    print("  蜗轮自锁：撑住车身不用通电；断电也不会塌")

    # ---- 稳定性（静止、满载）
    cog_h = 0.30                                            # 满载时重心高度估算（货在背上）
    tip_side = math.degrees(math.atan(TRACK_HALF / cog_h))
    tip_fwd = math.degrees(math.atan((HIP_X + lever) / cog_h))
    print(f"\n【稳定性】满载重心高约 {cog_h:.2f} m：侧翻临界坡度 {tip_side:.0f}°，前后翻临界 {tip_fwd:.0f}°（10% 坡只有 5.7°）")

    # ---- 电池
    p_drive = (f_roll / MEC_EFF) * V_CRUISE / 0.8           # 平地巡航机械功率 / 电机效率
    for label, p in (("平地巡航", p_drive + P_ELEC), ("平地巡航 + 机载电脑", p_drive + P_ELEC + P_JETSON), ("待机（站着不动）", P_ELEC)):
        print(f"【电池 {BATTERY_WH:.0f} Wh】{label}：约 {p:.0f} W → {BATTERY_WH * DOD / p:.1f} 小时")


if __name__ == "__main__":
    main()
