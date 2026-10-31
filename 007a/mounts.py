"""007A v20：传感器、电子板怎么装 —— 孔位、卡槽、走线槽，尺寸全按官方图纸（来源写在 SENSORS 里）。

前防撞杠（S12，打印 ASA 深灰）= 防撞杠 + 深度相机罩 + 热像仪座 + 探照灯筒 + 两个台阶测距窗，做成一件：
  前面的传感器全拧在杠上，再用 4 颗 M3 从机身里面穿 A05 前端板拧进杠背面的热熔螺母；
  杠背面一道走线槽（宽 6、深 8），所有线沿槽走到 A05 中间的大口进机身。
尾部接口板（S16，打印 ASA 深灰，4 厚）：急停、总开关、充电口，4 颗 M3 拧 A06（热熔螺母在板背面的柱子里）。
  A04 纵梁在 y ±16–20 一直通到 A06，急停后面有 66.5 深：急停挪到 y = -40（纵梁外面），开关居中，充电口 y = +42。
顶上：雷达 LD19 三颗 M2 自攻拧进上层壳里的螺丝柱；麦克风 INMP441 ×2 卡在拾音孔下面的圆槽里；喇叭 ×2 卡圈。
电子托板 C01：树莓派 5、两块 Pololu 降压、功放按官方孔位打孔；IMU 超核 HI12M0 按店铺尺寸表 3 颗 M2.5 铜柱（v21，卡座 P03 取消）；没安装孔的板子（宇树 485 模块是光板、集线器、12 V 降压）留扎带孔。
FASTENERS：每一颗螺丝从哪穿到哪，check() 逐颗查“孔对不对得上、有没有料咬螺丝”。
★ = 官方图没标、到货要量的。
"""
import math

import cadquery as cq

import model as M
from model import V, box, cyl_x, prism_xz

L2, W2, Z0, TOP = M.L2, M.W2, M.Z0, M.TOP
ZC = M.WAIST_TOP + (TOP - M.WAIST_TOP) / 2 - 2
X_A05 = L2 - 10.0                                 # A05 前端板外侧面（eng_body：X_END + T_END）
X_A07_TOP = TOP - 6.0                             # A07 顶板上表面（eng_body：Z_TOPP + T_TOP）

# ---------------------------------------------------------------- 官方尺寸（只放装配要用的）
SENSORS = [
    dict(name="深度相机", model="奥比中光 Gemini 335", src="奥比中光官网《Gemini 330 系列 Reference Design - Mounting》产品图 V1.0（2024-03-22）",
         dims="89.48 × 30 × 25", holes="背面 2-M3 深 3，孔距 45，高度正中；底面 1/4-20 深 5", mount="前防撞杠相机罩，2 颗 M3×6 从杠背面拧（低头 20°）"),
    dict(name="热像仪", model="Adafruit MLX90640 110°（4469）", src="Adafruit 官方 3D 模型 4469 MLX90640 Stemma.step（007a/ref/adafruit/）",
         dims="板 25.4 × 17.78 × 1.57；探头 Φ9.3 在板背面正中，凸出 5.72", holes="4-Φ2.5，孔距 20.32 × 12.7", mount="前防撞杠右头，探头朝前，4 颗 M2.5 自攻"),
    dict(name="台阶测距 ×2", model="Pololu VL53L1X（3415）", src="Pololu 官方尺寸图（007v/ref/pololu/dims_3415.jpg）",
         dims="12.7 × 17.78", holes="2-Φ2.18，孔距 12.7，离左边 2.54", mount="前防撞杠 y ±78，低头 20°，2 颗 M2 自攻"),
    dict(name="探照灯", model="XHP50 20 mm 铜基板 + 3° 聚光透镜（矮款）+ 20 × 20 × 6 散热铝块", src="淘宝店铺尺寸图（格锐特光电 3 度透镜：直径 21.78 高 17.6；铜基板 Φ20 × 1.6；铝块 20 × 20 × 6）",
         dims="透镜 Φ21.8 × 17.6；基板 Φ20 × 1.6；铝块 20 × 20 × 6", holes="—", mount="前防撞杠左头灯筒：透镜、基板、铝块从后往前叠，前面打印压圈压住"),
    dict(name="激光雷达", model="乐动 LD19", src="乐动 LD19 规格书 V2.5",
         dims="安装脚 54 × 46.29，总高 34.8，扫描面离底 27", holes="3-Φ2.5：前两个 ±23.4、前后 31.92", mount="上层壳左前角，3 颗 M2 自攻拧进壳里的柱子"),
    dict(name="麦克风 ×2", model="INMP441 圆板", src="模块资料", dims="Φ13.8", holes="—", mount="上层壳右前角拾音孔下面 Φ14.2 圆槽，胶粘"),
    dict(name="急停", model="施耐德 XB2BS542C", src="施耐德产品数据表（ref/estop/）", dims="头 Φ40；面板后 66.5，底座 30 × 40",
         holes="开孔 Φ22.5，面板厚 1–6", mount="尾部接口板 y = -40"),
    dict(name="总开关", model="GQ16F 自锁带灯", src="厂家图纸（ref/switch/）", dims="头 Φ18，螺母 19，面板后 26", holes="开孔 Φ16.2", mount="尾部接口板正中"),
    dict(name="充电口", model="Amass XT60E1-F", src="Amass 产品规格书（ref/xt60/）", dims="27.1 × 12.1 × 16.7",
         holes="后身截面 15.5 × 8.1（店铺 XT60 图纸，开孔 16.2 × 8.6）；★ 法兰固定孔距 20.5 按外形推的，到货卡尺量", mount="尾部接口板 y = +42，2 颗 M2.5 自攻"),
    dict(name="树莓派 5", model="Raspberry Pi 5", src="树莓派官方机械图", dims="85 × 56", holes="4-M2.5，孔距 58 × 49，离边 3.5", mount="C01 托板，M2.5 铜柱 ×4"),
    dict(name="5V 5.5A 降压", model="Pololu D36V50F5", src="Pololu 官方尺寸图（007v/ref/pololu/dims_4091.jpg）",
         dims="25.4 × 25.4", holes="3-Φ2.18（三个角，离边约 2）", mount="C01 托板，M2 铜柱 ×3"),
    dict(name="5V 3.2A 降压", model="Pololu D36V28F5", src="Pololu 官方尺寸图（007v/ref/pololu/dims_3782.jpg）",
         dims="20.32 × 17.78", holes="2-Φ2.18（对角）", mount="C01 托板，M2 铜柱 ×2"),
    dict(name="功放", model="Adafruit MAX98357A（3006）", src="Adafruit 官方 3D 模型 3006 MAX98357.step（007a/ref/adafruit/）",
         dims="17.78 × 19.05", holes="2-Φ2.5，孔距 12.7，离一边 2.54", mount="C01 托板，M2.5 铜柱 ×2"),
    dict(name="IMU", model="超核 HI12M0-MI1-000（UART + CAN）", src="天猫 HiPNUC 旗舰店详情页尺寸表（HI12 系列：A 10、D 22、D1 17、E 22、E1 16.6、孔 Φ2.6、缺口 R2.4）",
         dims="22 × 22 × 10", holes="3-Φ2.6：下面两个孔距 17，上面一个在正中、离下排 16.6", mount="C01 托板，M2.5 铜柱 ×3"),
    dict(name="抬头舵机", model="飞特 STS3032", src="店铺详情页尺寸图", dims="23 × 12 × 27.5；耳总长 32，离底 20.5–22；花键 25T Φ4.95，顶离底 31.9",
         holes="4-Φ2.5（★ 孔距 28 × 8 按 4-R2 同心推的；输出轴离机身一头 8.21 ★）", mount="打印舵机座 P04，后耳 2 颗 M2 自攻，座子 2 颗 M3 拧左头支架"),
    dict(name="USB 转 4 路 485", model="宇树", src="天猫宇树旗舰店照片：光板、没有安装孔（宇树没给尺寸）", dims="照片估约 60 × 35（托板上留 60 × 45）", holes="没有", mount="C01 托板：扎带 + 3M 双面泡棉胶，不靠尺寸"),
]

# ---------------------------------------------------------------- 前防撞杠
BAR_Y = W2 - 16.0                                 # 杠半长 114
BAR_SCREWS = [(sy * 66.0, Z0 + z) for sy in (1, -1) for z in (24.0, 40.0)]   # 杠 → A05：M3 热熔螺母
A05_CUT = (-60.0, 60.0, Z0 + 16.0, Z0 + 50.0)     # A05 中间的大口（线全从这里进）
CAM_TILT, CAM_F = 20.0, (L2 + 27.0, 0.0, Z0 + 23.0)          # 深度相机：正面中心，低头 20°（看台阶）
CAM_L, CAM_D, CAM_H, CAM_PITCH = 89.48, 30.0, 25.0, 45.0
TH_F = (L2 + 13.5, -101.0, Z0 + 32.0)             # 热像仪探头正面中心
VL_TILT, VL_Y, VL_Z = 20.0, 78.0, Z0 + 32.0       # 台阶测距
VL_F = [(L2 + 0.8, sy * VL_Y, VL_Z) for sy in (1, -1)]       # 往后缩一点：低头 20° 时上面那根螺丝柱不冒出杠的正面
LAMP_F = (L2 + 16.0, 101.0, Z0 + 32.0)            # 探照灯筒正面中心


def place(s, F, tilt=0.0):
    """局部坐标（原点 = 正面中心，x 朝前）→ 机身：先绕 y 低头 tilt 度，再挪到 F。"""
    if tilt:
        s = s.rotate(V(0, 0, 0), V(0, 1, 0), tilt)
    return s.translate(V(*F))


def pt(F, tilt, lx, ly, lz):
    """局部点 → 机身坐标。"""
    a = math.radians(tilt)
    return (F[0] + lx * math.cos(a) + lz * math.sin(a), F[1] + ly, F[2] - lx * math.sin(a) + lz * math.cos(a))


def axis(tilt, lx=-1.0, lz=0.0):
    a = math.radians(tilt)
    return (lx * math.cos(a) + lz * math.sin(a), 0.0, -lx * math.sin(a) + lz * math.cos(a))


def front_bar():
    """前防撞杠（一件打印）+ 里面装的传感器外形。返回 (杠, [(名字, 实体, 颜色)])。"""
    beam = prism_xz([(X_A05, Z0 + 18), (L2 - 6, Z0 + 18), (L2 + 8, Z0 + 24), (L2 + 8, Z0 + 42), (L2 - 6, Z0 + 46), (X_A05, Z0 + 46)], -BAR_Y, BAR_Y)
    hood = place(box(-34, 1.5, -60, 60, -15.5, 15.5), CAM_F, CAM_TILT)
    pod = prism_xz([(L2 - 2, TH_F[2] - 16), (L2 + 12, TH_F[2] - 16), (L2 + 16, TH_F[2] - 12), (L2 + 16, TH_F[2] + 12), (L2 + 12, TH_F[2] + 16),
                    (L2 - 2, TH_F[2] + 16)], TH_F[1] - 17, TH_F[1] + 17)
    tube = cyl_x(LAMP_F[1], LAMP_F[2], 34, L2 - 2, LAMP_F[0])
    bar = beam.fuse(hood).fuse(pod).fuse(tube).intersect(box(X_A05, 400, -200, 200, 0, 600))
    bodies = []
    # 深度相机：罩子里的腔（前面敞开，相机自己的前盖露出来）+ 右端 USB-C 弯头的位置和出线道 + 背面 2 颗 M3
    bar = bar.cut(place(box(-30.3, 3, -45.1, 45.1, -12.8, 12.8), CAM_F, CAM_TILT))
    bar = bar.cut(place(box(-26, -2, -58, -45, -10, 10), CAM_F, CAM_TILT))           # USB-C 弯头（相机右端，-y）
    bar = bar.cut(place(box(-90, -20, -58, -46, -6, 6), CAM_F, CAM_TILT))            # 出线道，往后通到杠背面
    for y in (-CAM_PITCH / 2, CAM_PITCH / 2):
        bar = bar.cut(place(cyl_x(y, 0, 3.4, -36, -29), CAM_F, CAM_TILT))           # 过孔（墙 3.7）
        bar = bar.cut(place(cyl_x(y, 0, 6.5, -90, -34), CAM_F, CAM_TILT))           # 沉头孔，从杠背面进螺丝刀
    cam_s = box(-CAM_D, 0, -CAM_L / 2, CAM_L / 2, -CAM_H / 2, CAM_H / 2)
    cam = place(cam_s.fillet(CAM_H / 2 - 0.5, [e for e in cam_s.Edges() if abs(e.Center().y) > CAM_L / 2 - 0.1 and abs(e.Center().z) > 1]), CAM_F, CAM_TILT)
    bodies.append(("深度相机 Gemini 335", cam, "skel_dk"))
    for y, d in ((-25.0, 14), (25.0, 14), (0.0, 9), (-14.0, 7)):
        bodies.append((f"相机镜头{y:.0f}", place(cyl_x(y, 0, d, -0.5, 0.3), CAM_F, CAM_TILT), "glass"))
    # 热像仪：探头 Φ9.3 朝前（在板背面），前面 Φ12 窗；板槽 + 4 个 M2.5 自攻柱；后面的腔通到杠背面（插座、螺丝刀）
    bar = bar.cut(place(cyl_x(0, 0, 10.0, -6.0, 0.5), TH_F))
    bar = bar.cut(place(cyl_x(0, 0, 12.0, 0, 10), TH_F))                             # 窗
    bar = bar.cut(place(box(-7.4, -5.6, -12.9, 12.9, -9.1, 9.1), TH_F))              # 板槽（0.15 余量）
    bar = bar.cut(place(box(-60, -7.3, -12.0, 12.0, -8.2, 8.2), TH_F))               # 后腔（板靠槽边托住）
    for y in (-10.16, 10.16):
        for z in (-6.35, 6.35):
            bar = bar.cut(place(cyl_x(y, z, 2.2, -7.4, -1.0), TH_F))                 # M2.5 自攻底孔，深 4.6
    th = place(box(-7.29, -5.72, -12.7, 12.7, -8.89, 8.89), TH_F).fuse(place(cyl_x(0, 0, 9.3, -4.85, 0), TH_F))
    bodies.append(("热像仪 MLX90640", th, "skel_dk"))
    # 台阶测距 ×2：板槽 + 前面的窗 + 2 个 M2 自攻柱 + 后腔
    for F in VL_F:
        bar = bar.cut(place(box(-1.7, 0.1, -6.55, 6.55, -9.1, 9.1), F, VL_TILT))
        bar = bar.cut(place(box(0.1, 2.0, -6.0, 6.0, -8.5, 8.5).cut(cyl_x(-3.81, 6.35, 4.4, 0, 3)).cut(cyl_x(-3.81, -6.35, 4.4, 0, 3)), F, VL_TILT))
        bar = bar.cut(place(box(2.0, 30, -1.0, 6.0, -5.0, 5.0), F, VL_TILT))           # 窗（★ 传感器在板上的位置按照片估；让开螺丝柱）
        bar = bar.cut(place(box(-60, -1.7, -6.0, 6.0, -8.0, 8.0), F, VL_TILT))
        for z in (-6.35, 6.35):
            bar = bar.cut(place(cyl_x(-3.81, z, 1.7, -1.7, 4.5), F, VL_TILT))
        bodies.append((f"台阶测距 VL53L1X{F[1]:+.0f}", place(box(-1.6, 0, -6.35, 6.35, -8.89, 8.89), F, VL_TILT), "skel_dk"))
    # 探照灯（v21 按店铺图）：透镜 Φ21.8 × 17.6 + 铜基板 Φ20 × 1.6 + 散热铝块 20 × 20 × 6 从前往后叠 + 后面的线孔
    bar = bar.cut(place(cyl_x(0, 0, 22.2, -18.1, 1), LAMP_F))
    bar = bar.cut(place(cyl_x(0, 0, 20.4, -19.8, -18.1), LAMP_F))
    bar = bar.cut(place(box(-26.0, -19.8, -10.2, 10.2, -10.2, 10.2), LAMP_F))
    bar = bar.cut(place(cyl_x(0, 0, 6.0, -60, -26), LAMP_F))
    bodies.append(("探照灯透镜", place(cyl_x(0, 0, 21.8, -18.1, -0.5), LAMP_F), "lamp"))
    bodies.append(("探照灯压圈（打印）", place(cyl_x(0, 0, 36, -0.5, 1.5).cut(cyl_x(0, 0, 19, -1, 2)), LAMP_F), "metal"))
    # 背面：走线槽 + 4 个热熔螺母孔
    for sy in (1, -1):                                                # 走线槽只在相机两边（|y| 50–110），中间是相机背后的墙
        bar = bar.cut(box(X_A05 - 1, X_A05 + 8, *sorted((sy * 50.0, sy * (BAR_Y - 4))), Z0 + 29, Z0 + 35))
    for y, z in BAR_SCREWS:
        bar = bar.cut(cyl_x(y, z, 4.0, X_A05 - 1, X_A05 + 6))
    return bar, bodies


def bar_envelope(gap=0.3):
    """外壳要给防撞杠让开的地方（杠穿过下层底盘 / 黑腰的前壁）。"""
    beam = prism_xz([(X_A05 - 1, Z0 + 18 - gap), (L2 - 6, Z0 + 18 - gap), (L2 + 9, Z0 + 23), (L2 + 9, Z0 + 43), (L2 - 6, Z0 + 46 + gap),
                     (X_A05 - 1, Z0 + 46 + gap)], -BAR_Y - gap, BAR_Y + gap)
    hood = place(box(-34 - gap, 1.5 + gap, -60 - gap, 60 + gap, -15.5 - gap, 15.5 + gap), CAM_F, CAM_TILT)
    return beam.fuse(hood)


# ---------------------------------------------------------------- 尾部接口板
TP_X0, TP_X1 = -(L2 - 1.0), -(L2 - 5.0)           # 板：x -239 … -235（正面在上层壳尾部凹槽底）
TP_Y, TP_Z = 69.5, 23.5
ESTOP_Y, SWITCH_Y, XT60_Y = -40.0, 0.0, 42.0
TAIL_SCREWS = [(sy * 63.0, ZC + dz) for sy in (1, -1) for dz in (-15.0, 15.0)]
A06_CUT = (-57.0, 52.0, ZC - 21.0, ZC + 21.0)


def tail_panel():
    pl = box(TP_X0, TP_X1, -TP_Y, TP_Y, ZC - TP_Z, ZC + TP_Z)
    for y, z in TAIL_SCREWS:
        pl = pl.fuse(cyl_x(y, z, 9.0, TP_X1, -X_A05))                       # 柱子顶到 A06 外侧面
        pl = pl.cut(cyl_x(y, z, 4.0, -X_A05 - 6, -X_A05 + 1))                # M3 热熔螺母孔
    pl = pl.cut(cyl_x(ESTOP_Y, ZC, 22.5, TP_X0 - 1, TP_X1 + 1))
    pl = pl.cut(cyl_x(SWITCH_Y, ZC, 16.2, TP_X0 - 1, TP_X1 + 1))
    pl = pl.cut(box(TP_X0 - 1, TP_X1 + 1, XT60_Y - 4.3, XT60_Y + 4.3, ZC - 8.1, ZC + 8.1))   # XT60 后身 8.1 × 15.5（店铺图纸）+ 余量
    for dz in (-10.25, 10.25):
        pl = pl.cut(cyl_x(XT60_Y, ZC + dz, 2.2, TP_X0 - 1, TP_X1 + 1))    # M2.5 自攻
    bodies = [("急停 XB2BS542C", cyl_x(ESTOP_Y, ZC, 40, TP_X0 - 32, TP_X0).fuse(box(TP_X1, TP_X1 + 66.5 - 4, ESTOP_Y - 15, ESTOP_Y + 15, ZC - 20, ZC + 20)), "estop"),
              ("总开关 GQ16", cyl_x(SWITCH_Y, ZC, 18, TP_X0 - 2, TP_X0).fuse(cyl_x(SWITCH_Y, ZC, 16, TP_X1, TP_X1 + 26)), "metal"),
              ("充电口 XT60E1-F", box(TP_X0 - 2, TP_X0, XT60_Y - 6.05, XT60_Y + 6.05, ZC - 13.55, ZC + 13.55).fuse(
                  box(TP_X1, TP_X0 + 16.7, XT60_Y - 4.0, XT60_Y + 4.0, ZC - 7.9, ZC + 7.9)), "rubber")]
    return pl, bodies


def tail_envelope(gap=0.3):
    return box(TP_X0 - 1, TP_X1 + 1, -TP_Y - gap, TP_Y + gap, ZC - TP_Z - gap, ZC + TP_Z + gap)


# ---------------------------------------------------------------- 顶上：雷达、麦克风、喇叭、外壳螺丝柱
RADAR = (L2 - 46.0, -(W2 - 50.0))                 # 雷达中心（model.py）
LD19_HOLES = [(15.96, 23.4), (15.96, -23.4), (-15.96, 0.0)]   # (x 前后, y 左右)，以雷达中心为原点（007v/config.py，规格书 V2.5）
MIC = (L2 - 46.0, W2 - 50.0)
MIC_PODS = [(MIC[0] + 11 * math.cos(a), MIC[1] + 11 * math.sin(a)) for a in (math.radians(135), math.radians(-45))]   # 两个拾音孔下面
SPK_Y = 75.0 + 29.0                               # 喇叭中心 y（扬声器孔那一排的中间）


def top_features(up, look):
    """上层甲板壳（已经掏空）加：外壳螺丝柱、雷达柱 + 孔、麦克风槽、喇叭卡圈。look = 没掏空的外形（柱子不能冒出外面）。"""
    import eng_body as B
    z_a07 = B.Z_TOPP + B.T_TOP
    for x, y in B.SHELL_SCREWS_TOP:                                  # 外壳 → A07：Φ9 柱子 + M3 热熔螺母 Φ4 × 5.5
        up = up.fuse(cq.Solid.makeCylinder(4.5, TOP - z_a07, V(x, y, z_a07)).intersect(look))
        up = up.cut(cq.Solid.makeCylinder(2.0, 5.5, V(x, y, z_a07 - 0.5)))
    rx, ry = RADAR                                                     # 雷达 3 颗 M2 自攻（雷达坐在壳顶上，护圈套在外面）
    for hx, hy in LD19_HOLES:
        x, y = rx + hx, ry + hy
        up = up.fuse(cq.Solid.makeCylinder(3.5, 10, V(x, y, TOP - 10)).intersect(look))
        up = up.cut(cq.Solid.makeCylinder(0.85, 9, V(x, y, TOP - 8.5)))
    up = up.cut(cq.Solid.makeCylinder(6.0, 20, V(rx - 8, ry, TOP - 15)))     # 雷达的线从底座后面进壳
    for x, y in MIC_PODS:                                             # 麦克风：拾音孔正下方 Φ14.2 圆槽（板底进声，孔 Φ1 通到外面）
        up = up.fuse(cq.Solid.makeCylinder(9.0, 3.5, V(x, y, TOP - 2.5 - 3.5)).intersect(look))
        up = up.cut(cq.Solid.makeCylinder(7.1, 1.8, V(x, y, TOP - 2.5 - 1.8)))
        up = up.cut(cq.Solid.makeCylinder(0.5, 4, V(x, y, TOP - 3)))
    for sy in (1, -1):                                                # 喇叭 Φ28：前壁里面一圈卡圈（内 Φ28.4，深 4），胶粘
        ring = cyl_x(sy * SPK_Y, ZC + 2, 33, L2 - 6.5, L2 - 2.5).cut(cyl_x(sy * SPK_Y, ZC + 2, 28.4, L2 - 7, L2 - 2))
        up = up.fuse(ring.intersect(look))
    return up


def radar_collar_holes(collar):
    """雷达护圈：中间开成安装脚的方口（54.6 × 46.9），护圈底面 2 个 M2.5 自攻孔（从壳里面往上拧）。"""
    rx, ry = RADAR
    collar = collar.cut(box(rx - 23.45, rx + 23.45, ry - 27.3, ry + 27.3, TOP - 1, TOP + 30))
    for sy in (1, -1):
        collar = collar.cut(cq.Solid.makeCylinder(1.1, 6, V(rx, ry + sy * 29.5, TOP - 0.5)))
    return collar


COLLAR_SCREWS = [(RADAR[0], RADAR[1] + sy * 29.5) for sy in (1, -1)]

# ---------------------------------------------------------------- 电子托板 C01
TRAY_BOARDS = [   # (名字, 板中心 x, y, 板长 x, 宽 y, 高, [(孔 x, 孔 y 相对板中心)], 孔径, 状态)；托板 x ±57.45、y ±113（|y| > 107 是插槽导轨，|x| > 48.45 且 |y| > 38 是让电机的缺角）
    ("树莓派 5（带散热）", 0.0, 63.5, 56.0, 85.0, 22.0, [(sx * 24.5, y) for sx in (1, -1) for y in (39.0, -19.0)], 2.7, "已确认：4-M2.5 孔距 58 × 49"),
    ("5V 5.5A 降压 D36V50F5", -41.0, 0.0, 25.4, 25.4, 9.5, [(-10.5, -10.6), (-10.5, 10.6), (10.5, 10.6)], 2.6, "已确认：Pololu 官方图（孔离边约 2，开 Φ2.6 留余量）"),
    ("5V 3.2A 降压 D36V28F5", 41.0, 6.0, 17.78, 20.32, 8.0, [(7.09, 7.74), (-6.51, -8.16)], 2.6, "已确认：Pololu 官方图"),
    ("功放 MAX98357A", 44.0, -16.0, 17.78, 19.05, 4.0, [(-6.35, 6.985), (6.35, 6.985)], 2.7, "已确认：Adafruit 官方 STEP"),
    ("USB 转 4 路 485（宇树）", 0.0, -40.0, 60.0, 45.0, 15.0, [], 0, "光板没孔：扎带 + 双面胶"),
    ("IMU 超核 HI12M0", 0.0, 0.0, 22.0, 22.0, 10.0, [(0.0, 8.3), (-8.5, -8.3), (8.5, -8.3)], 2.7, "已确认：店铺尺寸表（下排孔距 17、上下 16.6）"),
    ("USB 集线器", -20.0, -85.0, 40.0, 30.0, 12.0, [], 0, "到货实测：扎带"),
    ("12V 5A 降压（抬头舵机、探照灯）", 26.0, -88.0, 26.0, 20.0, 15.0, [], 0, "到货实测：扎带"),
]
STANDOFF = 6.0                                    # 铜柱高


def tray_holes(tray, z0):
    """C01（亚克力 3）：板子的孔、扎带孔。z0 = 托板下表面。"""
    for n, cx, cy, lx, ly, h, holes, d, st in TRAY_BOARDS:
        for hx, hy in holes:
            tray = tray.cut(cq.Solid.makeCylinder(d / 2, 5, V(cx + hx, cy + hy, z0 - 1)))
        if not holes:                                                  # 扎带孔：板子两边各一对 3 × 4.5
            for sx in (1, -1):
                for dy in (-ly / 4, ly / 4):
                    x = cx + sx * (lx / 2 + 3.5)
                    tray = tray.cut(box(x - 1.5, x + 1.5, cy + dy - 2.25, cy + dy + 2.25, z0 - 1, z0 + 4))
    return tray


def tray_bodies(z_top):
    """托板上的板子（按官方外形）+ 铜柱；z_top = 托板上表面。"""
    P = []
    for n, cx, cy, lx, ly, h, holes, d, st in TRAY_BOARDS:
        zb = z_top + (STANDOFF if holes else 0.0)
        P.append((n, box(cx - lx / 2, cx + lx / 2, cy - ly / 2, cy + ly / 2, zb, zb + h), "skel"))
        for hx, hy in holes:
            P.append((f"{n} 铜柱", cq.Solid.makeCylinder(2.0, STANDOFF, V(cx + hx, cy + hy, z_top)), "metal"))
    return P


# ---------------------------------------------------------------- 每一颗螺丝
def fasteners():
    """[(名字, 起点（螺丝头那面）, 方向（朝螺丝尖）, 长, 孔半径, [(零件名, 要求)])]。要求：clear = 过孔（中间空、周围有料）、
    grip = 自攻（有效长 ≥ 3）；insert = 铜热熔螺母（≥ 2.5）。零件名按开头匹配。"""
    import eng_body as B
    F = []
    z_a07 = B.Z_TOPP + B.T_TOP
    for x, y in B.SHELL_SCREWS_TOP:
        F.append((f"上层壳 M3×6 ({x:.0f},{y:.0f})", (x, y, B.Z_TOPP), (0, 0, 1), 6, 1.7, [("A07", "clear"), ("上层甲板壳", "insert")]))
    for x, y in B.SHELL_SCREWS_BOT:
        F.append((f"下层底盘 M3×8 ({x:.0f},{y:.0f})", (x, y, M.Z0), (0, 0, 1), 8, 1.7, [("下层底盘", "clear"), ("A01", "clear")]))
    for y, z in BAR_SCREWS:
        F.append((f"防撞杠 M3×8 (y{y:+.0f},z{z - Z0:.0f})", (X_A05 - 4.0, y, z), (1, 0, 0), 8, 1.7, [("A05", "clear"), ("前防撞杠", "insert")]))
    for y in (-CAM_PITCH / 2, CAM_PITCH / 2):
        p = pt(CAM_F, CAM_TILT, -34.5, y, 0)
        F.append((f"深度相机 M3×6 (y{y:+.0f})", p, axis(CAM_TILT, 1.0), 3.5, 1.7, [("前防撞杠", "clear")]))
    for y in (-10.16, 10.16):
        for z in (-6.35, 6.35):
            F.append((f"热像仪 M2.5×6 ({y:+.1f},{z:+.1f})", (TH_F[0] - 7.29, TH_F[1] + y, TH_F[2] + z), (1, 0, 0), 6, 1.1, [("前防撞杠", "grip")]))
    for Fv in VL_F:
        for z in (-6.35, 6.35):
            F.append((f"台阶测距 M2×6 (y{Fv[1]:+.0f},{z:+.1f})", pt(Fv, VL_TILT, -1.6, -3.81, z), axis(VL_TILT, 1.0), 6, 0.85, [("前防撞杠", "grip")]))
    for y, z in TAIL_SCREWS:
        F.append((f"尾部接口板 M3×8 (y{y:+.0f},z{z - ZC:+.0f})", (-X_A05 + 4.0, y, z), (-1, 0, 0), 8, 1.7, [("A06", "clear"), ("尾部接口板", "insert")]))
    for hx, hy in LD19_HOLES:
        F.append((f"雷达 M2×8 ({hx:+.1f},{hy:+.1f})", (RADAR[0] + hx, RADAR[1] + hy, TOP + 3.0), (0, 0, -1), 8, 0.85, [("上层甲板壳", "grip")]))
    import eng_head as H
    for x, z in H.CRADLE_SCREWS:
        F.append((f"舵机座 M3×8 (x{x:.0f})", (x, 35.0, z), (0, -1, 0), 8, 1.7, [("舵机座", "clear"), ("头支架左", "clear")]))
    for dz in (-4.0, 4.0):
        F.append((f"舵机后耳 M2×6 (z{dz:+.0f})", (H.SV_XC - H.SV_EAR / 2 + 2.0, H.SV_Y0 + H.SV_EAR_Y + 1.5, H.PZ + dz), (0, -1, 0), 6, 0.85, [("舵机座", "grip")]))
    for n, cx, cy, lx, ly, h, holes, d, st in TRAY_BOARDS:
        for hx, hy in holes:
            F.append((f"{n.split('（')[0]} 铜柱孔 ({hx:+.1f},{hy:+.1f})", (cx + hx, cy + hy, B.TRAY_Z), (0, 0, 1), 3, d / 2 - 0.15, [("C01", "clear")]))
    return F


def check(parts):
    """parts = {名字: 实体}。逐颗螺丝查：沿螺丝轴每 0.4 取一个点，中心要空、外面一圈（孔半径 + 0.9）要有料。"""
    rows = []
    for name, p0, d, length, r, need in fasteners():
        dx, dy, dz = d
        n = math.sqrt(dx * dx + dy * dy + dz * dz)
        dx, dy, dz = dx / n, dy / n, dz / n
        ux, uy, uz = (dy, -dx, 0) if abs(dz) < 0.9 else (1, 0, 0)       # 跟轴垂直的两个方向
        un = math.sqrt(ux * ux + uy * uy + uz * uz)
        ux, uy, uz = ux / un, uy / un, uz / un
        vx, vy, vz = dy * uz - dz * uy, dz * ux - dx * uz, dx * uy - dy * ux
        ok_all, notes = True, []
        for pname, kind in need:
            cands = [(k, s) for k, s in parts.items() if k.startswith(pname)]
            if not cands:
                ok_all = False
                notes.append(f"{pname}：没找到零件")
                continue
            best = None
            for k, s in cands:
                bb = s.BoundingBox()
                engaged, blocked = 0.0, 0.0
                t = 0.2
                while t <= length - 0.2:
                    c = V(p0[0] + dx * t, p0[1] + dy * t, p0[2] + dz * t)
                    if bb.xmin - 5 <= c.x <= bb.xmax + 5 and bb.ymin - 5 <= c.y <= bb.ymax + 5 and bb.zmin - 5 <= c.z <= bb.zmax + 5:
                        rr = r + 0.9
                        ring = [V(c.x + rr * (ux * math.cos(a) + vx * math.sin(a)), c.y + rr * (uy * math.cos(a) + vy * math.sin(a)),
                                  c.z + rr * (uz * math.cos(a) + vz * math.sin(a))) for a in [i * math.pi / 4 for i in range(8)]]
                        frac = sum(1 for q in ring if s.isInside(q, 0.01)) / 8.0
                        if frac >= 0.75:
                            if s.isInside(c, 0.01):
                                blocked += 0.4
                            else:
                                engaged += 0.4
                    t += 0.4
                if best is None or engaged - blocked > best[1] - best[2]:
                    best = (k, engaged, blocked)
            k, engaged, blocked = best
            want = {"grip": 3.0, "insert": 2.5}.get(kind, 1.5)               # 自攻咬 ≥ 3；铜热熔螺母（M3 × 5）螺纹强，拧进 ≥ 2.5 就够
            ok = engaged >= want and blocked < 0.5
            ok_all &= ok
            notes.append(f"{k.split('（')[0]}：{'有孔' if blocked < 0.5 else f'孔被堵 {blocked:.1f}'}、有料 {engaged:.1f}{'' if ok else ' ✗'}")
        rows.append((name, ok_all, "；".join(notes)))
    return rows
