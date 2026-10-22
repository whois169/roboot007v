"""头部电子件的安装结构（2026-10-22）：每块板子的尺寸、孔位都来自厂家图纸 / 电路板文件（007v/ref/boards/）或卖家图纸。

| 零件 | 尺寸依据 |
|---|---|
| 乐鑫 ESP32-S3-DevKitC-1 v1.1 | 官方尺寸图：62.74 × 25.40，没有安装孔，排针离边 1.27 |
| Adafruit MLX90640 110°（4469） | 官方 .brd：25.40 × 17.78，4 个 Φ2.5 孔（2.54 / 22.86 × 2.54 / 15.24），探头在背面正中 |
| Adafruit SHT40（4885） | 官方 .brd：同上尺寸和孔位，传感器在正面正中 |
| Adafruit MAX98357A（3006） | 官方 .brd：17.78 × 19.05，2 个 Φ2.5 孔（2.54 / 15.24, 16.51） |
| SparkFun VL53L5CX Mini（19013） | 官方 .brd + 尺寸图：25.40 × 12.70，2 个 Φ3.3 孔（2.54 / 22.86, 2.54），芯片在背面（12.70, 8.89） |
| OV5693 USB 摄像头（得虹 WN-L2112） | 卖家图纸：镜头模组 8 × 8.5 × 5.3，主板 15 × 30，4 孔 11 × 26，排线选 55 mm |
| INMP441 圆板 | 卖家图：Φ13.8，板厚约 1，麦克风是底部进声（声孔在板子背面） |
| GY-302 BH1750 | 通用板 13.9 × 18.5（各家差 ±0.5，卡槽留了余量） |
| 海凌科 LD2410B | 35 × 7（以原厂规格书为准，卡槽两头可调） |
| 5 mm 940 nm 红外发射管、VS1838B | 标准元件：Φ5 × 8.6（法兰 Φ5.8）；6.0 × 7.0 × 5.0 |

坐标：屏幕模块用俯仰轴坐标（x 前、y 左、z 上，mm）；耳朵用脖子坐标，-Y 耳朵先按 +Y 画好再镜像。
"""
import math

from shell import box, cyl_dir, cyl_y, cyl_z, prism_xz

# ================================================================ 下巴（俯仰轴坐标）
C0, C1 = (56.0, -42.0), (84.0, -24.0)                    # 下巴斜面外轮廓线
_L = math.hypot(C1[0] - C0[0], C1[1] - C0[1])
U = ((C1[0] - C0[0]) / _L, (C1[1] - C0[1]) / _L)          # 沿斜面往前上
N_OUT = ((C1[1] - C0[1]) / _L, -(C1[0] - C0[0]) / _L)     # 斜面朝外（往前下）
M = ((C0[0] + C1[0]) / 2, (C0[1] + C1[1]) / 2)            # 三个开孔都在这条中线上
WALL = 2.5


def chin(u, w):
    """下巴局部坐标 → (x, z)：u 沿斜面，w 从外表面往里（w = 2.5 是内壁）。"""
    return (M[0] + U[0] * u - N_OUT[0] * w, M[1] + U[1] * u - N_OUT[1] * w)


def lbox(u0, u1, v0, v1, w0, w1):
    return prism_xz([chin(u0, w0), chin(u1, w0), chin(u1, w1), chin(u0, w1)], v0, v1)


def lcyl(u, v, d, w0, w1):
    x, z = chin(u, w0)
    return cyl_dir((x, v, z), (-N_OUT[0], 0, -N_OUT[1]), d, w1 - w0)


# 摄像头镜头模组（沿排线方向 8.5 → u，8 → v，高 5.3）
CAM_V = 0.0
CAM = dict(u=8.5, v=8.0, h=5.3)
# VL53L5CX Mini：芯片（背面）对准下巴 Φ9 孔，板子 25.4 沿 v、12.7 沿 u；孔在离芯片 -6.35（u）、±10.16（v）
TOF_V = -30.0
TOF_BRD = dict(u0=-8.89, u1=3.81, v0=TOF_V - 12.7, v1=TOF_V + 12.7, w0=4.0, w1=5.6)
TOF_HOLES = [(-6.35, TOF_V - 10.16), (-6.35, TOF_V + 10.16)]
LED_V = 30.0                                             # 隐私灯（3 mm 红色 LED，从里面插）

# 左侧格栅（+Y 内壁 y = 60.5）后面：两个麦克风 + 光线传感器；格栅孔在 x 32..50、z -30..-12（间距 6）
Y_IN = 63.0 - WALL
MICS = [(32.0, -30.0), (50.0, -30.0)]
BH_CHIP = (44.0, -12.0)                                  # BH1750 芯片对准这个格栅孔
BH = dict(w=13.9, h=18.5, chip_from_bottom=4.5)

# 顶盖里面：毫米波 LD2410B（35 × 7），天线朝上隔着顶盖
LD = dict(x0=21.5, x1=56.5, w=7.0, z_top=63.5)

# 电子托板上表面 z = -32.5
TRAY_TOP = -32.5
ESP = dict(x0=-14.0, x1=-14.0 + 62.74, y0=8.0, y1=8.0 + 25.4, z0=-12.5)   # 板底离托板 20：下面插杜邦线
CAMB = dict(x0=22.0, x1=52.0, y0=-10.0, y1=5.0, z0=TRAY_TOP + 4.0)          # 摄像头主板，4 孔 11 × 26
CAMB_HOLES = [(24.0, -8.0), (50.0, -8.0), (24.0, 3.0), (50.0, 3.0)]
MAXB = dict(x0=22.0, x1=22.0 + 17.78, y0=-44.0, y1=-44.0 + 19.05, z0=TRAY_TOP + 4.0)
MAX_HOLES = [(22.0 + 2.54, -44.0 + 16.51), (22.0 + 15.24, -44.0 + 16.51)]


def head_mounts():
    """返回 dict(tub_add, tub_cut, cap_add, cap_cut, tray_add, tray_cut, bezel)。"""
    ta, tc, ca, cc, ra, rc = [], [], [], [], [], []
    # ---- 摄像头：镜头圈（单独打印，压进 Φ13 孔）+ 内壁上的方框（排线从后下方出来）
    hu, hv = CAM["u"] / 2 + 0.2, CAM["v"] / 2 + 0.2
    frame = lbox(-hu - 1.2, hu + 1.2, CAM_V - hv - 1.2, CAM_V + hv + 1.2, 2.0, 2.5 + CAM["h"] + 0.3)
    frame = frame.cut(lbox(-hu, hu, CAM_V - hv, CAM_V + hv, 2.4, 9.0))
    frame = frame.cut(lbox(-hu - 2, -hu + 0.1, CAM_V - 3.6, CAM_V + 3.6, 2.4, 9.0))   # 排线口
    ta.append(frame)
    bezel = lcyl(0, CAM_V, 12.8, -1.5, 2.0).cut(lcyl(0, CAM_V, 7.2, -3, 4))         # 外面露 1.5（和外观一样），里面让开方框
    # ---- 多点测距：两根定位柱穿过板子的 Φ3.3 孔，装好后用烙铁把柱头烫扁
    for u, v in TOF_HOLES:
        ta.append(lcyl(u, v, 5.5, 2.0, TOF_BRD["w0"]))
        ta.append(lcyl(u, v, 3.0, TOF_BRD["w0"] - 0.1, TOF_BRD["w1"] + 2.0))
    # ---- 隐私灯：3 mm LED 座（灯头刚好在外表面里 0.5）
    ta.append(lcyl(0, LED_V, 6.5, 2.0, 7.2))
    tc.append(lcyl(0, LED_V, 3.15, 0.0, 5.9))
    tc.append(lcyl(0, LED_V, 3.95, 5.8, 8.0))
    # ---- 麦克风：Φ13.8 圆板贴着内壁（中间垫 1 mm 泡棉），圈高 3，底下留出线口
    for x, z in MICS:
        ring = cyl_y(x, z, 17.4, Y_IN - 3.0, Y_IN + 0.3).cut(cyl_y(x, z, 14.3, Y_IN - 4, Y_IN + 1))
        ring = ring.cut(box(x - 2.5, x + 2.5, Y_IN - 4, Y_IN + 1, z - 10, z - 6))
        ta.append(ring)
    # ---- 光线传感器 GY-302：U 形卡槽，板子从上往下插；芯片一面朝墙，离墙 1.9
    bx0, bx1 = BH_CHIP[0] - BH["w"] / 2 - 0.25, BH_CHIP[0] + BH["w"] / 2 + 0.25
    bz0 = BH_CHIP[1] - BH["chip_from_bottom"] - 0.25
    for xa, xb in ((bx0 - 1.2, bx0), (bx1, bx1 + 1.2)):
        ta.append(box(xa, xb, Y_IN - 4.6, Y_IN + 0.3, bz0, bz0 + BH["h"] + 0.5))
    ta.append(box(bx0 - 1.2, bx1 + 1.2, Y_IN - 4.6, Y_IN + 0.3, bz0 - 1.2, bz0))
    for xa, xb in ((bx0, bx0 + 1.0), (bx1 - 1.0, bx1)):
        ta.append(box(xa, xb, Y_IN - 1.9, Y_IN + 0.3, bz0, bz0 + BH["h"] + 0.5))          # 靠墙的小台阶（芯片悬空）
        ta.append(box(xa, xb, Y_IN - 4.6, Y_IN - 4.0, bz0, bz0 + BH["h"] + 0.5))          # 外侧压边
    # ---- 毫米波：顶盖里面两条带压边的导轨，板子从前面推进去
    y = LD["w"] / 2 + 0.2
    for sy in (1, -1):
        ys = sorted((sy * y, sy * (y + 1.5)))
        ca.append(box(LD["x0"] - 1.5, LD["x1"] + 1.0, ys[0], ys[1], LD["z_top"] - 3.2, LD["z_top"] + 0.3))
        yl = sorted((sy * (y - 0.8), sy * y))
        ca.append(box(LD["x0"] - 1.5, LD["x1"] + 1.0, yl[0], yl[1], LD["z_top"] - 3.2, LD["z_top"] - 2.5))
    ca.append(box(LD["x0"] - 1.5, LD["x0"], -y - 1.5, y + 1.5, LD["z_top"] - 3.2, LD["z_top"] + 0.3))
    # ---- 托板：ESP32 托架（后端垫块 + 前端卡槽）、摄像头主板 4 根柱、功放 2 根柱 + 1 个托点
    ra.append(box(ESP["x0"] - 4, ESP["x0"] + 5, ESP["y0"] - 1.5, ESP["y1"] + 1.5, TRAY_TOP - 0.1, ESP["z0"]))
    ra.append(box(ESP["x1"] - 2.2, ESP["x1"] + 0.6, ESP["y0"] - 1.5, ESP["y1"] + 1.5, TRAY_TOP - 0.1, ESP["z0"]))      # 前端托条
    ra.append(box(ESP["x1"] + 0.6, ESP["x1"] + 3.3, ESP["y0"] - 1.5, ESP["y1"] + 1.5, TRAY_TOP - 0.1, ESP["z0"] + 3.5))  # 前端挡板（天线那头），板子用扎带绑在托板上
    for x, y in CAMB_HOLES:
        ra.append(cyl_z(x, y, 5, TRAY_TOP - 0.1, CAMB["z0"]))
        rc.append(cyl_z(x, y, 1.7, TRAY_TOP - 1, CAMB["z0"] + 1))
    for x, y in MAX_HOLES:
        ra.append(cyl_z(x, y, 5, TRAY_TOP - 0.1, MAXB["z0"]))
        rc.append(cyl_z(x, y, 2.1, TRAY_TOP - 1, MAXB["z0"] + 1))
    ra.append(cyl_z(31.0, -42.0, 5, TRAY_TOP - 0.1, MAXB["z0"]))
    for y0 in (ESP["y0"] - 3.0, ESP["y1"] + 1.0):                                       # ESP32 扎带槽
        rc.append(box(8.0, 12.0, y0, y0 + 2.0, TRAY_TOP - 3, TRAY_TOP + 1))
    return dict(tub_add=ta, tub_cut=tc, cap_add=ca, cap_cut=cc, tray_add=ra, tray_cut=rc, bezel=bezel)


def head_parts():
    """屏幕模块里买来的电子件（俯仰轴坐标），装配检查和爆炸图用。"""
    P = []
    P.append(("摄像头镜头模组", lbox(-CAM["u"] / 2, CAM["u"] / 2, CAM_V - CAM["v"] / 2, CAM_V + CAM["v"] / 2, 2.55, 2.55 + CAM["h"])))
    b = TOF_BRD
    tof = lbox(b["u0"], b["u1"], b["v0"], b["v1"], b["w0"], b["w1"]).fuse(lbox(-1.5, 1.5, TOF_V - 3.2, TOF_V + 3.2, 2.55, b["w0"]))
    for u, v in TOF_HOLES:
        tof = tof.cut(lcyl(u, v, 3.3, b["w0"] - 1, b["w1"] + 1))
    P.append(("多点测距 VL53L5CX", tof))
    P.append(("隐私灯 3mm", lcyl(0, LED_V, 3.0, 0.5, 5.8).fuse(lcyl(0, LED_V, 3.8, 5.8, 6.8))))
    for i, (x, z) in enumerate(MICS):
        P.append((f"麦克风{i + 1}", cyl_y(x, z, 13.8, Y_IN - 2.0, Y_IN - 1.0).fuse(box(x - 2, x + 2, Y_IN - 3.0, Y_IN - 2.0, z - 1.5, z + 1.5))))
    bz0 = BH_CHIP[1] - BH["chip_from_bottom"]
    P.append(("光线 GY-302", box(BH_CHIP[0] - BH["w"] / 2, BH_CHIP[0] + BH["w"] / 2, Y_IN - 3.5, Y_IN - 1.9, bz0, bz0 + BH["h"])
              .fuse(box(BH_CHIP[0] - 1.5, BH_CHIP[0] + 1.5, Y_IN - 1.9, Y_IN - 1.0, BH_CHIP[1] - 1, BH_CHIP[1] + 1))))
    P.append(("毫米波 LD2410B", box(LD["x0"], LD["x1"], -LD["w"] / 2, LD["w"] / 2, LD["z_top"] - 2.4, LD["z_top"] - 0.1)))
    P.append(("ESP32-S3-DevKitC-1", box(ESP["x0"], ESP["x1"], ESP["y0"], ESP["y1"], ESP["z0"], ESP["z0"] + 1.6)
              .fuse(box(ESP["x0"] + 25, ESP["x1"], ESP["y0"] + 3, ESP["y1"] - 3, ESP["z0"] + 1.6, ESP["z0"] + 4.8))))
    P.append(("摄像头主板", box(CAMB["x0"], CAMB["x1"], CAMB["y0"], CAMB["y1"], CAMB["z0"], CAMB["z0"] + 3.6)))
    P.append(("功放 MAX98357A", box(MAXB["x0"], MAXB["x1"], MAXB["y0"], MAXB["y1"], MAXB["z0"], MAXB["z0"] + 3.0)))
    return P


# ================================================================ 右耳方窗（脖子坐标，按 +Y 画）
UP0, UP1 = (78.0, 62.0), (50.0, 122.0)
_Ln = math.hypot(UP1[0] - UP0[0], UP1[1] - UP0[1])
D_S = ((UP1[0] - UP0[0]) / _Ln, (UP1[1] - UP0[1]) / _Ln)          # 沿斜面
N_E = ((UP1[1] - UP0[1]) / _Ln, -(UP1[0] - UP0[0]) / _Ln)          # 斜面朝外
S0 = 0.42 * _Ln                                                      # 方窗中心


def ear(s, d):
    """方窗局部坐标 → (x, z)：s 沿斜面（0 = 方窗中心），d 从斜面往里。"""
    return (UP0[0] + D_S[0] * (S0 + s) - N_E[0] * d, UP0[1] + D_S[1] * (S0 + s) - N_E[1] * d)


def ebox(s0, s1, y0, y1, d0, d1):
    return prism_xz([ear(s0, d0), ear(s1, d0), ear(s1, d1), ear(s0, d1)], y0, y1)


def ecyl(s, y, dia, d0, d1):
    x, z = ear(s, d0)
    return cyl_dir((x, y, z), (-N_E[0], 0, -N_E[1]), dia, d1 - d0)


POCKET_S = (-14.5, 14.5)
WALL_E = 3.0
MLX = dict(s=-5.5, y=89.0, board_d0=5.1)           # 探头罐子伸进 3 mm 墙上的 Φ10.4 孔，前面离外表面 0.5
MLX_HOLES = [(MLX["s"] + ds, MLX["y"] + dy) for ds in (-6.35, 6.35) for dy in (-10.16, 10.16)]
IR_TX = dict(s=9.0, y=84.0)
IR_RX = dict(s=9.0, y=94.5, w=6.0, h=7.0, d=5.0)
SHT = dict(s=0.0, d=20.0)                           # 温湿度板拧在中板内侧（y = 104），25.4 沿斜面
SHT_HOLES = [(SHT["s"] + ds, SHT["d"] + dd) for ds in (-10.16, 10.16) for dd in (-6.35, 6.35)]
WIN_HOLES = [(MLX["s"], MLX["y"], 10.4), (IR_TX["s"], IR_TX["y"], 5.2), (IR_RX["s"], IR_RX["y"], "rect")]
RECT = (2.5, 2.75)                                  # 接收头方孔半宽（沿斜面, y）


def ear_R_core():
    """-Y 耳朵骨架（按 +Y 画）里的传感器仓：返回 (cut, add)。"""
    cut, add = [], []
    cut.append(ebox(POCKET_S[0], POCKET_S[1], 76.0, 102.5, WALL_E, 31.0))
    cut.append(ebox(POCKET_S[0], POCKET_S[1], 76.0, 104.1, 4.5, 31.0))               # 从中板这面装零件（离斜面 4.5 以内留 1.5 的皮）
    for s, y in MLX_HOLES:                                                            # 热成像：4 根定位柱，烫头固定
        add.append(ecyl(s, y, 5.0, WALL_E - 0.5, MLX["board_d0"]))
        add.append(ecyl(s, y, 2.3, MLX["board_d0"] - 0.1, MLX["board_d0"] + 1.6 + 1.8))
    add.append(ecyl(IR_TX["s"], IR_TX["y"], 8.0, WALL_E - 0.5, 8.9))                 # 红外发射管座：灯头刚好在外表面里 0.3
    cut.append(ecyl(IR_TX["s"], IR_TX["y"], 5.1, 0.0, 8.95))
    cut.append(ecyl(IR_TX["s"], IR_TX["y"], 6.0, 8.9, 12))
    hs, hy = IR_RX["w"] / 2 + 0.2, IR_RX["h"] / 2 + 0.2                               # 接收头：方框卡住，正面贴墙
    fr = ebox(IR_RX["s"] - hs - 1.2, IR_RX["s"] + hs + 1.2, IR_RX["y"] - hy - 1.2, IR_RX["y"] + hy + 1.2, WALL_E - 0.5, WALL_E + IR_RX["d"])
    fr = fr.cut(ebox(IR_RX["s"] - hs, IR_RX["s"] + hs, IR_RX["y"] - hy, IR_RX["y"] + hy, WALL_E - 0.1, WALL_E + IR_RX["d"] + 1))
    add.append(fr)
    for s, y, kind in WIN_HOLES:
        if kind == "rect":
            cut.append(ebox(s - RECT[0], s + RECT[0], y - RECT[1], y + RECT[1], -1, WALL_E + 0.1))
        else:
            cut.append(ecyl(s, y, kind, -1, WALL_E + 0.1))
    # 线：传感器仓 → 铜套后面 → 空心轴销 → 头里（一根 Φ6 斜孔）
    x0, z0 = 0.0, 66.0
    x1, z1 = ear(-10.0, 25.0)
    L = math.hypot(x1 - x0, z1 - z0)
    cut.append(cyl_dir((x0, 91.0, z0), ((x1 - x0) / L, 0, (z1 - z0) / L), 6.0, L))
    return cut, add


def ear_R_mid():
    """-Y 耳朵中板（按 +Y 画）：温湿度板的 4 根柱（M2.5 自攻）。返回 (cut, add)。"""
    cut, add = [], []
    for s, d in SHT_HOLES:
        x, z = ear(s, d)
        add.append(cyl_y(x, z, 5.0, 103.0, 104.1))
        cut.append(cyl_y(x, z, 2.1, 102.0, 108.0))
    return cut, add


def ear_R_parts():
    """-Y 耳朵里的电子件（按 +Y 画，调用方镜像）。"""
    P = []
    b0 = MLX["board_d0"]
    mlx = ebox(MLX["s"] - 8.89, MLX["s"] + 8.89, MLX["y"] - 12.7, MLX["y"] + 12.7, b0, b0 + 1.6)
    for s, y in MLX_HOLES:
        mlx = mlx.cut(ecyl(s, y, 2.5, b0 - 1, b0 + 3))
    mlx = mlx.fuse(ecyl(MLX["s"], MLX["y"], 9.2, 0.5, b0))
    for dy in (-10.16, 10.16):
        mlx = mlx.fuse(ebox(MLX["s"] - 3, MLX["s"] + 3, MLX["y"] + dy - 2.5, MLX["y"] + dy + 2.5, b0 + 1.6, b0 + 1.6 + 4.25))
    P.append(("热成像 MLX90640", mlx))
    P.append(("红外发射 5mm", ecyl(IR_TX["s"], IR_TX["y"], 5.0, 0.3, 8.9).fuse(ecyl(IR_TX["s"], IR_TX["y"], 5.8, 8.9, 9.9))))
    P.append(("红外接收 VS1838B", ebox(IR_RX["s"] - IR_RX["w"] / 2, IR_RX["s"] + IR_RX["w"] / 2, IR_RX["y"] - IR_RX["h"] / 2,
                                     IR_RX["y"] + IR_RX["h"] / 2, WALL_E + 0.05, WALL_E + IR_RX["d"])))
    sht = None
    cs, cd = ear(SHT["s"], SHT["d"])
    sht = prism_xz([ear(SHT["s"] - 12.7, SHT["d"] - 8.89), ear(SHT["s"] + 12.7, SHT["d"] - 8.89),
                    ear(SHT["s"] + 12.7, SHT["d"] + 8.89), ear(SHT["s"] - 12.7, SHT["d"] + 8.89)], 101.4, 103.0)
    for s, d in SHT_HOLES:
        x, z = ear(s, d)
        sht = sht.cut(cyl_y(x, z, 2.5, 100, 104))
    P.append(("温湿度 SHT40", sht))
    return P
