"""头部 3D 打印件（工程版，2026-10-21）：外形直接用定稿外观（shell.head / ears / neck），只在里面加真正能装的东西——
拆分线、螺丝孔、热熔螺母孔、电路板卡槽和压框、舵机仓、走线孔。外面看到的样子不变。

用到的零件尺寸都来自厂家图纸（007v/ref/）：
- 微雪 ESP32-S3-Touch-LCD-4.3B：玻璃 112.4 × 75.1（R4），玻璃 + 屏 4.8 厚，到 PCB 背面 9.1，最厚 17.4（接线端子）；
  PCB 104 × 56，4 个 M2 孔：上两个相距 92（离 PCB 上边 3），下两个比上面低 50（左 -46、右 +45）
- 飞特 STS3215：机身 45.22 × 24.72，机壳上下面相距 32；输出轴离一头 10.11；舵盘 Φ19.2、厚 2.5，4 个 M2 孔在 Φ14 圆上（成 45°），中心一颗 M3；
  机壳上面 4 个 M2 螺丝孔（离输出轴那头 18.41 / 42.86，左右相距 20.5）
- 乐动 LD19：安装脚 54 × 46.29，3 个 Φ2.5 孔（见 config.LD19）

用法：python 007v/head_print.py          → 007v/out/head_print/（每个零件 STL = 打印摆放；STEP = 装配位置）+ 装配检查
"""
import json
import math
import os
import sys

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import head_electronics as HE  # noqa: E402
import shell  # noqa: E402
from config import (HEAD_PROFILE, HEAD_W, LD19, PIVOT_UP, POD_HEX, POD_IN, POD_W, SCREEN_GLASS, SCREEN_X,  # noqa: E402
                    SCREEN_Z)
from shell import V, box, cyl_dir, cyl_y, cyl_z, mirror_y, prism_xz, soft, strip  # noqa: E402

OUT = os.path.join(HERE, "out", "head_print")
REF = os.path.join(HERE, "ref")

# 热熔铜螺母：(孔径, 孔深)；螺丝过孔；沉头头径
INS = {"M2": (3.2, 4.0), "M3": (4.0, 5.5), "M4": (5.6, 6.5), "M5": (6.4, 8.5)}
CLR = {"M2": 2.4, "M2.5": 2.9, "M3": 3.4, "M4": 4.5, "M5": 5.5}
CSK = {"M3": 6.4, "M4": 8.6, "M5": 10.2}

HW = HEAD_W / 2                     # 63：屏幕模块半宽
WALL = 2.5
FRONT = max(x for x, _ in HEAD_PROFILE)          # 84
SPLIT_X, SPLIT_Z = FRONT - 14, 44.0              # 顶盖 = x < 70 且 z > 44（正好是外观上那两道分缝）

# 屏幕板（俯仰轴坐标；板子坐标 bx 右、by 上、bz 朝前 → x = 76.2 + bz，y = bx，z = 15 + 1.94 + by）
GL_W, GL_H, GL_T = 112.4, 75.1, 4.8
GL_X0 = SCREEN_X - GL_T                          # 76.2：玻璃背面；玻璃前面 = 81，正好顶住亚克力窗
PCB_BACK = SCREEN_X - 9.1                        # 71.9
BRD_DY = 1.94                                    # 玻璃中心比板子模型原点低 1.94


def brd(bx, by):
    return bx, SCREEN_Z + BRD_DY + by


PCB_HOLES = [brd(-46, 28), brd(46, 28), brd(-46, -22), brd(45, -22)]

# 舵机（耳朵 / 脖子坐标）：输出轴在俯仰轴上，朝 -Y（朝屏幕）；舵机长边沿 x，输出轴那头朝后
SV_TOP = 73.0                       # 机壳上面凸起盖板（比机壳面高 1.1）的 y；舵机模型里机壳面 z=15.9、凸起 17.0、舵盘 16.2..18.7
SV_L0, SV_L1 = -10.2, 35.2           # 机壳 x 范围（输出轴离后端 10.11）
SV_H = 24.72
HORN_FACE = SV_TOP - 1.7             # 71.3：舵盘外面（耳朵内壁只剩 1 mm，给背面插头多留空间）
SV_SCREWS = [(8.21, PIVOT_UP + 10.25), (8.21, PIVOT_UP - 10.25)]     # 机壳上面靠输出轴的两个 M2 孔（另两个露在外面，不用）

HEX = POD_HEX
YI, YO = POD_IN, POD_IN + POD_W       # 72 / 120
CORE1, MID1, OUT1 = 104.0, 110.0, YO - 4  # 三层板分界
TOP_X = sum(x for x, z in HEX if z == max(zz for _, zz in HEX)) / 2   # 21：耳顶中间
TOP_Z = max(z for _, z in HEX)                                          # 122
FL_Z = TOP_Z + 15.0                                                     # 雷达法兰上面
CH_Y = 88.0                          # 耳朵里走线孔的 y
LIDAR_CH = (37.0, 88.0)              # 雷达线往下走的孔（在雷达底座下面，看不见）
EAR_M4 = [(-12.0, 88.0), (42.0, 80.0)]                                  # 耳朵底面 → 脖子底座（M4，从下往上拧）
EAR_M5 = [(-12, 30), (36, 30), (-2, 108), (34, 108)]                    # 外板四角（外观上的 4 颗螺栓）
FL_M3 = [(TOP_X - 20, 78.0), (TOP_X + 20, 78.0), (TOP_X - 20, 100.0), (TOP_X + 20, 100.0)]


# ---------------------------------------------------------------- 小工具
def cyl_x(cy, cz, d, x0, x1):
    return cyl_dir((x0, cy, cz), (1, 0, 0), d, x1 - x0)


def unit(n):
    L = math.sqrt(sum(c * c for c in n))
    return tuple(c / L for c in n)


def csk_hole(p, n, size, L):
    """沉头螺丝孔：p 在零件表面，n 朝零件里面；表面往外再多切 2 mm。"""
    n = unit(n)
    dh, dc = CSK[size], CLR[size]
    h = (dh - dc) / 2
    cone = cq.Solid.makeCone(dh / 2, dc / 2, h, V(*p), V(*n))
    out = cyl_dir(tuple(p[i] - n[i] * 2 for i in range(3)), n, dh, 2.01)
    return cone.fuse(out).fuse(cyl_dir(p, n, dc, L))


def insert_hole(p, n, size, extra=0.0):
    """热熔螺母孔：p 在表面，n 朝里。"""
    d, L = INS[size]
    n = unit(n)
    return cyl_dir(tuple(p[i] - n[i] * 0.5 for i in range(3)), n, d, L + 0.5 + extra)


def envelope(y0=-HW, y1=HW):
    return prism_xz(HEAD_PROFILE, y0, y1)


def named(parts):
    return {n: s for n, s, _ in parts}


def fuse_all(base, items):
    for s in items:
        base = base.fuse(s)
    return base


def cut_all(base, items, tag=""):
    """逐个切；某一刀布尔运算失败时换个顺序（先把刀具合并）再试，还不行就报出来。"""
    for i, c in enumerate(items):
        try:
            base = base.cut(c)
        except Exception:
            try:
                base = base.clean().cut(c.clean())
            except Exception as e:
                raise RuntimeError(f"{tag} 第 {i} 刀失败：{e}")
    return base


# ================================================================ 屏幕模块
def screen_module():
    H = named(shell.head())
    S = H["屏幕头"]
    # 外观上的小螺栓：侧面 x-21 z35 那两颗是真螺丝（固定顶盖），其余照样打印出来（装饰）
    deco = [s for n, s in H.items() if ("螺栓" in n and "x-21z35" not in n)]
    cap_reg = box(-45, SPLIT_X, -80, 80, SPLIT_Z, 90)
    tub = S.cut(cap_reg)
    cap = S.intersect(cap_reg)
    tub = fuse_all(tub, [d for d in deco])
    cap = cap.fuse(H["外壳·屏幕模块小字"])
    inner = prism_xz(HEAD_PROFILE, -HW + WALL, HW - WALL, offset=-WALL)
    env = envelope()

    add = []
    for sy in (1, -1):
        def my(s):
            return s if sy > 0 else mirror_y(s)
        add.append(my(box(76.0, 80.9, GL_W / 2 + 0.4, HW - WALL + 0.3, -20, SPLIT_Z - 1)))        # 玻璃左右导槽（前面让开亚克力窗）
        for zc in (-5.5, 39.5):                                                                    # 压框螺丝座（M3，从后往前拧）
            add.append(my(box(64.0, 74.5, 53.5, HW - WALL + 0.3, zc - 4.5, zc + 4.5)))
        add.append(my(box(70.0, 74.0, 53.5, HW - WALL + 0.3, 46, 56)))                             # 顶盖前端磁铁座
    add.append(box(76.0, 81.6, -50, 50, -32, -22.85).intersect(env))                              # 玻璃下沿托条
    add.append(cyl_y(0, 0, 20.5, HW - WALL - 0.2, HORN_FACE))                                     # +Y 舵盘连接座（伸进转轴壳里）
    add.append(cyl_y(0, 0, 16, -HW + WALL - 0.2, -HW + WALL + 8.2))                                # -Y 轴销座
    add.append(shell.ring_y(41, -21, 32, 28.6, -HW + WALL - 0.2, -HW + WALL + 3.8))               # -Y 喇叭圈（Φ28 喇叭）
    for x in (-12.0, 48.0):                                                                        # 电子托板螺丝柱
        for y in (-38.0, 38.0):
            add.append(cyl_z(x, y, 8, -40.5, -35.0))
    EM = HE.head_mounts()
    tub = fuse_all(tub, add + EM["tub_add"])

    cut = list(EM["tub_cut"])
    cut.append(cyl_y(0, 0, 6.0, HW - WALL - 3, HORN_FACE + 1))                                     # 中心孔：从头里伸螺丝刀拧舵盘中心螺丝
    cut.append(cyl_y(0, 0, 7.6, HORN_FACE - 1.8, HORN_FACE + 1))                                  # 让开舵盘中心螺丝头
    for dx in (-4.95, 4.95):
        for dz in (-4.95, 4.95):
            cut.append(cyl_y(dx, dz, 1.8, HORN_FACE - 6, HORN_FACE + 1))                           # M2 自攻进舵盘：先打底孔
            cut.append(cyl_y(dx, dz, 4.2, HW - WALL - 3, HORN_FACE - 3.5))                         # 沉孔：螺丝头在头壳里面
    cut.append(cyl_y(0, 0, 7.9, -HW + WALL - 0.7, -HW + WALL + 8.5))                               # 空心轴销压入孔（打通：右耳传感器的线从轴销里穿进头）
    cut.append(cyl_x(49, 22, 10, -36, -22))                                                        # 背面出线孔（编织线从这里进头）
    for sy in (1, -1):
        p = (-21.0, sy * HW, 35.0)
        cut.append(csk_hole(p, (0, -sy, 0), "M5", 6))                                              # 顶盖螺丝（外观上的螺栓位置）
        for zc in (-5.5, 39.5):
            cut.append(insert_hole((64.0, sy * 57.0, zc), (1, 0, 0), "M3"))
        cut.append(cyl_x(sy * 57.0, 51.0, 6.2, 70.0 - 0.01, 73.2))                                  # Φ6×3 磁铁
    for x in (-12.0, 48.0):
        for y in (-38.0, 38.0):
            cut.append(insert_hole((x, y, -35.0), (0, 0, -1), "M3"))
    tub = cut_all(tub, cut, "主壳")

    # 顶盖：后面两块往下伸的螺母板（M5 热熔螺母）+ 前面两块磁铁座
    cadd = []
    for sy in (1, -1):
        def my(s):
            return s if sy > 0 else mirror_y(s)
        cadd.append(my(box(-25, -15, 49.5, HW - WALL - 0.2, 28.5, SPLIT_Z + 0.5)))
        cadd.append(my(box(-25, -15, 49.5, HW - WALL + 0.3, SPLIT_Z, 70)).intersect(env))
        cadd.append(my(box(64.0, SPLIT_X, 53.5, HW - WALL + 0.3, 46, 56)))
    cap = fuse_all(cap, cadd + EM["cap_add"])
    for sy in (1, -1):
        cap = cap.cut(insert_hole((-21.0, sy * (HW - WALL - 0.2), 35.0), (0, -sy, 0), "M5"))
        cap = cap.cut(cyl_x(sy * 57.0, 51.0, 6.2, 66.8, SPLIT_X + 0.01))

    # 屏板压框：两根竖条 + 4 个压脚（带 Φ1.7 定位销插进 PCB 的 M2 孔）+ 4 个耳朵（M3 拧在螺丝座上）
    ret = []
    for sy, yb in ((-1, -46.0), (1, 45.5)):
        ret.append(box(62.0, 67.0, yb - 3, yb + 3, -9, 48))
        for zc in (-5.5, 39.5):
            y0, y1 = sorted((yb, sy * (HW - WALL - 0.2)))
            ret.append(box(60.0, 64.0, y0, y1, zc - 4.5, zc + 4.5))
    for y, z in PCB_HOLES:
        ret.append(cyl_x(y, z, 5.0, 66.0, PCB_BACK))
        ret.append(cyl_x(y, z, 1.7, PCB_BACK - 0.1, PCB_BACK + 1.3))
    retainer = fuse_all(ret[0], ret[1:])
    for sy in (1, -1):
        for zc in (-5.5, 39.5):
            retainer = retainer.cut(cyl_x(sy * 57.0, zc, CLR["M3"], 58, 66))

    # 电子托板：洞洞板（M3 孔 10 mm 间距 + 扎带槽），树莓派以外的小板都用 M3 铜柱或扎带固定在上面
    tray = box(-18, 54, -46, 46, -35, -32.5)
    for x in range(-13, 54, 10):
        for y in range(-40, 41, 10):
            if (x, y) in ((-13, -40), (-13, 40), (47, -40), (47, 40)):
                continue
            tray = tray.cut(cyl_z(x, y, CLR["M3"], -36, -30))
    for x in (-12.0, 48.0):
        for y in (-38.0, 38.0):
            tray = tray.cut(cyl_z(x, y, CLR["M3"], -36, -30))
    for x in (-2, 22, 38):                                                                         # 扎带槽
        for y0 in (-44.0, 40.0):
            tray = tray.cut(box(x - 2, x + 2, y0, y0 + 4, -36, -30))
    tray = cut_all(fuse_all(tray, EM["tray_add"]), EM["tray_cut"], "托板")

    frame = H["外壳·屏框"]
    for n_, s_ in H.items():
        if n_.startswith("外壳·前框螺栓"):
            c = s_.Center()
            frame = frame.cut(cyl_x(c.y, c.z, 10.8, FRONT - 0.1, FRONT + 0.8))                     # 屏框背面让开装饰螺栓
    return dict(tub=tub, cap=cap, retainer=retainer, tray=tray, frame=frame, bezel=EM["bezel"])


# ================================================================ 耳朵（脖子坐标，先按 +Y 那只画，-Y 那只最后镜像）
def ear_core(servo):
    core = soft(prism_xz(HEX, YI, CORE1), 2.0)
    for x, y in EAR_M4:                                                                              # 底下垫到脖子底座上（差 2 mm）
        core = core.fuse(cyl_z(x, y, 12, 14.0, 16.5))
    cut = []
    if servo:
        cut.append(box(SV_L0 - 0.25, SV_L1 + 0.25, SV_TOP, CORE1 + 1, PIVOT_UP - SV_H / 2 - 0.2, PIVOT_UP + SV_H / 2 + 0.2))
        cut.append(cyl_y(0, PIVOT_UP, 22, YI - 1, SV_TOP + 0.5))                                    # 舵盘转动的空间
        for x, z in SV_SCREWS:
            cut.append(cyl_y(x, z, CLR["M2"], YI - 1, SV_TOP + 0.5))
        cut.append(cyl_y(-26, 70, 10, YI - 1, CH_Y))                                                 # 头里来的编织线从内侧面进耳朵
        cut.append(cyl_z(-26, CH_Y, 10, 38, 70))
        cut.append(cyl_x(CH_Y, 40, 10, -26, SV_L1 + 5.3))
        cut.append(box(SV_L1, SV_L1 + 5.3, CH_Y - 6, CORE1 + 1, 38, PIVOT_UP + SV_H / 2))          # 舵机前端一条竖槽：舵机线、雷达线都从这里下去
        cut.append(cyl_z(LIDAR_CH[0], LIDAR_CH[1], 8, 54, TOP_Z + 1))                               # 雷达线从耳顶下来
        for x, y in FL_M3:
            cut.append(insert_hole((x, y, TOP_Z), (0, 0, -1), "M3"))
    else:
        cut.append(cyl_y(0, PIVOT_UP, 9.95, YI - 1, YI + 18))                                       # 铜套 Φ10×18 压入
        cut.append(cyl_z(30, CH_Y, 10, 15, 88))
        rc, ra = HE.ear_R_core()
        core = cut_all(core, rc[:2], "耳骨架仓")                                                     # 先挖仓，再加柱子和座，最后打孔
        core = fuse_all(core, ra)
        cut += rc[2:]
    for x, z in EAR_M5:
        cut.append(insert_hole((x, CORE1, z), (0, -1, 0), "M5"))
    for i in range(6):
        a = 2 * math.pi * i / 6
        if servo and i == 0:
            continue                                                                                  # 这一颗正好在舵机仓上，做装饰
        cut.append(insert_hole((18 * math.cos(a), YI, PIVOT_UP + 18 * math.sin(a)), (0, 1, 0), "M2"))
    for x, y in EAR_M4:
        cut.append(insert_hole((x, y, 14.0), (0, 0, 1), "M4"))
    if servo:
        cut.append(cyl_z(30, CH_Y, 10, 13, 42))
    core = cut_all(core, cut, "耳骨架")
    return core


WIN_HOLES = HE.WIN_HOLES     # 方窗里的开孔：(沿斜面的位置 mm, y, 直径或 "rect")


def ear_mid(servo):
    m = soft(prism_xz(HEX, CORE1, MID1, offset=-4), 1.0)
    if servo:
        m = m.cut(box(SV_L0 - 0.25, SV_L1 + 0.25, CORE1 - 1, MID1 + 1, PIVOT_UP - SV_H / 2 - 0.2, PIVOT_UP + SV_H / 2 + 0.2))
        m = m.cut(box(SV_L1, SV_L1 + 5.3, CORE1 - 1, MID1 + 1, 54, PIVOT_UP + SV_H / 2))
    else:
        mc, ma = HE.ear_R_mid()
        m = cut_all(fuse_all(m, ma), mc, "中板")
    for x, z in EAR_M5:
        m = m.cut(cyl_y(x, z, CLR["M5"], CORE1 - 1, MID1 + 1))
    return m


def ear_outer(outer, servo):
    o = outer
    if servo:
        o = o.fuse(box(21, SV_L1 - 1, 107.9, MID1 + 0.5, PIVOT_UP - 9, PIVOT_UP + 9))                # 压住舵机底面的凸起盖板
        o = o.cut(cyl_y(0, PIVOT_UP, 22, 107.9, MID1 + 1.5))                                         # 让开舵机背面的从动舵盘
        o = o.cut(box(8.5, SV_L1 + 5.3, MID1 - 0.1, OUT1 - 1.5, 54, 78))                             # 舵机背面两个插头 + 线（留 8.6 mm 高）
    for x, z in EAR_M5:
        o = o.cut(csk_hole((x, OUT1, z), (0, -1, 0), "M5", 14))
    return o


def hub(outer_y, servo):
    """转轴壳 + 法兰（+Y 那只孔大，让开舵盘）。"""
    h = cyl_y(0, PIVOT_UP, 30, HW + 1.0, YI).fuse(cyl_y(0, PIVOT_UP, 44, YI - 2.5, YI))
    h = h.cut(cyl_y(0, PIVOT_UP, 22 if servo else 12, HW, YI + 1))
    for i in range(6):
        a = 2 * math.pi * i / 6
        x, z = 18 * math.cos(a), PIVOT_UP + 18 * math.sin(a)
        if servo and i == 0:
            h = h.fuse(cyl_y(x, z, 4, YI - 3.5, YI - 2.5))
            continue
        h = h.cut(cyl_y(x, z, CLR["M2"], YI - 4, YI + 1))
    if servo:
        for x, z in SV_SCREWS:
            h = h.cut(cyl_y(x, z, 4.4, YI - 1.7, YI + 0.1))                                           # 让开舵机 M2 螺丝头
    return h


def ears_and_neck():
    E, (lx, ly, lz) = shell.ears()
    E = named(E)
    N = named(shell.neck()[0])
    P = {}
    P["core_L"] = ear_core(True)
    P["core_R"] = mirror_y(ear_core(False))
    P["mid_L"] = ear_mid(True)
    P["mid_R"] = mirror_y(ear_mid(False))
    deco_L = [s for n, s in E.items() if n.startswith("外壳·左侧盖螺栓")]
    deco_R = [s for n, s in E.items() if n.startswith("外壳·右侧盖螺栓")]
    P["outer_L"] = ear_outer(fuse_all(E["左耳外板"], deco_L), True)
    P["outer_R"] = mirror_y(ear_outer(mirror_y(fuse_all(E["右耳外板"], deco_R)), False))
    P["hub_L"] = hub(YI, True)
    P["hub_R"] = mirror_y(hub(YI, False))
    P["ring"] = cyl_y(0, PIVOT_UP, 34, HW + 2.5, HW + 4.5).cut(cyl_y(0, PIVOT_UP, 30.3, HW, HW + 6))
    # 方窗面板（黑色，带传感器开孔，粘在 -Y 耳朵斜面上）
    win = mirror_y(E["外壳·方窗（热成像 / 红外）"]).fuse(mirror_y(E["外壳·方窗边框"]))
    up0, up1 = (78.0, 62.0), (50.0, 122.0)
    Ln = math.hypot(up1[0] - up0[0], up1[1] - up0[1])
    n = ((up1[1] - up0[1]) / Ln, -(up1[0] - up0[0]) / Ln)
    for (s, y, kind) in WIN_HOLES:
        t = 0.42 + s / Ln
        p = (up0[0] + (up1[0] - up0[0]) * t + n[0] * 3, up0[1] + (up1[1] - up0[1]) * t + n[1] * 3)
        if kind == "rect":
            win = win.cut((HE.ebox(s - HE.RECT[0], s + HE.RECT[0], y - HE.RECT[1], y + HE.RECT[1], -8, 4)))
        else:
            win = win.cut(cyl_dir((p[0], y, p[1]), (-n[0], 0, -n[1]), kind, 8))
    P["window"] = mirror_y(win)
    P["core_R"] = P["core_R"].cut(P["window"])                                                    # 骨架斜面上让出面板的凹位
    # 雷达法兰：4 颗 M3 沉头拧进耳顶；雷达 3 颗 M2 自攻（前两个孔离边太近，打不了热熔螺母）
    fl = E["雷达法兰"]
    for x, y in FL_M3:
        fl = fl.cut(csk_hole((x, y, FL_Z), (0, 0, -1), "M3", 18))
    for u, v in LD19["holes"]:
        fl = fl.cut(cyl_z(TOP_X + u, 96 + v, 1.7, FL_Z - 9, FL_Z + 1))
    fl = fl.cut(cyl_z(LIDAR_CH[0], LIDAR_CH[1], 8, TOP_Z - 1, FL_Z + 1))
    P["flange"] = fl
    P["lidar_ring"] = fuse_all(E["雷达护环"], [s for n, s in E.items() if n.startswith("外壳·护环支柱")])
    # 脖子底座：耳朵 4 颗 M4 沉头（从底下拧），底座 4 颗 M5 沉头拧到机身，两个走线孔
    nb = N["脖子底座"]
    for sy in (1, -1):
        for x, y in EAR_M4:
            nb = nb.cut(csk_hole((x, sy * y, 0.0), (0, 0, 1), "M4", 16))
        nb = nb.cut(cyl_z(30, sy * CH_Y, 12, -1, 16))
    for n_, s in N.items():
        if n_.startswith("外壳·底座螺栓"):
            c = s.Center()
            nb = nb.cut(csk_hole((c.x, c.y, 14.0), (0, 0, -1), "M5", 16))
    P["neck"] = nb
    return P


# ================================================================ 买来的零件（装配检查用）
def servo_solid():
    s = cq.importers.importStep(os.path.join(REF, "sts3215", "STS3215_03a.step")).val()
    # 舵机自己的坐标：输出轴 (12.5, 0)，轴朝 +z，机壳上面 z = 15.9 → 耳朵坐标：x = 12.5 - xs，y = 74 + 15.9 - zs，z = 66 - ys
    s = s.rotate(V(0, 0, 0), V(1, 0, 0), 90).rotate(V(0, 0, 0), V(0, 1, 0), 180)   # (x, y, z) → (-x, -z, -y)
    return s.translate(V(12.5, SV_TOP + 17.0, PIVOT_UP))


def board_solids():
    """屏幕板：厂家 3D 模型（转到俯仰轴坐标，第一次跑时缓存）。"""
    cache = os.path.join(REF, "waveshare_4.3B", "board_in_head.step")
    if not os.path.exists(cache):
        s = cq.importers.importStep(os.path.join(REF, "waveshare_4.3B", "ESP32-S3-Touch-LCD-4_3-B.stp")).val()
        s = s.rotate(V(0, 0, 0), V(1, 1, 1), 120).translate(V(GL_X0, 0, SCREEN_Z + BRD_DY))     # (bx, by, bz) → (bz, bx, by)
        cq.exporters.export(cq.Workplane().add(s), cache)
    return [cq.importers.importStep(cache).val()]


def bought():
    """装配位置的外购件：(名字, 实体, 坐标系 'head' 俯仰轴 / 'neck' 脖子)。"""
    gw, gh = SCREEN_GLASS
    B = [("亚克力窗 117.6×49.6×3", box(81.0, 84.0, -58.8, 58.8, SCREEN_Z - 24.8, SCREEN_Z + 24.8), "head"),
         ("空心轴销 Φ8×Φ5×35", cyl_y(0, 0, 8, -88, -53).cut(cyl_y(0, 0, 5, -90, -50)), "head"),
         ("喇叭 Φ28", cyl_y(41, -21, 28, -60.3, -55), "head")]
    for i, b in enumerate(board_solids()):
        B.append((f"屏幕板{i}", b, "head"))
    B.append(("舵机 STS3215", servo_solid(), "neck"))
    B.append(("铜套 Φ10×18", cyl_y(0, PIVOT_UP, 10, -YI - 18, -YI), "neck"))
    B.append(("雷达 LD19", __import__("model").ld19().translate(V(TOP_X, 96, FL_Z)), "neck"))
    for n_, s_ in HE.head_parts():
        B.append((n_, s_, "head"))
    for n_, s_ in HE.ear_R_parts():
        B.append((n_, mirror_y(s_), "neck"))
    return B


# ================================================================ 打印摆放 + 导出
PRINT = {   # 编号, 名字, 数量, 材料, 坐标系, 打印时朝下的面（方向向量，转到 -Z）
    "tub": ("H01", "屏幕模块主壳", 1, "ASA 浅灰白", "head", (0, 0, -1)),
    "cap": ("H02", "屏幕模块顶盖", 1, "ASA 浅灰白", "head", (0, 0, 1)),
    "retainer": ("H03", "屏板压框", 1, "PETG", "head", (-1, 0, 0)),
    "tray": ("H04", "电子托板（洞洞板）", 1, "PETG", "head", (0, 0, -1)),
    "frame": ("H05", "屏框（黑）", 1, "PETG 黑 / 深灰", "head", (-1, 0, 0)),
    "core_L": ("H06", "电机耳骨架（+Y，装舵机）", 1, "ASA 浅灰白", "neck", (0, -1, 0)),
    "core_R": ("H07", "方窗耳骨架（-Y，装铜套和传感器）", 1, "ASA 浅灰白", "neck", (0, 1, 0)),
    "mid_L": ("H08", "耳朵中板（+Y）", 1, "ASA 浅灰白", "neck", (0, -1, 0)),
    "mid_R": ("H09", "耳朵中板（-Y）", 1, "ASA 浅灰白", "neck", (0, 1, 0)),
    "outer_L": ("H10", "耳朵外板（+Y，带散热槽）", 1, "ASA 浅灰白", "neck", (0, 1, 0)),
    "outer_R": ("H11", "耳朵外板（-Y）", 1, "ASA 浅灰白", "neck", (0, -1, 0)),
    "hub_L": ("H12", "转轴壳（+Y）", 1, "PETG 深灰", "neck", (0, 1, 0)),
    "hub_R": ("H13", "转轴壳（-Y）", 1, "PETG 深灰", "neck", (0, -1, 0)),
    "ring": ("H14", "转轴光环（透明，以后装灯）", 1, "PETG 透明", "neck", (0, 1, 0)),
    "window": ("H15", "方窗面板（黑，带传感器孔）", 1, "PETG 黑", "neck", (-0.906, 0, -0.423)),
    "flange": ("H16", "雷达法兰", 1, "PETG 深灰（或铝）", "neck", (0, 0, -1)),
    "lidar_ring": ("H17", "雷达护环（带 4 根柱）", 1, "PETG 深灰", "neck", (0, 0, 1)),
    "neck": ("H18", "脖子底座", 1, "PETG 深灰（或铝）", "neck", (0, 0, -1)),
    "bezel": ("H19", "相机镜头圈（黑）", 1, "PETG 黑", "head", (HE.N_OUT[0], 0, HE.N_OUT[1])),
}


def lay_flat(s, down):
    if down is None:
        return s
    d = cq.Vector(*down).normalized()
    t = cq.Vector(0, 0, -1)
    ax = d.cross(t)
    if ax.Length < 1e-6:
        if d.dot(t) < 0:
            s = s.rotate(V(0, 0, 0), V(1, 0, 0), 180)
    else:
        ang = math.degrees(math.acos(max(-1, min(1, d.dot(t)))))
        s = s.rotate(V(0, 0, 0), ax, ang)
    b = s.BoundingBox()
    return s.translate(V(-(b.xmin + b.xmax) / 2, -(b.ymin + b.ymax) / 2, -b.zmin))


def to_neck(s):
    """俯仰轴坐标 → 脖子坐标（平视）。"""
    return s.translate(V(0, 0, PIVOT_UP))


def main():
    os.makedirs(OUT, exist_ok=True)
    print("屏幕模块 …")
    P = screen_module()
    print("耳朵和底座 …")
    P.update(ears_and_neck())
    info = []
    asm = {}
    for k, (code, name, qty, mat, frame, down) in PRINT.items():
        s = P[k]
        ok = s.isValid()
        sa = to_neck(s) if frame == "head" else s
        asm[k] = sa
        base = os.path.join(OUT, f"{code}_{name}")
        cq.exporters.export(cq.Workplane().add(sa), base + ".step")
        cq.exporters.export(cq.Workplane().add(lay_flat(s, down)), base + ".stl", tolerance=0.03, angularTolerance=0.1)
        b = s.BoundingBox()
        info.append(dict(code=code, name=name, qty=qty, material=mat, size=[round(b.xlen, 1), round(b.ylen, 1), round(b.zlen, 1)],
                         volume_cm3=round(s.Volume() / 1000, 1), valid=ok))
        print(f"  {code} {name}  {b.xlen:.0f}×{b.ylen:.0f}×{b.zlen:.0f}  {s.Volume() / 1000:.1f} cm³  {'' if ok else '！实体有问题'}")
    json.dump(info, open(os.path.join(OUT, "parts.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)

    # ---------------- 装配检查：打印件之间、打印件和外购件之间，实体相交 > 2 mm³ 算撞
    print("装配检查 …")
    B = [(n, to_neck(s) if f == "head" else s) for n, s, f in bought()]
    for n, s in B:
        asm["买·" + n] = s
    names = list(asm)
    hits = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if a.startswith("买·") and b.startswith("买·"):
                continue
            A, Bs = asm[a], asm[b]
            ba, bb = A.BoundingBox(), Bs.BoundingBox()
            if ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin:
                continue
            try:
                v = A.intersect(Bs).Volume()
            except Exception:
                v = -1
            if v > 2 or v < 0:
                hits.append((a, b, round(v, 1)))
    for h in hits:
        print("  撞：", h)
    print(f"装配检查：{len(hits)} 处")
    json.dump(dict(hits=hits), open(os.path.join(OUT, "check.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    # 装配体（看图用）
    cq.exporters.export(cq.Workplane().add(cq.Compound.makeCompound([asm[k] for k in names])), os.path.join(OUT, "装配体_头部.step"))


if __name__ == "__main__":
    main()
