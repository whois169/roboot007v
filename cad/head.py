"""
RoboDog v0.3 脖子 + 头部（机身坐标，零位：头朝正前方、不抬不低）
  偏航（左右转头）：GM3506 云台电机，轴线竖直，在前髋座顶上
  俯仰（抬头低头）：GM4108 云台电机，轴线沿 Y，尽量靠近头部重心，静态力矩很小
头里的电子件：2.25 寸长条彩屏（眼睛）、ESP32-S3（语音前端 + 触摸 + 屏幕 + 传感器）、USB 集线器、
2×I2S 麦克风、功放 + 28mm 扬声器、BH1750 光照、隐私开关
屏幕下面一排：USB 相机镜头、VL53L5CX 测距窗、隐私指示灯；屏幕上方白边后面藏 LD2410 毫米波
"""
import math
import cadquery as cq
from config import *
from parts import V, box, cyl_x, cyl_y, cyl_z, xz_prism, chamfer_edges
from armor import loft, inset, octa

TOP = BODY_ZMAX

# 云台电机（iPower 公开参数）
GM3506 = dict(D=40.0, H=18.0)    # 偏航，80g，空心轴 6mm
GM4108 = dict(D=47.0, H=24.0)    # 俯仰，125g，约 0.12–0.18 N·m

YAW_X = HIP_X - 10               # 偏航轴 x=140
YAW_Z0 = TOP + 6                 # 偏航电机底面（脖子底座顶面）
YAW_TOP = YAW_Z0 + GM3506["H"]   # 偏航转子顶面 z=74
PITCH_X = HIP_X + 34             # 俯仰轴 x=184（v0.3.3 前移 12mm：加了屏幕以后头部重心前移了）
PITCH_Z = 96.0
ARM_Y = (34.0, 38.0)             # 轭架两臂
YAW_RANGE = (-50.0, 50.0)
PITCH_RANGE = (-15.0, 25.0)      # 负 = 抬头（再往上头后部会碰到轭架底板，抬头可以配合身体前仰）
# 转头和低头/抬头的联合限位（站、坐、趴 3 个姿态下扫描，留 2mm 间隙）：|偏航| -> (最多抬头, 最多低头)
# 头转得很偏时再抬头，头后部会碰到机身；低头全程不受限。固件里按 |偏航| 往上取到下一档查表。
NECK_LIMITS = [(40, -15.0, 25.0), (45, -5.0, 25.0), (50, -2.5, 25.0)]

HEAD_SECT = [
    (HIP_X - 28, 54, 72, 132, 12, 6),
    (HIP_X + 58, 52, 72, 132, 14, 6),
    (HIP_X + 100, 46, 80, 126, 10, 6),
]
HEAD_FRONT_X = HEAD_SECT[-1][0]          # 250
VISOR = (-30, 30, 97.5, 114.5)           # 面罩窗（屏幕可视区 55.3×14.8 从这里露出来）

# 眼睛屏：2.25 寸 76×284 长条 IPS（ST7789，SPI），常见蓝色转接板 73.15×21.08，8 针在一端
# 玻璃 62.5×17.9×2.3，显示区 55.3×14.8；玻璃左右居中对准面罩窗，转接板往左（+Y）多出 10.65 放排针
SCREEN = dict(glass=(62.5, 17.9, 2.3), active=(55.3, 14.8), pcb=(73.15, 21.08, 1.4), zc=106.0)
SCR_X1 = HEAD_FRONT_X - 2.6              # 玻璃正面贴在头壳内壁 x=247.4
SCR_X0 = SCR_X1 - SCREEN["glass"][2]     # 玻璃背面 245.1
SCR_PCB_X0 = SCR_X0 - SCREEN["pcb"][2]   # 转接板背面 243.7
SCR_BOSS = [(-23.0, 120.0), (23.0, 120.0)]   # 压框螺丝柱（M2 自攻）

# 屏幕下面那条白边（z 80–97.5）：中间相机镜头，左边（-Y）测距窗，右边（+Y）隐私指示灯
# 相机用分体式 USB 模组（UVC 免驱）：镜头模组在软排线上，约 10×10；主控小板约 26×10 贴在头壳右侧内壁
LENS = (0.0, 88.0)                       # 镜头中心 (y, z)
CAM_MOD = 10.0                           # 镜头模组外形（方），到货核对
TOF_POS = (-16.0, 88.0)                  # VL53L5CX 窗口
PRIV_LED = (12.0, 88.0)                  # 隐私指示灯
RADAR = dict(y=(-17.5, 17.5), z=(116.0, 123.0))   # LD2410B（7×35 长条板），藏在屏幕上方的白边后面，毫米波能穿透塑料


# ------------------------------------------------------------------ 脖子（固定 / 偏航 / 俯仰）
def neck_base():
    """脖子底座：螺接在前髋座顶面，托住偏航电机定子"""
    b = box(YAW_X - 28, YAW_X + 28, -30, 30, TOP, TOP + 6)
    b = chamfer_edges(b, "|Z", 6.0)
    b = b.cut(cyl_z(5, TOP - 1, TOP + 7, YAW_X, 0))                     # 空心轴走线
    b = b.cut(box(YAW_X - 26, YAW_X - 16, -8, 8, TOP - 1, TOP + 7))     # 线束口
    for a in range(4):
        t = math.radians(45 + 90 * a)
        b = b.cut(cyl_z(1.7, TOP - 1, TOP + 7, YAW_X + 15 * math.cos(t), 15 * math.sin(t)))
    return b


def yaw_motor():
    st = cyl_z(GM3506["D"] / 2 - 4, YAW_Z0, YAW_Z0 + 5, YAW_X, 0)
    rot = cyl_z(GM3506["D"] / 2, YAW_Z0 + 5, YAW_TOP, YAW_X, 0)
    rot = chamfer_edges(rot, "%CIRCLE", 1.0)
    return st, rot


def yoke():
    """轭架（随偏航转）：底板接偏航转子，两臂托住俯仰轴"""
    z0 = YAW_TOP
    base = box(YAW_X - 21, PITCH_X + 10, -ARM_Y[1], ARM_Y[1], z0, z0 + 4)
    base = chamfer_edges(base, "|Z", 6.0)
    base = base.cut(box(PITCH_X - 27, PITCH_X + 27, ARM_Y[0] - GM4108["H"] - 2, ARM_Y[0], z0 - 1, z0 + 5))   # 俯仰电机让位
    y = base
    for s in (+1, -1):
        y0, y1 = (ARM_Y[0], ARM_Y[1]) if s > 0 else (-ARM_Y[1], -ARM_Y[0])
        arm = xz_prism([(PITCH_X - 24, z0), (PITCH_X + 18, z0), (PITCH_X + 18, PITCH_Z), (PITCH_X - 24, PITCH_Z)], y0, y1)
        arm = arm.fuse(cyl_y(25, y0, y1, PITCH_X, PITCH_Z))
        arm = arm.cut(cyl_y(9.55 if s < 0 else 4, y0 - 1, y1 + 1, PITCH_X, PITCH_Z))   # 右臂 6800 轴承座
        arm = arm.cut(cyl_y(11, y0 - 1, y1 + 1, PITCH_X - 4, z0 + 12))                  # 减重
        y = y.fuse(arm)
    y = y.cut(cyl_z(5, z0 - 1, z0 + 5, YAW_X, 0))
    return y.clean()


def pitch_motor():
    """GM4108：定子装在左臂内侧，转子朝 -Y 接头部框架"""
    st = cyl_y(GM4108["D"] / 2 - 3, ARM_Y[0] - 6, ARM_Y[0], PITCH_X, PITCH_Z)
    rot = cyl_y(GM4108["D"] / 2, ARM_Y[0] - GM4108["H"], ARM_Y[0] - 6, PITCH_X, PITCH_Z)
    return st, chamfer_edges(rot, "%CIRCLE", 1.0)


# ------------------------------------------------------------------ 头部（随俯仰转）
def head_frame():
    """头部框架：左板接俯仰转子，右板带轴头插进右臂轴承；两板在头壳内与外壳螺接"""
    out = None
    for (y0, y1) in ((ARM_Y[0] - GM4108["H"] - 4, ARM_Y[0] - GM4108["H"]), (-ARM_Y[0] + 2, -ARM_Y[0] + 6)):
        p = xz_prism([(PITCH_X - 26, 104), (PITCH_X + 34, 104), (PITCH_X + 40, 126), (PITCH_X - 26, 126)], y0, y1)
        p = p.fuse(cyl_y(17, y0, y1, PITCH_X, PITCH_Z))
        p = p.cut(cyl_y(9, y0 - 1, y1 + 1, PITCH_X + 14, 116))
        out = p if out is None else out.fuse(p)
    out = out.fuse(cyl_y(5, -ARM_Y[1] - 1, -ARM_Y[0] + 2, PITCH_X, PITCH_Z))   # 右侧 Φ10 轴头
    out = out.fuse(box(PITCH_X - 26, PITCH_X - 20, -ARM_Y[0] + 2, ARM_Y[0] - GM4108["H"], 118, 126))  # 顶部横梁
    return out.clean()


def head_shell():
    outer = loft(HEAD_SECT)
    inner = [inset(s, 2.6) for s in HEAD_SECT]
    inner[0] = (inner[0][0] + 2.6,) + inner[0][1:]
    inner[-1] = (inner[-1][0] - 2.6,) + inner[-1][1:]
    h = outer.cut(loft(inner))
    h = h.cut(box(HEAD_SECT[0][0] - 1, HIP_X, -46, 46, 70, 93))                 # 后下方敞开：抬头时给轭架底板让位
    h = h.cut(box(HIP_X - 2, HEAD_SECT[1][0] + 4, -46, 46, 70, 82))             # 底部开口（轭架进出）
    # 屏幕压框的固定柱（先加柱子再开窗，免得柱子堵住开口）
    for (y, z) in SCR_BOSS:
        h = h.fuse(cyl_x(3.0, SCR_PCB_X0, SCR_X1 + 0.3, y, z))
        h = h.cut(cyl_x(0.8, SCR_PCB_X0 - 0.1, SCR_X1 - 0.5, y, z))
    ly, lz = LENS                                                                 # 镜头模组座：从背后塞进去
    w = CAM_MOD / 2 + 0.1
    h = h.fuse(box(SCR_X1 - 5, SCR_X1 + 0.3, ly - w - 1.4, ly + w + 1.4, 80.0, lz + w + 0.9))
    h = h.cut(box(SCR_X1 - 5.1, SCR_X1, ly - w, ly + w, lz - w, lz + w))
    h = h.cut(cyl_x(3.5, SCR_X1 - 0.1, HEAD_FRONT_X + 1, ly, lz))                    # 镜头孔 Φ7
    h = h.cut(box(SCR_X1 - 0.1, HEAD_FRONT_X + 1, TOF_POS[0] - 3, TOF_POS[0] + 3, TOF_POS[1] - 2, TOF_POS[1] + 2))   # 测距窗
    h = h.cut(cyl_x(1.6, SCR_X1 - 0.1, HEAD_FRONT_X + 1, PRIV_LED[0], PRIV_LED[1]))   # 隐私指示灯孔
    y0, y1, z0, z1 = VISOR
    h = h.cut(box(HEAD_FRONT_X - 4, HEAD_FRONT_X + 1, y0, y1, z0, z1))            # 面罩窗
    for k in range(4):                                                            # 侧面散热槽
        x = HIP_X + 14 + k * 9
        for s in (+1, -1):
            slot = box(x, x + 4, -60, 60, 96, 120).cut(box(x - 1, x + 5, -50.5, 50.5, 80, 124))
            h = h.cut(slot.rotate(V(x, 0, 108), V(0, 1, 0), -18))
    for s in (+1, -1):                                                            # 两侧 I2S 麦克风孔
        h = h.cut(cyl_y(1.2, s * 46 - 6, s * 46 + 6, HIP_X + 52, 112))
    h = h.cut(cyl_z(3.0, 126, 134, HIP_X + 50, 0))                                # 光照传感器窗
    h = h.cut(box(HIP_X - 16, HIP_X - 4, 50, 56, 98, 104))                        # 隐私开关（左侧）
    for s in (+1, -1):                                                            # 耳鳍安装孔
        for x in (HIP_X - 12, HIP_X + 12):
            h = h.cut(cyl_z(1.7, 126, 134, x, s * 38))
    return h.clean()


def screen_clamp():
    """屏幕压框：压住屏幕转接板背面，2 颗 M2 自攻锁在头壳上方两个柱子上；下沿有托边"""
    x0, x1 = SCR_PCB_X0 - 2.5, SCR_PCB_X0
    zb = SCREEN["zc"] - SCREEN["pcb"][1] / 2                 # 转接板下沿 95.46
    c = box(x0, x1, -26, 26, zb - 1.3, 123.4)
    c = c.cut(box(x0 - 1, x1 + 1, -16, 16, 99, 114))        # 中间掏空，避开转接板背面元件
    c = c.fuse(box(x1, SCR_X0, -26, 26, zb - 1.3, zb - 0.06))
    for (y, z) in SCR_BOSS:
        c = c.cut(cyl_x(1.1, x0 - 1, x1 + 1, y, z))
    return c.clean()


def screen_parts():
    """屏幕：玻璃（含显示区）+ 转接板；返回 [(名字, shape, 颜色)]"""
    gw, gh, gt = SCREEN["glass"]
    pw, ph, pt = SCREEN["pcb"]
    zc = SCREEN["zc"]
    glass = box(SCR_X0, SCR_X1, -gw / 2, gw / 2, zc - gh / 2, zc + gh / 2)
    y_pcb0 = -gw / 2 - 0.35
    pcb = box(SCR_PCB_X0, SCR_X0, y_pcb0, y_pcb0 + pw, zc - ph / 2, zc + ph / 2)
    return [("长条彩屏", glass, "glass"), ("屏幕转接板", pcb, "pcb"),
            ("预留-屏幕排线", box(SCR_PCB_X0 - 12, SCR_PCB_X0, y_pcb0 + pw - 9, y_pcb0 + pw - 0.5, zc - 9, zc + 9), "keepout")]


def head_visor():
    """深色亚克力面罩 2mm，嵌在面罩窗里，屏幕玻璃在它后面贴着头壳内壁"""
    y0, y1, z0, z1 = VISOR
    return chamfer_edges(box(SCR_X1 + 0.1, SCR_X1 + 2.1, y0 + 0.3, y1 - 0.3, z0 + 0.3, z1 - 0.3), "|X", 2.0)


def head_eyes():
    """屏幕上显示的一对眼睛（只用于效果图；实际是屏幕动画，可以眨眼、看左右、眯眼）"""
    out = []
    for y in (-13.5, 13.5):
        e = box(SCR_X1 + 2.105, SCR_X1 + 2.16, y - 7.5, y + 7.5, SCREEN["zc"] - 4.5, SCREEN["zc"] + 4.5)   # 画在面罩表面，效果图里看得见
        out.append(chamfer_edges(e, "|X", 3.0))
    return out


def privacy_led():
    return cyl_x(1.5, SCR_X1 - 5, HEAD_FRONT_X - 0.2, PRIV_LED[0], PRIV_LED[1])


def ear_fin(side=+1, assembled=True):
    x0 = HIP_X - 20
    zb = 131.0
    pts = [(x0, zb), (x0 + 40, zb), (x0 + 30, zb + 7), (x0 - 4, zb + 37), (x0 - 10, zb + 35)]
    f = xz_prism(pts, -3, 3)
    f = chamfer_edges(f, "|Y", 1.2)
    f = f.fuse(box(x0 + 2, x0 + 36, -6, 6, zb - 1, zb + 2))
    f = f.cut(cyl_z(1.7, zb - 2, zb + 3, HIP_X - 12, 0)).cut(cyl_z(1.7, zb - 2, zb + 3, HIP_X + 12, 0))
    if not assembled:
        return f
    return f.rotate(V(0, 0, zb), V(1, 0, 0), -14 * side).translate(V(0, side * 38, 0))


def touch_pads_head():
    """头顶 + 下巴两块触摸铜箔（贴在外壳内侧，接 ESP32-S3 触摸引脚）"""
    top = box(HIP_X + 8, HIP_X + 40, -26, 26, 128.9, 129.3)
    # 下巴内表面：从 (x=208, z=74.6) 斜到 (x=247.4, z=82.6)
    (x1, _, z1a, _, _, _), (x2, _, z2a, _, _, _) = HEAD_SECT[1], HEAD_SECT[2]
    run = x2 - 2.6 - x1
    rise = z2a - z1a
    ang = math.degrees(math.atan2(rise, run))
    x0 = x1 + 6
    z0 = z1a + 2.6 + (x0 - x1) * rise / run + 0.1
    chin = box(0, 24, -18, 18, 0, 0.4).rotate(V(0, 0, 0), V(0, 1, 0), -ang).translate(V(x0, 0, z0))
    return [top, chin]


def head_electronics():
    """头内电子件占位：(名字, shape, 颜色)"""
    ly, lz = LENS
    w = CAM_MOD / 2
    e = [
        ("USB相机", box(SCR_X1 - 7, SCR_X1, ly - w, ly + w, lz - w, lz + w), "camera"),     # 镜头模组
        ("相机主板", box(HIP_X + 62, HIP_X + 88, -44, -41, 92, 102), "pcb"),               # 贴右侧内壁
        ("VL53L5CX测距", box(SCR_X1 - 3, SCR_X1, TOF_POS[0] - 7, TOF_POS[0] + 7, TOF_POS[1] - 5, TOF_POS[1] + 6), "pcb"),
        ("LD2410毫米波", box(SCR_X1 - 2.1, SCR_X1, RADAR["y"][0], RADAR["y"][1], RADAR["z"][0], RADAR["z"][1]), "pcb"),
        ("ESP32-S3", box(HIP_X - 23, HIP_X - 18, -24, 0, 98, 116), "pcb"),
        ("USB集线器", box(HIP_X - 23, HIP_X - 18, 4, 32, 98, 118), "pcb"),
        ("功放MAX98357A", box(HIP_X + 15, HIP_X + 35, 45.5, 48.5, 100, 118), "pcb"),
        ("28mm扬声器", cyl_y(14, -49, -44, HIP_X + 28, 104), "fan"),       # 右侧壁，声音从侧面斜槽出来
        ("BH1750光照", box(HIP_X + 44, HIP_X + 56, -6, 6, 126.4, 128.6), "pcb"),
        ("隐私开关", box(HIP_X - 15, HIP_X - 5, 44, 51, 98, 104), "pcb"),
    ]
    for s in (+1, -1):
        e.append((f"I2S麦克风{'L' if s > 0 else 'R'}", box(HIP_X + 46, HIP_X + 58, s * 46.5 - 1.5, s * 46.5 + 1.5, 106, 118), "pcb"))
    return e + screen_parts()
