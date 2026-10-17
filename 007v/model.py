"""机械狗007v骨架模型：每个零件是一个 cadquery 实体，按“刚体组”分好，build.py 负责摆姿态、检查、导出。

刚体组：
  body   机身 + 髋驱动 + 电子件 + 脖子底座 + 两块肩座 + 雷达 + 货斗（都不动）
  head   中间的屏幕头（绕俯仰轴转）
  leg_FL / leg_FR / leg_RL / leg_RR   摆臂 + 轮子电机 + 轮子（绕各自髋轴转）

摆臂和轮子先按“左前腿、摆臂水平朝前”画，build.py 再旋转、镜像到四条腿。
"""
import math

import cadquery as cq

from config import *  # noqa: F401,F403  所有尺寸
import shell  # noqa: E402  外壳、头部内部结构

V = cq.Vector
DECK = BODY_H + PLATE                 # 甲板上表面（机身坐标）
NECK_Z = DECK + TOP_PLATE_T + TOP_STEP   # 脖子坐标原点的高度（坐在头部平台盖上）
BED_Z = DECK + TOP_PLATE_T              # 货斗底板上表面（导轨坐在上面）
PIVOT = (NECK_X, NECK_Z + PIVOT_UP)   # 俯仰轴（x, z）


# ---------------------------------------------------------------- 基本形状
def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl_y(cx, cz, d, y0, y1):
    """轴线沿 Y 的圆柱。"""
    return cq.Solid.makeCylinder(d / 2, y1 - y0, V(cx, y0, cz), V(0, 1, 0))


def cyl_x(cy, cz, d, x0, x1):
    return cq.Solid.makeCylinder(d / 2, x1 - x0, V(x0, cy, cz), V(1, 0, 0))


def cyl_z(cx, cy, d, z0, z1):
    return cq.Solid.makeCylinder(d / 2, z1 - z0, V(cx, cy, z0), V(0, 0, 1))


def prism_xz(points, y0, y1):
    """侧面轮廓（x, z 点）沿 Y 拉伸成 y0..y1。"""
    wp = cq.Workplane("XY").polyline(points).close().extrude(y1 - y0)
    return wp.val().rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(0, y1, 0))


def stadium_xz(x0, x1, r, y0, y1):
    """两头圆的板（侧面看），从 x0 到 x1，半径 r。"""
    wp = (cq.Workplane("XY").moveTo(x0, -r).lineTo(x1, -r).threePointArc((x1 + r, 0), (x1, r))
          .lineTo(x0, r).threePointArc((x0 - r, 0), (x0, -r)).close().extrude(y1 - y0))
    return wp.val().rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(0, y1, 0))


def hull_pts(circles, n=48):
    """几个圆的凸包（用来画同步带的外形）。"""
    pts = []
    for cx, cz, r in circles:
        pts += [(cx + r * math.cos(2 * math.pi * i / n), cz + r * math.sin(2 * math.pi * i / n)) for i in range(n)]
    pts = sorted(set(pts))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def mirror_y(s):
    return s.mirror("XZ")


def mirror_x(s):
    return s.mirror("YZ")


# ---------------------------------------------------------------- 机身（左右对称的件画左边，再镜像）
def body_parts():
    """返回 [(名字, 实体, 颜色)]，都在机身坐标里。"""
    P = []
    L2, W2 = BODY_L / 2, BODY_W / 2
    add = lambda n, s, c: P.append((n, s, c))
    # 铝板盒子
    for sgn, tag in ((1, "左"), (-1, "右")):
        y0, y1 = (W2 - PLATE_SIDE, W2) if sgn > 0 else (-W2, -W2 + PLATE_SIDE)
        add(f"{tag}侧板", box(-L2, L2, y0, y1, 0, BODY_H), "anod")
    add("底板", box(-L2 + PLATE, L2 - PLATE, -W2 + PLATE_SIDE, W2 - PLATE_SIDE, 0, PLATE), "alu")
    add("前板", box(L2 - PLATE, L2, -W2 + PLATE_SIDE, W2 - PLATE_SIDE, PLATE, BODY_H), "anod")
    add("后板", box(-L2, -L2 + PLATE, -W2 + PLATE_SIDE, W2 - PLATE_SIDE, PLATE, BODY_H), "anod")
    dk = cq.Workplane("XY").box(BODY_L, BODY_W, PLATE, centered=(True, True, False)).edges("|Z").chamfer(2).val().translate(V(0, 0, BODY_H))
    add("甲板", shell.soft(dk, 1.2), "anod")
    # 顶面三块叠层板：头部平台盖（高）—— 货斗底板（低，货放这里）—— 后检修盖（高）；板边看得见 4 mm 厚，下面垫一圈缩进 3 mm 的垫条，像一块块装上去的
    def top_plate(x0, x1, raised):
        """顶板：前后两头的板往外悬挑 TOP_OVER_X，两侧伸到 W2 + TOP_OVER_Y；垫条缩在机身里面，下面有阴影。"""
        t, st = TOP_PLATE_T, (TOP_STEP if raised else 0.0)
        hw = W2 + TOP_OVER_Y
        p = cq.Workplane("XY").box(x1 - x0, 2 * hw, t, centered=False).edges("|Z").chamfer(2).val().translate(V(x0, -hw, DECK + st))
        p = shell.soft(p, 2.0)
        if raised:
            p = p.fuse(box(max(x0, -L2) + 3, min(x1, L2) - 3, -W2 + 4, W2 - 4, DECK, DECK + st + 0.5))
        return p
    add("头部平台盖", top_plate(BED_X1 + SEAM, L2 + TOP_OVER_X, True), "deck")
    add("货斗底板", top_plate(BED_X0 - 1, BED_X1, False), "deck")
    add("后检修盖", top_plate(-L2 - TOP_OVER_X, BED_X0 - 1 - SEAM, True), "deck")
    for x0 in (L2 - PLATE - PROFILE - 2, -L2 + PLATE + 2):            # 前后上下四根 2020 横梁
        for z0 in (PLATE, BODY_H - PROFILE):
            add(f"2020横梁 x{x0:+.0f} z{z0:.0f}", box(x0, x0 + PROFILE, -W2 + PLATE_SIDE, W2 - PLATE_SIDE, z0, z0 + PROFILE), "profile")
    for sgn in (1, -1):
        add(f"滑撬{'左' if sgn > 0 else '右'}", box(-150, 150, *sorted((sgn * 70, sgn * 90)), -SKID_H, 0), "black")

    # 四个髋驱动：蜗轮电机 → 小带轮 → 同步带 → 大带轮 → 髋轴 → 侧板外的轴承座
    for fx in (1, -1):
        for sy in (1, -1):
            tag = ("前" if fx > 0 else "后") + ("左" if sy > 0 else "右")
            for n, s, c in hip_drive_FL():
                if sy < 0:
                    s = mirror_y(s)
                if fx < 0:
                    s = mirror_x(s)
                add(f"{tag}{n}", s, c)

    # 电池、保护板、电子托板、树莓派、底盘 ESP32、驱动、DC-DC、深度相机
    bx, by, bz = BATTERY
    add("电池6S2P", box(-bx / 2, bx / 2, -by / 2, by / 2, PLATE, PLATE + bz), "battery")
    add("电池保护板", box(-BMS[0] / 2, BMS[0] / 2, -BMS[1] / 2, BMS[1] / 2, PLATE + bz, PLATE + bz + BMS[2]), "pcb")
    add("电子托板", box(-140, 140, -66, 66, TRAY_Z, TRAY_Z + 2), "printed")
    t = TRAY_Z + 2
    for name, (x0, y0), (sx, sy, sz) in ELECTRONICS:
        add(name, box(x0, x0 + sx, y0, y0 + sy, t, t + sz), "pcb")
    dx, dy, dz = DEPTH_CAM
    add("深度相机", box(L2 - PLATE - dx, L2 - PLATE, -dy / 2, dy / 2, 25, 25 + dz), "black")

    for n, s, c in shell.body_shell():
        add(n, s, c)
    # 脖子底座 + 两块肩座 + 舵机 + 雷达（都在脖子坐标里画，再挪到机身上）
    for n, s, c in neck_parts():
        add(n, s.translate(V(NECK_X, 0, NECK_Z)), c)

    # 货斗：2020 卡槽导轨 + 打印栏板 + 机械臂接口板
    rail_top = BED_Z + PROFILE
    for sy in (1, -1):
        y0 = sy * RAIL_Y - PROFILE / 2
        add(f"{'左' if sy > 0 else '右'}导轨2020", box(BED_X0, BED_X1, y0, y0 + PROFILE, BED_Z, rail_top), "profile")
        f0 = sy * RAIL_Y - FENCE_T / 2
        add(f"{'左' if sy > 0 else '右'}栏板", box(BED_X0 + 5, BED_X1 - 5, f0, f0 + FENCE_T, rail_top - FENCE_INSERT, rail_top - FENCE_INSERT + FENCE_H), "shell")
    inner = RAIL_Y - PROFILE / 2
    add("后导轨2020", box(BED_X0, BED_X0 + PROFILE, -inner, inner, BED_Z, rail_top), "profile")
    add("前导轨2020", box(BED_X1 - PROFILE, BED_X1, -inner, inner, BED_Z, rail_top), "profile")
    add("后栏板", box(BED_X0 + 10 - FENCE_T / 2, BED_X0 + 10 + FENCE_T / 2, -inner + 2, inner - 2, rail_top - FENCE_INSERT, rail_top - FENCE_INSERT + FENCE_H), "shell")
    add("前栏板", box(BED_X1 - 10 - FENCE_T / 2, BED_X1 - 10 + FENCE_T / 2, -inner + 2, inner - 2, rail_top - FENCE_INSERT, rail_top - FENCE_INSERT + 50), "shell")
    ax, at, ay = ARM_PORT
    cx, cy = ARM_PORT_XY
    add("机械臂接口板", box(cx - ax / 2, cx + ax / 2, cy - ay / 2, cy + ay / 2, BED_Z, BED_Z + at), "alu")
    return P


def hip_drive_FL():
    """左前髋驱动（机身坐标）。"""
    P = []
    hx, hz = HIP_X, HIP_Z
    wx, wz = hx + WORM_OUT_DX, hz + WORM_OUT_DZ
    W2 = BODY_W / 2
    yin = W2 - PLATE_SIDE                          # 侧板内表面
    bl, bt, bh = WORM_BOX
    gy1 = yin - PULLEY_W - 2                       # 减速箱外侧（带轮离侧板 1 mm，减速箱在带轮里面，中间 16 长的六角柱）
    P.append(("蜗轮减速箱", box(wx - bl / 2, wx + bl / 2, gy1 - bt, gy1, wz - bh / 2, wz + bh / 2), "motor"))
    P.append(("蜗轮电机", cyl_x(gy1 - bt / 2, wz, WORM_MOTOR_D, wx - bl / 2 - WORM_MOTOR_L, wx - bl / 2), "motor"))
    py0, py1 = yin - PULLEY_W - 1, yin - 1          # 带轮离侧板内面 1 mm（中间隔圈穿过侧板顶住轴承内圈）
    P.append(("小带轮15T", cyl_y(wx, wz, PULLEY_SMALL_PD + 6, py0, py1), "steel"))
    P.append(("大带轮45T", cyl_y(hx, hz, PULLEY_BIG_PD + 5, py0, py1), "steel"))
    outer = hull_pts([(wx, wz, PULLEY_SMALL_PD / 2 + 1.6), (hx, hz, PULLEY_BIG_PD / 2 + 1.6)])
    inner = hull_pts([(wx, wz, PULLEY_SMALL_PD / 2 - 0.8), (hx, hz, PULLEY_BIG_PD / 2 - 0.8)])
    by0 = py0 + (PULLEY_W - BELT_W) / 2
    belt = prism_xz(outer, by0, by0 + BELT_W).cut(prism_xz(inner, by0 - 1, by0 + BELT_W + 1))
    P.append(("同步带", belt, "black"))
    P.append(("髋轴", cyl_y(hx, hz, HIP_SHAFT_D, py0 - 6, W2 + BEARING_HOUSING_W + 16), "steel"))   # 穿过侧板，插进臂座
    P.append(("磁编码器AS5600", box(hx - AS5600[0] / 2, hx + AS5600[0] / 2, py0 - 12, py0 - 8, hz - AS5600[2] / 2, hz + AS5600[2] / 2), "pcb"))
    P.append(("轴承座", cyl_y(hx, hz, BEARING_HOUSING_D, W2, W2 + BEARING_HOUSING_W), "trim"))
    return P


# 电子托板上的布置：(名字, 左下角 (x, y), 尺寸)；托板 x ±140、y ±66
ELECTRONICS = [
    ("髋电机驱动1 前左", (32.0, 4.0), DRIVER), ("髋电机驱动2 前右", (32.0, -4.0 - DRIVER[1]), DRIVER),
    ("髋电机驱动3 后左", (87.0, 4.0), DRIVER), ("髋电机驱动4 后右", (87.0, -4.0 - DRIVER[1]), DRIVER),
    ("树莓派5", (-135.0, -PI5[1] / 2), PI5),
    ("底盘ESP32-S3", (-40.0, 20.0), ESP_CHASSIS),
    ("CAN收发 SN65HVD230", (-40.0, -42.0), CAN_MODULE),
    ("DC-DC 5V5A", (-8.0, -46.0), DCDC_5V5A),
    ("DC-DC 5V3A", (-40.0, -16.0), DCDC_5V3A),
]


def ld19():
    """乐动 LD19：原点在底面中心，安装脚 54 沿 X、46.29 沿 Y。"""
    L = LD19
    ears = box(-L["ear_w"] / 2, L["ear_w"] / 2, -L["ear_d"] / 2, L["ear_d"] / 2, 0, 2.5)
    for u, v in L["holes"]:
        ears = ears.cut(cyl_z(u, v, L["hole_d"], -1, 4))
    b = L["base"] / 2
    body = box(-b, b, -b, b, 0, L["h"] - L["head_h"]).fuse(cyl_z(0, 0, L["head_d"], L["h"] - L["head_h"], L["h"]))
    return ears.fuse(body)


def neck_parts():
    """脖子坐标：原点在甲板上表面、俯仰轴正下方。"""
    P = []
    nx, nh, ny = NECK_BASE
    yc = POD_IN + POD_W / 2
    u = box(-nx / 2, nx / 2, -ny / 2, ny / 2, 0, nh).cut(box(-nx / 2 + 30, nx / 2 + 1, -POD_IN, POD_IN, -1, nh + 1))
    for sy in (1, -1):
        u = u.cut(cyl_z(CABLE_X, sy * yc, 12, -1, nh + 1))       # 肩座走线孔往下穿过底座
    P.append(("脖子底座", u, "dark"))                       # U 形：后面一根横梁 + 两条垫在肩座下面，屏幕头下面挖空
    P += shell.pods()
    top = max(z for _, z in POD_HEX)
    top_x = sum(x for x, z in POD_HEX if z == top) / 2
    P.append(("雷达LD19", ld19().translate(V(top_x, yc, top + 3)), "black"))
    P.append(("雷达底座", box(top_x - 29, top_x + 29, POD_IN, POD_IN + POD_W, top, top + 3), "trim"))   # 和肩座顶一样宽，不往外伸
    return P


def head_parts():
    """中间屏幕头，画在“俯仰轴坐标”里：原点在俯仰轴上。build.py 绕 Y 轴转，再挪到 PIVOT。"""
    return shell.head()                                        # 空心头壳 + 屏板卡槽 + 舵盘 / 轴销 + 外面的装饰


# ---------------------------------------------------------------- 摆臂 + 轮子（左前腿，摆臂水平朝前；原点在髋轴上）
def arm_box_solid(y0, y1):
    """箱形臂座外壳（摆臂坐标，髋轴在原点，轮轴在 (ARM, 0)），Y 从 y0 到 y1：
    一圈 3 mm 的壳从髋一直包到轮子跟前，尽头是贴着轮子的弧面（离轮子 ARM_WHEEL_CLEAR），不留空档；
    靠机身那面在臂根内板以外的部分封 2 mm 的墙（内板管不到那里）。"""
    R = ARM_ROOT_R
    rc = WHEEL_D / 2 + ARM_WHEEL_CLEAR
    outer = stadium_xz(0, ARM - R, R, y0, y1).cut(cyl_y(ARM, 0, 2 * rc, y0 - 1, y1 + 1))
    inner = stadium_xz(0, ARM - R, R - 3, y0 - 1, y1 + 1).cut(cyl_y(ARM, 0, 2 * (rc + 3), y0 - 2, y1 + 2))
    wall = outer.intersect(box(70, ARM, y0, y0 + 2, -R - 1, R + 1))
    return outer.cut(inner).fuse(wall)


def leg_parts_FL():
    W2 = BODY_W / 2
    wy0 = W2 + WHEEL_GAP                       # 轮子内侧
    wy1 = wy0 + WHEEL_W
    my1 = wy1 + WHEEL_MOTOR_W                  # 轮子电机外侧 = 外板内侧
    oy1 = my1 + OUTER_PLATE_T
    hub0 = W2 + BEARING_HOUSING_W + 1
    root0 = hub0 + HUB["boss_l"] + HUB["flange_t"]          # 臂根内板内侧
    root1 = root0 + ARM_ROOT_PLATE_T
    P = [
        ("臂座轴毂", cyl_y(0, 0, HUB["boss_d"], hub0, hub0 + HUB["boss_l"]).fuse(
            cyl_y(0, 0, HUB["flange_d"], hub0 + HUB["boss_l"], hub0 + HUB["boss_l"] + HUB["flange_t"])), "alu"),
        ("臂根内板", stadium_xz(0, ARM_BOX_LEN - ARM_ROOT_R, ARM_ROOT_R, root0, root1), "alu"),
        ("箱形臂座外壳", shell.soft(arm_box_solid(root1, my1), 1.5), "armor"),
        ("摆臂外板", stadium_xz(0, ARM, ARM_END_R, my1, oy1), "anod"),
        ("轮子电机GIM4310", cyl_y(ARM, 0, WHEEL_MOTOR_D, wy1, my1), "motor"),
        ("麦轮外包络", cyl_y(ARM, 0, WHEEL_D, wy0, wy1), "envelope"),      # 只用来查碰撞，不显示
        ("麦轮轮毂", cyl_y(ARM, 0, 60, wy0 - 1, wy1 + 1), "trim"),
        ("麦轮侧板内", cyl_y(ARM, 0, WHEEL_PLATE_D, wy0, wy0 + WHEEL_PLATE_T), "steel"),
        ("麦轮侧板外", cyl_y(ARM, 0, WHEEL_PLATE_D, wy1 - WHEEL_PLATE_T, wy1), "steel"),
    ]
    P += shell.arm_marks(oy1, root1, my1)
    for i, (u, v) in enumerate(ARM_STANDOFFS):
        P.append((f"臂根支柱{i + 1}", cyl_y(u, v, STANDOFF_HEX, root1, my1), "alu"))
    yc = (wy0 + wy1) / 2
    barrel = roller_solid()
    for k in range(ROLLERS):
        p, a = roller_axis(k, -1)                    # 左前轮：着地那个滚子的轴线朝“前右”
        s = barrel
        ang = math.degrees(math.acos(max(-1.0, min(1.0, a[2]))))
        ax = (-a[1], a[0], 0.0)                      # z × a
        if ang > 1e-6:
            s = s.rotate(V(0, 0, 0), V(*ax), ang)
        P.append((f"麦轮滚子{k + 1:02d}", s.translate(V(ARM + p[0], yc + p[1], p[2])), "wheel"))
    return P


def roller_axis(k, s):
    """第 k 个滚子的中心和轴向（轮心坐标）。s = 着地滚子轴线的左右方向：-1 朝前右（左前、右后轮），+1 朝前左。"""
    al = 2 * math.pi * k / ROLLERS
    t = (-math.sin(al), 0.0, math.cos(al))
    c45 = math.sqrt(0.5)
    return (ROLLER_C * math.cos(al), 0.0, ROLLER_C * math.sin(al)), (c45 * t[0], s * c45, c45 * t[2])


def roller_radius(tau):
    """滚子在轴向位置 tau 处的半径：外缘正好落在 Φ152 圆柱上。"""
    return WHEEL_D / 2 - math.sqrt(ROLLER_C ** 2 + tau * tau / 2)


def roller_solid():
    """一个桶形滚子，轴线沿 Z、中心在原点。"""
    n = 12
    pts = [(0.0, -ROLLER_H)] + [(roller_radius(-ROLLER_H + 2 * ROLLER_H * i / n), -ROLLER_H + 2 * ROLLER_H * i / n) for i in range(n + 1)] + [(0.0, ROLLER_H)]
    return cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0)).val()


def half_width():
    """整机半宽（到摆臂外板外侧）。"""
    return BODY_W / 2 + WHEEL_GAP + WHEEL_W + WHEEL_MOTOR_W + OUTER_PLATE_T
