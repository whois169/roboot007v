"""
生成 RoboDog v0.5（可动装配；v0.5 = 皮带长度修正 + 外观设计规范 v1（深灰 + 银 + 琥珀橙）+ 背部任务导轨 / 激光雷达 / 任务箱 + 热成像和红外收发）：
  out/stl/                 可打印零件（已摆好打印方向）
  out/step/                站立姿态整机 STEP
  out/robodog.glb          按刚体分组的模型（零位），配合 out/rig.json 在网页里摆姿态
  out/rig.json             关节位置/轴线 + 各姿态的关节角
  out/report.txt           皮带、重心、各姿态着地点、干涉检查、打印件清单
用法：python3 build.py [--check]     --check 会对每个姿态做全量干涉检查（几分钟）
"""
import json
import math
import os
import sys
import numpy as np
import cadquery as cq
import trimesh

import parts as P
import leg as G
import armor as A
import head as H
from config import *

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "out")
os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
os.makedirs(os.path.join(OUT, "step"), exist_ok=True)
V = cq.Vector
O = V(0, 0, 0)

# 外观设计规范 v1：深灰哑光壳板（armor）+ 银色件（trim）+ 黑色传感器窗（black）+ 琥珀橙状态灯（amber）；眼睛屏幕青色（light）
COL = dict(
    frame="#17191c", block="#17191c", thigh_out="#1c1f22", armor="#2a2d32", trim="#d3d7dc", black="#08090a",
    motor="#1b1c1f", flange="#a7abb1", alu="#b9bec4", steel="#8d939a", belt="#141414",
    rubber="#121212", tpu="#2f3236", estop_base="#f2c300", estop_cap="#d32f2f", battery="#2f5d8a",
    pi="#3d7d4a", pcb="#1f3b6e", fan="#2a2a2a", camera="#202225", glass="#07090c", ir="#3a2448",
    copper="#c87533", light="#0b6f86", amber="#6a3000", light_red="#ff2d2d", keepout="#ffb020",
)
# 发光件：底色压暗 + 自发光，效果图里颜色才不会被冲成白色（实物颜色：眼睛青 #5CE1FF、状态灯琥珀橙 #FFA31F）
EMISSIVE = {"light": (0.22, 0.86, 1.0), "amber": (1.0, 0.40, 0.0), "light_red": (1.0, 0.12, 0.12)}

# ------------------------------------------------------------------ 刚体分组（零位、机身坐标）
groups = {}          # 组名 -> [(零件名, shape, 颜色)]
joints = {}          # 关节名 -> dict(parent, point, axis)


def add(g, name, shape, col):
    groups.setdefault(g, []).append((name, shape, col))


LEGS = [("FL", +1, +1, "前左"), ("FR", +1, -1, "前右"), ("RL", -1, +1, "后左"), ("RR", -1, -1, "后右")]
X_HFE = HIP_X + HFE_DX


def leg_to_body(s, sx, sy):
    w = s.translate(V(sx * X_HFE, Y_OUT, 0))
    return w.mirror("XZ") if sy < 0 else w


def body_mirror(s, sx, sy):
    if sx < 0:
        s = s.mirror("YZ")
    if sy < 0:
        s = s.mirror("XZ")
    return s


# 腿部零件（腿坐标零位，左前腿）
hb, hf = P.motor_y(-MOTOR["H"])
kb, kf = P.motor_y(Y_KFE[0])
idler_r, idler_ax, idler_sl = G.idler()
THIGH_PARTS = [("HFE法兰", hf, "flange"), ("大腿内板", G.thigh_inner(), "frame"),
               ("大腿外板", G.thigh_outer(), "thigh_out"), ("KFE电机", kb, "motor"), ("KFE法兰", kf, "flange"),
               ("带轮转接盘", G.pulley_adapter(), "frame"), ("小带轮20T", G.pulley_small(), "alu"),
               ("同步带HTD5M", G.belt(), "belt"), ("张紧轮", idler_r, "alu"), ("张紧轮轴", idler_ax, "steel"),
               ("张紧轮套筒", idler_sl, "frame"), ("膝轴", G.knee_shaft(), "alu"),
               ("大腿装甲", A.thigh_cover(), "armor"), ("髋端盖", A.hip_cap(), "trim"), ("膝端盖", A.knee_cap(), "trim"),
               ("膝电机前安装环", G.kfe_front_ring(), "frame"),
               ("大腿隔板", G.thigh_partition(), "frame")]
k_seat, k_plug = P.motor_connector_y(Y_KFE[0], G.KFE_CONN_ANG)
THIGH_PARTS += [("KFE插头座", k_seat, "motor"), ("预留-KFE插头", k_plug, "keepout")]
HAA_CONN_ANG = 55.0                                              # HAA 插头斜向上外侧（髋座内腔对角最宽）
THIGH_PARTS += [(f"立柱{i}", s, "alu") for i, s in enumerate(G.standoffs())]
SHANK_PARTS = [("小腿", G.to_knee(G.shank()), "frame"), ("大带轮36T", G.to_knee(G.pulley_big()), "alu"),
               ("小腿护甲", G.to_knee(G.shin_armor()), "trim"),
               ("足端", G.to_knee(G.foot_core()), "tpu"), ("足端防滑套", G.to_knee(G.foot_tread()), "rubber")]

SHROUD = P.hip_shroud_FL()
for code, sx, sy, cn in LEGS:
    gh, gt, gk = f"{code}_haa", f"{code}_hfe", f"{code}_knee"
    mb, mf = P.motor_x(HIP_X, HAA_Y)
    add("body", f"{cn}-HAA电机", body_mirror(mb, sx, sy), "motor")
    add(gh, f"{cn}-HAA法兰", body_mirror(mf, sx, sy), "flange")
    add(gh, f"{cn}-髋支架", body_mirror(P.haa_bracket_FL(), sx, sy), "frame")
    add(gh, f"{cn}-髋罩上半", body_mirror(SHROUD[0], sx, sy), "armor")
    add(gh, f"{cn}-髋罩下半", body_mirror(SHROUD[1], sx, sy), "armor")
    add(gh, f"{cn}-髋罩状态灯", body_mirror(P.hip_shroud_light_FL(), sx, sy), "amber")
    add(gh, f"{cn}-HFE电机", leg_to_body(hb, sx, sy), "motor")
    # HFE 插头：前腿朝前、后腿朝后（都朝远离机身的一侧），髋罩才能压低。腿上零件四条腿同朝向，所以这里按前后分别给角度
    h_seat, h_plug = P.motor_connector_y(-MOTOR["H"], HFE_CONN_ANG if sx > 0 else 180.0 - HFE_CONN_ANG)
    add(gh, f"{cn}-HFE插头座", leg_to_body(h_seat, sx, sy), "motor")
    add(gh, f"{cn}-预留-HFE插头", leg_to_body(h_plug, sx, sy), "keepout")
    a_seat, a_plug = P.motor_connector_x(HIP_X - MOTOR["H"], HAA_Y, HAA_CONN_ANG)
    add("body", f"{cn}-HAA插头座", body_mirror(a_seat, sx, sy), "motor")
    add("body", f"{cn}-预留-HAA插头", body_mirror(a_plug, sx, sy), "keepout")
    for nm, s, c in THIGH_PARTS:
        add(gt, f"{cn}-{nm}", leg_to_body(s, sx, sy), c)
    for nm, s, c in SHANK_PARTS:
        add(gk, f"{cn}-{nm}", leg_to_body(s, sx, sy), c)
    joints[gh] = dict(parent="body", point=(0, sy * HAA_Y, 0), axis=(1, 0, 0))
    joints[gt] = dict(parent=gh, point=(sx * X_HFE, sy * Y_OUT, 0), axis=(0, 1, 0))
    joints[gk] = dict(parent=gt, point=(sx * X_HFE, sy * Y_OUT, -L_THIGH), axis=(0, 1, 0))

# 机身
blk = P.hip_block_front()
add("body", "前髋座", blk, "block")
add("body", "后髋座", blk.mirror("YZ"), "block")
add("body", "前髋座底盖", P.hip_block_cover(), "block")
add("body", "后髋座底盖", P.hip_block_cover().mirror("YZ"), "block")
add("body", "前防跌落ToF", P.cliff_tof_front(), "pcb")
add("body", "后防跌落ToF", P.cliff_tof_front().mirror("YZ"), "pcb")
RX = HIP_X - BLOCK_DEPTH - 1
for yy in (RAIL_Y, -RAIL_Y):
    for zz in (40.0, -35.0):
        add("body", "2020型材", P.rail_2020(-RX, RX, yy, zz), "alu")
sa = A.side_armor_L()
add("body", "左侧甲", sa, "armor")
add("body", "右侧甲", sa.mirror("XZ"), "armor")
add("body", "左侧灯", A.side_light_L(), "amber")
add("body", "右侧灯", A.side_light_L().mirror("XZ"), "amber")
add("body", "左侧饰条", A.side_trim_L(), "trim")
add("body", "右侧饰条", A.side_trim_L().mirror("XZ"), "trim")
sh_a = A.shoulder_armor_FL()
for code, sx, sy, cn in LEGS:
    add("body", f"{cn}肩甲", body_mirror(sh_a, sx, sy), "armor")
add("body", "顶部装甲", A.top_armor(), "armor")
for i, l in enumerate(A.spine_lights()):
    add("body", f"脊灯{i}", l, "amber")
for i, p in enumerate(A.touch_pads_back()):
    add("body", f"背部触摸铜箔{i}", p, "copper")
add("body", "电池门", A.battery_door(), "frame")
add("body", "尾罩", A.tail_cowl(), "armor")
add("body", "尾灯", A.tail_light(), "light_red")
# v0.5 背部：两根任务导轨（兼提手）+ 激光雷达模块 + 示范任务箱 + GX16 任务接口
RAIL = A.mission_rail_L()
add("body", "左任务导轨", RAIL, "frame")
add("body", "右任务导轨", RAIL.mirror("XZ"), "frame")
add("body", "左导轨灯", A.rail_light_L(), "amber")
add("body", "右导轨灯", A.rail_light_L().mirror("XZ"), "amber")
LID_PLATE, LID_BODY, LID_RING = A.lidar_module()
add("body", "激光雷达桥板", LID_PLATE, "frame")
add("body", "激光雷达(占位)", LID_BODY, "black")
add("body", "激光雷达灯环", LID_RING, "amber")
TBOX, TLID = A.task_box()
add("body", "任务箱箱体", TBOX, "armor")
add("body", "任务箱盖", TLID, "armor")
_tw, _tl, _tc = A.task_box_parts()
add("body", "任务箱相机窗", _tw, "black")
add("body", "任务箱状态灯", _tl, "amber")
for i, c in enumerate(_tc):
    add("body", f"任务箱快拆夹{i}", c, "trim")
add("body", "任务接口GX16", A.gx16_socket(), "steel")
eb, ec = A.estop_v2()
add("body", "急停底座", eb, "estop_base")
add("body", "急停按钮", ec, "estop_cap")
add("body", "脖子底座", H.neck_base(), "frame")
ys, yr = H.yaw_motor()
add("body", "偏航电机定子GM3506", ys, "motor")

# 电子件（机身中段）。电池 x 位置由重心配平决定，先放 0
bl, bw, bh = BATTERY
add("body", "电池6S2P", P.box(-bl / 2, bl / 2, -bw / 2, bw / 2, BODY_ZMIN + 1, BODY_ZMIN + 1 + bh), "battery")
add("body", "MC02主控", P.box(-40, -40 + MC02[0], -MC02[1] / 2, MC02[1] / 2, 4, 4 + MC02[2]), "pcb")
add("body", "树莓派4B", P.box(-40, -40 + PI4[0], -PI4[1] / 2, PI4[1] / 2, 24, 24 + PI4[2]), "pi")
add("body", "配电板PDB(含INA228)", P.box(-88, -88 + PDB[1], -PDB[0] / 2 + 2, PDB[0] / 2 - 2, 4, 4 + PDB[2]), "pcb")
add("body", "DC-DC 5V", P.box(-88, -88 + DCDC[0], 32, 32 + DCDC[2], 4, 4 + DCDC[1]), "pcb")
add("body", "DC-DC 12V(脖子电机)", P.box(-88, -63, -46, -32, 4, 14), "pcb")
add("body", "MPR121触摸", P.box(48, 68, -46, -30, 26, 30), "pcb")
add("body", "4010风扇", P.box(-88, -48, -20, 20, 44, 54), "fan")
KEEPOUT = [
    ("预留-树莓派USB/网口插头", P.box(-40 + PI4[0], -40 + PI4[0] + 30, -26, 26, 26, 46)),
    ("预留-树莓派供电/HDMI侧", P.box(-30, 10, -PI4[1] / 2 - 18, -PI4[1] / 2, 26, 40)),
    ("预留-MC02左侧CAN口", P.box(-40, 24, MC02[1] / 2, MC02[1] / 2 + 12, 4, 20)),
    ("预留-MC02右侧CAN口", P.box(-40, 24, -MC02[1] / 2 - 12, -MC02[1] / 2, 4, 20)),
    ("预留-PDB出线", P.box(-88, -44, -26, 26, 26, 36)),
    ("预留-电池引线", P.box(-bl / 2 - 18, -bl / 2 - 2, -20, 20, BODY_ZMIN + 2, -4)),   # 电池后端出线，往上接配电板
]
for nm, s in KEEPOUT:
    add("body", nm, s, "keepout")

# 脖子 / 头
add("neck_yaw", "偏航电机转子", yr, "motor")
add("neck_yaw", "轭架", H.yoke(), "frame")
ps, pr = H.pitch_motor()
add("neck_yaw", "俯仰电机定子GM4108", ps, "motor")
add("neck_pitch", "俯仰电机转子", pr, "motor")
add("neck_pitch", "头部框架", H.head_frame(), "frame")
add("neck_pitch", "头壳", H.head_shell(), "armor")
add("neck_pitch", "屏幕压框", H.screen_clamp(), "frame")
add("neck_pitch", "面罩", H.head_visor(), "glass")
add("neck_pitch", "前脸面板", H.face_plate(), "trim")
add("neck_pitch", "传感器面板", H.sensor_panel(), "black")
for i, e in enumerate(H.head_eyes()):
    add("neck_pitch", f"屏幕眼睛{i}", e, "light")
add("neck_pitch", "隐私指示灯", H.privacy_led(), "light")
add("neck_pitch", "左耳鳍", H.ear_fin(+1), "armor")
add("neck_pitch", "右耳鳍", H.ear_fin(-1), "armor")
for i, p in enumerate(H.touch_pads_head()):
    add("neck_pitch", ["头顶触摸铜箔", "下巴触摸铜箔"][i], p, "copper")
for nm, s, c in H.head_electronics():
    add("neck_pitch", nm, s, c)
joints["neck_yaw"] = dict(parent="body", point=(H.YAW_X, 0, 0), axis=(0, 0, 1))
joints["neck_pitch"] = dict(parent="neck_yaw", point=(H.PITCH_X, 0, H.PITCH_Z), axis=(0, 1, 0))


# ------------------------------------------------------------------ 姿态
def ik(dx, dz):
    """髋 -> 足端球心向量（世界前后 dx、上下 dz） -> (大腿世界角 a_w, 膝角 β)；角度从竖直向下量，向后为正"""
    d = min(math.hypot(dx, dz), L_THIGH + L_SHANK)
    inner = math.degrees(math.acos((L_THIGH ** 2 + L_SHANK ** 2 - d ** 2) / (2 * L_THIGH * L_SHANK)))   # 膝内角
    alpha = math.degrees(math.acos(min(1.0, (L_THIGH ** 2 + d ** 2 - L_SHANK ** 2) / (2 * L_THIGH * d))))
    phi = math.degrees(math.atan2(-dx, -dz))
    return phi + alpha, 180.0 - inner


def leg_pose(q_haa, a_body, beta):
    return dict(haa=q_haa, hfe=a_body, knee=beta)


def make_pose(legs, body_pitch=0.0, yaw=0.0, pitch=0.0):
    return dict(legs=legs, body_pitch=body_pitch, yaw=yaw, pitch=pitch)


stand_leg = leg_pose(STAND_HAA, STAND_THIGH, STAND_THIGH + STAND_SHANK)
POSES = {}
POSES["stand"] = make_pose({c: stand_leg for c, *_ in LEGS})
POSES["look"] = make_pose({c: stand_leg for c, *_ in LEGS}, yaw=35.0, pitch=-12.0)
# 坐下：后腿折叠、前腿撑直，机身抬头 25°
p_sit = -25.0
h_rear = 145.0
h_front = h_rear + 2 * X_HFE * math.sin(math.radians(-p_sit))
aw_r, b_r = ik(25.0, -(h_rear - FOOT_D / 2))
aw_f, b_f = ik(0.0, -(h_front - FOOT_D / 2))
sit_f, sit_r = leg_pose(STAND_HAA, aw_f - p_sit, b_f), leg_pose(STAND_HAA, aw_r - p_sit, b_r)
POSES["sit"] = make_pose({"FL": sit_f, "FR": sit_f, "RL": sit_r, "RR": sit_r}, body_pitch=p_sit, pitch=-10.0)
# 趴下：前腿向前伸、后腿向后伸，腹部贴近地面（电机可以断电休息）
lie_f, lie_r = leg_pose(0.0, -85.0, 8.0), leg_pose(0.0, 85.0, -8.0)
POSES["lie"] = make_pose({"FL": lie_f, "FR": lie_f, "RL": lie_r, "RR": lie_r}, pitch=18.0)
POSE_CN = {"stand": "站立", "look": "转头抬头", "sit": "坐下", "lie": "趴下"}


def joint_angles(pose):
    """姿态 -> 每个关节绕自身轴线（右手）的转角"""
    ang = {}
    for code, sx, sy, cn in LEGS:
        lp = pose["legs"][code]
        ang[f"{code}_haa"] = sy * lp["haa"]
        ang[f"{code}_hfe"] = lp["hfe"]
        ang[f"{code}_knee"] = -lp["knee"]
    ang["neck_yaw"] = pose["yaw"]
    ang["neck_pitch"] = pose["pitch"]
    return ang


def chain(g):
    out = []
    while g != "body":
        out.append(g)
        g = joints[g]["parent"]
    return out           # 由内到外


def place(shape, g, ang, body_pitch):
    s = shape
    for j in chain(g):
        p, ax = joints[j]["point"], joints[j]["axis"]
        s = s.rotate(V(*p), V(p[0] + ax[0], p[1] + ax[1], p[2] + ax[2]), ang[j])
    return s.rotate(O, V(0, 1, 0), body_pitch)


def posed_items(pose):
    ang = joint_angles(pose)
    items = []
    for g, lst in groups.items():
        for nm, s, c in lst:
            items.append((g, nm, place(s, g, ang, pose["body_pitch"]), c))
    zmin = min(s.BoundingBox().zmin for _, _, s, c in items if c != "keepout")
    h = -zmin
    items = [(g, nm, s.translate(V(0, 0, h)), c) for g, nm, s, c in items]
    return items, h


# ------------------------------------------------------------------ 质量与重心
DENS = {"frame": 1.20 * 0.60, "block": 1.20 * 0.45, "thigh_out": 1.20 * 0.55, "armor": 1.07 * 0.85, "trim": 1.24 * 0.85, "black": 1.27,
        "rubber": 1.10, "tpu": 1.21 * 0.5, "alu": 1.20 * 0.8, "steel": 7.85, "belt": 1.25, "glass": 1.19, "copper": 0.0}
FIXED_MASS = {  # 外购件按公开数据/经验估（kg）
    "HAA电机": 0.28, "HFE电机": 0.28, "KFE电机": 0.28, "HAA法兰": 0.02, "HFE法兰": 0.02, "KFE法兰": 0.02,
    "电池6S2P": 0.88, "树莓派4B": 0.13, "MC02主控": 0.05, "配电板PDB": 0.09, "DC-DC 5V": 0.03, "DC-DC 12V": 0.02,
    "MPR121": 0.005, "4010风扇": 0.02, "偏航电机定子": 0.03, "偏航电机转子": 0.05, "俯仰电机定子": 0.05,
    "俯仰电机转子": 0.075, "USB相机": 0.03, "长条彩屏": 0.008, "28mm扬声器": 0.015, "ESP32-S3": 0.008, "USB集线器": 0.01,
    "急停": 0.025, "防跌落ToF": 0.003, "VL53L5CX": 0.003, "LD2410": 0.003, "BH1750": 0.002, "I2S麦克风": 0.002,
    "功放": 0.003, "隐私开关": 0.003, "激光雷达(占位)": 0.05, "GX16": 0.02, "MLX90640热成像探头": 0.003, "MLX90640转接板": 0.002,
    "红外发射管": 0.001, "红外接收头": 0.001, "2020型材": 0.5 * (2 * RX) / 1000, "张紧轮轴": 0.01, "膝轴": 0.02,
}


def mass_of(nm, s, c):
    if c in ("keepout", "light", "amber", "light_red", "copper", "ir"):
        return 0.0
    for k, m in FIXED_MASS.items():
        if k in nm:
            return m
    return s.Volume() / 1e6 * DENS.get(c, 1.0)       # cm³·g/cm³ -> kg


def com(items):
    M, acc = 0.0, np.zeros(3)
    for g, nm, s, c in items:
        m = mass_of(nm, s, c)
        if m > 0:
            cp = s.Center()
            acc += m * np.array([cp.x, cp.y, cp.z])
            M += m
    return M, acc / M


def feet_xy(items):
    return np.array([(s.Center().x, s.Center().y) for g, nm, s, c in items if nm.endswith("足端")])


report = []
stand_items, _ = posed_items(POSES["stand"])
M_total, c0 = com(stand_items)
need = feet_xy(stand_items)[:, 0].mean() - c0[0]
BATTERY_X = round(max(-23.0, min(23.0, need * M_total / FIXED_MASS["电池6S2P"])), 1)
groups["body"] = [(nm, s.translate(V(BATTERY_X, 0, 0)) if nm in ("电池6S2P", "预留-电池引线") else s, c)
                  for nm, s, c in groups["body"]]

belt_ok = abs(G.belt_length(G.IDLER_PRESS) - G.BELT_LEN) <= 0.3 and G.IDLER_RANGE[0] <= 519.5 and G.IDLER_RANGE[1] >= 521.0
report.append(f"膝关节：HTD-5M {BELT_W:.0f}mm 宽，{PULLEY_T_SMALL}T/{PULLEY_T_BIG}T = {KNEE_RATIO:.2f}:1，轴距 {L_THIGH:.1f}mm，"
              f"{G.BELT_TEETH} 齿（{G.BELT_LEN:.0f}mm）环形带")
report.append(f"        皮带长度检查：不压张紧轮需要 {G.belt_length():.2f}mm；张紧轮名义压进 {G.IDLER_PRESS:.1f}mm 时需要 "
              f"{G.belt_length(G.IDLER_PRESS):.2f}mm；长槽 ±{G.IDLER_ADJ:.0f}mm 能适应 {G.IDLER_RANGE[0]:.1f}–{G.IDLER_RANGE[1]:.1f}mm"
              f" -> {'通过' if belt_ok else '不通过!'}（要求名义误差 ≤0.3mm，调节范围覆盖 519.5–521.0mm）")
report.append(f"膝关节峰值 ≈ 7 × {KNEE_RATIO:.2f} = {7 * KNEE_RATIO:.1f} N·m；电机峰值时皮带张力 ≈ {7 / (G.R_S / 1000):.0f} N")
report.append(f"整机估重 ≈ {M_total:.2f} kg；重心配平：电池前后移 {BATTERY_X:+.1f} mm（需要 {need * M_total / FIXED_MASS['电池6S2P']:+.1f}，电池仓余量 ±23）")
report.append("")

POSE_INFO = {}
for pn, pose in POSES.items():
    items, h = posed_items(pose)
    M, c = com(items)
    contacts = sorted({nm for g, nm, s, col in items if col != "keepout" and s.BoundingBox().zmin < 1.5})
    f = feet_xy(items)
    POSE_INFO[pn] = dict(h=h, items=items)
    pose["body_h"] = h
    bb = cq.Compound.makeCompound([s for *_, s, col in items if col != "keepout"]).BoundingBox()
    report.append(f"[{POSE_CN[pn]}] 机身原点离地 {h:.0f}mm；重心 x={c[0]:+.1f} y={c[1]:+.1f} 离地 {c[2]:.0f}mm；"
                  f"足端中心 x={f[:, 0].mean():+.1f}")
    report.append(f"        着地：{'、'.join(contacts)}")
    report.append(f"        包络 长 {bb.xlen:.0f} × 宽 {bb.ylen:.0f} × 高 {bb.zmax:.0f} mm")
report.append("")
for pn, pose in POSES.items():
    l = pose["legs"]
    report.append(f"{POSE_CN[pn]}：前腿 HAA {l['FL']['haa']:+.0f}° HFE {l['FL']['hfe']:+.1f}° 膝 β {l['FL']['knee']:+.1f}°（内角 {180 - l['FL']['knee']:.0f}°）｜"
                  f"后腿 HFE {l['RL']['hfe']:+.1f}° 膝 β {l['RL']['knee']:+.1f}°（内角 {180 - l['RL']['knee']:.0f}°）｜"
                  f"机身俯仰 {pose['body_pitch']:+.0f}°｜头 偏航 {pose['yaw']:+.0f}° 俯仰 {pose['pitch']:+.0f}°")
report.append("")

# ------------------------------------------------------------------ 干涉检查
KNOWN = [("同步带", "小带轮"), ("同步带", "张紧轮"), ("同步带", "大带轮"), ("插头座", "电机")]   # 插头座是电机自身的一部分


def known(a, b):
    return any((x in a and y in b) or (x in b and y in a) for x, y in KNOWN)


def interference(items):
    hits = []
    bbs = [s.BoundingBox() for *_, s, c in items]
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            gi, ni, si, ci = items[i]
            gj, nj, sj, cj = items[j]
            if (ci == "keepout" and cj == "keepout") or known(ni, nj):
                continue
            a, b = bbs[i], bbs[j]
            if a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin or b.ymax < a.ymin or a.zmax < b.zmin or b.zmax < a.zmin:
                continue
            v = si.intersect(sj).Volume()
            if v > 2.0:
                hits.append((v, ni, nj))
    return sorted(hits, reverse=True)


if "--check" in sys.argv:
    for pn in POSES:
        hits = interference(POSE_INFO[pn]["items"])
        report.append(f"干涉检查 [{POSE_CN[pn]}]：{len(hits)} 处" +
                      ("" if not hits else "：" + "；".join(f"{a}↔{b} {v:.0f}mm³" for v, a, b in hits[:15])))
    report.append("")

# ------------------------------------------------------------------ 可打印零件
FLAT_Y = lambda s: s.rotate(O, V(1, 0, 0), -90)
FLAT_YP = lambda s: s.rotate(O, V(1, 0, 0), 90)
FLAT_YN = lambda s: s.rotate(O, V(1, 0, 0), -90)
FLIP = lambda s: s.rotate(O, V(1, 0, 0), 180)
KEEP = lambda s: s
br = P.haa_bracket_FL()
ti, to, tc = G.thigh_inner(), G.thigh_outer(), A.thigh_cover()
shk = G.shank()
ta, tl = A.top_armor(), A.tail_cowl()
PA = "PA-CF"
GR = "ASA 深灰哑光"          # 外观规范 v1：壳板
SV = "PETG 银色"            # 银色件（或任意料打印后喷银漆）
BK = "PETG 哑光黑"
DF = "PETG 半透明（导光条）"
sl = A.side_trim_L()
printables = {
    "hip_block_front": (blk, FLIP, 1, f"{PA}，4 层壁，40% 填充（顶面朝下）"),
    "hip_block_rear": (blk.mirror("YZ"), FLIP, 1, f"{PA}，4 层壁，40% 填充（顶面朝下）"),
    "hip_block_cover": (P.hip_block_cover(), FLIP, 2, "PETG"),
    "haa_bracket_FL": (br, FLAT_Y, 1, f"{PA}，5 层壁，60% 填充"),
    "haa_bracket_FR": (br.mirror("XZ"), FLAT_Y, 1, f"{PA}，5 层壁，60% 填充"),
    "haa_bracket_RL": (br.mirror("YZ"), FLAT_Y, 1, f"{PA}，5 层壁，60% 填充"),
    "haa_bracket_RR": (br.mirror("YZ").mirror("XZ"), FLAT_Y, 1, f"{PA}，5 层壁，60% 填充"),
    "thigh_inner_L": (ti, FLAT_Y, 2, f"{PA}，5 层壁，60% 填充"),
    "thigh_inner_R": (ti.mirror("XZ"), FLAT_Y, 2, f"{PA}，5 层壁，60% 填充"),
    "thigh_outer_L": (to, FLAT_Y, 2, f"{PA}，4 层壁，40% 填充（外板朝下，围边朝上，不用支撑）"),
    "thigh_outer_R": (to.mirror("XZ"), FLAT_YP, 2, f"{PA}，4 层壁，40% 填充（外板朝下，围边朝上，不用支撑）"),
    "thigh_partition": (G.thigh_partition(), FLAT_Y, 4, "PETG，3 层壁（左右通用）"),
    "hip_shroud_top_A": (SHROUD[0], FLIP, 2, f"{GR}，顶面朝下（左前、右后各 1）"),
    "hip_shroud_top_B": (SHROUD[0].mirror("XZ"), FLIP, 2, f"{GR}，顶面朝下（右前、左后各 1）"),
    "hip_shroud_bottom_A": (SHROUD[1], KEEP, 2, f"{GR}，底面朝下（左前、右后各 1）"),
    "hip_shroud_bottom_B": (SHROUD[1].mirror("XZ"), KEEP, 2, f"{GR}，底面朝下（右前、左后各 1）"),
    "hip_shroud_light": (P.hip_shroud_light_FL(), FLAT_Y, 4, DF),
    "thigh_cover_L": (tc, FLAT_YN, 2, f"{GR}，3 层壁，外面朝上"),
    "thigh_cover_R": (tc.mirror("XZ"), FLAT_YP, 2, f"{GR}，3 层壁，外面朝上"),
    "hip_cap": (A.hip_cap(), FLAT_YN, 4, f"{SV}，左右通用"),
    "knee_cap": (A.knee_cap(), FLAT_YN, 4, f"{SV}，左右通用"),
    "standoff_set": (cq.Compound.makeCompound(G.standoffs()), FLAT_Y, 4, "PETG 100% 填充（或换 M4 铝柱）"),
    "pulley_adapter": (G.pulley_adapter(), FLAT_Y, 4, f"{PA} 100% 填充"),
    "kfe_front_ring": (G.kfe_front_ring(), FLAT_Y, 4, f"{PA} 100% 填充（后期可换铝）"),
    "pulley_small_20T": (G.pulley_small(), FLAT_Y, 4, f"{PA} 100% 填充，0.12 层高（后期换铝 CNC）"),
    "pulley_big_36T": (G.pulley_big(), FLAT_Y, 4, f"{PA} 6 层壁 80% 填充，0.12 层高"),
    "idler_roller": (idler_r, FLAT_Y, 4, f"{PA}（内装 2×F625ZZ）"),
    "idler_sleeve": (idler_sl, FLAT_Y, 4, "PETG"),
    "shank_L": (shk, FLAT_Y, 2, f"{PA}，5 层壁，50% 填充"),
    "shank_R": (shk.mirror("XZ"), FLAT_Y, 2, f"{PA}，5 层壁，50% 填充"),
    "shin_armor_L": (G.shin_armor(), FLAT_YN, 2, f"{SV}，3 层壁，外面朝上"),
    "shin_armor_R": (G.shin_armor().mirror("XZ"), FLAT_YP, 2, f"{SV}，3 层壁，外面朝上"),
    "foot_core": (G.foot_core(), FLIP, 4, "TPU 95A，50% 填充"),
    "foot_tread": (G.foot_tread(), KEEP, 4, "TPU 85A 100% 填充；或用打印模具浇 40A 硅胶"),
    "side_armor_L": (sa, FLAT_Y, 1, f"{GR}，3 层壁"),
    "side_armor_R": (sa.mirror("XZ"), FLAT_Y, 1, f"{GR}，3 层壁"),
    "side_trim_L": (sl, FLAT_Y, 1, SV),
    "side_trim_R": (sl.mirror("XZ"), FLAT_Y, 1, SV),
    "side_light": (A.side_light_L(), FLAT_Y, 2, DF),
    "shoulder_armor_FL": (sh_a, FLAT_Y, 1, GR),
    "shoulder_armor_FR": (sh_a.mirror("XZ"), FLAT_Y, 1, GR),
    "shoulder_armor_RL": (sh_a.mirror("YZ"), FLAT_Y, 1, GR),
    "shoulder_armor_RR": (sh_a.mirror("YZ").mirror("XZ"), FLAT_Y, 1, GR),
    "top_armor_L": (A.split_half(ta, True), FLAT_YP, 1, f"{GR}，沿中线分两半，切面朝下"),
    "top_armor_R": (A.split_half(ta, False), FLAT_YN, 1, f"{GR}，切面朝下"),
    "spine_light": (A.spine_light_strip(), lambda s: s.rotate(O, V(0, 1, 0), 90), 2, f"{DF}，左右通用"),
    "mission_rail_L": (RAIL, FLAT_Y, 1, f"{PA}，6 层壁 60% 填充（兼提手，受力件）"),
    "mission_rail_R": (RAIL.mirror("XZ"), FLAT_Y, 1, f"{PA}，6 层壁 60% 填充"),
    "lidar_bridge": (LID_PLATE, FLIP, 1, f"{PA}，顶面朝下"),
    "task_box_body": (TBOX, KEEP, 1, f"{GR}，开口朝上（示范任务模块）"),
    "task_box_lid": (TLID, FLIP, 1, f"{GR}，顶面朝下"),
    "task_box_clamp": (_tc[0], KEEP, 4, SV),
    "battery_door": (A.battery_door(), FLIP, 1, "PETG-CF"),
    "tail_cowl_L": (A.split_half(tl, True), FLAT_YP, 1, f"{GR}，切面朝下"),
    "tail_cowl_R": (A.split_half(tl, False), FLAT_YN, 1, f"{GR}，切面朝下"),
    "neck_base": (H.neck_base(), KEEP, 1, f"{PA}，100% 填充"),
    "yoke": (H.yoke(), KEEP, 1, f"{PA}，6 层壁 60% 填充"),
    "head_frame": (H.head_frame(), FLAT_Y, 1, f"{PA}，5 层壁"),
    "head_shell": (H.head_shell(), FLIP, 1, f"{GR}，顶面朝下，前脸斜面加少量支撑"),
    "face_plate": (H.face_plate(), lambda s: s.rotate(O, V(0, 1, 0), -90), 1, f"{SV}，外面朝上"),
    "sensor_panel": (H.sensor_panel(), lambda s: s.rotate(O, V(0, 1, 0), -90), 1, f"{BK}，外面朝上"),
    "ear_fin": (H.ear_fin(+1, assembled=False), FLAT_Y, 2, f"{GR}，左右通用"),
    "screen_clamp": (H.screen_clamp(), KEEP, 1, "PETG 或 PA-CF"),
}
report.append(f"{'零件':<20}{'数量':>4}  {'打印尺寸 X×Y×Z (mm)':<26}{'体积cm3':>8}  床内?  材料")
total_g = 0.0
for name, (shape, orient, qty, mat) in printables.items():
    s = orient(shape)
    bb = s.BoundingBox()
    s = s.translate(V(-bb.xmin, -bb.ymin, -bb.zmin))
    cq.exporters.export(cq.Workplane().add(s), os.path.join(OUT, "stl", f"{name}.stl"), tolerance=0.05, angularTolerance=0.15)
    dims = sorted([bb.xlen, bb.ylen])
    fits = dims[1] <= BED and bb.zlen <= BED
    diag = (not fits) and (dims[0] + dims[1]) / math.sqrt(2) <= BED
    vol = shape.Volume() / 1000.0
    total_g += vol * qty * 1.15 * 0.6
    flag = "是" if fits else ("斜放" if diag else "否!")
    report.append(f"{name:<22}{qty:>3}  {bb.xlen:6.1f}×{bb.ylen:6.1f}×{bb.zlen:6.1f}      {vol:8.1f}  {flag:<4}  {mat}")
report.append("")
report.append(f"打印件总重粗估 ≈ {total_g / 1000:.2f} kg")

# ------------------------------------------------------------------ STEP（站立姿态）
asm = cq.Assembly(name="RoboDog_v0.5_stand")
for i, (g, nm, s, c) in enumerate(POSE_INFO["stand"]["items"]):
    if c != "keepout":
        asm.add(s, name=f"{i:03d}_{nm}", color=cq.Color(COL[c]))
asm.export(os.path.join(OUT, "step", "robodog_v0.5_stand.step"))


# ------------------------------------------------------------------ GLB（零位，按 刚体组|颜色 分节点）+ rig.json
def gl(p):
    """机身坐标 (X前 Y左 Z上, mm) -> glTF (x=左, y=上, z=前, m)，这是一个真旋转，不镜像"""
    return [p[1] / 1000.0, p[2] / 1000.0, p[0] / 1000.0]


scene = trimesh.Scene()
for g, lst in groups.items():
    bycol = {}
    for nm, s, c in lst:
        vs, ts = s.tessellate(0.25 if "带轮" in nm else 0.15, 0.3)
        if not ts:
            continue
        v = np.array([[p.x, p.y, p.z] for p in vs])
        bycol.setdefault(c, []).append(trimesh.Trimesh(vertices=np.column_stack([v[:, 1], v[:, 2], v[:, 0]]) / 1000.0,
                                                       faces=np.array(ts), process=False))
    for c, ms in bycol.items():
        m = trimesh.util.concatenate(ms)
        rgb = [int(COL[c][i:i + 2], 16) for i in (1, 3, 5)]
        metallic = 0.85 if c in ("alu", "steel", "flange", "copper") else (0.75 if c == "trim" else (0.35 if c == "motor" else 0.05))
        rough = 0.35 if c in ("alu", "steel", "flange", "copper") else (0.3 if c == "trim" else (0.08 if c == "glass" else (0.72 if c == "armor" else 0.7)))
        m.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
            name=c, baseColorFactor=rgb + [255], metallicFactor=metallic, roughnessFactor=rough,
            emissiveFactor=list(EMISSIVE.get(c, (0, 0, 0)))))
        scene.add_geometry(m, node_name=f"{g}|{c}", geom_name=f"{g}|{c}")
scene.export(os.path.join(OUT, "robodog.glb"))

rig = dict(
    joints={j: dict(parent=d["parent"], point=gl(d["point"]), axis=[d["axis"][1], d["axis"][2], d["axis"][0]])
            for j, d in joints.items()},
    poses={pn: dict(name=POSE_CN[pn], joints=joint_angles(p), body_pitch=p["body_pitch"], body_h=p["body_h"] / 1000.0)
           for pn, p in POSES.items()},
    body_pitch_axis=[1, 0, 0],
)
json.dump(rig, open(os.path.join(OUT, "rig.json"), "w"), ensure_ascii=False, indent=1)
open(os.path.join(OUT, "report.txt"), "w").write("\n".join(report) + "\n")
print("\n".join(report))
print(f"\n{sum(len(v) for v in groups.values())} 个零件，{len(groups)} 个刚体组，{len(printables)} 种打印件 -> {os.path.abspath(OUT)}")
