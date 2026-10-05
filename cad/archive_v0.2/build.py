"""
生成 RoboDog v0.2：
  out/stl/      可打印零件（已摆好打印方向，左右件分开）
  out/step/     整机装配 STEP（可导入 Fusion / SolidWorks / FreeCAD）
  out/robodog.glb  带颜色的整机模型（给 3D 预览用）
  out/report.txt   零件尺寸 / 打印床检查 / 连杆长度
用法：python3 build.py
"""
import math
import os
import numpy as np
import cadquery as cq
import trimesh

import parts as P
import armor as A
from config import *

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "out")
os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
os.makedirs(os.path.join(OUT, "step"), exist_ok=True)

COL = dict(
    frame="#2c2f34",     # 打印结构件：石墨灰（PA-CF）
    block="#2c2f34",     # 髋座（同色，单独分组以便预览时透明显示）
    thigh_out="#2c2f34", # 大腿外板（同色，单独分组以便预览时拆下）
    armor="#e3e6e9",     # 外观装甲：冷白（PETG / ASA）
    motor="#1b1c1f",
    flange="#a7abb1",
    alu="#b9bec4",
    steel="#8d939a",
    crank="#6f757c",
    rubber="#121212",
    estop_base="#f2c300",
    estop_cap="#d32f2f",
    battery="#2f5d8a",
    pi="#3d7d4a",
    pcb="#1f3b6e",
    fan="#2a2a2a",
    camera="#202225",
    glass="#07090c",
    light="#5ce1ff",     # 发光灯条：冷青
    light_red="#ff2d2d", # 尾灯
    keepout="#ffb020",   # 插头/线束预留空间（仅检查用）
)
EMISSIVE = {"light": (0.36, 0.88, 1.0), "light_red": (1.0, 0.12, 0.12)}

items = []      # (名字, shape, 颜色key)
printables = {} # 名字 -> (shape, 打印朝向, 数量, 材料)


def add(name, shape, col):
    items.append((name, shape, col))


# ------------------------------------------------------------------ 腿（腿坐标，左腿）
a, b = STAND_THIGH, STAND_SHANK
n_vec = P.rot_y_vec(1, 0, a)                      # 大腿“前法向”：站立时曲柄、摇臂都指这个方向
K = P.rot_y_vec(0, -L_THIGH, a)                   # 膝轴位置
C = (CRANK_R * n_vec[0], CRANK_R * n_vec[1])      # 曲柄销
Pp = (K[0] + LEVER_R * n_vec[0], K[1] + LEVER_R * n_vec[1])  # 摇臂销
lever_local = P.rot_y_vec(1, 0, a + b)
O, Y = cq.Vector(0, 0, 0), cq.Vector(0, 1, 0)


def thigh_rot(s):
    return s.rotate(O, Y, a)


def shank_place(s):
    return s.rotate(O, Y, -b).translate(cq.Vector(K[0], 0, K[1]))


leg = []
hb, hf = P.motor_y(-MOTOR["H"])
leg += [("HFE电机", hb, "motor"), ("HFE法兰", hf, "flange")]
ti, to = P.thigh_inner(), P.thigh_outer()
leg += [("大腿内板", thigh_rot(ti), "frame"), ("大腿外板", thigh_rot(to), "thigh_out")]
for s in P.standoffs():
    leg.append(("立柱", thigh_rot(s), "alu"))
kb, kf = P.motor_y(Y_KFE[0])
leg += [("KFE电机", thigh_rot(kb), "motor"), ("KFE法兰", thigh_rot(kf), "flange")]
cr = P.crank()
leg.append(("膝曲柄", cr.rotate(O, Y, a), "crank"))
tc = A.thigh_cover()
leg.append(("大腿装甲", thigh_rot(tc), "armor"))
rod_parts, ROD_L = P.rod_assembly(C, Pp)
for r in rod_parts:
    leg.append(("连杆", r, "steel"))
sh = P.shank(lever_local)
leg.append(("小腿", shank_place(sh), "frame"))
ft = P.foot()
leg.append(("足端", shank_place(ft), "rubber"))
leg.append(("膝轴", P.cyl_y(KNEE_SHAFT_D / 2, Y_INNER[0], Y_OUTER[1] + 2.5, K[0], K[1]), "steel"))

# ------------------------------------------------------------------ 四条腿装到机身
X_HFE = HIP_X + HFE_DX
for ex, en in ((+1, "前"), (-1, "后")):
    for sy, sn in ((+1, "左"), (-1, "右")):
        tag = en + sn
        for (nm, s, c) in leg:
            w = s.translate(cq.Vector(ex * X_HFE, Y_OUT, 0))
            if sy < 0:
                w = w.mirror("XZ")
            add(f"{tag}-{nm}", w, c)
        br = P.haa_bracket_FL()
        mb, mf = P.motor_x(HIP_X, HAA_Y)
        for nm, s, c in (("髋支架", br, "frame"), ("HAA电机", mb, "motor"), ("HAA法兰", mf, "flange")):
            w = s
            if ex < 0:
                w = w.mirror("YZ")
            if sy < 0:
                w = w.mirror("XZ")
            add(f"{tag}-{nm}", w, c)

# ------------------------------------------------------------------ 机身
blk = P.hip_block_front()
add("前髋座", blk, "block")
add("后髋座", blk.mirror("YZ"), "block")
RX = HIP_X - BLOCK_DEPTH - 1
for yy in (58.0, -58.0):
    for zz in (40.0, -35.0):
        add("2020型材", P.rail_2020(-RX, RX, yy, zz), "alu")
# 外观装甲 v0.2
sa = A.side_armor_L()
add("左侧甲", sa, "armor")
add("右侧甲", sa.mirror("XZ"), "armor")
add("左侧灯", A.side_light_L(), "light")
add("右侧灯", A.side_light_L().mirror("XZ"), "light")
sh_a = A.shoulder_armor_FL()
for nm, s in (("前左肩甲", sh_a), ("前右肩甲", sh_a.mirror("XZ")),
              ("后左肩甲", sh_a.mirror("YZ")), ("后右肩甲", sh_a.mirror("YZ").mirror("XZ"))):
    add(nm, s, "armor")
add("顶部装甲", A.top_armor(), "armor")
for i, l in enumerate(A.spine_lights()):
    add(f"脊灯{i}", l, "light")
add("提手", A.handle_v2(), "frame")
add("电池门", A.battery_door(), "frame")
add("头壳", A.head_shell_v2(), "armor")
add("头部底座", A.head_base_v2(), "frame")
add("面罩", A.head_visor(), "glass")
for i, e in enumerate(A.head_eyes()):
    add(f"眼灯{i}", e, "light")
add("左耳鳍", A.ear_fin(+1), "armor")
add("右耳鳍", A.ear_fin(-1), "armor")
add("尾罩", A.tail_cowl(), "armor")
add("尾鳍", A.tail_fin(), "frame")
add("尾鳍灯", A.tail_fin_light(), "light")
add("尾灯", A.tail_light(), "light_red")
add("前髋座底盖", P.hip_block_cover(), "block")
add("后髋座底盖", P.hip_block_cover().mirror("YZ"), "block")
eb, ec = A.estop_v2()
add("急停底座", eb, "estop_base")
add("急停按钮", ec, "estop_cap")

# 电子件占位（v0.2：树莓派 4 + 达妙 MC02）
bl, bw, bh = BATTERY
add("电池6S2P", P.box(-bl / 2, bl / 2, -bw / 2, bw / 2, BODY_ZMIN + 1, BODY_ZMIN + 1 + bh), "battery")
# 中段上层：MC02 在下、树莓派用 4mm 铜柱叠在上面，USB 口朝前，留出插头空间
add("MC02主控", P.box(-40, -40 + MC02[0], -MC02[1] / 2, MC02[1] / 2, 4, 4 + MC02[2]), "pcb")
add("树莓派4B", P.box(-40, -40 + PI4[0], -PI4[1] / 2, PI4[1] / 2, 24, 24 + PI4[2]), "pi")
add("配电板PDB", P.box(-88, -88 + PDB[1], -PDB[0] / 2 + 2, PDB[0] / 2 - 2, 4, 4 + PDB[2]), "pcb")
add("DC-DC 5V", P.box(-88, -88 + DCDC[0], 32, 32 + DCDC[2], 4, 4 + DCDC[1]), "pcb")
add("4010风扇", P.box(-88, -48, -20, 20, 44, 54), "fan")
# 插头 / 线束预留空间（检查用，半透明显示）
KEEPOUT = [
    ("预留-树莓派USB/网口插头", P.box(-40 + PI4[0], -40 + PI4[0] + 30, -26, 26, 26, 46)),
    ("预留-树莓派供电/HDMI侧", P.box(-30, 10, -PI4[1] / 2 - 18, -PI4[1] / 2, 26, 40)),
    ("预留-MC02左侧CAN口", P.box(-40, 24, MC02[1] / 2, MC02[1] / 2 + 12, 4, 20)),
    ("预留-MC02右侧CAN口", P.box(-40, 24, -MC02[1] / 2 - 12, -MC02[1] / 2, 4, 20)),
    ("预留-PDB出线", P.box(-88, -44, -26, 26, 26, 36)),
    ("预留-电池引线", P.box(70, 88, -20, 20, BODY_ZMIN + 2, -4)),
]
for nm, s in KEEPOUT:
    add(nm, s, "keepout")
cx1 = A.HEAD_FRONT_X - 34
add("USB相机", P.box(cx1 - CAMERA[2], cx1, -CAMERA[0] / 2, CAMERA[0] / 2, 80.5 - CAMERA[1] / 2, 80.5 + CAMERA[1] / 2), "camera")

FOOT_Z = K[1] + P.rot_y_vec(0, -L_SHANK, -b)[1] - FOOT_D / 2

# ------------------------------------------------------------------ 可打印零件
FLAT_Y = lambda s: s.rotate(O, cq.Vector(1, 0, 0), -90)    # 法向 Y 的板 -> 平放
FLAT_X = lambda s: s.rotate(O, cq.Vector(0, 1, 0), 90)     # 法向 X -> 平放
FLIP = lambda s: s.rotate(O, cq.Vector(1, 0, 0), 180)
KEEP = lambda s: s

br = P.haa_bracket_FL()
FLAT_YP = lambda s: s.rotate(O, cq.Vector(1, 0, 0), 90)    # 切半件：y>=0 的一半切面朝下
FLAT_YN = lambda s: s.rotate(O, cq.Vector(1, 0, 0), -90)   # y<=0 的一半
ta, tl = A.top_armor(), A.tail_cowl()
printables.update({
    "hip_block_front":  (blk, FLIP, 1, "PA-CF，4 层壁，40% 填充（顶面朝下）"),
    "hip_block_rear":   (blk.mirror("YZ"), FLIP, 1, "PA-CF，4 层壁，40% 填充（顶面朝下）"),
    "hip_block_cover":  (P.hip_block_cover(), FLIP, 2, "PETG"),
    "haa_bracket_FL":   (br, FLAT_Y, 1, "PA-CF，5 层壁，60% 填充"),
    "haa_bracket_FR":   (br.mirror("XZ"), FLAT_Y, 1, "PA-CF，5 层壁，60% 填充"),
    "haa_bracket_RL":   (br.mirror("YZ"), FLAT_Y, 1, "PA-CF，5 层壁，60% 填充"),
    "haa_bracket_RR":   (br.mirror("YZ").mirror("XZ"), FLAT_Y, 1, "PA-CF，5 层壁，60% 填充"),
    "thigh_inner_L":    (ti, FLAT_Y, 2, "PA-CF，5 层壁，60% 填充"),
    "thigh_inner_R":    (ti.mirror("XZ"), FLAT_Y, 2, "PA-CF，5 层壁，60% 填充"),
    "thigh_outer_L":    (to, FLAT_Y, 2, "PA-CF，4 层壁，50% 填充"),
    "thigh_outer_R":    (to.mirror("XZ"), FLAT_Y, 2, "PA-CF，4 层壁，50% 填充"),
    "thigh_cover_L":    (tc, FLAT_Y, 2, "ASA/PETG 白，3 层壁"),
    "thigh_cover_R":    (tc.mirror("XZ"), FLAT_Y, 2, "ASA/PETG 白，3 层壁"),
    "standoff_set":     (cq.Compound.makeCompound(P.standoffs()), FLAT_Y, 4, "PETG，100% 填充（或换铝柱）"),
    "crank":            (cr, FLAT_Y, 4, "PA-CF 100% 填充（后期建议换铝 CNC）"),
    "shank_L":          (sh, FLAT_Y, 2, "PA-CF，5 层壁，50% 填充"),
    "shank_R":          (sh.mirror("XZ"), FLAT_Y, 2, "PA-CF，5 层壁，50% 填充"),
    "foot":             (ft, FLIP, 4, "TPU 95A，30% 填充"),
    "side_armor_L":     (sa, FLAT_Y, 1, "ASA/PETG 白，3 层壁"),
    "side_armor_R":     (sa.mirror("XZ"), FLAT_Y, 1, "ASA/PETG 白，3 层壁"),
    "shoulder_armor_FL": (sh_a, FLAT_Y, 1, "ASA/PETG 白"),
    "shoulder_armor_FR": (sh_a.mirror("XZ"), FLAT_Y, 1, "ASA/PETG 白"),
    "shoulder_armor_RL": (sh_a.mirror("YZ"), FLAT_Y, 1, "ASA/PETG 白"),
    "shoulder_armor_RR": (sh_a.mirror("YZ").mirror("XZ"), FLAT_Y, 1, "ASA/PETG 白"),
    "top_armor_L":      (A.split_half(ta, True), FLAT_YP, 1, "ASA/PETG 白，沿中线分两半打印，切面朝下"),
    "top_armor_R":      (A.split_half(ta, False), FLAT_YN, 1, "ASA/PETG 白，切面朝下"),
    "handle":           (A.handle_v2(), KEEP, 1, "PA-CF，6 层壁"),
    "battery_door":     (A.battery_door(), FLIP, 1, "PETG-CF"),
    "head_shell":       (A.head_shell_v2(), FLIP, 1, "ASA/PETG 白，顶面朝下，前脸斜面加少量支撑"),
    "head_base":        (A.head_base_v2(), KEEP, 1, "PETG"),
    "ear_fin":          (A.ear_fin(+1, assembled=False), FLAT_Y, 2, "ASA/PETG 白，左右通用"),
    "tail_cowl_L":      (A.split_half(tl, True), FLAT_YP, 1, "ASA/PETG 白，沿中线分两半打印，切面朝下"),
    "tail_cowl_R":      (A.split_half(tl, False), FLAT_YN, 1, "ASA/PETG 白，切面朝下"),
    "tail_fin":         (A.tail_fin(), FLAT_Y, 1, "PA-CF"),
})

report = []
report.append(f"连杆中心距 = {ROD_L:.1f} mm   (曲柄 {CRANK_R} / 摇臂 {LEVER_R}，膝关节额外减速 {LEVER_R / CRANK_R:.2f}:1)")
report.append(f"站立髋高 = {-FOOT_Z:.1f} mm（地面到髋轴）")
bb_all = cq.Compound.makeCompound([s for _, s, _ in items]).BoundingBox()
report.append(f"整机包络（站立）: 长 {bb_all.xlen:.0f} × 宽 {bb_all.ylen:.0f} × 高 {bb_all.zmax - FOOT_Z:.0f} mm")
report.append("")
report.append(f"{'零件':<18}{'数量':>4}  {'打印尺寸 X×Y×Z (mm)':<26}{'体积cm3':>8}  床内?  材料")

total_print_g = 0.0
for name, (shape, orient, qty, mat) in printables.items():
    s = orient(shape)
    bb = s.BoundingBox()
    s = s.translate(cq.Vector(-bb.xmin, -bb.ymin, -bb.zmin))
    cq.exporters.export(cq.Workplane().add(s), os.path.join(OUT, "stl", f"{name}.stl"),
                        tolerance=0.05, angularTolerance=0.15)
    dims = sorted([bb.xlen, bb.ylen])
    fits = dims[1] <= BED and bb.zlen <= BED
    diag = (not fits) and math.hypot(*dims) <= BED * math.sqrt(2) - 10
    vol = shape.Volume() / 1000.0
    total_print_g += vol * qty * 1.15 * 0.55   # 粗估：密度 1.15，平均实心率 55%
    flag = "是" if fits else ("斜放" if diag else "否!")
    report.append(f"{name:<20}{qty:>3}  {bb.xlen:6.1f}×{bb.ylen:6.1f}×{bb.zlen:6.1f}      {vol:8.1f}  {flag:<4}  {mat}")
report.append("")
report.append(f"打印件总重粗估 ≈ {total_print_g / 1000:.2f} kg")

# ------------------------------------------------------------------ STEP 装配
asm = cq.Assembly(name="RoboDog_v0.1")
for i, (nm, s, c) in enumerate(items):
    asm.add(s, name=f"{i:03d}_{nm}", color=cq.Color(COL[c]))
asm.save(os.path.join(OUT, "step", "robodog_v0.1_assembly.step"))

# ------------------------------------------------------------------ GLB（给网页 3D 预览）
scene = trimesh.Scene()
groups = {}
for nm, s, c in items:
    vs, ts = s.tessellate(0.15, 0.25)
    v = np.array([[p.x, p.y, p.z] for p in vs], dtype=np.float64)
    t = np.array(ts, dtype=np.int64)
    if len(t) == 0:
        continue
    m = trimesh.Trimesh(vertices=v, faces=t, process=False)
    # 机身坐标 (X前 Y左 Z上) -> glTF (X右 Y上 Z前)：x_g = -y, y_g = z, z_g = x
    m.vertices = np.column_stack([-v[:, 1], v[:, 2] - FOOT_Z, v[:, 0]]) / 1000.0
    groups.setdefault(c, []).append(m)
for c, ms in groups.items():
    m = trimesh.util.concatenate(ms)
    rgb = [int(COL[c][i:i + 2], 16) for i in (1, 3, 5)]
    metallic = 0.85 if c in ("alu", "steel", "flange", "crank") else (0.35 if c == "motor" else 0.05)
    rough = 0.35 if c in ("alu", "steel", "flange", "crank") else (0.08 if c == "glass" else (0.45 if c == "armor" else 0.7))
    m.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
        name=c, baseColorFactor=rgb + [255], metallicFactor=metallic, roughnessFactor=rough,
        emissiveFactor=list(EMISSIVE.get(c, (0, 0, 0)))))
    scene.add_geometry(m, node_name=c, geom_name=c)
scene.export(os.path.join(OUT, "robodog.glb"))

open(os.path.join(OUT, "report.txt"), "w").write("\n".join(report) + "\n")
print("\n".join(report))
print(f"\n{len(items)} 个装配体零件，{len(printables)} 种打印件 -> {os.path.abspath(OUT)}")
