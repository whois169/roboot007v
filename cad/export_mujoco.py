"""
导出 MuJoCo 仿真模型（MJCF）：
  out/mujoco/robodog.xml    机器狗本体：15 个刚体、14 个关节、执行器、传感器、4 个姿态关键帧
  out/mujoco/scene.xml      场景：地面 + 灯光 + 引用 robodog.xml（打开这个）
  out/mujoco/meshes/*.stl   显示用网格（按 刚体_颜色 分开，零位、刚体局部坐标、米）
质量、重心、惯量按 CAD 实体逐个零件计算（外购件按公开质量、打印件按材料密度 × 填充率）。
碰撞用简化几何（机身盒子、腿胶囊、足端球），只和地面碰撞，不算自碰撞。
用法：cd cad && python3 export_mujoco.py
"""
import math
import os
import sys

import numpy as np

sys.argv = [sys.argv[0]]
import build as B          # noqa: E402
import cadquery as cq      # noqa: E402
import trimesh             # noqa: E402
import head as H           # noqa: E402
from config import *       # noqa: E402

MJ = os.path.join(B.OUT, "mujoco")
MESH = os.path.join(MJ, "meshes")
os.makedirs(MESH, exist_ok=True)

ARMATURE_HIP = 0.005     # 反射到关节的转子惯量（kg·m²），DM-J4310 10:1 估算
ARMATURE_KNEE = 0.016    # 膝再经过 1.8 倍皮带：×1.8²
DAMPING, FRICTIONLOSS = 0.05, 0.1
TAU_PEAK = 7.0           # DM-J4310 峰值 N·m
NECK_TAU = 0.15          # GM4108 量级


def jpt(g):
    return np.zeros(3) if g == "body" else np.array(B.joints[g]["point"], float)


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


# ---------------- 网格 + 惯量
mesh_assets, geoms_vis, inertials = [], {}, {}
for g, lst in B.groups.items():
    P = jpt(g)
    bycol = {}
    M, mc = 0.0, np.zeros(3)
    parts = []
    for nm, s, c in lst:
        if c == "keepout":
            continue
        vs, ts = s.tessellate(0.3 if "带轮" in nm else 0.2, 0.35)
        if ts:
            v = (np.array([[p.x, p.y, p.z] for p in vs]) - P) / 1000.0
            bycol.setdefault(c, []).append(trimesh.Trimesh(vertices=v, faces=np.array(ts), process=False))
        m = B.mass_of(nm, s, c)
        if m <= 0:
            continue
        vol = s.Volume()
        I = np.array(cq.Shape.matrixOfInertia(s)) * (m / vol) * 1e-6          # kg·m²，绕零件自身重心
        cp = (np.array(cq.Shape.centerOfMass(s).toTuple()) - P) / 1000.0
        parts.append((m, cp, I))
        M += m
        mc += m * cp
    com = mc / M
    Ig = np.zeros((3, 3))
    for m, cp, I in parts:
        d = cp - com
        Ig += I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    inertials[g] = (M, com, Ig)
    geoms_vis[g] = []
    for c, ms in bycol.items():
        name = f"{g}_{c}"
        trimesh.util.concatenate(ms).export(os.path.join(MESH, f"{name}.stl"))
        mesh_assets.append(name)
        rgb = [int(B.COL[c][i:i + 2], 16) / 255 for i in (1, 3, 5)]
        geoms_vis[g].append((name, rgb + [1.0]))

# ---------------- 碰撞几何（刚体局部坐标，米）
LEG_Y = {"thigh": (Y_INNER[0] + Y_OUTER[1]) / 2 / 1000, "shank": (SHANK_Y[0] + SHANK_Y[1]) / 2 / 1000}


def collision(g):
    out = []
    if g == "body":
        out.append(f'<geom name="col_body" class="col" type="box" pos="0 0 0.009" size="0.150 {(BODY_W / 2 + 5) / 1000:.3f} 0.058"/>')
        out.append('<geom name="col_tail" class="col" type="box" pos="-0.185 0 0.060" size="0.030 0.035 0.012"/>')
    elif g == "neck_pitch":
        p = np.array([186.0, 0, 102.0]) / 1000 - jpt(g) / 1000
        out.append(f'<geom name="col_head" class="col" type="box" pos="{fmt(p)}" size="0.064 0.054 0.030"/>')
    elif g.endswith("_hfe"):
        sy = 1 if g[1] == "L" else -1
        y = sy * LEG_Y["thigh"]
        out.append(f'<geom name="col_{g}" class="col" type="capsule" fromto="0 {y:.4f} 0 0 {y:.4f} {-L_THIGH / 1000:.4f}" size="0.036"/>')
    elif g.endswith("_knee"):
        sy = 1 if g[1] == "L" else -1
        y = sy * LEG_Y["shank"]
        out.append(f'<geom name="col_{g}" class="col" type="capsule" fromto="0 {y:.4f} -0.02 0 {y:.4f} {-(L_SHANK - 25) / 1000:.4f}" size="0.017"/>')
        out.append(f'<geom name="foot_{g[:2]}" class="foot" type="sphere" pos="0 {y:.4f} {-L_SHANK / 1000:.4f}" size="{FOOT_D / 2000:.4f}"/>')
        out.append(f'<site name="foot_{g[:2]}" pos="0 {y:.4f} {-L_SHANK / 1000:.4f}" size="{FOOT_D / 2000 + 0.001:.4f}" type="sphere" rgba="1 0.4 0 0"/>')
    return out


RANGE = {"haa": (-30, 30), "hfe": (-95, 95), "knee": (-150, 12), "neck_yaw": H.YAW_RANGE, "neck_pitch": H.PITCH_RANGE}


def kind(j):
    return j.split("_", 1)[1] if j[:2] in ("FL", "FR", "RL", "RR") else j


children = {}
for j, d in B.joints.items():
    children.setdefault(d["parent"], []).append(j)
ORDER = []


def body_xml(g, indent):
    sp = "  " * indent
    lines = []
    M, com, I = inertials[g]
    if g == "body":
        lines.append(f'{sp}<body name="base" pos="0 0 {B.POSES["stand"]["body_h"] / 1000:.4f}" childclass="robodog">')
        lines.append(f'{sp}  <freejoint name="root"/>')
    else:
        rel = (jpt(g) - jpt(B.joints[g]["parent"])) / 1000
        lines.append(f'{sp}<body name="{g}" pos="{fmt(rel)}">')
        lo, hi = RANGE[kind(g)]
        arm = ARMATURE_KNEE if kind(g) == "knee" else (0.0005 if kind(g).startswith("neck") else ARMATURE_HIP)
        lines.append(f'{sp}  <joint name="{g}" axis="{fmt(B.joints[g]["axis"])}" range="{lo} {hi}" armature="{arm}"/>')
        ORDER.append(g)
    lines.append(f'{sp}  <inertial pos="{fmt(com)}" mass="{M:.4f}" fullinertia="{I[0,0]:.6g} {I[1,1]:.6g} {I[2,2]:.6g} {I[0,1]:.6g} {I[0,2]:.6g} {I[1,2]:.6g}"/>')
    for name, rgba in geoms_vis[g]:
        lines.append(f'{sp}  <geom class="vis" mesh="{name}" rgba="{fmt(rgba)}"/>')
    for c in collision(g):
        lines.append(f"{sp}  {c}")
    if g == "body":
        lines.append(f'{sp}  <site name="imu" pos="-0.008 0 0.012" size="0.006" rgba="0 1 0 0.5"/>')
        a = math.radians(35)
        for sx, nm in ((1, "cliff_front"), (-1, "cliff_rear")):
            lines.append(f'{sp}  <site name="{nm}" pos="{sx * 0.131} 0 -0.040" zaxis="{sx * math.sin(a):.4f} 0 {-math.cos(a):.4f}" size="0.004" rgba="1 0 0 0.5"/>')
    if g == "neck_pitch":
        P = jpt(g)
        tof = (np.array([H.HEAD_FRONT_X, H.TOF_POS[0], H.TOF_POS[1]]) - P) / 1000
        cam = (np.array([H.HEAD_FRONT_X, H.LENS[0], H.LENS[1]]) - P) / 1000
        lines.append(f'{sp}  <site name="tof_front" pos="{fmt(tof)}" zaxis="1 0 0" size="0.004" rgba="1 0 0 0.5"/>')
        lines.append(f'{sp}  <camera name="head_cam" pos="{fmt(cam)}" xyaxes="0 -1 0 0 0 1" fovy="70"/>')
    for ch in children.get(g, []):
        lines += body_xml(ch, indent + 1)
    lines.append(f"{sp}</body>")
    return lines


tree = body_xml("body", 2)


def qpos(pose):
    ang = B.joint_angles(pose)
    p = math.radians(pose["body_pitch"])
    q = [0, 0, pose["body_h"] / 1000 + 0.001, math.cos(p / 2), 0, math.sin(p / 2), 0]
    return q + [math.radians(ang[j]) for j in ORDER]


acts = []
for j in ORDER:
    k = kind(j)
    if k.startswith("neck"):
        acts.append(f'    <motor name="{j}" joint="{j}" ctrlrange="{-NECK_TAU} {NECK_TAU}"/>')
    else:
        gear = KNEE_RATIO if k == "knee" else 1.0
        acts.append(f'    <motor name="{j}" joint="{j}" gear="{gear:g}" ctrlrange="{-TAU_PEAK} {TAU_PEAK}"/>')

sensors = ['    <framequat name="imu_quat" objtype="site" objname="imu"/>',
           '    <gyro name="imu_gyro" site="imu"/>',
           '    <accelerometer name="imu_acc" site="imu"/>',
           '    <rangefinder name="tof_front" site="tof_front"/>',
           '    <rangefinder name="cliff_front" site="cliff_front"/>',
           '    <rangefinder name="cliff_rear" site="cliff_rear"/>']
for leg in ("FL", "FR", "RL", "RR"):
    sensors.append(f'    <touch name="foot_{leg}" site="foot_{leg}"/>')

# 蹲伏：四条腿折到身体正下方（从趴下起身的中间姿态，仿真里验证过：趴下 -> 蹲伏 -> 站立）
_d = 135.0 - FOOT_D / 2
_aw, _b = B.ik(0.0, -_d)
crouch = B.make_pose({c: B.leg_pose(STAND_HAA, _aw, _b) for c, *_ in B.LEGS})
crouch["body_h"] = 135.0
POSES_MJ = dict(B.POSES, crouch=crouch)
keys = []
for pn, pose in POSES_MJ.items():
    keys.append(f'    <key name="{pn}" qpos="{fmt(qpos(pose))}"/>')

xml = f"""<!-- RoboDog v0.5 · 由 cad/export_mujoco.py 从 CAD 自动生成，不要手改（改 CAD 后重新导出） -->
<mujoco model="robodog_v0.5">
  <compiler angle="degree" meshdir="meshes" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast"/>

  <default>
    <default class="robodog">
      <joint damping="{DAMPING}" frictionloss="{FRICTIONLOSS}"/>
      <default class="vis">
        <geom type="mesh" contype="0" conaffinity="0" group="2" density="0"/>
      </default>
      <default class="col">
        <geom contype="0" conaffinity="1" group="3" density="0" rgba="0.9 0.5 0.1 0.25"/>
      </default>
      <default class="foot">
        <geom contype="0" conaffinity="1" group="3" density="0" friction="0.9 0.02 0.001" condim="3" rgba="0.1 0.1 0.1 0.4"/>
      </default>
    </default>
  </default>

  <asset>
{chr(10).join(f'    <mesh name="{n}" file="{n}.stl"/>' for n in mesh_assets)}
  </asset>

  <worldbody>
{chr(10).join(tree)}
  </worldbody>

  <actuator>
    <!-- 力矩控制。ctrl = 电机端力矩（N·m）；膝关节 gear=1.8 = 同步带减速，关节端力矩 = 1.8 × ctrl -->
{chr(10).join(acts)}
  </actuator>

  <sensor>
{chr(10).join(sensors)}
  </sensor>

  <keyframe>
{chr(10).join(keys)}
  </keyframe>
</mujoco>
"""
open(os.path.join(MJ, "robodog.xml"), "w", encoding="utf-8").write(xml)

scene = """<mujoco model="robodog_scene">
  <include file="robodog.xml"/>
  <statistic center="0 0 0.25" extent="0.9"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="135" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.12 0.14 0.17" rgb2="0.03 0.04 0.05" width="512" height="512"/>
    <texture type="2d" name="grid" builtin="checker" mark="edge" rgb1="0.2 0.22 0.25" rgb2="0.16 0.18 0.2" markrgb="0.35 0.38 0.42" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="0.6 0.8 2.0" dir="-0.3 -0.4 -1" directional="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid" contype="1" conaffinity="1" friction="0.9 0.02 0.001"/>
  </worldbody>
</mujoco>
"""
open(os.path.join(MJ, "scene.xml"), "w", encoding="utf-8").write(scene)
tot = sum(v[0] for v in inertials.values())
print(f"MuJoCo 模型 -> {os.path.abspath(MJ)}；{len(mesh_assets)} 个网格，{len(ORDER)} 个关节，总质量 {tot:.2f} kg")
