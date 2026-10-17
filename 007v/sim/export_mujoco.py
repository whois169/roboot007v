"""导出机械狗007v的 MuJoCo 仿真模型：007v/out/mujoco/007v.xml + meshes/*.stl

刚体：机身（自由浮动）、屏幕头、4 个摆臂、4 个麦轮、每个麦轮 12 个能自由转的 45° 滚子。
重量、重心、惯量按 CAD 实体逐个零件算（重量表在 007v/config.py）；外壳 1.5 kg 按机身盒子算。
碰撞：只有滚子（每个滚子 7 个球，外缘落在 Φ152 圆柱上）、机身盒子、滑撬、头部盒子和地面碰，不算自碰撞。
执行器都是力矩电机，控制器在 run_sim.py 里写（模拟真实电机的位置/速度环 + 力矩上限）。
货物：机身上的 cargo 刚体，run_sim.py 按场景把 PAYLOAD_MASS 换成 0 或 10。

用法：python 007v/sim/export_mujoco.py      （run_sim.py 会自动调用）
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import cadquery as cq  # noqa: E402
import trimesh  # noqa: E402

import build as B  # noqa: E402
import model as M  # noqa: E402
from config import *  # noqa: E402,F401,F403

MJ = os.path.join(ROOT, "out", "mujoco")
MESH = os.path.join(MJ, "meshes")
R = WHEEL_D / 2
WHEEL_YC = BODY_W / 2 + WHEEL_GAP + WHEEL_W / 2
ROLLER_MASS = PART_MASS["麦轮滚子"]
ROLLER_SPHERES = 7
SIGN_S = {"FL": -1, "FR": 1, "RL": 1, "RR": -1}          # 着地滚子轴线朝“前右”(-1) 还是“前左”(+1)，做成 X 形布置

# 执行器上限（力矩，N·m）。髋放宽到 60 只是为了测“需要多少”，合格线在 run_sim.py 里按 20.7 判
HIP_CTRL, WHEEL_CTRL, HEAD_CTRL = 60.0, 5.6, 1.9
HIP_ARMATURE, WHEEL_ARMATURE, HEAD_ARMATURE = 0.1, 0.003, 0.005
WHEEL_FRICTIONLOSS, ROLLER_FRICTIONLOSS = 0.05, 0.002


def f(v):
    return " ".join(f"{x:.6g}" for x in v)


def inertial(items, origin_mm):
    """items: [(质量 kg, cadquery 实体)]，实体坐标是 mm。返回 (质量, 重心 m（相对 origin）, 惯量 3×3 kg·m²，绕重心)。"""
    items = [(m, s) for m, s in items if m > 0]
    mt = sum(m for m, _ in items)
    cs = [np.array(s.Center().toTuple()) for _, s in items]
    c = sum(m * ci for (m, _), ci in zip(items, cs)) / mt
    I = np.zeros((3, 3))
    for (m, s), ci in zip(items, cs):
        Ip = np.array(cq.Shape.matrixOfInertia(s)) * (m / s.Volume())
        r = ci - c
        I += Ip + m * (r @ r * np.eye(3) - np.outer(r, r))
    return mt, (c - np.asarray(origin_mm)) / 1000.0, I * 1e-6


def box_inertia(m, lx, ly, lz):
    return np.diag([m * (ly ** 2 + lz ** 2), m * (lx ** 2 + lz ** 2), m * (lx ** 2 + ly ** 2)]) / 12.0


def add_point(mt, c, I, m2, c2, I2):
    """把一个（质量、重心、绕自身重心惯量）合进去。"""
    m = mt + m2
    cn = (mt * c + m2 * c2) / m
    out = np.zeros((3, 3))
    for mm, cc, II in ((mt, c, I), (m2, c2, I2)):
        r = cc - cn
        out += II + mm * (r @ r * np.eye(3) - np.outer(r, r))
    return m, cn, out


CARGO_SIZE = (0.18, 0.18, 0.2)                            # 货物箱（m），惯量按 1 kg 算，run_sim.py 乘真实重量
CARGO_I1 = np.diag(box_inertia(1.0, *CARGO_SIZE))


def inertial_xml(m, c, I):
    return f'<inertial pos="{f(c)}" mass="{m:.5g}" fullinertia="{I[0, 0]:.6g} {I[1, 1]:.6g} {I[2, 2]:.6g} {I[0, 1]:.6g} {I[0, 2]:.6g} {I[1, 2]:.6g}"/>'


def export_meshes(name, parts, origin_mm):
    """按颜色把零件合成 STL（米、刚体局部坐标），返回 [(mesh 名, 颜色)]。"""
    bycol = {}
    for p in parts:
        if p.c == "envelope":
            continue
        vs, fs = p.s.tessellate(0.3, 0.3)
        if not fs:
            continue
        v = (np.array([[q.x, q.y, q.z] for q in vs]) - origin_mm) / 1000.0
        bycol.setdefault(p.c, []).append(trimesh.Trimesh(vertices=v, faces=np.array(fs), process=False))
    out = []
    for c, ms in bycol.items():
        mn = f"{name}_{c}"
        trimesh.util.concatenate(ms).export(os.path.join(MESH, mn + ".stl"))
        out.append((mn, c))
    return out


from look import COL  # noqa: E402

RGBA = {c: [int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)] + [1] for c, h in COL.items()}


def main():
    os.makedirs(MESH, exist_ok=True)
    parts = B.PARTS
    assets, vis = [], {}

    def meshes(name, ps, origin):
        lst = export_meshes(name, ps, np.asarray(origin, float))
        for mn, _ in lst:
            assets.append(f'<mesh name="{mn}" file="{mn}.stl"/>')
        vis[name] = "".join(f'<geom class="vis" mesh="{mn}" rgba="{f(RGBA[c])}"/>' for mn, c in lst)

    # ---- 机身（含外壳）
    body_parts = [p for p in parts if p.g == "body"]
    mb, cb, Ib = inertial([(p.m, p.s) for p in body_parts], (0, 0, 0))
    mb, cb, Ib = add_point(mb, cb, Ib, MASS_SHELLS, np.array([0, 0, BODY_H / 2000]),
                           box_inertia(MASS_SHELLS, BODY_L / 1000, BODY_W / 1000, BODY_H / 1000))
    meshes("body", body_parts, (0, 0, 0))

    # ---- 头
    head_parts = [p for p in parts if p.g == "head"]
    mh, ch, Ih = inertial([(p.m, p.s) for p in head_parts], (0, 0, 0))
    meshes("head", head_parts, (0, 0, 0))
    pivot = np.array([NECK_X, 0, M.NECK_Z + PIVOT_UP]) / 1000

    # ---- 腿
    legs_xml = []
    for k, (fx, sy, tag) in B.LEGS.items():
        arm = [p for p in parts if p.g == k and not B.is_wheel(p.name)]
        wheel = [p for p in parts if p.g == k and B.is_wheel(p.name) and "滚子" not in p.name]
        wc = np.array([fx * ARM, sy * WHEEL_YC, 0.0])                    # 轮心（摆臂坐标，mm）
        ma, ca, Ia = inertial([(p.m, p.s) for p in arm], (0, 0, 0))
        mw, cw, Iw = inertial([(p.m, p.s) for p in wheel], wc)
        meshes(k + "_arm", arm, (0, 0, 0))
        meshes(k + "_wheel", wheel, wc)
        rollers = []
        for i in range(ROLLERS):
            p, a = M.roller_axis(i, -1)                                     # 先按左前轮算，再镜像
            p, a = np.array(p), np.array(a)
            if sy < 0:
                p[1], a[1] = -p[1], -a[1]
            if fx < 0:
                p[0], a[0] = -p[0], -a[0]
            ri = 0.4 * ROLLER_MASS * (ROLLER_R0 / 1000) ** 2
            spheres = "".join(
                f'<geom class="roller" pos="{f(a * t / 1000)}" size="{M.roller_radius(t) / 1000:.5g}"/>'
                for t in np.linspace(-ROLLER_H, ROLLER_H, ROLLER_SPHERES))
            rollers.append(
                f'<body name="{k}_r{i}" pos="{f(p / 1000)}"><joint name="{k}_r{i}" class="roller" axis="{f(a)}"/>'
                f'<inertial pos="0 0 0" mass="{ROLLER_MASS}" diaginertia="{ri:.3g} {ri:.3g} {ri:.3g}"/>{spheres}</body>')
        legs_xml.append(f'''
      <body name="{k}" pos="{f(np.array([fx * HIP_X, 0, HIP_Z]) / 1000)}">
        <joint name="hip_{k}" axis="0 {fx} 0" armature="{HIP_ARMATURE}" damping="0.5"/>
        {inertial_xml(ma, ca, Ia)}{vis[k + "_arm"]}
        <body name="{k}_wheel" pos="{f(wc / 1000)}">
          <joint name="wheel_{k}" axis="0 1 0" armature="{WHEEL_ARMATURE}" damping="0.01" frictionloss="{WHEEL_FRICTIONLOSS}"/>
          {inertial_xml(mw, cw, Iw)}{vis[k + "_wheel"]}
          {"".join(rollers)}
        </body>
      </body>''')

    # ---- 货物（货斗中间，按 20 cm 高的一箱）
    cargo_c = np.array([(BED_X0 + BED_X1) / 2, 0, M.DECK + PAYLOAD_UP]) / 1000
    bz0 = (R - HIP_Z + ARM * math.sin(math.radians(STANCE_DEG))) / 1000
    qpos_stand = [0, 0, bz0, 1, 0, 0, 0]
    # 关节顺序：4 条腿（髋、轮、12 个滚子）、头
    for k in B.LEGS:
        qpos_stand += [math.radians(STANCE_DEG), 0] + [0] * ROLLERS
    qpos_stand += [0]
    xml = f'''<mujoco model="dog007v">
  <!-- 由 007v/sim/export_mujoco.py 生成，不要手改 -->
  <compiler angle="radian" meshdir="meshes" autolimits="true"/>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <default>
    <geom contype="2" conaffinity="1" friction="0.9 0.005 0.0001" solref="0.004 1" solimp="0.9 0.95 0.001"/>
    <default class="vis"><geom type="mesh" contype="0" conaffinity="0" group="1" mass="0"/></default>
    <default class="roller">
      <geom type="sphere" condim="3" rgba="0.15 0.16 0.18 1" group="2"/>
      <joint damping="0.0002" armature="1e-5" frictionloss="{ROLLER_FRICTIONLOSS}"/>
    </default>
    <default class="col"><geom group="3" rgba="0.8 0.3 0.3 0.3"/></default>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.1 0.12 0.14" rgb2="0.14 0.17 0.19" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.1"/>
    {"".join(assets)}
  </asset>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="20 20 0.1" material="grid" contype="1" conaffinity="1"/>
    <!--OBSTACLES-->
    <body name="base" pos="0 0 {bz0:.5g}">
      <freejoint name="base"/>
      {inertial_xml(mb, cb, Ib)}{vis["body"]}
      <geom name="body_box" class="col" type="box" pos="0 0 {M.DECK / 2000:.5g}" size="{BODY_L / 2000:.5g} {BODY_W / 2000:.5g} {M.DECK / 2000:.5g}"/>
      <geom name="skid_L" class="col" type="box" pos="0 0.08 -0.003" size="0.15 0.01 0.003"/>
      <geom name="skid_R" class="col" type="box" pos="0 -0.08 -0.003" size="0.15 0.01 0.003"/>
      <geom name="pods" class="col" type="box" pos="{NECK_X / 1000 + 0.016:.5g} 0 {(M.DECK + 70) / 1000:.5g}" size="0.062 0.12 0.055"/>
      <body name="cargo" pos="{f(cargo_c)}">
        <inertial pos="0 0 0" mass="PAYLOAD_MASS" diaginertia="PAYLOAD_INERTIA"/>
        <geom class="vis" type="box" size="0.09 0.09 0.1" rgba="0.9 0.65 0.25 0.35"/>
      </body>
      {"".join(legs_xml)}
      <body name="head" pos="{f(pivot)}">
        <joint name="head" axis="0 1 0" armature="{HEAD_ARMATURE}" damping="0.05"/>
        {inertial_xml(mh, ch, Ih)}{vis["head"]}
      </body>
    </body>
  </worldbody>
  <actuator>
    {"".join(f'<motor name="hip_{k}" joint="hip_{k}" ctrlrange="-{HIP_CTRL} {HIP_CTRL}"/>' for k in B.LEGS)}
    {"".join(f'<motor name="wheel_{k}" joint="wheel_{k}" ctrlrange="-{WHEEL_CTRL} {WHEEL_CTRL}"/>' for k in B.LEGS)}
    <motor name="head" joint="head" ctrlrange="-{HEAD_CTRL} {HEAD_CTRL}"/>
  </actuator>
  <keyframe><key name="stand" qpos="{f(qpos_stand)}"/></keyframe>
</mujoco>
'''
    with open(os.path.join(MJ, "007v.xml"), "w", encoding="utf-8") as fo:
        fo.write(xml)
    print(f"导出 007v/out/mujoco/007v.xml：机身 {mb:.2f} kg，头 {mh:.2f} kg，网格 {len(assets)} 个")


if __name__ == "__main__":
    main()
