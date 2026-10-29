"""导出机械狗007A 的 MuJoCo 仿真模型：007a/out/mujoco/007a.xml + geo.json（控制器用的腿尺寸）

刚体：机身（自由浮动）+ 4 条腿 × 3 节（外摆 / 大腿 / 小腿），每节一个关节，共 12 个关节。
重量、重心、惯量：
  腿：按第 2 步的工程零件（eng_leg / eng_body.leg_parts_world）逐个算，密度和减重系数跟 mass.py 一样，电机按宇树 530 g；
  机身：mass.py 算出来的“机身（不含腿）”重量和重心（out/mass.json），惯量按机身盒子算；
  货：背上一块 cargo，run_sim.py 按场景改重量（0 或 5 kg）。
关节（宇树 GO-M8010-6，官方手册）：峰值 23.7 N·m、24 V 时最高 30 rad/s；膝经过同步带 1.5 倍 → 膝关节 35.5 N·m、20 rad/s。
  转速越高能出的扭矩越小（电压限制），这条曲线在 run_sim.py 里加。
碰撞：脚（球）、小腿、大腿、机身、头和地面 / 楼梯碰；机器人自己的零件之间不算。

用法：python 007a/sim/export_mujoco.py      （run_sim.py 会自动调用）
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import cadquery as cq  # noqa: E402

import eng_body as B  # noqa: E402
import eng_leg as E  # noqa: E402
import mass as MS  # noqa: E402
import model as M  # noqa: E402

OUT = os.path.join(ROOT, "out", "mujoco")
LEGS = [("FL", 1, 1), ("FR", 1, -1), ("RL", -1, 1), ("RR", -1, -1)]
GEAR_KNEE = E.Z_B / E.Z_S                      # 1.5
TAU_MOTOR = 23.7                               # 电机输出端峰值 N·m
W_MOTOR = 30.0                                 # 电机输出端最高转速 rad/s（24 V）
FOOT_R = 0.018                                 # 脚垫按半径 18 的球（底面就是 CAD 里的脚底）
# 电机转子折算到输出端的惯量（kg·m²）：转子约 2e-4 × 6.33²（估，★ 等宇树给数据）；膝再 × 1.5²
ARM_MOTOR = 2.0e-4 * 6.33 ** 2
FRICTION = 0.15                                # 减速器摩擦（N·m，估）


def f(v):
    return " ".join(f"{x:.6g}" for x in v)


def part_mass(n, s):
    """跟 mass.py 一样的算法：电机 530 g；皮带按 PU；小腿按半铝；再乘减重系数。"""
    if "电机" in n and "座" not in n:
        return MS.MOTOR_KG
    if "皮带" in n or "同步带" in n:
        rho = 1.2e-6
    elif "小腿" in n:
        rho = MS.RHO["铝"] * 0.5
    else:
        rho = MS.RHO[MS.material(n)]
    return s.Volume() * rho * MS.lighten(n)


def inertial(items, origin_mm):
    """items: [(kg, 实体 mm)] → (质量, 重心 m（相对 origin）, 惯量 3×3 kg·m²（绕重心）)。"""
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


def inertial_xml(m, c, I):
    I = (I + I.T) / 2
    ev = np.linalg.eigvalsh(I)
    if ev.min() <= 1e-7:                                   # 数值上压扁了就补一点
        I = I + np.eye(3) * (1e-6 - min(ev.min(), 0))
    return f'<inertial pos="{f(c)}" mass="{m:.5g}" fullinertia="{f([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]])}"/>'


def box_inertia(m, lx, ly, lz):
    return np.diag([m * (ly ** 2 + lz ** 2), m * (lx ** 2 + lz ** 2), m * (lx ** 2 + ly ** 2)]) / 12.0


def build():
    os.makedirs(OUT, exist_ok=True)
    HX, HZ = M.HIP_X, M.HIP_Z
    _, knee, foot = E.leg_module()
    kx, kz = knee
    fx0, fz0 = foot
    geo = dict(HX=HX / 1000, Y_HAA=E.Y_HAA / 1000, Y_LEG=E.Y_LEG / 1000, HZ=HZ / 1000, knee0=[kx / 1000, kz / 1000],
               foot0=[fx0 / 1000, (fz0 + FOOT_R * 1000) / 1000], foot_r=FOOT_R, gear_knee=GEAR_KNEE, tau=TAU_MOTOR, w=W_MOTOR,
               L2=M.L2 / 1000, W2=M.W2 / 1000, Z0=(M.Z0 - HZ) / 1000, TOP=(M.TOP - HZ) / 1000)
    mj = json.load(open(os.path.join(ROOT, "out", "mass.json"), encoding="utf-8"))
    bc = (np.array(mj["body_com"]) - np.array([0, 0, HZ])) / 1000
    bl, bw, bh = 2 * M.L2 / 1000, 2 * M.W2 / 1000, (M.TOP - M.Z0) / 1000
    zc = ((M.TOP + M.Z0) / 2 - HZ) / 1000
    Ib = box_inertia(mj["body"], bl, bw, bh)
    legs_xml, act_xml, sens_xml, link_mass = [], [], [], {}
    for name, fx, sy in LEGS:
        P = B.leg_parts_world(fx, sy)
        grp = {g: [(part_mass(n, s), s) for n, s, c, gg in P if gg == g] for g in ("机身", "外摆", "大腿", "小腿")}
        o_haa = np.array([fx * HX, sy * E.Y_HAA, HZ])
        o_th = np.array([fx * HX, sy * E.Y_LEG, HZ])
        o_sh = o_th + np.array([kx, 0, kz])
        mm, cm, Im = inertial(grp["机身"], o_haa)
        m1, c1, I1 = inertial(grp["外摆"], o_haa)
        m2, c2, I2 = inertial(grp["大腿"], o_th)
        m3, c3, I3 = inertial(grp["小腿"], o_sh)
        link_mass[name] = dict(haa_motor=mm, haa=m1, thigh=m2, shank=m3)
        foot_rel = np.array([fx0 - kx, 0, fz0 + FOOT_R * 1000 - kz]) / 1000
        knee_rel = np.array([kx, 0, kz]) / 1000
        dy = (o_th - o_haa) / 1000
        legs_xml.append(f"""
    <body name="{name}_haa_motor" pos="{f((o_haa - [0, 0, HZ]) / 1000)}">{inertial_xml(mm, cm, Im)}</body>
    <body name="{name}_hip" pos="{f((o_haa - [0, 0, HZ]) / 1000)}">
      {inertial_xml(m1, c1, I1)}
      <joint name="{name}_haa" axis="1 0 0" range="-0.45 0.45" armature="{ARM_MOTOR:.4g}" frictionloss="{FRICTION}" damping="0.05"/>
      <geom type="cylinder" size="0.048 0.021" pos="{f(dy * 0.5)}" euler="1.5708 0 0" rgba=".25 .27 .3 1"/>
      <body name="{name}_thigh" pos="{f(dy)}">
        {inertial_xml(m2, c2, I2)}
        <joint name="{name}_hfe" axis="0 1 0" range="-1.9 1.9" armature="{ARM_MOTOR:.4g}" frictionloss="{FRICTION}" damping="0.05"/>
        <geom class="hit" type="capsule" fromto="0 0 0 {f(knee_rel)}" size="0.032" rgba=".85 .87 .9 1"/>
        <body name="{name}_shank" pos="{f(knee_rel)}">
          {inertial_xml(m3, c3, I3)}
          <joint name="{name}_kfe" axis="0 1 0" range="-1.6 1.6" armature="{ARM_MOTOR * GEAR_KNEE ** 2:.4g}" frictionloss="{FRICTION * GEAR_KNEE}" damping="0.05"/>
          <geom class="hit" type="capsule" fromto="0 0 0 {f(foot_rel * 0.85)}" size="0.016" rgba=".3 .32 .35 1"/>
          <geom name="{name}_foot" class="foot" type="sphere" pos="{f(foot_rel)}" size="{FOOT_R}" rgba=".1 .1 .1 1"/>
          <site name="{name}_foot" pos="{f(foot_rel)}" size="0.005"/>
        </body>
      </body>
    </body>""")
        for j, g in (("haa", 1), ("hfe", 1), ("kfe", GEAR_KNEE)):
            act_xml.append(f'<motor name="{name}_{j}" joint="{name}_{j}" gear="{g}" ctrlrange="{-TAU_MOTOR} {TAU_MOTOR}" ctrllimited="true"/>')
        sens_xml.append(f'<touch name="{name}_touch" site="{name}_foot"/>')
    hx0 = (M.L2 + 10) / 1000
    xml = f"""<mujoco model="007A">
  <compiler angle="radian"/>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096"/></visual>
  <default>
    <geom contype="0" conaffinity="0" friction="0.8 0.02 0.001"/>
    <default class="hit"><geom contype="0" conaffinity="1"/></default>
    <default class="foot"><geom contype="0" conaffinity="1" friction="1.0 0.02 0.001" solref="0.004 1"/></default>
    <default class="terrain"><geom contype="1" conaffinity="0" rgba=".72 .72 .7 1"/></default>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".55 .57 .6" rgb2=".5 .52 .55" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8"/>
  </asset>
  <worldbody>
    <light pos="1 -1 3" dir="-.3 .3 -1" directional="true"/>
    <geom name="floor" class="terrain" type="plane" size="20 20 .1" material="grid"/>
    <!--TERRAIN-->
    <body name="base" pos="0 0 {HZ / 1000:.4f}">
      <freejoint/>
      {inertial_xml(mj["body"], bc, Ib)}
      <geom name="body" class="hit" type="box" size="{bl / 2:.4f} {bw / 2:.4f} {bh / 2:.4f}" pos="0 0 {zc:.4f}" rgba=".9 .91 .92 1"/>
      <geom name="head" class="hit" type="box" size="0.02 0.11 0.045" pos="{hx0:.4f} 0 {zc + 0.01:.4f}" rgba=".15 .16 .18 1"/>
      <site name="imu" pos="0 0 0" size="0.01"/>
      <body name="cargo" pos="0 0 {(M.TOP - HZ) / 1000 + 0.04:.4f}">
        <inertial pos="0 0 0" mass="5" diaginertia="0.02 0.04 0.05"/>
        <geom name="cargo" type="box" size="0.15 0.1 0.04" rgba=".85 .6 .2 1"/>
      </body>
{"".join(legs_xml)}
    </body>
  </worldbody>
  <actuator>
    {chr(10).join("    " + a for a in act_xml)}
  </actuator>
  <sensor>
    <framequat name="quat" objtype="site" objname="imu"/>
    <gyro name="gyro" site="imu"/>
    {chr(10).join("    " + s for s in sens_xml)}
  </sensor>
</mujoco>
"""
    open(os.path.join(OUT, "007a.xml"), "w", encoding="utf-8").write(xml)
    geo["link_mass"] = link_mass
    geo["body_mass"] = mj["body"]
    json.dump(geo, open(os.path.join(OUT, "geo.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    return xml, geo


if __name__ == "__main__":
    xml, geo = build()
    import mujoco
    m = mujoco.MjModel.from_xml_string(xml)                     # 路径有中文，MuJoCo 打不开文件，直接给字符串
    tot = sum(m.body_mass) - m.body_mass[m.body("cargo").id]
    print(f"整机（不含货）{tot:.2f} kg，关节 {m.njnt - 1} 个")
    for k, v in geo["link_mass"].items():
        print(k, {a: round(b, 3) for a, b in v.items()})
