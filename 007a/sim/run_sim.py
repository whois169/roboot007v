"""机械狗007A MuJoCo 仿真（第 3 步）：电机够不够、站不站得稳、楼梯上不上得去、电池能用多久。

场景（背上货 5 kg，除非写了“空载”）：
  standup     从趴着站起来，再趴下
  stand       站着不动 3 秒
  trot        对角小跑：平地加速到 0.5 m/s，跑一段，再停下
  trot_fast   对角小跑 0.8 m/s（看极限）
  stairs_up   住宅楼梯 175 高 × 260 宽 × 4 级，正着上（静步：一次只抬一条腿，重心始终在三条腿围成的三角形里）
  stairs_back 同一个楼梯，倒着上（膝朝后的腿倒着上，膝和小腿离踏步边更远）
  stairs_down 同一个楼梯，正着下
  stairs_pub  公共楼梯 150 × 300 × 4 级，正着上
  walk_cable  静步跨过地上 60 高的电缆槽（巡检电缆隧道常见）
用法：python 007a/sim/run_sim.py [场景名…] [--gif]     不写场景就全跑；--gif 另外出动图
输出：007a/out/sim_report.txt、sim_result.json；docs/机械狗007A/仿真_*.png（扭矩曲线）、仿真_*.gif（动图）

电机模型（宇树 GO-M8010-6 官方手册）：
  峰值 23.7 N·m，24 V 最高 30 rad/s；电池 6S 标称 22.2 V → 最高约 27.8 rad/s。
  转速超过一半以后能出的扭矩直线往下掉到 0（★ 估计，等宇树给扭矩—转速曲线）。
  膝经过 20/30 同步带：膝关节扭矩 = 电机 × 1.5，膝关节转速 = 电机 ÷ 1.5。
  电流 = 扭矩 ÷ 0.639（官方转矩常数，按输出端）；线圈电阻按 0.25 Ω 估（★）算发热。
控制：跟真机一样每个电机做位置 + 速度环（PD），再加上按重量分配的前馈扭矩；脚的落点、机身姿势由上层规划（这里写死在场景里，
  真机上由深度相机看台阶给出）。
"""
import json
import math
import os
import sys
import time

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "out")
DOC = os.path.join(os.path.dirname(ROOT), "docs", "机械狗007A")
XML = os.path.join(OUT, "mujoco", "007a.xml")
GEO = os.path.join(OUT, "mujoco", "geo.json")
LEGS = ["FL", "FR", "RL", "RR"]
SIDE = dict(FL=(1, 1), FR=(1, -1), RL=(-1, 1), RR=(-1, -1))
JN = ["haa", "hfe", "kfe"]
G = 9.81
V_BAT = 22.2
KT, R_COIL = 0.639, 0.25                 # 转矩常数（输出端，官方）、线圈电阻（★ 估）
P_ELEC = 25.0                            # 树莓派 + 传感器 + 屏幕 + 灯（W，估）
BAT_WH = 280.0 * 0.85                    # 电池 280 Wh，用到 85 %
DT_CTRL = 0.001                          # 控制周期 1 ms（宇树电机本身 6 kHz 环，这里是上层）

sys.path.insert(0, ROOT)
SC_L1, SC_LS = 200, 190
if not os.path.exists(XML):
    sys.path.insert(0, HERE)
    import export_mujoco
    export_mujoco.build()
GEOM = json.load(open(GEO, encoding="utf-8"))
HX, Y_HAA, Y_LEG = GEOM["HX"], GEOM["Y_HAA"], GEOM["Y_LEG"]
K0, F0 = np.array(GEOM["knee0"]), np.array(GEOM["foot0"])
L1, LSH = np.linalg.norm(K0), np.linalg.norm(F0 - K0)
FOOT_R = GEOM["foot_r"]
GEAR = np.array([1.0, 1.0, GEOM["gear_knee"]] * 4)
TAU_M, W_M = GEOM["tau"], GEOM["w"] * V_BAT / 24.0
LIMIT_J = TAU_M * GEAR                  # 关节端峰值扭矩：23.7 / 23.7 / 35.5
CROSS0 = F0[0] * K0[1] - F0[1] * K0[0]  # 站立时“膝在后”那一侧的符号


def ang(v):
    return math.atan2(v[1], v[0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ik(leg, p):
    """脚（球心）在机身坐标的位置 → (外摆, 髋, 膝) 关节角（相对 CAD 站立姿势，弧度）。"""
    fx, sy = SIDE[leg]
    rel = p - np.array([fx * HX, sy * Y_HAA, 0.0])
    o = sy * (Y_LEG - Y_HAA)
    y, z = rel[1], rel[2]
    zl = -math.sqrt(max(y * y + z * z - o * o, 1e-6))
    th = wrap(math.atan2(z, y) - math.atan2(zl, o))
    t = np.array([rel[0], zl])
    d = float(np.clip(np.linalg.norm(t), abs(L1 - LSH) + 1e-3, L1 + LSH - 1e-3))
    a = math.acos((L1 ** 2 + d ** 2 - LSH ** 2) / (2 * L1 * d))
    best = None
    for s in (1, -1):
        Kd = L1 * np.array([math.cos(ang(t) + s * a), math.sin(ang(t) + s * a)])
        if (t[0] * Kd[1] - t[1] * Kd[0]) * CROSS0 > 0:
            best = Kd
    qh = wrap(ang(K0) - ang(best))
    qk = wrap(ang(F0 - K0) - ang(t - best) - qh)
    return np.array([th, qh, qk])


def knee_angle(q):
    """膝内角（大腿和小腿的夹角，度）。"""
    return abs(math.degrees(wrap(ang(F0 - K0) - q[2] - ang(-K0))))


def quat_mat(q):
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, q)
    return m.reshape(3, 3)


def rpy_mat(roll, pitch, yaw=0.0):
    cr, sr, cp, sp, cy, sy_ = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy_, 0], [sy_, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


# ---------------------------------------------------------------- 仿真本体
class Sim:
    def __init__(self, terrain="", payload=5.0):
        xml = open(XML, encoding="utf-8").read().replace("<!--TERRAIN-->", terrain)
        self.m = mujoco.MjModel.from_xml_string(xml)
        cg = self.m.body("cargo").id
        if payload <= 0:
            self.m.body_mass[cg] = 1e-4
            self.m.body_inertia[cg] = 1e-7
        else:
            self.m.body_mass[cg] = payload
            self.m.body_inertia[cg] = payload * np.array([0.2 ** 2 + 0.08 ** 2, 0.3 ** 2 + 0.08 ** 2, 0.3 ** 2 + 0.2 ** 2]) / 12
        self.d = mujoco.MjData(self.m)
        self.mass = float(self.m.body_subtreemass[self.m.body("base").id])
        self.qadr = np.array([self.m.joint(f"{l}_{j}").qposadr[0] for l in LEGS for j in JN])
        self.vadr = np.array([self.m.joint(f"{l}_{j}").dofadr[0] for l in LEGS for j in JN])
        self.foot_site = [self.m.site(f"{l}_foot").id for l in LEGS]
        self.foot_geom = {self.m.geom(f"{l}_foot").id for l in LEGS}
        self.terrain = {i for i in range(self.m.ngeom) if self.m.geom_contype[i] == 1}
        self.kp = np.array([90.0, 90.0, 120.0] * 4)
        self.kd = np.array([2.0, 2.0, 2.5] * 4)
        self.log = dict(t=[], tau=[], w=[], rpy=[], pos=[], P=[], hit=[], margin=[], knee=[])
        self.n_sub = int(round(DT_CTRL / self.m.opt.timestep))
        self.frames = []

    def set_pose(self, p, R, feet_body):
        """把机器人直接摆到某个姿势（初始化用）。"""
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, R.flatten())
        self.d.qpos[0:3] = p
        self.d.qpos[3:7] = q
        for i, l in enumerate(LEGS):
            self.d.qpos[self.qadr[3 * i:3 * i + 3]] = ik(l, feet_body[i])
        self.d.qvel[:] = 0
        mujoco.mj_forward(self.m, self.d)

    def foot_world(self):
        return np.array([self.d.site_xpos[s].copy() for s in self.foot_site])

    def com(self):
        return self.d.subtree_com[self.m.body("base").id].copy()

    def ff_forces(self, stance, feet_w):
        """按重心位置把重量分给着地的脚（静力学，最小范数），返回每只脚的竖直力 N。"""
        idx = [i for i in range(4) if stance[i]]
        F = np.zeros(4)
        if not idx:
            return F
        c = self.com()
        A = np.array([[1.0] * len(idx), [feet_w[i][0] - c[0] for i in idx], [feet_w[i][1] - c[1] for i in idx]])
        b = np.array([self.mass * G, 0.0, 0.0])
        f = np.linalg.lstsq(A, b, rcond=None)[0]
        F[idx] = np.clip(f, 0, None)
        return F

    def step(self, p_des, R_des, feet_w_des, stance, v_feet_w=None):
        """一个控制周期：期望机身姿势 + 每只脚的世界坐标目标 → 关节目标 → PD + 前馈 → 电机扭矩（加上限）→ 仿真 n_sub 步。"""
        d = self.d
        q_des = np.zeros(12)
        for i, l in enumerate(LEGS):
            pb = R_des.T @ (feet_w_des[i] - p_des)
            q_des[3 * i:3 * i + 3] = ik(l, pb)
        q = d.qpos[self.qadr]
        dq = d.qvel[self.vadr]
        tau = self.kp * (q_des - q) - self.kd * dq
        Fz = self.ff_forces(stance, feet_w_des)
        J = np.zeros((3, self.m.nv))
        for i in range(4):
            if Fz[i] > 0:
                mujoco.mj_jacSite(self.m, d, J, None, self.foot_site[i])
                cols = self.vadr[3 * i:3 * i + 3]
                tau[3 * i:3 * i + 3] += J[:, cols].T @ np.array([0, 0, -Fz[i]])
        tau += d.qfrc_bias[self.vadr] * 0.0
        wm = dq * GEAR                                              # 电机转速
        cap = TAU_M * np.clip((W_M - np.abs(wm)) / (W_M * 0.5), 0, 1)   # 转速过半以后扭矩直线降到 0（★）
        tm = np.clip(tau / GEAR, -cap, cap)                         # 电机输出端扭矩
        d.ctrl[:] = tm                                              # actuator gear 已经乘了 1.5
        for _ in range(self.n_sub):
            mujoco.mj_step(self.m, d)
        # ---- 记录
        I = tm / KT
        P = float(np.sum(tm * wm) + np.sum(I ** 2) * R_COIL) + P_ELEC
        hit = 0
        for c in d.contact[:d.ncon]:
            g1, g2 = c.geom1, c.geom2
            if (g1 in self.terrain) != (g2 in self.terrain):
                other = g2 if g1 in self.terrain else g1
                if other not in self.foot_geom:
                    hit = max(hit, 1)
        R = quat_mat(d.qpos[3:7])
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        pitch = math.degrees(math.asin(np.clip(R[2, 0], -1, 1)))           # 正 = 抬头
        L = self.log
        L["t"].append(d.time)
        L["tau"].append(tm * GEAR)
        L["w"].append(wm.copy())
        L["rpy"].append((roll, pitch))
        L["pos"].append(d.qpos[0:3].copy())
        L["P"].append(P)
        L["hit"].append(hit)
        L["knee"].append([knee_angle(q[3 * i:3 * i + 3]) for i in range(4)])
        L["margin"].append(self.margin(stance))

    def margin(self, stance):
        """重心投影离着地脚围成的多边形边多远（mm，负 = 在外面）。两条腿着地（小跑）不算。"""
        idx = [i for i in range(4) if stance[i]]
        if len(idx) < 3:
            return float("nan")
        P = self.foot_world()[idx][:, :2]
        c = self.com()[:2]
        ctr = P.mean(axis=0)
        order = np.argsort([math.atan2(p[1] - ctr[1], p[0] - ctr[0]) for p in P])
        P = P[order]
        dmin = 1e9
        for k in range(len(P)):
            a, b = P[k], P[(k + 1) % len(P)]
            e = b - a
            n = np.array([e[1], -e[0]]) / np.linalg.norm(e)
            dist = -np.dot(c - a, n)
            dmin = min(dmin, dist)
        return dmin * 1000

    def grab(self, renderer, cam):
        renderer.update_scene(self.d, cam)
        self.frames.append(renderer.render().copy())


# ---------------------------------------------------------------- 姿势工具
def stand_feet_body(h=0.25, dx=0.0):
    """站立时每只脚（球心）在机身坐标的位置：脚在髋正下方，髋到脚底 h。"""
    return [np.array([SIDE[l][0] * HX + dx, SIDE[l][1] * Y_LEG, -h + FOOT_R]) for l in LEGS]


def smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


class Recorder:
    """每 1/15 秒拍一帧（--gif 时）。"""
    def __init__(self, sim, on):
        self.on, self.sim, self.next = on, sim, 0.0
        if on:
            self.r = mujoco.Renderer(sim.m, 360, 640)
            self.cam = mujoco.MjvCamera()
            self.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.cam.trackbodyid = sim.m.body("base").id
            self.cam.distance, self.cam.azimuth, self.cam.elevation = 1.6, 120.0, -15.0

    def tick(self):
        if self.on and self.sim.d.time >= self.next:
            self.sim.grab(self.r, self.cam)
            self.next += 1 / 15


# ---------------------------------------------------------------- 场景
def sc_stand(gif):
    s = Sim()
    H = 0.25
    feet_b = stand_feet_body(H)
    p0 = np.array([0, 0, H])
    s.set_pose(p0, np.eye(3), feet_b)
    feet_w = [p0 + f for f in feet_b]
    rec = Recorder(s, gif)
    while s.d.time < 3.0:
        s.step(p0, np.eye(3), feet_w, [1, 1, 1, 1])
        rec.tick()
    return s, dict(desc="站着不动 3 秒（背 5 kg）")


def sc_standup(gif):
    s = Sim()
    z_lie = -GEOM["Z0"] + 0.003                                    # 机身底贴地
    lie = [np.array([SIDE[l][0] * HX - 0.15, SIDE[l][1] * Y_LEG, -z_lie + FOOT_R]) for l in LEGS]
    p0 = np.array([0.0, 0, z_lie])
    s.set_pose(p0, np.eye(3), lie)
    feet_w = [p0 + f for f in lie]
    rec = Recorder(s, gif)
    H = 0.25
    T_UP = 1.5
    while s.d.time < 0.5 + T_UP + 1.0 + T_UP + 0.5:
        t = s.d.time
        if t < 0.5:
            k = 0.0
        elif t < 0.5 + T_UP:
            k = smooth((t - 0.5) / T_UP)
        elif t < 1.5 + T_UP:
            k = 1.0
        else:
            k = 1 - smooth((t - 1.5 - T_UP) / T_UP)
        p = np.array([-0.15 * k, 0, z_lie + (H - z_lie) * k])
        s.step(p, np.eye(3), feet_w, [1, 1, 1, 1])
        rec.tick()
    return s, dict(desc=f"趴着（机身贴地）→ {T_UP} 秒站到髋高 {H * 1000:.0f} → 再趴下（背 5 kg）")


def sc_trot(gif, v_top=0.5, T=0.36, H=0.25, clear=0.07, payload=5.0):
    """对角小跑。脚的落点按 Raibert：髋下面 + 速度 × 着地时间 / 2 + 速度误差修正。"""
    s = Sim(payload=payload)
    feet_b = stand_feet_body(H)
    p_des = np.array([0, 0, H])
    s.set_pose(p_des, np.eye(3), feet_b)
    feet_w = [p_des + f for f in feet_b]
    rec = Recorder(s, gif)
    phase0 = dict(FL=0.0, RR=0.0, FR=0.5, RL=0.5)
    lift = [f.copy() for f in feet_w]
    stance_prev = [1, 1, 1, 1]
    t_acc, t_hold, t_dec = 2.0, 4.0, 2.0
    t0 = 1.0
    t_end = t0 + t_acc + t_hold + t_dec + 1.0
    vx = 0.0
    n_half = math.ceil(((t_end - 0.8) - (t0 - 0.3)) / (T / 2))
    t_stop = t0 - 0.3 + n_half * T / 2                               # 停步正好在一对腿刚落地的时候（不在半空中停）
    while s.d.time < t_end:
        t = s.d.time
        tt = t - t0
        if tt < 0:
            v_cmd = 0.0
        elif tt < t_acc:
            v_cmd = v_top * smooth(tt / t_acc)
        elif tt < t_acc + t_hold:
            v_cmd = v_top
        elif tt < t_acc + t_hold + t_dec:
            v_cmd = v_top * (1 - smooth((tt - t_acc - t_hold) / t_dec))
        else:
            v_cmd = 0.0
        vx = s.d.qvel[0]
        p_des[0] += v_cmd * DT_CTRL
        p_des[0] = s.d.qpos[0] + np.clip(p_des[0] - s.d.qpos[0], -0.03, 0.03)
        p_des[1] = s.d.qpos[1] + np.clip(p_des[1] - s.d.qpos[1], -0.02, 0.02)
        stance = []
        targets = []
        gait_on = tt >= -0.3 and t < t_stop - 1e-4
        for i, l in enumerate(LEGS):
            if not gait_on:
                ph, st = 0.0, 1
            else:
                ph = ((tt + 0.3) / T + phase0[l]) % 1.0
                st = 1 if ph < 0.5 else 0
            if st and not stance_prev[i]:
                fw = s.foot_world()[i]
                feet_w[i] = np.array([fw[0], fw[1], FOOT_R])         # 落地：记下实际落点
            if not st and stance_prev[i]:
                lift[i] = feet_w[i].copy()
            if st:
                targets.append(feet_w[i])
            else:
                u = (ph - 0.5) / 0.5
                hip = np.array([s.d.qpos[0], s.d.qpos[1], 0]) + quat_mat(s.d.qpos[3:7]) @ np.array([SIDE[l][0] * HX, SIDE[l][1] * Y_LEG, 0])
                land = np.array([hip[0] + vx * T / 4 + 0.03 * (vx - v_cmd) + v_cmd * T * (1 - u) / 2, SIDE[l][1] * Y_LEG + p_des[1], FOOT_R])
                k = smooth(u)
                pt = lift[i] + (land - lift[i]) * k
                pt[2] = FOOT_R + clear * math.sin(math.pi * u) - 0.004 * (u > 0.9)
                targets.append(pt)
            stance.append(st)
        stance_prev = stance
        s.step(p_des.copy(), np.eye(3), targets, stance)
        rec.tick()
    return s, dict(desc=f"对角小跑：平地 2 秒加速到 {v_top} m/s → 跑 4 秒 → 2 秒减速停下（{'背 ' + str(payload) + ' kg' if payload else '空载'}，步频 {1 / T:.1f} 步/秒，抬脚 {clear * 100:.0f} cm）", v=v_top)


A_MAX = 0.5                              # 静步挪机身的最大加速度 m/s²（重心“等效”多挪 a·h/g ≈ 1.4 cm）
MARGIN = 0.05                            # 静步时重心离支撑三角形每条边至少 5 cm
M_OK = 0.02                              # 落脚点余量到 20 mm 就算够，再多不如脚放在髋下面
KMIN, KMAX = 40.0, 160.0                 # 膝内角范围（度），同 stair_calc.py
R_KNEE, R_THIGH, R_SHANK = 0.034, 0.032, 0.018   # 膝（大带轮 + 皮带 + 壳，半径约 31.5）、大腿、小腿的“粗细”（m），离台阶要大于这个


def tri_margin(tri, c):
    """点 c 离三角形每条边的最小距离（m，负 = 在外面）。"""
    ctr = tri.mean(axis=0)
    m = 1e9
    for k in range(3):
        a, b = tri[k], tri[(k + 1) % 3]
        e = b - a
        n = np.array([e[1], -e[0]]) / np.linalg.norm(e)
        if np.dot(ctr - a, n) < 0:
            n = -n
        m = min(m, float(np.dot(c - a, n)))
    return m


def reach(k):
    return math.sqrt(L1 ** 2 + LSH ** 2 - 2 * L1 * LSH * math.cos(math.radians(k)))


def seg_dist(p, a, b):
    ab = b - a
    u = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1))
    return float(np.linalg.norm(p - (a + u * ab)))


def prof_dist(p, prof, gnd):
    """侧面 2D 点到地形轮廓的距离（在地面以下为负）。只看附近 0.6 m 的线段。"""
    d = min(seg_dist(p, np.array(prof[k]), np.array(prof[k + 1])) for k in range(len(prof) - 1)
            if min(prof[k][0], prof[k + 1][0]) < p[0] + 0.6 and max(prof[k][0], prof[k + 1][0]) > p[0] - 0.6)
    return -d if p[1] < gnd(p[0]) else d


def leg_check(hip, foot, prof, gnd):
    """侧面 2D：髋、脚（球心）→ 余量（m，负 = 不行）：够不够得着 / 膝角范围、膝 / 大腿 / 小腿离台阶。"""
    t = foot - hip
    d = float(np.linalg.norm(t))
    m = min(d - reach(KMIN), reach(KMAX) - d)
    if m < -0.05:
        return m
    d = float(np.clip(d, abs(L1 - LSH) + 1e-4, L1 + LSH - 1e-4))
    a = math.acos((L1 ** 2 + d ** 2 - LSH ** 2) / (2 * L1 * d))
    for sgn in (1, -1):
        K = hip + L1 * np.array([math.cos(ang(t) + sgn * a), math.sin(ang(t) + sgn * a)])
        if (t[0] * (K - hip)[1] - t[1] * (K - hip)[0]) * CROSS0 > 0:
            break
    m = min(m, prof_dist(K, prof, gnd) - R_KNEE)
    for u in (0.3, 0.55, 0.8):
        m = min(m, prof_dist(hip + (K - hip) * u, prof, gnd) - R_THIGH)
        m = min(m, prof_dist(K + (foot - K) * u, prof, gnd) - R_SHANK)
    return m


class Terrain:
    """侧面看的地形：segs = [(x 起, x 止, 高)]（m）。zones = 能落脚的范围（离踏步边 ≥ 50、离踢面 ≥ 40）；env = 机身高度跟的“包络线”。"""
    def __init__(self, segs, env, zones):
        self.segs, self.env, self.zones = segs, env, zones
        self.prof = []
        for a, b, z in segs:
            self.prof += [(a, z), (b, z)]

    def gnd(self, x):
        for a, b, z in self.segs:
            if a <= x < b:
                return z
        return self.segs[0][2] if x < self.segs[0][0] else self.segs[-1][2]

    def xml(self, w=1.2):
        return "\n    ".join(f'<geom class="terrain" type="box" size="{(b - a) / 2:.4f} {w / 2} {z / 2:.4f}" pos="{(a + b) / 2:.4f} 0 {z / 2:.4f}"/>'
                               for a, b, z in self.segs if z > 0)

    def mirror(self):
        return Terrain([(-b, -a, z) for a, b, z in reversed(self.segs)], lambda x: self.env(-x), [(-b, -a) for a, b in reversed(self.zones)])


def stairs(h, t, n, x0=0.45):
    """往 +x 上的楼梯：x0 前是地面，n 级，最后一级后面是平台。"""
    segs = [(-10.0, x0, 0.0)] + [(x0 + k * t, x0 + (k + 1) * t if k < n - 1 else 10.0, (k + 1) * h) for k in range(n)]
    zones = [(-10.0, x0 - 0.04)] + [(x0 + k * t + 0.05, x0 + (k + 1) * t - 0.04) for k in range(n - 1)] + [(x0 + (n - 1) * t + 0.05, 10.0)]
    env = lambda x: float(np.clip((x - x0) / t * h + h, 0.0, n * h))   # 踏步边连成的斜线（楼梯前一级就开始抬）
    return Terrain(segs, env, zones)


def plan_crawl(T, feet0, x_start, x_goal, com_fn, H, D, direction, phase, order, clear=0.05, max_steps=400):
    """静步规划（纯几何，不跑物理）。返回 [(腿序号, 机身位置, 俯仰, 新落脚点, 抬脚高度, 余量, 腿余量, 稳定余量)]。
    节奏：按 order 一次跨一条腿，每跨一条机身往前 D/4（匀速，不来回晃）。
    机身高度、俯仰跟 stair_calc.py：低的那头的髋在包络线上 H，另一头也想 H，但机身最多斜 20°。
    落脚点：每条腿单独做动态规划 —— 把整段楼梯所有“跨的时刻 × 候选落脚点（每 2 cm）”都算一遍，
      挑一串落脚点，让整段路上（站着的时候 + 跨的时候）膝 / 大腿 / 小腿离台阶、膝角范围的最小余量最大；一样好就挑离髋下面近的。
    重心：跨之前机身左右挪，让重心（按这个姿势真算，腿很重）离另外三只脚的三角形每条边 ≥ MARGIN。"""
    TH = math.radians(20)
    dx = direction * D / 4
    CAP = 0.03

    def body_pose(x):
        """低的那头的髋（往上走时是后髋，往下 / 倒着上是前髋）在包络线上 H；另一头最多仰 / 俯 20°。"""
        zr = T.env(x - HX) + H
        zf = T.env(x + HX) + H
        th = float(np.clip(math.atan2(zf - zr, 2 * HX), -TH, TH))
        return (zr + HX * math.tan(th), th) if zr <= zf else (zf - HX * math.tan(th), th)

    def hip2d(leg, x):
        z, th = body_pose(x)
        h = np.array([x, 0, z]) + rpy_mat(0, -th) @ np.array([SIDE[leg][0] * HX, 0, 0])
        return np.array([h[0], h[2]])

    # 1 节奏：第 k 步跨 order[k % 4]，跨的时候机身在 xs[k]
    xs = []
    while len(xs) < max_steps:
        x = x_start + direction * phase + (len(xs) + 1) * dx
        if direction * (x_goal - x) <= 0:
            break
        xs.append(x)
    K = len(xs)
    feet_x = {}
    leg_margin = {}
    for leg in order:
        i = LEGS.index(leg)
        js = [k for k in range(K) if order[k % 4] == leg]
        if not js:
            feet_x[i] = []
            continue
        # 每次跨完以后站到下一次抬起（或者走完）
        ends = js[1:] + [K - 1]

        def cands(m):
            mid = hip2d(leg, (xs[js[m]] + xs[ends[m]]) / 2)[0]
            out = []
            for a, b in T.zones:
                out += [x for x in np.arange(max(a, mid - 0.24), min(b, mid + 0.24) + 1e-9, 0.02)]
            return out, mid

        def stance(m, f):
            fp = np.array([f, T.gnd(f) + FOOT_R])
            return min(leg_check(hip2d(leg, xs[k]), fp, T.prof, T.gnd) for k in range(js[m], ends[m] + 1))

        def swing(m, f0, f1):
            ss = np.linspace(min(f0, f1) - 0.03, max(f0, f1) + 0.03, 20)
            zt = max(T.gnd(x) for x in ss) + FOOT_R + clear
            hp = hip2d(leg, xs[js[m]])
            return min(leg_check(hp, np.array([f0, zt]), T.prof, T.gnd), leg_check(hp, np.array([f1, zt]), T.prof, T.gnd))
        f_init = feet0[i][0]
        m0 = min(leg_check(hip2d(leg, xs[k]), np.array([f_init, T.gnd(f_init) + FOOT_R]), T.prof, T.gnd) for k in range(0, js[0] + 1))
        prev = {f_init: ((min(m0, CAP), 0.0), None)}                    # 落脚点 → ((余量, -偏离), 上一个)
        hist = []
        for m in range(len(js)):
            cs, mid = cands(m)
            cur = {}
            for f in cs:
                sm = stance(m, f)
                best = None
                for f0, (val, _) in prev.items():
                    if abs(f - f0) > 0.36 or min(val[0], sm) < (best[0][0] if best else -1e9):
                        continue
                    v = (min(val[0], sm, swing(m, f0, f), CAP), val[1] - abs(f - mid))
                    if best is None or v > best[0]:
                        best = (v, f0)
                if best is None:                                        # 都太远：随便接一个最好的（会被记成很差）
                    f0, (val, _) = max(prev.items(), key=lambda kv: kv[1][0])
                    best = ((min(val[0], sm, -0.1), val[1]), f0)
                cur[f] = best
            hist.append(cur)
            prev = cur
        f = max(prev, key=lambda kk: prev[kk][0])
        leg_margin[i] = prev[f][0][0]
        seq = []
        for m in range(len(js) - 1, -1, -1):
            seq.append(f)
            f = hist[m][f][1]
        feet_x[i] = list(reversed(seq))
    # 2 按规划的落脚点走一遍：算每一步的机身位置（左右挪到稳）、抬脚高度、余量
    feet = [f.copy() for f in feet0]
    cnt = {i: 0 for i in range(4)}
    out = []
    y_prev = 0.0
    for k in range(K):
        leg = order[k % 4]
        i = LEGS.index(leg)
        xb = xs[k]
        sup = [j for j in range(4) if j != i]
        tri = np.array([feet[j][:2] for j in sup])
        old = feet[i]
        lifted = [f.copy() for f in feet]
        lifted[i] = old + np.array([0, 0, 0.06])
        ys = np.arange(-0.09, 0.0901, 0.005)                           # 机身只左右挪：挑够稳（≥ MARGIN）且离上一步最近的
        mg = np.array([tri_margin(tri, np.array([xb, y])) for y in ys])
        okk = mg >= MARGIN
        y = ys[okk][np.argmin(np.abs(ys[okk] - y_prev))] if okk.any() else ys[np.argmax(mg)]
        want = np.array([xb, y])
        stab = float(tri_margin(tri, want))
        y_prev = y
        bxy = want.copy()
        for _ in range(3):                                              # 机身放哪，重心（抬着这条腿）才正好在 want
            z, th = body_pose(bxy[0])
            bxy += want - com_fn(np.array([bxy[0], bxy[1], z]), th, lifted)
        z, th = body_pose(bxy[0])
        xc = feet_x[i][cnt[i]]
        cnt[i] += 1
        ss = np.linspace(min(old[0], xc) - 0.03, max(old[0], xc) + 0.03, 20)
        z_top = max(T.gnd(x) for x in ss) + FOOT_R + clear
        new = np.array([xc, old[1], T.gnd(xc) + FOOT_R])
        lm = leg_margin[i]
        out.append((i, np.array([bxy[0], bxy[1], z]), th, new, z_top, min(lm, stab - MARGIN + M_OK), lm, stab))
        feet[i] = new
    return out


def crawl(s, T, x_goal, rec, H=0.25, D=0.13, direction=1, phases=None, t_shift=0.5, t_swing=0.7, clear=0.05, dry=False, gait=None):
    """静步：先规划（试几个起步相位，挑整段余量最大的），再在物理仿真里一步一步做。"""
    feet0 = [f.copy() for f in s.foot_world()]
    for f in feet0:
        f[2] = T.gnd(f[0]) + FOOT_R
    x0 = s.d.qpos[0]
    dd = mujoco.MjData(s.m)

    def com_fn(p, th, feet):
        """机身在 p、俯仰 th、四只脚在 feet（世界坐标）时整机重心的 xy。"""
        R = rpy_mat(0, -th)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, R.flatten())
        dd.qpos[0:3], dd.qpos[3:7] = p, q
        for k, l in enumerate(LEGS):
            dd.qpos[s.qadr[3 * k:3 * k + 3]] = ik(l, R.T @ (feet[k] - p))
        mujoco.mj_kinematics(s.m, dd)
        mujoco.mj_comPos(s.m, dd)
        return dd.subtree_com[s.m.body("base").id][:2].copy()
    gaits = [["RL", "FL", "RR", "FR"], ["RL", "RR", "FL", "FR"]]       # 普通静步（左后 左前 右后 右前）/ 一对一对跨（后面一对 → 前面一对）
    if direction < 0:
        gaits = [["FL", "RL", "FR", "RR"], ["FL", "FR", "RL", "RR"]]
    gaits = gaits[gait:gait + 1] if gait is not None else gaits
    best = None
    for order in gaits:
        for ph in (phases if phases is not None else np.arange(0, D / 2 + 1e-9, 0.02)):
            plan = plan_crawl(T, feet0, x0, x_goal, com_fn, H, D, direction, ph, order, clear)
            m = min(p[5] for p in plan)
            if best is None or m > best[0] + 1e-4:
                best = (m, ph, plan, order)
    m_plan, ph, plan, order = best
    s.gait = order
    if dry:
        return best
    feet_w = [f.copy() for f in feet0]
    p = s.d.qpos[0:3].copy()
    pitch = 0.0

    def run(T_, fn):
        t0 = s.d.time
        while s.d.time - t0 < T_:
            fn(min((s.d.time - t0) / T_, 1.0))
            rec.tick()

    for i, p1, th_new, new, z_top, m, m_leg, stab in plan:
        p0, th0 = p.copy(), pitch
        Tsh = max(t_shift, math.sqrt(6 * np.linalg.norm(p1 - p0) / A_MAX))   # 挪得远就挪慢点（加速度 ≤ A_MAX，不然惯性把重心甩出去）

        def shift(u):
            nonlocal p, pitch
            kk = smooth(u)
            p = p0 + (p1 - p0) * kk
            pitch = th0 + (th_new - th0) * kk
            s.step(p, rpy_mat(0, -pitch), feet_w, [1, 1, 1, 1])
        run(Tsh, shift)
        old = feet_w[i].copy()
        R = rpy_mat(0, -pitch)
        st = [1, 1, 1, 1]
        st[i] = 0

        def swing(u):
            if u < 0.3:
                q = old.copy()
                q[2] = old[2] + (z_top - old[2]) * smooth(u / 0.3)
            elif u < 0.7:
                q = old + (new - old) * smooth((u - 0.3) / 0.4)
                q[2] = z_top
            else:
                q = new.copy()
                q[2] = z_top + (new[2] - 0.004 - z_top) * smooth((u - 0.7) / 0.3)
            tg = list(feet_w)
            tg[i] = q
            s.step(p, R, tg, st)
        run(t_swing, swing)
        feet_w[i] = new
    t0 = s.d.time                                                     # 收尾：四只脚着地站 1 秒
    while s.d.time - t0 < 1.0:
        s.step(p, rpy_mat(0, -pitch), feet_w, [1, 1, 1, 1])
        rec.tick()
    s.plan = plan
    return len(plan), m_plan, ph


def sc_stairs(gif, h=0.175, t=0.26, n=4, mode="up", name="住宅"):
    """mode：up = 正着上，back = 倒着上（屁股朝楼梯），down = 正着下。"""
    T = stairs(h, t, n)
    top = n * h
    x_end = 0.45 + (n - 1) * t + 0.45
    import stair_calc as SC
    H = SC.climb(SC_L1, SC_LS, h * 1000, t * 1000)[2] / 1000          # 后髋离踏步边斜线的高度（stair_calc 算的最好值）
    if mode == "up":
        x_start, x_goal, direction, z_start = 0.0, x_end, 1, 0.0
    else:
        T = T.mirror()                                                # 楼梯改成往 -x 上
        if mode == "back":
            x_start, x_goal, direction, z_start = 0.0, -x_end, -1, 0.0
        else:
            x_start, x_goal, direction, z_start = -x_end, 0.0, 1, top
    s = Sim(terrain=T.xml())
    feet_b = stand_feet_body(H)
    p0 = np.array([x_start, 0, z_start + H])
    s.set_pose(p0, np.eye(3), feet_b)
    rec = Recorder(s, gif)
    if gif:
        rec.cam.azimuth, rec.cam.distance = 90.0, 2.0
    feet_w = [p0 + f for f in feet_b]
    while s.d.time < 0.5:
        s.step(p0, np.eye(3), feet_w, [1, 1, 1, 1])
        rec.tick()
    steps, m_plan, ph = crawl(s, T, x_goal, rec, H=H, D=t / 2, direction=direction)
    z_goal = top if mode in ("up", "back") else 0.0
    ok = direction * (s.d.qpos[0] - x_goal) > -0.2 and abs(s.d.qpos[2] - H - z_goal) < 0.08
    how = dict(up="正着往上", back="倒着往上（屁股朝楼梯，膝朝后的腿这样更顺）", down="正着往下")[mode]
    return s, dict(desc=f"{name}楼梯 {h * 1000:.0f} 高 × {t * 1000:.0f} 宽 × {n} 级，{how}，静步（背 5 kg）", steps=steps, ok=bool(ok),
                   plan_margin=round(m_plan * 1000), climbed=f"走了 {abs(s.d.qpos[0] - x_start):.2f} m，机身高度变了 {(s.d.qpos[2] - p0[2]) * 1000:+.0f} mm（楼梯总高 {top * 1000:.0f}）")


def sc_cable(gif):
    """地上一道 60 高、200 宽的电缆槽，静步跨过去。"""
    x0, w, hh = 0.45, 0.20, 0.06
    gnd0 = lambda x: hh if x0 <= x < x0 + w else 0.0
    env = lambda x: float(max(gnd0(x + d) for d in np.linspace(-0.12, 0.12, 13)))   # 槽前后 12 cm 就开始抬高机身
    T = Terrain([(-10.0, x0, 0.0), (x0, x0 + w, hh), (x0 + w, 10.0, 0.0)], env, [(-10, x0 - 0.04), (x0 + 0.04, x0 + w - 0.04), (x0 + w + 0.04, 10)])
    s = Sim(terrain=T.xml())
    H = 0.25
    feet_b = stand_feet_body(H)
    p0 = np.array([0, 0, H])
    s.set_pose(p0, np.eye(3), feet_b)
    rec = Recorder(s, gif)
    if gif:
        rec.cam.azimuth = 90.0
    steps, m_plan, ph = crawl(s, T, x0 + w + 0.5, rec, H=H, D=0.16)
    ok = s.d.qpos[0] > x0 + w + 0.4
    return s, dict(desc="地上 60 高 × 200 宽的电缆槽，静步跨过去（背 5 kg）", steps=steps, ok=bool(ok), plan_margin=round(m_plan * 1000))


SCENES = dict(
    standup=sc_standup,
    stand=sc_stand,
    trot=lambda g: sc_trot(g, 0.5),
    trot_fast=lambda g: sc_trot(g, 0.8, T=0.32),
    stairs_up=lambda g: sc_stairs(g, 0.175, 0.26, 4, "up", "住宅"),
    stairs_back=lambda g: sc_stairs(g, 0.175, 0.26, 4, "back", "住宅"),
    stairs_down=lambda g: sc_stairs(g, 0.175, 0.26, 4, "down", "住宅"),
    stairs_pub=lambda g: sc_stairs(g, 0.15, 0.30, 4, "up", "公共"),
    walk_cable=sc_cable,
)
NAMES = dict(standup="趴下站起", stand="站立", trot="小跑 0.5 m/s", trot_fast="小跑 0.8 m/s", stairs_up="住宅楼梯·正着上", stairs_back="住宅楼梯·倒着上",
             stairs_down="住宅楼梯·正着下", stairs_pub="公共楼梯·正着上", walk_cable="跨电缆槽")


# ---------------------------------------------------------------- 统计、出图
def summarize(s, info):
    L = s.log
    tau = np.array(L["tau"])
    w = np.array(L["w"])
    rpy = np.array(L["rpy"])
    t = np.array(L["t"])
    P = np.array(L["P"])
    pk = np.abs(tau).max(axis=0)
    rms = np.sqrt((tau ** 2).mean(axis=0))
    res = dict(info)
    res["peak"] = {j: float(max(pk[k::3])) for k, j in enumerate(JN)}
    res["rms"] = {j: float(max(rms[k::3])) for k, j in enumerate(JN)}
    res["limit"] = {j: float(LIMIT_J[k]) for k, j in enumerate(JN)}
    wm = np.abs(w).max(axis=0)
    res["motor_speed"] = float(wm.max())
    res["sat"] = float((np.abs(tau) >= LIMIT_J * 0.98).any(axis=1).mean() * 100)
    res["roll"] = float(np.abs(rpy[:, 0]).max())
    res["pitch"] = float(np.abs(rpy[:, 1]).max())
    res["hit"] = int(np.sum(L["hit"]))
    mg = np.array(L["margin"])
    res["margin_min"] = float(np.nanmin(mg)) if np.isfinite(mg).any() else None
    kn = np.array(L["knee"])
    res["knee_min"], res["knee_max"] = float(kn.min()), float(kn.max())
    res["P_avg"] = float(P.mean())
    res["P_peak"] = float(P.max())
    res["hours"] = BAT_WH / res["P_avg"]
    pos = np.array(L["pos"])
    res["fell"] = bool(pos[:, 2].min() < 0.12 and info.get("desc", "").find("趴") < 0)
    res["dist"] = float(pos[-1, 0] - pos[0, 0])
    res["T"] = float(t[-1])
    if "v" in info:
        sel = (t > 4.0) & (t < 7.0)
        res["v_meas"] = float(np.mean(np.array(L["pos"])[sel][-1, 0] - np.array(L["pos"])[sel][0, 0]) / (t[sel][-1] - t[sel][0]))
    return res


def plot(name, s, res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
    plt.rcParams["axes.unicode_minus"] = False
    L = s.log
    t = np.array(L["t"])
    tau = np.array(L["tau"])
    fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    cols = dict(FL="#1f77b4", FR="#ff7f0e", RL="#2ca02c", RR="#d62728")
    for k, j in enumerate(JN):
        ax = axs[k]
        for i, l in enumerate(LEGS):
            ax.plot(t, tau[:, 3 * i + k], lw=0.8, color=cols[l], label=l)
        lim = LIMIT_J[k]
        ax.axhline(lim, color="k", ls="--", lw=0.8)
        ax.axhline(-lim, color="k", ls="--", lw=0.8)
        ax.set_ylabel(f"{dict(haa='外摆', hfe='髋', kfe='膝')[j]} N·m")
        ax.text(t[0], lim * 0.92, f"电机上限 {lim:.1f}", fontsize=8, va="top")
    axs[0].legend(ncol=4, fontsize=8, loc="lower right")
    rpy = np.array(L["rpy"])
    axs[3].plot(t, rpy[:, 0], label="横滚°", lw=0.8)
    axs[3].plot(t, rpy[:, 1], label="俯仰°", lw=0.8)
    axs[3].plot(t, np.array(L["P"]) / 10, label="功率 ÷10 W", lw=0.8, color="gray")
    axs[3].legend(fontsize=8, ncol=3)
    axs[3].set_xlabel("时间 s")
    fig.suptitle(f"007A 仿真：{NAMES[name]} —— {res['desc']}", fontsize=11)
    plt.tight_layout()
    plt.savefig(os.path.join(DOC, f"仿真_{fname(name)}.png"), dpi=90)
    plt.close(fig)


def fname(name):
    return NAMES[name].replace(" ", "").replace("/", "每").replace("·", "_")


def save_gif(name, s):
    """动图：最多 150 帧，太长就快放（标题上写了几倍速）。"""
    if not s.frames:
        return
    from PIL import Image, ImageDraw
    step = max(1, math.ceil(len(s.frames) / 150))
    ims = []
    for f in s.frames[::step]:
        im = Image.fromarray(f)
        ImageDraw.Draw(im).text((8, 6), f"007A {name}  x{step / (15 * 0.08):.0f}" if step > 1 else f"007A {name}", fill=(20, 20, 20))
        ims.append(im.convert("P", palette=Image.ADAPTIVE, colors=96))
    ims[0].save(os.path.join(DOC, f"仿真_{fname(name)}.gif"), save_all=True, append_images=ims[1:], duration=80, loop=0)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    gif = "--gif" in sys.argv
    names = args or list(SCENES)
    results = {}
    rp = os.path.join(OUT, "sim_result.json")
    if os.path.exists(rp) and args:
        results = json.load(open(rp, encoding="utf-8"))
    for n in names:
        t0 = time.time()
        s, info = SCENES[n](gif)
        r = summarize(s, info)
        results[n] = r
        plot(n, s, r)
        save_gif(n, s)
        pk = r["peak"]
        print(f"[{NAMES[n]}] {time.time() - t0:.0f}s  峰值 外摆 {pk['haa']:.1f}/23.7  髋 {pk['hfe']:.1f}/23.7  膝 {pk['kfe']:.1f}/35.5  "
              f"电机最高 {r['motor_speed']:.1f}/{W_M:.1f} rad/s  顶满 {r['sat']:.1f}%  横滚 {r['roll']:.1f}° 俯仰 {r['pitch']:.1f}°  "
              f"碰 {r['hit']}  稳定裕度 {r['margin_min']}  膝角 {r['knee_min']:.0f}–{r['knee_max']:.0f}°  功率 {r['P_avg']:.0f} W  走了 {r['dist']:.2f} m  "
              f"{'ok=' + str(r.get('ok')) if 'ok' in r else ''} {r.get('climbed', '')} {'实测 %.2f m/s' % r['v_meas'] if 'v_meas' in r else ''}", flush=True)
    json.dump(results, open(rp, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
