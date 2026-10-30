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
TAU_M, W_M = float(os.environ.get("MOTOR_TAU", GEOM["tau"])), GEOM["w"] * V_BAT / 24.0   # 环境变量 MOTOR_TAU：试别的电机峰值扭矩
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
    def __init__(self, terrain="", payload=float(os.environ.get("PAYLOAD", 5.0))):   # 背的货（kg）；环境变量 PAYLOAD 可改
        xml = open(XML, encoding="utf-8").read().replace("<!--TERRAIN-->", terrain)
        self.m = mujoco.MjModel.from_xml_string(xml)
        cg = self.m.body("cargo").id
        if payload <= 0:
            self.m.body_mass[cg] = 0.02                                 # 空载：货台只剩一块很轻的板（太小的质量 / 惯量仿真会抖）
            self.m.body_inertia[cg] = 0.02 * np.array([0.2 ** 2 + 0.08 ** 2, 0.3 ** 2 + 0.08 ** 2, 0.3 ** 2 + 0.2 ** 2]) / 12
        else:
            self.m.body_mass[cg] = payload
            self.m.body_inertia[cg] = payload * np.array([0.2 ** 2 + 0.08 ** 2, 0.3 ** 2 + 0.08 ** 2, 0.3 ** 2 + 0.2 ** 2]) / 12
        self.d = mujoco.MjData(self.m)
        mujoco.mj_setConst(self.m, self.d)                               # 改了货的质量：整机总质量（subtreemass）要重算，不然前馈还按背 5 kg 出力
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
        p_now, R_now = d.qpos[0:3].copy(), quat_mat(d.qpos[3:7])
        for i, l in enumerate(LEGS):
            if stance[i]:
                pb = R_des.T @ (feet_w_des[i] - p_des)              # 撑着的腿：按期望机身姿势算（机身姿势就靠它们撑出来）
            else:
                pb = R_now.T @ (feet_w_des[i] - p_now)              # 跨的腿：按机身实际姿势算，脚才落到世界坐标里规划的那个点
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


def sc_trot(gif, v_top=0.5, T=0.36, H=0.25, clear=0.07, payload=float(os.environ.get("PAYLOAD", 5.0))):
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
KMIN, KMAX = 40.0, 165.0                 # 膝内角范围（度）：stair_calc.py 用 160，这里放到 165（离伸直还有 15°，宇树 Go1 也这样用）
GAP = 0.005                              # 腿的真实外形（髋盘 + 大腿 + 膝圆头 + 小腿）离台阶至少 5 mm
FOOT_SKIP = 0.030                        # 脚球心 30 mm 以内的小腿不算（那是脚，本来就踩在地上）


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


def _outline():
    """model.py / eng_leg.py 的真实腿外形（侧面），站立姿势下取轮廓上的点（每 4 mm 一个），存成两个“局部坐标”：
    大腿：u 沿 髋 → 膝、v 垂直；小腿：u 沿 膝 → 脚球心、v 垂直（m）。"""
    sys.path.insert(0, ROOT)
    import model as M
    import eng_leg as E
    M.RK, M.HEX_A, M.HUB_CUT = E.RK_NEW, E.HEX_A_NEW, E.HEX_A_NEW + 3.0
    thigh, shank, knee, foot = M.leg_2d()

    def dense(poly):
        pts = []
        for k in range(len(poly)):
            a_, b_ = np.array(poly[k]) / 1000, np.array(poly[(k + 1) % len(poly)]) / 1000
            n = max(int(np.linalg.norm(b_ - a_) / 0.004), 1)
            pts += [a_ + (b_ - a_) * t for t in np.arange(n) / n]
        return np.array(pts)

    def local(pts, o, d):
        u = d / np.linalg.norm(d)
        v = np.array([-u[1], u[0]])
        return np.stack([(pts - o) @ u, (pts - o) @ v], axis=1)
    th = local(dense(thigh), np.zeros(2), K0)
    sh = local(dense(shank), K0, F0 - K0)
    sh = sh[np.linalg.norm(K0 + sh[:, :1] * (F0 - K0) / np.linalg.norm(F0 - K0)
                           + sh[:, 1:] * np.array([-(F0 - K0)[1], (F0 - K0)[0]]) / np.linalg.norm(F0 - K0) - F0, axis=1) > FOOT_SKIP]
    return th, sh


TH_PTS, SH_PTS = _outline()


def knee_of(hip, foot):
    """侧面 2D：髋、脚球心 → 膝（膝朝后那个解）。"""
    t = foot - hip
    d = float(np.clip(np.linalg.norm(t), abs(L1 - LSH) + 1e-4, L1 + LSH - 1e-4))
    a = math.acos((L1 ** 2 + d ** 2 - LSH ** 2) / (2 * L1 * d))
    for sgn in (1, -1):
        K = hip + L1 * np.array([math.cos(ang(t) + sgn * a), math.sin(ang(t) + sgn * a)])
        if (t[0] * (K - hip)[1] - t[1] * (K - hip)[0]) * CROSS0 > 0:
            break
    return K


def leg_points(hip, foot):
    """这个姿势下腿外形上的所有点（侧面 2D，世界坐标）。"""
    K = knee_of(hip, foot)
    out = []
    for pts, o, d in ((TH_PTS, hip, K - hip), (SH_PTS, K, foot - K)):
        u = d / np.linalg.norm(d)
        v = np.array([-u[1], u[0]])
        out.append(o + pts[:, :1] * u + pts[:, 1:] * v)
    return np.vstack(out)


def leg_check(hip, foot, T):
    """侧面 2D：髋、脚（球心）→ 余量（m，负 = 不行）：够不够得着 / 膝角范围；真实腿外形离地形 ≥ GAP。"""
    t = foot - hip
    d = float(np.linalg.norm(t))
    m = min(d - reach(KMIN), reach(KMAX) - d)
    if m < -0.05:
        return m
    return min(m, float(T.sdist(leg_points(hip, foot)).min()) - GAP)


class Terrain:
    """侧面看的地形：segs = [(x 起, x 止, 高)]（m）。zones = 能落脚的范围（离踏步边 ≥ 50、离踢面 ≥ 40）；env = 机身高度跟的“包络线”。"""
    def __init__(self, segs, env, zones):
        self.segs, self.env, self.zones = segs, env, zones
        self.prof = []
        for a, b, z in segs:
            self.prof += [(max(a, -3.0), z), (min(b, 3.0), z)]
        P = np.array(self.prof)
        self._A, self._B = P[:-1], P[1:]
        self._x0 = np.array([a for a, b, z in segs])
        self._z = np.array([z for a, b, z in segs])

    def sdist(self, P):
        """一堆侧面点（N×2）到地形轮廓的距离（在地面以下为负）。"""
        A, B = self._A, self._B
        AB = B - A
        u = np.clip(((P[:, None, :] - A[None]) * AB[None]).sum(-1) / np.maximum((AB * AB).sum(-1), 1e-12)[None], 0, 1)
        d = np.linalg.norm(P[:, None, :] - (A[None] + u[..., None] * AB[None]), axis=-1).min(1)
        return np.where(P[:, 1] < self.gnd_v(P[:, 0]), -d, d)

    def gnd_v(self, x):
        k = np.clip(np.searchsorted(self._x0, x, side="right") - 1, 0, len(self.segs) - 1)
        return self._z[k]

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


def body_pose_fn(T, H):
    """机身高度、俯仰跟 stair_calc.py：低的那头的髋在包络线上 H，另一头也想 H，但机身最多斜 20°。"""
    TH = math.radians(20)

    def body_pose(x):
        zr = T.env(x - HX) + H
        zf = T.env(x + HX) + H
        th = float(np.clip(math.atan2(zf - zr, 2 * HX), -TH, TH))
        return (zr + HX * math.tan(th), th) if zr <= zf else (zf - HX * math.tan(th), th)
    return body_pose


def hip_fn(body_pose):
    def hip2d(leg, x):
        z, th = body_pose(x)
        h = np.array([x, 0, z]) + rpy_mat(0, -th) @ np.array([SIDE[leg][0] * HX, 0, 0])
        return np.array([h[0], h[2]])
    return hip2d


def leg_dp(T, leg, f_init, x_start, swings, x_end, hip2d, clear, CAP=0.03):
    """一条腿的落脚点（动态规划）：这条腿在机身走到 swings[m] 时跨，跨完一直撑到下一次跨（机身从 swings[m] 走到 swings[m+1]）。
    把每次跨的“候选落脚点（每 2 cm）”都算一遍，挑一串让整段路上（撑着 + 跨的时候）真实腿外形离台阶、膝角范围的最小余量最大的；
    一样好（都 ≥ CAP）就挑离髋下面近的。返回 (落脚点列表, 最小余量)。"""
    def xs_between(a, b):
        n = max(int(abs(b - a) / 0.015), 1)
        return [a + (b - a) * u for u in np.linspace(0, 1, n + 1)]

    def stance(m, f):
        fp = np.array([f, T.gnd(f) + FOOT_R])
        b = swings[m + 1] if m + 1 < len(swings) else x_end
        return min(leg_check(hip2d(leg, x), fp, T) for x in xs_between(swings[m], b))

    def swing(m, f0, f1):
        ss = np.linspace(min(f0, f1) - 0.03, max(f0, f1) + 0.03, 20)
        zt = max(T.gnd(x) for x in ss) + FOOT_R + clear
        hp = hip2d(leg, swings[m])
        return min(leg_check(hp, np.array([f0, zt]), T), leg_check(hp, np.array([f1, zt]), T))

    if not swings:
        fp = np.array([f_init, T.gnd(f_init) + FOOT_R])
        return [], min(leg_check(hip2d(leg, x), fp, T) for x in xs_between(x_start, x_end))
    m0 = min(leg_check(hip2d(leg, x), np.array([f_init, T.gnd(f_init) + FOOT_R]), T) for x in xs_between(x_start, swings[0]))
    prev = {f_init: ((min(m0, CAP), 0.0), None)}
    hist = []
    for m in range(len(swings)):
        b = swings[m + 1] if m + 1 < len(swings) else x_end
        mid = hip2d(leg, (swings[m] + b) / 2)[0]
        cs = []
        for a_, b_ in T.zones:
            cs += [x for x in np.arange(max(a_, mid - 0.24), min(b_, mid + 0.24) + 1e-9, 0.02)]
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
            if best is None:
                f0, (val, _) = max(prev.items(), key=lambda kv: kv[1][0])
                best = ((min(val[0], sm, -0.1), val[1]), f0)
            cur[f] = best
        hist.append(cur)
        prev = cur
    f = max(prev, key=lambda kk: prev[kk][0])
    margin = prev[f][0][0]
    seq = []
    for m in range(len(swings) - 1, -1, -1):
        seq.append(f)
        f = hist[m][f][1]
    return list(reversed(seq)), margin


def plan_crawl(T, feet0, x_start, x_goal, com_fn, H, D, direction, ph_r, ph_f, clear=0.05, dp_cache=None):
    """静步规划（纯几何，不跑物理）。返回 [(腿序号, 机身位置, 俯仰, 新落脚点, 抬脚高度, 余量, 腿余量, 稳定余量)]。
    节奏：机身每走 D，后面两条腿在 x_start + ph_r + kD 处（机身停住）先左后右各跨一次，前面两条腿在 x_start + ph_f + kD 处各跨一次。
      ph_r、ph_f 分开挑 —— 前腿、后腿在楼梯上最舒服的跨步时机不一样（stair_calc.py 也是这样分开算的）。
    落脚点：每条腿单独动态规划（leg_dp）。
    重心：跨之前机身左右挪（不够稳再前后挪 ±6 cm），让重心（按这个姿势真算，腿很重）离另外三只脚的三角形每条边 ≥ MARGIN。
    左右挪最多 ±12 cm、前后最多 ±10 cm；同一处跨的一对腿先跨哪条，挑两条都稳的顺序。"""
    body_pose = body_pose_fn(T, H)
    hip2d = hip_fn(body_pose)
    dp_cache = {} if dp_cache is None else dp_cache

    def swings_of(ph):
        out, k = [], 0
        while True:
            x = x_start + direction * (ph + k * D)
            if direction * (x_goal - x) <= 0 or k > 200:
                return out
            out.append(x)
            k += 1
    plan_leg = {}
    for leg in LEGS:
        ph = ph_f if leg[0] == "F" else ph_r
        i = LEGS.index(leg)
        key = (leg[0], round(ph, 4), round(H, 4), round(clear, 4), round(D, 4))   # 左右两条腿在侧面一样，算一次
        if key not in dp_cache:
            sw = swings_of(ph)
            dp_cache[key] = (sw,) + leg_dp(T, leg, feet0[i][0], x_start, sw, x_goal, hip2d, clear)
        plan_leg[leg] = dp_cache[key]
    events = []
    for leg in LEGS:
        sw, fs, _ = plan_leg[leg]
        for m, (x, f) in enumerate(zip(sw, fs)):
            events.append((direction * x, 0 if leg[1] == "L" else 1, LEGS.index(leg), x, f))
    events.sort()
    # 同一处跨的一对腿（左右），先跨哪条：挑两条都跨得稳的那个顺序（只看支撑三角形，机身左右挪 ±12 cm）
    feet_t = [f.copy() for f in feet0]

    def quick_stab(i, xb, ft):
        tri = np.array([ft[j][:2] for j in range(4) if j != i])
        return max(tri_margin(tri, np.array([xb + ddx, y])) for ddx in (-0.08, -0.04, 0.0, 0.04, 0.08) for y in np.arange(-0.12, 0.1201, 0.02))
    k = 0
    while k < len(events):
        if k + 1 < len(events) and abs(events[k][3] - events[k + 1][3]) < 1e-9:
            best_o = None
            for a, b in ((k, k + 1), (k + 1, k)):
                ft = [f.copy() for f in feet_t]
                m = 1e9
                for e in (events[a], events[b]):
                    m = min(m, quick_stab(e[2], e[3], ft))
                    ft[e[2]] = np.array([e[4], ft[e[2]][1], 0.0])
                if best_o is None or m > best_o[0]:
                    best_o = (m, a, b)
            events[k], events[k + 1] = events[best_o[1]], events[best_o[2]]
            for e in events[k:k + 2]:
                feet_t[e[2]] = np.array([e[4], feet_t[e[2]][1], 0.0])
            k += 2
        else:
            feet_t[events[k][2]] = np.array([events[k][4], feet_t[events[k][2]][1], 0.0])
            k += 1

    def legs_ok(x, i, feet):
        return min(leg_check(hip2d(LEGS[j], x), np.array([feet[j][0], feet[j][2]]), T) for j in range(4) if j != i)

    def hip_at(leg, x, z, th):
        h = np.array([x, 0, z]) + rpy_mat(0, -th) @ np.array([SIDE[leg][0] * HX, 0, 0])
        return np.array([h[0], h[2]])

    def pose_margin(x, z, th, i, feet, swing_pts):
        """机身在 (x, z, th) 时：撑着的腿 + 跨的那条腿（抬到最高的两个点）离台阶 / 够不够得着的最小余量。"""
        m = min(leg_check(hip_at(LEGS[j], x, z, th), np.array([feet[j][0], feet[j][2]]), T) for j in range(4) if j != i)
        if i >= 0:
            m = min([m] + [leg_check(hip_at(LEGS[i], x, z, th), q, T) for q in swing_pts])
        return m

    def best_pose(x, z0, th0, i, feet, swing_pts, prev_off):
        """机身高度、俯仰在规则给的值附近再挑：高低 ±5 cm、俯仰 ±10°（最多斜 22°，背上货不滑）。
        上下楼梯的开头、结尾机身在“变坡”，规则给的高度 / 俯仰常常让腿拉太直或收太紧，这里补上。
        跟上一步比，改的量每步最多变 2 cm / 5°（不然两步之间机身猛地一抬一低会翻）；余量够了（≥ 2 cm）就挑跟上一步最接近的。"""
        best = None
        for dz in np.arange(-0.05, 0.0501, 0.01):
            for dth in np.radians(np.arange(-10, 10.01, 2.5)):
                if abs(dz - prev_off[0]) > 0.0201 or abs(dth - prev_off[1]) > math.radians(5.01):
                    continue
                th = float(np.clip(th0 + dth, -math.radians(22), math.radians(22)))
                m = pose_margin(x, z0 + dz, th, i, feet, swing_pts)
                key = (min(m, 0.02), -abs(dz - prev_off[0]) - 0.2 * abs(dth - prev_off[1]) - 0.1 * abs(dz) - 0.02 * abs(dth))
                if best is None or key > best[0]:
                    best = (key, dz, dth, m)
        return best[1], best[2], best[3]

    feet = [f.copy() for f in feet0]
    out = []
    y_prev = 0.0
    prev_pose = None
    off = (0.0, 0.0)
    for _, _, i, xb, xc in events:
        leg = LEGS[i]
        sup = [j for j in range(4) if j != i]
        tri = np.array([feet[j][:2] for j in sup])
        old = feet[i]
        lifted = [f.copy() for f in feet]
        lifted[i] = old + np.array([0, 0, 0.06])
        best = None
        for ddx in sorted(np.arange(-0.10, 0.1001, 0.01), key=abs):
            ys = np.arange(-0.12, 0.1201, 0.005)
            mg = np.array([tri_margin(tri, np.array([xb + ddx, y])) for y in ys])
            okk = mg >= MARGIN
            y = ys[okk][np.argmin(np.abs(ys[okk] - y_prev))] if okk.any() else ys[np.argmax(mg)]
            cand = (float(tri_margin(tri, np.array([xb + ddx, y]))), ddx, y)
            if cand[0] >= MARGIN:
                if ddx == 0 or legs_ok(xb + ddx, i, feet) >= min(0.0, legs_ok(xb, i, feet)):
                    best = cand
                    break
            elif best is None or cand[0] > best[0]:
                best = cand
        stab, ddx, y = best
        want = np.array([xb + ddx, y])
        y_prev = y
        bxy = want.copy()
        for _ in range(3):                                              # 机身放哪，重心（抬着这条腿）才正好在 want
            z, th = body_pose(bxy[0])
            bxy += want - com_fn(np.array([bxy[0], bxy[1], z]), th, lifted)
        z, th = body_pose(bxy[0])
        ss = np.linspace(min(old[0], xc) - 0.03, max(old[0], xc) + 0.03, 20)
        z_top = max(T.gnd(x) for x in ss) + FOOT_R + clear
        new = np.array([xc, old[1], T.gnd(xc) + FOOT_R])
        sw_pts = [np.array([old[0], z_top]), np.array([xc, z_top])]
        m_rule = pose_margin(bxy[0], z + off[0] * 0.5, th + off[1] * 0.5, i, feet, sw_pts)
        if m_rule >= 0.01:
            dz, dth, pm = off[0] * 0.5, off[1] * 0.5, m_rule                 # 规则姿势（上一步改的量减半）够好：不再改
        else:
            dz, dth, pm = best_pose(bxy[0], z, th, i, feet, sw_pts, off)
        off = (dz, dth)
        for _ in range(3):                                              # 高度 / 俯仰改了，重心也跟着变：再对一次重心
            z, th = body_pose(bxy[0])
            z, th = z + dz, th + dth
            bxy += want - com_fn(np.array([bxy[0], bxy[1], z]), th, lifted)
        z, th = body_pose(bxy[0])
        z, th = z + dz, th + dth
        stab = float(tri_margin(tri, com_fn(np.array([bxy[0], bxy[1], z]), th, lifted)))
        pm = min(pm, pose_margin(bxy[0], z, th, i, feet, sw_pts))
        if prev_pose is not None:                                       # 从上一个姿势挪过来的半路上也查一遍（撑着的四条腿）
            pm = min(pm, pose_margin((prev_pose[0] + bxy[0]) / 2, (prev_pose[1] + z) / 2, (prev_pose[2] + th) / 2, -1, feet, []))
        prev_pose = (bxy[0], z, th)
        lm = pm                                                          # 落脚点规划（leg_dp）按规则姿势算，这里按挑过的姿势真查
        out.append((i, np.array([bxy[0], bxy[1], z]), th, new, z_top, min(lm, stab - MARGIN + M_OK), lm, stab))
        feet[i] = new
    # 收尾：最后一次跨完，机身走到终点（四只脚撑着）
    z, th = body_pose(x_goal)
    dz, dth, pm = best_pose(x_goal, z, th, -1, feet, [], off)
    z, th = z + dz, th + dth
    out.append((None, np.array([x_goal, 0.0, z]), th, None, None, min(pm, M_OK), pm, 1.0))
    return out


def crawl(s, T, x_goal, rec, H=0.25, D=0.26, direction=1, t_shift=0.5, t_swing=0.7, clear=0.05, dry=False, dHs=(0.0, 0.015, 0.03, 0.045, 0.06), ph_step=0.02):
    """静步：先规划（前腿、后腿的跨步时机 × 机身高度 × 抬脚高度都试一遍，挑整段余量最大的），再在物理仿真里一步一步做。"""
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
    best = None
    phs = np.arange(0.0, D - 1e-9, ph_step)
    for dh in dHs:
        for cl in sorted({clear, 0.035}):
            cache = {}
            body_pose = body_pose_fn(T, H + dh)
            hip2d = hip_fn(body_pose)
            # 前腿、后腿各自挑最好的几个时机（腿余量只跟自己的时机有关），再组合起来看稳不稳
            score = {}
            for ph in phs:
                for side, leg in (("R", "RL"), ("F", "FL")):
                    key = (side, round(ph, 4), round(H + dh, 4), round(cl, 4), round(D, 4))
                    sw = []
                    k = 0
                    while True:
                        x = x0 + direction * (ph + k * D)
                        if direction * (x_goal - x) <= 0 or k > 200:
                            break
                        sw.append(x)
                        k += 1
                    cache[key] = (sw,) + leg_dp(T, leg, feet0[LEGS.index(leg)][0], x0, sw, x_goal, hip2d, cl)
                    score[(side, ph)] = cache[key][2]
            top_r = sorted(phs, key=lambda p: -score[("R", p)])[:3]
            top_f = sorted(phs, key=lambda p: -score[("F", p)])[:3]
            for pr in top_r:
                for pf in top_f:
                    plan = plan_crawl(T, feet0, x0, x_goal, com_fn, H + dh, D, direction, pr, pf, cl, cache)
                    m = min(p[5] for p in plan)
                    if best is None or m > best[0] + 1e-4:
                        best = (m, (pr, pf, dh, cl), plan)
    m_plan, ph, plan = best
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
        if i is None:
            break
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
        act = s.foot_world()
        for j in range(4):
            if j != i and np.linalg.norm(act[j] - feet_w[j]) < 0.05:
                feet_w[j] = act[j].copy()                               # 撑着的脚滑了一点：就按它现在的位置撑，不往回拽
        if os.environ.get("CRAWL_DEBUG"):
            Rn = quat_mat(s.d.qpos[3:7])
            act = s.foot_world()
            slip = max(np.linalg.norm((act[j] - feet_w[j])[:2]) for j in range(4))
            print(f"  步 {LEGS[i]} 机身 计划 {p1.round(3)} 实际 {s.d.qpos[0:3].round(3)}  俯仰 计划 {math.degrees(th_new):.1f} 实际 "
                  f"{math.degrees(math.asin(np.clip(Rn[2, 0], -1, 1))):.1f}  横滚 {math.degrees(math.atan2(Rn[2, 1], Rn[2, 2])):.1f}  "
                  f"脚偏 {slip * 1000:.0f} mm  规划余量 {m_leg * 1000:+.0f} 稳 {stab * 1000:+.0f}", flush=True)
    t0 = s.d.time                                                     # 收尾：四只脚着地站 1 秒
    while s.d.time - t0 < 1.0:
        s.step(p, rpy_mat(0, -pitch), feet_w, [1, 1, 1, 1])
        rec.tick()
    s.plan = plan
    return len(plan) - 1, m_plan, ph


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
    steps, m_plan, ph = crawl(s, T, x_goal, rec, H=H, D=t / 2, direction=direction)   # 机身每走半个踏步，四条腿各跨一次
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
    TAG = "_空载" if os.environ.get("PAYLOAD") is not None and float(os.environ["PAYLOAD"]) <= 0 else ""
    if TAG:                                                             # 不背货：结果、图另存一份，不覆盖背 5 kg 的
        NAMES = {k: v + "（空载）" for k, v in NAMES.items()}
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
    mine = {n + TAG: results[n] for n in names}                                # 几个场景分开同时跑时，写之前再读一遍，只更新自己跑的
    results = json.load(open(rp, encoding="utf-8")) if os.path.exists(rp) else {}
    results.update(mine)
    json.dump(results, open(rp, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
