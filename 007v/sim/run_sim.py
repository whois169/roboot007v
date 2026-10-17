"""机械狗007vMuJoCo 仿真：电机扭矩够不够、会不会翻。

场景（都按背上 10 kg）：
  stand      站着不动：髋关节要撑多大力、每个轮子分多少重
  standup    从折叠趴着站起来，再趴下（髋 80°/秒）
  drive      平地加速到 1.4 m/s（跟人走）→ 正常刹车 → 再加速 → 急停
  slope      10% 坡：停住、上坡起步、匀速、坡上停车、倒着下坡
  strafe     横着走 0.5 m/s、原地转 90°/秒（麦轮能不能真横走）
  bump       0.8 m/s 压过 15 mm 高的门槛条
  lift_direct  其他腿不动、直接抬左后腿（演示为什么要先调姿势）
  lift_FL / lift_RL   先按 lift_plan 调姿势，再把腿抬离地 8 cm
用法：python 007v/sim/run_sim.py [场景名…]      不写就全跑（约 1–2 分钟）
输出：007v/out/sim_report.txt，007v/out/sim.json（3D 页回放用），并重新打包 007v/out/007v_data.js

电机模型：
  髋 = 5840 蜗轮（自锁）× 3:1 同步带，位置环；额定按 20.7 N·m 判（仿真里放到 60 只为测出“需要多少”）
  轮 = GIM4310-10，位置/速度环，峰值 5.6 N·m，并按 24V 电压限制随转速降低（额定点 150 转/分 2.05 N·m，空载 228 转/分）
  头 = 总线舵机，上限 1.9 N·m
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
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
from config import *  # noqa: E402,F401,F403
from lift_plan import LEGS, Geo  # noqa: E402

OUT = os.path.join(ROOT, "out")
MJ = os.path.join(OUT, "mujoco")
DT = 0.0005
FPS = 25
G = 9.81
R_M = WHEEL_D / 2000
FX = np.array([1, 1, -1, -1])
SY = np.array([1, -1, 1, -1])
S_ROLL = np.array([-1, 1, 1, -1])                     # 和 export_mujoco.SIGN_S 一样（FL FR RL RR）
YW = SY * (BODY_W / 2 + WHEEL_GAP + WHEEL_W / 2) / 1000

HIP_RATED = 20.7                                       # 6.9 × 3
WHEEL_RATED, WHEEL_PEAK = 2.05, 5.6
WHEEL_RPM_RATED, WHEEL_RPM_FREE = 150.0, 228.0
WHEEL_STALL = WHEEL_RATED / (1 - WHEEL_RPM_RATED / WHEEL_RPM_FREE)   # 电压限制线外推到 0 转速
HEAD_LIMIT = 1.9
HIP_SPEED = 80.0
V_TOP = 1.1                                             # 最高速度：110 轮 + GIM4310-10 带载约 1.1 m/s（2026-10-17 轮子改小后）
HIP_ACC = 300.0                                         # °/s²，髋从 0 加到 80°/秒 约 0.27 秒

KP_H, KD_H = 1500.0, 30.0
KP_W, KD_W, KD_FREE = 30.0, 2.0, 0.02
KP_HD, KD_HD = 15.0, 0.4
EMA = DT / 0.01                                         # 扭矩读数做 10 ms 平滑（电流环本身的带宽量级）


def f(v):
    return " ".join(f"{x:.6g}" for x in v)


XML = None
CARGO_I1 = np.array([0.18 ** 2 + 0.2 ** 2, 0.18 ** 2 + 0.2 ** 2, 2 * 0.18 ** 2]) / 12    # 和 export_mujoco.CARGO_SIZE 一致，按 1 kg
# self.slow 每一行的列
C_LOAD, C_VX, C_VY, C_WZ, C_ROLL, C_PITCH, C_YAW, C_X, C_Y, C_Z, C_VZ = slice(2, 6), 6, 7, 8, 9, 10, 11, 12, 13, 14, 15


def load_model(payload, obstacles):
    global XML
    if XML is None:
        with open(os.path.join(MJ, "007v.xml"), encoding="utf-8") as fo:
            XML = fo.read()
    pm = max(payload, 1e-4)
    obs = "".join(f'<geom name="obs{i}" type="box" pos="{f(o["pos"])}" size="{f(o["size"])}" rgba="0.9 0.65 0.25 1" '
                  f'contype="1" conaffinity="1" friction="0.9 0.005 0.0001"/>' for i, o in enumerate(obstacles))
    xml = XML.replace("PAYLOAD_MASS", f"{pm:g}").replace("PAYLOAD_INERTIA", f(CARGO_I1 * pm)).replace("<!--OBSTACLES-->", obs)
    cwd = os.getcwd()
    os.chdir(MJ)                                        # 路径里有中文，MuJoCo 用相对路径读网格
    try:
        return mujoco.MjModel.from_xml_string(xml)
    finally:
        os.chdir(cwd)


class Robot:
    def __init__(self, payload=PAYLOAD, slope_deg=0.0, obstacles=()):
        self.m = m = load_model(payload, obstacles)
        self.d = d = mujoco.MjData(m)
        a = math.radians(slope_deg)
        m.opt.gravity[:] = (-G * math.sin(a), 0, -G * math.cos(a))   # 坡：把重力斜过来（+X 是上坡）
        self.payload, self.slope, self.obstacles = payload, slope_deg, list(obstacles)
        jq = lambda n: m.joint(n).qposadr[0]  # noqa: E731
        jv = lambda n: m.joint(n).dofadr[0]  # noqa: E731
        self.hq = np.array([jq(f"hip_{k}") for k in LEGS])
        self.hv = np.array([jv(f"hip_{k}") for k in LEGS])
        self.wq = np.array([jq(f"wheel_{k}") for k in LEGS])
        self.wv = np.array([jv(f"wheel_{k}") for k in LEGS])
        self.headq, self.headv = jq("head"), jv("head")
        self.a_hip = np.array([m.actuator(f"hip_{k}").id for k in LEGS])
        self.a_wheel = np.array([m.actuator(f"wheel_{k}").id for k in LEGS])
        self.a_head = m.actuator("head").id
        self.base = m.body("base").id
        # 哪个碰撞体属于哪个轮子
        self.gw = np.full(m.ngeom, -1)
        for g in range(m.ngeom):
            name = m.body(m.geom_bodyid[g]).name
            for i, k in enumerate(LEGS):
                if name.startswith(k + "_r"):
                    self.gw[g] = i
        self.ground = {m.geom("floor").id} | {m.geom(f"obs{i}").id for i in range(len(self.obstacles))}
        mujoco.mj_resetDataKeyframe(m, d, 0)
        mujoco.mj_forward(m, d)
        self.hip_ref = d.qpos[self.hq].copy()
        self.hip_vref = np.zeros(4)
        self.head_ref = 0.0
        self.mode = "hold"                                # hold / drive：轮子位置环；free：轮子松开（只有很小的阻尼）
        self.w_ref = d.qpos[self.wq].copy()
        self.cmd = np.zeros(3)                            # 车身速度指令 vx, vy, wz（车身坐标）
        self.t = 0.0
        self.phase = ""
        self.phases = []
        self.tau = np.zeros(9)                            # 平滑后的扭矩：髋 4、轮 4、头 1
        self.loads = np.zeros(4)
        self.steps = []                                   # 每步：相位、扭矩
        self.slow = []                                    # 每 10 ms：相位、t、轮子载荷、车身速度、姿态
        self.frames = []
        self._n = 0
        self.wheel_sat = 0

    # ------------------------------------------------------------ 姿态
    def set_pose(self, hips_deg, z, head_deg=0.0):
        d = self.d
        d.qpos[:] = 0
        d.qpos[2] = z
        d.qpos[3] = 1
        d.qpos[self.hq] = np.radians(hips_deg)
        d.qpos[self.headq] = math.radians(head_deg)
        d.qvel[:] = 0
        self.hip_ref = d.qpos[self.hq].copy()
        self.head_ref = math.radians(head_deg)
        self.w_ref = d.qpos[self.wq].copy()
        mujoco.mj_forward(self.m, d)

    def rpy(self):
        Rm = self.d.xmat[self.base].reshape(3, 3)
        return np.degrees([math.atan2(Rm[2, 1], Rm[2, 2]), -math.asin(max(-1, min(1, Rm[2, 0]))), math.atan2(Rm[1, 0], Rm[0, 0])])

    def body_vel(self):
        Rm = self.d.xmat[self.base].reshape(3, 3)
        v = Rm.T @ self.d.qvel[0:3]
        return np.array([v[0], v[1], self.d.qvel[5]])

    def wheel_height(self, i):
        """第 i 个轮子最低点离地（看它所有滚子球）。"""
        gs = np.where(self.gw == i)[0]
        return float(np.min(self.d.geom_xpos[gs, 2] - self.m.geom_size[gs, 0]))

    def wheel_loads(self):
        d, m = self.d, self.m
        out = np.zeros(4)
        f6 = np.zeros(6)
        for i in range(d.ncon):
            g1, g2 = d.contact.geom1[i], d.contact.geom2[i]
            k = self.gw[g1] if self.gw[g1] >= 0 else self.gw[g2]
            if k < 0 or not ({g1, g2} & self.ground):
                continue
            mujoco.mj_contactForce(m, d, i, f6)
            out[k] += f6[0]
        return out

    # ------------------------------------------------------------ 控制（每 0.5 ms）
    def control(self):
        d = self.d
        th = d.qpos[self.hq]
        tau_h = KP_H * (self.hip_ref - th) + KD_H * (self.hip_vref - d.qvel[self.hv])
        d.ctrl[self.a_hip] = np.clip(tau_h, -60, 60)
        # 麦轮逆运动学：ω = (vx + s·vy + wz·(s·x − y)) / r
        xc = FX * (HIP_X / 1000 + ARM / 1000 * np.cos(th))
        vx, vy, wz = self.cmd
        wref = (vx + S_ROLL * vy + wz * (S_ROLL * xc - YW)) / R_M
        wq, wv = d.qpos[self.wq], d.qvel[self.wv]
        if self.mode == "free":
            tau_w = -KD_FREE * wv
            self.w_ref = wq.copy()
        else:
            self.w_ref += wref * DT
            err = np.clip(self.w_ref - wq, -0.3, 0.3)
            self.w_ref = wq + err
            tau_w = KP_W * err + KD_W * (wref - wv)
            # 电机能出的力矩：峰值 5.6；同方向转得越快能出得越少（24V 电压限制）
            rpm = np.abs(wv) * 60 / (2 * math.pi)
            lim_drive = np.minimum(WHEEL_PEAK, np.maximum(0, WHEEL_STALL * (1 - rpm / WHEEL_RPM_FREE)))
            lim = np.where(np.sign(tau_w) == np.sign(wv), lim_drive, WHEEL_PEAK)
            self.wheel_sat += int(np.any(np.abs(tau_w) > lim))
            tau_w = np.clip(tau_w, -lim, lim)
        d.ctrl[self.a_wheel] = tau_w
        d.ctrl[self.a_head] = np.clip(KP_HD * (self.head_ref - d.qpos[self.headq]) - KD_HD * d.qvel[self.headv], -HEAD_LIMIT, HEAD_LIMIT)

    def step(self):
        self.control()
        mujoco.mj_step(self.m, self.d)
        self.t += DT
        self._n += 1
        a = np.concatenate([self.d.ctrl[self.a_hip], self.d.ctrl[self.a_wheel], [self.d.ctrl[self.a_head]]])
        self.tau += EMA * (a - self.tau)
        self.steps.append((len(self.phases) - 1, *self.tau))
        if self._n % 20 == 0:
            self.loads = self.wheel_loads()
            self.slow.append((len(self.phases) - 1, self.t, *self.loads, *self.body_vel(), *self.rpy(), *self.d.qpos[0:3], self.d.qvel[2]))
        if self._n % int(1 / (FPS * DT)) == 0:
            q = self.d.qpos
            self.frames.append([round(float(x), 4) for x in (
                self.t, *q[0:7], *q[self.hq], *q[self.wq], q[self.headq], *self.tau[:8], *(self.loads / G))])

    def begin(self, label):
        if label != self.phase:
            self.phase = label
            self.phases.append((round(self.t, 3), label))

    # ------------------------------------------------------------ 动作
    def run(self, seconds, label, fn=None):
        self.begin(label)
        n = int(round(seconds / DT))
        for i in range(n):
            if fn:
                fn(i * DT)
            self.step()

    def traj(self, path_deg, label, speed=HIP_SPEED, heads_deg=None, slow=None, acc=HIP_ACC):
        """髋沿一串角度（K×4，度；第一个点是当前位置）走。路程按“动得最多的那个关节”算，
        限速 speed、限加速度 acc（真电机不能一下子从 0 跳到全速）；
        slow=(lo, hi, v)：前左髋角度在 lo..hi 之间时限速 v（比如轮子接地前后放慢）。"""
        P = np.radians(np.asarray(path_deg, float))
        Hd = np.radians(np.asarray(heads_deg, float)) if heads_deg is not None else np.full(len(P), self.head_ref)
        lam = np.concatenate([[0], np.cumsum(np.max(np.abs(np.diff(P, axis=0)), axis=1))])
        Lt = lam[-1]
        self.begin(label)
        if Lt < 1e-6:
            return
        a = math.radians(acc)
        grid = np.linspace(0, Lt, max(2, int(Lt / math.radians(0.1)) + 1))
        vlim = np.full(len(grid), math.radians(speed))
        if slow:
            th0 = np.degrees(np.interp(grid, lam, P[:, 0]))
            vlim[(th0 >= slow[0]) & (th0 <= slow[1])] = math.radians(slow[2])
        vlim[-1] = 0.0
        dl = grid[1] - grid[0]
        for i in range(len(grid) - 2, -1, -1):                       # 倒着推：提前减速
            vlim[i] = min(vlim[i], math.sqrt(vlim[i + 1] ** 2 + 2 * a * dl))
        dP = np.diff(P, axis=0) / np.maximum(np.diff(lam), 1e-12)[:, None]
        s, v = 0.0, 0.0
        while Lt - s > 1e-5:
            v = min(v + a * DT, max(float(np.interp(s, grid, vlim)), a * DT))
            s = min(Lt, s + v * DT)
            seg = min(int(np.searchsorted(lam, s, side="right")) - 1, len(dP) - 1)
            self.hip_ref = np.array([np.interp(s, lam, P[:, j]) for j in range(4)])
            self.hip_vref = dP[seg] * v
            self.head_ref = float(np.interp(s, lam, Hd))
            self.step()
        self.hip_vref = np.zeros(4)

    def move(self, hips_deg, label, speed=HIP_SPEED, head_deg=None, slow=None):
        """髋（和头）转到目标角度。"""
        goal = np.broadcast_to(np.asarray(hips_deg, float), (4,))
        heads = None if head_deg is None else [math.degrees(self.head_ref), head_deg]
        self.traj([np.degrees(self.hip_ref), goal], label, speed, heads, slow)

    def follow(self, path_deg, label, speed=40.0):
        """沿一串髋角度走（第一个点应当是当前位置）。"""
        self.traj([np.degrees(self.hip_ref)] + [list(p) for p in path_deg[1:]], label, speed)

    def ramp(self, cmd, acc, label):
        """车身速度指令平滑地变到 cmd（S 形，加速度从 0 慢慢加上去，最大等于 acc）。acc 可以是一个数或 (ax, ay, aw)。"""
        c0 = self.cmd.copy()
        c1 = np.asarray(cmd, float)
        acc = np.broadcast_to(np.asarray(acc, float), (3,))
        T = max(1.5 * float(np.max(np.abs(c1 - c0) / acc)), 0.05)

        def fn(t):
            k = min(1.0, t / T)
            self.cmd = c0 + (c1 - c0) * (3 * k * k - 2 * k ** 3)
        self.mode = "drive"
        self.run(T, label, fn)
        self.cmd = c1

    # ------------------------------------------------------------ 统计
    def arr(self):
        S = np.array(self.steps)
        L = np.array(self.slow)
        return S, L

    def mask(self, labels, tmin=0.0):
        """相位名字在 labels 里、并且进入该相位 tmin 秒以后的样本。"""
        labels = [labels] if isinstance(labels, str) else labels
        ids = {i for i, (_, lb) in enumerate(self.phases) if lb in labels}
        starts = {i: t for i, (t, _) in enumerate(self.phases)}
        S, L = self.arr()
        ts = np.arange(len(S)) * DT + DT
        ms = np.array([int(p) in ids for p in S[:, 0]]) & (ts >= np.array([starts.get(int(p), 0) for p in S[:, 0]]) + tmin)
        ml = np.array([int(p) in ids for p in L[:, 0]]) & (L[:, 1] >= np.array([starts.get(int(p), 0) for p in L[:, 0]]) + tmin)
        return S[ms], L[ml]


# ---------------------------------------------------------------- 场景
def smooth(x, n=5):
    """沿时间做 n 点（10 ms 一点）滑动平均，去掉滚子交接时的瞬时跳动。"""
    k = np.ones(n) / n
    return np.stack([np.convolve(x[:, j], k, mode="valid") for j in range(x.shape[1])], 1) if len(x) >= n else x


def row(item, value, unit="", limit=None, ok=None, note=""):
    return dict(item=item, value=value, unit=unit, limit=limit, ok=ok, note=note)


def pack(r, key, title, desc, rows, notes=(), demo=False):
    return dict(key=key, title=title, desc=desc, payload=r.payload, slope=r.slope, obstacles=r.obstacles,
                phases=r.phases, frames=r.frames, rows=rows, notes=list(notes), demo=demo)


def hip_rows(S, label="髋关节扭矩最大"):
    h = np.abs(S[:, 1:5]).max(0)
    i = int(np.argmax(h))
    return row(label, round(float(h[i]), 1), "N·m", HIP_RATED, bool(h[i] <= HIP_RATED), f"{['前左', '前右', '后左', '后右'][i]}腿")


def sc_stand():
    r = Robot()
    r.run(2.5, "站着不动（轮子锁住）")
    S, L = r.mask("站着不动（轮子锁住）", 1.5)
    h = np.abs(S[:, 1:5]).mean(0)
    loads = L[:, 2:6].mean(0) / G
    rows = [row("前腿髋关节撑住车身", round(float(h[:2].mean()), 1), "N·m", HIP_RATED, bool(h.max() <= HIP_RATED)),
            row("后腿髋关节撑住车身", round(float(h[2:].mean()), 1), "N·m", HIP_RATED, bool(h.max() <= HIP_RATED)),
            row("每个轮子分到的重量", " / ".join(f"{x:.1f}" for x in loads), "kg（前左/前右/后左/后右）"),
            row("轮子电机要出力吗", "不用", "", None, True, "蜗轮自锁，站着不耗电")]
    return pack(r, "stand", "站着不动", "背上 10 kg，看髋关节要撑多大力。", rows)


TOUCH_SLOW = 8.0      # 轮子接地（摆臂约 -3°）前后 ±8° 放慢到 20°/秒，不然 25 kg 一下子被“拽”起来


def standup(r, slow=True):
    """从折叠站起来、再趴下。返回 (站起来那段的标签, 趴下那段的标签, 站起来后的机身高度)。"""
    tg = math.degrees(math.asin((SKID_H - WHEEL_D / 2 + HIP_Z) / ARM))
    up, dn = "展开、站起来", "趴下、折叠"
    r.set_pose([-110.0] * 4, SKID_H / 1000 + 0.0005, HEAD_PITCH["fold"])
    r.mode = "free"
    r.run(0.6, "折叠趴着")
    zone = (tg - TOUCH_SLOW, tg + TOUCH_SLOW, 20.0) if slow else None
    r.move(STANCE_DEG, up, head_deg=0.0, slow=zone)
    r.mode = "hold"
    r.run(1.2, "站稳")
    z_up = r.d.qpos[2]
    r.mode = "free"
    r.move(-110.0, dn, head_deg=HEAD_PITCH["fold"], slow=zone)
    r.run(0.8, "折叠趴着 ")
    return up, dn, z_up


def sc_standup():
    fast = Robot()
    up, dn, _ = standup(fast, slow=False)
    peak_fast = float(np.abs(fast.mask(up)[0][:, 1:5]).max())
    r = Robot()
    up, dn, z_up = standup(r)
    S1, L1 = r.mask(up)
    S2, _ = r.mask(dn)
    tilt = np.abs(L1[:, C_ROLL:C_PITCH + 1]).max()
    exp = (WHEEL_D / 2 - HIP_Z + ARM * math.sin(math.radians(STANCE_DEG))) / 1000
    T = r.phases[2][0] - r.phases[1][0]
    rows = [hip_rows(S1, "站起来时髋扭矩最大"), hip_rows(S2, "趴下时髋扭矩最大"),
            row("站起来用时", round(T, 1), "秒"),
            row("站起来后机身离地", round(float(z_up * 1000)), "mm", None, bool(abs(z_up - exp) < 0.01), f"设计值 {exp * 1000:.0f}"),
            row("站起来过程车身最大倾斜", round(float(tilt), 1), "°", None, bool(tilt < 5))]
    return pack(r, "standup", "折叠 → 站起来 → 趴下", "背上 10 kg，从折叠趴着展开站起来，再趴下折叠（髋 80°/秒，轮子接地前后放慢到 20°/秒）。", rows,
                [f"如果一直 80°/秒不放慢，轮子接地那一下髋扭矩冲到 {peak_fast:.1f} N·m（之后稳定在 8–10）；接地前后放慢以后见上面"])


def sc_drive():
    r = Robot()
    r.run(0.5, "站稳")
    r.ramp((V_TOP, 0, 0), 0.5, f"加速到 {V_TOP} m/s（0.5 m/s²）")
    r.run(1.5, f"匀速 {V_TOP} m/s")
    r.ramp((0, 0, 0), 1.0, "正常刹车（1 m/s²）")
    r.run(0.6, "停稳")
    r.ramp((V_TOP, 0, 0), 0.5, f"再加速到 {V_TOP} m/s")
    r.run(1.0, f"匀速 {V_TOP} m/s")
    x0 = r.d.qpos[0]
    r.ramp((0, 0, 0), 4.0, "急停（4 m/s²）")
    r.run(1.0, "急停后停住")
    x1 = r.d.qpos[0]
    S_acc, _ = r.mask([f"加速到 {V_TOP} m/s（0.5 m/s²）", f"再加速到 {V_TOP} m/s"])
    S_cru, L_cru = r.mask(f"匀速 {V_TOP} m/s", 0.5)
    S_brk, L_brk = r.mask(["急停（4 m/s²）", "急停后停住"])
    v_cru = L_cru[:, C_VX].mean()
    wt = lambda S: float(np.abs(S[:, 5:9]).max())  # noqa: E731
    vb = L_brk[:, C_VX]
    tb = L_brk[:, 1]
    i0 = int(np.argmax(vb < V_TOP - 0.1)) if (vb < V_TOP - 0.1).any() else 0
    i1 = int(np.argmax(vb < 0.1)) if (vb < 0.1).any() else len(vb) - 1
    dec = (vb[i0] - vb[i1]) / max(tb[i1] - tb[i0], 1e-3)
    rear = smooth(L_brk[:, 4:6]).min() / G
    pitch = np.abs(L_brk[:, C_PITCH]).max()
    rows = [row("加速时轮子扭矩最大", round(wt(S_acc), 2), "N·m", WHEEL_PEAK, wt(S_acc) <= WHEEL_PEAK, f"额定 {WHEEL_RATED}"),
            row(f"{V_TOP} m/s 匀速时轮子扭矩", round(float(np.abs(S_cru[:, 5:9]).mean()), 2), "N·m", WHEEL_RATED,
                bool(np.abs(S_cru[:, 5:9]).mean() <= WHEEL_RATED)),
            row("实际速度", round(float(v_cru), 2), "m/s", None, bool(v_cru > V_TOP * 0.95), f"指令 {V_TOP}"),
            row("急停实际减速度", round(float(dec), 1), "m/s²", None, bool(dec > 3.0), "指令 4"),
            row("急停刹车距离", round(float((x1 - x0) * 100)), "cm"),
            row("急停时后轮最少还压着", round(float(rear), 1), "kg", None, bool(rear > 0.5), "大于 0 就没翘起来"),
            row("急停时车身最大点头", round(float(pitch), 1), "°")]
    return pack(r, "drive", "平地加速和急停", f"背上 10 kg，加速到最高速 {V_TOP} m/s（110 轮 + GIM4310-10 的上限），正常刹车，再加速后急停。", rows,
                [f"轮子电机限幅触发 {r.wheel_sat * DT:.2f} 秒"] if r.wheel_sat else [])


def sc_slope():
    deg = math.degrees(math.atan(0.10))
    r = Robot(slope_deg=deg)
    r.run(1.5, "坡上停住（轮子锁住）")
    r.ramp((1.0, 0, 0), 0.5, "上坡起步（0.5 m/s²）")
    r.run(2.0, "匀速上坡 1.0 m/s")
    r.ramp((0, 0, 0), 0.5, "坡上减速停车")
    r.run(1.5, "坡上停住（轮子锁住）")
    r.ramp((-1.0, 0, 0), 0.5, "倒着下坡起步")
    r.run(1.5, "匀速下坡 1.0 m/s")
    r.ramp((0, 0, 0), 0.5, "下坡刹车")
    r.run(1.0, "坡上停住（轮子锁住）")
    S_hold, L_hold = r.mask("坡上停住（轮子锁住）", 0.5)
    first = L_hold[L_hold[:, 1] < 1.6]
    slip = abs(first[-1, C_X] - first[0, C_X])
    S_up, L_up = r.mask("匀速上坡 1.0 m/s", 0.5)
    S_st, _ = r.mask("上坡起步（0.5 m/s²）")
    S_dn, _ = r.mask(["下坡刹车"])
    S_all, _ = r.mask([lb for _, lb in r.phases])
    up = float(np.abs(S_up[:, 5:9]).mean(0).max())
    rpm = 1.0 / R_M * 60 / (2 * math.pi)
    avail = min(WHEEL_PEAK, WHEEL_STALL * (1 - rpm / WHEEL_RPM_FREE))
    rows = [row("坡上停住时轮子扭矩", round(float(np.abs(S_hold[:, 5:9]).mean(0).max()), 2), "N·m", WHEEL_RATED,
                bool(np.abs(S_hold[:, 5:9]).mean(0).max() <= WHEEL_RATED)),
            row("匀速上坡时轮子扭矩（最累的轮）", round(up, 2), "N·m", WHEEL_RATED, up <= WHEEL_RATED, f"这个转速下电机最多能出 {avail:.1f}"),
            row("上坡起步时轮子扭矩最大", round(float(np.abs(S_st[:, 5:9]).max()), 2), "N·m", WHEEL_PEAK,
                bool(np.abs(S_st[:, 5:9]).max() <= WHEEL_PEAK)),
            row("下坡刹车时轮子扭矩最大", round(float(np.abs(S_dn[:, 5:9]).max()), 2), "N·m", WHEEL_PEAK,
                bool(np.abs(S_dn[:, 5:9]).max() <= WHEEL_PEAK)),
            row("实际上坡速度", round(float(L_up[:, C_VX].mean()), 2), "m/s", None, bool(L_up[:, C_VX].mean() > 0.95), "指令 1.0"),
            hip_rows(S_all, "坡上髋扭矩最大"),
            row("停在坡上会不会溜", f"{slip * 1000:.1f}", "mm（第一次停住的 1 秒里滑了多少）", None, bool(slip < 0.005))]
    return pack(r, "slope", "10% 坡", "背上 10 kg，在 10% 的坡（5.7°）上停住、上坡、坡上停车、倒着下坡。", rows)


def sc_strafe():
    r = Robot()
    r.run(0.5, "站稳")
    r.ramp((0, 0.5, 0), 0.5, "横着往左走起来")
    yaw0 = r.rpy()[2]
    r.run(2.0, "匀速横走 0.5 m/s")
    yaw1 = r.rpy()[2]
    r.ramp((0, 0, 0), 0.5, "横走停车")
    r.run(0.5, "停稳")
    r.ramp((0, 0, math.radians(90)), (1, 1, math.radians(180)), "原地转起来")
    r.run(2.0, "原地转 90°/秒")
    r.ramp((0, 0, 0), (1, 1, math.radians(180)), "停转")
    r.run(0.5, "停稳")
    S_s, L_s = r.mask("匀速横走 0.5 m/s", 0.5)
    S_t, L_t = r.mask("原地转 90°/秒", 0.5)
    S_all, _ = r.mask([lb for _, lb in r.phases])
    vy = float(L_s[:, C_VY].mean())
    vx = float(np.abs(L_s[:, C_VX]).mean())
    wz = float(np.degrees(L_t[:, C_WZ].mean()))
    rows = [row("实际横走速度", round(vy, 2), "m/s", None, vy > 0.45, "指令 0.5"),
            row("横走时往前后跑偏", round(vx, 3), "m/s", None, vx < 0.05),
            row("横走 2 秒车头转偏", round(float(abs(yaw1 - yaw0)), 1), "°", None, abs(yaw1 - yaw0) < 5),
            row("横走时轮子扭矩", round(float(np.abs(S_s[:, 5:9]).mean()), 2), "N·m", WHEEL_RATED,
                bool(np.abs(S_s[:, 5:9]).mean() <= WHEEL_RATED)),
            row("实际原地转速", round(wz, 0), "°/秒", None, wz > 80, "指令 90"),
            row("原地转时轮子扭矩", round(float(np.abs(S_t[:, 5:9]).mean()), 2), "N·m", WHEEL_RATED,
                bool(np.abs(S_t[:, 5:9]).mean() <= WHEEL_RATED)),
            row("整段轮子扭矩最大", round(float(np.abs(S_all[:, 5:9]).max()), 2), "N·m", WHEEL_PEAK,
                bool(np.abs(S_all[:, 5:9]).max() <= WHEEL_PEAK))]
    return pack(r, "strafe", "横走和原地转", "背上 10 kg，麦轮横着往左走 0.5 m/s，再原地转 90°/秒。", rows)


def bump_peak(v):
    """用速度 v 压过门槛，返回髋扭矩最大值。"""
    r = Robot(obstacles=[BUMP])
    r.run(0.3, "站稳")
    r.ramp((v, 0, 0), 0.5, "加速")
    r.run(1.6 / v, "压过 15 mm 门槛")
    return float(np.abs(r.mask("压过 15 mm 门槛")[0][:, 1:5]).max())


BUMP = dict(pos=[1.0, 0, 0.0075], size=[0.02, 0.6, 0.0075])


def sc_bump():
    bump = BUMP
    r = Robot(obstacles=[bump])
    r.run(0.5, "站稳")
    r.ramp((0.8, 0, 0), 0.5, "加速到 0.8 m/s")
    r.run(2.2, "压过 15 mm 门槛")
    r.ramp((0, 0, 0), 1.0, "停车")
    r.run(0.5, "停稳")
    S, L = r.mask("压过 15 mm 门槛")
    S0, _ = r.mask("加速到 0.8 m/s")
    acc = np.abs(np.diff(L[:, C_VZ])) / (20 * DT) / G
    passed = r.d.qpos[0] > 1.0 + 0.40
    h_flat = float(np.abs(S0[:, 1:5]).max())
    rows = [row("过得去吗", "过得去" if passed else "卡住了", "", None, bool(passed)),
            hip_rows(S, "压门槛时髋扭矩最大"),
            row("平地时髋扭矩（对比）", round(h_flat, 1), "N·m"),
            row("压门槛时轮子扭矩最大", round(float(np.abs(S[:, 5:9]).max()), 2), "N·m", WHEEL_PEAK,
                bool(np.abs(S[:, 5:9]).max() <= WHEEL_PEAK)),
            row("车身上下颠簸最大", round(float(np.max(acc)), 1), "g", None, None, "仿真里冲击只是估算"),
            row("车身最大点头", round(float(np.abs(L[:, C_PITCH]).max()), 1), "°")]
    slow = bump_peak(0.3)
    return pack(r, "bump", "压过 15 mm 门槛", "背上 10 kg，0.8 m/s 直接压过一根 15 mm 高、4 cm 宽的门槛条。", rows,
                ["髋关节是蜗轮自锁的“硬腿”，颠簸全靠轮子和轮胎吸收，冲击直接传到蜗轮上",
                 f"同样的门槛慢到 0.3 m/s 过，髋扭矩最大 {slow:.1f} N·m"])


def lift_path(geo, lifted, support, n=30):
    sup = [k for k in LEGS if k != lifted]
    path = []
    for u in np.linspace(0, 1, n):
        th = {k: STANCE_DEG + u * (support[k] - STANCE_DEG) for k in sup}
        th[lifted] = geo.touch_angle(th, lifted)
        path.append([th[k] for k in LEGS])
    return path


def sc_lift(lifted, direct=False, h=0.08):
    r = Robot()
    geo = Geo(r.m)
    i = LEGS.index(lifted)
    cn = ["前左", "前右", "后左", "后右"][i]
    r.run(0.5, "站稳")
    if direct:
        plan = None
        _, th_l = geo.straight(lifted, h_min=h)
        goal = [STANCE_DEG] * 4
        goal[i] = th_l
        r.move(goal, f"其他腿不动，直接抬{cn}腿", speed=40)
        r.run(2.0, "腿抬着停 2 秒")
        hold_lb = "腿抬着停 2 秒"
        lift_h = r.wheel_height(i)
        r.move(STANCE_DEG, "放下", speed=40)
        r.run(0.8, "站稳 ")
    else:
        plan = geo.plan(lifted, h)
        path = lift_path(geo, lifted, plan["support"])
        r.mode = "free"
        r.follow(path, "调姿势：把重心挪到另外三个轮子中间")
        r.mode = "hold"
        r.run(0.5, "锁住轮子")
        goal = list(path[-1])
        goal[i] = plan["theta_lift"]
        r.move(goal, f"抬{cn}腿", speed=40)
        r.run(2.0, "腿抬着停 2 秒")
        hold_lb = "腿抬着停 2 秒"
        lift_h = r.wheel_height(i)
        r.move(path[-1], "放下", speed=40)
        r.mode = "free"
        r.follow(path[::-1], "姿势复原")
        r.mode = "hold"
        r.run(0.8, "站稳 ")
    S, L = r.mask(hold_lb, 0.5)
    S_all, L_all = r.mask([lb for _, lb in r.phases])
    sup = [j for j in range(4) if j != i]
    min_sup = float(smooth(L[:, 2:6][:, sup]).min() / G)
    lifted_load = float(L[:, 2 + i].max() / G)
    # 抬起的轮子离地：看它所有滚子球的最低点
    tilt = float(np.hypot(L[:, C_ROLL], L[:, C_PITCH]).max())
    stable = min_sup > 1.0 and lifted_load < 0.05
    rows = []
    if plan:
        rows += [row("调姿势后（静力计算）重心离三角形边", round(plan["margin"] * 1000), "mm", None, plan["margin"] > 0.03),
                 row("三条支撑腿的角度", " / ".join(f"{['前左', '前右', '后左', '后右'][LEGS.index(k)]} {v:.0f}°" for k, v in plan["support"].items()), ""),
                 row("车身倾斜（计算）", round(plan["tilt"], 1), "°")]
    rows += [row(f"{cn}腿抬离地面了吗", "抬起来了" if lifted_load < 0.05 else f"没抬起来（还压着 {lifted_load:.1f} kg）", "", None, lifted_load < 0.05),
             row("抬起的轮子离地（仿真）", round(lift_h * 1000), "mm"),
             row("抬着时另外三个轮子最少还压着", round(min_sup, 1), "kg", None, min_sup > 1.0, "大于 0 就不会翻"),
             row("抬着时车身倾斜（仿真）", round(tilt, 1), "°"),
             hip_rows(S_all, "整个过程髋扭矩最大"),
             row("结论", "稳" if stable else "不稳 / 抬不起来", "", None, stable)]
    if direct:                                   # 演示：本来就是要看它不行，不算进合格判断
        for r_ in rows:
            r_["ok"] = None
        rows[-1]["value"] = f"抬不起来：整个车往{cn}方向歪了 {tilt:.0f}°，所以抬腿前必须先调姿势"
    key = f"lift_{lifted}" + ("_direct" if direct else "")
    title = f"直接抬{cn}腿" if direct else f"调姿势后抬{cn}腿"
    desc = (f"背上 10 kg，其他腿不动、直接把{cn}腿往上抬。" if direct else
            f"背上 10 kg，先让另外三条腿换角度把重心挪开，再把{cn}腿抬离地 {h * 100:.0f} cm 以上，停 2 秒放下。")
    return pack(r, key, title, desc, rows, demo=direct)


SCENARIOS = dict(stand=sc_stand, standup=sc_standup, drive=sc_drive, slope=sc_slope, strafe=sc_strafe, bump=sc_bump,
                 lift_direct=lambda: sc_lift("RL", direct=True), lift_FL=lambda: sc_lift("FL"), lift_RL=lambda: sc_lift("RL"))


def lift_table():
    """静力学：每条腿、空车/满载，直接抬 vs 调姿势后抬。"""
    lines = []
    for pl in (PAYLOAD, 0.0):
        geo = Geo(load_model(pl, []))
        for k in LEGS:
            cn = ["前左", "前右", "后左", "后右"][LEGS.index(k)]
            m0, _ = geo.straight(k, h_min=0.03)
            parts = [f"其他腿不动直接抬 3 cm：{m0 * 1000:+.0f} mm"]
            p = geo.plan(k, 0.10)
            parts.append("先调姿势再抬 10 cm：" + ("做不到" if p is None else f"{p['margin'] * 1000:+.0f} mm（车身歪 {p['tilt']:.0f}°，髋 {p['hip_tau']:.1f} N·m）"))
            lines.append(f"  {'满载' if pl else '空车'} {cn}腿：" + "；".join(parts))
    return lines


def main(names):
    t0 = time.time()
    if not os.path.exists(os.path.join(MJ, "007v.xml")) or "--export" in sys.argv:
        import export_mujoco
        export_mujoco.main()
        global XML
        with open(os.path.join(MJ, "007v.xml"), encoding="utf-8") as fo:
            XML = fo.read()
    names = names or list(SCENARIOS)
    old = {}
    sim_path = os.path.join(OUT, "sim.json")
    if os.path.exists(sim_path):
        with open(sim_path, encoding="utf-8") as fo:
            old = {s["key"]: s for s in json.load(fo)["scenarios"]}
    results = dict(old)
    for n in names:
        t1 = time.time()
        res = SCENARIOS[n]()
        results[res["key"]] = res
        bad = [r_["item"] for r_ in res["rows"] if r_["ok"] is False]
        print(f"{res['title']}：{len(res['frames'])} 帧，{time.time() - t1:.0f} 秒" + (f"  ✗ {'、'.join(bad)}" if bad else "  ✓"))
    order = [s for s in ("stand", "standup", "drive", "slope", "strafe", "bump", "lift_RL_direct", "lift_FL", "lift_RL") if s in results]

    report = [f"机械狗007v仿真报告  {time.strftime('%Y-%m-%d %H:%M')}",
              "重新跑：python 007v/sim/run_sim.py（改了 CAD 先跑 python 007v/build.py，再加 --export）", "",
              "电机模型：髋 = 5840 蜗轮 × 3:1 同步带（额定按 20.7 N·m 判）；轮 = GIM4310-10（峰值 5.6、额定 2.05 N·m，转速越高能出的越少）；",
              "麦轮按 12 个 45° 滚子真实建模（每个滚子能自由转）；地面摩擦系数 0.9；货物 10 kg 在货斗中间、重心高出甲板 10 cm。", ""]
    summary = []
    for k in order:
        s = results[k]
        report.append(f"【{s['title']}】{s['desc']}")
        for r_ in s["rows"]:
            mark = "" if r_["ok"] is None else ("  ✓" if r_["ok"] else "  ✗")
            lim = f"（上限 {r_['limit']}）" if r_["limit"] is not None else ""
            note = f"；{r_['note']}" if r_["note"] else ""
            report.append(f"  {r_['item']}：{r_['value']} {r_['unit']}{lim}{note}{mark}")
        for nt in s["notes"]:
            report.append(f"  注：{nt}")
        report.append("")
        if not s.get("demo"):
            summary.append((s["title"], all(r_["ok"] is not False for r_ in s["rows"])))
    report.append("【抬腿静力计算】重心离三个着地轮围成的三角形的边（正 = 在里面，越大越稳；限制车身倾斜 ≤ 12°、滑撬离地 ≥ 4 cm）")
    report += lift_table()
    report.append("")
    report.append("【结论】" + "；".join(f"{t} {'✓' if ok else '✗'}" for t, ok in summary) + "（“直接抬腿”是演示，不算）")
    report.append(f"用时 {time.time() - t0:.0f} 秒")
    with open(os.path.join(OUT, "sim_report.txt"), "w", encoding="utf-8") as fo:
        fo.write("\n".join(report) + "\n")
    print("\n".join(report))
    fields = ["t", "x", "y", "z", "qw", "qx", "qy", "qz", "hFL", "hFR", "hRL", "hRR", "wFL", "wFR", "wRL", "wRR", "head",
              "tauhFL", "tauhFR", "tauhRL", "tauhRR", "tauwFL", "tauwFR", "tauwRL", "tauwRR", "nFL", "nFR", "nRL", "nRR"]
    tmp = sim_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fo:
        json.dump(dict(fields=fields, limits=dict(hip=HIP_RATED, wheel_rated=WHEEL_RATED, wheel_peak=WHEEL_PEAK),
                       scenarios=[results[k] for k in order]), fo, ensure_ascii=False, separators=(",", ":"),
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))     # numpy 的数和真假值
    os.replace(tmp, sim_path)
    from pack import pack_viewer
    pack_viewer()


if __name__ == "__main__":
    main([a for a in sys.argv[1:] if not a.startswith("--")])
