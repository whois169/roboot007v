"""
无界面检查：用简单 PD 控制让模型 站立保持 → 坐下 → 站起 → 蹲伏 → 趴下 → 收腿蹲伏 → 站起，
（趴下和起身都经过“蹲伏”：腿收在身体下面时髋电机力矩最小）
打印机身高度、姿态、各类关节的电机端力矩（对照 DM-J4310：额定 3 N·m，峰值 7 N·m）。
用法：python3 check_poses.py
"""
import os
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))
d = mujoco.MjData(m)

KEY = {m.key(i).name: m.key_qpos[i].copy() for i in range(m.nkey)}
J = [m.joint(i).name for i in range(1, m.njnt)]
QADR = np.array([m.jnt_qposadr[i] for i in range(1, m.njnt)])
VADR = np.array([m.jnt_dofadr[i] for i in range(1, m.njnt)])
GEAR = m.actuator_gear[:, 0].copy()
KP = np.array([20.0 if "neck" in j else (60.0 if "knee" in j else 40.0) for j in J])     # 关节端 N·m/rad
KD = np.array([0.3 if "neck" in j else (1.5 if "knee" in j else 1.0) for j in J])


def target(name):
    return KEY[name][7:].copy()


def step_pd(qref):
    q, qd = d.qpos[QADR], d.qvel[VADR]
    tau_joint = KP * (qref - q) - KD * qd
    d.ctrl[:] = np.clip(tau_joint / GEAR, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
    mujoco.mj_step(m, d)


def upright():
    R = d.xmat[1].reshape(3, 3)
    return R[2, 2]          # 机身 z 轴与竖直的夹角余弦


def run(segments, label):
    """segments: [(起始姿态, 目标姿态, 过渡秒数, 保持秒数)]"""
    log = {"h": [], "up": [], "ctrl": []}
    for a, b, t_move, t_hold in segments:
        qa, qb = target(a), target(b)
        n_move, n_hold = int(t_move / m.opt.timestep), int(t_hold / m.opt.timestep)
        for k in range(n_move + n_hold):
            s = min(1.0, k / max(n_move, 1))
            s = s * s * (3 - 2 * s)
            step_pd(qa + (qb - qa) * s)
            log["h"].append(d.qpos[2]); log["up"].append(upright()); log["ctrl"].append(d.ctrl.copy())
    C = np.abs(np.array(log["ctrl"]))
    print(f"\n[{label}] 最终机身高度 {d.qpos[2] * 1000:.0f} mm，最低时机身直立度 {min(log['up']):.3f}（1 = 完全水平）")
    for grp in ("haa", "hfe", "knee", "neck"):
        idx = [i for i, j in enumerate(J) if grp in j]
        peak = C[:, idx].max()
        sat = (C[:, idx] >= np.abs(m.actuator_ctrlrange[idx, 1]) - 1e-6).mean() * 100
        last = C[-int(0.5 / m.opt.timestep):, idx].mean()
        print(f"   {grp:5s} 电机端峰值 {peak:5.2f} N·m  最后 0.5s 平均 {last:5.2f} N·m  顶到限幅的时间占比 {sat:4.1f}%")
    return log


mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
mujoco.mj_forward(m, d)
run([("stand", "stand", 0.0, 3.0)], "站立保持 3 秒")
run([("stand", "sit", 2.0, 1.5), ("sit", "stand", 2.0, 1.5)], "站立 → 坐下 → 站起")
run([("stand", "crouch", 2.5, 0.8), ("crouch", "lie", 3.0, 1.5)], "站立 → 蹲伏 → 趴下")
run([("lie", "crouch", 4.0, 1.0), ("crouch", "stand", 3.0, 1.5)], "趴下 → 收腿蹲伏 → 站起")
mujoco.mj_resetDataKeyframe(m, d, m.key("lie").id)
mujoco.mj_forward(m, d)
d.ctrl[:] = 0
for _ in range(int(2.0 / m.opt.timestep)):
    mujoco.mj_step(m, d)
print(f"\n[趴下后电机全部断电 2 秒] 机身高度 {d.qpos[2] * 1000:.0f} mm，直立度 {upright():.3f}")
