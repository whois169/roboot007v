"""
交互演示（在你自己的电脑上运行，需要显示器）：
  pip install mujoco
  python demo.py
打开 MuJoCo 窗口，机器狗按 站立 → 转头 → 坐下 → 站起 → 蹲伏 → 趴下 → 收腿蹲伏 → 站起 循环做动作。
控制器只是最简单的关节 PD（不是步态策略），用来检查模型、质量和电机力矩是否合理。
窗口里按 Tab 显示/隐藏左侧面板；按 0–3 切换显示的几何组（2 = 外观网格，3 = 碰撞体）。
"""
import os
import time
import numpy as np
import mujoco
import mujoco.viewer

HERE = os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(HERE, "scene.xml"))
d = mujoco.MjData(m)
KEY = {m.key(i).name: m.key_qpos[i][7:].copy() for i in range(m.nkey)}
J = [m.joint(i).name for i in range(1, m.njnt)]
QADR = np.array([m.jnt_qposadr[i] for i in range(1, m.njnt)])
VADR = np.array([m.jnt_dofadr[i] for i in range(1, m.njnt)])
GEAR = m.actuator_gear[:, 0].copy()
KP = np.array([20.0 if "neck" in j else (60.0 if "knee" in j else 40.0) for j in J])
KD = np.array([0.3 if "neck" in j else (1.5 if "knee" in j else 1.0) for j in J])
SEQ = [("stand", 2.0), ("look", 1.5), ("stand", 1.5), ("sit", 2.0), ("stand", 2.0),
       ("crouch", 2.5), ("lie", 3.0), ("crouch", 4.0), ("stand", 3.0)]

mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
with mujoco.viewer.launch_passive(m, d) as v:
    v.opt.geomgroup[3] = 0
    cur = KEY["stand"].copy()
    i, t0 = 0, d.time
    while v.is_running():
        name, dur = SEQ[i]
        s = min(1.0, (d.time - t0) / dur)
        s = s * s * (3 - 2 * s)
        qref = cur + (KEY[name] - cur) * s
        tau = KP * (qref - d.qpos[QADR]) - KD * d.qvel[VADR]
        d.ctrl[:] = np.clip(tau / GEAR, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
        step_start = time.time()
        mujoco.mj_step(m, d)
        v.sync()
        if d.time - t0 > dur + 1.0:
            cur, i, t0 = KEY[name].copy(), (i + 1) % len(SEQ), d.time
        time.sleep(max(0, m.opt.timestep - (time.time() - step_start)))
