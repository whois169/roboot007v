"""虚拟底盘：MuJoCo 仿真的车 + 底盘程序（controller.py），对外和真底盘一样“一行一条 JSON”。

大脑用它的方法和用真串口一样：send_line(一行指令)，收到的事件通过 on_line(一行) 回调出来。
电机模型和 sim/run_sim.py 一样（髋是蜗轮位置环；轮子是 GIM4310，24V 电压限制随转速降低）。

两种跑法：
  start()          后台线程按真实时间跑（给大脑用）；view=True 时再开一个 MuJoCo 窗口看车怎么动
  run(秒)          不按真实时间，能跑多快跑多快（测试用）
"""
import json
import math
import os
import sys
import threading
import time
from collections import deque

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "sim"))
import run_sim as S  # noqa: E402
from config import HEAD_PITCH, SKID_H, STANCE_DEG  # noqa: E402

from .controller import FOLD_DEG, RATE, Controller  # noqa: E402

SUB = int(round(1 / (RATE * S.DT)))           # 每算一次底盘程序，仿真走几步（5 ms / 0.5 ms = 10）


class Plant(S.Robot):
    """仿真里的车：髋按目标角度走位置环，轮子按每个轮子的目标转速走速度环（或松开）。不记录逐步数据。"""

    def __init__(self, payload, folded):
        super().__init__(payload=payload)
        if folded:
            self.set_pose([FOLD_DEG] * 4, SKID_H / 1000 + 0.0005, HEAD_PITCH["fold"])
        self.wref = np.zeros(4)
        self.wheel_mode = "vel"
        self.tau_w = np.zeros(4)

    def control(self):
        d = self.d
        th = d.qpos[self.hq]
        tau_h = S.KP_H * (self.hip_ref - th) + S.KD_H * (self.hip_vref - d.qvel[self.hv])
        d.ctrl[self.a_hip] = np.clip(tau_h, -60, 60)
        wq, wv = d.qpos[self.wq], d.qvel[self.wv]
        if self.wheel_mode == "free":
            tau_w = -S.KD_FREE * wv
            self.w_ref = wq.copy()
        else:
            self.w_ref += self.wref * S.DT
            err = np.clip(self.w_ref - wq, -0.3, 0.3)
            self.w_ref = wq + err
            tau_w = S.KP_W * err + S.KD_W * (self.wref - wv)
            rpm = np.abs(wv) * 60 / (2 * math.pi)
            lim_drive = np.minimum(S.WHEEL_PEAK, np.maximum(0, S.WHEEL_STALL * (1 - rpm / S.WHEEL_RPM_FREE)))
            lim = np.where(np.sign(tau_w) == np.sign(wv), lim_drive, S.WHEEL_PEAK)
            tau_w = np.clip(tau_w, -lim, lim)
        self.tau_w = tau_w
        d.ctrl[self.a_wheel] = tau_w
        d.ctrl[self.a_head] = np.clip(S.KP_HD * (self.head_ref - d.qpos[self.headq]) - S.KD_HD * d.qvel[self.headv],
                                      -S.HEAD_LIMIT, S.HEAD_LIMIT)

    def step(self):
        self.control()
        mujoco.mj_step(self.m, self.d)
        self.t += S.DT


class VirtualChassis:
    def __init__(self, payload=10.0, folded=True, on_line=None, vbat=24.6):
        self.plant = Plant(payload, folded)
        self.ctl = Controller(list(np.degrees(self.plant.d.qpos[self.plant.hq])))
        self.on_line = on_line or (lambda line: None)
        self.inbox = deque()
        self.estop_btn = False
        self.vbat = vbat
        self.lock = threading.Lock()
        self._thread = None
        self._run = False
        self.viewer = None
        self._flush()

    # ---- 和串口一样的接口
    def send_line(self, line: str):
        """大脑发来一行（线程安全）。"""
        self.inbox.append(line)

    def _flush(self):
        for ev in self.ctl.take_events():
            self.on_line(json.dumps(ev, ensure_ascii=False, separators=(",", ":")))

    # ---- 每 5 ms
    def tick(self):
        p, c = self.plant, self.ctl
        while self.inbox:
            line = self.inbox.popleft()
            try:
                c.command(json.loads(line))
            except json.JSONDecodeError:
                pass
        d = p.d
        moving = float(np.abs(d.qvel[p.wv]).max())
        self.vbat = max(18.0, self.vbat - (2e-6 + 4e-6 * moving) / RATE)   # 慢慢掉电（大约几小时掉一格）
        sens = dict(hip=list(np.degrees(d.qpos[p.hq])), wv=list(d.qvel[p.wv]), wt=list(p.tau_w),
                    estop=self.estop_btn, vbat=self.vbat)
        out = c.update(1 / RATE, sens)
        p.hip_ref = np.radians(out["hip_ref"])
        p.hip_vref = np.full(4, math.radians(out["hip_vel"]))
        p.wheel_mode = out["wheel_mode"]
        p.wref = np.array(out["wheel_w"])
        for _ in range(SUB):
            p.step()
        self._flush()

    def run(self, seconds):
        for _ in range(int(round(seconds * RATE))):
            self.tick()

    # ---- 按真实时间在后台跑
    def start(self, view=False):
        self._run = True
        if view:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.plant.m, self.plant.d, show_left_ui=False, show_right_ui=False)
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.viewer.cam.trackbodyid = self.plant.base
            self.viewer.cam.distance, self.viewer.cam.elevation, self.viewer.cam.azimuth = 2.2, -20, 135
        self._thread = threading.Thread(target=self._loop, daemon=True, name="virtual-chassis")
        self._thread.start()

    def _loop(self):
        t0 = time.perf_counter()
        n = 0
        while self._run:
            with self.lock:
                self.tick()
            n += 1
            if self.viewer and n % 7 == 0:                       # 约 30 帧刷新窗口
                if not self.viewer.is_running():
                    self.viewer = None
                else:
                    self.viewer.sync()
            ahead = t0 + n / RATE - time.perf_counter()
            if ahead > 0:
                time.sleep(ahead)
            elif ahead < -0.5:                                    # 电脑太忙跟不上：别一直追
                t0 = time.perf_counter() - n / RATE

    def stop(self):
        self._run = False
        if self._thread:
            self._thread.join(timeout=1)
        if self.viewer:
            self.viewer.close()

    # ---- 测试用
    def pose_xyth(self):
        d = self.plant.d
        return float(d.qpos[0]), float(d.qpos[1]), math.degrees(self.plant.rpy()[2] * math.pi / 180)

    def stance_deg(self):
        return STANCE_DEG
