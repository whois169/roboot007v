"""底盘程序：底盘 ESP32-S3 上跑的逻辑（Python 参考实现，测通以后照着翻成 C++）。

每 5 ms（200 Hz）调一次 update(dt, 传感器)，返回给电机的目标：
  传感器：hip 四个髋角度（度，AS5600）、wv 四个轮子转速（rad/s，GIM4310 回传）、wt 四个轮子扭矩（N·m，GIM4310 回传）、
          estop 急停按钮按下没有、vbat 电池电压
  输出：  hip_ref / hip_vel 四个髋的目标角度（度）和目标角速度（度/秒）—— ESP32 自己用 PWM 做髋的位置环；
          wheel_mode "vel" 速度环（速度 0 就是原地锁住）/ "free" 松开（站起来、趴下的过程中轮子要能自己滚）；wheel_w 四个轮子目标转速（rad/s，CAN 发给 GIM4310）
指令和事件的格式见 protocol.py。只用 math，没有 numpy，方便翻成 C++。

规矩都来自仿真（docs/机械狗007v/仿真报告.md）：
  - 最高 1.1 m/s（110 轮 + GIM4310-10）；横走 0.5、原地转 90°/秒
  - 加速 0.5 m/s²，刹车 1 m/s²，急停 4 m/s²（后轮还压着 2.6 kg，不会往前翻）
  - 站起来、趴下：髋 80°/秒，轮子接地前后 ±8° 放慢到 20°/秒（不然 25 kg 一下子被拽起来，髋冲到 17.8 N·m）
  - 过门槛要慢到 0.3 m/s：大脑看到门槛发 limit
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import ARM, HIP_X, HIP_Z, SKID_H, STANCE_DEG, WHEEL_D, WHEEL_YC  # noqa: E402

from . import protocol  # noqa: E402

FW = "0.1.0"
RATE = 200                                  # 每秒算几次
STATUS_EVERY = 20                           # 每 20 次（0.1 秒）发一次状态

# ---- 几何（C++ 里写成常数）
R = WHEEL_D / 2000
LEG_FX = (1, 1, -1, -1)                     # 前左 前右 后左 后右
S_ROLL = (-1, 1, 1, -1)                     # 麦轮滚子方向（和仿真 export_mujoco.SIGN_S 一样）
YW = tuple(sy * WHEEL_YC / 1000 for sy in (1, -1, 1, -1))
FOLD_DEG = 0.0                              # 趴平（2026-10-18 不再往上折叠）
TOUCH_DEG = math.degrees(math.asin((SKID_H - WHEEL_D / 2 + HIP_Z) / ARM))   # 轮子刚碰到地的摆臂角（约 -3.4°）

# ---- 限制
V_MAX, VY_MAX, WZ_MAX = 1.0, 0.5, math.radians(90)       # Φ97 麦轮 + GIM4310-10 带载约 1.0 m/s
A_ACC, A_BRAKE, A_Y, A_W = 0.5, 1.0, 0.5, math.radians(180)
A_ESTOP, ESTOP_RAMP = 4.0, 0.1                 # 急停减速度 0.1 秒内从 1 加到 4（一下子到 4，车会往前一栽，后轮瞬间失重）
W_MAX = V_MAX / R                           # 轮子最快转速（rad/s）
DRIVE_TIMEOUT, LINK_TIMEOUT = 0.5, 2.0
HIP_SPEED, HIP_ACC, HIP_SLOW, TOUCH_BAND = 30.0, 150.0, 15.0, 8.0   # 5840 减速比 505 ÷ 3 ≈ 30°/秒
HIP_STALL_DEG, HIP_STALL_S = 15.0, 1.0
MOVE_V, MOVE_A = 0.4, 0.4                   # move 默认最高速度、规划用的加减速度
MOVE_VY, MOVE_WZ = 0.3, math.radians(60)
POS_TOL, ANG_TOL = 0.01, math.radians(1.0)
BAT_LOW = 15
# 载重估计：轮子总推力 F = m·a + b（b 是滚动阻力、坡），用加速、刹车时的样本拟合 m；m − 空载时的 m = 背上的重量
M_EMPTY = 14.3                              # 空车估出来的“等效质量”（仿真里标的，含轮子转动惯量）。
# 到货后标定：空车前后各走 1 米，状态里 load 应该是 0；不是 0 就把 M_EMPTY 加上这个数（显示 0 但背 5 kg 只显示 3 时，把 M_EMPTY 减 2）
LOAD_MIN_N, LOAD_MIN_VAR, LOAD_RESET_S = 120, 0.02, 3.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def solve3(A, b):
    """3×3 线性方程（高斯消元）。"""
    M = [list(A[i]) + [b[i]] for i in range(3)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        for r in range(3):
            if r != c:
                f = M[r][c] / M[c][c]
                M[r] = [M[r][k] - f * M[c][k] for k in range(4)]
    return [M[i][3] / M[i][i] for i in range(3)]


def bat_pct(v):
    """6S 锂电（21700）电压 → 电量 %（粗略，按放电曲线分段）。"""
    pts = ((18.0, 0), (20.4, 5), (21.6, 20), (22.2, 40), (22.8, 60), (23.7, 80), (25.2, 100))
    if v <= pts[0][0]:
        return 0
    for (v0, p0), (v1, p1) in zip(pts, pts[1:]):
        if v <= v1:
            return round(p0 + (p1 - p0) * (v - v0) / (v1 - v0))
    return 100


class Controller:
    def __init__(self, hips_deg, now=0.0):
        self.now = now
        self.state = "folded" if hips_deg[0] < TOUCH_DEG else "standing"
        self.hip_ref = list(hips_deg)
        self.hip_vel = 0.0
        self.hip_goal = None                 # 站起来 / 趴下的目标角度
        self.v_cmd = [0.0, 0.0, 0.0]         # 现在给出去的车身速度（已经限过加速度）
        self.v_tgt = [0.0, 0.0, 0.0]         # 想要的车身速度
        self.v_meas = [0.0, 0.0, 0.0]
        self.odo = [0.0, 0.0, 0.0]
        self.drive_t = -9.0                  # 最后一次 drive 指令的时间
        self.rx_t = now                      # 最后一次收到任何一行的时间
        self.link_ok = True
        self.mode = "idle"                   # idle / drive / move / pose / stop
        self.task = None                     # 正在做的 move / pose：dict(id, kind, ...)
        self.limit = None                    # 限速
        self.fault = None
        self.estop_soft = False
        self.estop_t = 0.0
        self.before_estop = self.state
        self.stall_t = 0.0
        self.vbat, self.bat_warned = 0.0, False
        self.out = []                        # 要发给大脑的事件
        self.n = 0
        self.load_reset()
        self.still_t = 0.0
        self.a_f = self.f_f = 0.0
        self._vx_prev = 0.0
        self.out.append(dict(ev="hello", fw=FW, caps=["mecanum", "hip4", "odom", "load", "estop"]))

    # ------------------------------------------------------------ 收指令
    def command(self, c: dict):
        self.rx_t = self.now
        err = protocol.check_cmd(c)
        cid = c.get("id") if isinstance(c, dict) else None
        if err:
            if cid is not None:
                self._ack(cid, False, "BAD")
            return
        k = (set(c) - {"id"}).pop()
        v = c[k]
        if k == "hb":
            return
        if k == "reset":
            self._reset()
            return self._ack(cid, True)
        if k == "estop":
            self._enter_estop("ESTOP_CMD", "大脑发了急停")
            return self._ack(cid, True)
        if k == "limit":
            self.limit = v["v"]
            return self._ack(cid, True)
        if k == "stop":
            self._cancel("STOPPED")
            self.v_tgt = [0.0, 0.0, 0.0]
            if self.mode in ("drive", "move"):
                self.mode = "stop"
            return self._ack(cid, True)
        blocked = self._blocked(k)
        if blocked:
            if cid is not None:
                self._ack(cid, False, blocked)
            elif k == "drive":
                self._warn_once("FOLDED" if blocked == "FOLDED" else "LIMIT", f"现在不能走（{blocked}）")
            return
        if k == "drive":
            self._cancel("STOPPED")
            self.mode = "drive"
            self.drive_t = self.now
            self.v_tgt = self._clamp(list(v))
            return self._ack(cid, True)
        if k == "move":
            self._cancel("STOPPED")
            th0 = self.odo[2]
            dx, dy = v.get("dx", 0.0), v.get("dy", 0.0)
            gx = self.odo[0] + dx * math.cos(th0) - dy * math.sin(th0)
            gy = self.odo[1] + dx * math.sin(th0) + dy * math.cos(th0)
            vm = min(v.get("v", MOVE_V), V_MAX)
            dist = math.hypot(dx, dy)
            ang = abs(math.radians(v.get("dth", 0.0)))
            t_est = dist / max(min(vm, self.limit or 9), 0.05) + ang / MOVE_WZ + 2.0
            self.task = dict(id=cid, kind="move", goal=(gx, gy, th0 + math.radians(v.get("dth", 0.0))), v=vm,
                             deadline=self.now + 2 * t_est + 3)
            self.mode = "move"
            return self._ack(cid, True)
        if k == "pose":
            if (v == "stand" and self.state == "standing") or (v == "fold" and self.state == "folded"):
                self._ack(cid, True)
                return self._done(cid, True)
            self._cancel("STOPPED")
            self.task = dict(id=cid, kind="pose", pose=v, deadline=self.now + 15)
            self.state = "unfolding" if v == "stand" else "folding"
            self.hip_goal = STANCE_DEG if v == "stand" else FOLD_DEG
            self.mode = "pose"
            return self._ack(cid, True)

    def _blocked(self, k):
        if self.fault:
            return "FAULT"
        if self.state == "estop":
            return "ESTOP"
        if self.task and self.task["kind"] == "pose":
            return "BUSY"
        if k in ("drive", "move") and self.state != "standing":
            return "FOLDED"
        if k == "pose" and self._moving():
            return "MOVING"
        return ""

    def _clamp(self, v):
        vmax = min(V_MAX, self.limit or V_MAX)
        v = [max(-vmax, min(vmax, v[0])), max(-min(VY_MAX, vmax), min(min(VY_MAX, vmax), v[1])), max(-WZ_MAX, min(WZ_MAX, v[2]))]
        w = max(abs(x) for x in self.wheel_speeds(v, self.hip_ref))
        if w > W_MAX:                                       # 合起来某个轮子超速：整体按比例降
            v = [x * W_MAX / w for x in v]
        return v

    # ------------------------------------------------------------ 运动学
    @staticmethod
    def wheel_xy(hips):
        return [LEG_FX[i] * (HIP_X / 1000 + ARM / 1000 * math.cos(math.radians(hips[i]))) for i in range(4)]

    def wheel_speeds(self, v, hips):
        """车身速度 → 四个轮子转速：ω = (vx + s·vy + wz·(s·x − y)) / r"""
        xc = self.wheel_xy(hips)
        return [(v[0] + S_ROLL[i] * v[1] + v[2] * (S_ROLL[i] * xc[i] - YW[i])) / R for i in range(4)]

    def body_speed(self, wv, hips):
        """四个轮子转速 → 车身速度（最小二乘，麦轮打滑时取平均）。"""
        xc = self.wheel_xy(hips)
        rows = [(1.0, S_ROLL[i], S_ROLL[i] * xc[i] - YW[i]) for i in range(4)]
        AtA = [[sum(r[a] * r[b] for r in rows) for b in range(3)] for a in range(3)]
        Atb = [sum(rows[i][a] * wv[i] * R for i in range(4)) for a in range(3)]
        return solve3(AtA, Atb)

    def _moving(self):
        return max(abs(self.v_meas[0]), abs(self.v_meas[1]), abs(self.v_meas[2]) * 0.3) > 0.03 or any(abs(x) > 1e-3 for x in self.v_cmd)

    # ------------------------------------------------------------ 每 5 ms
    def update(self, dt, s):
        self.now += dt
        self.n += 1
        hips, wv = s["hip"], s["wv"]
        self.vbat = s.get("vbat", 0.0)
        # 里程计
        self.v_meas = self.body_speed(wv, hips)
        th = self.odo[2]
        self.odo[0] += (self.v_meas[0] * math.cos(th) - self.v_meas[1] * math.sin(th)) * dt
        self.odo[1] += (self.v_meas[0] * math.sin(th) + self.v_meas[1] * math.cos(th)) * dt
        self.odo[2] = wrap(th + self.v_meas[2] * dt)
        # 急停按钮、断线、电量
        if s.get("estop") and self.state != "estop":
            self._enter_estop("ESTOP_BTN", "急停按钮被按下")
        if self.now - self.rx_t > LINK_TIMEOUT:
            if self.link_ok and self._moving():
                self._warn("LINK_LOST", "2 秒没收到大脑的消息，自己停下")
            if self.link_ok:
                self._cancel("STOPPED")
                self.v_tgt = [0.0, 0.0, 0.0]
                if self.mode in ("drive", "move"):
                    self.mode = "stop"
            self.link_ok = False
        else:
            self.link_ok = True
        if self.vbat and bat_pct(self.vbat) < BAT_LOW and not self.bat_warned:
            self.bat_warned = True
            self._warn("BAT_LOW", f"电量 {bat_pct(self.vbat)}%")
        # 各模式
        if self.mode == "drive" and self.now - self.drive_t > DRIVE_TIMEOUT:
            self.v_tgt = [0.0, 0.0, 0.0]                       # drive 断了：慢慢停
            self.mode = "stop"
        if self.mode == "move":
            self._move_step()
        if self.task and self.now > self.task["deadline"]:
            self._cancel("TIMEOUT")
            self.v_tgt = [0.0, 0.0, 0.0]
            self.mode = "stop"
        if self.mode == "pose":
            self._pose_step(dt, hips)
        self._shape(dt)
        self._stall_check(hips, dt)
        self._load_step(s, dt)
        if self.mode == "stop" and not self._moving():
            self.mode = "idle"
        out = dict(hip_ref=list(self.hip_ref), hip_vel=self.hip_vel,
                   wheel_mode="free" if self.state in ("unfolding", "folding") else "vel",
                   wheel_w=self.wheel_speeds(self.v_cmd, hips))
        if self.n % STATUS_EVERY == 0:
            self.out.append(self.status(hips))
        return out

    def _shape(self, dt):
        """车身速度往目标靠，限加速度（同向加速 0.5、减速 1.0；急停 4）。"""
        tgt = self.v_tgt if self.state == "standing" else [0.0, 0.0, 0.0]
        tgt = self._clamp(tgt) if self.state == "standing" else tgt
        for i, (acc, dec) in enumerate(((A_ACC, A_BRAKE), (A_Y, A_Y * 2), (A_W, A_W * 2))):
            if self.state == "estop":
                k = min(1.0, (self.now - self.estop_t) / ESTOP_RAMP)
                acc = dec = (A_BRAKE + (A_ESTOP - A_BRAKE) * k) if i < 2 else A_W * 4
            cur, t = self.v_cmd[i], tgt[i]
            a = acc if (t * cur >= 0 and abs(t) > abs(cur)) else dec
            step = a * dt
            self.v_cmd[i] = cur + max(-step, min(step, t - cur))

    def _move_step(self):
        """move：按剩下的距离算速度（快到了提前减速），在“现在的车身方向”上分解。"""
        t = self.task
        gx, gy, gth = t["goal"]
        x, y, th = self.odo
        ex_w, ey_w = gx - x, gy - y
        ex = ex_w * math.cos(th) + ey_w * math.sin(th)
        ey = -ex_w * math.sin(th) + ey_w * math.cos(th)
        eth = wrap(gth - th)
        vlim = min(t["v"], self.limit or V_MAX)

        def prof(e, vmax, a, tol):
            if abs(e) < tol * 0.5:
                return 0.0
            return math.copysign(min(vmax, math.sqrt(2 * a * abs(e)) * 0.9, 4 * abs(e) + 0.03), e)
        self.v_tgt = [prof(ex, vlim, MOVE_A, POS_TOL), prof(ey, min(vlim, MOVE_VY), MOVE_A, POS_TOL),
                      prof(eth, MOVE_WZ, math.radians(90), ANG_TOL)]
        if abs(ex) < POS_TOL and abs(ey) < POS_TOL and abs(eth) < ANG_TOL and not self._moving():
            self.v_tgt = [0.0, 0.0, 0.0]
            self.mode = "idle"
            self._finish(True)

    def _pose_step(self, dt, hips):
        """站起来 / 趴下：四个髋一起转，限速 80°/秒、加速度 300°/秒²，轮子接地前后 ±8° 放慢到 20°/秒。"""
        goal, cur = self.hip_goal, self.hip_ref[0]
        d = goal - cur
        if abs(d) < 0.05:
            self.hip_ref = [goal] * 4
            self.hip_vel = 0.0
            if max(abs(h - goal) for h in hips) > 3.0:                # 指令走完了，但髋实际还没到：等它（卡住了由 _stall_check 报故障）
                return
            self.state = "standing" if goal == STANCE_DEG else "folded"
            self.mode = "idle"
            self._finish(True)
            return
        sgn = 1 if d > 0 else -1
        lo, hi = TOUCH_DEG - TOUCH_BAND, TOUCH_DEG + TOUCH_BAND
        vmax = HIP_SLOW if lo <= cur <= hi else HIP_SPEED
        entry = lo if sgn > 0 else hi                          # 还没进放慢区：提前减到 20°/秒
        if (entry - cur) * sgn > 0:
            vmax = min(vmax, math.sqrt(HIP_SLOW ** 2 + 2 * HIP_ACC * (entry - cur) * sgn))
        vmax = min(vmax, math.sqrt(2 * HIP_ACC * abs(d)))     # 快到了：减速
        v = abs(self.hip_vel)
        v = min(v + HIP_ACC * dt, vmax) if v < vmax else max(v - HIP_ACC * dt, vmax)
        v = max(v, 2.0)
        step = min(abs(d), v * dt)
        self.hip_vel = sgn * v
        self.hip_ref = [cur + sgn * step] * 4

    def _stall_check(self, hips, dt):
        err = max(abs(self.hip_ref[i] - hips[i]) for i in range(4))
        self.stall_t = self.stall_t + dt if err > HIP_STALL_DEG else 0.0
        if self.stall_t > HIP_STALL_S and not self.fault:
            self.fault = "HIP_STALL"
            self.state = "fault"
            self._cancel("FAULT")
            self.hip_ref = list(hips)                          # 原地不动（蜗轮自锁）
            self.hip_vel = 0.0
            self.v_tgt = [0.0, 0.0, 0.0]
            self.mode = "idle"
            self.out.append(dict(ev="fault", code="HIP_STALL", text=f"髋关节跟不上（差 {err:.0f}°），停下"))

    # ------------------------------------------------------------ 载重估计
    def load_reset(self):
        self.ls = [0.0] * 5                                    # n, Σa, Σa², ΣF, ΣaF
        self.load = None

    def _load_step(self, s, dt):
        if "wt" not in s or self.state != "standing":
            return
        vx = self.v_cmd[0]                                     # 用给出去的速度求加速度：测出来的速度有滚子交接的抖动，求导后噪声大，会把质量估小
        a = (vx - self._vx_prev) / dt
        self._vx_prev = vx
        k = min(1.0, dt / 0.05)                                # 加速度和推力用同一个 50 ms 平滑，相位对齐
        self.a_f += (a - self.a_f) * k
        self.f_f += (sum(s["wt"]) / R - self.f_f) * k
        still = not self._moving()
        self.still_t = self.still_t + dt if still else 0.0
        if self.still_t > LOAD_RESET_S and self.load is not None:
            self.load_reset()                                  # 停下来一阵：可能装卸了货，重新估
        straight = abs(self.v_cmd[1]) < 0.02 and abs(self.v_cmd[2]) < 0.05
        if not straight or abs(self.a_f) < 0.15:
            return
        F = self.f_f
        n, sa, saa, sf, saf = self.ls
        self.ls = [n + 1, sa + self.a_f, saa + self.a_f ** 2, sf + F, saf + self.a_f * F]
        n, sa, saa, sf, saf = self.ls
        var = saa / n - (sa / n) ** 2
        if n >= LOAD_MIN_N and var > LOAD_MIN_VAR:
            m = (saf / n - sa / n * sf / n) / var
            self.load = max(0.0, round((m - M_EMPTY) * 2) / 2)

    # ------------------------------------------------------------ 急停、故障、收尾
    def _enter_estop(self, code, text):
        if self.state != "estop":
            self.before_estop = self.state if self.state in ("standing", "folded") else self._rest_state()
        if self.state != "estop":
            self.estop_t = self.now
        self.state = "estop"
        self.estop_soft = code == "ESTOP_CMD"
        self._cancel("ESTOP")
        self.v_tgt = [0.0, 0.0, 0.0]
        self.hip_vel = 0.0
        self.mode = "idle"
        self.out.append(dict(ev="fault", code=code, text=text))

    def _reset(self):
        if self.state == "estop":
            self.state = self.before_estop
        if self.fault:
            self.fault = None
            self.state = self._rest_state()
        self.stall_t = 0.0

    def _rest_state(self):
        return "standing" if abs(self.hip_ref[0] - STANCE_DEG) < 3 else "folded"

    def _cancel(self, why):
        if self.task:
            if self.task["kind"] == "pose":                    # 停在半路（蜗轮自锁）；只有在站姿 ±3° 以内才算站着，不然要再发一次 stand
                self.hip_vel = 0.0
                if self.state not in ("estop", "fault"):
                    self.state = self._rest_state()
            self._finish(False, why)

    def _finish(self, ok, why=""):
        t, self.task = self.task, None
        if t and t["id"] is not None:
            self._done(t["id"], ok, why)

    def _ack(self, cid, ok, why=""):
        if cid is not None:
            self.out.append(dict(ev="ack", id=cid, ok=ok, **({} if ok else {"why": why})))

    def _done(self, cid, ok, why=""):
        self.out.append(dict(ev="done", id=cid, ok=ok, **({} if ok else {"why": why})))

    def _warn(self, code, text):
        self.out.append(dict(ev="warn", code=code, text=text))

    def _warn_once(self, code, text):
        last = getattr(self, "_warn_t", {})
        if self.now - last.get(code, -9) > 1.0:
            last[code] = self.now
            self._warn_t = last
            self._warn(code, text)

    def status(self, hips):
        return dict(ev="chassis", st=self.state, v=[round(x, 3) for x in self.v_meas],
                    odo=[round(self.odo[0], 3), round(self.odo[1], 3), round(math.degrees(self.odo[2]), 1)],
                    hip=[round(h, 1) for h in hips], bat=bat_pct(self.vbat) if self.vbat else None,
                    vbat=round(self.vbat, 2) if self.vbat else None, lim=self.limit, load=self.load,
                    busy=self.task["id"] if self.task else None)

    def take_events(self):
        out, self.out = self.out, []
        return out
