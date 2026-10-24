"""底盘程序回归测试：在 MuJoCo 仿真的车上把每条规矩跑一遍（约 1–2 分钟）。

用法：python 007v/chassis/test_chassis.py
改了 controller.py / protocol.py 必须跑，全过才算数。
"""
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from chassis import protocol  # noqa: E402
from chassis.controller import FOLD_DEG, STANCE_DEG, V_MAX, Controller  # noqa: E402
from config import ARM, HIP_X, WHEEL_D  # noqa: E402
from chassis.virtual import VirtualChassis  # noqa: E402
import run_sim as S  # noqa: E402

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(ok)
    print(f"  {'通过' if ok else '失败'}  {name}" + (f"（{detail}）" if detail else ""))


def section(t):
    print(f"\n== {t}")


class Bench:
    """一台虚拟车 + 记录它发出来的事件；跑的时候自动每 0.5 秒发心跳（像大脑一样）。"""

    def __init__(self, payload=10.0, folded=False, hb=True):
        self.ev = []
        self.vc = VirtualChassis(payload=payload, folded=folded, on_line=lambda s: self.ev.append(json.loads(s)))
        self.hb = hb
        self.hip_peak = 0.0
        self.trace = []                              # (t, 真实 x, 真实 vx, 后轮载荷)

    def cmd(self, c):
        self.vc.send_line(json.dumps(c))

    def run(self, seconds, trace=False):
        p = self.vc.plant
        for i in range(int(round(seconds * 200))):
            if self.hb and i % 100 == 0:
                self.cmd({"hb": 1})
            self.vc.tick()
            self.hip_peak = max(self.hip_peak, float(np.abs(p.d.ctrl[p.a_hip]).max()))
            if trace:
                vx = p.body_vel()[0]
                self.trace.append((self.vc.ctl.now, float(p.d.qpos[0]), float(vx), float(p.wheel_loads()[2:].sum() / S.G)))

    def until(self, pred, timeout):
        t = 0.0
        while t < timeout and not pred():
            self.run(0.05)
            t += 0.05
        return pred()

    def events(self, kind, **kw):
        return [e for e in self.ev if e["ev"] == kind and all(e.get(k) == v for k, v in kw.items())]

    def status(self):
        st = self.events("chassis")
        return st[-1] if st else None

    def truth(self):
        p = self.vc.plant
        return float(p.d.qpos[0]), float(p.d.qpos[1]), float(p.rpy()[2])

    def speed(self):
        return self.vc.plant.body_vel()


def t_protocol():
    section("协议：指令和事件的检查")
    good = [{"drive": [0.4, 0, 0]}, {"move": {"dx": 1.0, "dth": 90}, "id": 3}, {"pose": "stand", "id": 1}, {"stop": True},
            {"estop": True}, {"reset": True}, {"limit": {"v": 0.3}}, {"limit": {"v": None}}, {"hb": 1},
            {"calib": "step", "id": 2}, {"light": {"front": 60}}, {"light": {"front": 0, "dim": 30}}]
    bad = [{"drive": [2, 0, 0]}, {"drive": [0.1, 0]}, {"move": {"dx": 9}}, {"move": {"dz": 1}}, {"pose": "sit"},
           {"stop": True, "drive": [0, 0, 0]}, {"fly": True}, {"limit": {"v": 3}}, {"pose": "stand", "id": -1}, [1],
           {"calib": "hip"}, {"light": {"front": 120}}, {"light": {"rgb": 1}}, {"light": {}}]
    check("合法指令都通过", all(protocol.check_cmd(c) == "" for c in good))
    check("不合法指令都被拦下", all(protocol.check_cmd(c) for c in bad))
    c = Controller([STANCE_DEG] * 4)
    c.command({"move": {"dx": 99}, "id": 4})
    check("不合法的带 id 指令回 ack ok=false why=BAD", any(e.get("ev") == "ack" and e.get("why") == "BAD" for e in c.out))
    evs = [{"ev": "hello", "fw": "0.1.0"}, c.status([55] * 4), {"ev": "ack", "id": 1, "ok": True},
           {"ev": "done", "id": 1, "ok": False, "why": "TIMEOUT"}, {"ev": "warn", "code": "LINK_LOST"}, {"ev": "fault", "code": "HIP_STALL"},
           {"ev": "warn", "code": "CLIFF", "where": "front"}, {"ev": "warn", "code": "STEP", "where": "front", "h": 15},
           {"ev": "done", "id": 2, "ok": False, "why": "CLIFF"}]
    check("底盘发出来的事件格式都对", all(protocol.check_event(e) == "" for e in evs), str([protocol.check_event(e) for e in evs]))


def t_stand_fold():
    section("站起来、趴下")
    b = Bench(folded=True)
    check("开机先发 hello", bool(b.events("hello")))
    b.cmd({"drive": [0.3, 0, 0]})
    b.cmd({"move": {"dx": 1}, "id": 1})
    b.run(0.3)
    check("趴着时不能走：drive 提醒 FOLDED，move 回 ack ok=false", bool(b.events("warn", code="FOLDED"))
          and bool(b.events("ack", id=1, ok=False, why="FOLDED")))
    b.cmd({"pose": "stand", "id": 2})
    ok = b.until(lambda: b.events("done", id=2), 6)
    t_up = b.vc.ctl.now
    b.run(0.2)
    check("站起来：做完回 done，状态 standing", ok and b.events("done", id=2)[0]["ok"] and b.status()["st"] == "standing")
    b.run(1.0)
    z = b.vc.plant.d.qpos[2] * 1000
    import math as _m
    from config import ARM, HIP_Z, WHEEL_D
    z_exp = WHEEL_D / 2 - HIP_Z + ARM * _m.sin(_m.radians(STANCE_DEG))
    check(f"站起来后机身离地约 {z_exp:.0f} mm", abs(z - z_exp) < 6, f"{z:.0f} mm，用时约 {t_up:.1f} 秒")
    check("站起来时髋扭矩没超额定 20.7 N·m（接地前后放慢起作用）", b.hip_peak < 20.7, f"最大 {b.hip_peak:.1f}")
    for _ in range(3):
        b.cmd({"drive": [0.5, 0, 0]})
        b.run(0.3)
    b.cmd({"pose": "fold", "id": 3})
    b.run(0.1)
    check("走着的时候不能趴下：ack ok=false why=MOVING", bool(b.events("ack", id=3, ok=False, why="MOVING")))
    b.cmd({"stop": True})
    b.run(1.5)
    b.hip_peak = 0.0
    b.cmd({"pose": "fold", "id": 4})
    ok = b.until(lambda: b.events("done", id=4), 6)
    b.run(0.5)
    check("停下来以后趴下：done，状态 folded，坐在滑撬上", ok and b.status()["st"] == "folded" and b.vc.plant.d.qpos[2] < 0.012,
          f"机身离地 {b.vc.plant.d.qpos[2] * 1000:.0f} mm，髋最大 {b.hip_peak:.1f} N·m")


def t_move():
    section("走一段（move）：走直线、原地转、横走，看真实走了多少")
    b = Bench()
    b.run(0.5)
    x0, y0, _ = b.truth()
    b.cmd({"move": {"dx": 1.0}, "id": 1})
    ok = b.until(lambda: b.events("done", id=1), 8)
    x, y, yaw = b.truth()
    check("往前 1 米：真实走了 1 米 ±2 cm，没跑偏", ok and abs(x - x0 - 1) < 0.02 and abs(y - y0) < 0.02,
          f"走了 {x - x0:.3f} m，偏 {y - y0:+.3f} m，用时 {b.vc.ctl.now - 0.5:.1f} 秒")
    load = b.status()["load"]
    check("走一趟就估出背上的重量（实际 10 kg，误差 ±1.5）", load is not None and abs(load - 10) <= 1.5, f"估 {load} kg")
    b.cmd({"move": {"dth": 90}, "id": 2})
    ok = b.until(lambda: b.events("done", id=2), 8)
    _, _, yaw = b.truth()
    check("原地左转 90°：真实转了 90° ±3°", ok and abs(yaw - 90) < 3, f"{yaw:.1f}°")
    x1, y1, _ = b.truth()
    b.cmd({"move": {"dy": 0.5}, "id": 3})
    ok = b.until(lambda: b.events("done", id=3), 8)
    x2, y2, _ = b.truth()
    moved = math.hypot(x2 - x1, y2 - y1)
    check("往左横走 0.5 米（麦轮）：±3 cm", ok and abs(moved - 0.5) < 0.03, f"{moved:.3f} m")
    b.cmd({"move": {"dx": 2.0, "dth": -90}, "id": 4})
    b.run(1.0)
    b.cmd({"stop": True, "id": 5})
    b.run(1.5)
    d = b.events("done", id=4)
    check("走到一半发 stop：move 回 done ok=false why=STOPPED，车停住", bool(d) and d[0]["ok"] is False and d[0]["why"] == "STOPPED"
          and abs(b.speed()[0]) < 0.02)


def t_drive():
    section("按速度走（drive）：最高速、限速、断了自己停")
    b = Bench()
    b.run(0.3)
    for _ in range(10):
        b.cmd({"drive": [V_MAX, 0, 0]})
        b.run(0.4)
    v = b.speed()[0]
    check(f"最高速 {V_MAX} m/s 跟得上", v > V_MAX * 0.95, f"{v:.2f} m/s")
    b.cmd({"limit": {"v": 0.3}})
    for _ in range(8):
        b.cmd({"drive": [1.1, 0, 0]})
        b.run(0.4)
    v = b.speed()[0]
    check("限速 0.3（过门槛）：再怎么要也只走 0.3", v < 0.32, f"{v:.2f} m/s")
    b.cmd({"limit": {"v": None}})
    b.cmd({"drive": [0.6, 0, 0]})
    b.run(1.5)
    t0 = b.vc.ctl.now
    b.run(2.0)                                                 # 不再发 drive（心跳还在）
    v = b.speed()[0]
    check("drive 断了 0.5 秒：自己慢慢停下", abs(v) < 0.02, f"2 秒后 {v:.2f} m/s")
    nb = Bench(hb=False)
    nb.run(0.2)
    for _ in range(5):
        nb.cmd({"drive": [0.6, 0, 0]})
        nb.run(0.3)
    nb.vc.ctl.drive_t = 1e9                                    # 只测断线：drive 本身不超时
    nb.run(4.0)
    check("和大脑断线 2 秒：提醒 LINK_LOST 并停下", bool(nb.events("warn", code="LINK_LOST")) and abs(nb.speed()[0]) < 0.02,
          f"{nb.speed()[0]:.2f} m/s")


def t_estop():
    section("急停")
    b = Bench()
    b.run(0.3)
    for _ in range(8):
        b.cmd({"drive": [1.0, 0, 0]})
        b.run(0.4)
    b.trace.clear()
    x0 = b.truth()[0]
    b.cmd({"estop": True})
    b.run(1.5, trace=True)
    x1 = b.truth()[0]
    tr = np.array(b.trace)
    rear = np.convolve(tr[:, 3], np.ones(10) / 10, mode="valid").min()     # 50 ms 平均（和仿真报告同一个口径，去掉滚子交接的瞬间跳动）
    check("软急停：1 m/s 刹停在 25 cm 以内，回 fault ESTOP_CMD", x1 - x0 < 0.25 and bool(b.events("fault", code="ESTOP_CMD")),
          f"刹车距离 {(x1 - x0) * 100:.0f} cm")
    check("急停时后轮没翘起来（还压着 > 0.5 kg）", rear > 0.5, f"最少 {rear:.1f} kg")
    b.cmd({"drive": [0.5, 0, 0]})
    b.cmd({"move": {"dx": 1}, "id": 9})
    b.run(0.5)
    check("急停后不能走：move 回 ack ok=false why=ESTOP", bool(b.events("ack", id=9, ok=False, why="ESTOP")) and abs(b.speed()[0]) < 0.02)
    b.cmd({"reset": True})
    b.cmd({"move": {"dx": 0.3}, "id": 10})
    ok = b.until(lambda: b.events("done", id=10), 5)
    check("reset 以后能接着走", ok and b.events("done", id=10)[0]["ok"])
    b.vc.estop_btn = True
    b.run(0.2)
    b.cmd({"reset": True})
    b.run(0.2)
    st1 = b.status()["st"]
    b.vc.estop_btn = False
    b.cmd({"reset": True})
    b.run(0.2)
    check("急停按钮：按下进 ESTOP_BTN；按着时 reset 无效；松开再 reset 恢复",
          bool(b.events("fault", code="ESTOP_BTN")) and st1 == "estop" and b.status()["st"] == "standing")


def t_stall():
    section("髋卡住（故障）：用假传感器测")
    c = Controller([FOLD_DEG - 5] * 4)
    c.command({"pose": "stand", "id": 1})
    for _ in range(600):                                       # 3 秒，髋一直不动
        c.update(0.005, dict(hip=[FOLD_DEG - 5] * 4, wv=[0.0] * 4, estop=False, vbat=24.0))
    check("髋跟不上超过 1 秒：报 HIP_STALL，站起来那条回 done ok=false why=FAULT，原地不动",
          any(e.get("code") == "HIP_STALL" for e in c.out) and any(e.get("ev") == "done" and e.get("why") == "FAULT" for e in c.out)
          and c.state == "fault")
    c.command({"move": {"dx": 1}, "id": 2})
    check("故障时不能动：ack why=FAULT", any(e.get("ev") == "ack" and e.get("id") == 2 and e.get("why") == "FAULT" for e in c.out))


def t_load_empty():
    section("载重估计：空车")
    b = Bench(payload=0.0)
    b.run(0.3)
    b.cmd({"move": {"dx": 1.0}, "id": 1})
    b.until(lambda: b.events("done", id=1), 8)
    load = b.status()["load"]
    check("空车估出来接近 0（±1.5 kg）", load is not None and load <= 1.5, f"估 {load} kg")


def t_step():
    section("台阶测距：地面没了自己刹车、有坎限速、测距板坏了限速")
    b = Bench(payload=10.0)
    b.run(0.5)
    x0 = b.truth()[0]
    edge = x0 + 1.2                                             # 前面 1.2 米处地面没了（楼梯往下）
    b.vc.edge = edge
    for _ in range(20):
        b.cmd({"drive": [V_MAX, 0, 0]})                         # 全速冲过去
        b.run(0.1)
    b.run(1.0)
    wheel_x = b.truth()[0] + (HIP_X + ARM * math.cos(math.radians(STANCE_DEG)) + WHEEL_D / 2) / 1000
    st = b.status()
    check("全速冲向楼梯边：自己刹住，前轮没过边", wheel_x < edge and abs(b.speed()[0]) < 0.02,
          f"前轮最前沿离边 {(edge - wheel_x) * 1000:.0f} mm")
    check("报 CLIFF、锁住前面", bool(b.events("warn", code="CLIFF")) and st["cliff"] == "front")
    xs = b.truth()[0]
    for _ in range(5):
        b.cmd({"drive": [0.3, 0, 0]})
        b.run(0.2)
    check("锁住以后再往前开：不动", b.truth()[0] - xs < 0.005, f"{(b.truth()[0] - xs) * 1000:.1f} mm")
    b.cmd({"move": {"dx": -0.4, "v": 0.3}, "id": 21})
    b.until(lambda: b.events("done", id=21), 8)
    b.run(0.5)
    check("往后退开：允许，退开后自动解锁", bool(b.events("done", id=21, ok=True)) and b.status()["cliff"] is None)
    b.cmd({"move": {"dx": 1.0, "v": 0.5}, "id": 22})
    b.until(lambda: b.events("done", id=22), 8)
    check("用 move 往边上走：半路停下，done 的原因是 CLIFF", bool(b.events("done", id=22, ok=False, why="CLIFF")))
    # 门槛
    c = Bench(payload=10.0)
    c.run(0.5)
    x0 = c.truth()[0]
    c.vc.bump = (x0 + 0.6, x0 + 0.66, 0.015)                    # 前面 0.6 米处一道 15 mm 的门槛
    for _ in range(12):
        c.cmd({"drive": [0.8, 0, 0]})
        c.run(0.1)
    warn = c.events("warn", code="STEP")
    check("前面有 15 mm 门槛：报 STEP、自己限速 0.3", bool(warn) and c.status()["lim"] == 0.3 and c.speed()[0] < 0.35,
          f"估的高度 {warn[0]['h'] if warn else '-'} mm，速度 {c.speed()[0]:.2f}")
    # 测距板坏了
    d = Bench(payload=10.0)
    d.run(0.3)
    d.vc.broken = 2
    for _ in range(10):
        d.cmd({"drive": [0.8, 0, 0]})
        d.run(0.2)
    check("后左测距板没回应：报 STEP_SENSOR、限速 0.2", bool(d.events("warn", code="STEP_SENSOR", where="rear_left"))
          and d.speed()[0] < 0.22, f"{d.speed()[0]:.2f} m/s")
    for _ in range(5):
        d.cmd({"drive": [0, 0.5, 0]})
        d.run(0.2)
    check("横着走最快 0.2（两边没有台阶测距）", abs(d.speed()[1]) < 0.21, f"{d.speed()[1]:.2f} m/s")
    d.vc.broken = None
    d.run(1.5)                                                  # 等车停稳
    d.cmd({"calib": "step", "id": 30})
    d.run(0.1)
    check("停稳时标定台阶测距：成功", bool(d.events("ack", id=30, ok=True)))


def t_light():
    section("机身的灯：按状态自己亮")
    b = Bench()
    b.run(0.3)
    L = b.vc.ctl.lights()
    check("平时：腰线青色常亮、尾灯暗红、电机光环亮", L["waist"] == ["solid", "cyan"] and L["tail"] == ["solid", 30] and L["ring"])
    for _ in range(6):
        b.cmd({"drive": [0.6, 0, 0]})
        b.run(0.2)
    b.cmd({"stop": True})
    b.run(0.1)
    check("刹车：尾灯全亮", b.vc.ctl.lights()["tail"] == ["solid", 100])
    b.run(1.5)
    for _ in range(4):
        b.cmd({"drive": [-0.3, 0, 0]})
        b.run(0.2)
    check("倒车：尾灯闪", b.vc.ctl.lights()["tail"][0] == "blink")
    b.cmd({"light": {"front": 60, "dim": 40}, "id": 5})
    b.run(0.2)
    st = b.status()
    check("前白灯、总亮度听大脑的", st["light"] == {"front": 60, "dim": 40})
    b.vc.ibat = 3.0
    b.run(0.2)
    check("插上充电器：状态 chg、腰线绿色呼吸", b.status()["chg"] and b.vc.ctl.lights()["waist"] == ["breath", "green"])
    b.cmd({"estop": True})
    b.run(0.2)
    L = b.vc.ctl.lights()
    check("急停：腰线、尾灯红色快闪，电机光环灭", L["waist"] == ["flash", "red"] and L["tail"][0] == "flash" and not L["ring"])


def main():
    t0 = time.time()
    for t in (t_protocol, t_stand_fold, t_move, t_drive, t_estop, t_stall, t_load_empty, t_step, t_light):
        t()
    n = len(RESULTS)
    print(f"\n共 {n} 项，通过 {sum(RESULTS)} 项，失败 {n - sum(RESULTS)} 项，用时 {time.time() - t0:.0f} 秒")
    sys.exit(0 if all(RESULTS) else 1)


if __name__ == "__main__":
    main()
