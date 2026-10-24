"""C++ 底盘程序 和 Python 底盘程序 逐步对比（电脑上跑，不用板子）。

1. 在 MuJoCo 仿真里让 Python 底盘程序跑一长串动作（站起来、走、转、限速、横走、楼梯边、门槛、测距板坏、灯、充电、
   急停、断线、轮子故障、髋标定、趴下），把每 5 ms 的传感器读数、每条指令、Python 算出来的输出和事件都记下来；
2. 用电脑上的 C++ 编译器（zig）编译 test/replay.cpp + src/controller.cpp + src/protocol.cpp；
3. 把同样的传感器和指令喂给 C++ 版，逐步比较输出（髋目标、轮子转速、灯）和发给大脑的每一条事件。

用法：python 007v/firmware/chassis/test/compare.py
（要先编译过一次固件，好让 PlatformIO 把 ArduinoJson 下载到 .pio/libdeps/）
"""
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.dirname(HERE)
ROOT = os.path.dirname(os.path.dirname(FW))                # 007v/
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "sim"))
from chassis import controller as CM  # noqa: E402
from chassis.virtual import VirtualChassis  # noqa: E402

OUT = os.path.join(FW, ".pio", "compare")
os.makedirs(OUT, exist_ok=True)


# ================================================================ 1. 录 Python
class Rec:
    def __init__(self):
        self.inp, self.exp = [], []

    def events(self, ctl, n0):
        for e in ctl.out[n0:]:
            self.exp.append("E " + json.dumps(e, ensure_ascii=False))


def record():
    rec = Rec()
    orig_cmd, orig_upd, orig_fault = CM.Controller.command, CM.Controller.update, CM.Controller.external_fault

    def command(self, c):
        n0 = len(self.out)
        orig_cmd(self, c)
        rec.inp.append("C " + json.dumps(c))
        rec.exp.append("C")
        rec.events(self, n0)

    def update(self, dt, s):
        n0 = len(self.out)
        o = orig_upd(self, dt, s)
        st = s.get("step")
        f = lambda x: f"{float(x):.9g}"
        rec.inp.append("U " + " ".join([f(dt)] + [f(x) for x in s["hip"]] + [f(x) for x in s["wv"]]
                                        + [f(x) for x in s.get("wt", [0] * 4)]
                                        + ["1" if "wt" in s else "0", "1" if s.get("estop") else "0", f(s.get("vbat", 0)), f(s.get("ibat", 0)),
                                           "1" if st is not None else "0"]
                                        + [f(0 if (st is None or x is None) else x) for x in (st or [0] * 4)]
                                        + ["0" if (st is not None and x is None) else "1" for x in (st or [0] * 4)]))
        L = o["light"]
        rec.exp.append("O " + " ".join([f"{x:.6f}" for x in o["hip_ref"]] + [f"{o['hip_vel']:.6f}", "1" if o["wheel_mode"] == "free" else "0"]
                                        + [f"{x:.6f}" for x in o["wheel_w"]]
                                        + [L["waist"][0], L["waist"][1], L["tail"][0], str(L["tail"][1]), "1" if L["ring"] else "0",
                                           str(L["front"]), str(L["dim"])]))
        rec.events(self, n0)
        return o

    def external_fault(self, code, text):
        n0 = len(self.out)
        orig_fault(self, code, text)
        rec.inp.append(f"F {code} {text}")
        rec.exp.append("C")
        rec.events(self, n0)

    CM.Controller.command, CM.Controller.update, CM.Controller.external_fault = command, update, external_fault
    try:
        vc = VirtualChassis(payload=8.0, folded=True)
        rec.inp.insert(0, "I " + " ".join(f"{x:.9g}" for x in vc.ctl.hip_ref))
        hello = CM.Controller(list(vc.ctl.hip_ref)).out           # 开机的 hello（虚拟底盘建的时候已经发走了，这里再生成一份同样的）
        rec.exp[0:0] = ["E " + json.dumps(e, ensure_ascii=False) for e in hello]
        script(vc)
    finally:
        CM.Controller.command, CM.Controller.update, CM.Controller.external_fault = orig_cmd, orig_upd, orig_fault
    return rec


def script(vc):
    """一长串动作（和底盘测试里的场景一样，连成一次）。"""
    t = [0]

    def send(c):
        vc.send_line(json.dumps(c))

    def run(sec, hb=True, every=None):
        for i in range(int(round(sec * 200))):
            if hb and t[0] % 100 == 0:
                send({"hb": 1})
            if every and t[0] % 40 == 0:
                send(every)
            vc.tick()
            t[0] += 1

    p = vc.plant
    run(0.3)
    send({"calib": "hip", "id": 1})                            # 趴着标定髋零点
    send({"drive": [0.3, 0, 0]})                               # 趴着想走：FOLDED
    send({"pose": "stand", "id": 2})
    run(5.0)
    send({"light": {"front": 60, "dim": 80}, "id": 3})
    run(1.0, every={"drive": [0.8, 0, 0]})                     # 往前开
    send({"stop": True})
    run(1.0)
    send({"move": {"dx": 0.5, "dth": 30}, "id": 4})            # 走一段带转弯
    run(4.0)
    send({"limit": {"v": 0.3}, "id": 5})
    run(1.0, every={"drive": [1.0, 0.5, 0.2]})                 # 限速、横走
    send({"limit": {"v": None}})
    run(1.5)
    x0 = float(p.d.qpos[0])                                    # 楼梯边：全速冲过去
    vc.edge = x0 + 1.0
    run(2.0, every={"drive": [1.0, 0, 0]})
    send({"move": {"dx": -0.4, "v": 0.3}, "id": 6})            # 退开
    run(3.0)
    vc.edge = None
    vc.bump = (float(p.d.qpos[0]) + 0.5, float(p.d.qpos[0]) + 0.56, 0.015)   # 门槛
    run(1.5, every={"drive": [0.8, 0, 0]})
    vc.bump = None
    send({"stop": True})
    run(1.5)
    vc.broken = 3                                              # 测距板坏了
    run(1.0, every={"drive": [-0.5, 0, 0]})
    vc.broken = None
    send({"stop": True})
    run(1.5)
    send({"calib": "step", "id": 7})
    vc.ibat = 3.0                                              # 充电
    run(0.5)
    vc.ibat = -2.0
    send({"estop": True, "id": 8})                             # 软急停、复位
    run(0.5)
    send({"reset": True, "id": 9})
    run(0.3)
    vc.estop_btn = True                                        # 急停按钮
    run(0.5)
    vc.estop_btn = False
    send({"reset": True})
    run(0.3)
    send({"move": {"dx": 0.3}, "id": 10})                      # 走着时轮子没回应
    run(0.5)
    vc.ctl.external_fault("WHEEL_LOST", "前左轮子电机 100 ms 没回应")
    run(1.0)
    send({"reset": True})
    send({"drive": [2, 0, 0], "id": 11})                       # 不合法的指令
    send({"move": {"dx": 0.2, "v": 0.5}, "id": 12})
    run(0.5)
    run(3.0, hb=False)                                         # 断线 2 秒
    send({"pose": "fold", "id": 13})                           # 趴下
    run(6.0)


# ================================================================ 2. 编译 C++
def build():
    aj = os.path.join(FW, ".pio", "libdeps", "chassis", "ArduinoJson", "src")
    if not os.path.isdir(aj):
        sys.exit("找不到 ArduinoJson：先编译一次固件（python -m platformio run -d 007v/firmware/chassis）")
    zc = os.path.join(os.path.expanduser("~"), "zigcache")          # zig 在带中文的路径里会出错：缓存和输出都放英文目录
    env = dict(os.environ, ZIG_GLOBAL_CACHE_DIR=os.path.join(zc, "g"), ZIG_LOCAL_CACHE_DIR=os.path.join(zc, "l"))
    os.makedirs(zc, exist_ok=True)
    exe = os.path.join(zc, "replay.exe")
    cmd = [sys.executable, "-m", "ziglang", "c++", "-std=c++17", "-O2", "-I", aj, "-I", os.path.join(FW, "src"),
           os.path.join(HERE, "replay.cpp"), os.path.join(FW, "src", "controller.cpp"), os.path.join(FW, "src", "protocol.cpp"),
           "-o", exe]
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if r.returncode:
        sys.exit("C++ 编译失败：\n" + r.stderr[-3000:])
    return exe


# ================================================================ 3. 比较
def close(a, b, tol):
    if isinstance(a, dict) and isinstance(b, dict):                 # 载重估计按 0.5 kg 取整：卡在边界上差 0.5 不算错
        return a.keys() == b.keys() and all(close(a[k], b[k], 0.51 if k == "load" else tol) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool) and not isinstance(b, bool):
        return math.isclose(a, b, rel_tol=1e-3, abs_tol=tol)
    return a == b


def main():
    t0 = time.time()
    rec = record()
    scen = os.path.join(OUT, "scenario.txt")
    open(scen, "w", encoding="utf-8").write("\n".join(rec.inp) + "\n")
    exe = build()
    r = subprocess.run([exe, scen], capture_output=True)
    got = r.stdout.decode("utf-8").splitlines()
    exp = [x for x in rec.exp if x != "C"]
    n_out = sum(1 for x in exp if x.startswith("O "))
    n_ev = sum(1 for x in exp if x.startswith("E "))
    bad = []
    for i, (e, g) in enumerate(zip(exp, got)):
        if e[0] != g[0]:
            bad.append((i, e, g))
        elif e[0] == "O":
            ev, gv = e.split()[1:], g.split()[1:]
            ok = all((math.isclose(float(a), float(b), rel_tol=1e-3, abs_tol=2e-3) if a.replace('.', '').replace('-', '').isdigit() else a == b)
                     for a, b in zip(ev, gv))
            if not ok:
                bad.append((i, e, g))
        elif not close(json.loads(e[2:]), json.loads(g[2:]), 3e-3):
            bad.append((i, e, g))
    if len(exp) != len(got):
        bad.append((min(len(exp), len(got)), f"Python {len(exp)} 行", f"C++ {len(got)} 行"))
    print(f"对比了 {n_out} 步（{n_out / 200:.0f} 秒）的输出、{n_ev} 条事件；不一样的 {len(bad)} 处；用时 {time.time() - t0:.0f} 秒")
    for i, e, g in bad[:40]:
        print(f"  第 {i} 行\n    Python：{e[:200]}\n    C++   ：{g[:200]}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
