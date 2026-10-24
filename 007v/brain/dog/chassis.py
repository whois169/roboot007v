"""底盘（四个麦轮 + 四个髋）：大脑这边的接口。指令和事件格式见 007v/chassis/protocol.py。

  virtual  虚拟底盘：MuJoCo 仿真的车 + 底盘程序，在笔记本上按真实时间跑（view: true 时开一个窗口看车怎么动）
  serial   真底盘：USB 串口接底盘 ESP32-S3（固件照着 007v/chassis/controller.py 翻成 C++）
  none     没有底盘：动不了，大模型也不会收到走路的技能

大脑只用这几个方法：move / turn / pose（等做完，返回 (成功没有, 原因)）、stop / estop / reset / limit（马上生效），
和 status（最近一次状态）。底盘每 0.1 秒发一次状态；大脑每 0.5 秒发一次心跳，底盘 2 秒收不到就自己停。
"""
import asyncio
import json
import logging
import os
import sys
import time

log = logging.getLogger("chassis")

BRAIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT_007V = os.path.dirname(BRAIN_DIR)
if ROOT_007V not in sys.path:
    sys.path.insert(0, ROOT_007V)
from chassis import protocol  # noqa: E402

STATE_CN = {"folded": "趴着（折叠）", "unfolding": "正在站起来", "standing": "站着", "folding": "正在趴下",
            "estop": "急停中", "fault": "故障停机"}
WHY_CN = {"FOLDED": "我现在趴着", "MOVING": "我还在走", "ESTOP": "急停没解除", "FAULT": "底盘有故障", "BAD": "指令不对",
          "STOPPED": "被叫停了", "TIMEOUT": "超时没走到", "BUSY": "正在站起来或趴下", "NO_LINK": "底盘没连上",
          "NO_CHASSIS": "没有接底盘", "CLIFF": "前面地面没了，停下了", "NODATA": "台阶测距没有读数"}


class Chassis:
    kind = "none"

    def __init__(self):
        self.status = None              # 最近一次 {"ev":"chassis",...}
        self.status_t = 0.0
        self.on_event = None            # 大脑注册：async on_event(事件)，warn / fault / hello 交给它
        self._id = 0
        self._wait = {}                 # id -> Future（等 ack / done）
        self._tasks = []
        self.loop = None

    @property
    def online(self):
        return self.status is not None and time.monotonic() - self.status_t < 1.5

    @property
    def state(self):
        return self.status["st"] if self.status else None

    async def start(self):
        self.loop = asyncio.get_running_loop()
        self._tasks.append(asyncio.create_task(self._hb_loop()))

    async def close(self):
        for t in self._tasks:
            t.cancel()

    async def _hb_loop(self):
        while True:
            self._send({"hb": 1})
            await asyncio.sleep(0.5)

    # ---- 发
    def _send(self, cmd: dict):
        err = protocol.check_cmd(cmd)
        if err:
            log.warning("底盘指令不合规，丢掉：%s（%s）", cmd, err)
            return
        self._send_line(json.dumps(cmd, separators=(",", ":")))

    def _send_line(self, line: str):
        pass

    async def _call(self, key, value, timeout):
        """发一条带 id 的指令，等 ack，再等 done。返回 (成功没有, 原因代码)。"""
        if not self.online:
            return False, "NO_LINK"
        self._id += 1
        cid = self._id
        ack, done = self.loop.create_future(), self.loop.create_future()
        self._wait[cid] = (ack, done)
        self._send({key: value, "id": cid})
        try:
            a = await asyncio.wait_for(ack, 2.0)
            if not a["ok"]:
                return False, a.get("why", "BAD")
            d = await asyncio.wait_for(done, timeout)
            return d["ok"], d.get("why", "")
        except asyncio.TimeoutError:
            self.stop()
            return False, "TIMEOUT"
        finally:
            self._wait.pop(cid, None)

    async def move(self, dx=0.0, dy=0.0, dth=0.0, v=0.4):
        dist = abs(dx) + abs(dy)
        return await self._call("move", {"dx": round(dx, 3), "dy": round(dy, 3), "dth": round(dth, 1), "v": v},
                                timeout=dist / max(v, 0.05) * 3 + abs(dth) / 30 + 8)

    async def pose(self, p: str):
        return await self._call("pose", p, timeout=15)

    def stop(self):
        self._send({"stop": True})

    def estop(self):
        self._send({"estop": True})

    def reset(self):
        self._send({"reset": True})

    def limit(self, v):
        self._send({"limit": {"v": v}})

    # ---- 收
    def receive_line(self, line: str):
        line = line.strip()
        if not line:
            return
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            log.warning("底盘发来的不是 JSON，丢掉：%s", line[:80])
            return
        err = protocol.check_event(ev)
        if err:
            log.warning("底盘事件不合规，丢掉：%s（%s）", line[:80], err)
            return
        kind = ev["ev"]
        if kind == "chassis":
            self.status, self.status_t = ev, time.monotonic()
            return
        if kind in ("ack", "done"):
            w = self._wait.get(ev["id"])
            if w:
                f = w[0] if kind == "ack" else w[1]
                if not f.done():
                    f.set_result(ev)
                if kind == "ack" and not ev["ok"] and not w[1].done():
                    w[1].set_result(ev)
            return
        if self.on_event:
            asyncio.get_running_loop().create_task(self.on_event(ev))

    # ---- 给感觉板和屏幕
    def readings(self):
        """感觉板上的底盘读数（Reading 列表）。"""
        from .senses import Reading
        src = "底盘 ESP32（轮子编码器、髋角度、电池电压）" if self.kind == "serial" else "虚拟底盘（MuJoCo 仿真，不是真车）"
        conf = 0.9 if self.kind == "serial" else 0.0
        st = self.status
        if not self.online:
            return [Reading("base.state", "底盘", "没连上", source=src, location="机身", confidence=1.0, max_age=5)]
        vx, vy, wz = st["v"]
        x, y, h = st["odo"]
        out = [Reading("base.state", "底盘状态", STATE_CN.get(st["st"], st["st"]), source=src, location="机身", confidence=1.0, max_age=5),
               Reading("base.speed", "移动速度", f"前后 {vx:+.2f} m/s，左右 {vy:+.2f} m/s，转 {wz * 57.3:+.0f}°/秒",
                       source=src, location="四个麦轮", confidence=1.0, max_age=5),
               Reading("base.odo", "从开机算起走到的位置", f"前 {x:+.2f} 米、左 {y:+.2f} 米，朝向 {h:+.0f}°（左转为正）",
                       source=src, location="四个麦轮（轮子打滑会有误差）", confidence=0.7, max_age=5,
                       note="只是轮子算出来的相对位置，不知道在屋里的哪里")]
        if st.get("load") is not None:
            out.append(Reading("base.load", "背上货物重量（估算）", st["load"], "kg", source=src + "，按加速时轮子推力估算",
                               location="货斗", confidence=0.6 if self.kind == "serial" else 0.0, max_age=600,
                               note="走动时才估得出来，误差约 ±1.5 kg"))
        if st.get("bat") is not None:
            out.append(Reading("base.battery", "电池电量", st["bat"], "%", source=src + "，按电压估算",
                               location="机身电池", confidence=conf or 0.0, max_age=60))
        if st.get("lim") is not None:
            out.append(Reading("base.limit", "限速", st["lim"], "m/s", source="大脑设置或底盘自己（看到坎、测距板坏了）",
                               location="底盘", max_age=1e9))
        if st.get("cliff"):
            out.append(Reading("base.cliff", "地面没了（台阶 / 边沿）", "前面" if st["cliff"] == "front" else "后面",
                               source=src + "，台阶测距", location="前保险杠、后格栅", confidence=conf or 0.0, max_age=5,
                               note="那个方向锁住了，只能往反方向走"))
        if st.get("chg"):
            out.append(Reading("base.charging", "正在充电", "是", source=src + "，电池电流", location="机身电池", max_age=5))
        return out


class VirtualChassis(Chassis):
    """虚拟底盘：仿真在后台线程按真实时间跑，收到的行转回大脑的事件循环。"""
    kind = "virtual"

    def __init__(self, payload=10.0, view=False, start_folded=True, **_):
        super().__init__()
        self.payload, self.view, self.folded = payload, view, start_folded
        self.sim = None

    async def start(self):
        await super().start()
        from chassis.virtual import VirtualChassis as Sim
        loop = self.loop
        self.sim = Sim(payload=self.payload, folded=self.folded,
                       on_line=lambda line: loop.call_soon_threadsafe(self.receive_line, line))
        self.sim.start(view=self.view)
        log.info("底盘：虚拟底盘（MuJoCo 仿真，背上 %g kg，%s）%s", self.payload, "趴着" if self.folded else "站着",
                 "，开了仿真窗口" if self.view else "")
        for _ in range(20):                              # 等第一条状态
            if self.online:
                break
            await asyncio.sleep(0.05)

    def _send_line(self, line):
        if self.sim:
            self.sim.send_line(line)

    async def close(self):
        await super().close()
        if self.sim:
            self.sim.stop()


class SerialChassis(Chassis):
    kind = "serial"

    def __init__(self, port, baud=115200, **_):
        super().__init__()
        try:
            import serial
        except ImportError:
            raise SystemExit("串口底盘要先安装：pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0.1)
        log.info("底盘：串口 %s @ %d", port, baud)

    async def start(self):
        await super().start()
        self._tasks.append(asyncio.create_task(self._read_loop()))

    async def _read_loop(self):
        buf = b""
        while True:
            chunk = await asyncio.to_thread(self.ser.read, 256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self.receive_line(line.decode("utf-8", errors="replace"))

    def _send_line(self, line):
        self.ser.write((line + "\n").encode("utf-8"))

    async def close(self):
        await super().close()
        self.ser.close()


class NoChassis(Chassis):
    kind = "none"

    async def start(self):
        log.info("底盘：没接（config.yaml 里 chassis.type: none）")

    async def _call(self, key, value, timeout):
        return False, "NO_CHASSIS"

    def readings(self):
        return []


def make(cfg: dict) -> Chassis:
    kind = (cfg or {}).get("type", "none")
    if kind == "virtual":
        return VirtualChassis(**cfg.get("virtual", {}))
    if kind == "serial":
        return SerialChassis(**cfg.get("serial", {}))
    if kind == "none":
        return NoChassis()
    raise SystemExit(f"config.yaml 里 chassis.type 只能是 virtual / serial / none，现在是 {kind!r}")
