"""眼睛（宽屏 HUD）。大脑只发统一的 JSON 指令，不关心另一头是谁：

  sim      笔记本上的模拟器窗口（dog/hud.py 渲染，800×440 原始分辨率）
  serial   同样的 JSON 一行一条，通过串口发给屏幕板（微雪 4.3B），由它渲染
  console  只在控制台打印模式变化（没有显示器时用）

指令格式见 dog/hud.py 开头。模拟器收到的也是“先变成一行 JSON 文字、再解析”的结果，
和串口上走的字节完全一样，所以以后换成 ESP32，大脑代码不用改。

反方向（头部的板 → 大脑）的事件：屏幕触摸、传感器读数、隐私开关、心跳，格式见 dog/protocol.py。
收到的每一行先过 protocol.check_event()，合法的交给 on_event（大脑注册的回调）。
模拟器里用键盘代替：T = 点屏幕（按住 1 秒以上 = hold；也可以直接用鼠标点窗口），P = 拨隐私开关，
R = 红外学习时“拿遥控器对着它按了一下”。模拟器收到发码指令，会回一个 sent。
模拟器还假装有热成像：按大脑要的频率发假房间的热画面（dog/thermal.py 的 FakeScene），H = 打开 / 关掉假灶台。
还假装有 SHT40 温湿度：每 5 秒发一次屋里的温度、湿度，[ ] 键调温度（每次 1 度），, . 键调湿度（每次 5%）。
"""
import asyncio
import json
import logging
import os
import random
import time

from . import hud, protocol, thermal

log = logging.getLogger("eyes")

DIRECTIONS = ("center", "left", "right", "up", "down")
# look 技能的方向 -> gaze（按机器人自己的方向：左 = -1）
GAZE = {"center": [0, 0], "left": [-0.8, 0], "right": [0.8, 0], "up": [0, -0.8], "down": [0, 0.8]}


class Eyes:
    def __init__(self):
        self.mode_now = "idle"
        self.on_quit = None          # 模拟器窗口被关掉时调用
        self.on_key = {}             # 其他模块注册的快捷键
        self.on_event = None         # 收到 ESP32 事件时调用：async on_event(事件 dict)
        self.last_hb = 0.0           # 最后一次收到心跳（time.monotonic）
        self.is_real = False         # 是不是真的 ESP32（真的才检查心跳）

    async def start(self):
        pass

    async def close(self):
        pass

    def send(self, cmd: dict):
        err = hud.check(cmd)
        if err:
            log.warning("眼睛指令不合规，丢掉：%s（%s）", cmd, err)
            return
        if "mode" in cmd and cmd["mode"] != self.mode_now:
            self.mode_now = cmd["mode"]
            log.debug("[眼睛] %s", hud.LABEL[cmd["mode"]])
        if "log" in cmd:
            log.debug("[屏幕] %s", " | ".join(cmd["log"]))
        self._send_line(json.dumps(cmd, ensure_ascii=False, separators=(",", ":")))

    def send_ir(self, ir: dict):
        """红外指令（学、发、取消），和屏幕指令走同一根串口。"""
        err = protocol.check_ir_cmd(ir)
        if err:
            log.warning("红外指令不合规，丢掉：%s（%s）", ir, err)
            return
        log.debug("[红外] %s", ir)
        self._send_line(json.dumps({"ir": ir}, ensure_ascii=False, separators=(",", ":")))

    def send_thermal(self, th: dict):
        """热成像指令：每秒发几帧过来。"""
        err = protocol.check_thermal_cmd(th)
        if err:
            log.warning("热成像指令不合规，丢掉：%s（%s）", th, err)
            return
        self._send_line(json.dumps({"thermal": th}, separators=(",", ":")))

    def _send_line(self, line: str):
        raise NotImplementedError

    def receive_line(self, line: str):
        """收到 ESP32 发来的一行：解析、检查，合法的交给大脑。"""
        line = line.strip()
        if not line:
            return
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            log.warning("头部发来的不是 JSON，丢掉：%s", line[:80])
            return
        err = protocol.check_event(ev)
        if err:
            log.warning("头部事件不合规，丢掉：%s（%s）", line[:80], err)
            return
        if ev["ev"] == "hb":
            self.last_hb = time.monotonic()
        elif ev["ev"] not in ("sensor", "thermal"):        # 读数和热画面太频繁，不打印
            log.debug("[头部] %s", line)
        if self.on_event:
            asyncio.get_running_loop().create_task(self.on_event(ev))

    # ---- 常用的几种指令
    def mode(self, m: str, **kw):
        self.send({"mode": m, **kw})

    def gaze(self, x: float, y: float):
        self.send({"gaze": [round(float(x), 3), round(float(y), 3)]})     # numpy 的数转成普通数字才能进 JSON

    def level(self, v: float):
        self.send({"level": round(max(0.0, min(1.0, float(v))), 3)})

    def status(self, **kw):
        self.send({"status": kw})

    def log(self, *lines: str):
        """往屏幕的数据流里加几行（自动转成大写、去掉字体里没有的字符）。"""
        self.send({"log": [hud.clean(s) for s in lines]})


class SimEyes(Eyes):
    """模拟器：一个 pygame 窗口，按屏幕亮区 800×440 原始分辨率画，再按 scale 缩放显示。"""

    KEYS = "1234567890-="      # 数字键依次切换 12 个模式

    def __init__(self, scale=1.0, fps=30, name="007v", **_):
        super().__init__()
        self.scale, self.fps = scale, fps
        self.title = f"{name} HUD 模拟器（{hud.W}×{hud.H}）"
        self._task = None
        self._ir_until = 0.0         # 模拟红外：学习到什么时候为止
        self._th_hz, self._th_next, self._th_draw = 0, 0.0, 0.0   # 模拟热成像：发给大脑每秒几帧、下一帧何时发、何时重画
        self.scene = thermal.FakeScene()
        self.room = [24.0, 55.0]     # 模拟 SHT40：屋里温度、湿度
        self._room_next = 0.0

    async def start(self):
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame
        self.pg = pygame
        pygame.init()
        self.win = pygame.display.set_mode((int(hud.W * self.scale), int(hud.H * self.scale)))
        pygame.display.set_caption(self.title)
        self.canvas = pygame.Surface((hud.W, hud.H))
        self.r = hud.HudRenderer(hud.Gfx(self.canvas), now=time.monotonic())
        self._task = asyncio.create_task(self._loop())
        log.info("HUD 模拟器已打开。窗口快捷键：T 或鼠标点窗口 = 点屏幕（按住 1 秒 = 长按）、P 拨隐私开关、R 模拟按遥控器、H 开关假灶台、[ ] 调屋里温度、, . 调湿度；"
                 "1-9、0、-、= 切换 12 个模式，方向键移动视线，Q/A 调音量，Esc 退出")

    def _send_line(self, line):
        cmd = json.loads(line)             # 和串口一样：收到一行文字再解析
        if "ir" in cmd:
            self._sim_ir(cmd["ir"])
        elif "thermal" in cmd:
            self._th_hz = cmd["thermal"]["hz"]
        else:
            self.r.apply(cmd)

    def _sim_ir(self, ir):
        """假装是头上的红外收发管。"""
        if ir["op"] == "learn":
            self._ir_until = time.monotonic() + ir.get("timeout", 15)
            log.info("[模拟红外] 开始学习：在 HUD 窗口按 R，就当拿遥控器对着它按了一下")
        elif ir["op"] == "cancel":
            self._ir_until = 0.0
        elif ir["op"] == "send":
            log.info("[模拟红外] 发出：%s", json.dumps(ir["code"], ensure_ascii=False))
            asyncio.get_running_loop().call_later(0.15, self._fake, {"ev": "ir", "op": "sent"})

    def _send_room(self):
        self._room_next = time.monotonic() + 5
        t, h = self.room
        self._fake({"ev": "sensor", "name": "room_temp", "value": round(t + random.uniform(-0.1, 0.1), 1), "unit": "°C"})
        self._fake({"ev": "sensor", "name": "room_humidity", "value": round(h + random.uniform(-0.5, 0.5)), "unit": "%"})

    async def close(self):
        if self._task:
            self._task.cancel()
        self.pg.quit()

    def _fake(self, ev: dict):
        """模拟 ESP32 发来一行事件（和真串口走同一条路：先变成文字再解析、检查）。"""
        self.receive_line(json.dumps(ev, ensure_ascii=False))

    async def _loop(self):
        pg = self.pg
        gx = gy = lv = 0.0
        privacy = False
        pressed = {}                                     # 触摸键按下的时间，用来分“点一下”和“按住”
        while True:
            now = time.monotonic()
            if now >= self._th_draw:                      # 模拟热成像：屏幕一直用自己的传感器画（每秒 4 帧，和真模块差不多）
                self._th_draw = now + 0.25
                fr = self.scene.frame(now)
                self.r.set_thermal(fr.tolist(), sim=True)
                if self._th_hz and now >= self._th_next:  # 按大脑要的频率，把这一帧发给大脑
                    self._th_next = now + 1 / self._th_hz
                    self._fake(thermal.encode(fr))
            if now >= self._room_next:                    # 模拟 SHT40：每 5 秒发一次
                self._send_room()
            if self._ir_until and now > self._ir_until:   # 学习超时
                self._ir_until = 0.0
                self._fake({"ev": "ir", "op": "timeout"})
            for key, (zone, t0, held) in list(pressed.items()):
                if not held and now - t0 >= 1.0:          # 按住满 1 秒：发 hold
                    pressed[key] = (zone, t0, True)
                    self._fake({"ev": "touch", "zone": zone, "action": "hold"})
            for e in pg.event.get():
                if e.type == pg.QUIT or (e.type == pg.KEYDOWN and e.key == pg.K_ESCAPE):
                    if self.on_quit:
                        self.on_quit()
                    return
                if e.type == pg.MOUSEBUTTONDOWN and e.button == 1:     # 鼠标点窗口 = 手指点屏幕
                    pressed["mouse"] = ("screen", now, False)
                    continue
                if e.type == pg.MOUSEBUTTONUP and e.button == 1 and "mouse" in pressed:
                    zone, t0, held = pressed.pop("mouse")
                    self._fake({"ev": "touch", "zone": zone, "action": "release" if held else "tap"})
                    continue
                if e.type == pg.KEYUP and e.key in pressed:
                    zone, t0, held = pressed.pop(e.key)
                    self._fake({"ev": "touch", "zone": zone, "action": "release" if held else "tap"})
                    continue
                if e.type != pg.KEYDOWN:
                    continue
                if e.key == pg.K_t:
                    pressed[e.key] = ("screen", now, False)
                    continue
                if e.key == pg.K_p:
                    privacy = not privacy
                    self._fake({"ev": "privacy", "on": privacy})
                    continue
                if e.key in (pg.K_LEFTBRACKET, pg.K_RIGHTBRACKET, pg.K_COMMA, pg.K_PERIOD):   # 调假的屋里温湿度
                    i = 0 if e.key in (pg.K_LEFTBRACKET, pg.K_RIGHTBRACKET) else 1
                    step = (1.0 if i == 0 else 5.0) * (1 if e.key in (pg.K_RIGHTBRACKET, pg.K_PERIOD) else -1)
                    self.room[i] = max(0.0 if i == 0 else 5.0, min(45.0 if i == 0 else 95.0, self.room[i] + step))
                    log.info("[模拟温湿度] 屋里 %.0f°C、%.0f%%", *self.room)
                    self._send_room()
                    continue
                if e.key == pg.K_h:                      # 假灶台：开 / 关
                    self.scene.stove = not self.scene.stove
                    log.info("[模拟热成像] 假灶台%s", "打开（90°C 以上）" if self.scene.stove else "关掉")
                    continue
                if e.key == pg.K_r:                      # 模拟按了一下遥控器
                    if self._ir_until:
                        self._ir_until = 0.0
                        value = "0x%08X" % random.getrandbits(32)
                        self._fake({"ev": "ir", "op": "learned", "code": {"proto": "NEC", "bits": 32, "value": value}})
                    else:
                        log.info("[模拟红外] 现在不在学习，R 键没用（先让它“学一下某个遥控器按键”）")
                    continue
                ch = e.unicode
                if ch and ch in self.KEYS:
                    self.mode(hud.MODES[self.KEYS.index(ch)])
                elif e.key in (pg.K_LEFT, pg.K_RIGHT, pg.K_UP, pg.K_DOWN):   # 按你看到的方向
                    gx += {pg.K_LEFT: 0.25, pg.K_RIGHT: -0.25}.get(e.key, 0)
                    gy += {pg.K_UP: -0.25, pg.K_DOWN: 0.25}.get(e.key, 0)
                    gx, gy = max(-1, min(1, gx)), max(-1, min(1, gy))
                    self.gaze(gx, gy)
                elif ch in ("q", "a"):
                    lv = max(0.0, min(1.0, lv + (0.2 if ch == "q" else -0.2)))
                    self.level(lv)
                elif e.key in self.on_key:
                    self.on_key[e.key]()
            self.r.tick(time.monotonic())
            self.pg.transform.scale(self.canvas, self.win.get_size(), self.win)   # 按真实像素放大
            self.pg.display.flip()
            await asyncio.sleep(1 / self.fps)


class SerialEyes(Eyes):
    """真硬件：屏幕板（port）收屏幕指令，传感器板（sensor_port）收红外、热成像指令；两根线来的事件都交给大脑。
    只写了 port 时，所有指令都走这一根线（以前一块板的接法）。每条指令一行 JSON（UTF-8，结尾 \\n）。"""

    def __init__(self, port, baud=115200, sensor_port="", **_):
        super().__init__()
        try:
            import serial
        except ImportError:
            raise SystemExit("串口眼睛要先安装：pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.sensor = serial.Serial(sensor_port, baud, timeout=0.1) if sensor_port else None
        self.is_real = True
        self._tasks = []
        self.base_hz = 0                 # 大脑平时要的热画面频率；屏幕切到 THERMAL 时临时提到 THERMAL_HZ
        log.info("屏幕板：串口 %s；传感器板：%s", port, sensor_port or "没单独接（和屏幕板共用一根）")

    THERMAL_HZ = 4

    def send(self, cmd: dict):
        """分线时，屏幕自己读不到热成像：切到 THERMAL 就让传感器板每秒发 4 帧（转给屏幕板画），切走再恢复。"""
        was = self.mode_now
        super().send(cmd)
        if self.sensor and "mode" in cmd and (was == "thermal") != (self.mode_now == "thermal"):
            hz = self.THERMAL_HZ if self.mode_now == "thermal" else self.base_hz
            self._send_line(json.dumps({"thermal": {"hz": hz}}, separators=(",", ":")))

    def send_thermal(self, th: dict):
        self.base_hz = th.get("hz", 0) if isinstance(th, dict) else 0
        if self.sensor and self.mode_now == "thermal":
            th = {"hz": max(self.base_hz, self.THERMAL_HZ)}
        super().send_thermal(th)

    async def start(self):
        self._tasks = [asyncio.create_task(self._read_loop(s)) for s in (self.ser, self.sensor) if s]

    async def _read_loop(self, ser):
        """一直读一块板发来的行（在线程里读，不卡住大脑）。"""
        buf = b""
        while True:
            chunk = await asyncio.to_thread(ser.read, 256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if ser is self.sensor and b'"ev":"thermal"' in line[:40]:
                    self.ser.write(line + b"\n")        # 热画面原样转给屏幕板，THERMAL 模式用它画
                self.receive_line(line.decode("utf-8", errors="replace"))

    def _send_line(self, line):
        to_sensor = self.sensor and (line.startswith('{"ir"') or line.startswith('{"thermal"'))
        (self.sensor if to_sensor else self.ser).write((line + "\n").encode("utf-8"))

    async def close(self):
        for t in self._tasks:
            t.cancel()
        for s in (self.ser, self.sensor):
            if s:
                s.close()


class ConsoleEyes(Eyes):
    """只打印模式变化（音量、视线这种高频指令不打印）。"""

    def _send_line(self, line):
        cmd = json.loads(line)
        if "ir" in cmd:
            log.info("[红外] %s（控制台模式没有红外硬件）", cmd["ir"]["op"])
        elif "thermal" in cmd:
            pass
        elif "mode" in cmd:
            log.info("[眼睛] %s %s", hud.LABEL[cmd["mode"]], cmd.get("text", ""))


def make(cfg_eyes: dict, persona_name="007v") -> Eyes:
    kind = cfg_eyes.get("type", "sim")
    if kind in ("sim", "window"):
        return SimEyes(name=persona_name, **cfg_eyes.get("sim", cfg_eyes.get("window", {})))
    if kind == "serial":
        return SerialEyes(**cfg_eyes.get("serial", {}))
    if kind == "console":
        return ConsoleEyes()
    raise SystemExit(f"config.yaml 里 eyes.type 只能是 sim / serial / console，现在是 {kind!r}")
