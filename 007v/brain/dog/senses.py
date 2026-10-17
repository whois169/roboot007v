"""感觉板：所有传感器的最新读数汇总在这里。每一条都带：值、来源、位置、时间、可信度。

两种方式往板子上放读数：
  board.put(Reading(...))         传感器自己推（温湿度、麦克风音量……）
  board.add_provider(函数)        需要时现算（摄像头检测结果、时钟……），函数返回 Reading 列表

大模型的【我自己感觉到的】就是 board.text() 的输出。
"""
import asyncio
import logging
import math
import random
import time
from dataclasses import dataclass, field

log = logging.getLogger("senses")


@dataclass
class Reading:
    key: str                 # 唯一名字，例如 "room.temp"
    name: str                # 给人看的名字，例如 "室内温度"
    value: object            # 值
    unit: str = ""           # 单位
    source: str = ""         # 来源：哪个传感器、怎么测的
    location: str = ""       # 位置：传感器装在哪
    confidence: float = 1.0  # 可信度 0–1；0 = 假数据
    note: str = ""           # 补充说明（比如“只说明有东西在动，不一定是人”）
    max_age: float = 600     # 超过这么多秒没更新就标“过期”
    t: float = field(default_factory=time.time)   # 测量时间（时间戳）

    def conf_text(self) -> str:
        if self.confidence <= 0:
            return "无（假数据）"
        return "高" if self.confidence >= 0.8 else "中" if self.confidence >= 0.5 else "低"

    def line(self, now=None) -> str:
        age = (now or time.time()) - self.t
        when = f"{age:.0f} 秒前" if age < 90 else f"{age / 60:.0f} 分钟前" if age < 5400 else f"{age / 3600:.1f} 小时前"
        if age > self.max_age:
            when += "，已经过期，可能不准了"
        val = f"{self.value:g}" if isinstance(self.value, float) else str(self.value)
        s = f"- {self.name}：{val}{(' ' + self.unit) if self.unit else ''}｜来源：{self.source}｜位置：{self.location}｜时间：{when}｜可信度：{self.conf_text()}"
        return s + (f"｜说明：{self.note}" if self.note else "")


class SenseBoard:
    def __init__(self):
        self.readings: dict[str, Reading] = {}
        self.providers = []

    def put(self, r: Reading):
        self.readings[r.key] = r

    def add_provider(self, fn):
        self.providers.append(fn)

    def all(self) -> list[Reading]:
        out = dict(self.readings)
        for fn in self.providers:
            try:
                for r in fn():
                    out[r.key] = r
            except Exception as e:      # 某个传感器出错不影响其他的
                log.warning("读传感器出错（%s）：%s", getattr(fn, "__qualname__", fn), e)
        return sorted(out.values(), key=lambda r: r.key)

    def get(self, key):
        return next((r for r in self.all() if r.key == key), None)

    def text(self) -> str:
        now = time.time()
        rs = self.all()
        return "\n".join(r.line(now) for r in rs) if rs else ""


# ---------------------------------------------------------------- 时钟（自己的“生物钟”）
def clock_provider(started: float):
    def fn():
        now = time.time()
        up = now - started
        uptime = f"{up / 60:.0f} 分钟" if up < 5400 else f"{up / 3600:.1f} 小时"
        return [
            Reading("clock.now", "现在时间", time.strftime("%Y-%m-%d %H:%M 星期") + "一二三四五六日"[time.localtime().tm_wday],
                    source="我自己的时钟", location="主控", confidence=0.95),
            Reading("clock.uptime", "我这次开机多久了", uptime, source="我自己的时钟", location="主控", confidence=1.0),
        ]
    return fn


# ---------------------------------------------------------------- 温湿度（先用假数据，留好接口）
class TempHumSensor:
    """温湿度传感器接口。以后换真的（比如 SHT40 接 ESP32），只要实现 read() 返回 (温度, 湿度)。"""
    name = "温湿度传感器"
    location = "头部通风处"
    fake = False

    async def read(self) -> tuple[float, float]:
        raise NotImplementedError


class FakeTempHum(TempHumSensor):
    """假数据：22°C、55% 左右慢慢飘。读数会标“假数据”，大模型知道这不是真测的。"""
    name = "温湿度传感器（还没装，现在是假数据）"
    location = "屋里（模拟）"
    fake = True

    def __init__(self, temp=22.0, hum=55.0):
        self.temp, self.hum = temp, hum
        self.t0 = time.time()

    async def read(self):
        drift = math.sin((time.time() - self.t0) / 600)
        return round(self.temp + drift + random.uniform(-0.1, 0.1), 1), round(self.hum + 3 * drift, 0)


async def temp_hum_loop(board: SenseBoard, sensor: TempHumSensor, period=30):
    conf = 0.0 if sensor.fake else 0.9
    note = "这是假数据，不是真的测量值；只有聊到温度湿度时才提它，提的时候要说明是假的" if sensor.fake else ""
    while True:
        try:
            t, h = await sensor.read()
            board.put(Reading("room.temp", "室内温度", t, "°C", sensor.name, sensor.location, conf, note, max_age=period * 4))
            board.put(Reading("room.humidity", "室内湿度", h, "%", sensor.name, sensor.location, conf, note, max_age=period * 4))
        except Exception as e:
            log.warning("温湿度读取失败：%s", e)
        await asyncio.sleep(period)


# ---------------------------------------------------------------- 麦克风音量
class SoundLevel:
    """从麦克风数据算环境音量，每秒往感觉板上放一次（只算音量，不录音）。"""

    def __init__(self, board: SenseBoard):
        self.board = board
        self.acc = []
        self.last = 0.0

    def feed(self, samples):
        import numpy as np
        self.acc.append(float(np.sqrt(np.mean(samples ** 2)) + 1e-9))
        now = time.time()
        if now - self.last < 1.0:
            return
        self.last = now
        db = 20 * math.log10(sum(self.acc) / len(self.acc))
        self.acc = []
        level = "很安静" if db < -55 else "安静" if db < -45 else "有点声音（可能有人说话或电器在响）" if db < -30 else "很吵"
        self.board.put(Reading("sound.level", "周围声音", f"{db:.0f} dBFS（{level}）", source="麦克风音量",
                               location="笔记本麦克风（以后在头部）", confidence=0.7,
                               note="只是音量大小，没有录音，也听不出内容；我自己说话时不算", max_age=30))
