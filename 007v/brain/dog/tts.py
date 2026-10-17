"""嘴巴：把文字变成声音。只有一种做法：找“语音合成服务”合成。

语音合成服务是单独的程序，在桌面的 RoboDog_TTS 文件夹里（Qwen3-TTS，跑在笔记本显卡上），双击 start.bat 启动。
大脑通过 HTTP 找它：config.yaml 的 tts.url。现在大脑和服务都在笔记本上，用 http://127.0.0.1:8765；
以后大脑搬到树莓派 5，只要把地址改成笔记本在家里局域网的地址。

边合成边播：voice.say(文字) 马上去要这句话，返回一个 Utterance，声音一段一段地到（第一段一般 0.4 秒内）。
连着说几句时，几句会同时去要，服务那边排队一句一句合成，所以句子之间不会有空档。
服务连不上或出错时，这句话不出声（文字照样打印在控制台），大脑不会停下来；5 秒内不再反复去连。
"""
import asyncio
import hashlib
import logging
import os
import time

import numpy as np

from . import config

log = logging.getLogger("tts")


class Utterance:
    """一句话的声音，一段一段地到。"""

    def __init__(self, voice: "Voice", text: str):
        self.text = text
        self.sr = 24000
        self.ok = None                       # None = 还不知道；True = 至少到了一段；False = 失败了
        self._q: asyncio.Queue = asyncio.Queue()
        self._first = None
        self._task = asyncio.create_task(self._run(voice))

    async def _run(self, voice):
        if time.monotonic() < voice.down_until:
            self._q.put_nowait(None)
            return
        try:
            body = {"text": speakable(self.text), "speaker": voice.speaker, "effect": voice.effect}
            async with voice.client().stream("POST", voice.url + "/tts/stream", json=body) as r:
                r.raise_for_status()
                self.sr = int(r.headers.get("X-Sample-Rate", 24000))
                buf = b""
                async for b in r.aiter_bytes():
                    buf += b
                    n = len(buf) // 2 * 2
                    if n:
                        self._q.put_nowait(np.frombuffer(buf[:n], "<i2").astype(np.float32) / 32768)
                        buf = buf[n:]
        except asyncio.CancelledError:
            pass
        except Exception as e:
            voice.down_until = time.monotonic() + 5
            log.warning("语音合成失败（%s）：%s", type(e).__name__, str(e)[:120] or "连不上语音合成服务")
        finally:
            self._q.put_nowait(None)

    async def first(self):
        """等第一段到（返回它），或者失败（返回 None）。"""
        if self._first is None and self.ok is None:
            self._first = await self._q.get()
            self.ok = self._first is not None
        return self._first

    async def chunks(self):
        """所有的段（包括 first() 已经拿到的那段）。"""
        if await self.first() is None:
            return
        yield self._first
        while (x := await self._q.get()) is not None:
            yield x

    def cancel(self):
        self._task.cancel()


class Voice:
    def __init__(self, url="http://127.0.0.1:8765", speaker="", effect="", timeout_seconds=30, **_):
        self.url = url.rstrip("/")
        self.speaker, self.effect, self.timeout = speaker, effect, timeout_seconds
        self.name = f"语音合成服务 {self.url}"
        self.down_until = 0.0               # 连不上以后，几秒内不再反复去连（免得每句都卡住）
        self._client = None

    def client(self):
        import httpx
        if self._client is None:
            # trust_env=False：不走系统代理。语音服务只在本机或家里局域网上，走代理反而会被代理拦下（报 503）
            self._client = httpx.AsyncClient(timeout=httpx.Timeout(self.timeout, connect=2), trust_env=False)
        return self._client

    async def health(self) -> dict | None:
        """服务在不在、用的什么音色。连不上返回 None。"""
        try:
            r = await self.client().get(self.url + "/health", timeout=3)
            r.raise_for_status()
            return r.json()
        except Exception as e:
            log.warning("连不上语音合成服务 %s（%s）。先双击桌面 RoboDog_TTS 文件夹里的 start.bat", self.url, type(e).__name__)
            return None

    def say(self, text: str) -> Utterance:
        """马上开始要这句话的声音。"""
        return Utterance(self, text)

    async def synth(self, text: str):
        """整句合成完再返回 (pcm, 采样率)；失败返回 None。用在要存起来反复用的短句上。"""
        u = self.say(text)
        parts = [x async for x in u.chunks()]
        return (np.concatenate(parts), u.sr) if parts else None

    async def cached(self, text: str):
        """常用短句合成一次存到 data/cache，以后直接读。"""
        key = hashlib.md5(f"{self.url}|{self.speaker}|{self.effect}|{text}".encode()).hexdigest()[:12]
        f = config.path(f"data/cache/{key}.npz")
        if os.path.exists(f):
            z = np.load(f)
            return z["pcm"], int(z["sr"])
        out = await self.synth(text)
        if out is None:
            log.warning("「%s」合成失败，先用提示音代替", text)
            return beep(), 24000
        os.makedirs(os.path.dirname(f), exist_ok=True)
        np.savez(f, pcm=out[0], sr=out[1])
        return out


def make(cfg_tts: dict) -> Voice:
    return Voice(**cfg_tts)


def speakable(text: str) -> str:
    """念不准的符号先换成中文：59% -> 百分之59，22:14 -> 22点14分，℃ -> 度，km/h -> 公里每小时。"""
    import re
    text = re.sub(r"(\d+(?:\.\d+)?)\s*%", r"百分之\1", text)
    text = re.sub(r"(?<!\d)([01]?\d|2[0-3])[:：]([0-5]\d)(?!\d)", r"\1点\2分", text)   # 中文和数字之间没有 \b，用前后不是数字来判断
    for a, b in (("°C", "度"), ("℃", "度"), ("°", "度"), ("km/h", "公里每小时"), ("m/s", "米每秒"),
                 ("~", "到"), ("～", "到"), ("–", "到")):
        text = text.replace(a, b)
    return text


def chirp(sr=24000):
    """一声很轻、很短的上扬提示音：开始聆听时用。"""
    t = np.arange(int(sr * 0.07)) / sr
    f = 1200 + 900 * t / t[-1]
    return (0.12 * np.sin(2 * np.pi * np.cumsum(f) / sr) * np.hanning(len(t))).astype(np.float32)


def beep(sr=24000):
    """两声短促的“嘀嘀”：唤醒、摸头后的本地反应。"""
    t = np.arange(int(sr * 0.08)) / sr
    tone = lambda hz: 0.3 * np.sin(2 * np.pi * hz * t) * np.hanning(len(t))
    gap = np.zeros(int(sr * 0.04))
    return np.concatenate([tone(880), gap, tone(1320)]).astype(np.float32)
