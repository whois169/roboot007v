"""嘴巴：把文字变成声音。两种引擎，在 config.yaml 的 tts.engine 里切换：

local  本地 MeloTTS（sherpa-onnx）：不联网、速度稳定（笔记本上一句约 0.4–1 秒），音色略生硬
edge   微软 edge-tts：音色自然，但每句都要连微软服务器，国内常常要 2 秒以上，偶尔十几秒

两种都提供同样的方法：synth(文字) -> (pcm float32, 采样率) 或 None；cached(文字) 用于常用短句。
"""
import asyncio
import hashlib
import logging
import os

import numpy as np

from . import config

log = logging.getLogger("tts")


class _Base:
    sr = 24000

    def cache_key(self, text: str) -> str:
        raise NotImplementedError

    async def synth(self, text: str):
        raise NotImplementedError

    async def cached(self, text: str):
        """常用短句（比如唤醒后的“嗯？”）合成一次存到 data/cache，以后直接读，保证 0.1 秒内能出声。"""
        key = hashlib.md5(f"{self.cache_key(text)}|{text}".encode()).hexdigest()[:12]
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


class LocalVoice(_Base):
    def __init__(self, model_dir, speed=1.0, num_threads=2, **_):
        import sherpa_onnx
        d = config.rel(model_dir)            # 用相对路径：规则文件的读取库不认中文路径
        model = os.path.join(d, "model.onnx")
        if not os.path.exists(model):
            raise FileNotFoundError(f"找不到 {model}，先运行 python scripts/download_models.py")
        fsts = ",".join(os.path.join(d, f) for f in ("date.fst", "phone.fst", "number.fst")
                        if os.path.exists(os.path.join(d, f)))
        self.tts = sherpa_onnx.OfflineTts(sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                    model=model, lexicon=os.path.join(d, "lexicon.txt"),
                    tokens=os.path.join(d, "tokens.txt"), dict_dir=os.path.join(d, "dict")),
                num_threads=num_threads),
            rule_fsts=fsts))
        self.sr = self.tts.sample_rate
        self.speed = speed
        self.name = f"本地 MeloTTS（{os.path.basename(d)}）"
        self._lock = asyncio.Lock()          # 一个模型同一时间只合成一句

    def cache_key(self, text):
        return f"local|{self.name}|{self.speed}"

    async def synth(self, text: str):
        try:
            async with self._lock:
                a = await asyncio.to_thread(self.tts.generate, speakable(text), sid=0, speed=self.speed)
            pcm = np.array(a.samples, dtype=np.float32)
            return (pcm, a.sample_rate) if len(pcm) else None
        except Exception as e:
            log.warning("本地合成失败（%s）：%s", type(e).__name__, e)
            return None


class EdgeVoice(_Base):
    def __init__(self, voice="zh-CN-XiaoyiNeural", rate="+0%", pitch="+0Hz", **_):
        self.voice, self.rate, self.pitch = voice, rate, pitch
        self.name = f"edge-tts（{voice}）"

    def cache_key(self, text):
        return f"edge|{self.voice}|{self.rate}|{self.pitch}"

    async def synth(self, text: str):
        """合成失败（比如断网）返回 None，不让整个大脑停下来。"""
        import edge_tts
        import miniaudio
        try:
            mp3 = bytearray()
            com = edge_tts.Communicate(text, self.voice, rate=self.rate, pitch=self.pitch)
            async for chunk in com.stream():
                if chunk["type"] == "audio":
                    mp3 += chunk["data"]
            if not mp3:
                return None
            dec = miniaudio.decode(bytes(mp3), output_format=miniaudio.SampleFormat.SIGNED16,
                                   nchannels=1, sample_rate=self.sr)
            return np.frombuffer(dec.samples, dtype=np.int16).astype(np.float32) / 32768, self.sr
        except Exception as e:
            log.warning("edge-tts 合成失败（%s）：%s", type(e).__name__, e)
            return None


def make(cfg_tts: dict):
    engine = cfg_tts.get("engine", "local")
    if engine == "local":
        return LocalVoice(**cfg_tts.get("local", {}))
    if engine == "edge":
        return EdgeVoice(**cfg_tts.get("edge", {}))
    raise SystemExit(f"config.yaml 里 tts.engine 只能是 local 或 edge，现在是 {engine!r}")


def speakable(text: str) -> str:
    """本地合成念不出的符号先换成中文：59% -> 百分之59，22:14 -> 22点14分，℃ -> 度，km/h -> 公里每小时。"""
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
    """两声短促的“嘀嘀”：喊唤醒词后的本地反应。"""
    t = np.arange(int(sr * 0.08)) / sr
    tone = lambda hz: 0.3 * np.sin(2 * np.pi * hz * t) * np.hanning(len(t))
    gap = np.zeros(int(sr * 0.04))
    return np.concatenate([tone(880), gap, tone(1320)]).astype(np.float32)
