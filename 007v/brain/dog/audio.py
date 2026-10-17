"""麦克风和扬声器（sounddevice，Windows / Linux 通用）。

麦克风：回调线程里拿到的每一小段声音，交给 asyncio 队列。
扬声器：一次播一段；播放期间 busy=True，主循环据此丢掉麦克风数据（半双工，防止听到自己）。
"""
import asyncio
import logging
import time

import numpy as np
import sounddevice as sd

log = logging.getLogger("audio")


def resample(x: np.ndarray, src: int, dst: int) -> np.ndarray:
    if src == dst or len(x) == 0:
        return x
    n = int(round(len(x) * dst / src))
    return np.interp(np.linspace(0, len(x) - 1, n), np.arange(len(x)), x).astype(np.float32)


class Microphone:
    def __init__(self, sample_rate=16000, device=None, block_seconds=0.1):
        self.sr = sample_rate
        self.device = device
        self.block_seconds = block_seconds

    def _open(self, rate, callback):
        return sd.InputStream(samplerate=rate, channels=1, dtype="float32", device=self.device,
                              blocksize=int(rate * self.block_seconds), callback=callback)

    async def frames(self):
        """一直产出 16k 单声道 float32 的小段声音（每段约 0.1 秒）。"""
        loop = asyncio.get_running_loop()
        q: asyncio.Queue = asyncio.Queue()
        rate = self.sr

        def cb(indata, frames, t, status):
            loop.call_soon_threadsafe(q.put_nowait, indata[:, 0].copy())

        try:
            stream = self._open(rate, cb)
        except sd.PortAudioError:
            rate = int(sd.query_devices(self.device, "input")["default_samplerate"])
            log.info("麦克风不支持 %d Hz，改用 %d Hz 再重采样", self.sr, rate)
            stream = self._open(rate, cb)
        with stream:
            while True:
                x = await q.get()
                yield resample(x, rate, self.sr) if rate != self.sr else x


class Speaker:
    def __init__(self, device=None, mute_tail_seconds=0.3):
        self.device = device
        self.mute_tail = mute_tail_seconds
        self._busy = 0
        self._last_end = 0.0
        self._lock = asyncio.Lock()

    @property
    def busy(self) -> bool:
        """正在出声，或刚说完还在回声尾巴里。"""
        return self._busy > 0 or time.monotonic() - self._last_end < self.mute_tail

    def play(self, pcm: np.ndarray, sample_rate: int) -> asyncio.Task:
        """排队播放，立刻返回一个 task。busy 马上变成 True，不等真正开始播。"""
        self._busy += 1
        return asyncio.create_task(self._play(pcm, sample_rate))

    def play_stream(self, chunks, sample_rate: int, on_chunk=None, prebuffer_seconds=0.3) -> asyncio.Task:
        """边收边播：chunks 是一段一段到的声音（async 迭代器）。先攒 prebuffer_seconds 再开始，免得刚开头就断。
        on_chunk(段) 在每段开始播时调用（屏幕声波用）。"""
        self._busy += 1
        return asyncio.create_task(self._play_stream(chunks, sample_rate, on_chunk, prebuffer_seconds))

    def stop(self):
        """马上停止正在播放的声音（被点了屏幕要它别说了）。"""
        sd.stop()
        self._abort = True
        s = getattr(self, "_stream", None)
        if s is not None:
            try:
                s.abort()
            except Exception:
                pass

    async def _play_stream(self, chunks, sr, on_chunk, prebuffer):
        try:
            async with self._lock:
                self._abort = False
                stream = sd.OutputStream(samplerate=sr, channels=1, dtype="float32", device=self.device, latency="high")
                self._stream = stream
                pending, started = [], False
                try:
                    async for x in chunks:
                        if self._abort:
                            break
                        pending.append(x)
                        if not started and sum(map(len, pending)) < prebuffer * sr:
                            continue
                        if not started:
                            stream.start()
                            started = True
                        for p in pending:
                            if on_chunk:
                                on_chunk(p)
                            await asyncio.to_thread(stream.write, p.reshape(-1, 1))
                        pending = []
                    if pending and not self._abort:      # 整句比攒的量还短
                        if not started:
                            stream.start()
                        for p in pending:
                            if on_chunk:
                                on_chunk(p)
                            await asyncio.to_thread(stream.write, p.reshape(-1, 1))
                    if not self._abort:
                        await asyncio.to_thread(stream.stop)     # 等缓冲里的声音播完
                except sd.PortAudioError:
                    if not self._abort:
                        raise
                finally:
                    self._stream = None
                    stream.close()
        finally:
            self._busy -= 1
            self._last_end = time.monotonic()

    async def _play(self, pcm, sr):
        try:
            async with self._lock:
                await asyncio.to_thread(sd.play, pcm, sr, device=self.device, blocking=True)
        finally:
            self._busy -= 1
            self._last_end = time.monotonic()


def list_devices():
    print(sd.query_devices())
    print(f"\n默认设备（输入, 输出）：{sd.default.device}")
