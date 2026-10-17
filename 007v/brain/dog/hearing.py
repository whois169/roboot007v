"""耳朵：唤醒词、说话检测（VAD）、语音识别。全部在本机跑，声音不出电脑。"""
import glob
import logging
import os
import time

import numpy as np
import sherpa_onnx
from pypinyin import Style, pinyin

from . import config

log = logging.getLogger("hearing")

KWS_DIR = "models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01"
VAD_MODEL = "models/silero_vad.onnx"


def _pick(d, prefix):
    """在模型目录里找 epoch-12 的 int8 文件（官方推荐的那组）。"""
    hits = sorted(glob.glob(os.path.join(d, f"{prefix}-epoch-12-*.int8.onnx")))
    if not hits:
        raise FileNotFoundError(f"{d} 里没有 {prefix} 模型，先运行 python scripts/download_models.py")
    return hits[0]


def word_to_tokens(word: str) -> list[str]:
    """中文 -> 唤醒模型用的拼音 token：声母 + 带声调的韵母，例如 你好 -> n ǐ h ǎo。"""
    ini = pinyin(word, style=Style.INITIALS, strict=False)
    fin = pinyin(word, style=Style.FINALS_TONE, strict=False)
    out = []
    for i, f in zip(ini, fin):
        if i[0]:
            out.append(i[0])
        out.append(f[0])
    return out


class WakeWord:
    def __init__(self, keywords, threshold=0.25, boost=1.5, sample_rate=16000):
        d = config.path(KWS_DIR)
        tokens_file = os.path.join(d, "tokens.txt")
        known = {line.split()[0] for line in open(tokens_file, encoding="utf-8")}
        lines = []
        for w in keywords:
            toks = word_to_tokens(w)
            bad = [t for t in toks if t not in known]
            if bad:
                raise ValueError(f"唤醒词「{w}」里有模型不认识的音：{bad}。唤醒词只能用中文。")
            lines.append(f"{' '.join(toks)} :{boost} #{threshold} @{w}")
        kw_file = config.path("data/keywords.txt")
        os.makedirs(os.path.dirname(kw_file), exist_ok=True)
        with open(kw_file, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        self.kws = sherpa_onnx.KeywordSpotter(
            tokens=tokens_file, encoder=_pick(d, "encoder"), decoder=_pick(d, "decoder"),
            joiner=_pick(d, "joiner"), keywords_file=kw_file, num_threads=1,
            keywords_threshold=threshold, keywords_score=boost)
        self.sr = sample_rate
        self.stream = self.kws.create_stream()
        log.info("唤醒词：%s", "、".join(keywords))

    def feed(self, samples: np.ndarray) -> str:
        """喂一段声音；听到唤醒词就返回那个词，否则返回空字符串。"""
        self.stream.accept_waveform(self.sr, samples)
        while self.kws.is_ready(self.stream):
            self.kws.decode_stream(self.stream)
            hit = self.kws.get_result(self.stream)
            if hit:
                self.kws.reset_stream(self.stream)
                return hit
        return ""

    def reset(self):
        """丢掉之前听到的半截声音（比如自己刚说完话以后）。"""
        self.stream = self.kws.create_stream()


class Listener:
    """唤醒后听一句话：开口前最多等 wait 秒，说完停顿 end_silence 秒就结束。"""

    def __init__(self, wait=5.0, max_speech=15.0, end_silence=0.8, sample_rate=16000):
        cfg = sherpa_onnx.VadModelConfig()
        cfg.silero_vad.model = config.path(VAD_MODEL)
        cfg.silero_vad.min_silence_duration = end_silence
        cfg.silero_vad.min_speech_duration = 0.25
        cfg.silero_vad.max_speech_duration = max_speech
        cfg.sample_rate = sample_rate
        self.vad = sherpa_onnx.VoiceActivityDetector(cfg, buffer_size_in_seconds=max_speech + 5)
        self.wait = wait
        self.deadline = 0.0
        self.heard_speech = False

    def start(self, wait=None):
        self.vad.reset()
        self.heard_speech = False
        self.deadline = time.monotonic() + (self.wait if wait is None else wait)

    def feed(self, samples: np.ndarray):
        """返回 (状态, 声音)。状态：'listening' 还在听 / 'done' 说完了 / 'timeout' 没人说话。"""
        self.vad.accept_waveform(samples)
        if self.vad.is_speech_detected():
            self.heard_speech = True
        if not self.vad.empty():
            seg = np.array(self.vad.front.samples, dtype=np.float32)
            self.vad.pop()
            return "done", seg
        if not self.heard_speech and time.monotonic() > self.deadline:
            return "timeout", None
        return "listening", None


class Recognizer:
    def __init__(self, model_dir, num_threads=2):
        d = config.path(model_dir)
        model = os.path.join(d, "model.int8.onnx")
        if not os.path.exists(model):
            raise FileNotFoundError(f"找不到 {model}，先运行 python scripts/download_models.py")
        self.rec = sherpa_onnx.OfflineRecognizer.from_sense_voice(
            model=model, tokens=os.path.join(d, "tokens.txt"), num_threads=num_threads,
            language="auto", use_itn=True)

    def transcribe(self, samples: np.ndarray, sample_rate=16000) -> str:
        s = self.rec.create_stream()
        s.accept_waveform(sample_rate, samples)
        self.rec.decode_stream(s)
        return s.result.text.strip()
