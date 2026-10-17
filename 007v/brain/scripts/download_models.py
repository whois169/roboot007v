"""下载本地模型（唤醒词、说话检测、语音识别、人脸）到 brain/models/（第一次运行前执行一次；树莓派上也一样）。

用法：python scripts/download_models.py
网络断了再运行一次就行，会从断的地方接着下。
"""
import os
import sys
import tarfile
import urllib.request

BASE = "https://github.com/k2-fsa/sherpa-onnx/releases/download"
MODELS = [
    # (说明, 下载地址, 解压后的文件夹或文件名)
    ("唤醒词 KWS（中文，约 13MB）",
     f"{BASE}/kws-models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01.tar.bz2",
     "sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01"),
    ("语音识别 SenseVoice（中英日韩粤；下载约 1GB，解压后只留 230MB 的压缩版）",
     f"{BASE}/asr-models/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17.tar.bz2",
     "sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17"),
    ("人脸检测 YuNet（OpenCV，约 300KB）",
     "https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx",
     "face_detection_yunet_2023mar.onnx"),
    ("说话检测 VAD（silero，约 2MB）",
     f"{BASE}/asr-models/silero_vad.onnx",
     "silero_vad.onnx"),
    ("人脸识别 SFace（OpenCV，认人用，约 37MB）",
     "https://github.com/opencv/opencv_zoo/raw/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx",
     "face_recognition_sface_2021dec.onnx"),
]

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEST = os.path.join(HERE, "models")


def fetch(url, fname, tries=8):
    """断点续传下载：网络断了就从断的地方接着下，最多重试 tries 次。"""
    for attempt in range(1, tries + 1):
        have = os.path.getsize(fname) if os.path.exists(fname) else 0
        req = urllib.request.Request(url, headers={"Range": f"bytes={have}-"} if have else {})
        try:
            with urllib.request.urlopen(req, timeout=30) as r:
                if have and r.status != 206:      # 服务器不支持续传：从头来
                    have = 0
                total = have + int(r.headers.get("Content-Length", 0))
                with open(fname, "ab" if have else "wb") as f:
                    while chunk := r.read(1 << 16):
                        f.write(chunk)
                        have += len(chunk)
                        sys.stdout.write(f"\r  {have / 1e6:6.1f} / {total / 1e6:.1f} MB")
                        sys.stdout.flush()
            if have >= total:
                print()
                return
        except OSError as e:
            print(f"\n  第 {attempt} 次中断（{e}），接着下……")
    raise SystemExit(f"下载失败：{url}，网络好一点再运行一次这个脚本（会接着下）")


def main():
    os.makedirs(DEST, exist_ok=True)
    for desc, url, name in MODELS:
        target = os.path.join(DEST, name)
        if os.path.isfile(target) or (os.path.isdir(target) and os.listdir(target)):
            print(f"[已有] {desc}")
            continue
        print(f"[下载] {desc}\n  {url}")
        fname = os.path.join(DEST, url.rsplit("/", 1)[1])
        fetch(url, fname)
        if fname.endswith(".tar.bz2"):
            print("  解压中…")
            with tarfile.open(fname) as t:
                t.extractall(DEST)
            os.remove(fname)
            # SenseVoice 包里同时带了完整精度（900MB）和 int8 压缩版（230MB），我们只用 int8。
            # 注意有的包里 int8 只是个几百字节的占位文件，那就不能删完整版
            fp32 = os.path.join(target, "model.onnx")
            int8 = os.path.join(target, "model.int8.onnx")
            if os.path.exists(fp32) and os.path.exists(int8) and os.path.getsize(int8) > 1_000_000:
                os.remove(fp32)
    print("模型都在", DEST)


if __name__ == "__main__":
    main()
