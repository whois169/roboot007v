"""不开窗口，把 HUD 每个状态跑一遍：每个状态截图 + 一张总览图 + 一段演示 GIF。

用法：python scripts/hud_capture.py
输出：data/hud_shots/  （每个状态的 PNG、overview.png、demo.gif）

演示里的日志行是照着大脑真实会推的格式写的示例。
"""
import json
import math
import os
import sys

import pygame
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from dog import hud, thermal  # noqa: E402

SCALE = 3
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "hud_shots")
FPS = 30
TELE = ["T22.1* H55* SND -52DB", "FACE {f} MOT 0.0{m}", "10:32:{ss:02d} UP 12M STATE {st}"]   # 和大脑每秒推的读数同一个格式
BOOT = ["BOOT ROBODOG BRAIN", "TTS LOCAL MELO OK", "CAM OK FACE YUNET", "KWS OK ASR SENSEVOICE",
        "WX OK OUT 21.2C RH77", "LLM DEEPSEEK-CHAT READY"]
# (模式, 秒数, 额外指令, 这一段里推的日志, 截图: [(第几秒, 名字)])
SEQ = [
    ("idle", 7.0, {}, BOOT, [(6.6, "idle")]),
    ("track", 2.5, {}, ["EVT PERSON IN"], [(0.3, "assemble"), (2.0, "track")]),
    ("wake", 1.0, {}, ["KWS WAKE"], [(0.6, "wake")]),
    ("listen", 2.5, {}, [], [(1.8, "listen")]),
    ("think", 2.0, {}, ["ASR 9CH 0.05S"], [(1.5, "think")]),
    ("speak", 2.5, {}, ["LLM 1ST 1.92S", "TTS 0.41S"], [(1.8, "speak")]),
    ("idle", 3.0, {}, ["EVT PERSON OUT"], [(0.25, "dissolve")]),
    ("scan", 3.0, {}, ["SCAN ZONE 1/4"], [(2.0, "scan")]),
    ("thermal", 3.0, {}, [], [(2.0, "thermal")]),
    ("alert", 2.5, {"text": "DOOR OPEN"}, ["ALERT DOOR 2 OPEN"], [(1.8, "alert")]),
    ("error", 2.5, {"text": "E-NET"}, ["LLM ERR NET TIMEOUT"], [(1.8, "error")]),
    ("low_bat", 2.5, {}, ["BAT 8% RETURN TO DOCK"], [(1.8, "low_bat")]),
    ("sleep", 3.0, {}, [], [(2.0, "sleep")]),
]


def main():
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        if f.endswith(".png"):
            os.remove(os.path.join(OUT, f))
    surf = pygame.Surface((hud.W, hud.H))
    r = hud.HudRenderer(hud.Gfx(surf), now=0.0, seed=7)
    send = lambda cmd: r.apply(json.loads(json.dumps(cmd)))      # 和串口一样：先变成一行 JSON 再解析
    send({"status": {"temp": 22.1, "sim": True, "bat": 87}})
    scene = thermal.FakeScene(seed=3)                            # 热成像用模拟器的假房间
    t, frames, shots = 0.0, [], []
    last_sec, tele_n = -1, 0
    for mode, dur, extra, logs, wanted in SEQ:
        send({"mode": mode, **extra})
        if mode == "low_bat":
            send({"status": {"bat": 8}})
        if mode == "sleep":
            send({"status": {"bat": 87}})
        if mode == "idle" and not shots:
            r.next_sweep = 4.0
        start = t
        log_at = {round(start + 0.1 + k * (0.35 if mode == 'idle' and not shots else dur * 0.6 / max(1, len(logs))), 2): s
                  for k, s in enumerate(logs)}
        pending = sorted(wanted)
        for i in range(int(dur * FPS)):
            t = start + i / FPS
            u = t - start
            if int(t) != last_sec and t > 2.5:                    # 开机自检之后，每秒一行读数
                last_sec = int(t)
                tele_n += 1
                line = TELE[tele_n % 3].format(f=1 if mode in ("track", "wake", "listen", "think", "speak") else 0,
                                               m=tele_n % 7, ss=(41 + tele_n) % 60, st=mode.upper())
                send({"log": line})
            for at, s in list(log_at.items()):
                if t >= at:
                    send({"log": s})
                    del log_at[at]
            if mode == "listen":
                send({"level": 0.35 + 0.45 * abs(math.sin(u * 3.1))})
            elif mode == "speak":
                send({"level": 0.2 + 0.7 * abs(math.sin(u * 5.3) * math.sin(u * 1.7))})
            if mode in ("track", "wake", "listen", "think", "speak"):
                send({"gaze": [0.45 * math.sin(t * 0.9), 0.3 * math.sin(t * 1.7)]})
            else:
                send({"gaze": [0, 0]})
            if mode == "thermal" and i % 8 == 0:
                r.set_thermal(scene.frame(t).tolist(), sim=True)
            r.tick(t)
            big = pygame.transform.scale(surf, (hud.W * SCALE, hud.H * SCALE))
            img = Image.frombytes("RGB", big.get_size(), pygame.image.tobytes(big, "RGB"))
            if i % 2 == 0:
                frames.append(img)
            while pending and u >= pending[0][0]:
                name = pending.pop(0)[1]
                img.save(os.path.join(OUT, f"{len(shots) + 1:02d}_{name}.png"))
                shots.append((name, img))
        t += 1 / FPS
    w, h = shots[0][1].size
    pad, lab = 10, 22
    sheet = Image.new("RGB", (2 * w + 3 * pad, (len(shots) + 1) // 2 * (h + lab + pad) + pad), (40, 44, 52))
    d = ImageDraw.Draw(sheet)
    for k, (name, img) in enumerate(shots):
        x = pad + (k % 2) * (w + pad)
        y = pad + (k // 2) * (h + lab + pad)
        d.text((x, y + 4), f"{k + 1}. {name.upper()}", fill=(230, 230, 230))
        sheet.paste(img, (x, y + lab))
    sheet.save(os.path.join(OUT, "overview.png"))
    frames[0].save(os.path.join(OUT, "demo.gif"), save_all=True, append_images=frames[1:],
                   duration=int(2000 / FPS), loop=0, optimize=True)
    print(f"{len(shots)} 张截图、overview.png、demo.gif（{len(frames)} 帧）-> {OUT}")


if __name__ == "__main__":
    main()
