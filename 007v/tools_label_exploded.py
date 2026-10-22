"""给头部爆炸图加中文标注：读 Blender 存的零件屏幕位置，标签排在左右两边，用细线连到零件。"""
import json
import os

from PIL import Image, ImageDraw, ImageFont

D = os.path.join(os.path.dirname(__file__), "out", "head_print", "图")
NAMES = {
    "tub": "H01 屏幕模块主壳", "cap": "H02 顶盖", "retainer": "H03 屏板压框", "tray": "H04 电子托板", "frame": "H05 屏框",
    "core_L": "H06 电机耳骨架", "core_R": "H07 方窗耳骨架", "mid_L": "H08 中板", "mid_R": "H09 中板（温湿度）",
    "outer_L": "H10 外板", "outer_R": "H11 外板", "hub_L": "H12 转轴壳", "hub_R": "H13 转轴壳", "window": "H15 方窗面板",
    "flange": "H16 雷达法兰", "lidar_ring": "H17 雷达护环", "neck": "H18 脖子底座", "bezel": "H19 镜头圈",
    "b_board": "4.3 寸屏幕板", "b_acrylic": "烟黑亚克力窗", "b_servo": "舵机 STS3215", "b_lidar": "雷达 LD19",
    "b_pin": "空心轴销", "b_bush": "铜套", "b_speaker": "喇叭 Φ28",
    "b_camlens": "摄像头镜头", "b_tof": "多点测距 VL53L5CX", "b_led": "隐私灯",
    "b_mic1": "麦克风 ×2", "b_bh1750": "光线 BH1750", "b_ld2410": "毫米波 LD2410B",
    "b_esp32": "ESP32-S3", "b_camboard": "摄像头主板", "b_max": "功放 MAX98357A",
    "b_mlx": "热成像 MLX90640", "b_irtx": "红外发射", "b_irrx": "红外接收", "b_sht40": "温湿度 SHT40",
}
img = Image.open(os.path.join(D, "头部打印件_爆炸图.png")).convert("RGB")
W, H = img.size
pad = 560
out = Image.new("RGB", (W + 2 * pad, H), (236, 238, 241))
out.paste(img, (pad, 0))
dr = ImageDraw.Draw(out)
font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 30)
title = ImageFont.truetype("C:/Windows/Fonts/msyhbd.ttc", 44)
pos = json.load(open(os.path.join(D, "爆炸图标注.json")))
pts = [(NAMES[k], pad + x * W, y * H, k.startswith("b_")) for k, (x, y) in pos.items() if k in NAMES and 0 < x < 1 and 0 < y < 1]
left = sorted([p for p in pts if p[1] < pad + W / 2], key=lambda p: p[2])
right = sorted([p for p in pts if p[1] >= pad + W / 2], key=lambda p: p[2])
for side, items in (("L", left), ("R", right)):
    n = len(items)
    top, bot = 140, H - 60
    for i, (name, x, y, bought) in enumerate(items):
        ly = top + (bot - top) * (i + 0.5) / max(n, 1)
        col = (20, 110, 190) if bought else (40, 44, 50)
        if side == "L":
            tx = 30
            tw = dr.textlength(name, font=font)
            ax = tx + tw + 12
        else:
            tx = W + pad + 40
            ax = tx - 12
        dr.line([(ax, ly), (x, y)], fill=col, width=2)
        dr.ellipse([x - 6, y - 6, x + 6, y + 6], fill=col)
        dr.text((tx, ly - 20), name, font=font, fill=col)
dr.text((30, 30), "007V 头部爆炸图", font=title, fill=(20, 22, 26))
dr.text((30, 86), "黑字 = 3D 打印件    蓝字 = 买来的零件", font=font, fill=(90, 96, 104))
out.save(os.path.join(D, "头部爆炸图_带标注.png"))
print("ok", out.size)
