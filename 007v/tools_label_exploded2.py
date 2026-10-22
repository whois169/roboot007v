"""给爆炸图加中文标注（通用版）：读 render_exploded.py 存的 *_标注.json。
用法：python 007v/tools_label_exploded2.py <png> <标题>"""
import json
import sys

from PIL import Image, ImageDraw, ImageFont

png, title = sys.argv[1], sys.argv[2]
img = Image.open(png).convert("RGB")
W, H = img.size
pad = 600
out = Image.new("RGB", (W + 2 * pad, H), (236, 238, 241))
out.paste(img, (pad, 0))
dr = ImageDraw.Draw(out)
font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 30)
tf = ImageFont.truetype("C:/Windows/Fonts/msyhbd.ttc", 46)
pts = [(d["label"], pad + d["x"] * W, d["y"] * H, d["printed"]) for d in json.load(open(png[:-4] + "_标注.json", encoding="utf-8"))
       if 0 < d["x"] < 1 and 0 < d["y"] < 1]
left = sorted([p for p in pts if p[1] < pad + W / 2], key=lambda p: p[2])
right = sorted([p for p in pts if p[1] >= pad + W / 2], key=lambda p: p[2])
for side, items in (("L", left), ("R", right)):
    n = len(items)
    for i, (name, x, y, printed) in enumerate(items):
        ly = 150 + (H - 210) * (i + 0.5) / max(n, 1)
        col = (40, 44, 50) if printed else (20, 110, 190)
        if side == "L":
            tx = 30
            ax = tx + dr.textlength(name, font=font) + 12
        else:
            tx = W + pad + 40
            ax = tx - 12
        dr.line([(ax, ly), (x, y)], fill=col, width=2)
        dr.ellipse([x - 6, y - 6, x + 6, y + 6], fill=col)
        dr.text((tx, ly - 20), name, font=font, fill=col)
dr.text((30, 30), title, font=tf, fill=(20, 22, 26))
dr.text((30, 90), "黑字 = 3D 打印件    蓝字 = 买的 / 加工件", font=font, fill=(90, 96, 104))
o = png[:-4] + "_带标注.png"
out.save(o)
print(o)
