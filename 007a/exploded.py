"""007A 爆炸图：整机（外壳 + 骨架 + 电子 + 4 个腿模块）和单腿（左前腿，沿 y 一层层拉开）。
读 out/parts（外观，model.py / body_v10.py）和 out/eng_parts（第 2 步工程件）的 STL + manifest，按零件名分组给偏移，
写 out/exploded/{whole,leg}/manifest.json，再调 blender/render_exploded.py 出图、tools_label 加中文标注。
用法：python 007a/exploded.py [whole|leg] [--no-render]
"""
import json
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "exploded")
DOC = os.path.join(os.path.dirname(HERE), "docs", "机械狗007A")
BLENDER = r"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe"
PRINTED = ("打印", "外层壳", "壳", "底盘", "黑腰", "头", "防撞杠", "护圈", "装甲", "货台", "罩")


def centre(path):
    """二进制 STL 的包围盒中心（mm）。"""
    with open(path, "rb") as f:
        f.seek(80)
        n = int(np.frombuffer(f.read(4), np.uint32)[0])
        a = np.frombuffer(f.read(50 * n), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    v = a["v"].reshape(-1, 3)
    return (v.min(0) + v.max(0)) / 2


def load(d):
    man = json.load(open(os.path.join(HERE, "out", d, "manifest.json"), encoding="utf-8"))
    for it in man:
        it["path"] = os.path.join(HERE, "out", d, it["file"])
        it["c"] = centre(it["path"])
    return man


def has(n, *keys):
    return any(k in n for k in keys)


# ---------------------------------------------------------------- 整机
TOP = ("上层甲板壳", "货台", "绑带孔底", "检修板螺栓", "雷达", "拾音", "麦克风", "录音灯", "电源开关", "电源光圈", "护角")
HEAD = ("头", "屏幕", "眼睛", "深度相机", "相机镜头", "探照灯", "热像仪", "前防撞杠", "台阶测距", "凹槽内衬")
TAIL = ("尾灯", "急停", "充电口", "尾部", "总开关")
SIDE = ("侧装甲板", "装甲进风槽底", "装甲螺丝", "字样")
LOW = ("下层底盘", "黑腰", "腰线灯", "底盘出风槽", "卡口", "髋窝底", "扬声器孔底")
LEG_APP = ("髋台阶", "髋盖", "髋光环", "髋中心盖", "髋螺栓", "膝盖帽外", "膝光点", "脚垫")      # 外观腿上的盖子、灯、脚垫（工程腿里没有）
LEG_TAG = {"前左": (1, 1), "前右": (1, -1), "后左": (-1, 1), "后右": (-1, -1)}
LEG_DX, LEG_DY = 170.0, 300.0


def whole():
    items = []
    labels = set()

    def add(it, off, label=None, printed=None):
        lab = label if label and label not in labels else ""
        labels.add(lab)
        items.append(dict(file=os.path.relpath(it["path"], os.path.join(OUT, "whole")), color=it["color"], offset=[float(x) for x in off],
                          label=lab, printed=bool(printed if printed is not None else has(lab, *PRINTED))))
    for it in load("parts"):
        n, c = it["name"], it["c"]
        tag = n[:2]
        if tag in LEG_TAG:
            if not has(n, *LEG_APP):
                continue                                               # 外观大腿 / 小腿换成工程件
            fx, sy = LEG_TAG[tag]
            extra = 0.0 if "脚垫" in n else 60.0
            add(it, (fx * LEG_DX, sy * (LEG_DY + extra), 0), "髋盖 + 青色光环" if "髋光环" in n else ("脚垫（橡胶）" if "脚垫" in n else None), False)
        elif has(n, *SIDE):
            add(it, (0, np.sign(c[1]) * 230, 40), "侧装甲板（打印）" if "侧装甲板" in n else None, True)
        elif has(n, *TOP):
            add(it, (0, 0, 330), "上层甲板壳 + 货台（打印）" if n == "上层甲板壳" else ("激光雷达" if n == "雷达" else None))
        elif has(n, *HEAD):
            add(it, (330, 0, 60), {"头": "头（屏幕 + 抬头低头）", "前防撞杠": "前防撞杠（深度相机、热像仪、探照灯、台阶测距都装在上面）",
                                   "前防撞杠": "前防撞杠（打印）"}.get(n))
        elif has(n, *TAIL):
            add(it, (-300, 0, 30), {"尾部接口板": "尾部接口板（急停、总开关、充电口）", "气体进气槽底": "气体检测"}.get(n))
        elif has(n, *LOW):
            add(it, (0, 0, -300), {"下层底盘": "下层底盘（打印）", "黑腰": "黑腰 + 腰线灯（打印）"}.get(n), True)
        else:
            add(it, (0, 0, 330))
    for it in load("eng_parts"):
        n, c = it["name"], it["c"]
        m = re.match(r"\((-?1), (-?1)\)(.*)", n)
        if m:
            fx, sy, nm = int(m.group(1)), int(m.group(2)), m.group(3)
            lab = {"外摆电机": "外摆电机（宇树 GO-M8010-6）", "髋电机": "髋电机", "膝电机": "膝电机", "同步带 HTD5M 宽20": "同步带 HTD5M（膝 1:1.5）",
                   "大腿外层壳（打印，完整外形）": "大腿外层壳（打印）", "小腿（打印，夹住小腿根）": "小腿（打印，夹住小腿根）"}.get(nm) if (fx, sy) == (1, 1) else None
            add(it, (fx * LEG_DX, sy * LEG_DY, 0), lab, "打印" in (lab or ""))
            continue
        if n.startswith("L 角码"):
            continue                                                   # 62 个角码：拉开以后悬在空中，图上太乱，不画
        if n.startswith("A07"):
            add(it, (0, 0, 190), "A07 顶板（铝 3）")
        elif n.startswith("A05"):
            add(it, (140, 0, 0), "A05 前端板（铝 4）")
        elif n.startswith("A06"):
            add(it, (-140, 0, 0), "A06 后端板（铝 4）")
        elif n.startswith("A03") or "插槽导轨" in n:
            add(it, (0, np.sign(c[1]) * 110, 0), "A03 侧梁（铝 3）" if n.startswith("A03") else None)
        elif n.startswith("A01"):
            add(it, (0, 0, -170), "A01 底板（铝 3）")
        elif n.startswith("A02"):
            add(it, (np.sign(c[0]) * 60, 0, 0), "A02 外摆隔板（铝 5）")
        elif n.startswith("A04"):
            add(it, (np.sign(c[0]) * 100, np.sign(c[1]) * 40, 0), "A04 纵梁（铝 4）")
        elif n.startswith("C01"):
            add(it, (0, 0, 100), "C01 电子托板（亚克力）")
        elif has(n, "电池"):
            add(it, (0, 0, -60), "电池 6S3P 280 Wh" if n.startswith("电池 6S") else None)
        elif has(n, "继电器", "保险丝"):
            add(it, (0, 0, -60), n.split("（")[0] if "继电器" in n else None)
        else:
            add(it, (0, 0, 130), {"树莓派 5（带散热）": "树莓派 5", "IMU 超核 HI13（正中间）": "IMU（正中间）", "USB 转 4 路 485": "USB 转 4 路 485（电机通信）"}.get(n))
    return items


# ---------------------------------------------------------------- 单腿（左前）
LEG_LAB = {"外摆电机": "外摆电机", "髋座（铝，CNC）": "髋座（铝 CNC）", "髋电机": "髋电机", "大腿杯 + 内板（铝，CNC）": "大腿杯 + 内板（铝 CNC）",
           "膝电机": "膝电机（跟大腿一起转）", "转接盘 + 轴（钢，车削）": "转接盘 + 轴（钢）", "小带轮 HTD5M 16 齿": "小带轮 16 齿",
           "同步带 HTD5M 宽20": "同步带 HTD5M 宽 20", "张紧轮 Φ18": "张紧轮", "小腿根 L09（铝 CNC，带 HTD5M 24 齿）": "小腿根（铝 CNC，24 齿 + 舌头）",
           "小腿（打印，夹住小腿根）": "小腿（打印，夹住小腿根）", "大腿外板（铝 4）": "大腿外板（铝 4）", "大腿外层壳（打印，完整外形）": "大腿外层壳（打印）",
           "膝轴 Φ12（两头 M5，固定）": "膝轴 Φ12（两头 M5，固定）", "膝轴承 6801 外": "轴承 6801（在小腿根里）", "立柱 M4 0": "立柱 M4"}


def leg():
    items = []
    y0 = 44.0
    for it in load("eng_parts"):
        if not it["name"].startswith("(1, 1)"):
            continue
        nm = it["name"][6:]
        c = it["c"]
        if nm == "外摆电机":
            off = (0, -110, 0)
        else:
            off = [0, (c[1] - y0) * 2.6, 0]                             # y 方向（从机身往外）按 2.6 倍拉开
            if has(nm, "外层壳"):
                off[1] += 150
            if has(nm, "外板", "立柱"):
                off[1] += 90
            if has(nm, "大带轮", "小腿", "膝轴"):
                off[0] -= 0; off[2] -= 140                             # 小腿这组往下拉开，看得见大带轮
        lab = LEG_LAB.get(nm, "")
        items.append(dict(file=os.path.relpath(it["path"], os.path.join(OUT, "leg")), color=it["color"], offset=[float(x) for x in off],
                          label=lab, printed="打印" in lab))
    return items


if __name__ == "__main__":
    jobs = [("whole", whole(), "1.25,1.35,0.95", "007A 整机爆炸图", "整机爆炸图"),
            ("leg", leg(), "1.2,1.0,0.7", "007A 腿部爆炸图（左前腿，斜前方看：从机身一层层往外）", "腿部爆炸图")]
    for key, items, cam, title, fn in jobs:
        d = os.path.join(OUT, key)
        os.makedirs(d, exist_ok=True)
        json.dump(items, open(os.path.join(d, "manifest.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=0)
        if "--no-render" in sys.argv or (len(sys.argv) > 1 and sys.argv[1] in ("whole", "leg") and sys.argv[1] != key):
            continue
        png = os.path.join(d, f"{fn}.png")
        subprocess.run([BLENDER, "--background", "--factory-startup", "--python", os.path.join(HERE, "blender", "render_exploded.py"), "--", d, png, cam],
                       check=True, stdout=subprocess.DEVNULL)
        subprocess.run([sys.executable, os.path.join(HERE, "tools_label_exploded.py"), png, title], check=True)
