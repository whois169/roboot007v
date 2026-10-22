"""身体 / 腿部爆炸图：把零件导成 STL（装配位置），写一份清单（颜色、往外拉多少、标注名字、是不是打印件），给 blender/render_exploded.py 用。

用法：python 007v/tools_exploded.py   → 007v/out/exploded/{body,leg}/*.stl + manifest.json
"""
import glob
import json
import os
import re

import cadquery as cq

import model

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "exploded")
BP = os.path.join(HERE, "out", "body_print")
HEAD_KEYS = ("耳", "转轴", "雷达", "脖子", "舵机", "铜套", "护环", "法兰螺栓", "底座螺栓", "方窗", "光环", "屏", "编织线缆", "俯仰")


def export(group, items):
    d = os.path.join(OUT, group)
    os.makedirs(d, exist_ok=True)
    for f in glob.glob(os.path.join(d, "*.stl")):
        os.remove(f)
    man = []
    for i, (name, solid, color, off, label, printed) in enumerate(items):
        fn = f"{i:03d}.stl"
        cq.exporters.export(cq.Workplane().add(solid), os.path.join(d, fn), tolerance=0.15, angularTolerance=0.3)
        man.append(dict(file=fn, name=name, color=color, offset=off, label=label, printed=printed))
    json.dump(man, open(os.path.join(d, "manifest.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=0)
    print(group, len(man), "个零件")


def step(code):
    f = glob.glob(os.path.join(BP, f"{code}_*.step"))[0]
    return cq.importers.importStep(f).val(), os.path.basename(f)[4:-5]


# ================================================================ 机身
def body_items():
    items = []
    P = {n: (s, c) for n, s, c in model.body_parts()}
    seen_label = set()

    def lab(t):
        if t in seen_label:
            return ""
        seen_label.add(t)
        return t

    def add(name, solid, color, off, label="", printed=False):
        items.append((name, solid, color, off, lab(label) if label else "", printed))

    # ---- 铝板盒子、型材
    add("左侧板", P["左侧板"][0], "alu_dk", (0, 120, 0), "A01 侧板 ×2（铝 4，黑色阳极）")
    add("右侧板", P["右侧板"][0], "alu_dk", (0, -120, 0), "A01 侧板 ×2（铝 4，黑色阳极）")
    add("底板", P["底板"][0], "alu", (0, 0, -120), "A03 底板（铝 3）")
    add("前板", P["前板"][0], "alu_dk", (150, 0, 0), "A04 前板（铝 3）")
    add("后板", P["后板"][0], "alu_dk", (-150, 0, 0), "A05 后板（急停、开关、充电口）")
    add("甲板", P["甲板"][0], "alu_dk", (0, 0, 170), "A02 甲板（铝 3）")
    for n in P:
        if n.startswith("2020横梁"):
            up = 170 if float(re.findall(r"z(\d+)", n)[-1]) > 50 else -120
            add(n, P[n][0], "alu", (0, 0, up), "2020 横梁 ×4")
        if n.startswith(("左导轨", "右导轨", "前导轨", "后导轨")):
            add(n, P[n][0], "alu", (0, 0, 245), "货斗导轨 2020")
        if n.startswith("底盘边梁"):
            add(n, P[n][0], "alu_dk", (0, 175 if "左" in n else -175, -30), "底盘边梁（铝方管）")
        if n.startswith("底盘横管"):
            add(n, P[n][0], "alu_dk", (185 if "前" in n else -185, 0, -40), "底盘横管 Φ16")
        if re.match(r"髋支座[前后][左右]$", n):
            add(n, P[n][0], "alu_dk", (0, 200 if "左" in n else -200, 0), "D31 髋支座（铝 7）")
        if re.match(r"[前后][左右]轴承座$", n):
            add(n, P[n][0], "silver", (0, 165 if "左" in n else -165, 0), "B01 髋轴承座（6001 ×2）")
    # ---- 髋关节传动（跟着侧板往外拉一半）
    for n in P:
        m = re.match(r"([前后][左右])(蜗轮减速箱|蜗轮电机|小带轮15T|大带轮45T|同步带|髋轴|磁编码器AS5600|编码器磁铁|编码器支架)$", n)
        if not m:
            continue
        sy = 1 if "左" in m.group(1) else -1
        k = m.group(2)
        lbl = {"蜗轮减速箱": "髋电机 NFP-5840（505，竖装）", "蜗轮电机": "髋电机 NFP-5840（505，竖装）", "小带轮15T": "HTD3M 带轮 15T / 45T + 同步带",
               "大带轮45T": "HTD3M 带轮 15T / 45T + 同步带", "同步带": "HTD3M 带轮 15T / 45T + 同步带", "髋轴": "B04 髋轴",
               "磁编码器AS5600": "Adafruit AS5600 编码器", "编码器磁铁": "Adafruit AS5600 编码器", "编码器支架": "P26 编码器支架"}[k]
        dy = {"蜗轮减速箱": 50, "蜗轮电机": 50, "磁编码器AS5600": 35, "编码器磁铁": 35, "编码器支架": 20}.get(k, 70)
        col = {"蜗轮减速箱": "motor", "蜗轮电机": "motor", "同步带": "black", "编码器支架": "print_dk", "磁编码器AS5600": "pcb"}.get(k, "silver")
        add(n, P[n][0], col, (0, sy * dy, 0), lbl, printed=(k == "编码器支架"))
    # ---- 电池和底板上的电源件（往下）
    add("电池", P["电池6S2P"][0], "battery", (0, 0, -60), "6S2P 电池 75×135×47")
    add("电池保护板", P["电池保护板"][0], "pcb", (0, 0, -60), "")
    for n in ("6路保险丝盒", "继电器1（总电源）", "继电器2（电机电源）"):
        add(n, P[n][0], "black", (0, 0, -60), {"6路保险丝盒": "6 路保险丝盒", "继电器1（总电源）": "继电器 ×2"}.get(n, "继电器 ×2"))
    # ---- 托板和上面的板子（往上一点）
    add("电子托板", P["电子托板"][0], "black", (0, 0, 40), "C01 电子托板（亚克力 3）")
    import body_electronics as BE
    lbl = {"树莓派5": "树莓派 5", "底盘ESP32-S3": "ESP32-S3 底盘主控", "CAN Pal": "CAN Pal", "TCA9548A": "TCA9548A",
           "5V 5.5A D36V50F5": "Pololu 降压 ×2", "5V 3.2A D36V28F5": "Pololu 降压 ×2", "12V 降压": "12V 降压", "INA226 电流计": "INA226"}
    for r in BE.TRAY:
        n = r[0]
        add(n, P[n][0], "pcb", (0, 0, 75), "BTS7960 驱动 ×4" if n.startswith("BTS") else lbl.get(n, n))
    add("深度相机", P["深度相机"][0], "black", (70, 0, 0), "深度相机（以后装）")
    for n in P:
        if n.startswith("台阶测距板"):
            add(n, P[n][0], "pcb", (230 if "前" in n else -230, 0, 0), "Pololu VL53L1X 台阶测距 ×4")
    # ---- 打印件（用 body_print 的工程版）
    lay = {"P01": (215, 0, 0), "P02": (-215, 0, 0), "P03": (0, 215, 0), "P04": (0, 215, 0), "P05": (0, -215, 0), "P06": (0, -215, 0),
           "P07": (215, 0, 170), "P08": (-215, 0, 170), "P09": (0, 215, 170), "P10": (0, 215, 170), "P11": (0, -215, 170), "P12": (0, -215, 170),
           "P13": (0, 255, 0), "P14": (0, -255, 0), "P15": (280, 0, 0), "P16": (-280, 0, 0), "P17": (0, 0, 215), "P18": (0, 0, 215),
           "P19": (0, 0, 215), "P20": (0, 0, -165), "P21": (0, 0, -165), "P22": (0, 0, 300), "P23": (0, 0, 300), "P24": (0, 0, 300), "P25": (0, 0, 300)}
    group_lbl = {"P01": "P01–P06 下层外壳（6 块）", "P07": "P07–P12 上层甲板壳（6 块）", "P13": "P13 / P14 侧挂板", "P15": "P15 前保险杠", "P16": "P16 后格栅",
                 "P17": "P17 头部平台盖", "P18": "P18 货斗底板", "P19": "P19 后检修盖", "P20": "P20 / P21 滑撬", "P22": "P22–P25 栏板"}
    col = {"P13": "print_dk", "P14": "print_dk", "P15": "print_dk", "P16": "black", "P20": "black", "P21": "black"}
    for code, off in lay.items():
        s, nm = step(code)
        g = next((k for k in ("P01", "P07", "P13", "P15", "P16", "P17", "P18", "P19", "P20", "P22") if code == k), None)
        if code in ("P02", "P03", "P04", "P05", "P06"):
            g = "P01"
        if code in ("P08", "P09", "P10", "P11", "P12"):
            g = "P07"
        if code == "P14":
            g = "P13"
        if code == "P21":
            g = "P20"
        if code in ("P23", "P24", "P25"):
            g = "P22"
        add(code, s, col.get(code, "print"), off, group_lbl.get(g, nm), printed=True)
    return items


# ================================================================ 一条腿（左前腿，腿坐标：髋轴原点，摆臂沿 +x，y 往外）
def leg_items():
    items = []
    P = {n: (s, c) for n, s, c in model.leg_parts_FL()}

    def add(name, color, off, label, printed=False, solid=None):
        items.append((name, solid if solid is not None else P[name][0], color, off, label, printed))
    add("臂座轴毂", "silver", (0, -60, 0), "B02 臂座轴毂（键接髋轴）")
    add("腿板内层", "alu_dk", (0, 0, 0), "A07 腿板（铝 6）")
    add("轮子电机GIM4310", "motor", (0, -150, 0), "轮子电机 GIM4310-10")
    add("轮轴承座", "silver", (0, -100, 0), "B07 轮轴承座兼电机座（6801 ×2）")
    add("麦轮轮轴", "steel", (0, -75, -0), "B08 轮轴（法兰 3×M3 接电机）")
    add("轮轴支座内", "alu_dk", (0, 0, 0), "")
    add("摆臂外板", "alu_dk", (0, 80, 0), "A06 外板（铝 4）")
    add("轮毂法兰", "steel", (0, 125, 0), "B09 轮毂法兰（4×M4 接麦轮）")
    for n, c, l in (("麦轮轮毂", "wheel_hub", "goBILDA 96 mm 麦轮"), ("麦轮侧板内", "silver", ""), ("麦轮侧板外", "silver", "")):
        add(n, c, (0, 175, 0), l)
    for n in P:
        if n.startswith("麦轮滚子"):
            add(n, "black", (0, 175, 0), "")
    # 打印件
    for code, off, lbl, col in (("L01", (0, 40, 0), "L01 箱形臂座外壳", "print"), ("L02", (0, -150, -110), "L02 电机护罩", "print_dk"),
                                ("L03", (0, 115, 0), "L03 摆臂外盖板", "print_dk"), ("L04", (0, 150, 0), "L04 髋部大圆盖", "print_dk"),
                                ("L05", (0, 235, 0), "L05 轮毂盖", "print_dk"), ("L06", (0, -205, -110), "L06 护罩端圈", "print_dk")):
        s, nm = step(code)
        items.append((code, s, col, off, lbl, True))
    return items


if __name__ == "__main__":
    import sys
    if "leg" not in sys.argv[1:]:
        export("body", body_items())
    export("leg", leg_items())
