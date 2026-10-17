"""导出给 SolidWorks 用的文件（行驶姿态，按运动部件分组）：
  007v/out/solidworks/机械狗007v_装配体.step   一个装配体，11 个子装配 = 11 个运动部件（机身、屏幕头、4 个摆臂、4 个麦轮）
  007v/out/solidworks/部件/<名字>.step          每个运动部件单独一个文件，原点就在它的转轴上
  007v/out/solidworks/关节表.csv                 9 个转轴：经过的点、方向、转动范围、几个姿态的角度
  007v/out/solidworks/部件质量.csv               每个部件的估算质量和重心（零件质量表在 config.py）
每个转轴两边各有一个 Φ6 的“铰链标记”小圆柱（父部件一个、子部件一个），在 SolidWorks 里选这两个圆柱面做“同轴心”、
选它们的端面做“重合”，就是一个铰链配合。说明见 docs/机械狗007v/SolidWorks和Blender使用说明.md。
用法：python 007v/export_sw.py
"""
import csv
import math
import os
import re
import sys

import cadquery as cq
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build as B  # noqa: E402
import model as M  # noqa: E402
from config import *  # noqa: E402,F401,F403
from look import rgb  # noqa: E402

V = cq.Vector
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "solidworks")
W2 = BODY_W / 2
WY1 = W2 + WHEEL_GAP + WHEEL_W                     # 轮子外侧（= 轮子电机内侧）
HUB0 = W2 + BEARING_HOUSING_W + 1
POSE = B.POSES["deploy"]                           # 行驶姿态
CN = {"body": "机身", "head": "屏幕头", **{k: f"{v[2]}摆臂" for k, v in B.LEGS.items()}, **{k + "w": f"{v[2]}麦轮" for k, v in B.LEGS.items()}}
SUB = {g: f"{i + 1:02d}_{n}部件" for i, (g, n) in enumerate(CN.items())}     # 子装配名字（不能和零件重名）


def step_unicode(path):
    r"""STEP 文件里的中文按标准写成 \X2\hhhh\X0\（SolidWorks 才不会显示乱码）。
    OCC 写出来的中文是“UTF-8 字节当成 Latin-1 再存一遍”，先还原成真正的中文。"""
    def enc(m):
        s = m.group(0)
        try:
            s = s.encode("latin-1").decode("utf-8")
        except (UnicodeEncodeError, UnicodeDecodeError):
            pass
        return "\\X2\\" + "".join(f"{ord(ch):04X}" for ch in s) + "\\X0\\"
    with open(path, encoding="utf-8") as f:
        t = f.read()
    t = re.sub(r"[^\x00-\x7f]+", enc, t)
    with open(path, "w", encoding="ascii", newline="") as f:
        f.write(t)


def marker(c, y0, y1):
    """Φ6 铰链标记：沿 Y 的小圆柱。c = (x, z)。"""
    lo, hi = min(y0, y1), max(y0, y1)
    return M.cyl_y(c[0], c[1], 6.0, lo, hi)


def groups():
    """{组: [(名字, 实体(组局部坐标), 颜色)]}"""
    G = {g: [] for g in CN}
    for p in B.PARTS:
        if p.c == "envelope":
            continue
        if p.g in B.LEGS and B.is_wheel(p.name):
            fx = B.LEGS[p.g][0]
            G[p.g + "w"].append((p.name, p.s.translate(V(-fx * ARM, 0, 0)), p.c))
        else:
            G[p.g].append((p.name, p.s, p.c))
    for k, (fx, sy, cn) in B.LEGS.items():
        y_out = sy * (W2 + BEARING_HOUSING_W)
        G["body"].append((f"铰链标记-{cn}髋（机身侧）", marker((fx * HIP_X, HIP_Z), y_out - sy * 2, y_out), "trim"))
        G[k].append((f"铰链标记-{cn}髋（摆臂侧）", marker((0, 0), sy * HUB0, sy * (HUB0 + 2)), "trim"))
        G[k].append((f"铰链标记-{cn}轮（摆臂侧）", marker((fx * ARM, 0), sy * WY1, sy * (WY1 + 2)), "trim"))
        G[k + "w"].append((f"铰链标记-{cn}轮（麦轮侧）", marker((0, 0), sy * (WY1 - 2), sy * WY1), "trim"))
    G["body"].append(("铰链标记-头（左耳轴座侧）", marker((NECK_X, B.PIVOT_Z), TRUNNION_Y0 - 2, TRUNNION_Y0), "trim"))
    G["head"].append(("铰链标记-头（屏幕头侧）", marker((0, 0), HEAD_W / 2 - 2, HEAD_W / 2), "trim"))
    return G


def location(g):
    """组在行驶姿态下的位置（cq.Location）。"""
    bz = POSE["bz"]
    if g == "body":
        return cq.Location(V(0, 0, bz))
    if g == "head":
        return cq.Location(V(NECK_X, 0, B.PIVOT_Z + bz)) * cq.Location(V(0, 0, 0), V(0, 1, 0), POSE["pitch"])
    k = g[:2]
    fx = B.LEGS[k][0]
    leg = cq.Location(V(fx * HIP_X, 0, HIP_Z + bz)) * cq.Location(V(0, 0, 0), V(0, 1, 0), fx * POSE["theta"])
    return leg if g == k else leg * cq.Location(V(fx * ARM, 0, 0))


def main():
    os.makedirs(os.path.join(OUT, "部件"), exist_ok=True)
    G = groups()
    top = cq.Assembly(name="机械狗007v")
    for g, parts in G.items():
        sub = cq.Assembly(name=SUB[g])
        for name, s, c in parts:
            sub.add(s, name=name, color=cq.Color(*rgb(c)))
        sub.save(os.path.join(OUT, "部件", f"{SUB[g]}.step"))
        step_unicode(os.path.join(OUT, "部件", f"{SUB[g]}.step"))
        top.add(sub, name=SUB[g], loc=location(g))
    top.save(os.path.join(OUT, "机械狗007v_装配体.step"))
    step_unicode(os.path.join(OUT, "机械狗007v_装配体.step"))

    # 关节表
    rows = []
    for k, (fx, sy, cn) in B.LEGS.items():
        rows.append([f"{cn}髋", "机身", f"{cn}摆臂", f"{fx * HIP_X:g}", "任意", f"{HIP_Z:g}", f"0, {fx}, 0",
                     "正 = 轮子往下（摆臂往下转）", f"{STANCE_DEG:g}", f"{B.THETA_FOLD:g}", f"{B.THETA_FOLD:g} ～ 100", "蜗轮自锁 + 3:1 同步带"])
        rows.append([f"{cn}轮", f"{cn}摆臂", f"{cn}麦轮", f"摆臂坐标 {fx * ARM:g}", "任意", "0", "0, 1, 0",
                     "正 = 往前滚", "—", "—", "连续转", "GIM4310-10"])
    rows.append(["头俯仰", "机身", "屏幕头", f"{NECK_X:g}", "任意", f"{B.PIVOT_Z:g}", "0, 1, 0", "正 = 低头",
                 f"{HEAD_PITCH['level']:g}", f"{HEAD_PITCH['fold']:g}", f"{HEAD_PITCH['up']:g} ～ {HEAD_PITCH['down']:g}", "STS3215"])
    with open(os.path.join(OUT, "关节表.csv"), "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["关节", "父部件", "子部件", "轴经过的点 X", "Y", "Z", "轴方向", "角度正方向", "行驶角 °", "折叠角 °", "范围 °", "电机"])
        w.writerows(rows)
        w.writerow([])
        w.writerow(["坐标：X 向前、Y 向左、Z 向上，mm；原点在机身底板下表面中心（零位：摆臂水平朝前或朝后、头平视）。"])
        w.writerow([f"行驶姿态：机身底离地 {POSE['bz']:.1f} mm；摆臂角是相对水平的角度。"])

    # 部件质量（按 config.py 的零件重量表）
    with open(os.path.join(OUT, "部件质量.csv"), "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["部件", "质量 kg", "重心 X", "重心 Y", "重心 Z（部件自己的坐标，mm）"])
        for g in CN:
            ps = []
            for p in B.PARTS:
                in_g = (p.g == g) if g in ("body", "head") else (p.g == g[:2] and B.is_wheel(p.name) == g.endswith("w"))
                if in_g and p.m > 0:
                    off = np.array([B.LEGS[g[:2]][0] * ARM, 0, 0]) if g.endswith("w") else np.zeros(3)
                    ps.append((p.m, p.cen - off))
            if g == "body":
                ps.append((MASS_SHELLS, np.array([0, 0, BODY_H / 2])))
            m = sum(x for x, _ in ps)
            c = sum(x * cc for x, cc in ps) / m
            w.writerow([CN[g], f"{m:.3f}", f"{c[0]:.1f}", f"{c[1]:.1f}", f"{c[2]:.1f}"])
    n = sum(len(v) for v in G.values())
    print(f"导出 007v/out/solidworks/：装配体 1 个、部件 {len(G)} 个、零件 {n} 个、关节表、部件质量")


if __name__ == "__main__":
    main()
