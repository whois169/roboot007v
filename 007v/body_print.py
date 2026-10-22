"""机身和腿部 3D 打印件（工程版，2026-10-22）：外形用定稿外观，只加能装的东西 + 按 256×256 打印机拆块。

- 下层外壳、上层甲板壳：1.5 / 3.5 mm 的皮，贴在铝板外面（3M VHB）；超过 256 的侧面拆成两块（下层在侧挂板后面 x = 0 拆，
  上层左侧 x = -30、右侧 x = +30 拆，避开“007V”字样），接缝 2 mm，和外观上已有的分缝一样
- 黑腰不打印：就是露出来的铝板（黑色阳极），腰线灯条直接贴在上面
- 侧挂板：4 颗 M5 沉头（外观上那 4 颗螺栓），拧进侧板 A01 的 M5 螺纹孔
- 前保险杠、后格栅：背面各 4 个 M3 热熔螺母，从机身里面穿过前 / 后板拧；台阶测距的仓从背面装
- 顶面三块板：和甲板、导轨的螺丝对齐开孔（脖子底座 M5、导轨 M5、前后横梁 M5、机械臂接口）
- 滑撬：4 颗 M4 沉头从下面拧进底板
- 腿：臂座外壳 D01 加 M4（外板那面）、M3（腿板那面）热熔螺母；电机护罩 D29 里面加 4 根 M4 螺丝柱对腿板的孔

用法：python 007v/body_print.py → 007v/out/body_print/（STL = 打印摆放；STEP = 装配位置）
"""
import json
import math
import os
import sys

import cadquery as cq

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import model  # noqa: E402
from config import (ARM, ARM_PORT, ARM_PORT_XY, BED_X0, BED_X1, BODY_L, BODY_W, FRAME_GAP, FRAME_RAIL, LEG_Y1, LP0,  # noqa: E402
                    NECK_X, PROFILE, RAIL_Y, WHEEL_W, WHEEL_Y0)
from head_print import CLR, INS, csk_hole, cyl_x, insert_hole, lay_flat  # noqa: E402
from shell import TUB_TOP, V, box, cyl_y, cyl_z  # noqa: E402

OUT = os.path.join(HERE, "out", "body_print")
L2, W2 = BODY_L / 2, BODY_W / 2
SEAM = 2.0

# 甲板上的螺丝（和 mfg.py 的 A02 一致）
NECK_BOLTS = [(NECK_X - 42.0, sy * 96.0) for sy in (1, -1)] + [(NECK_X + 55.0, sy * 96.0) for sy in (1, -1)]
NECK_CABLE = [(NECK_X + 30.0, sy * 96.0) for sy in (1, -1)]
RAIL_HOLES_X = None   # 运行时从 mfg 读


def _rail_holes():
    import mfg
    H = []
    for x in mfg.RAIL_HOLES_X:
        for sy in (1, -1):
            H.append((x, sy * RAIL_Y))
    for y in (-60.0, 0.0, 60.0):
        H.append((BED_X0 + PROFILE / 2, y))
        H.append((BED_X1 - PROFILE / 2, y))
    return H


def split_x(s, x, seam=SEAM):
    """沿 x 切开，中间留 seam 宽的缝。返回 (x 小的那块, x 大的那块)。"""
    big = 1000
    a = s.cut(box(x - seam / 2, big, -big, big, -big, big))
    b = s.cut(box(-big, x + seam / 2, -big, big, -big, big))
    return a, b


def solids_by(s, key):
    return sorted(s.Solids(), key=key)


def body():
    P = {n: s for n, s, c in model.body_parts()}
    out = {}
    # ---------------- 下层外壳：前、后、左右各两块
    tub = P["外壳·下层外壳"]
    parts = tub.Solids()
    for so in parts:
        c = so.BoundingBox().center
        if c.x > L2 - 40:
            out["tub_front"] = so
        elif c.x < -(L2 - 40):
            out["tub_rear"] = so
        else:
            a, b = split_x(so, 0.0)
            tag = "L" if c.y > 0 else "R"
            out[f"tub_{tag}1"], out[f"tub_{tag}2"] = b, a          # 1 = 前半、2 = 后半
    # ---------------- 上层甲板壳：前鼻、后鼻、左右各两块（字样、小字一起打印，喷银）
    slab = P["上层甲板壳"]
    for n in ("外壳·左字样007V", "外壳·左小字", "外壳·右字样007V", "外壳·右小字"):
        slab = slab.fuse(P[n])
    for so in slab.Solids():
        bb = so.BoundingBox()
        c = bb.center
        if bb.xlen < 100 and c.x > 0:
            out["slab_front"] = so
        elif bb.xlen < 100 and c.x < 0:
            out["slab_rear"] = so
        elif bb.xlen >= 100:
            tag = "L" if c.y > 0 else "R"
            xs = -30.0 if tag == "L" else 30.0
            a, b = split_x(so, xs)
            out[f"slab_{tag}1"], out[f"slab_{tag}2"] = b, a
    # ---------------- 侧挂板：4 颗 M5 沉头
    z0 = FRAME_RAIL[2] + FRAME_GAP
    for tag, sy in (("L", 1), ("R", -1)):
        p = P[f"侧挂板{'左' if sy > 0 else '右'}"]
        for x in (-88.0, 88.0):
            for z in (z0 + 9.5, TUB_TOP - 9.5):
                p = p.cut(csk_hole((x, sy * (W2 + 7.5), z), (0, -sy, 0), "M5", 10))
        out[f"panel_{tag}"] = p
    # ---------------- 前保险杠、后格栅：背面 4 个 M3 热熔螺母
    bump = P["前保险杠"]
    for y in (-88.0, 88.0):
        for z in (25.0, 55.0):
            bump = bump.cut(insert_hole((L2, y, z), (1, 0, 0), "M3"))
    out["bumper"] = bump
    gr = P["后格栅"]
    for y in (-70.0, 70.0):
        for z in (24.0, 46.0):
            gr = gr.cut(insert_hole((-L2, y, z), (-1, 0, 0), "M3"))
    out["grille"] = gr
    # ---------------- 顶面三块板
    hp = P["头部平台盖"]
    zt = hp.BoundingBox().zmax
    for x, y in NECK_BOLTS:
        hp = hp.cut(cyl_z(x, y, CLR["M5"], zt - 30, zt + 1))
    for x, y in NECK_CABLE:
        hp = hp.cut(cyl_z(x, y, 16.0, zt - 30, zt + 1))
    for y in (-60.0, 0.0, 60.0):
        hp = hp.cut(csk_hole((185.0, y, zt), (0, 0, -1), "M5", 20))
    out["top_front"] = hp
    bed = P["货斗底板"]
    zt = bed.BoundingBox().zmax
    for x, y in _rail_holes():
        bed = bed.cut(cyl_z(x, y, CLR["M5"], zt - 30, zt + 1))
    ax, ay = ARM_PORT_XY
    for k in range(4):
        a = math.radians(45 + 90 * k)
        bed = bed.cut(cyl_z(ax + 25.0 * math.sqrt(2) * math.cos(a), ay + 25.0 * math.sqrt(2) * math.sin(a), CLR["M5"], zt - 30, zt + 1))
    bed = bed.cut(cyl_z(ax, ay, 16.0, zt - 30, zt + 1))
    out["top_bed"] = bed
    rh = P["后检修盖"]
    zt = rh.BoundingBox().zmax
    for y in (-30.0, 30.0):
        rh = rh.cut(csk_hole((-185.0, y, zt), (0, 0, -1), "M5", 20))
    out["top_rear"] = rh
    # ---------------- 滑撬：4 颗 M4 沉头从下面拧
    for tag, sy in (("L", 1), ("R", -1)):
        sk = P[f"滑撬{'左' if sy > 0 else '右'}"]
        zb = sk.BoundingBox().zmin
        for x in (-130.0, -45.0, 45.0, 130.0):
            sk = sk.cut(csk_hole((x, sy * 80.0, zb), (0, 0, 1), "M4", 10))
        out[f"skid_{tag}"] = sk
    # ---------------- 栏板、编码器支架（不用改）
    for k, n in (("fence_L", "左栏板"), ("fence_R", "右栏板"), ("fence_rear", "后栏板"), ("fence_front", "前栏板"), ("enc_bracket", "前左编码器支架")):
        out[k] = P[n]
    return out


def legs():
    P = {n: s for n, s, c in model.leg_parts_FL()}
    out = {}
    op1 = LEG_Y1 - 2
    op0 = op1 - 4
    lp1 = LP0 + 6
    arm = P["箱形臂座外壳"]
    for u, v in ((20, 14), (20, -14), (95, 14), (95, -14)):
        arm = arm.cut(insert_hole((u, op0, v), (0, -1, 0), "M4"))          # 外板 A06 的 M4 沉头拧进来
    for u in (30, 70, 110):
        for v in (17, -17):
            arm = arm.cut(insert_hole((u, lp1, v), (0, 1, 0), "M3"))       # 腿板 A07 的 M3 从里面拧进来
    out["arm_shell"] = arm
    g = P["电机护罩"]
    for X, Z in ((-24, 6), (24, 6), (-24, -20), (24, -20)):
        u, v = ARM + X * 0.574 - Z * 0.819, X * 0.819 + Z * 0.574
        g = g.fuse(cyl_y(u, v, 9.0, LP0 - 8.0, LP0 - 0.5))
        g = g.cut(insert_hole((u, LP0, v), (0, -1, 0), "M4"))              # 腿板 A07 的 4 颗 M4 从外面拧进来
    out["guard"] = g
    out["arm_cover"] = P["摆臂外盖板"]
    out["hip_cap"] = P["髋部大圆盖"]
    return out


PRINT = {   # key: (编号, 名字, 数量, 材料, 打印时朝下的面)
    "tub_front": ("P01", "下层外壳·前", 1, "ASA 浅灰白", (0, 0, -1)),
    "tub_rear": ("P02", "下层外壳·后", 1, "ASA 浅灰白", (0, 0, -1)),
    "tub_L1": ("P03", "下层外壳·左前", 1, "ASA 浅灰白", (0, 1, 0)),
    "tub_L2": ("P04", "下层外壳·左后", 1, "ASA 浅灰白", (0, 1, 0)),
    "tub_R1": ("P05", "下层外壳·右前", 1, "ASA 浅灰白", (0, -1, 0)),
    "tub_R2": ("P06", "下层外壳·右后", 1, "ASA 浅灰白", (0, -1, 0)),
    "slab_front": ("P07", "上层甲板壳·前鼻", 1, "ASA 浅灰白", (0, 0, -1)),
    "slab_rear": ("P08", "上层甲板壳·后鼻", 1, "ASA 浅灰白", (0, 0, -1)),
    "slab_L1": ("P09", "上层甲板壳·左前", 1, "ASA 浅灰白", (0, 1, 0)),
    "slab_L2": ("P10", "上层甲板壳·左后", 1, "ASA 浅灰白", (0, 1, 0)),
    "slab_R1": ("P11", "上层甲板壳·右前", 1, "ASA 浅灰白", (0, -1, 0)),
    "slab_R2": ("P12", "上层甲板壳·右后", 1, "ASA 浅灰白", (0, -1, 0)),
    "panel_L": ("P13", "侧挂板·左", 1, "ASA 深灰", (0, -1, 0)),
    "panel_R": ("P14", "侧挂板·右", 1, "ASA 深灰", (0, 1, 0)),
    "bumper": ("P15", "前保险杠", 1, "ASA 深灰", (-1, 0, 0)),
    "grille": ("P16", "后格栅", 1, "PETG 黑", (1, 0, 0)),
    "top_front": ("P17", "头部平台盖", 1, "ASA 浅灰白", (0, 0, 1)),
    "top_bed": ("P18", "货斗底板", 1, "ASA 浅灰白", (0, 0, 1)),
    "top_rear": ("P19", "后检修盖", 1, "ASA 浅灰白", (0, 0, 1)),
    "skid_L": ("P20", "滑撬·左", 1, "PETG / TPU 95A", (0, 0, -1)),
    "skid_R": ("P21", "滑撬·右", 1, "PETG / TPU 95A", (0, 0, -1)),
    "fence_L": ("P22", "侧栏板·左", 1, "PETG", (0, 1, 0)),
    "fence_R": ("P23", "侧栏板·右", 1, "PETG", (0, -1, 0)),
    "fence_rear": ("P24", "后栏板", 1, "PETG", (1, 0, 0)),
    "fence_front": ("P25", "前栏板", 1, "PETG", (-1, 0, 0)),
    "enc_bracket": ("P26", "编码器支架（左右镜像各打 2）", 4, "PETG", (0, 1, 0)),
    "arm_shell": ("L01", "箱形臂座外壳", 4, "ASA 深灰", (0, -1, 0)),
    "guard": ("L02", "电机护罩", 4, "PA12-CF / PETG-CF", (0, 1, 0)),
    "arm_cover": ("L03", "摆臂外盖板", 4, "ASA 深灰", (0, -1, 0)),
    "hip_cap": ("L04", "髋部大圆盖", 4, "ASA 深灰", (0, -1, 0)),
}


def main():
    os.makedirs(OUT, exist_ok=True)
    print("机身 …")
    P = body()
    print("腿 …")
    P.update(legs())
    info, too_big = [], []
    for k, (code, name, qty, mat, down) in PRINT.items():
        s = P[k]
        flat = lay_flat(s, down)
        b = flat.BoundingBox()
        fits = (b.xlen <= 250 and b.ylen <= 250) or (b.xlen + b.ylen) * 0.7071 <= 250
        if not fits:
            too_big.append((code, name, round(b.xlen), round(b.ylen)))
        base = os.path.join(OUT, f"{code}_{name}")
        cq.exporters.export(cq.Workplane().add(s), base + ".step")
        cq.exporters.export(cq.Workplane().add(flat), base + ".stl", tolerance=0.05, angularTolerance=0.12)
        diag = "" if (b.xlen <= 250 and b.ylen <= 250) else "（斜着放）"
        info.append(dict(code=code, name=name, qty=qty, material=mat, size=[round(b.xlen), round(b.ylen), round(b.zlen)],
                         volume_cm3=round(s.Volume() / 1000, 1), diag=bool(diag), valid=s.isValid()))
        print(f"  {code} {name} ×{qty}  {b.xlen:.0f}×{b.ylen:.0f}×{b.zlen:.0f}{diag}  {s.Volume() / 1000:.1f} cm³{'' if s.isValid() else '  ！实体有问题'}")
    json.dump(info, open(os.path.join(OUT, "parts.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("超出 256 打印范围：", too_big or "没有")


if __name__ == "__main__":
    main()
