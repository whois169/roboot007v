"""007A 机身 v10：原版方正机身（model.py v8），表面加层次——凹进去的、凸出来的、线条（参考用户给的工业机器狗图）。

每一样都有用处（用户原则：没作用的不设计）：
  髋关节轮拱：机身侧面在髋的位置挖进去一个弧形窝、窝里深色 —— 髋外摆、前后摆转动的让位
  侧装甲板：深色，凸出 3，盖住机身里的电池 / 电路板（四颗螺丝可拆），上面是“007A”
  散热格栅：装甲板前下角、尾部下方、下层底盘检修板上，一条条凹进去 —— 电机驱动、树莓派的进出风口
  斜肩：上层甲板壳顶上两条长棱切 45° × 22 —— 侧翻时先着地的是斜面（不是尖棱），棱线也是顶盖的分界
  底盘收腰：下层底盘两条底棱切 45° × 18 —— 腿往里摆、过坎时不刮机身底
  护角：上层甲板壳四个竖棱包深色橡胶护角 —— 撞墙、撞门框先碰护角
  雷达保护架：雷达外面一圈金属架 —— 倒了、撞了保护雷达
  前防撞杠：车头下面一根深色杠，台阶测距窗在杠里 —— 保护车头和测距窗
  尾部接口板：凹进去的深色板，急停、充电口在上面
用法：python 007a/body_v10.py → 007a/out/parts
"""
import json
import os

import cadquery as cq

import model as M
from model import V, box, cyl_x, cyl_y, prism_xz, prism_yz, ring_y, soft

L2, W2, Z0 = M.L2, M.W2, M.Z0
TUB_TOP, WAIST_TOP, TOP = M.TUB_TOP, M.WAIST_TOP, M.TOP
HX, HZ = M.HIP_X, M.HIP_Z
ZC = WAIST_TOP + (TOP - WAIST_TOP) / 2 - 2                    # 和 model.py 里头、字样同一个高度


def slots(x0, x1, z0, z1, n, sy, y_face, depth=3.0, along="x"):
    """一排凹进去的散热槽（每条槽宽 = 间距的一半），返回 (要挖掉的, 槽底深色件)。"""
    cut, bottoms = None, []
    for k in range(n):
        if along == "x":
            a = x0 + (x1 - x0) * (k + 0.25) / n
            b = x0 + (x1 - x0) * (k + 0.75) / n
            c = box(a, b, *sorted((y_face - sy * depth, y_face + sy * 5)), z0, z1)
            bt = box(a, b, *sorted((y_face - sy * depth, y_face - sy * (depth - 0.6))), z0, z1)
        else:                                                     # 竖排（沿 z）
            a = z0 + (z1 - z0) * (k + 0.25) / n
            b = z0 + (z1 - z0) * (k + 0.75) / n
            c = box(x0, x1, *sorted((y_face - sy * depth, y_face + sy * 5)), a, b)
            bt = box(x0, x1, *sorted((y_face - sy * depth, y_face - sy * (depth - 0.6))), a, b)
        cut = c if cut is None else cut.fuse(c)
        bottoms.append(bt)
    return cut, bottoms


def body_parts():
    src = M.body_parts()
    P, shells = [], {}
    for n, s, c in src:
        if n in ("上层甲板壳", "下层底盘", "黑腰"):
            shells[n] = [s, c]
        elif n.startswith("字样"):
            continue                                              # 字样挪到装甲板上（浅色）
        elif n.startswith("测距窗"):
            continue                                              # 测距窗挪进防撞杠
        else:
            P.append((n, s, c))
    up, tub, waist = shells["上层甲板壳"][0], shells["下层底盘"][0], shells["黑腰"][0]
    for sy in (1, -1):
        ys = sy * (W2 + 2)                                        # 上层甲板壳侧面
        # ---- 髋窝：髋盖后面一圈浅窝
        for fx in (1, -1):                                         # 浅窝：半径 66、深 5，比髋盖大一圈（髋转动的让位），窝底中灰
            y0, y1 = sorted((sy * (W2 - 3), sy * (W2 + 20)))
            pit = cyl_y(fx * HX, HZ, 124, y0, y1)
            up, tub, waist = up.cut(pit), tub.cut(pit), waist.cut(pit)
            P.append((f"髋窝底{fx}{sy}", ring_y(fx * HX, HZ, 124, 100, *sorted((sy * (W2 - 4), sy * (W2 - 3)))), "skel"))
        # ---- 侧装甲板：深色、凸出 3、前后端 45° 斜（和机身切角一致），四颗螺丝
        ax0, ax1, az0, az1 = -72.0, 72.0, WAIST_TOP + 6, TOP - 20
        c45 = 18.0
        armor = prism_xz([(ax0, az0), (ax1 - c45, az0), (ax1, az0 + c45), (ax1, az1), (ax0 + c45, az1), (ax0, az1 - c45)],
                         *sorted((ys, ys + sy * 3)))
        # 装甲板前下角的进风格栅（凹 2）
        g_cut, g_bt = slots(ax1 - 58, ax1 - 10, az0 + 8, az0 + 30, 5, sy, ys + sy * 3, depth=2.4)
        armor = armor.cut(g_cut)
        P.append((f"侧装甲板{sy}", soft(armor, 0.8), "skel"))
        P += [(f"装甲进风槽底{sy}{i}", b, "skel_dk") for i, b in enumerate(g_bt)]
        for x, z in ((ax0 + 8, az0 + 8), (ax1 - 26, az1 - 8), (ax0 + 26, az1 - 8), (ax1 - 8, az0 + 26)):
            P.append((f"装甲螺丝{sy}{x:.0f}{z:.0f}", cyl_y(x, z, 6, *sorted((ys + sy * 3, ys + sy * 4))), "metal"))
        txt = cq.Workplane("XZ").text("007A", 26, 0.8, halign="center", valign="center", fontPath="C:/Windows/Fonts/bahnschrift.ttf").val()
        if sy > 0:
            txt = txt.rotate(V(0, 0, 0), V(0, 0, 1), 180).translate(V(-8, ys + 3.8, ZC + 8))
        else:
            txt = txt.translate(V(-8, ys - 3.8, ZC + 8))
        P.append((f"字样{sy}", txt, "shell"))
        # ---- 下层底盘检修板上的散热格栅（电池舱、驱动的出风）
        yt = sy * (W2 - 7.5)
        t_cut, t_bt = slots(-60, 60, Z0 + 21, TUB_TOP - 7, 7, sy, yt, depth=2.0)
        tub = tub.cut(t_cut)
        P += [(f"底盘出风槽底{sy}{i}", b, "skel_dk") for i, b in enumerate(t_bt)]
    # ---- 尾部：凹进去的接口板（急停、充电口在上面）+ 下方出风格栅
    rp_x = -L2 - 4
    up = up.cut(box(rp_x - 2, rp_x + 6, -70, 70, ZC - 24, ZC + 24))
    P.append(("尾部接口板", box(rp_x + 5, rp_x + 6, -70, 70, ZC - 24, ZC + 24), "skel_dk"))
    r_cut = None
    for k in range(4):
        z = Z0 + 18 + k * 5.5
        c = box(-L2 + 1, -L2 + 8, -70, 70, z, z + 2.5)
        r_cut = c if r_cut is None else r_cut.fuse(c)
    tub = tub.cut(r_cut)
    P.append(("尾部出风槽底", box(-L2 + 7, -L2 + 8, -70, 70, Z0 + 18, Z0 + 38), "skel_dk"))
    # ---- 前防撞杠（深色，凸出 14），台阶测距窗在杠里
    bumper = prism_xz([(L2 - 6, Z0 + 18), (L2 + 8, Z0 + 24), (L2 + 8, Z0 + 42), (L2 - 6, Z0 + 46)], -W2 + 16, W2 - 16)
    for sy in (1, -1):
        bumper = bumper.cut(box(L2 + 4, L2 + 10, sy * 70 - 9, sy * 70 + 9, Z0 + 29, Z0 + 37))
        P.append((f"测距窗{sy}", box(L2 + 4, L2 + 5, sy * 70 - 9, sy * 70 + 9, Z0 + 29, Z0 + 37), "glass"))
    P.append(("前防撞杠", soft(bumper, 1.5), "skel_dk"))
    # ---- 雷达保护架（雷达在 (L2-46, -(W2-40))）
    lx, ly = L2 - 46, -(W2 - 50)
    for dx in (-30.0, 30.0):
        P.append((f"雷达架立柱{dx:.0f}", cq.Solid.makeCylinder(4, 44, V(lx + dx, ly, TOP + 10)), "metal"))
    P.append(("雷达架顶环", ring_y(lx, TOP + 54, 68, 60, ly - 4, ly + 4).rotate(V(lx, ly, TOP + 54), V(lx + 1, ly, TOP + 54), 90), "metal"))
    # ---- 斜肩（顶上两条长棱 45° × 22）、底盘收腰（底下两条长棱 45° × 18）
    SH, BL = 22.0, 18.0
    for sy in (1, -1):
        yo = sy * (W2 + 3)
        up = up.cut(prism_yz([(yo - sy * (SH + 1), TOP + 1), (yo, TOP + 1), (yo, TOP - SH - 1 + 1)], -L2 - 10, L2 + 10))
        yt = sy * (W2 - 5)
        tub = tub.cut(prism_yz([(yt - sy * (BL + 1), Z0 - 1), (yt, Z0 - 1), (yt, Z0 + BL)], -L2 - 10, L2 + 10))
    # ---- 护角：上层甲板壳四个竖棱（俯视切角面）外面包一层深色橡胶，前后各伸出 16
    zg0, zg1 = WAIST_TOP + M.CH_BIG, TOP - M.CH_BIG
    for sx in (1, -1):
        for sy in (1, -1):
            X, Y, ch, t = sx * (L2 + 4), sy * (W2 + 2), M.CH_PLAN, 3.0
            pts = [(X - sx * 30, Y), (X - sx * ch, Y), (X, Y - sy * ch), (X, Y - sy * 16),
                   (X + sx * t, Y - sy * 16), (X + sx * t, Y - sy * (ch - 1.2)), (X - sx * (ch - 1.2), Y + sy * t), (X - sx * 30, Y + sy * t)]
            g = cq.Workplane("XY").polyline(pts).close().extrude(zg1 - zg0).val().translate(V(0, 0, zg0))
            P.append((f"护角{sx}{sy}", soft(g, 0.8), "rubber"))
    P += [("上层甲板壳", up, "shell"), ("下层底盘", tub, "shell"), ("黑腰", waist, "skel_dk")]
    return P


def all_parts():
    P = body_parts()
    for fx in (1, -1):
        for sy in (1, -1):
            P += [(n, s, c) for n, s, c in M.leg_parts(fx, sy) if not n.endswith("髋座")]
    return P


if __name__ == "__main__":
    out = M.OUT
    os.makedirs(out, exist_ok=True)
    for f in os.listdir(out):
        os.remove(os.path.join(out, f))
    man = []
    for i, (n, s, c) in enumerate(all_parts()):
        fn = f"{i:03d}.stl"
        cq.exporters.export(cq.Workplane().add(s), os.path.join(out, fn), tolerance=0.08, angularTolerance=0.15)
        man.append(dict(file=fn, name=n, color=c))
    json.dump(man, open(os.path.join(out, "manifest.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=0)
    print(f"{len(man)} 个零件（v12：压低 + 斜肩 + 收腰 + 护角）")
