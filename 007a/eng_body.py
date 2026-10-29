"""007A 第 2 步：机身骨架 + 4 条腿 + 电池 + 电子件，干涉检查（照 007v 的 build.py 思路）。

机身坐标：原点在机身中心、地面上，x 前、y 左、z 上（和 model.py 一样）。
骨架（铝板激光切割）：
  A01 底板 3 mm（整块，四个角让开腿的摆动空间）
  A02 外摆隔板 5 mm × 2（x = ±53，左右两个外摆电机拧在上面；前后各一块）
  A03 侧梁 3 mm × 2（两块隔板之间，y = ±116，内侧有电子托板的插槽）
  A04 纵梁 4 mm × 4（隔板往前 / 往后到端板，y = ±18，两条腿中间）
  A05 前端板 / A06 后端板 4 mm（头、相机罩、防撞杠、尾部接口板都拧在上面）
  A07 顶板 3 mm（货台底下，货物的力走这里；四周 M3 孔拧上层外壳）
中间（两块隔板之间）：下层电池（6S3P，65 × 129 × 70，打印电池托 + 两条绑带），两边是继电器、保险丝盒、分流器；
上层亚克力电子托板 C01（插在侧梁的槽里）：树莓派 5、USB 转 4 路 485、USB 集线器、三块降压、电平转换、IMU（正中间）。
每条腿：外摆 ±20° 扫过的空间（加 3 mm 余量）不能碰任何骨架 / 电池 / 电子件。
用法：python 007a/eng_body.py → 007a/out/eng_parts（给 blender/render.py）、007a/out/eng_report.txt、docs/机械狗007A/骨架_俯视.png
"""
import json
import math
import os

import cadquery as cq

import eng_leg as E
import model as M
from model import V, box, cyl_x, cyl_y, soft

HERE = os.path.dirname(os.path.abspath(__file__))
L2, W2, Z0, TOP, HX, HZ = M.L2, M.W2, M.Z0, M.TOP, M.HIP_X, M.HIP_Z
HAA_SWING = 20.0                                   # 外摆 ±20°
MARGIN = 3.0

# ---------------------------------------------------------------- 骨架尺寸
XB = HX + E.CAV_R * 0 - (E.CAV_R + 4.0) - E.MH     # 外摆电机背面 = 隔板外侧面（前腿那边 x = 55.7）
T_BH, T_BOT, T_SIDE, T_BEAM, T_END, T_TOP = 5.0, 3.0, 3.0, 4.0, 4.0, 3.0
Z_BOT = Z0 + 2.0                                   # 底板下表面（下层外壳里面）
Z_TOPP = TOP - 6.0 - T_TOP                         # 顶板下表面（货台凹 6 底下；髋座外摆 20° 最高到 z 离底 130）
Y_SR = 113.0                                       # 侧梁内侧面
Y_BEAM = 16.0                                      # 纵梁内侧面
X_END = L2 - 14.0                                  # 端板内侧面（端板外面是头 / 尾部接口板）
BAT = (65.0, 129.0, 70.0)                          # 电池 6S3P 21700：x × y × z
TRAY_Z = Z_BOT + T_BOT + BAT[2] + 8.0              # 电子托板下表面

# 托板上的板子：(名字, x 中心, y 中心, 长 x, 宽 y, 高, 状态)
BOARDS = [
    ("树莓派 5（带散热）", 0.0, 67.0, 56.0, 85.0, 22.0, "已确认"),
    ("USB 转 4 路 485", 0.0, -40.0, 70.0, 45.0, 15.0, "★要图"),
    ("USB 集线器", -15.0, -90.0, 40.0, 30.0, 12.0, "到货实测"),
    ("12V 5A 降压（灯、舵机）", 25.0, -90.0, 26.0, 20.0, 15.0, "到货实测"),
    ("5V 5.5A 降压（树莓派）", -32.0, 11.0, 25.4, 25.4, 9.5, "已确认"),
    ("5V 3.2A 降压（传感器）", 32.0, 4.0, 20.3, 17.8, 8.0, "已确认"),
    ("IMU 超核 HI13（正中间）", 0.0, 0.0, 24.0, 26.0, 12.0, "★要图"),
    ("电平转换", -38.0, -10.0, 15.0, 15.0, 4.0, "到货实测"),
]
# 电池两边（底板上）
SIDE_PARTS = [
    ("电机电源继电器 80A（带座）", 0.0, 90.0, 40.0, 40.0, 45.0),
    ("保险丝盒", 0.0, -90.0, 46.0, 40.0, 30.0),
]


def leg_parts_world(fx, sy, haa_deg=0.0):
    """一条腿放到机身上：fx = 1 前 / -1 后，sy = 1 左 / -1 右；haa_deg = 外摆角（+ = 脚往外）。返回 [(名字, 实体, 颜色, 组)]。"""
    P, knee, foot = E.leg_module()
    out = []
    for n, s, c, g in P:
        if n.startswith("外摆电机座"):
            continue                                               # 机身上用 A02 外摆隔板代替
        if fx < 0 and g in ("机身", "外摆"):
            s = s.mirror("YZ")                                     # 后腿：外摆电机在髋前面（朝机身中间）
        if g in ("外摆", "大腿", "小腿") and haa_deg:
            s = s.rotate(V(0, E.Y_HAA, 0), V(1, E.Y_HAA, 0), haa_deg)   # 绕外摆轴（x 方向）转；+ 角让 z 下面的点往外（+y）
        s = s.translate(V(fx * HX, 0, HZ))
        if sy < 0:
            s = s.mirror("XZ")
        out.append((n, s, c, g))
    return out


def sweep_envelope(fx, sy):
    """髋座 + 髋电机 + 大腿杯 / 膝电机 在外摆 ±20° 里扫过的空间（加余量），机身坐标。"""
    r = E.CUP_OD / 2 + MARGIN
    xa1 = -(E.CAV_R + 4.0)
    core = cyl_y(0, 0, 2 * r, E.Y_SIDE[0] - MARGIN, E.Y_TIN[0])           # 髋电机、杯、膝电机（绕髋轴转，是圆柱）
    seat = box(xa1 - MARGIN, -xa1 - 4 + MARGIN, E.Y_SIDE[0] - MARGIN, E.Y_HFE[1] + 6 + MARGIN, -55 - MARGIN, 55 + MARGIN)
    base = core.fuse(seat)
    env = None
    for a in (-HAA_SWING, -10.0, 0.0, 10.0, HAA_SWING):
        s = base.rotate(V(0, E.Y_HAA, 0), V(1, E.Y_HAA, 0), a) if a else base
        env = s if env is None else env.fuse(s)
    if fx < 0:
        env = env.mirror("YZ")
    env = env.translate(V(fx * HX, 0, HZ))
    return env if sy > 0 else env.mirror("XZ")


def frame_parts(envs):
    P = []
    # A01 底板
    bot = box(-X_END, X_END, -Y_SR - T_SIDE, Y_SR + T_SIDE, Z_BOT, Z_BOT + T_BOT)
    for e in envs:
        bot = bot.cut(e)
    for x in (-X_END + 20, -60.0, 60.0, X_END - 20):                 # 外壳卡扣孔（下层外壳的卡舌插进来）
        for y in (-Y_SR + 8, Y_SR - 8):
            bot = bot.cut(box(x - 6, x + 6, y - 1.5, y + 1.5, Z_BOT - 1, Z_BOT + T_BOT + 1))
    P.append(("A01 底板（铝 3）", bot, "metal"))
    # A02 外摆隔板 ×2：外摆电机背面拧在上面（R42.5 × 8 孔 + 中间 Φ40 过线孔）
    for fx in (1, -1):
        x0 = fx * XB
        bh = box(*sorted((x0, x0 - fx * T_BH)), -Y_SR - T_SIDE, Y_SR + T_SIDE, Z_BOT + T_BOT, Z_TOPP)
        for sy in (1, -1):
            yc = sy * E.Y_HAA
            bh = bh.cut(cyl_x(yc, HZ, 40, -300, 300).intersect(box(*sorted((x0 - fx * 10, x0 + fx * 5)), -200, 200, 0, 600)))
            for k in range(E.BACK_N):
                a = 2 * math.pi * k / E.BACK_N
                bh = bh.cut(cyl_x(yc + E.BACK_R * math.cos(a), HZ + E.BACK_R * math.sin(a), 3.4, -300, 300).intersect(
                    box(*sorted((x0 - fx * 10, x0 + fx * 5)), -200, 200, 0, 600)))
        bh = bh.cut(box(*sorted((x0 - fx * 10, x0 + fx * 5)), -12, 12, Z_BOT + T_BOT + 20, Z_BOT + T_BOT + 50))   # 中间走线孔（前后的线过来）
        P.append((f"A02 外摆隔板{'前' if fx > 0 else '后'}（铝 5）", bh, "metal"))
    # A03 侧梁 ×2（隔板之间），内侧两道托板插槽
    for sy in (1, -1):
        y0, y1 = sorted((sy * Y_SR, sy * (Y_SR + T_SIDE)))
        sr = box(-XB + T_BH, XB - T_BH, y0, y1, Z_BOT + T_BOT, Z_TOPP)
        sr = sr.cut(box(-XB + T_BH + 8, XB - T_BH - 8, y0 - 1, y1 + 1, Z_BOT + 20, Z_BOT + 60))       # 减重 + 绑带穿过去
        P.append((f"A03 侧梁{'左' if sy > 0 else '右'}（铝 3）", sr, "metal"))
        P.append((f"托板插槽导轨{'左' if sy > 0 else '右'}（打印）", box(-XB + T_BH, XB - T_BH, *sorted((sy * (Y_SR - 6), sy * Y_SR)), TRAY_Z - 3, TRAY_Z)
                  .fuse(box(-XB + T_BH, XB - T_BH, *sorted((sy * (Y_SR - 6), sy * Y_SR)), TRAY_Z + 3, TRAY_Z + 6)), "skel_dk"))
    # A04 纵梁 ×4
    for fx in (1, -1):
        for sy in (1, -1):
            x0, x1 = sorted((fx * XB, fx * X_END))
            y0, y1 = sorted((sy * Y_BEAM, sy * (Y_BEAM + T_BEAM)))
            bm = box(x0, x1, y0, y1, Z_BOT + T_BOT, Z_TOPP)
            for e in envs:
                bm = bm.cut(e)
            P.append((f"A04 纵梁{'前' if fx > 0 else '后'}{'左' if sy > 0 else '右'}（铝 4）", bm, "metal"))
    # A05 / A06 端板
    for fx, nm in ((1, "A05 前端板"), (-1, "A06 后端板")):
        x0, x1 = sorted((fx * X_END, fx * (X_END + T_END)))
        ep = box(x0, x1, -Y_SR, Y_SR, Z_BOT + T_BOT, Z_TOPP)
        for e in envs:
            ep = ep.cut(e)
        if fx > 0:
            ep = ep.cut(box(x0 - 1, x1 + 1, -50, 50, Z0 + 16, Z0 + 50))   # 深度相机、探照灯、热像仪的线从这里进
            ep = ep.cut(box(x0 - 1, x1 + 1, -60, 60, Z0 + 56, Z0 + 120))  # 头（屏幕 + 抬头舵机）的窗口
        else:
            ep = ep.cut(box(x0 - 1, x1 + 1, -70, 70, Z0 + 69, Z0 + 117))  # 尾部接口板（急停、开关、充电口）
        P.append((f"{nm}（铝 4）", ep, "metal"))
    # A07 顶板（货台底下），四周 M3 孔拧上层外壳
    tp = box(-X_END, X_END, -Y_SR - T_SIDE, Y_SR + T_SIDE, Z_TOPP, Z_TOPP + T_TOP)
    for fx in (1, -1):                                               # 四个髋上面开方口（髋座外摆时从这里过），口子四角倒圆
        for sy in (1, -1):
            x0, x1 = sorted((fx * (XB + 2), fx * (X_END + 10)))         # 扫过的空间 x 94.8–221.2、y ≥ 22.9（sweep_envelope 量的）；切通到边，不留细条
            y0, y1 = sorted((sy * (Y_BEAM + T_BEAM + 1), sy * (Y_SR + T_SIDE + 1)))
            tp = tp.cut(box(x0, x1, y0, y1, Z_TOPP - 1, Z_TOPP + T_TOP + 1).edges("|Z").fillet(6) if False else box(x0, x1, y0, y1, Z_TOPP - 1, Z_TOPP + T_TOP + 1))
    for x in (-X_END + 12, -XB, 0.0, XB, X_END - 12):
        for y in (-Y_SR + 6, Y_SR - 6):
            tp = tp.cut(cyl_x(0, 0, 1, 0, 1).translate(V(0, 0, 0)) if False else cq.Solid.makeCylinder(1.7, 10, V(x, y, Z_TOPP - 1)))
    P.append(("A07 顶板（铝 3）", tp, "metal"))
    return P


def inside_parts():
    P = []
    bz = Z_BOT + T_BOT
    P.append(("电池 6S3P（65 × 129 × 70）", box(-BAT[0] / 2, BAT[0] / 2, -BAT[1] / 2, BAT[1] / 2, bz + 3, bz + 3 + BAT[2]), "rubber"))
    cr = box(-BAT[0] / 2 - 4, BAT[0] / 2 + 4, -BAT[1] / 2 - 4, BAT[1] / 2 + 4, bz, bz + 25).cut(
        box(-BAT[0] / 2, BAT[0] / 2, -BAT[1] / 2, BAT[1] / 2, bz + 3, bz + 30))
    for x in (-18.0, 18.0):                                          # 两条绑带的卡槽
        cr = cr.cut(box(x - 13, x + 13, -BAT[1] / 2 - 5, BAT[1] / 2 + 5, bz + 6, bz + 9))
    P.append(("电池托（打印 PETG，两道绑带槽）", cr, "skel_dk"))
    for n, x, y, sx, sy_, sz in SIDE_PARTS:
        P.append((n, box(x - sx / 2, x + sx / 2, y - sy_ / 2, y + sy_ / 2, bz, bz + sz), "skel"))
    P.append(("C01 电子托板（亚克力 3，插在侧梁槽里）", box(-XB + T_BH + 1, XB - T_BH - 1, -Y_SR, Y_SR, TRAY_Z, TRAY_Z + 3), "glass"))
    for n, x, y, sx, sy_, sz, st in BOARDS:
        P.append((n, box(x - sx / 2, x + sx / 2, y - sy_ / 2, y + sy_ / 2, TRAY_Z + 3 + 4, TRAY_Z + 7 + sz), "cyan" if "IMU" in n else "skel"))
    return P


def vol(s):
    try:
        return s.Volume()
    except Exception:
        return 0.0


def interference(groups):
    """groups: [(组名, [(名字, 实体)])]；两两查，体积 > 1 mm³ 算撞。"""
    hits = []
    flat = [(g, n, s) for g, items in groups for n, s in items]
    for i in range(len(flat)):
        for j in range(i + 1, len(flat)):
            gi, ni, si = flat[i]
            gj, nj, sj = flat[j]
            if gi == gj:
                continue
            bi, bj = si.BoundingBox(), sj.BoundingBox()
            if bi.xmax < bj.xmin or bj.xmax < bi.xmin or bi.ymax < bj.ymin or bj.ymax < bi.ymin or bi.zmax < bj.zmin or bj.zmax < bi.zmin:
                continue
            v = vol(si.intersect(sj))
            if v > 1.0:
                hits.append((ni, nj, v))
    return hits


def layout_png(path):
    """两层布置图（只用参数，跑得快）：下层（底板上：电池、继电器、保险丝盒）、上层（电子托板上的板子）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
    fig, axs = plt.subplots(1, 2, figsize=(16, 7.4))
    for ax, title in zip(axs, (f"下层（底板上，离机身底 {Z_BOT + T_BOT - Z0:.0f}）", f"上层（亚克力托板上，离机身底 {TRAY_Z - Z0:.0f}，能放高 {Z_TOPP - TRAY_Z - 7:.0f}）")):
        ax.add_patch(Rectangle((-L2, -W2), 2 * L2, 2 * W2, fc="#f4f5f6", ec="k"))
        for fx in (1, -1):
            for sy in (1, -1):
                x0, x1 = sorted((fx * XB, fx * X_END))
                y0, y1 = sorted((sy * (Y_BEAM + T_BEAM), sy * W2))
                ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc="#ffe3e3", ec="r", ls="--"))
                ax.text((x0 + x1) / 2, (y0 + y1) / 2, "腿（髋、膝、外摆电机\n+ 外摆 ±20° 的空间）", ha="center", va="center", fontsize=8, color="r")
            ax.add_patch(Rectangle((min(fx * XB, fx * (XB - T_BH)), -Y_SR - T_SIDE), T_BH, 2 * (Y_SR + T_SIDE), fc="#6b7680"))
            ax.add_patch(Rectangle((min(fx * X_END, fx * (X_END + T_END)), -Y_SR), T_END, 2 * Y_SR, fc="#6b7680"))
            for sy in (1, -1):
                ax.add_patch(Rectangle((min(fx * XB, fx * X_END), min(sy * Y_BEAM, sy * (Y_BEAM + T_BEAM))), abs(X_END - XB), T_BEAM, fc="#6b7680"))
        for sy in (1, -1):
            ax.add_patch(Rectangle((-XB, min(sy * Y_SR, sy * (Y_SR + T_SIDE))), 2 * XB, T_SIDE, fc="#6b7680"))
        ax.text(XB + 2, -W2 + 4, "A02 外摆隔板", fontsize=8)
        ax.text(X_END - 60, -W2 + 4, "A05 前端板", fontsize=8)
        ax.set_title(title, fontsize=12)
        ax.set_xlim(-L2 - 8, L2 + 8)
        ax.set_ylim(-W2 - 8, W2 + 8)
        ax.set_aspect("equal")
        ax.text(L2 - 4, W2 - 4, "前 →", ha="right", va="top", fontsize=10)
    a = axs[0]
    a.add_patch(Rectangle((-BAT[0] / 2, -BAT[1] / 2), BAT[0], BAT[1], fc="#f2c14e", ec="k"))
    a.text(0, 0, "电池 6S3P\n65 × 129 × 70\n（两条绑带）", ha="center", va="center", fontsize=9)
    for n, x, y, sx, sy_, sz in SIDE_PARTS:
        a.add_patch(Rectangle((x - sx / 2, y - sy_ / 2), sx, sy_, fc="#7fb3d5", ec="k"))
        a.text(x, y, n.split("（")[0].replace("电机电源", "电机电源\n"), ha="center", va="center", fontsize=8)
    a = axs[1]
    for n, x, y, sx, sy_, sz, st in BOARDS:
        a.add_patch(Rectangle((x - sx / 2, y - sy_ / 2), sx, sy_, fc="#7fb3d5" if "IMU" not in n else "#58d0e0", ec="k"))
        a.text(x, y, n.split("（")[0] + ("\n★" if st == "★要图" else ""), ha="center", va="center", fontsize=7.5)
    plt.tight_layout()
    plt.savefig(path, dpi=110)


def top_png(path, frame, inside, envs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
    fig, ax = plt.subplots(figsize=(15, 8.6))
    ax.add_patch(Rectangle((-L2, -W2), 2 * L2, 2 * W2, fc="#f4f5f6", ec="k", lw=1))
    for e in envs:
        bb = e.BoundingBox()
        ax.add_patch(Rectangle((bb.xmin, bb.ymin), bb.xlen, bb.ylen, fc="#ffd9d9", ec="r", ls="--", lw=0.8))
    for n, s, c in frame:
        bb = s.BoundingBox()
        ax.add_patch(Rectangle((bb.xmin, bb.ymin), bb.xlen, bb.ylen, fc="#a3a9af", ec="k", lw=0.6, alpha=0.6 if "底板" in n or "顶板" in n else 1.0, zorder=1 if "底板" in n or "顶板" in n else 3))
    for n, s, c in inside:
        bb = s.BoundingBox()
        ax.add_patch(Rectangle((bb.xmin, bb.ymin), bb.xlen, bb.ylen, fc="#7fb3d5" if c != "rubber" else "#f2c14e", ec="k", lw=0.6, alpha=0.9, zorder=4))
        if "托板" not in n and "电池托" not in n:
            ax.text(bb.center.x, bb.center.y, n.split("（")[0], ha="center", va="center", fontsize=7, zorder=5)
    ax.set_title("骨架俯视（灰 = 铝板，蓝 = 电子件，黄 = 电池，红虚线 = 腿外摆 ±20° 扫过的范围）", fontsize=12)
    ax.set_xlim(-L2 - 10, L2 + 10)
    ax.set_ylim(-W2 - 10, W2 + 10)
    ax.set_aspect("equal")
    plt.tight_layout()
    plt.savefig(path, dpi=110)


if __name__ == "__main__":
    import sys
    if "--png" in sys.argv:
        layout_png(os.path.join(os.path.dirname(HERE), "docs", "机械狗007A", "骨架_俯视.png"))
        raise SystemExit
    envs = [sweep_envelope(fx, sy) for fx in (1, -1) for sy in (1, -1)]
    frame = frame_parts(envs)
    inside = inside_parts()
    legs = {(fx, sy): leg_parts_world(fx, sy) for fx in (1, -1) for sy in (1, -1)}
    rep = ["007A 骨架检查（eng_body.py）", ""]
    # 1 站立：骨架 / 电池 / 电子件 / 4 条腿 互相不撞
    groups = [("骨架", [(n, s) for n, s, c in frame]), ("里面", [(n, s) for n, s, c in inside])]
    for k, P in legs.items():
        groups.append((f"腿{k}", [(n, s) for n, s, c, g in P if g != "机身"]))
        groups.append((f"外摆电机{k}", [(n, s) for n, s, c, g in P if g == "机身"]))
    hits = interference(groups)
    rep.append(f"站立：{len(hits)} 处干涉")
    rep += [f"  {a} × {b}：{v:.0f} mm³" for a, b, v in hits]
    # 2 外摆 ±20°：腿扫过的空间 vs 骨架 / 电池 / 电子件
    g2 = [("骨架", [(n, s) for n, s, c in frame]), ("里面", [(n, s) for n, s, c in inside])] + [(f"扫{i}", [("腿外摆扫过的空间", e)]) for i, e in enumerate(envs)]
    hits2 = interference(g2)
    rep.append(f"外摆 ±{HAA_SWING:.0f}°（加 {MARGIN:.0f} mm 余量）：{len(hits2)} 处干涉")
    rep += [f"  {a} × {b}：{v:.0f} mm³" for a, b, v in hits2]
    rep.append("")
    rep.append(f"电子托板 z = {TRAY_Z - Z0:.0f}（离机身底），顶板 z = {Z_TOPP - Z0:.0f}，托板上能放高 {Z_TOPP - TRAY_Z - 7:.0f} 的东西")
    tall = [(n, sz) for n, x, y, sx, sy_, sz, st in BOARDS if TRAY_Z + 7 + sz > Z_TOPP]
    rep.append("托板上太高的：" + ("无" if not tall else ", ".join(f"{n} {sz}" for n, sz in tall)))
    txt = "\n".join(rep)
    open(os.path.join(HERE, "out", "eng_report.txt"), "w", encoding="utf-8").write(txt + "\n")
    print(txt)
    # 导出给 blender（不带外壳，看骨架）
    out = os.path.join(HERE, "out", "eng_parts")
    os.makedirs(out, exist_ok=True)
    for f in os.listdir(out):
        os.remove(os.path.join(out, f))
    man = []
    allp = [(n, s, c) for n, s, c in frame + inside] + [(f"{k}{n}", s, c) for k, P in legs.items() for n, s, c, g in P]
    for i, (n, s, c) in enumerate(allp):
        fn = f"{i:03d}.stl"
        cq.exporters.export(cq.Workplane().add(s), os.path.join(out, fn), tolerance=0.1, angularTolerance=0.2)
        man.append(dict(file=fn, name=n, color=c))
    json.dump(man, open(os.path.join(out, "manifest.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=0)
    layout_png(os.path.join(os.path.dirname(HERE), "docs", "机械狗007A", "骨架_俯视.png"))
