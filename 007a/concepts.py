"""007A 设计语言：三个方向的概念模型（腿统一用 v6.1，只换机身 / 头 / 关节窝）。

共同的设计语言（2026-10-28 和用户定的思路）：
  一个主体（不是几个盒子）；六边形 / 30°·60° 一组角度贯穿全身；一条主线从头走到尾灯，状态灯藏在线里；
  分缝跟着结构走；关节嵌进机身的窝里；上浅下深；细节都有用（散热口、传感器窗、绑带孔）。
三个方向：
  A 多面体硬朗风：机身截面是拉长的六边形，前后多面切角，头是六边形面罩从车头长出来
  B 整体曲面 + 锐利折线：大圆角的整体壳，中间一道锐利的折线（灯在里面），头是包住车头的一整条烟黑面罩
  C 骨架 + 面板：深色骨架（侧梁、顶梁、前后框，开六边形减重孔）露在外面，浅色面板嵌在骨架里
用法：python 007a/concepts.py A|B|C → 007a/out/concept_A/parts/*.stl + manifest.json
"""
import json
import math
import os
import sys

import cadquery as cq

import model as M
from model import V, box, cyl_y, prism_xz, prism_yz, ring_y, soft

L2, W2, Z0 = M.L2, M.W2, M.Z0
H = M.BODY_H
ZT = Z0 + H                                    # 机身顶
ZC = Z0 + 92                                   # 主线（折线）高度
HX = M.HIP_X


def hexagon_xz(cx, cz, a, rot=0.0):
    """x-z 平面里的正六边形（边心距 a），平边朝上下。"""
    r = a / math.cos(math.radians(30))
    return [(cx + r * math.cos(math.radians(rot + 60 * k)), cz + r * math.sin(math.radians(rot + 60 * k))) for k in range(6)]


def hexagon_yz(cy, cz, w, h):
    """y-z 平面里的“扁六边形”（宽 w、高 h，左右两头尖），头的面罩用。"""
    c = h / 2 * math.tan(math.radians(30))
    return [(cy - w / 2, cz), (cy - w / 2 + c, cz - h / 2), (cy + w / 2 - c, cz - h / 2), (cy + w / 2, cz),
            (cy + w / 2 - c, cz + h / 2), (cy - w / 2 + c, cz + h / 2)]


def cut_halfspace(s, p, n):
    """切掉法向 n 那一侧（过点 p 的平面）。"""
    big = cq.Solid.makeBox(4000, 4000, 4000, V(-2000, -2000, 0))
    nz = V(*n).normalized()
    pl = cq.Plane(origin=V(*p), xDir=nz.cross(V(0, 0, 1)) if abs(nz.z) < 0.9 else V(1, 0, 0), normal=nz)
    blk = big.transformShape(pl.rG)
    return s.cut(blk)


def legs(socket=True):
    P = []
    for fx in (1, -1):
        for sy in (1, -1):
            P += [(n, s, c) for n, s, c in M.leg_parts(fx, sy) if not n.endswith("髋座")]
    return P


def screen_and_eyes(xf, zc, sw=124.0, sh=64.0):
    """屏幕（玻璃）+ 眼睛（屏上的图案）。xf = 屏幕前表面的 x。"""
    P = [("屏幕", box(xf - 2, xf, -sw / 2, sw / 2, zc - sh / 2, zc + sh / 2), "glass")]
    for ey in (-30, 30):
        eye = cq.Workplane("YZ").rect(30, 24).extrude(0.4).edges("|X").fillet(6).val().translate(V(xf + 0.1, ey, zc + 4))
        P.append((f"眼睛{ey}", eye, "eye"))
    P.append(("屏幕下沿灯", box(xf + 0.1, xf + 0.5, -40, 40, zc - sh / 2 + 8, zc - sh / 2 + 10), "cyan"))
    return P


def cargo_tray(shell, x0=-175.0, x1=110.0, w=190.0, d=14.0, ch=24.0, top=ZT):
    """背上的一体货台：切角的凹槽 + 橡胶底 + 防滑筋（凹槽四角是 45° 切角，和全身语言一致）。"""
    pts = [(x0 + ch, -w / 2), (x1 - ch, -w / 2), (x1, -w / 2 + ch), (x1, w / 2 - ch), (x1 - ch, w / 2), (x0 + ch, w / 2),
           (x0, w / 2 - ch), (x0, -w / 2 + ch)]
    well = cq.Workplane("XY").polyline(pts).close().extrude(d + 10).val().translate(V(0, 0, top - d))
    shell = shell.cut(well)
    floor = cq.Workplane("XY").polyline(pts).close().offset2D(-3, "intersection").extrude(2).val().translate(V(0, 0, top - d))
    P = [("货台底垫", floor, "rubber")]
    for k in range(8):
        y = -w / 2 + 24 + k * (w - 48) / 7
        P.append((f"货台筋{k}", soft(box(x0 + 20, x1 - 20, y - 2, y + 2, top - d + 2, top - d + 5), 0.8), "rubber"))
    return shell, P


def lidar(x, y, z, hexbase=True):
    P = []
    if hexbase:
        base = cq.Workplane("XY").polygon(6, 60).extrude(10).val().translate(V(x, y, z))
        P.append(("雷达座", soft(base, 1.5), "skel_dk"))
    P.append(("雷达", soft(cq.Solid.makeCylinder(20, 26, V(x, y, z + 10)), 1.5), "skel_dk"))
    P.append(("雷达窗", cq.Solid.makeCylinder(20.4, 8, V(x, y, z + 18)).cut(cq.Solid.makeCylinder(19, 8, V(x, y, z + 18))), "glass"))
    return P


def hex_socket(sh, sy, depth=10.0, a=64.0):
    """机身侧面在髋的位置挖六边形的窝（腿从窝里伸出来），窝底深色。"""
    P = []
    for fx in (1, -1):
        hx = fx * HX
        y0, y1 = (W2 - depth, W2 + 40) if sy > 0 else (-W2 - 40, -W2 + depth)
        pocket = prism_xz(hexagon_xz(hx, M.HIP_Z, a), y0, y1)
        sh = sh.cut(pocket)
        yb = (W2 - depth - 1, W2 - depth + 0.5) if sy > 0 else (-W2 + depth - 0.5, -W2 + depth + 1)
        P.append((f"髋窝底{fx}{sy}", prism_xz(hexagon_xz(hx, M.HIP_Z, a), *yb), "skel_dk"))
    return sh, P


# ================================================================ A 多面体硬朗风
def concept_A():
    P = []
    sec = [(-W2 + 34, Z0), (W2 - 34, Z0), (W2, ZC), (W2 - 26, ZT), (-W2 + 26, ZT), (-W2, ZC)]   # 截面：拉长的六边形
    hull = prism_yz(sec, -L2, L2)
    t30 = math.tan(math.radians(30))
    for sx in (1, -1):                                        # 前后：上下两个斜面（30°）+ 两侧切角（平面 60°）
        x = sx * L2
        hull = cut_halfspace(hull, (x - sx * 46, 0, ZT), (sx * math.sin(math.radians(30)), 0, math.cos(math.radians(30))))
        hull = cut_halfspace(hull, (x - sx * 40, 0, Z0), (sx * math.sin(math.radians(45)), 0, -math.cos(math.radians(45))))
        for sy in (1, -1):
            hull = cut_halfspace(hull, (x - sx * 34, sy * W2, 0), (sx * math.cos(math.radians(60)), sy * math.sin(math.radians(60)), 0))
    hull = soft(hull, 1.5)
    # 主线：侧面最宽的那条棱上开一道细槽，青色状态灯藏在里面（前后两段，中间让开髋）
    for sy in (1, -1):
        for x0, x1 in ((-HX + 75, HX - 75),):
            hull = hull.cut(box(x0, x1, *sorted((sy * (W2 - 4), sy * (W2 + 2))), ZC - 2.5, ZC + 2.5))
            P.append((f"主线灯{sy}", box(x0 + 2, x1 - 2, *sorted((sy * (W2 - 3.5), sy * (W2 - 3))), ZC - 1.5, ZC + 1.5), "cyan"))
    # 头：六边形面罩从车头长出来（车头竖面上开六边形口，面罩往外伸 24）
    zh = ZC + 26
    face = hexagon_yz(0, zh, 176, 96)
    hull = hull.cut(prism_yz(face, L2 - 20, L2 + 40))
    mask = prism_yz(face, L2 - 20, L2 + 24).cut(prism_yz(hexagon_yz(0, zh, 150, 76), L2 + 18, L2 + 30))
    P.append(("头面罩", soft(mask, 1.2), "skel_dk"))
    P += screen_and_eyes(L2 + 18, zh)
    # 上浅下深：主线以上浅色、以下深色（同一个壳切开）
    hull, tray = cargo_tray(hull, top=ZT)
    up = hull.intersect(box(-L2 - 50, L2 + 50, -W2 - 50, W2 + 50, ZC, ZT + 20))
    lo = hull.intersect(box(-L2 - 50, L2 + 50, -W2 - 50, W2 + 50, Z0 - 20, ZC))
    for sy in (1, -1):
        lo, sk = hex_socket(lo, sy)
        P += sk
        up, _ = hex_socket(up, sy)
    P += [("机身上壳", up, "shell"), ("机身下壳", lo, "skel")] + tray
    # 尾部：后斜面上的尾灯（红，沿主线）、急停、散热口（六边形一组，给电机和电脑散热）
    for sy in (1, -1):
        P.append((f"尾灯{sy}", box(-L2 - 0.5, -L2 + 1, *sorted((sy * 40, sy * 96)), ZC + 6, ZC + 12), "red"))
    P.append(("急停", M.cyl_x(0, ZC + 34, 34, -L2 - 14, -L2 + 2), "estop"))
    for k in range(5):
        y = -60 + k * 30
        P.append((f"散热口{k}", prism_yz(hexagon_yz(y, ZC - 34, 24, 14), -L2 - 0.4, -L2 + 1), "skel_dk"))
    P += lidar(L2 - 70, -(W2 - 56), ZT)
    return P + legs()


# ================================================================ B 整体曲面 + 锐利折线
def concept_B():
    P = []
    up = cq.Workplane("XY").box(2 * L2, 2 * W2, ZT - ZC).edges("|Z").fillet(60).edges(">Z").fillet(26).val()
    up = up.translate(V(0, 0, (ZT + ZC) / 2))
    lo = cq.Workplane("XY").box(2 * L2 - 30, 2 * W2 - 24, ZC - Z0 - 8).edges("|Z").fillet(50).edges("<Z").fillet(18).val()
    lo = lo.translate(V(0, 0, (ZC - 8 + Z0) / 2))
    # 锐利折线：上壳底边是一道锋利的“唇”，下面一圈凹进去的深色带，灯在带里
    band = cq.Workplane("XY").box(2 * L2 - 20, 2 * W2 - 16, 8).edges("|Z").fillet(54).val().translate(V(0, 0, ZC - 4))
    P.append(("折线深色带", band, "skel_dk"))
    for sy in (1, -1):
        P.append((f"折线灯{sy}", box(-HX + 80, HX - 80, *sorted((sy * (W2 - 8.6), sy * (W2 - 8))), ZC - 5.5, ZC - 2.5), "cyan"))
    # 头：车头包一整条烟黑面罩（屏幕和前面的传感器都在面罩后面，有作用）
    vz0, vz1 = ZC + 18, ZT - 22
    visor = up.intersect(box(L2 - 70, L2 + 10, -W2 - 10, W2 + 10, vz0, vz1))
    up = up.cut(box(L2 - 70, L2 + 10, -W2 - 10, W2 + 10, vz0, vz1))
    inner = up.intersect(box(-L2, L2, -W2, W2, 0, 1))                         # 占位（保持变量）
    P.append(("面罩", visor, "glass"))
    P += screen_and_eyes(L2 + 0.3, (vz0 + vz1) / 2 + 2)[1:]                 # 只要眼睛和下沿灯（面罩本身就是玻璃，屏幕在后面）
    up, tray = cargo_tray(up, top=ZT, ch=40)
    for sy in (1, -1):
        lo, sk = hex_socket(lo, sy, depth=8, a=62)
        P += sk
        up, _ = hex_socket(up, sy, depth=8, a=62)
    P += [("机身上壳", up, "shell"), ("机身下壳", lo, "skel")] + tray
    for sy in (1, -1):                                                        # 尾灯：后面一道弧形红灯
        P.append((f"尾灯{sy}", box(-L2 - 0.5, -L2 + 2, *sorted((sy * 30, sy * 100)), ZC + 10, ZC + 15), "red"))
    P.append(("急停", M.cyl_x(0, ZC + 40, 34, -L2 - 14, -L2 + 4), "estop"))
    dome = cq.Solid.makeSphere(26, V(L2 - 90, -(W2 - 60), ZT)).cut(box(L2 - 130, L2 - 50, -W2, 0, ZT - 40, ZT))
    P.append(("雷达罩", dome, "glass"))
    return P + legs()


# ================================================================ C 骨架 + 面板
def concept_C():
    P = []
    frame = []
    # 两侧的主梁（主线就是主梁的上沿），开六边形减重孔
    for sy in (1, -1):
        y0, y1 = sorted((sy * (W2 - 16), sy * W2))
        beam = box(-L2, L2, y0, y1, ZC - 18, ZC + 18)
        for k in range(-3, 4):
            if abs(k * 52) in (0,) or abs(abs(k * 52) - HX) < 60:
                continue
            beam = beam.cut(prism_xz(hexagon_xz(k * 52, ZC, 11, 30), y0 - 1, y1 + 1))
        frame.append(beam)
        # 顶梁（兼货物导轨，带绑带孔）
        tb = box(-L2 + 20, L2 - 20, *sorted((sy * (W2 - 34), sy * (W2 - 4))), ZT - 14, ZT)
        for x in range(-180, 181, 60):
            tb = tb.cut(box(x - 12, x + 12, *sorted((sy * (W2 - 24), sy * (W2 - 14))), ZT - 20, ZT + 1))
        frame.append(tb)
        # 髋节点：竖柱把顶梁、主梁、髋座连起来（六边形的髋座）
        for fx in (1, -1):
            node = prism_xz(hexagon_xz(fx * HX, M.HIP_Z, 60), y0 - 2, y1)
            post = box(fx * HX - 22, fx * HX + 22, y0, y1, M.HIP_Z, ZT)
            frame += [node, post]
    for sx in (1, -1):                                       # 前后框
        x0, x1 = sorted((sx * (L2 - 16), sx * L2))
        fr = box(x0, x1, -W2, W2, Z0 + 14, ZT)
        fr = fr.cut(box(x0 - 1, x1 + 1, -W2 + 18, W2 - 18, Z0 + 30, ZT - 16))
        frame.append(fr)
    fb = frame[0]
    for f in frame[1:]:
        fb = fb.fuse(f)
    fb = soft(fb.clean(), 1.5)
    P.append(("骨架", fb, "skel"))
    # 面板：浅色，嵌在骨架之间，四周留 3 mm 缝（看得到里面深色）
    core = box(-L2 + 14, L2 - 14, -W2 + 14, W2 - 14, Z0 + 4, ZT - 6)
    P.append(("内胆", core, "skel_dk"))
    top = box(-L2 + 19, L2 - 19, -W2 + 37, W2 - 37, ZT - 12, ZT - 2)
    top, tray = cargo_tray(top, x0=-170, x1=150, w=150, top=ZT - 2)
    P.append(("顶盖", soft(top, 1.2), "shell"))
    P += tray
    for sy in (1, -1):
        for z0, z1 in ((ZC + 21, ZT - 17), (Z0 + 18, ZC - 21)):
            for x0, x1 in ((-L2 + 19, -HX - 64), (-HX + 64, HX - 64), (HX + 64, L2 - 19)):
                if x1 - x0 < 10:                             # 髋节点占满了的地方不放面板
                    continue
                pnl = box(x0, x1, *sorted((sy * (W2 - 13), sy * (W2 - 3))), z0, z1)
                P.append((f"侧板{sy}{z0:.0f}{x0:.0f}", soft(pnl, 1.5), "shell"))
        P.append((f"主梁灯{sy}", box(-HX + 70, HX - 70, *sorted((sy * W2, sy * (W2 + 0.6))), ZC + 14, ZC + 16), "cyan"))
    # 头：装在前框里的模块（深色框 + 屏幕），框就是骨架的一部分
    zh = (Z0 + 30 + ZT - 16) / 2 + 6
    head = box(L2 - 14, L2 + 18, -95, 95, zh - 52, zh + 52)
    head = cq.Workplane().add(head).edges("|X").chamfer(14).val()
    head = soft(head.cut(box(L2 + 14, L2 + 20, -64, 64, zh - 34, zh + 34)), 1.2)
    P.append(("头模块", head, "skel_dk"))
    P += screen_and_eyes(L2 + 16, zh)
    for sy in (1, -1):
        P.append((f"尾灯{sy}", box(-L2 - 0.5, -L2 + 1, *sorted((sy * 30, sy * 100)), ZT - 12, ZT - 6), "red"))
    P.append(("急停", M.cyl_x(0, ZC + 40, 34, -L2 - 14, -L2 + 2), "estop"))
    P += lidar(L2 - 60, -(W2 - 19), ZT)
    return P + legs()


if __name__ == "__main__":
    which = sys.argv[1].upper()
    out = os.path.join(M.HERE, "out", f"concept_{which}", "parts")
    os.makedirs(out, exist_ok=True)
    for f in os.listdir(out):
        os.remove(os.path.join(out, f))
    parts = {"A": concept_A, "B": concept_B, "C": concept_C}[which]()
    man = []
    for i, (n, s, c) in enumerate(parts):
        fn = f"{i:03d}.stl"
        cq.exporters.export(cq.Workplane().add(s), os.path.join(out, fn), tolerance=0.08, angularTolerance=0.15)
        man.append(dict(file=fn, name=n, color=c))
    json.dump(man, open(os.path.join(out, "manifest.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=0)
    print(which, len(man), "个零件")
