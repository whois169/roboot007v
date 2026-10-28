"""007A 机身 v9：不要“猪儿虫”——楔形收腹、前宽后窄、棱边利落、没有把身子切成一节一节的竖缝。

（用户 2026-10-28：v8 身体太圆润，像一条猪儿虫。）头、腿、雷达、货台沿用 model.py 的设计，跟着新机身重新定位。
  侧面：前面胸口深（底 Z0），往后肚子收上去（尾部底 Z0 + 50）；顶面前高后低 8（几乎平）
  俯视：前宽 260，尾部收到 220
  截面：上下各切 45° 的棱（八边形），大面不倒圆，只有 1 mm 小倒角
  黑腰：一道跟着收腹走向的斜线（深色凹槽，青色状态灯在里面），从前胸斜到后腿上方
用法：python 007a/body_v9.py → 007a/out/parts（整机：新机身 + 原来的腿）
"""
import json
import math
import os

import cadquery as cq

import model as M
from model import V, box, cyl_x, cyl_y, prism_xz, prism_yz, ring_y, soft

L2, Z0 = M.L2, M.Z0
TOP_F = Z0 + 185.0                       # 前面顶
TOP_R = TOP_F - 8.0                      # 尾部顶
BOT_F, BOT_R = Z0, Z0 + 50.0             # 前胸底、尾部底（收腹）
W_F, W_R = 260.0, 220.0                  # 前宽、尾宽
CH = 18.0                                # 截面上下的 45° 棱


def sec(x, w, z0, z1, ch=CH):
    """x 处的截面（八边形：上下各切 45° 棱），返回 3D 点。"""
    h = w / 2
    pts = [(-h + ch, z0), (h - ch, z0), (h, z0 + ch), (h, z1 - ch), (h - ch, z1), (-h + ch, z1), (-h, z1 - ch), (-h, z0 + ch)]
    return [V(x, y, z) for y, z in pts]


def lerp(a, b, t):
    return a + (b - a) * t


def at(x):
    """x 处的 (宽, 底, 顶)。"""
    t = (L2 - x) / (2 * L2)
    return lerp(W_F, W_R, t), lerp(BOT_F, BOT_R, t), lerp(TOP_F, TOP_R, t)


def loft_body(shrink=0.0):
    ws = []
    for x in (L2, -L2):
        w, z0, z1 = at(x)
        pts = sec(x, w - 2 * shrink, z0 + shrink, z1 - shrink)
        ws.append(cq.Wire.makePolygon(pts, close=True))
    return cq.Solid.makeLoft(ws, True)


def cut_plane(s, p, n):
    big = cq.Solid.makeBox(4000, 4000, 4000, V(-2000, -2000, 0))
    nz = V(*n).normalized()
    pl = cq.Plane(origin=V(*p), xDir=nz.cross(V(0, 1, 0)).normalized() if abs(nz.y) < 0.9 else V(1, 0, 0), normal=nz)
    return s.cut(big.transformShape(pl.rG))


def body_parts():
    P = []
    body = loft_body()
    c = 24.0                                                       # 前后上下的 45° 大斜面
    for p, n in (((L2 - c, 0, TOP_F), (1, 0, 1)), ((L2 - c, 0, BOT_F), (1, 0, -1)),
                 ((-L2 + c, 0, TOP_R), (-1, 0, 1)), ((-L2 + c, 0, BOT_R), (-1, 0, -1))):
        body = cut_plane(body, p, n)
    # ---- 黑腰：一道斜线（前胸 z = Z0+62 → 尾部 z = Z0+88），宽 14、凹进去 6，灯在里面
    zf, zr, bw = Z0 + 62.0, Z0 + 88.0, 14.0
    band = prism_xz([(L2 + 10, zf), (L2 + 10, zf + bw), (-L2 - 10, zr + bw), (-L2 - 10, zr)], -200, 200)
    core = loft_body(6.0)
    body = body.cut(band.cut(core))
    P.append(("黑腰", band.intersect(loft_body(5.5)), "skel_dk"))
    ang = math.atan2(zr - zf, -2 * L2)
    for sy in (1, -1):
        x0, x1 = -110.0, 110.0
        t0, t1 = (L2 - x0) / (2 * L2), (L2 - x1) / (2 * L2)
        za, zb = lerp(zf, zr, t0) + bw / 2, lerp(zf, zr, t1) + bw / 2
        wa, wb = at(x0)[0] / 2 - 6.2, at(x1)[0] / 2 - 6.2
        strip = cq.Solid.makeLoft([cq.Wire.makePolygon([V(x0, sy * wa, za - 2), V(x0, sy * (wa + 0.5), za - 2), V(x0, sy * (wa + 0.5), za + 2), V(x0, sy * wa, za + 2)], close=True),
                                   cq.Wire.makePolygon([V(x1, sy * wb, zb - 2), V(x1, sy * (wb + 0.5), zb - 2), V(x1, sy * (wb + 0.5), zb + 2), V(x1, sy * wb, zb + 2)], close=True)], True)
        P.append((f"腰线灯{sy}", strip, "cyan"))
    # 黑腰以上浅色、以下深灰（下面是“底盘”，上面是“甲板壳”），分界就是那条斜线
    lower = body.intersect(prism_xz([(L2 + 30, BOT_F - 30), (L2 + 30, zf), (-L2 - 30, zr), (-L2 - 30, BOT_F - 30)], -200, 200))
    upper = body.cut(prism_xz([(L2 + 30, BOT_F - 30), (L2 + 30, zf + bw), (-L2 - 30, zr + bw), (-L2 - 30, BOT_F - 30)], -200, 200))
    # ---- 头：沿用 v8 的头（深色屏幕模块，上沿下巴 45°），放在前脸上部，四周 4 mm 影线缝
    sw, sh = 124.0, 64.0
    HW, HH, HD = sw + 26, sh + 24, 30.0
    zc = TOP_F - c - HH / 2 - 4
    hx0, hx1 = L2 + 0.0, L2 + HD
    zb_, zt_ = zc - HH / 2, zc + HH / 2
    upper = upper.cut(box(L2 - 6, L2 + 10, -HW / 2 - 4, HW / 2 + 4, zb_ - 4, zt_ + 4))
    lower = lower.cut(box(L2 - 6, L2 + 10, -HW / 2 - 4, HW / 2 + 4, zb_ - 4, zt_ + 4))
    P.append(("头影线缝底", box(L2 - 6, L2 - 5, -HW / 2 - 4, HW / 2 + 4, zb_ - 4, zt_ + 4), "skel_dk"))
    prof = [(hx0 - 10, zb_), (hx1 - 18, zb_), (hx1, zb_ + 18), (hx1, zt_ - 18), (hx1 - 18, zt_), (hx0 - 10, zt_)]
    head = prism_xz(prof, -HW / 2, HW / 2)
    for sy in (1, -1):
        tri = cq.Workplane("XY").polyline([(hx1 - 12, sy * (HW / 2 + 1)), (hx1 + 12, sy * (HW / 2 + 1)),
                                            (hx1 + 12, sy * (HW / 2 - 13))]).close().extrude(HH + 20).val()
        head = head.cut(tri.translate(V(0, 0, zb_ - 10)))
    head = soft(head, 1.0).cut(box(hx1 - 3, hx1 + 1, -sw / 2, sw / 2, zc - sh / 2, zc + sh / 2))
    P.append(("头", head, "skel_dk"))
    P.append(("屏幕", box(hx1 - 3, hx1 - 1.5, -sw / 2, sw / 2, zc - sh / 2, zc + sh / 2), "glass"))
    for ey in (-30, 30):
        eye = cq.Workplane("YZ").rect(30, 24).extrude(0.4).edges("|X").fillet(6).val().translate(V(hx1 - 1.4, ey, zc + 4))
        P.append((f"眼睛{ey}", eye, "eye"))
    P.append(("屏幕下沿灯", box(hx1 - 1.3, hx1 - 0.9, -40, 40, zc - sh / 2 + 8, zc - sh / 2 + 10), "cyan"))
    for sy in (1, -1):
        P.append((f"头转轴盖{sy}", soft(cyl_y(hx0 + 6, zc, 20, *sorted((sy * HW / 2, sy * (HW / 2 + 3)))), 0.8), "metal"))
        P.append((f"头转轴光环{sy}", ring_y(hx0 + 6, zc, 28, 23, *sorted((sy * HW / 2, sy * (HW / 2 + 0.6)))), "cyan"))
    # 前脸两边斜槽（外沿对齐头正面上下沿）
    ff = HH / 2 - 18
    for sy in (1, -1):
        y_in, y_end = sy * (HW / 2 + 12), sy * (W_F / 2 - 16)
        for z0, z1 in ((zc + ff - 12, zc + ff), (zc - ff, zc - ff + 12)):
            h = z1 - z0
            pts = [(y_in, z0), (y_end - sy * h, z0), (y_end, z1), (y_in + sy * h, z1)]
            upper = upper.cut(prism_yz(pts, L2 - 3, L2 + 10))
            P.append((f"斜槽内衬{sy}{z0:.0f}", prism_yz(pts, L2 - 3.5, L2 - 2), "skel_dk"))
    # ---- 背上货台（45° 切角，跟着顶面）
    BX0, BX1, BW, BD = -170.0, 120.0, 190.0, 14.0
    tray = box(BX0, BX1, -BW / 2, BW / 2, TOP_F - BD, TOP_F + 20)
    for sx in (1, -1):
        for sy in (1, -1):
            x, y = (BX1 if sx > 0 else BX0), sy * BW / 2
            tri = cq.Workplane("XY").polyline([(x + sx, y + sy), (x - sx * 16, y + sy), (x + sx, y - sy * 16)]).close().extrude(60).val()
            tray = tray.cut(tri.translate(V(0, 0, TOP_F - BD - 5)))
    upper = upper.cut(tray)
    P.append(("货台底垫", box(BX0 + 6, BX1 - 6, -BW / 2 + 6, BW / 2 - 6, TOP_F - BD, TOP_F - BD + 2), "rubber"))
    for k in range(8):
        y = -BW / 2 + 24 + k * (BW - 48) / 7
        P.append((f"货台筋{k}", box(BX0 + 20, BX1 - 20, y - 2, y + 2, TOP_F - BD + 2, TOP_F - BD + 5), "rubber"))
    # ---- 字样（上层甲板壳侧面，跟着侧面的斜度贴）
    for sy in (1, -1):
        x = 0.0
        w = at(x)[0] / 2
        txt = cq.Workplane("XZ").text("007A", 26, 0.8, halign="center", valign="center", fontPath="C:/Windows/Fonts/bahnschrift.ttf").val()
        if sy > 0:
            txt = txt.rotate(V(0, 0, 0), V(0, 0, 1), 180).translate(V(x, w + 0.6, zc + 6))
        else:
            txt = txt.translate(V(x, -w - 0.6, zc + 6))
        P.append((f"字样{sy}", txt, "skel_dk"))
    # ---- 雷达（人面对它看的左上角）
    lx, ly = L2 - 50, -(W_F / 2 - 40)
    P.append(("雷达座", soft(box(lx - 26, lx + 26, ly - 26, ly + 26, TOP_F - 2, TOP_F + 8), 1.0), "skel_dk"))
    P.append(("雷达", soft(cq.Solid.makeCylinder(20, 26, V(lx, ly, TOP_F + 8)), 1.5), "skel_dk"))
    P.append(("雷达窗", cq.Solid.makeCylinder(20.4, 8, V(lx, ly, TOP_F + 16)).cut(cq.Solid.makeCylinder(19, 8, V(lx, ly, TOP_F + 16))), "glass"))
    # ---- 尾部：尾灯沿黑腰斜线、急停、充电口
    zt = zr + bw / 2
    for sy in (1, -1):
        P.append((f"尾灯{sy}", box(-L2 - 0.5, -L2 + 2, *sorted((sy * 34, sy * 90)), zt + 14, zt + 20), "red"))
    P.append(("急停", cyl_x(0, zt + 44, 34, -L2 - 14, -L2 + 2), "estop"))
    P.append(("充电口", box(-L2 - 1, -L2 + 1, -52, -30, zt - 20, zt - 8), "rubber"))
    P += [("机身上壳", soft(upper, 1.0), "shell"), ("机身下壳", soft(lower, 1.0), "skel")]
    return P


def all_parts():
    P = body_parts()
    for fx in (1, -1):
        for sy in (1, -1):
            P += M.leg_parts(fx, sy)
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
    print(f"{len(man)} 个零件（v9 机身）")
