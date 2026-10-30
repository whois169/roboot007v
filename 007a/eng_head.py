"""007A 第 2 步：头（屏幕抬头低头）的安装和转动检查（v19：抬 40°、低 20°，头壳壁 3）。

头绕 y 轴转，转轴在前脸前面 10（x = L2 + 10，高度 = 头中心）。
机身上（不动）：两块头支架（铝 3，拧在 A05 前端板上，从影线缝伸进头后面）；左支架上装抬头舵机 飞特 STS3032（23.2 × 12.1 × 28.5，
  20 g，4.5 kg·cm，5 V 总线；输出轴就是左边的转轴，★ 尺寸按经销商资料，到货核）；右边一根 Φ5 销子 + 法兰轴承 MF105。
头上（跟着转）：头壳、屏幕玻璃、眼睛、灯、两边的鳍，加里面的屏幕板 微雪 4.3B（112.4 × 75.1，厚按 10 算）。
检查：头抬 40° … 低 20°（每 5°；负 = 抬头），头上的零件不能碰机身外壳、前防撞杠、相机罩、灯、热像仪、前端板、头支架、舵机。
用法：python 007a/eng_head.py
"""
import math

import cadquery as cq

import body_v10 as BV
import eng_body as B
import model as M
from model import V, box, cyl_x, cyl_y

L2, Z0, TOP = M.L2, M.Z0, M.TOP
ZC = M.WAIST_TOP + (TOP - M.WAIST_TOP) / 2 - 2
PX, PZ = L2 + 10.0, ZC                              # 转轴
HEAD_NAMES = ("头", "屏幕", "眼睛", "屏幕下沿灯")
# 抬头舵机 飞特 STS3032（店铺详情页的尺寸图）：机身 23 × 12 × 高 27.5；安装耳总长 32、在离底 20.5–22；4-Φ2.5 孔、4-R2 角（孔跟圆角同心 → 孔离耳边 2：
#   孔距 28 × 8，★ 按图推的）；输出 25T Φ4.95，花键顶离底 31.9；输出轴离机身一头 8.21（★ 图上标注有两种读法，到货核）
SV_L, SV_W, SV_H, SV_EAR, SV_EAR_Y, SV_SHAFT = 23.0, 12.0, 27.5, 32.0, 20.5, 8.21
SV_Y0 = 37.5                                         # 舵机底面 y：耳朵（离底 20.5）要高过屏幕板（y ≤ 56.2），抬头时不扫到它
SV_XC = PX + SV_SHAFT - SV_L / 2                     # 舵机机身中心 x（输出轴对准转轴，短的一头朝前）
CRADLE_SCREWS = [(238.0, PZ), (248.0, PZ)]           # 舵机座 → 左头支架：2 颗 M3×8，螺丝头在座子里（先装座子再放舵机），外面 M3 螺母
SKIP = ("头影线缝底", "头转轴盖", "头转轴光环")      # 转轴盖、光环绕转轴转（圆的），外壳上给它们开圆窝


def head_inside():
    """头里面的屏幕板（跟着头转）。"""
    hx1 = L2 + 4 + 30.0
    return [("屏幕板 微雪 4.3B", box(hx1 - 3 - 10, hx1 - 3, -112.4 / 2, 112.4 / 2, PZ - 75.1 / 2, PZ + 75.1 / 2))]


def body_side():
    """机身上不动的：头支架 ×2、舵机、舵盘转接件、右销。"""
    P = []
    x0 = B.X_END + B.T_END                                          # 前端板外侧
    for sy, (ya, yb), (f0, f1) in ((1, (30.0, 33.0), (33.0, 47.0)), (-1, (-69.0, -66.0), (-83.0, -69.0))):
        br = box(x0, PX + 4, ya, yb, PZ - 10, PZ + 10).fuse(box(x0, x0 + 3, f0, f1, PZ - 12, PZ + 12))   # 折弯件：一边伸进头，一边是法兰
        yh = (f0 + f1) / 2
        for dz in (-6.0, 6.0):
            br = br.cut(cyl_x(yh, PZ + dz, 3.4, x0 - 1, x0 + 4))
        if sy > 0:
            for x, z in CRADLE_SCREWS:                                   # 舵机座的 2 个孔
                br = br.cut(cyl_y(x, z, 3.4, ya - 1, yb + 1))
        P.append((f"头支架{'左' if sy > 0 else '右'}（铝 3 折弯）", br))
    sv = box(SV_XC - SV_L / 2, SV_XC + SV_L / 2, SV_Y0, SV_Y0 + SV_H, PZ - SV_W / 2, PZ + SV_W / 2).fuse(
        box(SV_XC - SV_EAR / 2, SV_XC + SV_EAR / 2, SV_Y0 + SV_EAR_Y, SV_Y0 + SV_EAR_Y + 1.5, PZ - SV_W / 2, PZ + SV_W / 2)).fuse(
        cyl_y(PX, PZ, 4.95, SV_Y0 + SV_H, SV_Y0 + 31.9))
    P.append(("抬头舵机 STS3032", sv))
    P.append(("舵机座 P04（打印 PETG）", servo_cradle()))
    # 右转轴销 Φ5 穿过头的侧壁进法兰轴承（本来就要穿过去，不算干涉）
    return P


def servo_cradle():
    """舵机座 P04：底板贴在左头支架上（y 33–37.5），三面墙托住舵机，墙顶 = 耳朵底面；后面两个耳孔 M2 自攻（前面两个耳朵伸到转轴前面，
    那里的座子会扫到屏幕板，不托）；底板 2 个 M3 沉头过孔拧头支架；后墙开线口。前面不超过转轴前 5（抬头低头时离屏幕板 ≥ 1.6）。"""
    x0, x1 = B.X_END + B.T_END + 0.2, PX + 5.0
    ytop = SV_Y0 + SV_EAR_Y
    c = box(x0, x1, 33.0, ytop, PZ - 8.0, PZ + 8.0)
    c = c.cut(box(SV_XC - SV_L / 2 - 0.2, x1 + 1, SV_Y0, ytop + 1, PZ - SV_W / 2 - 0.2, PZ + SV_W / 2 + 0.2))   # 舵机槽（前面敞开）
    c = c.cut(box(x0 - 1, SV_XC - SV_L / 2 + 0.5, SV_Y0 + 2, SV_Y0 + 10, PZ - 3.5, PZ + 3.5))                   # 线口（舵机线从机身后头出）
    for x, z in CRADLE_SCREWS:
        c = c.cut(cyl_y(x, z, 3.4, 32, 38)).cut(cyl_y(x, z, 6.2, 35.0, 38))                                 # M3 过孔 + 沉头（头埋进底板，舵机底下不顶）
    for dz in (-4.0, 4.0):
        c = c.cut(cyl_y(SV_XC - SV_EAR / 2 + 2.0, PZ + dz, 1.7, ytop - 6, ytop + 1))                         # 后耳 M2 自攻
    return c


def rot(s, deg):
    return s.rotate(V(PX, 0, PZ), V(PX, 1, PZ), deg)


def bracket_clearance():
    """头支架（不动）相对头转过的空间：头壳上挖掉这一块（在头背面里面，外面看不到），头抬 40° 才不蹭右支架的法兰。"""
    out = None
    for n, s in body_side():
        if not n.startswith("头支架"):
            continue
        a = 0.0
        while a <= M.HEAD_UP + 1e-6:
            r = rot(s, a)
            out = r if out is None else out.fuse(r)
            a += 2.5
    return out


if __name__ == "__main__":
    look = BV.body_parts()
    hx1 = L2 + 4 + 30.0
    hollow = M.head_hollow(3.0)                                       # 头是壳（v19 壁 3，用户：不能太薄），后面敞开让支架伸进去；弧形裙边留着
    head = [(n, s.cut(hollow).cut(bracket_clearance()) if n == "头" else s) for n, s, c in look if n.startswith(HEAD_NAMES) and not n.startswith(SKIP)] + head_inside()
    fixed = [(n, s) for n, s, c in look if n in ("上层甲板壳", "前防撞杠", "黑腰", "下层底盘", "雷达护圈", "雷达") or n.startswith(("深度相机", "热像仪", "台阶测距", "探照灯"))]
    envs = [B.sweep_envelope(fx, sy) for fx in (1, -1) for sy in (1, -1)]
    fixed += [(n, s) for n, s, c in B.frame_parts(envs) if n.startswith("A05")]
    fixed += body_side()
    print("头上的零件：", ", ".join(n for n, _ in head))
    bad = []
    for deg in range(-int(M.HEAD_UP), int(M.HEAD_DOWN) + 1, 5):
        for hn, hs in head:
            r = rot(hs, deg) if deg else hs
            rb = r.BoundingBox()
            for fn, fs in fixed:
                fb = fs.BoundingBox()
                if rb.xmax < fb.xmin or fb.xmax < rb.xmin or rb.ymax < fb.ymin or fb.ymax < rb.ymin or rb.zmax < fb.zmin or fb.zmax < rb.zmin:
                    continue
                v = B.vol(r.intersect(fs))
                if v > 1:
                    bad.append((deg, hn, fn, v))
    print(f"头抬 {M.HEAD_UP:.0f}° … 低 {M.HEAD_DOWN:.0f}°：{len(bad)} 处干涉")
    for d, a, b, v in bad:
        print(f"  {d:+d}°  {a} × {b}：{v:.0f} mm³")
