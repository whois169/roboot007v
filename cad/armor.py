"""
RoboDog v0.5 外观件（机身坐标，X 前 / Y 左 / Z 上），按《外观设计规范 v1》：
  深灰哑光壳板层叠在骨架上；银色件只用在前脸面板、髋/膝端盖、小腿护甲、侧面饰条；黑色传感器窗口；琥珀橙状态灯。
  全部平面 + 斜切角；缝线 0.6 深 × 0.8 宽。
  尾部：收窄的尾罩 + 红色尾灯 + 充电口 + 急停（v0.5 去掉尾鳍）
  机身：带中脊的顶部装甲、分层侧甲、肩甲、大腿装甲；背部两根任务导轨（兼提手）+ 激光雷达模块 + 示范任务箱
灯条只做外形（渲染时发光），实物用 WS2812B 灯带 + 半透明 PETG 导光条。
"""
import math
import cadquery as cq
from config import *
from parts import V, box, cyl_y, cyl_z, cyl_x, xz_prism, yz_prism, xy_prism, chamfer_edges, tapered_slot_xz, stadium_y, offset_poly

TOP = BODY_ZMAX           # 机身骨架顶面 z=50
MID_X = HIP_X - BLOCK_DEPTH - 1   # 中段型材端面 x=±93


def octa(x, hw, z0, z1, ct, cb):
    """在 x 处的八边形截面（y,z），ct/cb = 上/下倒角"""
    pts = [(-hw + cb, z0), (hw - cb, z0), (hw, z0 + cb), (hw, z1 - ct),
           (hw - ct, z1), (-hw + ct, z1), (-hw, z1 - ct), (-hw, z0 + cb)]
    return cq.Wire.makePolygon([V(x, y, z) for y, z in pts], close=True)


def loft(sections):
    return cq.Solid.makeLoft([octa(*s) for s in sections], True)


def inset(s, d):
    x, hw, z0, z1, ct, cb = s
    return (x, hw - d, z0 - 0.01 if z0 <= TOP + 6.01 else z0 + d, z1 - d, max(ct - d * 0.4, 0.5), max(cb - d * 0.4, 0.5))


SEAM_W, SEAM_D = 0.8, 0.6          # 外观规范：缝线 0.8 宽 × 0.6 深


# ------------------------------------------------------------------ 头部
HEAD_SECT = [
    (HIP_X - 38, 54, TOP + 6, 106, 12, 4),     # 后端
    (HIP_X + 48, 52, TOP + 6, 106, 14, 4),     # 中段（顶面保持水平，便于倒扣打印）
    (HIP_X + 90, 40, TOP + 16, 92, 10, 8),     # 前脸
]
HEAD_FRONT_X = HEAD_SECT[-1][0]
VISOR = (-34, 34, 69, 88)    # 面罩窗 y0,y1,z0,z1


def head_shell_v2():
    outer = loft(HEAD_SECT)
    inner_secs = [inset(s, 2.6) for s in HEAD_SECT]
    inner_secs[0] = (inner_secs[0][0] + 2.6,) + inner_secs[0][1:]
    inner_secs[-1] = (inner_secs[-1][0] - 2.6,) + inner_secs[-1][1:]
    h = outer.cut(loft(inner_secs))
    h = h.cut(box(HEAD_SECT[0][0] - 1, HEAD_SECT[1][0] + 6, -49, 49, TOP + 5, TOP + 8.6))   # 底部敞开
    # 面罩窗（装深色亚克力）
    y0, y1, z0, z1 = VISOR
    h = h.cut(box(HEAD_FRONT_X - 14, HEAD_FRONT_X + 1, y0, y1, z0, z1))
    # 两侧斜向散热槽
    for k in range(4):
        x = HIP_X + 4 + k * 9
        for s in (+1, -1):
            slot = box(x, x + 4, -60, 60, 72, 96).cut(box(x - 1, x + 5, -50.5, 50.5, 60, 100))
            h = h.cut(slot.rotate(V(x, 0, 84), V(0, 1, 0), -18))
    # 顶部麦克风阵列孔
    for (dx, dy) in ((0, 26), (0, -26), (26, 0), (-26, 0)):
        h = h.cut(cyl_z(1.5, 100, 108, HIP_X + 10 + dx, dy))
    # 耳鳍安装孔
    for s in (+1, -1):
        for x in (HIP_X - 22, HIP_X + 2):
            h = h.cut(cyl_z(1.7, 100, 108, x, s * 38))
    return h


def head_visor():
    y0, y1, z0, z1 = VISOR
    g = box(HEAD_FRONT_X - 3.2, HEAD_FRONT_X - 0.6, y0 + 0.3, y1 - 0.3, z0 + 0.3, z1 - 0.3)
    return chamfer_edges(g, "|X", 3.0)


def head_eyes():
    """双眼灯：面罩后面的两段灯条"""
    out = []
    for (a, b) in ((-27, -8), (8, 27)):
        e = box(HEAD_FRONT_X - 0.6, HEAD_FRONT_X - 0.1, a, b, 80.5, 84.0)
        out.append(chamfer_edges(e, "|X", 1.4))
    return out


def ear_fin(side=+1, assembled=True):
    """后掠耳鳍：头部辨识度的关键，独立打印后螺接在头顶"""
    x0 = HIP_X - 30
    pts = [(x0, 105), (x0 + 40, 105), (x0 + 30, 112), (x0 - 4, 142), (x0 - 10, 140)]
    f = xz_prism(pts, -3, 3)
    f = chamfer_edges(f, "|Y", 1.2)
    f = f.fuse(box(x0 + 2, x0 + 36, -6, 6, 104, 107))                 # 底座
    f = f.cut(cyl_z(1.7, 103, 108, HIP_X - 22, 0)).cut(cyl_z(1.7, 103, 108, HIP_X + 2, 0))
    if not assembled:
        return f
    f = f.rotate(V(0, 0, 105), V(1, 0, 0), -14 * side)                  # 向外张开
    return f.translate(V(0, side * 38, 0))


def head_base_v2():
    """头部底座：伸到 HFE 电机上方，托住更长的头壳"""
    xb, xf = HEAD_SECT[0][0], HEAD_SECT[1][0] + 8
    n = box(xb, xf, -50, 50, TOP, TOP + 6)
    n = chamfer_edges(n, "|Z", 5.0)
    n = n.fuse(box(xb + 3.2, xf - 2, -46.6, 46.6, TOP + 6, TOP + 9).cut(box(xb + 6, xf - 5, -44, 44, TOP + 5, TOP + 10)))
    n = n.cut(cyl_z(9, TOP - 1, TOP + 7, HIP_X - 21, 0))
    n = n.cut(box(xb + 45, xf - 14, -30, 30, TOP - 1, TOP + 7))
    for (x, y) in ((xb + 10, 30), (xb + 10, -30), (HIP_X - 12, 30), (HIP_X - 12, -30)):
        n = n.cut(cyl_z(1.7, TOP - 1, TOP + 7, x, y))
    return n


# ------------------------------------------------------------------ 尾部
TAIL_SECT = [
    (-MID_X + 1, BODY_W / 2 - 4, TOP, 66, 8, 0.5),
    (-(HIP_X + 2), BODY_W / 2 - 9, TOP, 72, 10, 0.5),
    (-(HIP_X + 58), 30, TOP + 2, 64, 6, 2),
]
TAIL_BACK_X = TAIL_SECT[-1][0]


def tail_cowl():
    outer = loft(TAIL_SECT)
    inner = [inset(s, 2.6) for s in TAIL_SECT]
    inner[0] = (inner[0][0] + 1,) + inner[0][1:]
    inner[-1] = (inner[-1][0] + 2.6,) + inner[-1][1:]
    t = outer.cut(loft(inner))
    # 尾灯槽
    t = t.cut(box(TAIL_BACK_X - 1, TAIL_BACK_X + 3, -22, 22, 56, 60))
    # 急停孔 Φ22 + 充电口（XT60 母座面板）
    t = t.cut(cyl_z(11.2, 60, 76, -(HIP_X - 30), 0))
    t = t.cut(box(-(HIP_X + 20), -(HIP_X + 4), -60 - _DW, -40 - _DW, 54, 70).rotate(V(-(HIP_X + 12), -46 - _DW, 62), V(1, 0, 0), -20))   # 右侧
    # v0.5 去掉尾鳍（宠物元素）；尾罩顶面加一道横向缝线
    skin = outer.cut(loft([inset(sc, SEAM_D) for sc in TAIL_SECT]))
    t = t.cut(skin.intersect(box(-(HIP_X + 30) - SEAM_W / 2, -(HIP_X + 30) + SEAM_W / 2, -90, 90, 40, 90)))
    return t


def tail_fin():
    """上翘尾鳍（一体打印，插进尾罩槽里用 2 颗 M3 固定）"""
    xa = -(HIP_X + 20)
    pts = [(xa, 62), (xa - 30, 62), (xa - 30, 66), (xa - 86, 122), (xa - 72, 126), (xa - 4, 70)]
    f = xz_prism(pts, -3, 3)
    f = chamfer_edges(f, "|Y", 1.5)
    f = f.cut(cyl_y(1.7, -4, 4, xa - 12, 64)).cut(cyl_y(1.7, -4, 4, xa - 24, 64))
    return f.cut(tail_fin_light())    # 灯条槽


def tail_fin_light():
    xa = -(HIP_X + 20)
    pts = [(xa - 60, 103), (xa - 74, 117), (xa - 72, 119), (xa - 58, 105)]
    return xz_prism(pts, -3.4, 3.4)


def tail_light():
    return box(TAIL_BACK_X - 0.6, TAIL_BACK_X + 2.6, -21.5, 21.5, 56.4, 59.6)


# ------------------------------------------------------------------ 机身
_DW = BODY_W / 2 - 70              # v0.4 机身加宽，每边多出的宽度（顶甲外缘跟着往外，中脊不变）
SPINE_PROFILE = [(-72 - _DW, TOP), (72 + _DW, TOP), (72 + _DW, 55), (56 + _DW, 63), (20, 67), (-20, 67), (-56 - _DW, 63), (-72 - _DW, 55)]
SPINE_INNER = [(-69 - _DW, TOP - 1), (69 + _DW, TOP - 1), (69 + _DW, 53.5), (54.5 + _DW, 60.5), (19, 64), (-19, 64),
               (-54.5 - _DW, 60.5), (-69 - _DW, 53.5)]


def top_armor():
    a = yz_prism(SPINE_PROFILE, -MID_X + 1, MID_X - 1)
    a = a.cut(yz_prism(SPINE_INNER, -MID_X + 4, MID_X - 4))
    # 前端进风格栅（树莓派上方）+ 后段排风格栅（4010 风扇上方）
    for k in range(6):
        x = 20 + k * 9
        a = a.cut(box(x, x + 4, -16, 16, 60, 70))
    for k in range(5):
        x = -86 + k * 8
        a = a.cut(box(x, x + 4, -16, 16, 60, 70))
    # 侧面灯槽
    for s in (+1, -1):
        pts = [(s * (66.6 + _DW), 57.2), (s * (61.0 + _DW), 60.0), (s * (61.5 + _DW), 61.4), (s * (67.1 + _DW), 58.6)]
        a = a.cut(yz_prism(pts, -70, 70))
    for x in (-MID_X + 15, MID_X - 15):
        for y in (RAIL_Y, -RAIL_Y):
            a = a.cut(cyl_z(2.75, TOP - 1, 58, x, y))
    # v0.5：缝线（两道横向）、任务导轨立柱让位孔、任务接口 GX16 航空插座孔
    skin = yz_prism(SPINE_PROFILE, -MID_X + 1, MID_X - 1).cut(yz_prism(offset_poly(SPINE_PROFILE, SEAM_D), -MID_X, MID_X))
    for x in (-38.0, 12.0):
        a = a.cut(skin.intersect(box(x - SEAM_W / 2, x + SEAM_W / 2, -90, 90, TOP + 2, 80)))
    for x in RAIL_POSTS_X:
        for y in (RAIL_YC, -RAIL_YC):
            a = a.cut(box(x - RAIL_POST[0] / 2 - 0.5, x + RAIL_POST[0] / 2 + 0.5, y - RAIL_POST[1] / 2 - 0.5, y + RAIL_POST[1] / 2 + 0.5, TOP - 1, 80))
    a = a.cut(cyl_z(8.1, TOP - 1, 80, *GX16_XY))
    a = a.cut(cyl_z(6.0, TOP - 1, 80, LIDAR["x"], 24.0))          # 激光雷达过线孔（Φ12 橡胶护线圈）
    return a


def touch_pads_back():
    """背部前后两个触摸区（每区左右两片铜箔并联），贴在顶甲两侧斜面内侧，避开中脊上的进/排风格栅"""
    out = []
    for (xa, xb) in ((12, 82), (-66, -12)):                     # v0.5 后区缩短，让出 GX16 任务接口
        for s in (+1, -1):
            # 内斜面 (19,64)->(54.5+_DW,60.5)，铜箔贴在下面 0.3mm
            zs = lambda y: 64 - (y - 19) * 3.5 / (35.5 + _DW) - 0.6
            ya, yb = 22, 46 + _DW                                 # v0.5 外缘收窄，让出任务导轨立柱
            pts = [(s * ya, zs(ya)), (s * yb, zs(yb)), (s * yb, zs(yb) - 0.4), (s * ya, zs(ya) - 0.4)]
            out.append(yz_prism(pts, xa, xb))
    return out


def spine_lights():
    """机脊灯：琥珀橙，导光条是一整条，效果图按 3 段画（WS2812 分段点亮）"""
    out = []
    for s in (+1, -1):
        pts = [(s * (66.4 + _DW), 57.4), (s * (61.2 + _DW), 60.0), (s * (61.6 + _DW), 61.0), (s * (66.8 + _DW), 58.4)]
        for (a, b) in ((-54, -9), (9, 54)):                     # 两段，让开导轨立柱
            out.append(yz_prism(pts, a, b))
    return out


def spine_light_strip():
    """机脊导光条（整条，左右通用）"""
    pts = [(66.4 + _DW, 57.4), (61.2 + _DW, 60.0), (61.6 + _DW, 61.0), (66.8 + _DW, 58.4)]
    return yz_prism(pts, -68, 68)


def side_armor_L():
    """侧甲：3mm 底板 + 3.5mm 外层装甲（斜切角）。v0.5：缝线把外层分成 3 块，后下方浅格栅，前上方一小段琥珀橙状态灯"""
    y0, y1, y2 = BODY_W / 2 - 2, BODY_W / 2 + 1, BODY_W / 2 + 4.5
    p = box(-MID_X + 0.5, MID_X - 0.5, y0, y1, BODY_ZMIN + 2, BODY_ZMAX - 2)
    pts = [(-86, -38), (62, -38), (86, -16), (86, 30), (72, 40), (-72, 40), (-86, 30)]
    top = xz_prism(pts, y1, y2)
    top = chamfer_edges(top, ">Y", 2.0)
    p = p.fuse(top).clean()
    g = lambda x0, x1, z0, z1: box(x0, x1, y2 - SEAM_D, y2 + 1, z0, z1)
    p = p.cut(g(-10 - SEAM_W / 2, -10 + SEAM_W / 2, -40, 42))                  # 竖缝
    p = p.cut(g(-90, -10, 6 - SEAM_W / 2, 6 + SEAM_W / 2))                     # 后半块再横分一刀
    for k in range(5):                                                         # 后下方浅格栅（不打通，侧面不进风）
        z = -30 + k * 6
        p = p.cut(box(-76, -36, y2 - 1.2, y2 + 1, z, z + 2.4))
    p = p.cut(box(34, 72, y2 - 1.2, y2 + 1, 25, 28))                           # 状态灯槽
    for x in (-MID_X + 15, 0, MID_X - 15):
        for z in (35.0, -35.0):
            p = p.cut(cyl_y(2.75, y0 - 1, y2 + 1, x, z))
    return p


def side_trim_L():
    """侧面银色饰条：贴在侧甲底板上沿，外层装甲上面那一条（银色 PLA）"""
    y1, y2 = BODY_W / 2 + 1, BODY_W / 2 + 3.5
    t = chamfer_edges(box(-80, 80, y1, y2, 41.2, 46.6), ">Y", 0.8)
    for x in (-60.0, 60.0):
        t = t.cut(cyl_y(1.1, y1 - 1, y2 + 1, x, 43.9))
    return t


def side_light_L():
    y2 = BODY_W / 2 + 4.5
    return box(34.5, 71.5, y2 - 1.1, y2 + 0.4, 25.4, 27.6)


def shoulder_armor_FL():
    """肩甲：贴在前左髋座侧面（镜像得到其余三块）"""
    y0, y1 = BODY_W / 2, BODY_W / 2 + 3.5
    x0, x1 = MID_X + 3, HIP_X - 1 - HIP_CORNER - 2          # v0.4 跟着髋座前外角缩短
    pts = [(x0, -36), (x1 - 16, -36), (x1, -20), (x1, 34), (x1 - 10, 44), (x0, 44)]
    p = xz_prism(pts, y0, y1)
    p = chamfer_edges(p, ">Y", 1.2)
    p = p.cut(xz_prism([(x0 + 10, -24), (x0 + 14, -24), (x0 + 30, 10), (x0 + 26, 10)], y0 - 1, y1 + 1))
    return p


def _shield_outline(z_cut=-148.0, n=40):
    """大腿装甲外形 = 大腿外壳轮廓（髋端 R44 圆弧 + 两侧切线收到膝盖），边缘和外壳齐平，没有一圈凸边；
    在 z_cut 以下斜切收口，让出银色膝盖。（外壳不能再放大：大腿板再宽就斜放不进 256 打印床）"""
    r1, r2, L = R_HIP_PLATE, R_KNEE_PLATE, L_THIGH
    s = (r1 - r2) / L
    nx, nz = math.sqrt(1 - s * s), -s
    a0 = math.atan2(nz, nx)                                         # 右侧切点在髋端圆上的角度（略低于水平）
    arc = [(r1 * math.cos(a0 + (math.pi - 2 * a0) * k / n), r1 * math.sin(a0 + (math.pi - 2 * a0) * k / n)) for k in range(n + 1)]
    A, B = (r1 * nx, r1 * nz), (r2 * nx, -L + r2 * nz)              # 右侧切线 A -> B
    t = (z_cut - A[1]) / (B[1] - A[1])
    xc = A[0] + (B[0] - A[0]) * t
    return [(xc, z_cut)] + arc + [(-xc, z_cut), (-22.0, z_cut - 12.0), (22.0, z_cut - 12.0)]


THIGH_SHIELD = _shield_outline()   # 腿坐标 (x, z)：圆弧肩，往下越来越细，和大腿外壳齐平


def thigh_cover():
    """大腿装甲（腿坐标零位）。v0.6 参考概念图：盾牌形折面外形，上宽下窄，在膝盖上面就收住，
    让银色膝盖露出来；装甲边缘和大腿外壳齐平（v0.6 用户要求：外面一圈不能凸出来）；2mm 底板 + 2.5mm 凸起中板（大斜切）
    + 两道斜向折线 + 顶部一小段琥珀橙灯；髋端留给银色端盖。v0.6 去掉张紧轮长孔：调皮带时拆下装甲（4 颗 M3）"""
    import leg
    y0, y1, y2 = Y_OUTER[1], Y_OUTER[1] + 2.0, Y_OUTER[1] + 4.5
    c = chamfer_edges(xz_prism(THIGH_SHIELD, y0, y1), ">Y", 0.8)
    mid = chamfer_edges(xz_prism(offset_poly(THIGH_SHIELD, 6.0), y1 - 0.01, y2), ">Y", 2.2)
    c = c.fuse(mid).clean()
    c = c.cut(cyl_y(16, y0 - 1, y2 + 1))                         # 让出轴承凸台（髋端盖盖住）
    for (x, z) in THIGH_COVER_SCREWS:
        c = c.cut(cyl_y(1.7, y0 - 1, y2 + 1, x, z)).cut(cyl_y(3.0, y2 - 1.6, y2 + 1, x, z))   # M3 沉头，锁进大腿外板的热熔螺母
    panel = xz_prism(offset_poly(THIGH_SHIELD, 8.0), y2 - SEAM_D - 0.1, y2 + 1)
    for (pa, pb) in (((-50, -44), (50, -84)), ((-50, -98), (50, -134))):   # 斜向折线（缝线），只刻在中板上
        (xa, za), (xb, zb) = pa, pb
        k = SEAM_W / 2 / math.cos(math.atan2(zb - za, xb - xa))
        c = c.cut(xz_prism([(xa, za - k), (xb, zb - k), (xb, zb + k), (xa, za + k)], y2 - SEAM_D, y2 + 1).intersect(panel))
    c = c.cut(box(-12, 12, y2 - 1.2, y2 + 1, 27.5, 30.5))         # 顶部状态灯槽
    for (x, z) in HIP_CAP_SCREWS:
        c = c.cut(cyl_y(0.8, y0 - 1, y2 + 1, x, z))                # 髋端盖 M2 自攻孔
    return c


def thigh_light():
    """大腿装甲顶部的琥珀橙状态灯（导光条）"""
    y2 = Y_OUTER[1] + 4.5
    return box(-11.6, 11.6, y2 - 1.1, y2 + 0.3, 27.9, 30.1)


THIGH_COVER_SCREWS = [(-30.0, -40.0), (30.0, -40.0), (-25.0, -138.0), (25.0, -138.0)]
HIP_CAP_SCREWS = [(19.0 * math.cos(math.radians(a)), 19.0 * math.sin(math.radians(a))) for a in (90, 210, 330)]
KNEE_CAP_SCREWS = [(18.0 * math.cos(math.radians(a)), 18.0 * math.sin(math.radians(a))) for a in (30, 150, 270)]


def hip_cap():
    """髋端银色端盖（腿坐标）：盖住小带轮外支撑轴承，看起来像电机端面"""
    y0 = Y_OUTER[1] + 4.5
    c = chamfer_edges(cyl_y(23, y0, y0 + 3.0), ">Y", 1.2)
    c = c.cut(cyl_y(9, y0 + 2.2, y0 + 4))                         # 中心浅凹
    for (x, z) in HIP_CAP_SCREWS:
        c = c.cut(cyl_y(1.1, y0 - 1, y0 + 4, x, z)).cut(cyl_y(2.0, y0 + 2.2, y0 + 4, x, z))
    for a in range(0, 360, 60):                                   # 外圈 6 道放射缝
        c = c.cut(box(15, 22, -SEAM_W / 2, SEAM_W / 2, -0.5, 0.5).rotate(V(0, 0, 0), V(0, 0, 1), 0)
                  .rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(0, y0 + 3.0, 0)).rotate(V(0, 0, 0), V(0, 1, 0), a))
    return c.clean()


def knee_cap():
    """银色膝盖（腿坐标零位）：v0.6 大腿装甲在膝盖上面收住，膝盖整个露出来。
    Φ50 端盖 + 下面一圈垫环（直接贴在大腿外板上，3 颗 M2 自攻），中间深色凹槽，像概念图的金属关节"""
    y0, y1 = Y_OUTER[1], Y_OUTER[1] + 2.6
    kz = -L_THIGH
    ring = cyl_y(21, y0, y1, 0, kz).cut(cyl_y(12.5, y0 - 1, y1 + 1, 0, kz))
    disc = chamfer_edges(cyl_y(25, y1, y1 + 3.4, 0, kz), ">Y", 1.6)
    c = ring.fuse(disc).clean()
    c = c.cut(cyl_y(9, y1 + 2.4, y1 + 5, 0, kz))
    c = c.cut(cyl_y(16, y1 + 2.8, y1 + 5, 0, kz).cut(cyl_y(15.2, y1 + 2.0, y1 + 6, 0, kz)))   # 一圈细环线
    for (x, z) in KNEE_CAP_SCREWS:
        c = c.cut(cyl_y(1.1, y0 - 1, y1 + 5, x, z + kz)).cut(cyl_y(2.0, y1 + 2.0, y1 + 5, x, z + kz))
    return c.clean()


def battery_door():
    """底部电池门：两颗 1/4 转快拆扣，电池托盘挂在门上"""
    w = BODY_W / 2
    b = box(-MID_X + 0.5, MID_X - 0.5, -w, w, BODY_ZMIN - 4, BODY_ZMIN)
    b = chamfer_edges(b, "|Z", 4.0)
    for x in (-MID_X + 14, MID_X - 14):
        b = b.cut(cyl_z(5.2, BODY_ZMIN - 5, BODY_ZMIN + 1, x, 0))      # 快拆扣孔
    b = b.cut(box(-20, 20, -w - 1, -w + 6, BODY_ZMIN - 5, BODY_ZMIN - 2))   # 抠手位
    # 托盘侧挡
    for s in (+1, -1):
        b = b.fuse(box(-72, 72, s * 33 - 1.5, s * 33 + 1.5, BODY_ZMIN, BODY_ZMIN + 10))
    return b


def handle_v2():
    z0 = 67.0
    pts = [(-66, z0), (66, z0), (58, z0 + 26), (-58, z0 + 26)]
    h = xz_prism(pts, -10, 10)
    h = h.cut(xz_prism([(-48, z0 - 1), (48, z0 - 1), (45, z0 + 15), (-45, z0 + 15)], -11, 11))
    h = chamfer_edges(h, "|Y", 3.0)
    h = chamfer_edges(h, "|X", 2.0)
    for x in (-57.0, 57.0):
        h = h.cut(cyl_z(2.75, z0 - 1, z0 + 12, x, 0))
    return h


# ------------------------------------------------------------------ 背部：任务导轨 + 激光雷达模块 + 示范任务箱
RAIL_YC = 66.0                     # 导轨中心 y（正下方就是机身上层 2020 型材，立柱 M5 直接锁进型材槽）
RAIL_POSTS_X = (-62.0, 0.0, 62.0)      # 避开顶甲 x=±78 的固定螺丝和机脊灯段
RAIL_POST = (12.0, 12.0)           # 立柱截面 x × y
RAIL_BAR = (84.0, 94.0)            # 导轨横杆 z（下面离顶甲约 21mm，手指能伸进去当提手）
RAIL_X = (-84.0, 64.0)
GX16_XY = (-80.0, -34.0)           # 任务接口 GX16 航空插座（顶甲右后斜面，竖直朝上）
LIDAR = dict(x=46.0, d=60.0, h=35.0, plate=(30.0, 62.0))   # 单线激光雷达占位 Φ60×35（买前按实物改），装在跨两根导轨的桥板上
LIDAR_Z0 = RAIL_BAR[1] + 4.0       # 雷达底面（桥板 4mm）
LIDAR_PLANE = LIDAR_Z0 + 22.0      # 扫描平面大约在底面以上 22mm（按实物改）
BOX = dict(x=(-80.0, 8.0), w=112.0, z=(80.0, 114.0))      # 示范任务箱：夹在两根导轨之间，顶面低于雷达扫描平面


def mission_rail_L():
    """左任务导轨（右边镜像）：横杆 + 3 根立柱，立柱 M5 锁进下面的 2020 型材。兼做提手"""
    y0, y1 = RAIL_YC - 6, RAIL_YC + 6
    z0, z1 = RAIL_BAR
    bar = xz_prism([(RAIL_X[0] + 8, z1), (RAIL_X[1] - 8, z1), (RAIL_X[1], z1 - 8), (RAIL_X[1] - 4, z0),
                    (RAIL_X[0] + 4, z0), (RAIL_X[0], z1 - 8)], y0, y1)
    r = bar
    for x in RAIL_POSTS_X:
        r = r.fuse(box(x - RAIL_POST[0] / 2, x + RAIL_POST[0] / 2, y0, y1, TOP, z0 + 1))
    r = chamfer_edges(r.clean(), "|X", 1.5)
    for x in RAIL_POSTS_X:
        r = r.cut(cyl_z(2.75, TOP - 1, z1 + 1, x, RAIL_YC)).cut(cyl_z(5.0, z1 - 6, z1 + 1, x, RAIL_YC))   # M5 + 沉头
    r = r.cut(box(RAIL_X[0] + 10, RAIL_X[1] - 10, y1 - SEAM_D, y1 + 1, (z0 + z1) / 2 - 1.5, (z0 + z1) / 2 + 1.5))   # 外侧细灯槽
    return r.clean()


def rail_light_L():
    y1 = RAIL_YC + 6
    zc = (RAIL_BAR[0] + RAIL_BAR[1]) / 2
    return box(RAIL_X[0] + 10.5, RAIL_X[0] + 40, y1 - 0.5, y1 + 0.2, zc - 1.2, zc + 1.2)


def lidar_module():
    """激光雷达模块：桥板（跨在两根导轨上，4 个快拆夹） + 雷达占位 + 琥珀橙灯环。返回 (桥板, 雷达, 灯环)"""
    L = LIDAR
    xa, xb = L["x"] - L["plate"][0] / 2, L["x"] + L["plate"][0] / 2
    z0 = RAIL_BAR[1]
    plate = chamfer_edges(box(xa, xb, -RAIL_YC - 6, RAIL_YC + 6, z0, z0 + 4), "|Z", 4.0)
    plate = plate.fuse(cyl_z(L["d"] / 2 - 7.5, z0 + 4, LIDAR_Z0 + 1.5, L["x"], 0))
    for s in (+1, -1):                                           # 夹住导轨的下爪（导轨横杆两侧各一片）
        for (ya, yb) in ((RAIL_YC + 6.2, RAIL_YC + 9.2), (RAIL_YC - 9.2, RAIL_YC - 6.2)):
            plate = plate.fuse(box(xa + 3, xb - 3, min(s * ya, s * yb), max(s * ya, s * yb), z0 - 8, z0 + 0.1))
    plate = plate.cut(cyl_z(6, z0 - 1, LIDAR_Z0 + 2, L["x"], 0))  # 雷达线从中间穿下去
    lidar = chamfer_edges(cyl_z(L["d"] / 2, LIDAR_Z0 + 1.5, LIDAR_Z0 + L["h"], L["x"], 0), ">Z", 3.0)
    ring = cyl_z(L["d"] / 2 - 3.6, z0 + 4.05, LIDAR_Z0 + 1.45, L["x"], 0).cut(cyl_z(L["d"] / 2 - 7.3, z0 + 3, LIDAR_Z0 + 2, L["x"], 0))
    return plate.clean(), lidar, ring


def task_box():
    """示范任务箱（外观规范的样板）：箱体夹在两根导轨之间，两侧翻边压在导轨上，4 个快拆夹锁住。
    前面一个相机窗，后面一个 GX16 线缆口，顶面凹进去一个提手。返回 (箱体, 盖子)"""
    x0, x1 = BOX["x"]
    w = BOX["w"] / 2
    z0, z1 = BOX["z"]
    body = box(x0, x1, -w, w, z0, z1 - 3)
    body = chamfer_edges(body, "|X", 5.0)
    body = chamfer_edges(body, "|Y", 3.0)
    body = body.cut(box(x0 + 2, x1 - 2, -w + 2, w - 2, z0 + 2, z1))
    for s in (+1, -1):                                           # 压在导轨上的翻边
        fl = box(x0 + 6, x1 - 6, min(s * (w - 2), s * (RAIL_YC + 6)), max(s * (w - 2), s * (RAIL_YC + 6)), RAIL_BAR[1], RAIL_BAR[1] + 3)
        body = body.fuse(chamfer_edges(fl, "|Z", 3.0))
    body = body.cut(box(x1 - 3, x1 + 1, -14, 14, 88, 104))       # 前面相机窗
    body = body.cut(cyl_x(8.1, x0 - 1, x0 + 3, 0, 97))           # 后面 GX16 线缆口
    body = body.cut(box(x0 - 1, x0 + 2.5, -30.2, -17.8, 104.8, 107.2))   # 状态灯槽
    body = body.cut(box(x0 - 1, x1 + 1, -w - 1, w + 1, (z0 + RAIL_BAR[1]) / 2 - SEAM_W / 2, (z0 + RAIL_BAR[1]) / 2 + SEAM_W / 2)
                    .cut(box(x0 + 0.6, x1 - 0.6, -w + 0.6, w - 0.6, 60, 130)))   # 腰线
    lid = chamfer_edges(box(x0, x1, -w, w, z1 - 3, z1), "|X", 5.0)
    lid = chamfer_edges(lid, ">Z", 1.5)
    lid = lid.cut(chamfer_edges(box(x0 + 26, x1 - 26, -16, 16, z1 - 2, z1 + 1), "|Z", 6.0))   # 凹进去的提手
    lid = lid.fuse(box(x0 + 30, x1 - 30, -4, 4, z1 - 3, z1 - 0.5))
    lid = lid.cut(box(x0 + 8, x0 + 20, -w - 1, w + 1, z1 - SEAM_D, z1 + 1))
    return body.clean(), lid.clean()


def task_box_parts():
    """示范任务箱上的小件：相机窗（黑）、状态灯（琥珀）、快拆夹（银）"""
    x0, x1 = BOX["x"]
    win = box(x1 - 2.4, x1 - 0.6, -13.5, 13.5, 88.5, 103.5)
    led = box(x0 - 0.4, x0 + 1.0, -30, -18, 105, 107)
    clamps = []
    for x in (x0 + 12, x1 - 12):
        for s in (+1, -1):
            clamps.append(chamfer_edges(box(x - 6, x + 6, s * (RAIL_YC + 6.2) - (0 if s > 0 else 4), s * (RAIL_YC + 6.2) + (4 if s > 0 else 0),
                                            RAIL_BAR[0] + 2, RAIL_BAR[1] + 3), ">Z", 1.0))
    return win, led, clamps


def gx16_socket():
    """任务接口 GX16 航空插座（面板座占位）：24V / 5V / CAN"""
    x, y = GX16_XY
    zt = 67 - (abs(y) - 20) * 4 / (46 + _DW - 10) + 1.0           # 斜面上最高那一侧再抬 1mm
    flange = cyl_z(9.5, zt, zt + 2.0, x, y)
    body = cyl_z(8.0, zt - 18, zt, x, y)
    plug = cyl_z(7.0, zt + 2.0, zt + 9, x, y)
    return flange.fuse(body).fuse(plug).clean()


def estop_v2():
    x, z0 = -(HIP_X - 30), 70.0
    base = cyl_z(11, z0 - 2, z0 + 6, x, 0)
    cap = chamfer_edges(cyl_z(15, z0 + 6, z0 + 16, x, 0), ">Z", 3.0)
    return base, cap


def split_half(shape, keep_pos=True):
    """沿 y=0 切半（大件分两半打印，切面朝下）"""
    big = 400
    cutter = box(-big, big, 0 if keep_pos else -big, big if keep_pos else 0, -big, big)
    return shape.intersect(cutter)
