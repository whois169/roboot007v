"""
RoboDog v0.2 外观件：科幻装甲（机身坐标，X 前 / Y 左 / Z 上）
  头部：前倾楔形面罩 + 双眼灯 + 一对后掠“耳鳍”
  尾部：收窄的尾罩 + 上翘尾鳍 + 红色尾灯 + 充电口 + 急停
  机身：带中脊的顶部装甲、分层侧甲、肩甲、大腿装甲片
灯条只做外形（渲染时发光），实物用 WS2812B / COB 灯带塞进凹槽。
"""
import math
import cadquery as cq
from config import *
from parts import V, box, cyl_y, cyl_z, cyl_x, xz_prism, yz_prism, xy_prism, chamfer_edges, tapered_slot_xz

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
    (-MID_X + 1, 66, TOP, 66, 8, 0.5),
    (-(HIP_X + 2), 56, TOP, 72, 10, 0.5),
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
    t = t.cut(box(-(HIP_X + 20), -(HIP_X + 4), -60, -40, 54, 70).rotate(V(-(HIP_X + 12), -46, 62), V(1, 0, 0), -20))   # 右侧
    # 尾鳍插槽
    t = t.cut(box(-(HIP_X + 50), -(HIP_X + 20), -3.2, 3.2, 60, 80))
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
SPINE_PROFILE = [(-72, TOP), (72, TOP), (72, 55), (56, 63), (20, 67), (-20, 67), (-56, 63), (-72, 55)]
SPINE_INNER = [(-69, TOP - 1), (69, TOP - 1), (69, 53.5), (54.5, 60.5), (19, 64), (-19, 64), (-54.5, 60.5), (-69, 53.5)]


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
        pts = [(s * 66.6, 57.2), (s * 61.0, 60.0), (s * 61.5, 61.4), (s * 67.1, 58.6)]
        a = a.cut(yz_prism(pts, -70, 70))
    for x in (-MID_X + 15, MID_X - 15):
        for y in (58.0, -58.0):
            a = a.cut(cyl_z(2.75, TOP - 1, 58, x, y))
    return a


def touch_pads_back():
    """背部前后两个触摸区（每区左右两片铜箔并联），贴在顶甲两侧斜面内侧，避开中脊上的进/排风格栅"""
    out = []
    for (xa, xb) in ((12, 82), (-82, -12)):
        for s in (+1, -1):
            # 内斜面 (19,64)->(54.5,60.5)，铜箔贴在下面 0.3mm
            pts = [(s * 22, 63.4), (s * 52, 60.4), (s * 52, 60.0), (s * 22, 63.0)]
            out.append(yz_prism(pts, xa, xb))
    return out


def spine_lights():
    out = []
    for s in (+1, -1):
        pts = [(s * 66.4, 57.4), (s * 61.2, 60.0), (s * 61.6, 61.0), (s * 66.8, 58.4)]
        out.append(yz_prism(pts, -68, 68))
    return out


def side_armor_L():
    """侧甲：3mm 底板 + 3.5mm 外层装甲（斜切角），外层上开状态灯槽"""
    y0, y1, y2 = BODY_W / 2 - 2, BODY_W / 2 + 1, BODY_W / 2 + 4.5
    p = box(-MID_X + 0.5, MID_X - 0.5, y0, y1, BODY_ZMIN + 2, BODY_ZMAX - 2)
    pts = [(-86, -38), (62, -38), (86, -16), (86, 30), (72, 44), (-72, 44), (-86, 30)]
    top = xz_prism(pts, y1, y2)
    top = chamfer_edges(top, ">Y", 1.4)
    p = p.fuse(top).clean()
    # 斜向装饰槽：只切外层装甲，不打通底板（避免侧面进风抢走流过树莓派的风）
    for k in range(5):
        x = -70 + k * 10
        p = p.cut(xz_prism([(x, -28), (x + 4, -28), (x + 20, 8), (x + 16, 8)], y1 + 0.8, y2 + 1))
    p = p.cut(box(-20, 70, y2 - 1.2, y2 + 1, 26, 29))          # 状态灯槽
    for x in (-MID_X + 15, 0, MID_X - 15):
        for z in (40.0, -35.0):
            p = p.cut(cyl_y(2.75, y0 - 1, y2 + 1, x, z))
    return p


def side_light_L():
    y2 = BODY_W / 2 + 4.5
    return box(-19.5, 69.5, y2 - 1.1, y2 + 0.4, 26.4, 28.6)


def shoulder_armor_FL():
    """肩甲：贴在前左髋座侧面（镜像得到其余三块）"""
    y0, y1 = BODY_W / 2, BODY_W / 2 + 3.5
    x0, x1 = MID_X + 3, HIP_X - 3
    pts = [(x0, -36), (x1 - 16, -36), (x1, -20), (x1, 34), (x1 - 10, 44), (x0, 44)]
    p = xz_prism(pts, y0, y1)
    p = chamfer_edges(p, ">Y", 1.2)
    p = p.cut(xz_prism([(x0 + 10, -24), (x0 + 14, -24), (x0 + 30, 10), (x0 + 26, 10)], y0 - 1, y1 + 1))
    return p


def thigh_cover():
    """大腿装甲片（腿坐标，零位），螺接在大腿外板外侧"""
    y0, y1 = Y_OUTER[1], Y_OUTER[1] + 3.5
    c = tapered_slot_xz(R_HIP_PLATE - 3, R_KNEE_PLATE - 3, L_THIGH - 12, y0, y1)
    c = chamfer_edges(c, ">Y", 1.4)
    c = c.cut(cyl_y(16, y0 - 1, y1 + 1))                         # 让出轴承凸台
    c = c.cut(cyl_y(12.5, y0 - 1, y1 + 1, 0, -L_THIGH))         # 让出膝轴凸台
    import leg
    ix, iz = leg.IDLER_POS
    c = c.cut(cyl_y(6.5, y0 - 1, y1 + 1, ix, iz))                # 张紧轮螺丝头（调皮带时不用拆装甲）
    c = c.cut(xz_prism([(-6, -70), (6, -70), (4, -150), (-4, -150)], y1 - 1.2, y1 + 1))   # 中线凹槽
    return c


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
