"""
RoboDog v0.1 零件几何。所有零件按“装配坐标”建模：
  - 机身件：机身坐标（X 前 / Y 左 / Z 上，原点机身中心）
  - 腿部件：腿坐标（原点 = HFE 轴线与 HFE 输出面交点，y 向外；零位时大腿竖直向下）
  - 小腿件：膝坐标（原点 = 膝轴与 y=0 平面交点；零位时小腿竖直向下）
只建左前腿，其余三条腿用镜像得到。
"""
import math
import cadquery as cq
from config import *

V = cq.Vector


# ---------------------------------------------------------------- 基础工具
def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl_x(r, x0, x1, y=0.0, z=0.0):
    return cq.Solid.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return cq.Solid.makeCylinder(r, y1 - y0, V(x, y0, z), V(0, 1, 0))


def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def xz_prism(pts, y0, y1):
    """(x,z) 多边形沿 +Y 拉伸到 [y0,y1]"""
    return (cq.Workplane("XZ", origin=(0, y0, 0)).polyline(pts).close()
            .extrude(-(y1 - y0)).val())


def xy_prism(pts, z0, z1):
    return (cq.Workplane("XY", origin=(0, 0, z0)).polyline(pts).close()
            .extrude(z1 - z0).val())


def yz_prism(pts, x0, x1):
    """(y,z) 多边形沿 +X 拉伸到 [x0,x1]"""
    return (cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close()
            .extrude(x1 - x0).val())


def tapered_slot_xz(r1, r2, L, y0, y1, c1=(0.0, 0.0)):
    """两圆 (c1,r1) 与 (c1 - (0,L), r2) 的外切包络，沿 +Y 拉伸"""
    cx, cz = c1
    s = (r1 - r2) / L
    nx, nz = math.sqrt(1 - s * s), -s
    A = (cx + r1 * nx, cz + r1 * nz)
    B = (cx + r2 * nx, cz - L + r2 * nz)
    M2 = (cx, cz - L - r2)
    C = (cx - r2 * nx, cz - L + r2 * nz)
    D = (cx - r1 * nx, cz + r1 * nz)
    M1 = (cx, cz + r1)
    return (cq.Workplane("XZ", origin=(0, y0, 0)).moveTo(*A).lineTo(*B)
            .threePointArc(M2, C).lineTo(*D).threePointArc(M1, A).close()
            .extrude(-(y1 - y0)).val())


def stadium_y(p1, p2, r, y0, y1):
    """XZ 平面内 p1-p2 的长圆孔（腰形孔），沿 Y"""
    s = cyl_y(r, y0, y1, *p1).fuse(cyl_y(r, y0, y1, *p2))
    (x1, z1), (x2, z2) = p1, p2
    ang = math.atan2(z2 - z1, x2 - x1)
    nx, nz = -math.sin(ang) * r, math.cos(ang) * r
    pts = [(x1 + nx, z1 + nz), (x2 + nx, z2 + nz), (x2 - nx, z2 - nz), (x1 - nx, z1 - nz)]
    return s.fuse(xz_prism(pts, y0, y1)).clean()


def bolt_circle(pcd, n, phase_deg=0.0):
    out = []
    for i in range(n):
        a = math.radians(phase_deg + 360.0 * i / n)
        out.append((pcd / 2 * math.cos(a), pcd / 2 * math.sin(a)))
    return out


def fillet_edges(solid, selector, r):
    try:
        return cq.Workplane().add(solid).edges(selector).fillet(r).val()
    except Exception:
        return solid


def chamfer_edges(solid, selector, r):
    try:
        return cq.Workplane().add(solid).edges(selector).chamfer(r).val()
    except Exception:
        return solid


def rot_y_vec(x, z, deg):
    """右手系绕 Y 旋转 (x,z) 向量，与 cadquery rotate 一致"""
    a = math.radians(deg)
    return (x * math.cos(a) + z * math.sin(a), -x * math.sin(a) + z * math.cos(a))


# ---------------------------------------------------------------- 外购件（占位模型）
def motor_y(y_back, cx=0.0, cz=0.0, outward=True):
    """沿 Y 的关节电机，背面在 y_back，输出法兰朝 +Y（outward）"""
    M = MOTOR
    body_h = M["H"] - M["flange_h"]
    if outward:
        body = cyl_y(M["D"] / 2, y_back, y_back + body_h, cx, cz)
        fl = cyl_y(M["flange_D"] / 2, y_back + body_h, y_back + M["H"], cx, cz)
    else:
        body = cyl_y(M["D"] / 2, y_back - body_h, y_back, cx, cz)
        fl = cyl_y(M["flange_D"] / 2, y_back - M["H"], y_back - body_h, cx, cz)
    body = chamfer_edges(body, "%CIRCLE", 1.5)
    # 机壳散热环纹，纯视觉
    for k in range(3):
        y = (y_back + 8 + k * 6) if outward else (y_back - 10 - k * 6)
        ring = cyl_y(M["D"] / 2 + 0.01, y, y + 2, cx, cz).cut(cyl_y(M["D"] / 2 - 0.8, y, y + 2, cx, cz))
        body = body.cut(ring)
    return body, fl


def motor_connector_y(y_back, ang, cx=0.0, cz=0.0):
    """沿 Y 的电机侧面插头座 + 插头预留空间。ang = 插头朝向（XZ 平面内，0°=+X，90°=+Z）"""
    M = MOTOR
    w, l, h = M["conn"]
    pw, pl, ph = M["plug"]
    r0 = M["D"] / 2
    yc = y_back + M["conn_from_rear"]
    seat = box(r0 - 1, r0 + h, yc - l / 2, yc + l / 2, -w / 2, w / 2)
    plug = box(r0 + h, r0 + h + ph, yc - pl / 2, yc + pl / 2, -pw / 2, pw / 2)
    # box 是沿 +X 建的，绕 Y 转到 ang（cadquery 绕 +Y 正转是 X->-Z，所以取负角）
    return [s.rotate(V(0, 0, 0), V(0, 1, 0), -ang).translate(V(cx, 0, cz)) for s in (seat, plug)]


def motor_connector_x(x_back, cy, ang, cz=0.0):
    """沿 X 的电机侧面插头座 + 预留。ang 在 YZ 平面内，0°=+Y，90°=+Z"""
    M = MOTOR
    w, l, h = M["conn"]
    pw, pl, ph = M["plug"]
    r0 = M["D"] / 2
    xc = x_back + M["conn_from_rear"]
    seat = box(xc - l / 2, xc + l / 2, r0 - 1, r0 + h, -w / 2, w / 2)
    plug = box(xc - pl / 2, xc + pl / 2, r0 + h, r0 + h + ph, -pw / 2, pw / 2)
    return [s.rotate(V(0, 0, 0), V(1, 0, 0), ang).translate(V(0, cy, cz)) for s in (seat, plug)]


def dowel_holes_x(solid, x0, x1, cy, cz, pcd, n, d, phase):
    for (dy, dz) in bolt_circle(pcd, n, phase):
        solid = solid.cut(cyl_x(d / 2 + 0.05, x0, x1, cy + dy, cz + dz))
    return solid


def dowel_holes_y(solid, y0, y1, cx, cz, pcd, n, d, phase):
    for (dx, dz) in bolt_circle(pcd, n, phase):
        solid = solid.cut(cyl_y(d / 2 + 0.05, y0, y1, cx + dx, cz + dz))
    return solid


def motor_x(x_face, cy, cz=0.0, facing=+1):
    """沿 X 的 HAA 电机，输出法兰面在 x_face，facing=+1 朝前"""
    M = MOTOR
    if facing > 0:
        body = cyl_x(M["D"] / 2, x_face - M["H"], x_face - M["flange_h"], cy, cz)
        fl = cyl_x(M["flange_D"] / 2, x_face - M["flange_h"], x_face, cy, cz)
    else:
        body = cyl_x(M["D"] / 2, x_face + M["flange_h"], x_face + M["H"], cy, cz)
        fl = cyl_x(M["flange_D"] / 2, x_face, x_face + M["flange_h"], cy, cz)
    return chamfer_edges(body, "%CIRCLE", 1.5), fl


def rail_2020(x0, x1, cy, cz):
    """2020 铝型材（外购，仅做外形示意：方管 + 四面 6mm 槽）"""
    r = box(x0, x1, cy - 10, cy + 10, cz - 10, cz + 10)
    r = r.cut(box(x0, x1, cy + 7, cy + 10, cz - 3, cz + 3))
    r = r.cut(box(x0, x1, cy - 10, cy - 7, cz - 3, cz + 3))
    r = r.cut(box(x0, x1, cy - 3, cy + 3, cz + 7, cz + 10))
    r = r.cut(box(x0, x1, cy - 3, cy + 3, cz - 10, cz - 7))
    return r.cut(cyl_x(2.1, x0, x1, cy, cz))


# ---------------------------------------------------------------- 机身打印件
def hip_block_front():
    """前髋座：装两个 HAA 电机（输出朝前），后壁接 4 根 2020 铝型材"""
    x0, x1 = HIP_X - BLOCK_DEPTH - 1, HIP_X - 1     # 前面留 1mm，法兰凸出于前面
    w = BODY_W / 2
    z0, z1 = BODY_ZMIN, BODY_ZMAX
    blk = box(x0, x1, -w, w, z0, z1)
    blk = chamfer_edges(blk, "|X", 8.0)
    # 内腔：前壁 8、后壁 10、侧壁 4、顶底 4
    # 底面敞开（倒扣打印无需内部支撑），由 hip_block_cover 封底
    cav = box(x0 + 10, x1 - 8, -w + 4, w - 4, z0 - 1, z1 - 4)
    blk = blk.cut(cav)
    M = MOTOR
    for s in (+1, -1):
        cy = s * HAA_Y
        blk = blk.cut(cyl_x(M["D"] / 2 + 0.3, x1 - 9, x1 + 1, cy, 0))           # 前壁电机孔
        blk = blk.cut(cyl_x(M["cable_hole"] / 2, x0 - 1, x0 + 11, cy, 0))       # 后壁出线孔
        for (dy, dz) in bolt_circle(M["rear_pcd"], M["rear_n"], 45):
            blk = blk.cut(cyl_x(M["rear_hole"] / 2, x0 - 1, x0 + 11, cy + dy, dz))
        # 后壁型材固定孔 M5
        for rz in (40.0, -35.0):
            blk = blk.cut(cyl_x(2.75, x0 - 1, x0 + 11, s * 58.0, rz))
    # 侧面凹槽装饰（机械感的分型线）
    for s in (+1, -1):
        blk = blk.cut(box(x0 + 14, x1 - 12, s * w - 1.2 if s > 0 else -w - 0.1,
                          s * w + 0.1 if s > 0 else -w + 1.2, z0 + 14, z1 - 14))
    # 顶面走线孔（到头部 / 尾部）
    blk = blk.cut(cyl_z(9, z1 - 5, z1 + 1, x0 + 34, 0))
    # 前壁线束口：腿上 2 个电机的线从这里进机身（外摆关节处留一圈余量）
    for s in (+1, -1):
        blk = blk.cut(box(x1 - 9, x1 + 1, s * 56 - 7, s * 56 + 7, 30, 41))
    return blk


def hip_block_cover():
    """髋座底盖（前后通用），4×M3 自攻到髋座侧壁"""
    x0, x1 = HIP_X - BLOCK_DEPTH - 1, HIP_X - 1
    w = BODY_W / 2
    c = box(x0 + 10.2, x1 - 8.2, -w + 4.2, w - 4.2, BODY_ZMIN, BODY_ZMIN + 3)
    c = c.fuse(box(x0 + 2, x1 - 2, -w + 6, w - 6, BODY_ZMIN - 2, BODY_ZMIN))
    c = c.cut(cyl_z(8, BODY_ZMIN - 3, BODY_ZMIN + 4, (x0 + x1) / 2 - 8, 0))  # 调试口
    # 散热槽：髋座里的 2 个外摆电机长时间出力会发热
    for k in range(5):
        y = -40 + k * 20
        c = c.cut(box(x0 + 16, x0 + 30, y - 2.5, y + 2.5, BODY_ZMIN - 3, BODY_ZMIN + 4))
    # 防跌落 ToF 窗口（VL53L1X 斜向前下方 35°）
    c = c.cut(cyl_z(4.5, BODY_ZMIN - 3, BODY_ZMIN + 4, x1 - 14, 0))
    return c


def cliff_tof_front():
    """防跌落 ToF（装在前髋座底盖内侧，斜向前下方）"""
    x1 = HIP_X - 1
    t = box(-6.5, 6.5, -9, 9, 0, 3)
    t = t.rotate(V(0, 0, 0), V(0, 1, 0), 35).translate(V(x1 - 17, 0, BODY_ZMIN + 9))
    return t


def haa_bracket_FL():
    """左前髋支架（L 形）：板 A 接 HAA 法兰，板 B 装 HFE 电机背面"""
    M = MOTOR
    xa0, xa1 = HIP_X + 0.5, HIP_X + 8.5
    x_hfe = HIP_X + HFE_DX
    yb0, yb1 = HFE_IN_Y - 8, HFE_IN_Y
    # 板 A（法向 X）：绕 HAA 轴的圆 + 向外伸到板 B
    RA = 30.0
    a = cyl_x(RA, xa0, xa1, HAA_Y, 0).fuse(box(xa0, xa1, HAA_Y, yb1, -RA, RA))
    # 板 B（法向 Y）：从板 A 伸到 HFE 电机外侧，HFE 处圆头
    RB = 34.0
    b = cyl_y(RB, yb0, yb1, x_hfe, 0).fuse(box(xa0, x_hfe, yb0, yb1, -RB, RB))
    br = a.fuse(b)
    # 上下加强筋
    for z in (RB - 6, -RB):
        g = xy_prism([(xa1, yb0), (xa1, 16.0), (x_hfe - 6, yb0)], z, z + 6)
        br = br.fuse(g)
    br = br.clean()
    # 板 A：HAA 输出法兰孔 + 中心孔
    br = br.cut(cyl_x(6, xa0 - 1, xa1 + 1, HAA_Y, 0))
    for (dy, dz) in bolt_circle(M["out_pcd"], M["out_n"], 0):
        br = br.cut(cyl_x(M["out_hole"] / 2, xa0 - 1, xa1 + 1, HAA_Y + dy, dz))
        br = br.cut(cyl_x(3.2, xa1 - 3.5, xa1 + 1, HAA_Y + dy, dz))           # 沉头
    br = dowel_holes_x(br, xa0 - 1, xa0 + 5, HAA_Y, 0, M["out_dowel_pcd"], M["out_dowel_n"],
                       M["out_dowel_d"], M["out_dowel_phase"])                  # Φ4 定位销
    # 板 B：HFE 背面安装孔 + 出线孔
    br = br.cut(cyl_y(M["cable_hole"] / 2, yb0 - 1, yb1 + 1, x_hfe, 0))
    for (dx, dz) in bolt_circle(M["rear_pcd"], M["rear_n"], 45):
        br = br.cut(cyl_y(M["rear_hole"] / 2, yb0 - 1, yb1 + 1, x_hfe + dx, dz))
    # 板 B 减重孔
    br = br.cut(stadium_y((xa1 + 8, 18), (xa1 + 8, -18), 4.5, yb0 - 1, yb1 + 1))
    return br


# ---------------------------------------------------------------- 腿（腿坐标）
# 大腿内外板之间的立柱（零位坐标 x,z）：全部放在大腿后/上侧，避开曲柄和连杆扫过的前侧
_RS = 36.0
STANDOFF_POS = [(-_RS * math.sin(math.radians(a)), -_RS * math.cos(math.radians(a))) for a in (90.0, 135.0, 180.0)]
STANDOFF_POS += [(-14.0, -L_THIGH / 2), (-11.0, -L_THIGH + 48)]


def _thigh_plate(y0, y1):
    p = tapered_slot_xz(R_HIP_PLATE, R_KNEE_PLATE, L_THIGH, y0, y1)
    # 减重腰形孔（偏后侧，前侧留给连杆的视觉通道）
    p = p.cut(stadium_y((4, -62), (2, -86), 7.0, y0 - 1, y1 + 1))
    p = p.cut(stadium_y((1, -116), (0, -136), 5.5, y0 - 1, y1 + 1))
    for (x, z) in STANDOFF_POS:
        p = p.cut(cyl_y(2.2, y0 - 1, y1 + 1, x, z))   # M4
    p = p.cut(cyl_y(KNEE_SHAFT_D / 2 + 0.1, y0 - 1, y1 + 1, 0, -L_THIGH))
    return p


def thigh_inner():
    M = MOTOR
    y0, y1 = Y_INNER
    p = _thigh_plate(y0, y1)
    p = p.cut(cyl_y(7, y0 - 1, y1 + 1))  # 中心走线孔
    # HFE 输出法兰：螺栓从 y1 侧装入，沉头
    for (dx, dz) in bolt_circle(M["out_pcd"], M["out_n"], 0):
        p = p.cut(cyl_y(M["out_hole"] / 2, y0 - 1, y1 + 1, dx, dz))
        p = p.cut(cyl_y(3.2, y1 - 3.5, y1 + 1, dx, dz))
    # KFE 电机背面：螺栓从 y0 侧装入，沉头，错开 45°
    for (dx, dz) in bolt_circle(M["rear_pcd"], M["rear_n"], 45):
        p = p.cut(cyl_y(M["rear_hole"] / 2, y0 - 1, y1 + 1, dx, dz))
        p = p.cut(cyl_y(3.2, y0 - 1, y0 + 3.5, dx, dz))
    # 后缘加强翻边
    lip = tapered_slot_xz(R_HIP_PLATE, R_KNEE_PLATE, L_THIGH, y1, y1 + 6)
    inner = tapered_slot_xz(R_HIP_PLATE - 4, R_KNEE_PLATE - 4, L_THIGH, y1 - 1, y1 + 7)
    lip = lip.cut(inner).cut(box(-0.5, 60, y1 - 1, y1 + 7, -L_THIGH - 30, 60))    # 只保留后半圈
    lip = lip.cut(cyl_y(MOTOR["D"] / 2 + 3, y1 - 1, y1 + 7))
    lip = lip.cut(cyl_y(22, y1 - 1, y1 + 7, 0, -L_THIGH))          # 给小腿轮毂让位
    p = p.fuse(lip).clean()
    return p


def thigh_outer():
    y0, y1 = Y_OUTER
    p = _thigh_plate(y0, y1)
    # 曲柄外端支撑轴承 688ZZ（8×16×5）
    boss = cyl_y(14, y1, y1 + 3)
    p = p.fuse(boss)
    p = p.cut(cyl_y(8.05, y0 - 1, y0 + 5))
    p = p.cut(cyl_y(5.0, y0 - 1, y1 + 4))
    # 膝轴外端凸台
    p = p.fuse(cyl_y(11, y1, y1 + 2.5, 0, -L_THIGH)).cut(cyl_y(KNEE_SHAFT_D / 2 + 0.1, y0, y1 + 4, 0, -L_THIGH))
    return p.clean()


def standoffs():
    out = []
    for (x, z) in STANDOFF_POS:
        s = cyl_y(4.5, Y_INNER[1], Y_OUTER[0], x, z).cut(cyl_y(2.2, Y_INNER[1] - 1, Y_OUTER[0] + 1, x, z))
        out.append(s)
    return out


def crank():
    """膝曲柄：装在 KFE 输出法兰上，销孔沿 +x"""
    M = MOTOR
    y0, y1 = Y_CRANK
    c = cyl_y(19, y0, y1)
    arm = tapered_slot_xz(9, 7, CRANK_R, y0, y1, (0, 0)).rotate(V(0, 0, 0), V(0, 1, 0), -90)
    c = c.fuse(arm)
    c = c.fuse(cyl_y(4.0, y1, Y_OUTER[0] + 5))          # Φ8 外支撑轴头
    c = c.cut(cyl_y(2.6, y0 - 1, y1 + 1, CRANK_R, 0))  # M5 销
    for (dx, dz) in bolt_circle(M["out_pcd"], M["out_n"], 0):
        c = c.cut(cyl_y(M["out_hole"] / 2, y0 - 1, y1 + 1, dx, dz))
        c = c.cut(cyl_y(3.2, y1 - 2.5, y1 + 1, dx, dz))
    return c.clean()


def shank(lever_dir_local):
    """小腿（膝坐标）：轮毂 + 摇臂耳 + 杆身。lever_dir_local = 摇臂销方向（单位向量 x,z）"""
    lx, lz = lever_dir_local
    px, pz = LEVER_R * lx, LEVER_R * lz
    hub = cyl_y(17, *Y_SHANK_HUB)
    hub = chamfer_edges(hub, "%CIRCLE", 1.5)
    # 摇臂耳片
    ear = cyl_y(12, *Y_LEVER).fuse(cyl_y(8, *Y_LEVER, px, pz))
    ang = math.atan2(pz, px)
    nx, nz = -math.sin(ang), math.cos(ang)
    ear = ear.fuse(xz_prism([(12 * nx, 12 * nz), (px + 8 * nx, pz + 8 * nz),
                             (px - 8 * nx, pz - 8 * nz), (-12 * nx, -12 * nz)], *Y_LEVER))
    # 杆身：上宽下窄的梯形截面
    zt = -L_SHANK + 12
    bar = xz_prism([(-15, -8), (15, -8), (9, zt), (-9, zt)], *SHANK_Y)
    bar = bar.cut(stadium_y((0, -50), (0, -120), 5.0, SHANK_Y[0] - 1, SHANK_Y[1] + 1))
    tip = box(-9, 9, SHANK_Y[0] + 3, SHANK_Y[1] - 3, -L_SHANK + 2, zt + 1)
    bar = chamfer_edges(bar, "|Y", 2.0)
    s = hub.fuse(ear).fuse(bar).fuse(tip).clean()
    # 轴承座：两端 6800ZZ（10×19×5），中间 Φ12 通孔
    s = s.cut(cyl_y(6, Y_SHANK_HUB[0] - 1, Y_LEVER[1] + 1))
    s = s.cut(cyl_y(9.55, Y_SHANK_HUB[0] - 1, Y_SHANK_HUB[0] + 5))
    s = s.cut(cyl_y(9.55, Y_LEVER[1] - 5, Y_LEVER[1] + 1))
    s = s.cut(cyl_y(2.6, Y_LEVER[0] - 1, Y_LEVER[1] + 1, px, pz))
    s = s.cut(cyl_x(2.2, -20, 20, (SHANK_Y[0] + SHANK_Y[1]) / 2, -L_SHANK + 6))   # 足端 M4 横销
    return s


def foot():
    yc = (SHANK_Y[0] + SHANK_Y[1]) / 2
    f = cq.Solid.makeSphere(FOOT_D / 2, V(0, yc, -L_SHANK), angleDegrees1=-90, angleDegrees2=90)
    f = f.fuse(cyl_z(12, -L_SHANK, -L_SHANK + 10, 0, yc))
    f = chamfer_edges(f, ">Z", 1.0)
    f = f.cut(box(-9.2, 9.2, SHANK_Y[0] + 2.8, SHANK_Y[1] - 2.8, -L_SHANK + 1.8, -L_SHANK + 20))
    f = f.cut(box(-17, 17, yc - 17, yc + 17, -L_SHANK + 11.5, -L_SHANK + 20))   # 球顶削平，给杆身让位
    f = f.cut(cyl_x(2.2, -20, 20, yc, -L_SHANK + 6))
    return f


def rod_assembly(C, P):
    """连杆：两端杆端关节轴承 + 中间 M5 牙杆/碳管（腿坐标 x,z）"""
    yc = (Y_ROD[0] + Y_ROD[1]) / 2
    parts = []
    for (x, z) in (C, P):
        e = cyl_y(8.5, Y_ROD[0], Y_ROD[1], x, z).cut(cyl_y(2.5, Y_ROD[0] - 1, Y_ROD[1] + 1, x, z))
        parts.append(e)
    dx, dz = P[0] - C[0], P[1] - C[1]
    L = math.hypot(dx, dz)
    ux, uz = dx / L, dz / L
    a = (C[0] + ux * 8, yc, C[1] + uz * 8)
    rod = cq.Solid.makeCylinder(3.5, L - 16, V(*a), V(ux, 0, uz))
    parts.append(rod)
    return parts, L


# ---------------------------------------------------------------- 机身外壳
def side_panel_L():
    y0, y1 = BODY_W / 2 - 2, BODY_W / 2 + 1
    x0, x1 = -(HIP_X - BLOCK_DEPTH - 1), HIP_X - BLOCK_DEPTH - 1
    p = box(x0 + 0.5, x1 - 0.5, y0, y1, BODY_ZMIN + 2, BODY_ZMAX - 2)
    for k in range(4):
        z = 18 - k * 13
        p = p.cut(stadium_y((x0 + 40, z), (x1 - 40, z), 2.5, y0 - 1, y1 + 1))
    # 型材 T 型螺母固定孔 M5
    for x in (x0 + 15, 0, x1 - 15):
        for z in (40.0, -35.0):
            p = p.cut(cyl_y(2.75, y0 - 1, y1 + 1, x, z))
    return p


def top_cover():
    x0, x1 = -(HIP_X - BLOCK_DEPTH - 1), HIP_X - BLOCK_DEPTH - 1
    w = BODY_W / 2
    c = box(x0 + 0.5, x1 - 0.5, -w, w, BODY_ZMAX, BODY_ZMAX + 4)
    c = chamfer_edges(c, "|Z", 4.0)
    # Jetson 散热进风格栅
    for k in range(6):
        y = -25 + k * 10
        c = c.cut(box(-40, 40, y - 2, y + 2, BODY_ZMAX - 1, BODY_ZMAX + 5))
    for x in (x0 + 15, x1 - 15):
        for y in (58.0, -58.0):
            c = c.cut(cyl_z(2.75, BODY_ZMAX - 1, BODY_ZMAX + 5, x, y))
    return c


def bottom_plate():
    x0, x1 = -(HIP_X - BLOCK_DEPTH - 1), HIP_X - BLOCK_DEPTH - 1
    w = BODY_W / 2
    b = box(x0 + 0.5, x1 - 0.5, -w, w, BODY_ZMIN - 4, BODY_ZMIN)
    b = chamfer_edges(b, "|Z", 4.0)
    for x in (x0 + 15, x1 - 15):
        for y in (58.0, -58.0):
            b = b.cut(cyl_z(2.75, BODY_ZMIN - 5, BODY_ZMIN + 1, x, y))
    return b


def handle():
    """顶部提手：机身中段两立柱 + 横梁，便于搬运也是视觉重心"""
    z0 = BODY_ZMAX + 4
    pts = [(-72, z0), (72, z0), (66, z0 + 32), (-66, z0 + 32)]
    h = xz_prism(pts, -11, 11)
    h = h.cut(xz_prism([(-52, z0 - 1), (52, z0 - 1), (50, z0 + 19), (-50, z0 + 19)], -12, 12))
    h = chamfer_edges(h, "|Y", 3.0)
    h = chamfer_edges(h, "|X", 2.0)
    for x in (-62.0, 62.0):
        h = h.cut(cyl_z(2.75, z0 - 1, z0 + 12, x, 0))
    return h


def head_shell():
    """传感器头：楔形，前脸装深度相机，顶部麦克风阵列开孔"""
    z0 = BODY_ZMAX + 6
    xb, xf = HIP_X - 40, HIP_X + 58
    zt = z0 + 44
    out = [(xb, z0), (xf - 6, z0), (xf, z0 + 8), (xf, z0 + 32), (xf - 16, zt), (xb, zt)]
    h = xz_prism(out, -50, 50)
    h = chamfer_edges(h, "|X", 6.0)
    inner = [(xb + 3, z0 + 3), (xf - 8, z0 + 3), (xf - 3, z0 + 9), (xf - 3, z0 + 31),
             (xf - 18, zt - 3), (xb + 3, zt - 3)]
    inner[0] = (xb + 3, z0 - 1)
    inner[1] = (xf - 8, z0 - 1)
    h = h.cut(xz_prism(inner, -47, 47))   # 底面敞开，扣在 head_base 上
    # 相机窗
    h = h.cut(box(xf - 5, xf + 1, -46, 46, z0 + 12, z0 + 27))
    # 侧面散热槽
    for k in range(3):
        z = z0 + 14 + k * 8
        for s in (+1, -1):
            h = h.cut(box(xb + 20, xb + 55, s * 50 - 4 if s > 0 else -51, 51 if s > 0 else -46, z - 1.5, z + 1.5))
    # 顶部麦克风阵列孔（4 个）
    for (dx, dy) in ((0, 30), (0, -30), (30, 0), (-30, 0)):
        h = h.cut(cyl_z(1.5, zt - 4, zt + 1, xb + 50 + dx * 0.6, dy))
    return h


def head_base():
    """头部底座：螺接在前髋座顶面，外壳扣在上面"""
    z0 = BODY_ZMAX
    xb, xf = HIP_X - 40, HIP_X + 58
    n = box(xb, xf - 6, -50, 50, z0, z0 + 6)
    n = chamfer_edges(n, "|Z", 5.0)
    n = n.fuse(box(xb + 3.3, xf - 8.3, -46.7, 46.7, z0 + 6, z0 + 10).cut(
        box(xb + 6, xf - 11, -44, 44, z0 + 5, z0 + 11)))                      # 定位止口
    n = n.cut(cyl_z(9, z0 - 1, z0 + 7, HIP_X - 21, 0))
    n = n.cut(box(xb + 45, xf - 20, -30, 30, z0 - 1, z0 + 7))                  # 减重/走线
    for (x, y) in ((xb + 10, 30), (xb + 10, -30), (HIP_X - 12, 30), (HIP_X - 12, -30)):
        n = n.cut(cyl_z(1.7, z0 - 1, z0 + 7, x, y))
    return n


def estop():
    x, z0 = -(HIP_X - 28), BODY_ZMAX
    base = cyl_z(12, z0, z0 + 8, x, 0)
    cap = cyl_z(15, z0 + 8, z0 + 18, x, 0)
    cap = chamfer_edges(cap, ">Z", 3.0)
    return base, cap
