"""007A 机身 v10：原版方正机身（model.py v8），表面加层次——凹进去的、凸出来的、线条（参考用户给的工业机器狗图）。

每一样都有用处（用户原则：没作用的不设计）：
  髋关节轮拱：机身侧面在髋的位置挖进去一个弧形窝、窝里深色 —— 髋外摆、前后摆转动的让位
  侧装甲板：深色，凸出 3，盖住机身里的电池 / 电路板（四颗螺丝可拆），上面是“007A”
  散热格栅：装甲板前下角、尾部下方、下层底盘检修板上，一条条凹进去 —— 电机驱动、树莓派的进出风口
  斜肩：上层甲板壳顶上两条长棱切 45° × 22 —— 侧翻时先着地的是斜面（不是尖棱），棱线也是顶盖的分界
  底盘收腰：下层底盘两条底棱切 45° × 18 —— 腿往里摆、过坎时不刮机身底
  护角：上层甲板壳四个竖棱包深色橡胶护角 —— 撞墙、撞门框先碰护角
  扬声器孔：前脸两组斜槽中间各一排小孔 —— 喇叭（USB 声卡）往前出声
  麦克风阵列 + 录音灯：顶上右前角一个深色圆座、5 个拾音孔、外圈青色灯 —— 会议助手录音用，灯亮 = 正在录（大家看得见）
  电源开关：尾部接口板右边一个自锁开关（GQ16）带青色光圈 —— 开机，光圈亮 = 通电
  探照灯（一个）：前防撞杠左头，圆筒形强光灯 —— 管廊、楼道里照路，给相机补光
  红外热像仪：前防撞杠右头（和探照灯左右对称），方形镜头座、锗镜片 —— 看电缆接头哪里发热（管道巡检最主要的检查）
  气体检测：尾部右侧（尾灯下面）一块平的进气格栅 + 状态灯，不凸出来 —— 测氧气、一氧化碳、硫化氢、甲烷，判断管道里人能不能进
  底部绑带卡口 ×4：机身底下四个角，凹进去的卡口带横销（外面看不到）—— 以后绑带子用
  深度相机：前防撞杠正中，往下斜的相机罩（两个红外镜头 + 投射器 + 彩色镜头）—— 看台阶、看前面地面（上楼梯的眼睛）
  雷达护圈：雷达底下一圈八角形矮护圈，上沿低于扫描窗 —— 不挡扫描，侧面撞到先碰护圈
  前防撞杠：车头下面一根深色杠，台阶测距窗在杠里 —— 保护车头和测距窗
  尾部接口板：凹进去的深色板，急停、充电口在上面
用法：python 007a/body_v10.py → 007a/out/parts
"""
import json
import math
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
        elif n == "雷达座":
            continue                                              # 换成八角护圈
        elif n.startswith("测距窗"):
            continue                                              # 测距窗挪进防撞杠
        else:
            P.append((n, s, c))
    up, tub, waist = shells["上层甲板壳"][0], shells["下层底盘"][0], shells["黑腰"][0]
    for sy in (1, -1):
        ys = sy * (W2 + 2)                                        # 上层甲板壳侧面
        # ---- 髋窝：髋盖后面一圈浅窝
        for fx in (1, -1):                                         # 浅窝：半径 66、深 5，比髋盖大一圈（髋转动的让位），窝底中灰
            y0, y1 = sorted((sy * (W2 - 5), sy * (W2 + 20)))       # R62 深 5（大腿内层只包皮带，外摆 ±15° 也不碰，eng_leg.py 算的）
            pit = cyl_y(fx * HX, HZ, 128, y0, y1)                  # R64：髋机构穿过侧壁的口（x ±52、z ±63，eng_body.py 量的）整个藏在窝里
            up, tub, waist = up.cut(pit), tub.cut(pit), waist.cut(pit)
            liner = cyl_y(fx * HX, HZ, 128, *sorted((sy * (W2 - 6), sy * (W2 - 5)))).cut(
                cq.Workplane("XY").box(108, 2 * W2 + 20, 130).edges("|Y").fillet(20).val().translate(V(fx * HX, 0, HZ))).intersect(box(-L2, L2, -W2, W2, Z0 + 1, TOP - 1))
            P.append((f"髋窝底{fx}{sy}", liner, "skel"))
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
    # ---- 雷达护圈：八角形矮护圈（和机身 45° 切角一样），上沿低于雷达扫描窗 2 —— 不挡扫描，侧面撞到先碰护圈
    lx, ly = L2 - 46, -(W2 - 50)
    collar = M.plan_chamfer(box(lx - 30, lx + 30, ly - 30, ly + 30, TOP, TOP + 16), 14.0)
    collar = collar.cut(cq.Solid.makeCylinder(22, 20, V(lx, ly, TOP + 4)))
    collar = collar.cut(M.plan_chamfer(box(lx - 26, lx + 26, ly - 26, ly + 26, TOP + 14, TOP + 17), 12.0))   # 上沿内侧一圈台阶
    P.append(("雷达护圈", soft(collar, 1.0), "skel"))
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
    # ---- 扬声器孔：前脸两组斜槽中间（左右立体声）
    hw = 150.0 / 2                                                  # 头宽一半（model.py：sw + 26）
    for sy in (1, -1):
        for i in range(5):
            for j in (-1, 0, 1):
                y, z = sy * (hw + 18 + i * 5.5), ZC + 2 + j * 5.5
                up = up.cut(cyl_x(y, z, 3.0, L2 + 1, L2 + 8))
        y0, y1 = sorted((sy * (hw + 15), sy * (hw + 43)))
        P.append((f"扬声器孔底{sy}", box(L2 + 1, L2 + 2, y0, y1, ZC - 6, ZC + 10), "skel_dk"))
    # ---- 麦克风阵列 + 录音灯：顶上右前角（雷达在左前角，对称）
    mx, my = L2 - 46, W2 - 50
    up = up.cut(cq.Solid.makeCylinder(23, 4, V(mx, my, TOP - 1.5)))
    P.append(("麦克风座", cq.Solid.makeCylinder(20, 1.5, V(mx, my, TOP - 1.5)), "skel_dk"))
    P.append(("录音灯", ring_y(0, 0, 46, 41, 0, 1).rotate(V(0, 0, 0), V(1, 0, 0), 90).translate(V(mx, my, TOP - 0.6)), "cyan"))
    for a in range(4):
        ang = a * 3.14159 / 2 + 0.785
        P.append((f"拾音孔{a}", cq.Solid.makeCylinder(1.6, 0.4, V(mx + 11 * math.cos(ang), my + 11 * math.sin(ang), TOP)), "rubber"))
    P.append(("拾音孔中", cq.Solid.makeCylinder(1.6, 0.4, V(mx, my, TOP)), "rubber"))
    # ---- 电源开关（GQ16 自锁，带光圈）：尾部接口板右边
    P.append(("电源开关", cyl_x(44, ZC, 19, -L2 - 8, -L2 - 2), "metal"))
    P.append(("电源光圈", cyl_x(44, ZC, 22, -L2 - 3, -L2 - 1).cut(cyl_x(44, ZC, 18, -L2 - 4, -L2)), "cyan"))
    # ---- 深度相机：前防撞杠正中，正面往下斜 20°（看台阶）
    cam = prism_xz([(L2 + 4, Z0 + 18), (L2 + 22, Z0 + 18), (L2 + 26, Z0 + 29), (L2 + 26, Z0 + 38), (L2 + 16, Z0 + 48), (L2 + 4, Z0 + 48)], -48, 48)
    P.append(("深度相机罩", soft(cam, 1.0), "skel"))
    P.append(("深度相机窗", prism_xz([(L2 + 25.5, Z0 + 21), (L2 + 26.6, Z0 + 21), (L2 + 26.6, Z0 + 37), (L2 + 25.5, Z0 + 37)], -42, 42), "glass"))
    for y, d in ((-32, 10), (32, 10), (0, 7), (14, 7)):             # 左右红外、投射器、彩色
        P.append((f"相机镜头{y}", cyl_x(y, Z0 + 29, d, L2 + 26.4, L2 + 27.4), "metal"))
    # ---- 探照灯（一个）：前防撞杠左头（+y，和右上角的雷达斜对着），圆筒灯 Φ34，凸出 18
    ly_, lz_ = 101.0, Z0 + 32
    P.append(("探照灯筒", soft(cyl_x(ly_, lz_, 34, L2 - 2, L2 + 16), 1.0), "skel"))
    P.append(("探照灯圈", cyl_x(ly_, lz_, 36, L2 + 14, L2 + 18).cut(cyl_x(ly_, lz_, 27, L2 + 13, L2 + 19)), "metal"))
    P.append(("探照灯", cyl_x(ly_, lz_, 25, L2 + 14, L2 + 15), "lamp"))
    for k in range(4):                                              # 灯筒上的散热环
        x = L2 + 1 + k * 3
        P.append((f"灯散热环{k}", cyl_x(ly_, lz_, 37, x, x + 1.2), "skel_dk"))
    # ---- 红外热像仪：前防撞杠右头（-y），和探照灯对称
    ty, tz = -101.0, Z0 + 32
    pod = prism_xz([(L2 - 2, tz - 16), (L2 + 12, tz - 16), (L2 + 16, tz - 12), (L2 + 16, tz + 12), (L2 + 12, tz + 16), (L2 - 2, tz + 16)], ty - 17, ty + 17)
    pod = pod.cut(cyl_x(ty, tz, 24, L2 + 13, L2 + 18))
    P.append(("热像仪座", soft(pod, 1.0), "skel"))
    P.append(("热像仪镜片", cyl_x(ty, tz, 24, L2 + 13, L2 + 14), "glass"))
    P.append(("热像仪镜圈", cyl_x(ty, tz, 27, L2 + 15.5, L2 + 17).cut(cyl_x(ty, tz, 22, L2 + 15, L2 + 18)), "metal"))
    # ---- 气体检测：尾部右侧（-y，尾灯下面），平的进气格栅，不凸出来；旁边一个状态灯
    gx = -L2 - 4
    g0, g1, gz0, gz1 = -112.0, -76.0, ZC - 20, ZC + 8
    for k in range(6):
        y = g0 + 2 + k * 6
        up = up.cut(box(gx - 1, gx + 3, y, y + 3, gz0, gz1))
    P.append(("气体进气槽底", box(gx + 2.5, gx + 3, g0, g1, gz0, gz1), "skel_dk"))
    P.append(("气体状态灯", cyl_x(-94, gz0 - 7, 5, gx - 0.6, gx + 0.4), "cyan"))
    # ---- 底部绑带卡口 ×4：底下四个角，凹 10 的槽 + 一根横销（带子穿过去）
    for x in (-150.0, 150.0):
        for y in (-70.0, 70.0):
            tub = tub.cut(box(x - 18, x + 18, y - 6, y + 6, Z0 - 1, Z0 + 10))
            P.append((f"卡口底{x:.0f}{y:.0f}", box(x - 18, x + 18, y - 6, y + 6, Z0 + 9, Z0 + 10), "skel_dk"))
            P.append((f"卡口横销{x:.0f}{y:.0f}", cq.Solid.makeCylinder(2.5, 12, V(x, y - 6, Z0 + 4), V(0, 1, 0)), "metal"))
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
    print(f"{len(man)} 个零件（v16：+ 红外热像仪、气体检测（尾部格栅））")
