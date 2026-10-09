"""
膝关节全行程扫描（v0.6）：只看一条腿自己（大腿上的零件 vs 小腿上的零件），膝关节从 -12° 转到 150°，
每 5° 用实体求交检查一次（体积 > 2mm³ 算碰撞），找出小腿造型改了以后还能安全转到哪里。
皮带、膝轴、大带轮和皮带的啮合是设计接触，不算。
用法：LEG_STYLE=B python3 check_knee.py
"""
import cadquery as cq
import parts as P
import leg as G
import armor as A
from config import *

V = cq.Vector
hb, hf = P.motor_y(-MOTOR["H"])
kb, kf = P.motor_y(Y_KFE[0])
idler_r, idler_ax, idler_sl = G.idler()
THIGH = [("大腿内板", G.thigh_inner()), ("大腿外板", G.thigh_outer()), ("KFE电机", kb), ("KFE法兰", kf),
         ("带轮转接盘", G.pulley_adapter()), ("小带轮", G.pulley_small()), ("张紧轮", idler_r), ("张紧轮套筒", idler_sl),
         ("大腿装甲", A.thigh_cover()), ("髋端盖", A.hip_cap()), ("膝盖", A.knee_cap()), ("膝电机前安装环", G.kfe_front_ring()),
         ("大腿隔板", G.thigh_partition())] + [(f"立柱{i}", s) for i, s in enumerate(G.standoffs())]
k_seat, k_plug = P.motor_connector_y(Y_KFE[0], G.KFE_CONN_ANG)
THIGH += [("KFE插头座", k_seat), ("预留-KFE插头", k_plug)]
SHANK = [("小腿", G.to_knee(G.shank())), ("大带轮", G.to_knee(G.pulley_big())), ("小腿护甲", G.to_knee(G.shin_armor())),
         ("小腿内嵌板", G.to_knee(G.shin_insert())), ("脚踝护圈", G.to_knee(G.ankle_collar())), ("足端", G.to_knee(G.foot_core())), ("足端防滑套", G.to_knee(G.foot_tread()))]
tb = [(n, s, s.BoundingBox()) for n, s in THIGH]


def hits(beta):
    out = []
    for ns, s in SHANK:
        r = s.rotate(V(0, 0, -L_THIGH), V(0, 1, -L_THIGH), -beta)
        b = r.BoundingBox()
        for nt, t, a in tb:
            if a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin or b.ymax < a.ymin or a.zmax < b.zmin or b.zmax < a.zmin:
                continue
            v = r.intersect(t).Volume()
            if v > 2.0:
                out.append((round(v), ns, nt))
    return out


ok_lo = ok_hi = None
bad = []
for beta in range(-10, 151, 5):
    h = hits(beta)
    if h:
        bad.append((beta, h[:3]))
    else:
        ok_lo = beta if ok_lo is None else ok_lo
        ok_hi = beta
print(f"小腿造型 {LEG_STYLE}：膝关节 β 安全的角度 {ok_lo}°…{ok_hi}°（设计范围 {KNEE_BETA_RANGE[0]:.0f}°…{KNEE_BETA_RANGE[1]:.0f}°）")
for beta, h in bad:
    print(f"  β={beta}° 碰撞：" + "；".join(f"{a}↔{b} {v}mm³" for v, a, b in h))
