"""腿的真实外形（侧面：髋盘 + 大腿 + 膝圆头 + 小腿）离台阶有多远 —— 按 run_sim.py 规划出来的静步，一个姿势一个姿势地查。

run_sim.py 里规划用的是“膝 = 圆、大腿 / 小腿 = 一根有粗细的线”，这里换成 model.py 里真实的 2D 外形（eng_leg.py 用的同一套），
看离台阶最近的到底是哪一块：大腿下沿 / 膝圆头 / 小腿靠膝那段 / 小腿下段。
用法：python 007a/sim/leg_clear.py [up|back|down|pub] [--png]
"""
import math
import os
import sys

import numpy as np
from shapely.geometry import LineString, Point, Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import run_sim as RS  # noqa: E402
import model as M  # noqa: E402
import eng_leg as E  # noqa: E402

M.RK, M.HEX_A, M.HUB_CUT = E.RK_NEW, E.HEX_A_NEW, E.HEX_A_NEW + 3.0
STAND_FOOT = np.array([0.0, -M.HIP_Z])
K0, F0 = RS.K0 * 1000, RS.F0 * 1000                                # 站立时的膝、脚球心（mm）                              # model.py 的脚（脚垫底下那点），站立时在髋正下方
SKIP_FOOT = 0.030                                                   # 脚球心 30 mm 以内不算（脚本来就踩在踏面上）


def plan_stairs(h=0.175, t=0.26, n=4, mode="up"):
    T = RS.stairs(h, t, n)
    import stair_calc as SC
    H = SC.climb(RS.SC_L1, RS.SC_LS, h * 1000, t * 1000)[2] / 1000
    x_end = 0.45 + (n - 1) * t + 0.45
    if mode == "up":
        xs, xg, dr, z0 = 0.0, x_end, 1, 0.0
    else:
        T = T.mirror()
        xs, xg, dr, z0 = (0.0, -x_end, -1, 0.0) if mode == "back" else (-x_end, 0.0, 1, n * h)
    s = RS.Sim(terrain=T.xml())
    p0 = np.array([xs, 0, z0 + H])
    s.set_pose(p0, np.eye(3), RS.stand_feet_body(H))
    feet0 = [p0 + f for f in RS.stand_feet_body(H)]
    for f in feet0:
        f[2] = T.gnd(f[0]) + RS.FOOT_R
    m, ph, plan, order = RS.crawl(s, T, xg, None, H=H, D=t / 2, direction=dr, dry=True)
    return T, p0, feet0, plan


def poses(p0, feet0, plan):
    """展开成一串 (机身位置, 俯仰, 四只脚球心, 在跨的腿)。"""
    feet = [f.copy() for f in feet0]
    p, th = p0.copy(), 0.0
    for i, p1, th1, new, z_top, *_ in plan:
        for u in (0.0, 0.5, 1.0):                                    # 挪机身
            yield p + (p1 - p) * u, th + (th1 - th) * u, feet, None
        p, th = p1, th1
        old = feet[i]
        for q in (np.array([old[0], old[1], (old[2] + z_top) / 2]), np.array([old[0], old[1], z_top]),
                  np.array([(old[0] + new[0]) / 2, old[1], z_top]), np.array([new[0], old[1], z_top]),
                  np.array([new[0], old[1], (new[2] + z_top) / 2])):
            f2 = list(feet)
            f2[i] = q
            yield p, th, f2, i
        feet = list(feet)
        feet[i] = new


def leg_shapes(hip, centre):
    """髋、脚球心（侧面 x-z，m）→ 大腿 / 小腿 多边形（m，世界坐标）。"""
    rel = (centre - hip) * 1000
    # 用 run_sim 的连杆（L1 = 200、膝 → 球心 178）解膝，再把 model.py 的小腿（膝 → 脚底 190）按同样的转角摆上去
    L1, LC = np.linalg.norm(K0), np.linalg.norm(F0 - K0)
    d = float(np.clip(np.linalg.norm(rel), abs(L1 - LC) + 0.1, L1 + LC - 0.1))
    a = math.acos(np.clip((L1 ** 2 + d ** 2 - LC ** 2) / (2 * L1 * d), -1, 1))
    th = math.atan2(rel[1], rel[0])
    knee = None
    for sg in (1, -1):
        K = L1 * np.array([math.cos(th + sg * a), math.sin(th + sg * a)])
        if knee is None or (rel[0] * K[1] - rel[1] * K[0]) * RS.CROSS0 > 0:
            knee = K
    rot = math.atan2(*(rel - knee)[::-1]) - math.atan2(*(F0 - K0)[::-1])
    v = STAND_FOOT - K0
    c, s_ = math.cos(rot), math.sin(rot)
    foot_m = knee + np.array([v[0] * c - v[1] * s_, v[0] * s_ + v[1] * c])
    thigh, shank, kn, ft = M.leg_2d(tuple(foot_m))
    h = np.asarray(hip)
    return (Polygon([(h[0] + x / 1000, h[1] + z / 1000) for x, z in thigh]),
            Polygon([(h[0] + x / 1000, h[1] + z / 1000) for x, z in shank]),
            h + np.array(kn) / 1000)


def ground_poly(T):
    pts = [(-10.0, -1.0)]
    for a, b, z in T.segs:
        pts += [(max(a, -10), z), (min(b, 10), z)]
    pts += [(10.0, -1.0)]
    return Polygon(pts)


def where(pt, hip, knee, foot, part):
    """最近点在腿上的位置：大腿 → 髋到膝的百分比；小腿 → 膝到脚的百分比。"""
    p = np.array(pt)
    a, b = (hip, knee) if part == "大腿" else (knee, foot)
    u = float(np.dot(p - a, b - a) / np.dot(b - a, b - a))
    return u


def check(T, p0, feet0, plan):
    G = ground_poly(T)
    GB = G.boundary
    res = []
    for p, th, feet, sw in poses(p0, feet0, plan):
        R = RS.rpy_mat(0, -th)
        for k, l in enumerate(RS.LEGS):
            fx, sy = RS.SIDE[l]
            hp3 = p + R @ np.array([fx * RS.HX, sy * RS.Y_LEG, 0])
            hip, ctr = hp3[[0, 2]], feet[k][[0, 2]]
            thigh, shank, knee = leg_shapes(hip, ctr)
            ctr_pt = Point(*ctr).buffer(SKIP_FOOT)
            for part, poly in (("大腿", thigh), ("小腿", shank.difference(ctr_pt))):
                if poly.is_empty:
                    continue
                xy = np.array(poly.exterior.segmentize(0.002).coords) if poly.geom_type == "Polygon" else                     np.vstack([np.array(g.exterior.segmentize(0.002).coords) for g in poly.geoms])
                ds = np.array([GB.distance(Point(*q)) * (-1 if G.contains(Point(*q)) else 1) for q in xy])
                j = int(np.argmin(ds))
                dist, pt = float(ds[j]), xy[j]
                res.append(dict(leg=l, swing=(sw == k), part=part, d=dist, u=where(pt, hip, knee, ctr, part), pt=pt,
                                pose=(hip, ctr, thigh, shank)))
    return res


def report(res, title):
    print(f"== {title}")
    for front in (True, False):
        for part in ("大腿", "小腿"):
            rs = [r for r in res if r["part"] == part and (r["leg"][0] == "F") == front]
            r = min(rs, key=lambda r: r["d"])
            near = sorted(rs, key=lambda r: r["d"])[:20]
            us = [round(x["u"], 2) for x in near]
            print(f"  {'前腿' if front else '后腿'} {part}：最近 {r['d'] * 1000:+.1f} mm（{'跨' if r['swing'] else '撑'}，位置 {r['u']:.2f}），最近 20 个的位置 {min(us)}–{max(us)}")
    return min(r["d"] for r in res)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "up"
    args = dict(up=(0.175, 0.26, 4, "up"), back=(0.175, 0.26, 4, "back"), down=(0.175, 0.26, 4, "down"), pub=(0.15, 0.30, 4, "up"))[mode]
    T, p0, feet0, plan = plan_stairs(*args)
    report(check(T, p0, feet0, plan), mode)
