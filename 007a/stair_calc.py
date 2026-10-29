"""007A 上楼梯腿长计算（侧面 2D）：大腿 L1、小腿 LS 取多少，能稳稳上标准楼梯。

楼梯：住宅最陡 175 高 × 260 宽（GB 50096 上限）；公共建筑常见 150 × 300。
机器人：前后髋距 320，机身底在髋下 72、顶在髋上 68，前面头伸出到 274。
每个 (L1, LS) 组合，机身俯仰 θ 取 0 ~ 楼梯坡度，沿楼梯走一个踏步周期（每 10 mm 一个位置），每个位置都要找得到机身高度满足：
  1 站立：每条腿的脚能踩在踏面中间（离踏步边 ≥ 50、离踢面 ≥ 40），膝角在 [KMIN, KMAX]，髋→脚连线离竖直 ≤ 30°
  2 抬脚：每条腿能把脚抬到前面那个踏步边上方 50（过台阶不踢到）
  3 机身：机身底离每个踏步边 ≥ 25（不托底）
  4 膝：膝朝后，离楼梯面 ≥ 15（不磕膝）
输出：每个组合能不能上；能上的里面，抬脚余量（脚还能再抬多高）最大的那个。
用法：python 007a/stair_calc.py
"""
import math

HX2 = 320.0                         # 前后髋距
BODY_X0, BODY_X1, BODY_ZB = -80.0, HX2 + 114.0, -72.0   # 机身底（髋坐标：后髋 = 0，前髋 = 320）：从后髋后 80 到前髋前 114（含头）
TH_MAX = 20                          # 机身最多仰 20°（背上 5 kg 货不能滑下来）
KMIN, KMAX = 40.0, 160.0             # 膝角（大腿和小腿的夹角）范围
FOOT_EDGE, FOOT_RISER = 50.0, 40.0
LIFT, BELLY, KNEE_CLR = 50.0, 25.0, 15.0


def reach(L1, LS, k):
    return math.sqrt(L1 ** 2 + LS ** 2 - 2 * L1 * LS * math.cos(math.radians(k)))


def ground(x, h, t):
    """楼梯面高度（踏步 k：x 在 [k t, (k+1) t) 上高 k h）。"""
    return math.floor(x / t) * h


def knee_pt(L1, LS, hip, foot):
    """膝朝后（-x）的那个解。"""
    dx, dz = foot[0] - hip[0], foot[1] - hip[1]
    d = math.hypot(dx, dz)
    a = math.acos(max(-1, min(1, (L1 ** 2 + d ** 2 - LS ** 2) / (2 * L1 * d))))
    ang = math.atan2(dz, dx) - a
    k1 = (hip[0] + L1 * math.cos(ang), hip[1] + L1 * math.sin(ang))
    ang2 = math.atan2(dz, dx) + a
    k2 = (hip[0] + L1 * math.cos(ang2), hip[1] + L1 * math.sin(ang2))
    return k1 if k1[0] < k2[0] else k2


DUTY = 0.75                          # 每条腿 3/4 的时间踩着地，1/4 的时间抬起来跨（慢速爬楼）
STEP = 10


def leg_margin(L1, LS, hip_at, h, t, dmin, dmax):
    """一条腿：髋位置 hip_at(s)。找脚踩的位置 u 和起跨时机 φ，返回最好的余量（mm），不行返回 None。
    余量 = min(站立时离伸直 / 收到底还差多少, 跨台阶时脚还能再收多少)。"""
    best = None
    for u in range(int(FOOT_EDGE), int(t - FOOT_RISER) + 1, 10):
        for phi in range(0, int(t), STEP):
            k = math.floor((hip_at(phi)[0] - u) / t + 0.5)          # 起站时脚落在髋下面附近那一级
            f = (k * t + u, k * h)
            m = 1e9
            for s in range(phi, phi + int(DUTY * t) + 1, STEP):     # 站立：脚不动，机身往前走
                hp = hip_at(s)
                dx, dz = f[0] - hp[0], f[1] - hp[1]
                d = math.hypot(dx, dz)
                if dz >= 0 or abs(math.degrees(math.atan2(dx, -dz))) > 35:
                    m = None
                    break
                kn = knee_pt(L1, LS, hp, f)
                if kn[1] - ground(kn[0], h, t) < KNEE_CLR:
                    m = None
                    break
                m = min(m, dmax - d, d - dmin)
                if m < 0:
                    break
            if m is None or m < 0:
                continue
            hp = hip_at(phi + (1 + DUTY) / 2 * t)                   # 跨：脚从这一级到上一级，中间要过踏步边上方 LIFT
            nx = (k + 1) * t
            p = (nx + 20, (k + 1) * h + LIFT)
            d = math.hypot(p[0] - hp[0], p[1] - hp[1])
            if p[1] >= hp[1] or d > dmax:
                continue
            m = min(m, d - dmin)
            if m >= 0 and (best is None or m > best):
                best = m
    return best


def climb(L1, LS, h, t):
    """能上返回 (余量 mm, 俯仰°, 后髋离楼梯斜线的高度)；不能返回 None。机身顺着楼梯斜线平着往上走。"""
    dmin, dmax = reach(L1, LS, KMIN), reach(L1, LS, KMAX)
    alpha = math.degrees(math.atan2(h, t))
    best = None
    for th in range(0, min(int(alpha), TH_MAX) + 1, 2):
        c, s_ = math.cos(math.radians(th)), math.sin(math.radians(th))
        for z0 in range(60, int(dmax) + 1, 10):
            rear = lambda s: (s, z0 + s * h / t)
            front = lambda s: (s + HX2 * c, z0 + s * h / t + HX2 * s_)
            belly_ok = True                                         # 机身底不托底
            for s in range(0, int(t), STEP):
                z = z0 + s * h / t
                for k in range(int((s + BODY_X0 * c) // t), int((s + BODY_X1 * c) // t) + 2):
                    lam = (k * t - s) / c
                    if BODY_X0 <= lam <= BODY_X1 and z + lam * s_ + BODY_ZB / c - k * h < BELLY:
                        belly_ok = False
            if not belly_ok:
                continue
            m1 = leg_margin(L1, LS, rear, h, t, dmin, dmax)
            if m1 is None:
                continue
            m2 = leg_margin(L1, LS, front, h, t, dmin, dmax)
            if m2 is None:
                continue
            m = min(m1, m2)
            if best is None or m > best[0]:
                best = (m, th, z0)
    return best


if __name__ == "__main__":
    stairs = [("住宅 175×260", 175.0, 260.0), ("公共 150×300", 150.0, 300.0)]
    L1s, LSs = range(150, 241, 10), range(140, 241, 10)
    for name, h, t in stairs:
        print(); print(f"{name}：余量 mm（- = 上不去）。行 = 大腿，列 = 小腿")
        print("      " + "".join(f"{b:5d}" for b in LSs))
        for a in L1s:
            print(f"{a:5d} " + "".join("    -" if (r := climb(a, b, h, t)) is None else f"{r[0]:5.0f}" for b in LSs), flush=True)
