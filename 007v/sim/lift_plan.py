"""抬一条腿的姿势规划（静力学，用 MuJoCo 模型里的真实重量分布）。

思路：每条腿只有一个髋关节，抬腿前没法像四足那样“挪重心”，只能靠另外三条腿换角度：
轮子着地点前后挪、车身跟着倾斜，让重心落到三个着地轮围成的三角形里面，离边越远越稳。
这里把三条支撑腿和抬起那条腿的角度网格搜一遍（每 5°），挑“重心离三角形边最远”的组合。
"""
import math

import mujoco
import numpy as np

LEGS = ["FL", "FR", "RL", "RR"]
FX = dict(FL=1, FR=1, RL=-1, RR=-1)
G = 9.81


def rot(v, phi):
    """绕 +Y 转 phi（弧度，可以是数组）。v: (..., 3)"""
    c, s = np.cos(phi), np.sin(phi)
    x, y, z = v[..., 0], v[..., 1], v[..., 2]
    return np.stack([x * c + z * s, np.broadcast_to(y, np.shape(x * c)), -x * s + z * c], axis=-1)


class Geo:
    def __init__(self, m, R=0.076):
        self.R = R
        d = mujoco.MjData(m)
        d.qpos[:] = 0
        d.qpos[3] = 1
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        base = m.body("base").id
        self.M = float(m.body_subtreemass[base])
        fm, fmc = self.M, self.M * d.subtree_com[base].copy()
        self.leg = {}
        for k in LEGS:
            b = m.body(k).id
            mk = float(m.body_subtreemass[b])
            self.leg[k] = dict(m=mk, com=d.subtree_com[b] - d.xpos[b], hip=d.xpos[b].copy(), wheel=m.body(k + "_wheel").pos.copy())
            fm -= mk
            fmc -= mk * d.subtree_com[b]
        self.fixed_m, self.fixed_c = fm, fmc / fm
        self.skids = np.array([[x, y, -0.006] for x in (-0.15, 0.15) for y in (-0.09, 0.09)])

    # ------------------------------------------------------------ 给定四条腿的角度（度，数组）和哪条腿抬起，算车身姿态和稳定裕度
    def evaluate(self, th, lifted):
        sup = [k for k in LEGS if k != lifted]
        N = np.broadcast(*[np.asarray(th[k], float) for k in LEGS]).shape
        phi = {k: np.radians(np.broadcast_to(np.asarray(th[k], float), N)) * FX[k] for k in LEGS}
        C = {k: self.leg[k]["hip"] + rot(self.leg[k]["wheel"], phi[k]) for k in LEGS}          # 轮心（机身坐标）
        com = self.fixed_m * self.fixed_c + sum(self.leg[k]["m"] * (self.leg[k]["hip"] + rot(self.leg[k]["com"], phi[k])) for k in LEGS)
        com = com / self.M
        y = np.array([0.0, 1.0, 0.0])

        def contact(c, n):
            dw = -(n - (n @ y)[..., None] * y)
            dw /= np.linalg.norm(dw, axis=-1, keepdims=True)
            return c + self.R * dw

        P = [C[k] for k in sup]
        n = np.cross(P[1] - P[0], P[2] - P[0])
        for _ in range(4):
            n = n * np.sign(n[..., 2:3])
            n /= np.linalg.norm(n, axis=-1, keepdims=True)
            P = [contact(C[k], n) for k in sup]
            n = np.cross(P[1] - P[0], P[2] - P[0])
        n = n * np.sign(n[..., 2:3])
        n /= np.linalg.norm(n, axis=-1, keepdims=True)
        d0 = np.sum(n * P[0], axis=-1)
        h_lift = np.sum(n * contact(C[lifted], n), axis=-1) - d0                       # 抬起的轮子离地
        tilt = np.degrees(np.arccos(np.clip(n[..., 2], -1, 1)))
        skid = np.min(np.einsum("...j,kj->...k", n, self.skids), -1) - d0                 # 滑撬最低点离地
        # 重心沿重力投到地面，在地面上建二维坐标
        cp = com - (np.sum(n * com, -1) - d0)[..., None] * n
        e1 = np.array([1.0, 0, 0]) - n[..., 0:1] * n
        e1 /= np.linalg.norm(e1, axis=-1, keepdims=True)
        e2 = np.cross(n, e1)
        p2 = lambda p: np.stack([np.sum(p * e1, -1), np.sum(p * e2, -1)], -1)  # noqa: E731
        a, b, c, q = p2(P[0]), p2(P[1]), p2(P[2]), p2(cp)
        area = (b[..., 0] - a[..., 0]) * (c[..., 1] - a[..., 1]) - (b[..., 1] - a[..., 1]) * (c[..., 0] - a[..., 0])
        flip = area < 0
        b2 = np.where(flip[..., None], c, b)
        c2 = np.where(flip[..., None], b, c)
        dist = []
        for u, v in ((a, b2), (b2, c2), (c2, a)):
            e = v - u
            dist.append((e[..., 0] * (q[..., 1] - u[..., 1]) - e[..., 1] * (q[..., 0] - u[..., 0])) / np.linalg.norm(e, axis=-1))
        margin = np.min(np.stack(dist, -1), -1)
        # 三个轮子各分多少重量（重心的重心坐标），和髋关节扭矩
        T = np.stack([b - a, c - a], -1)
        w12 = np.linalg.solve(T, (q - a)[..., None])[..., 0]
        w = np.stack([1 - w12[..., 0] - w12[..., 1], w12[..., 0], w12[..., 1]], -1)
        tau = np.stack([np.abs(np.sum(np.cross(P[i] - self.leg[k]["hip"], n) * y, -1)) * w[..., i] * self.M * G for i, k in enumerate(sup)], -1)
        return dict(margin=margin, tilt=tilt, h_lift=h_lift, skid=skid, hip_tau=tau.max(-1), load=w * self.M, n=n, d0=d0)

    def touch_angle(self, th_sup, lifted):
        """三条支撑腿角度给定时，抬起那条腿刚好着地的角度（-30°…90° 里二分）。"""
        lo, hi = -30.0, 90.0
        for _ in range(40):
            mid = (lo + hi) / 2
            h = self.evaluate({**th_sup, lifted: mid}, lifted)["h_lift"]
            lo, hi = (mid, hi) if h > 0 else (lo, mid)
        return (lo + hi) / 2

    def plan(self, lifted, h_min, tilt_max=12.0, skid_min=0.04, step=5.0, enough=0.05, stance=55.0):
        """裕度够 enough（m）以后，优先车身歪得少、关节动得少。"""
        sup = [k for k in LEGS if k != lifted]
        g = np.arange(10.0, 100.0 + 1e-6, step)
        gl = np.arange(-30.0, 45.0 + 1e-6, step)
        A, B, C, L = np.meshgrid(g, g, g, gl, indexing="ij")
        th = {sup[0]: A.ravel(), sup[1]: B.ravel(), sup[2]: C.ravel(), lifted: L.ravel()}
        r = self.evaluate(th, lifted)
        ok = (r["h_lift"] >= h_min) & (r["tilt"] <= tilt_max) & (r["skid"] >= skid_min)
        if not ok.any():
            return None
        move = sum(np.abs(th[k] - stance) for k in sup)
        score = np.where(ok, np.minimum(r["margin"], enough) - 0.002 * r["tilt"] - 0.0001 * move
                         - 0.01 * (r["h_lift"] - h_min), -1e9)                      # 抬够就行，不多抬
        i = int(np.argmax(score))
        return dict(lifted=lifted, support={k: float(th[k][i]) for k in sup}, theta_lift=float(th[lifted][i]),
                    margin=float(r["margin"][i]), tilt=float(r["tilt"][i]), h_lift=float(r["h_lift"][i]),
                    skid=float(r["skid"][i]), hip_tau=float(r["hip_tau"][i]))

    def straight(self, lifted, stance=55.0, h_min=0.03):
        """其他腿不动、直接抬：抬到离地 h_min 时的裕度。"""
        th_sup = {k: stance for k in LEGS if k != lifted}
        lo, hi = -30.0, stance
        for _ in range(40):
            mid = (lo + hi) / 2
            h = self.evaluate({**th_sup, lifted: mid}, lifted)["h_lift"]
            lo, hi = (lo, mid) if h > h_min else (mid, hi)
        r = self.evaluate({**th_sup, lifted: lo}, lifted)
        return float(r["margin"]), float(lo)
