"""机械狗007v骨架：摆姿态、碰撞检查、折叠轨迹检查、尺寸和重心、导出 3D 模型。

用法：python 007v/build.py        （约 1–3 分钟）
输出：007v/out/report.txt   检查报告（改了尺寸就重新跑，看这里）
      007v/out/007v.glb     3D 模型（按刚体组分节点，给 3D 查看页用）
      007v/out/rig.json     关节位置和姿态参数（给 3D 查看页用）
      007v/out/007v_data.js 上面两个打包成一个文件，007v/viewer.html 用它

坐标：X 向前、Y 向左、Z 向上，mm；地面 z = 0。
"""
import base64
import json
import math
import os
import sys
import time

import cadquery as cq
import fcl
import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from config import *  # noqa: E402,F401,F403
import model as M  # noqa: E402
from pack import pack_viewer  # noqa: E402

V = cq.Vector
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)
R = WHEEL_D / 2
LEGS = {"FL": (1, 1, "前左"), "FR": (1, -1, "前右"), "RL": (-1, 1, "后左"), "RR": (-1, -1, "后右")}
PIVOT_Z = M.DECK + PIVOT_UP
THETA_FOLD = -(180.0 - FOLD_DEG)                                      # 摆臂角：正 = 往下；-110 = 往上往里翻 70°
THETA_GROUND = math.degrees(math.asin((SKID_H - R + HIP_Z) / ARM))   # 机身刚好坐到滑撬上、轮子还着地的角度
t0 = time.time()


def bz_ground(theta):
    """轮子着地时机身原点的离地高度。"""
    return R - HIP_Z + ARM * math.sin(math.radians(theta))


POSES = {
    "deploy": dict(name="行驶（平视）", theta=STANCE_DEG, bz=bz_ground(STANCE_DEG), pitch=HEAD_PITCH["level"]),
    "up": dict(name="行驶（抬头）", theta=STANCE_DEG, bz=bz_ground(STANCE_DEG), pitch=HEAD_PITCH["up"]),
    "down": dict(name="行驶（低头）", theta=STANCE_DEG, bz=bz_ground(STANCE_DEG), pitch=HEAD_PITCH["down"]),
    "fold": dict(name="折叠", theta=THETA_FOLD, bz=float(SKID_H), pitch=HEAD_PITCH["fold"]),
}


# ---------------------------------------------------------------- 零件（局部坐标）
class Part:
    def __init__(self, g, name, s, c):
        self.g, self.name, self.s, self.c = g, name, s, c
        vs, fs = s.tessellate(0.5, 0.5)
        self.V = np.array([[p.x, p.y, p.z] for p in vs])
        self.F = np.array(fs, dtype=np.int64)
        self.vol = s.Volume()
        c0 = s.Center()
        self.cen = np.array([c0.x, c0.y, c0.z])
        self.obj = None

    def fcl_obj(self):
        if self.obj is None:
            m = fcl.BVHModel()
            m.beginModel(len(self.V), len(self.F))
            m.addSubModel(self.V, self.F)
            m.endModel()
            self.obj = fcl.CollisionObject(m, fcl.Transform())
        return self.obj


def leg_local(fx, sy):
    out = []
    for n, s, c in M.leg_parts_FL():
        if sy < 0:
            s = M.mirror_y(s)
        if fx < 0:
            s = M.mirror_x(s)
        out.append((n, s, c))
    return out


PARTS = [Part("body", n, s, c) for n, s, c in M.body_parts()]
PARTS += [Part("head", n, s, c) for n, s, c in M.head_parts()]
for k, (fx, sy, tag) in LEGS.items():
    PARTS += [Part(k, tag + n, s, c) for n, s, c in leg_local(fx, sy)]
print(f"零件 {len(PARTS)} 个，建模 {time.time() - t0:.0f} 秒")


def roty(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def group_tf(g, theta, bz, pitch):
    """刚体组的 (旋转矩阵, 平移, 绕 Y 的角度, 转轴点)。"""
    if g == "body":
        return np.eye(3), np.array([0.0, 0.0, bz]), 0.0, None
    if g == "head":
        return roty(pitch), np.array([NECK_X, 0.0, PIVOT_Z + bz]), pitch, (NECK_X, PIVOT_Z + bz)
    fx = LEGS[g][0]
    a = theta * fx                           # 后腿几何是前腿镜像过去的，转角反号
    return roty(a), np.array([fx * HIP_X, 0.0, HIP_Z + bz]), a, (fx * HIP_X, HIP_Z + bz)


def place(p, pose):
    Rm, t, a, _ = group_tf(p.g, pose["theta"], pose["bz"], pose["pitch"])
    s = p.s
    if a:
        s = s.rotate(V(0, 0, 0), V(0, 1, 0), a)
    return s.translate(V(*t))


def world_verts(p, tf):
    Rm, t = tf[0], tf[1]
    return p.V @ Rm.T + t


# ---------------------------------------------------------------- 干涉检查（实体求交）
KNOWN = [("髋轴", "侧板"), ("髋轴", "大带轮"), ("髋轴", "轴承座"), ("髋轴", "轴毂"), ("同步带", "带轮"),
         ("髋轴", "箱形臂座"), ("俯仰舵机", "肩座"), ("屏板", "屏幕头"), ("屏幕玻璃", "屏幕头"), ("轴毂", "箱形臂座"),
         ("麦轮轮毂", "麦轮"), ("麦轮轮毂", "轮子电机"), ("栏板", "导轨"), ("俯仰轴销", "俯仰铜套"), ("雷达底座", "肩座")]


def known(a, b):
    return any((x in a and y in b) or (x in b and y in a) for x, y in KNOWN)


def interference(pose):
    shapes = [place(p, pose) for p in PARTS]
    bbs = [s.BoundingBox() for s in shapes]
    hits = []
    for i in range(len(PARTS)):
        for j in range(i + 1, len(PARTS)):
            if known(PARTS[i].name, PARTS[j].name) or is_visual(PARTS[i].name) or is_visual(PARTS[j].name):
                continue
            if PARTS[i].g == PARTS[j].g and (PARTS[i].name.startswith("外壳·") or PARTS[j].name.startswith("外壳·")):
                continue                                   # 装饰件贴在同一部件的零件上，本来就挨着
            a, b = bbs[i], bbs[j]
            if a.xmax < b.xmin or b.xmax < a.xmin or a.ymax < b.ymin or b.ymax < a.ymin or a.zmax < b.zmin or b.zmax < a.zmin:
                continue
            v = shapes[i].intersect(shapes[j]).Volume()
            if v > 2.0:
                hits.append((v, PARTS[i].name, PARTS[j].name))
    return sorted(hits, reverse=True)


# ---------------------------------------------------------------- 尺寸、离地
def pose_geometry(pose):
    tfs = {g: group_tf(g, pose["theta"], pose["bz"], pose["pitch"]) for g in ("body", "head", *LEGS)}
    lo, hi = np.full(3, 1e9), np.full(3, -1e9)
    zmin = {}
    for p in PARTS:
        w = world_verts(p, tfs[p.g])
        lo, hi = np.minimum(lo, w.min(0)), np.maximum(hi, w.max(0))
        zmin[p.name] = w[:, 2].min()
    return lo, hi, zmin


def is_wheel(n):
    return "麦轮" in n


def is_visual(n):
    """只用来显示的零件（滚子、侧板），都在麦轮外包络里面，查碰撞用外包络代替。"""
    return "麦轮滚子" in n or "麦轮侧板" in n


# ---------------------------------------------------------------- 重量、重心
def part_mass(p, frame_vol, bed_vol):
    for key in sorted(PART_MASS, key=len, reverse=True):
        if key in p.name:
            return PART_MASS[key]
    if is_bed(p.name):
        return MASS_BED * p.vol / bed_vol
    if is_frame(p.name):
        return MASS_FRAME * p.vol / frame_vol
    raise KeyError(f"没给重量：{p.name}")


def is_bed(n):
    return any(k in n for k in ("导轨", "栏板", "机械臂接口板"))


def is_frame(n):
    return not is_bed(n) and not is_wheel(n) and any(k in n for k in ("侧板", "底板", "前板", "后板", "甲板", "2020横梁", "滑撬"))


FRAME_VOL = sum(p.vol for p in PARTS if is_frame(p.name))
BED_VOL = sum(p.vol for p in PARTS if is_bed(p.name))
for p in PARTS:
    p.m = part_mass(p, FRAME_VOL, BED_VOL)


def tri_margin(p, tri):
    """点 p 到三角形各边的最小有向距离（正 = 在里面）。"""
    a, b, c = tri
    if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) < 0:
        b, c = c, b                                   # 统一成逆时针
    out = []
    for u, v in ((a, b), (b, c), (c, a)):
        e = v - u
        out.append((e[0] * (p[1] - u[1]) - e[1] * (p[0] - u[0])) / np.linalg.norm(e))
    return min(out)


def com(pose, payload):
    tfs = {g: group_tf(g, pose["theta"], pose["bz"], pose["pitch"]) for g in ("body", "head", *LEGS)}
    ms, cs = [], []
    for p in PARTS:
        Rm, t = tfs[p.g][:2]
        ms.append(p.m)
        cs.append(Rm @ p.cen + t)
    ms.append(MASS_SHELLS)
    cs.append(np.array([0.0, 0.0, BODY_H / 2 + pose["bz"]]))
    if payload:
        ms.append(PAYLOAD)
        cs.append(np.array([(BED_X0 + BED_X1) / 2, 0.0, M.DECK + PAYLOAD_UP + pose["bz"]]))
    ms = np.array(ms)
    return ms.sum(), (np.array(cs) * ms[:, None]).sum(0) / ms.sum()


# ---------------------------------------------------------------- 折叠轨迹（逐步摆姿态，用 fcl 算最近距离）
def sweep_steps():
    steps = []
    lv, fd = HEAD_PITCH["level"], HEAD_PITCH["fold"]
    n1 = int(abs(STANCE_DEG - THETA_GROUND) / 2) + 1
    for i in range(n1 + 1):
        t = STANCE_DEG + (THETA_GROUND - STANCE_DEG) * i / n1
        steps.append(("①轮子着地、机身降到滑撬上", t, bz_ground(t), lv + (fd - lv) * i / n1))
    n2 = int(abs(THETA_GROUND - THETA_FOLD) / 2) + 1
    for i in range(1, n2 + 1):
        steps.append(("②摆臂往上往里翻", THETA_GROUND + (THETA_FOLD - THETA_GROUND) * i / n2, float(SKID_H), fd))
    up, dn = HEAD_PITCH["up"], HEAD_PITCH["down"]
    for i in range(23):
        steps.append(("③行驶时抬头到低头", STANCE_DEG, bz_ground(STANCE_DEG), up + (dn - up) * i / 22))
    return steps


SWEEP_SKIP = ("髋轴",)          # 髋轴和轴毂本来就套在一起（转轴），不算
SWEEP_GAP = [("轴承座", "臂座轴毂"), ("轴承座", "箱形臂座")]   # 髋部固定的转动缝（1 mm 垫片缝、3 mm），不随姿态变，不算进“最近距离”


def sweep():
    MARGIN = 40.0
    near = {}          # (组a, 组b) -> (距离, 零件a, 零件b, 阶段, 摆臂角, 头角)
    hits = []
    ground = {}        # 阶段 -> (最低, 零件)
    req_c, req_d = fcl.CollisionRequest(), fcl.DistanceRequest()
    parts = [p for p in PARTS if not any(k in p.name for k in SWEEP_SKIP) and not is_visual(p.name)]
    for stage, th, bz, pt in sweep_steps():
        tfs = {g: group_tf(g, th, bz, pt) for g in ("body", "head", *LEGS)}
        boxes = []
        for p in parts:
            w = world_verts(p, tfs[p.g])
            lo, hi = w.min(0), w.max(0)
            boxes.append((lo, hi))
            if not is_wheel(p.name) and "滑撬" not in p.name:
                z = lo[2]
                if stage not in ground or z < ground[stage][0]:
                    ground[stage] = (z, p.name)
            elif lo[2] < -0.5:
                hits.append((stage, th, pt, p.name, "地面", -lo[2]))
        for i in range(len(parts)):
            pi = parts[i]
            for j in range(i + 1, len(parts)):
                pj = parts[j]
                if pi.g == pj.g or known(pi.name, pj.name):
                    continue
                (la, ha), (lb, hb) = boxes[i], boxes[j]
                if np.any(la - MARGIN > hb) or np.any(lb - MARGIN > ha):
                    continue
                oa, ob = pi.fcl_obj(), pj.fcl_obj()
                oa.setTransform(fcl.Transform(*tfs[pi.g][:2]))
                ob.setTransform(fcl.Transform(*tfs[pj.g][:2]))
                if fcl.collide(oa, ob, req_c, fcl.CollisionResult()):
                    d = 0.0
                    hits.append((stage, th, pt, pi.name, pj.name, 0.0))
                else:
                    d = fcl.distance(oa, ob, req_d, fcl.DistanceResult())
                if any((x in pi.name and y in pj.name) or (x in pj.name and y in pi.name) for x, y in SWEEP_GAP):
                    continue
                key = tuple(sorted((pi.g, pj.g)))
                if key not in near or d < near[key][0]:
                    near[key] = (d, pi.name, pj.name, stage, th, pt)
    return near, hits, ground


if __name__ == "__main__":
    # ---------------------------------------------------------------- 开始
    report = [f"机械狗007v骨架检查报告  {time.strftime('%Y-%m-%d %H:%M')}", "改尺寸：007v/config.py；重新生成：python 007v/build.py", ""]
    rep = report.append

    rep("【姿态】")
    for k, ps in POSES.items():
        lo, hi, zmin = pose_geometry(ps)
        L, W, H = hi - lo
        ps["dims"] = (L, W, H)
        rep(f"  {ps['name']}：摆臂 {ps['theta']:+.1f}°，头 {ps['pitch']:+.0f}°，机身底离地 {ps['bz']:.0f} mm → "
            f"长 {L:.0f} × 宽 {W:.0f} × 高 {H:.0f} mm")
        low = min((z, n) for n, z in zmin.items() if not is_wheel(n) and "滑撬" not in n)
        wheel = min(z for n, z in zmin.items() if is_wheel(n))
        skid = min(z for n, z in zmin.items() if "滑撬" in n)
        rep(f"      滑撬离地 {skid:.0f} mm；轮子最低 {wheel:.1f} mm；除轮子和滑撬外最低的是 {low[1]}（{low[0]:.0f} mm）")
    rep(f"  机身坐上滑撬时的摆臂角 {THETA_GROUND:+.1f}°（从这里往上翻，轮子离地）")
    rep("")

    rep("【干涉检查】（实体求交，> 2 mm³ 算撞；转轴、皮带、卡槽这些本来就套在一起的不算）")
    for k, ps in POSES.items():
        hits = interference(ps)
        ps["hits"] = hits
        rep(f"  {ps['name']}：{len(hits)} 处" + ("" if not hits else "：" + "；".join(f"{a} ↔ {b} {v:.0f} mm³" for v, a, b in hits[:20])))
    print(f"干涉检查完 {time.time() - t0:.0f} 秒")
    rep("")

    rep("【折叠过程 + 抬头低头】（每 2° 摆一次，算不同刚体之间的最近距离；40 mm 以外不细算；髋部轴承座和臂座之间固定的 1 mm / 3 mm 转动缝不算）")
    near, shits, ground = sweep()
    if shits:
        seen = set()
        for stage, th, pt, a, b, d in shits:
            if (a, b) in seen:
                continue
            seen.add((a, b))
            rep(f"  撞：{stage}，摆臂 {th:+.0f}°、头 {pt:+.0f}° 时 {a} ↔ {b}" + (f"（进地 {d:.1f} mm）" if b == "地面" else ""))
    else:
        rep("  全程没有相撞，也没有零件进到地面以下")
    CN = dict(body="机身", head="屏幕头", **{k: v[2] + "腿" for k, v in LEGS.items()})
    for (ga, gb), (d, a, b, stage, th, pt) in sorted(near.items(), key=lambda x: x[1][0]):
        rep(f"  {CN[ga]} ↔ {CN[gb]}：最近 {d:.1f} mm（{a} ↔ {b}，{stage}，摆臂 {th:+.0f}°、头 {pt:+.0f}°）")
    for stage, (z, n) in ground.items():
        rep(f"  {stage}：除轮子和滑撬外离地最低 {z:.0f} mm（{n}）")
    print(f"折叠轨迹完 {time.time() - t0:.0f} 秒")
    rep("")

    rep("【重量和重心】")
    dep = POSES["deploy"]
    xf = HIP_X + ARM * math.cos(math.radians(STANCE_DEG))
    yw = BODY_W / 2 + WHEEL_GAP + WHEEL_W / 2
    summary = {}
    for label, pl in (("空车", False), (f"满载 {PAYLOAD:.0f} kg", True)):
        m, c = com(dep, pl)
        h = c[2]
        front = (c[0] + xf) / (2 * xf)
        nf, nr = m * front / 2, m * (1 - front) / 2
        lever = ARM * math.cos(math.radians(STANCE_DEG)) / 1000
        tip_side = math.degrees(math.atan((yw - abs(c[1])) / h))
        tip_fwd = math.degrees(math.atan((xf - c[0]) / h))
        tip_back = math.degrees(math.atan((xf + c[0]) / h))
        # 原地直接抬一条腿（其他腿不动）：重心要落在另外三个轮子围成的三角形里
        wheels = {k: np.array([LEGS[k][0] * xf, LEGS[k][1] * yw]) for k in LEGS}
        margins = {k: tri_margin(c[:2], [wheels[j] for j in LEGS if j != k]) for k in LEGS}
        rep(f"  {label}：{m:.1f} kg，重心 x {c[0]:+.0f} / y {c[1]:+.0f} / 离地 {h:.0f} mm")
        rep(f"      前轮各 {nf:.1f} kg、后轮各 {nr:.1f} kg；撑住车身的髋扭矩 前 {nf * 9.81 * lever:.1f} / 后 {nr * 9.81 * lever:.1f} N·m")
        rep(f"      侧翻临界 {tip_side:.0f}°，往前翻 {tip_fwd:.0f}°，往后翻 {tip_back:.0f}°")
        rep("      其他腿不动、直接抬一条腿，重心离三轮三角形的边：" + "，".join(f"{LEGS[k][2]} {v:+.0f} mm" for k, v in margins.items())
            + "（正 = 在里面；要靠调姿势加大，见仿真报告）")
        summary[label] = dict(mass=round(m, 1), com=[round(float(v)) for v in c], tip_side=round(tip_side), tip_fwd=round(tip_fwd),
                              tip_back=round(tip_back), lift_margin={LEGS[k][2]: round(float(v)) for k, v in margins.items()})
    # 从趴着撑起来（满载）：沿第①段反着走，髋扭矩最大的地方
    m, c0 = com(dep, True)
    worst = (0, 0)
    for _, th, bz, _ in [s for s in sweep_steps() if s[0].startswith("①")]:
        x = HIP_X + ARM * math.cos(math.radians(th))
        front = (c0[0] + x) / (2 * x)
        tq = max(front, 1 - front) * m / 2 * 9.81 * ARM * math.cos(math.radians(th)) / 1000
        worst = max(worst, (tq, th))
    rep(f"  满载从趴着撑起来：髋扭矩最大 {worst[0]:.1f} N·m（摆臂 {worst[1]:+.0f}°）；髋驱动额定约 {6.9 * 3:.1f} N·m")
    rep("")

    rep("【整机宽度】")
    rep(f"  机身 {BODY_W:.0f} + 两边各（间隙 {WHEEL_GAP:.0f} + 轮宽 {WHEEL_W:.0f} + 轮子电机 {WHEEL_MOTOR_W:.0f} + 外板 {OUTER_PLATE_T:.0f}）"
        f"= {2 * M.half_width():.0f} mm")
    rep("")

    total_m = sum(p.m for p in PARTS) + MASS_SHELLS
    rep(f"零件 {len(PARTS)} 个，自重合计 {total_m:.1f} kg（含外壳 {MASS_SHELLS} kg）；用时 {time.time() - t0:.0f} 秒")

    with open(os.path.join(OUT, "report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(report) + "\n")
    print("\n".join(report))


    # ---------------------------------------------------------------- GLB（各刚体组在自己的局部坐标里，按 组|颜色 分节点）+ rig.json
    from look import COL, EMISSIVE, METAL, gl, lin


    def node_of(p):
        """GLB 节点：麦轮单独一组（原点在轮轴上，能转），其余按刚体组。"""
        if p.g in LEGS and is_wheel(p.name):
            return p.g + "w", np.array([LEGS[p.g][0] * ARM, 0.0, 0.0])
        return p.g, np.zeros(3)


    scene = trimesh.Scene()
    for g in ("body", "head", *LEGS, *[k + "w" for k in LEGS]):
        bycol = {}
        for p in PARTS:
            ng, off = node_of(p)
            if ng != g or p.c == "envelope":
                continue
            vs, fs = p.s.tessellate(0.1, 0.12) if "滚子" not in p.name else p.s.tessellate(0.3, 0.3)
            if not fs:
                continue
            v = np.array([[q.x, q.y, q.z] for q in vs]) - off
            bycol.setdefault(p.c, []).append(trimesh.Trimesh(vertices=np.column_stack([v[:, 1], v[:, 2], v[:, 0]]) / 1000.0,
                                                             faces=np.array(fs), process=False))
        for c, ms in bycol.items():
            mesh = trimesh.util.concatenate(ms)
            rgb = [lin(int(COL[c][i:i + 2], 16)) for i in (1, 3, 5)]          # glTF 的颜色是线性空间
            mesh.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
                name=c, baseColorFactor=rgb + [255], metallicFactor=0.85 if c in METAL else (0.4 if c == "motor" else 0.05),
                roughnessFactor=0.35 if c in METAL else (0.1 if c == "glass" else 0.7),
                emissiveFactor=list(EMISSIVE.get(c, (0, 0, 0)))))
            scene.add_geometry(mesh, node_name=f"{g}|{c}", geom_name=f"{g}|{c}")
    scene.export(os.path.join(OUT, "007v.glb"))

    rig = dict(
        pivots={g: gl((LEGS[g][0] * HIP_X, 0, HIP_Z)) for g in LEGS} | {"head": gl((NECK_X, 0, PIVOT_Z)), "body": [0, 0, 0]},
        sign={g: LEGS[g][0] for g in LEGS},
        wheel={g: gl((LEGS[g][0] * ARM, 0, 0)) for g in LEGS},           # 轮轴在摆臂坐标里的位置
        R=R / 1000, HIP_Z=HIP_Z / 1000, ARM=ARM / 1000, SKID_H=SKID_H / 1000,
        deck=M.DECK / 1000, bed=[(BED_X0 + PROFILE) / 1000, (BED_X1 - PROFILE) / 1000], bed_half_w=(RAIL_Y - PROFILE / 2) / 1000,
        payload=PAYLOAD, payload_up=PAYLOAD_UP / 1000,
        theta=dict(stance=STANCE_DEG, ground=THETA_GROUND, fold=THETA_FOLD),
        pitch=HEAD_PITCH,
        dims={POSES[k]["name"]: [round(float(v)) for v in POSES[k]["dims"]] for k in POSES},
        hits={POSES[k]["name"]: len(POSES[k]["hits"]) for k in POSES},
        sweep_hits=len(shits),
        near={f"{CN[a]}↔{CN[b]}": round(v[0], 1) for (a, b), v in near.items()},
        mass=summary,
    )
    with open(os.path.join(OUT, "rig.json"), "w", encoding="utf-8") as f:
        json.dump(rig, f, ensure_ascii=False, indent=1)
    pack_viewer()
    print(f"导出 007v/out/007v.glb、rig.json、007v_data.js，共 {time.time() - t0:.0f} 秒")
