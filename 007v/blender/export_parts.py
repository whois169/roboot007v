"""给 Blender 准备数据：每个零件单独一个节点的 GLB + 场景说明（关节位置、拆散方向、动画时间轴）。

用法：python 007v/blender/export_parts.py      （之后 make_blend.py 在 Blender 里读这些）
输出：007v/out/blender/parts.glb、scene.json
动画时间轴（25 帧/秒）：① 展开 → 折叠 → 展开  ② 抬头 / 低头  ③ 拆散 / 装回  ④ 仿真回放（每个场景一段，时间轴上有标记）
"""
import json
import math
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import build as B  # noqa: E402
from config import *  # noqa: E402,F401,F403
from look import ANOD, COL, EMISSIVE, METAL, lin  # noqa: E402

OUT = os.path.join(ROOT, "out", "blender")
FPS = 25
GROUPS = dict(body="机身", head="屏幕头", **{k: f"{v[2]}摆臂" for k, v in B.LEGS.items()}, **{k + "w": f"{v[2]}麦轮" for k, v in B.LEGS.items()})


def part_group(p):
    if p.g in B.LEGS and B.is_wheel(p.name):
        return p.g + "w", np.array([B.LEGS[p.g][0] * ARM, 0.0, 0.0])
    return p.g, np.zeros(3)


def to_b(v):
    """机身坐标 mm（N×3）→ Blender m（N×3）。"""
    v = np.asarray(v, float)
    return np.column_stack([v[:, 1], -v[:, 0], v[:, 2]]) / 1000.0


def export_glb():
    """GLB 里节点名用编号（Blender 物体名有长度限制），中文名放 scene.json。GLB 是 Y 向上，导入 Blender 会自动转成 Z 向上。"""
    scene = trimesh.Scene()
    parts = []
    for i, p in enumerate(B.PARTS):
        if p.c == "envelope":
            continue
        g, off = part_group(p)
        vs, fs = p.s.tessellate(0.1, 0.12) if "滚子" not in p.name else p.s.tessellate(0.3, 0.3)
        if not fs:
            continue
        v = np.array([[q.x, q.y, q.z] for q in vs]) - off
        mesh = trimesh.Trimesh(vertices=np.column_stack([v[:, 1], v[:, 2], v[:, 0]]) / 1000.0, faces=np.array(fs), process=False)
        rgb = [lin(int(COL[p.c][k:k + 2], 16)) for k in (1, 3, 5)]
        mesh.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
            name=p.c, baseColorFactor=rgb + [255],
            metallicFactor=0.5 if p.c in ANOD else (0.85 if p.c in METAL else 0.05),
            roughnessFactor=0.42 if p.c in ANOD else (0.35 if p.c in METAL else (0.06 if p.c in ("glass", "gloss", "visor") else (0.88 if p.c in ("wheel", "black") else 0.72))), emissiveFactor=list(EMISSIVE.get(p.c, (0, 0, 0)))))
        node = f"P{i:03d}"
        scene.add_geometry(mesh, node_name=node, geom_name=node)
        cen = to_b([p.cen - off])[0]
        parts.append(dict(node=node, group=g, name=p.name, color=p.c, centroid=cen.tolist()))
    scene.export(os.path.join(OUT, "parts.glb"))
    return parts


def explode(parts):
    """拆散方向：组里的零件从组中心往外散；各组再整体往外挪（在父物体坐标里，m）。"""
    for g in GROUPS:
        ps = [p for p in parts if p["group"] == g]
        c = np.mean([p["centroid"] for p in ps], axis=0) if g != "body" else np.array([0.0, 0.0, BODY_H / 2000])
        k = 1.0 if g == "body" else 1.3
        for p in ps:
            p["explode"] = ((np.array(p["centroid"]) - c) * k).round(5).tolist()
    gd = {"body": [0, 0, 0], "head": [0, -0.05, 0.2]}
    for key, (fx, sy, _) in B.LEGS.items():
        gd[key] = [sy * 0.2, -fx * 0.12, 0.0]
        gd[key + "w"] = [sy * 0.16, 0.0, 0.0]
    return gd


def tracks():
    """整条时间轴：返回 {通道: [值，每帧一个]}、标记、障碍物。"""
    T = B.THETA_GROUND
    SPAN = STANCE_DEG - B.THETA_FOLD
    R, HZ, A = WHEEL_D / 2000, HIP_Z / 1000, ARM / 1000

    def leg_at(s):
        th = STANCE_DEG - SPAN * s
        bz = R - HZ + A * math.sin(math.radians(th)) if th >= T else SKID_H / 1000
        return th, bz

    ch = {k: [] for k in ["bx", "by", "bz", "qw", "qx", "qy", "qz", "head", "explode", "slope", "cargo", "bump"] +
          [f"h{k}" for k in B.LEGS] + [f"w{k}" for k in B.LEGS]}
    markers, obstacles = [], []

    def static(th, bz, pitch=0.0, e=0.0, cargo=0):
        for k, v in dict(bx=0, by=0, bz=bz, qw=1, qx=0, qy=0, qz=0, head=math.radians(pitch), explode=e, slope=0, cargo=cargo, bump=0).items():
            ch[k].append(v)
        for k in B.LEGS:
            ch[f"h{k}"].append(math.radians(th))
            ch[f"w{k}"].append(0.0)

    def ease(k):
        return k * k * (3 - 2 * k)

    def seg(name, seconds, fn):
        markers.append((len(ch["bz"]), name))
        n = int(seconds * FPS)
        for i in range(n):
            fn(i / max(n - 1, 1))

    th0, bz0 = leg_at(0)
    lv, fd = HEAD_PITCH["level"], HEAD_PITCH["fold"]
    seg("展开站着", 1.5, lambda k: static(th0, bz0))
    seg("折叠（摆臂往上往里翻）", 4.0, lambda k: static(*leg_at(ease(k)), lv + (fd - lv) * ease(k)))
    seg("折叠好了", 1.5, lambda k: static(*leg_at(1), fd))
    seg("展开（站起来）", 4.0, lambda k: static(*leg_at(1 - ease(k)), fd + (lv - fd) * ease(k)))
    up, dn = HEAD_PITCH["up"], HEAD_PITCH["down"]
    seg("抬头 / 低头", 5.0, lambda k: static(th0, bz0, lv + (up - lv) * math.sin(k * math.pi * 2) if k < 0.5 else lv + (dn - lv) * math.sin((k - 0.5) * math.pi * 2)))
    seg("拆散", 2.5, lambda k: static(th0, bz0, e=ease(k)))
    seg("拆散（停住看）", 3.0, lambda k: static(th0, bz0, e=1.0))
    seg("装回", 2.5, lambda k: static(th0, bz0, e=1 - ease(k)))

    sim = json.load(open(os.path.join(ROOT, "out", "sim.json"), encoding="utf-8"))
    F = {n: i for i, n in enumerate(sim["fields"])}
    for sc in sim["scenarios"]:
        start = len(ch["bz"])
        markers.append((start, "仿真：" + sc["title"]))
        for fr in sc["frames"]:
            x, y, z = fr[F["x"]], fr[F["y"]], fr[F["z"]]
            for k, v in dict(bx=y, by=-x, bz=z, qw=fr[F["qw"]], qx=fr[F["qy"]], qy=-fr[F["qx"]], qz=fr[F["qz"]],
                             head=fr[F["head"]], explode=0, slope=-math.radians(sc.get("slope", 0)), cargo=1,
                             bump=1 if sc.get("obstacles") else 0).items():
                ch[k].append(v)
            for k in B.LEGS:
                ch[f"h{k}"].append(fr[F["h" + k]])
                ch[f"w{k}"].append(fr[F["w" + k]])
        for o in sc.get("obstacles", []):
            obstacles.append(dict(loc=to_b([np.array(o["pos"]) * 1000])[0].tolist(),
                                  size=[o["size"][1] * 2, o["size"][0] * 2, o["size"][2] * 2]))
    return ch, markers, obstacles


def main():
    os.makedirs(OUT, exist_ok=True)
    parts = export_glb()
    gd = explode(parts)
    sign = {k: B.LEGS[k][0] for k in B.LEGS}
    groups = [dict(key="body", name=GROUPS["body"], parent=None, loc=[0, 0, 0]),
              dict(key="head", name=GROUPS["head"], parent="body", loc=to_b([[NECK_X, 0, B.PIVOT_Z]])[0].tolist())]
    for k, (fx, sy, _) in B.LEGS.items():
        groups.append(dict(key=k, name=GROUPS[k], parent="body", loc=to_b([[fx * HIP_X, 0, HIP_Z]])[0].tolist(), sign=fx))
        groups.append(dict(key=k + "w", name=GROUPS[k + "w"], parent=k, loc=to_b([[fx * ARM, 0, 0]])[0].tolist()))
    for g in groups:
        g["explode"] = gd[g["key"]]
    ch, markers, obstacles = tracks()
    cargo = dict(loc=to_b([[(BED_X0 + BED_X1) / 2, 0, B.M.DECK + PAYLOAD_UP]])[0].tolist(), size=[0.18, 0.18, 0.2])
    data = dict(fps=FPS, parts=parts, groups=groups, sign=sign, tracks=ch, markers=markers, obstacles=obstacles, cargo=cargo,
                frames=len(ch["bz"]))
    with open(os.path.join(OUT, "scene.json"), "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)
    print(f"导出 {len(parts)} 个零件、{len(ch['bz'])} 帧（{len(ch['bz']) / FPS:.0f} 秒），标记 {len(markers)} 个")


if __name__ == "__main__":
    main()
