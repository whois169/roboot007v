"""
外摆关节（HAA）全范围扫描：站立姿态下，每条腿单独从 -30° 摆到 +30°，看腿（髋罩、大腿、小腿）会不会碰到机身、头和另一条腿。
输出每条腿能安全摆动的范围（留 1mm 间隙；髋支架 A 板和髋座之间设计间隙就是 1.5mm）。先运行 build.py 生成 ../out/robodog.glb 和 rig.json。
用法：pip install python-fcl && python3 check_haa.py
"""
import os, json
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix, translation_matrix

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "out")
sc = trimesh.load(os.path.join(OUT, "robodog.glb"))
rig = json.load(open(os.path.join(OUT, "rig.json"), encoding="utf-8"))
J = rig["joints"]
meshes = {}
for node in sc.graph.nodes_geometry:
    T, g = sc.graph[node]
    grp, col = node.split("|")
    if col == "keepout" or (grp == "body" and col == "motor") or col == "flange":
        continue   # 外摆电机（机身上）和它的输出法兰（腿上）本来就贴在一起，不算碰撞
    meshes.setdefault(grp, []).append(sc.geometry[g].copy().apply_transform(T))
meshes = {k: trimesh.util.concatenate(v) for k, v in meshes.items()}


def world(grp, ang):
    if grp == "body":
        return np.eye(4)
    j = J[grp]
    p = np.array(j["point"])
    return world(j["parent"], ang) @ translation_matrix(p) @ rotation_matrix(np.radians(ang.get(grp, 0.0)), j["axis"]) @ translation_matrix(-p)


def gap_mm(leg, a):
    ang = dict(rig["poses"]["stand"]["joints"])
    ang[f"{leg}_haa"] = a
    mine = [f"{leg}_haa", f"{leg}_hfe", f"{leg}_knee"]
    rest, me = trimesh.collision.CollisionManager(), trimesh.collision.CollisionManager()
    for g, m in meshes.items():
        (me if g in mine else rest).add_object(g, m, world(g, ang))
    return me.min_distance_other(rest) * 1000


for leg in ("FL", "FR", "RL", "RR"):
    a0 = rig["poses"]["stand"]["joints"][leg + "_haa"]
    lo = hi = a0
    while lo - 5 >= -30 and gap_mm(leg, lo - 5) >= 1.0:
        lo -= 5
    while hi + 5 <= 30 and gap_mm(leg, hi + 5) >= 1.0:
        hi += 5
    print(f"{leg}：从站立的 {a0:+.0f}° 往两边摆，{lo:+.0f}° ~ {hi:+.0f}° 内都安全（每 5° 检查一次，到 ±30° 为止）")
