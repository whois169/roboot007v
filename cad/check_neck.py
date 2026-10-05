"""
脖子转头 × 低头/抬头 全范围扫描：在站、坐、趴 3 个姿态下，找出每个偏航角能安全低头/抬头的最大角度（留 2mm 间隙）。
结果用来更新 head.py 里的 NECK_LIMITS 和《电子系统与接线》里的限位表。
先运行 build.py 生成 ../out/robodog.glb 和 rig.json。
用法：pip install python-fcl && python3 check_neck.py
"""
import os, json
import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix, translation_matrix

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "out")
sc = trimesh.load(os.path.join(OUT, "robodog.glb"))
rig = json.load(open(os.path.join(OUT, "rig.json")))
J = rig["joints"]
meshes = {}
for node in sc.graph.nodes_geometry:
    T, g = sc.graph[node]
    grp, col = node.split("|")
    if col == "keepout":
        continue
    meshes.setdefault(grp, []).append(sc.geometry[g].copy().apply_transform(T))
meshes = {k: trimesh.util.concatenate(v) for k, v in meshes.items()}


def world(grp, ang):
    if grp == "body":
        return np.eye(4)
    j = J[grp]
    p = np.array(j["point"])
    R = translation_matrix(p) @ rotation_matrix(np.radians(ang.get(grp, 0.0)), j["axis"]) @ translation_matrix(-p)
    return world(j["parent"], ang) @ R


def gap_mm(pose, yaw, pitch):
    ang = dict(rig["poses"][pose]["joints"])
    ang["neck_yaw"], ang["neck_pitch"] = yaw, pitch
    rest = trimesh.collision.CollisionManager()
    for g, me in meshes.items():
        if g not in ("neck_pitch", "neck_yaw"):
            rest.add_object(g, me, world(g, ang))
    head = trimesh.collision.CollisionManager()
    head.add_object("neck_pitch", meshes["neck_pitch"], world("neck_pitch", ang))
    return rest.min_distance_other(head) * 1000


for pose in ("stand", "sit", "lie"):
    print(pose)
    for yaw in (0, 10, 20, 25, 30, 35, 40, 45, 50):
        dn = max([p for p in np.arange(0, 25.1, 2.5) if gap_mm(pose, yaw, p) >= 2.0] or [None])
        up = min([p for p in np.arange(0, -15.1, -2.5) if gap_mm(pose, yaw, p) >= 2.0] or [None])
        print(f"  偏航 {yaw:2d}°：低头最多 {dn}°  抬头最多 {up}°（负 = 抬头）")
