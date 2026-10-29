"""007A 爆炸图（从 007v 拷来，换成 007A 配色 B）：读 007a/exploded.py 写的 manifest.json + STL（mm，机身 / 腿坐标：x 前、y 左、z 上），按 offset 拉开，自动对焦出图，
再存一份每个零件在图上的位置（给 007a/tools_label_exploded.py 标中文名）。

用法：blender --background --factory-startup --python 007a/blender/render_exploded.py -- <目录> <输出 png> <相机方向 x,y,z> [炸开倍数]
"""
import json
import math
import os
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
SRC, PNG = os.path.abspath(argv[0]), os.path.abspath(argv[1])
CAMDIR = Vector([float(v) for v in argv[2].split(",")]).normalized()
K = float(argv[3]) if len(argv) > 3 else 1.0
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
def srgb(h):
    f = lambda x: x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4
    return tuple(f(int(h[i:i + 2], 16) / 255) for i in (1, 3, 5))


COL = dict(shell=srgb("#d9dbdd"), skel=srgb("#3d4247"), skel_dk=srgb("#25282c"), metal=srgb("#a3a9af"), rubber=srgb("#141516"),
           glass=srgb("#0a0d10"), cyan=(0.0, 0.6, 1.0), eye=(0.3, 0.7, 1.0), red=(1.0, 0.05, 0.03), estop=srgb("#c8102e"), lamp=(0.95, 0.97, 1.0))
EMIT = dict(cyan=((0.0, 0.75, 1.0), 4.0), eye=((0.47, 0.78, 1.0), 3.0), red=((1.0, 0.04, 0.02), 5.0), lamp=((1.0, 0.97, 0.9), 2.5))
mats = {}


def mat(k):
    if k not in mats:
        m = bpy.data.materials.new(k)
        m.use_nodes = True
        b = m.node_tree.nodes.get("Principled BSDF")
        b.inputs["Base Color"].default_value = (*COL.get(k, (0.5, 0.5, 0.5)), 1)
        b.inputs["Roughness"].default_value = dict(shell=0.42, skel=0.38, metal=0.28, rubber=0.85, glass=0.05).get(k, 0.45)
        b.inputs["Metallic"].default_value = dict(metal=1.0, skel=0.35, skel_dk=0.2).get(k, 0.0)
        if k in EMIT:
            b.inputs["Emission Color"].default_value = (*EMIT[k][0], 1)
            b.inputs["Emission Strength"].default_value = EMIT[k][1]
        mats[k] = m
    return mats[k]


man = json.load(open(os.path.join(SRC, "manifest.json"), encoding="utf-8"))
objs = []
for it in man:
    bpy.ops.wm.stl_import(filepath=os.path.join(SRC, it["file"]))
    o = bpy.context.selected_objects[0]
    o.scale = (0.001, 0.001, 0.001)
    o.rotation_euler = (0, 0, math.radians(90))                       # (x, y, z) → (-y, x, z)：机身前 = Blender +Y
    ox, oy, oz = [v * K / 1000 for v in it["offset"]]
    o.location = (-oy, ox, oz)
    o.data.materials.clear()
    o.data.materials.append(mat(it["color"]))
    objs.append((o, it))
bpy.context.view_layer.update()
lo, hi = Vector((1e9, 1e9, 1e9)), Vector((-1e9, -1e9, -1e9))
for o, _ in objs:
    for v in o.bound_box:
        w = o.matrix_world @ Vector(v)
        lo = Vector(map(min, lo, w))
        hi = Vector(map(max, hi, w))
ctr, size = (lo + hi) / 2, (hi - lo).length
world = bpy.data.worlds.new("w")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.92, 0.93, 0.95, 1)
world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.6
for nm, e, d in (("主光", 3.2, (-0.5, 0.5, -0.71)), ("补光", 1.2, (0.6, -0.3, -0.5))):
    L = bpy.data.objects.new(nm, bpy.data.lights.new(nm, "SUN"))
    L.data.energy = e
    L.rotation_euler = Vector(d).normalized().to_track_quat("-Z", "Y").to_euler()
    scene.collection.objects.link(L)
cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
scene.collection.objects.link(cam)
scene.camera = cam
cam.data.lens = 35
d = Vector((-CAMDIR.y, CAMDIR.x, CAMDIR.z))                              # 相机方向也按机身坐标给
cam.location = ctr + d * size * 1.05
cam.rotation_euler = (-d).to_track_quat("-Z", "Y").to_euler()
scene.render.engine = "CYCLES"
try:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "CUDA"
    prefs.get_devices()
    for dv in prefs.devices:
        dv.use = True
    scene.cycles.device = "GPU"
except Exception:
    pass
scene.cycles.samples = 64
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = 2000, 1400
scene.view_settings.view_transform = "Standard"
scene.render.filepath = PNG
try:
    bpy.ops.render.render(write_still=True)
except RuntimeError:
    scene.cycles.device = "CPU"
    bpy.ops.render.render(write_still=True)
from bpy_extras.object_utils import world_to_camera_view  # noqa: E402
lab = []
for o, it in objs:
    if not it["label"]:
        continue
    c = sum((o.matrix_world @ Vector(v) for v in o.bound_box), Vector()) / 8
    p = world_to_camera_view(scene, cam, c)
    lab.append(dict(label=it["label"], printed=it["printed"], x=p.x, y=1 - p.y))
json.dump(lab, open(PNG[:-4] + "_标注.json", "w", encoding="utf-8"), ensure_ascii=False)
print("完成", PNG)
