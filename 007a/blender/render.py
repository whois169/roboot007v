"""007A 效果图：读 007a/model.py 导出的 STL + manifest，按配色 B（外壳浅灰白、骨架深灰）上材质，出几个角度。

用法：blender --background --factory-startup --python 007a/blender/render.py -- <parts 目录> <输出目录> [角度,逗号分开]
角度：前45 / 侧面 / 后45 / 正面 / 俯视45
"""
import json
import math
import os
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
SRC, OUT = os.path.abspath(argv[0]), os.path.abspath(argv[1])
ONLY = argv[2].split(",") if len(argv) > 2 else None
os.makedirs(OUT, exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene


def srgb(h):
    def f(x):
        x /= 255
        return x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4
    return tuple(f(int(h[i:i + 2], 16)) for i in (1, 3, 5)) + (1.0,)


def principled(name, col, rough, metal=0.0, coat=0.0, emit=None, strength=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = col
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    if coat:
        b.inputs["Coat Weight"].default_value = coat
        b.inputs["Coat Roughness"].default_value = 0.05
    if emit:
        b.inputs["Emission Color"].default_value = emit
        b.inputs["Emission Strength"].default_value = strength
    return m


M = dict(shell=principled("外壳·喷涂", srgb("#d9dbdd"), 0.42),
         skel=principled("骨架·阳极氧化", srgb("#3d4247"), 0.38, 0.35),
         skel_dk=principled("深灰", srgb("#25282c"), 0.45, 0.2),
         metal=principled("拉丝金属", srgb("#a3a9af"), 0.28, 1.0),
         rubber=principled("橡胶", srgb("#141516"), 0.85),
         glass=principled("烟黑玻璃", srgb("#0a0d10"), 0.05, coat=1.0),
         cyan=principled("青色光", (0.0, 0.6, 1.0, 1), 0.3, emit=(0.0, 0.75, 1.0, 1), strength=4.0),
         eye=principled("眼睛", (0.3, 0.7, 1.0, 1), 0.3, emit=(0.47, 0.78, 1.0, 1), strength=3.0),
         red=principled("尾灯", (1.0, 0.05, 0.03, 1), 0.3, emit=(1.0, 0.04, 0.02, 1), strength=5.0),
         estop=principled("急停红", srgb("#c8102e"), 0.3),
         lamp=principled("探照灯", (0.95, 0.97, 1.0, 1), 0.15, emit=(1.0, 0.97, 0.9, 1), strength=2.5))

man = json.load(open(os.path.join(SRC, "manifest.json"), encoding="utf-8"))
objs = []
for it in man:
    bpy.ops.wm.stl_import(filepath=os.path.join(SRC, it["file"]))
    o = bpy.context.selected_objects[0]
    o.name = it["name"]
    o.scale = (0.001, 0.001, 0.001)
    o.rotation_euler = (0, 0, math.radians(90))                   # 机身 x 前 → Blender +Y 前
    o.data.materials.clear()
    o.data.materials.append(M.get(it["color"], M["skel"]))
    bpy.ops.object.shade_auto_smooth(angle=math.radians(35))
    objs.append(o)
bpy.context.view_layer.update()
lo, hi = Vector((1e9,) * 3), Vector((-1e9,) * 3)
for o in objs:
    for v in o.bound_box:
        w = o.matrix_world @ Vector(v)
        lo, hi = Vector(map(min, lo, w)), Vector(map(max, hi, w))
ctr = (lo + hi) / 2

# ---- 地面（无缝背景）+ 灯光
bpy.ops.mesh.primitive_plane_add(size=30, location=(0, 0, 0))
floor = bpy.context.object
floor.data.materials.append(principled("地面", srgb("#8d9196"), 0.6))
world = bpy.data.worlds.new("w")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs[0].default_value = (0.82, 0.84, 0.87, 1)
world.node_tree.nodes["Background"].inputs[1].default_value = 0.35


def light(name, kind, energy, direction, loc=None, size=1.0):
    L = bpy.data.objects.new(name, bpy.data.lights.new(name, kind))
    L.data.energy = energy
    if kind == "AREA":
        L.data.size = size
    d = Vector(direction).normalized()
    L.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    L.location = loc if loc is not None else ctr - d * 3
    scene.collection.objects.link(L)


light("主光", "SUN", 3.2, (-0.5, -0.4, -0.75))
light("补光", "AREA", 260, (0.7, 0.5, -0.3), loc=ctr + Vector((-1.6, -1.2, 0.8)), size=2.5)
light("轮廓光", "AREA", 180, (0.0, -0.8, -0.3), loc=ctr + Vector((0.0, 1.8, 1.0)), size=2.0)

scene.render.engine = "CYCLES"
scene.cycles.samples = 96
scene.cycles.use_denoising = True
try:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "CUDA"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = True
    scene.cycles.device = "GPU"
except Exception:
    pass
scene.render.resolution_x, scene.render.resolution_y = 1800, 1100
scene.view_settings.view_transform = "AgX"
cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
scene.collection.objects.link(cam)
scene.camera = cam
cam.data.lens = 60

VIEWS = {"前45": (1.35, 1.35, 0.55), "侧面": (2.0, 0.0, 0.25), "后45": (-1.3, -1.35, 0.6), "正面": (0.0, 2.0, 0.3),
         "俯视45": (1.1, 1.0, 1.4)}
for nm, d in VIEWS.items():
    if ONLY and nm not in ONLY:
        continue
    dv = Vector(d).normalized()
    cam.location = ctr + dv * 1.75
    cam.rotation_euler = (ctr - cam.location).to_track_quat("-Z", "Y").to_euler()
    scene.render.filepath = os.path.join(OUT, f"007A_{nm}.png")
    try:
        bpy.ops.render.render(write_still=True)
    except RuntimeError:
        scene.cycles.device = "CPU"
        bpy.ops.render.render(write_still=True)
print("渲染完成：", OUT)
