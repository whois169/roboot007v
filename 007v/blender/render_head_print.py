"""头部打印件爆炸图：读 007v/out/head_print/_asm/*.stl（装配位置，mm），按零件往外拉开，出两张图（爆炸图 + 装好的剖面图）。

用法：blender --background --factory-startup --python 007v/blender/render_head_print.py -- <_asm 目录> <输出目录>
"""
import math
import os
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
SRC, OUT = os.path.abspath(argv[0]), os.path.abspath(argv[1])
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene

# 脖子坐标（x 前、y 左、z 上，mm）→ Blender（x 左、y 后、z 上，m）
COL = dict(shell=(0.69, 0.70, 0.72), dark=(0.045, 0.05, 0.055), black=(0.012, 0.012, 0.014), petg=(0.30, 0.33, 0.36),
           clear=(0.55, 0.85, 0.95), pcb=(0.05, 0.25, 0.55), servo=(0.03, 0.03, 0.035), glass=(0.01, 0.012, 0.015))
PARTS = {   # 名字: (颜色, 爆炸方向 mm)
    "tub": ("shell", (0, 0, 0)), "cap": ("shell", (0, 0, 95)), "retainer": ("petg", (-75, 0, 0)), "tray": ("petg", (0, 0, -55)),
    "frame": ("dark", (70, 0, 0)), "b_acrylic": ("glass", (50, 0, 0)), "b_board": ("pcb", (-38, 0, 0)),
    "core_L": ("shell", (0, 75, 0)), "mid_L": ("shell", (0, 120, 0)), "outer_L": ("shell", (0, 160, 0)), "hub_L": ("dark", (0, 38, 0)),
    "ring": ("clear", (0, 30, 0)), "b_servo": ("servo", (0, 98, 0)),
    "core_R": ("shell", (0, -75, 0)), "mid_R": ("shell", (0, -120, 0)), "outer_R": ("shell", (0, -160, 0)), "hub_R": ("dark", (0, -38, 0)),
    "window": ("black", (35, -75, 25)),
    "flange": ("dark", (0, 75, 45)), "b_lidar": ("black", (0, 75, 85)), "lidar_ring": ("dark", (0, 75, 130)),
    "neck": ("dark", (0, 0, -95)),
}


def mat(name, rgb):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (*rgb, 1)
    b.inputs["Roughness"].default_value = 0.55
    if name == "clear":
        b.inputs["Alpha"].default_value = 0.5
    return m


M = {k: mat(k, v) for k, v in COL.items()}
objs = {}
for k, (c, off) in PARTS.items():
    p = os.path.join(SRC, k + ".stl")
    if not os.path.exists(p):
        continue
    bpy.ops.wm.stl_import(filepath=p)
    o = bpy.context.selected_objects[0]
    o.name = k
    o.scale = (0.001, 0.001, 0.001)
    # 脖子坐标 → Blender：绕 Z 转 90°（x 前 → -y）
    o.rotation_euler = (0, 0, math.radians(90))
    o.data.materials.clear()
    o.data.materials.append(M[c])
    for poly in o.data.polygons:
        poly.use_smooth = False
    objs[k] = (o, Vector(off))

ground = bpy.data.objects.new("地面", bpy.data.meshes.new("地面"))
bm = ground.data
bm.from_pydata([(-3, -3, 0), (3, -3, 0), (3, 3, 0), (-3, 3, 0)], [], [(0, 1, 2, 3)])
scene.collection.objects.link(ground)
ground.data.materials.append(mat("floor", (0.55, 0.57, 0.6)))

world = bpy.data.worlds.new("w")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.85, 0.87, 0.9, 1)
world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.35


def sun(name, e, d, ang):
    L = bpy.data.objects.new(name, bpy.data.lights.new(name, "SUN"))
    L.data.energy = e
    L.data.angle = math.radians(ang)
    dv = Vector(d).normalized()
    L.rotation_euler = dv.to_track_quat("-Z", "Y").to_euler()
    scene.collection.objects.link(L)


sun("主光", 3.5, (-0.5, 0.5, -0.71), 3)
sun("补光", 1.0, (0.6, -0.2, -0.45), 15)

cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
scene.collection.objects.link(cam)
scene.camera = cam
aim = bpy.data.objects.new("aim", None)
scene.collection.objects.link(aim)
tc = cam.constraints.new("TRACK_TO")
tc.target = aim
tc.track_axis = "TRACK_NEGATIVE_Z"
tc.up_axis = "UP_Y"

scene.render.engine = "CYCLES"
try:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "CUDA"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = True
    scene.cycles.device = "GPU"
except Exception:
    pass
scene.cycles.samples = 64
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = 1800, 1350
scene.view_settings.view_transform = "Standard"
os.makedirs(OUT, exist_ok=True)


# 绕 Z 转 90°：(x, y, z) → (-y, x, z)，机身前面 = Blender +Y；相机放在 +X +Y 那边，看前脸和右（-Y）耳朵
def shot(name, explode, lift, cam_loc, aim_loc, lens):
    for k, (o, off) in objs.items():
        v = off * explode
        o.location = (-v.y / 1000, v.x / 1000, (v.z + lift) / 1000)
    cam.location = cam_loc
    cam.data.lens = lens
    aim.location = aim_loc
    scene.render.filepath = os.path.join(OUT, name + ".png")
    try:
        bpy.ops.render.render(write_still=True)
    except RuntimeError:
        scene.cycles.device = "CPU"
        bpy.ops.render.render(write_still=True)


shot("头部打印件_爆炸图", 1.0, 110, (0.62, 0.50, 0.42), (0.0, 0.01, 0.19), 30)
for k in ("cap", "frame", "b_acrylic"):                                   # 拿掉顶盖看里面
    if k in objs:
        objs[k][0].hide_render = True
shot("头部打印件_拿掉顶盖", 0.0, 0, (0.30, 0.22, 0.42), (0.0, 0.02, 0.10), 40)
for k in ("cap", "frame", "b_acrylic"):
    if k in objs:
        objs[k][0].hide_render = False
shot("头部打印件_装好", 0.0, 0, (0.55, 0.48, 0.38), (0.0, 0.0, 0.09), 50)
print("完成：", OUT)
