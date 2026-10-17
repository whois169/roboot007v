"""白模渲染：打开 make_blend.py 做好的 .blend，所有零件换成纯白哑光、灯带不发光，出正面、侧面、前 45° 三张。
看结构层次用（没有颜色和灯光干扰）。

用法：blender --background --factory-startup --python 007v/blender/render_white.py -- <机械狗007v.blend> <输出目录> [前缀]
"""
import math
import os
import sys

import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
BLEND, OUT = argv[0], argv[1]
PREFIX = argv[2] if len(argv) > 2 else "白模"
bpy.ops.wm.open_mainfile(filepath=BLEND)
scene = bpy.context.scene

white = bpy.data.materials.new("白模")
white.use_nodes = True
b = white.node_tree.nodes.get("Principled BSDF")
b.inputs["Base Color"].default_value = (0.82, 0.82, 0.82, 1)
b.inputs["Roughness"].default_value = 0.85
b.inputs["Metallic"].default_value = 0.0
dark = bpy.data.materials.new("白模·地面")
dark.use_nodes = True
dark.node_tree.nodes.get("Principled BSDF").inputs["Base Color"].default_value = (0.55, 0.56, 0.58, 1)
for o in bpy.data.objects:
    if o.type != "MESH":
        continue
    o.data.materials.clear()
    o.data.materials.append(dark if o.name == "地面" else white)

world = scene.world or bpy.data.worlds.new("白模")
scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes.get("Background")
bg.inputs["Color"].default_value = (0.9, 0.9, 0.92, 1)
bg.inputs["Strength"].default_value = 0.6
for o in bpy.data.objects:
    if o.type == "LIGHT":
        o.data.color = (1, 1, 1)

scene.render.engine = "BLENDER_EEVEE"
scene.eevee.taa_render_samples = 64
scene.render.resolution_x, scene.render.resolution_y = 1000, 640
cam = scene.camera
aim = next((c.target for c in cam.constraints if hasattr(c, "target") and c.target), None)
cam.data.lens = 70
if aim:
    aim.location = (0, -0.02, 0.09)
os.makedirs(OUT, exist_ok=True)
# 机身前方 = Blender 的 -Y，左边 = +X
for nm, loc in (("正面", (0.0, -1.85, 0.22)), ("侧面", (1.85, -0.05, 0.2)), ("前45度", (1.25, -1.3, 0.42))):
    cam.location = loc
    scene.frame_set(5)
    scene.render.filepath = os.path.join(OUT, f"{PREFIX}_{nm}.png")
    bpy.ops.render.render(write_still=True)
print("白模渲染完成：", OUT)
