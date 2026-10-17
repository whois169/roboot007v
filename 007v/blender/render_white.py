"""白模渲染：打开 make_blend.py 做好的 .blend，换成两种灰度的哑光材质，出正面、侧面、前 45° 三张。看结构层次用。

- 框架、关节、螺栓、电机、轮子（金属件）= 深灰；外壳、盖板（打印件）= 浅灰；灯带不发光
- 一盏单侧强主光（左前方 45° 侧上方，平行光，影子硬）+ 一盏很弱的补光；用 Cycles 光线追踪，缝和孔里有真实阴影（环境光遮蔽）
- 1800×1150，显卡渲染（没有显卡就用 CPU）

用法：blender --background --factory-startup --python 007v/blender/render_white.py -- <机械狗007v.blend> <输出目录> [前缀]
"""
import math
import os
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
BLEND, OUT = argv[0], argv[1]
PREFIX = argv[2] if len(argv) > 2 else "白模"
bpy.ops.wm.open_mainfile(filepath=BLEND)
scene = bpy.context.scene

LIGHT_KEYS = ("shell", "deck", "armor", "printed")     # 打印外壳、盖板


def matte(name, v):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (v, v, v, 1)
    b.inputs["Roughness"].default_value = 0.8
    b.inputs["Metallic"].default_value = 0.0
    return m


light, dark, floor = matte("白模·外壳", 0.75), matte("白模·框架", 0.16), matte("白模·地面", 0.45)
for o in bpy.data.objects:
    if o.type != "MESH":
        continue
    keys = [ms.material.name.split(".")[0] for ms in o.material_slots if ms.material]
    mat = floor if o.name == "地面" else light if (keys and keys[0] in LIGHT_KEYS) else dark
    o.data.materials.clear()
    o.data.materials.append(mat)

# ---- 灯：原来的影棚灯关掉，换成一盏主光 + 一盏补光
for o in bpy.data.objects:
    if o.type == "LIGHT":
        o.hide_render = True
world = scene.world or bpy.data.worlds.new("白模")
scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes.get("Background")
bg.inputs["Color"].default_value = (0.85, 0.87, 0.9, 1)
bg.inputs["Strength"].default_value = 0.12


def add_light(name, kind, energy, direction, **kw):
    L = bpy.data.objects.new(name, bpy.data.lights.new(name, kind))
    L.data.energy = energy
    for k, v in kw.items():
        setattr(L.data, k, v)
    d = Vector(direction).normalized()                    # 光照过去的方向
    L.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    L.location = -d * 3
    scene.collection.objects.link(L)
    return L


# 机身前方 = Blender 的 -Y，机身左边 = +X。主光从左前方、45° 侧上方照过来
add_light("主光", "SUN", 4.5, (-0.5, 0.5, -0.71), angle=math.radians(2.0))
add_light("补光", "SUN", 0.6, (0.6, -0.2, -0.45), angle=math.radians(15))

# ---- Cycles，显卡优先
scene.render.engine = "CYCLES"
try:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    for dev in ("CUDA",):                                  # OptiX 在这台电脑上内核加载失败，用 CUDA
        try:
            prefs.compute_device_type = dev
            prefs.get_devices()
            if any(d.type == dev for d in prefs.devices):
                for d in prefs.devices:
                    d.use = d.type in (dev, "CPU")
                scene.cycles.device = "GPU"
                print("用显卡渲染：", dev)
                break
        except Exception:
            continue
except Exception as e:
    print("没有显卡渲染，用 CPU：", e)
scene.cycles.samples = 128
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = 1800, 1150
scene.view_settings.view_transform = "Standard"
scene.view_settings.look = "None"

cam = scene.camera
aim = next((c.target for c in cam.constraints if hasattr(c, "target") and c.target), None)
cam.data.lens = 70
if aim:
    aim.location = (0, -0.02, 0.09)
os.makedirs(OUT, exist_ok=True)
for nm, loc in (("正面", (0.0, -1.85, 0.22)), ("侧面", (1.85, -0.05, 0.2)), ("前45度", (1.25, -1.3, 0.42))):
    cam.location = loc
    scene.frame_set(5)
    scene.render.filepath = os.path.join(OUT, f"{PREFIX}_{nm}.png")
    try:
        bpy.ops.render.render(write_still=True)
    except RuntimeError as e:                              # 显卡渲染出错：退回 CPU 再来
        print("显卡渲染失败，改用 CPU：", e)
        scene.cycles.device = "CPU"
        bpy.ops.render.render(write_still=True)
print("白模渲染完成：", OUT)
