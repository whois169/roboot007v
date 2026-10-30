"""把 007A 存成一个 .blend（Blender 5.x），打开就能转着看、拆开看。

用法：blender --background --factory-startup --python 007a/blender/make_blend.py -- <输出.blend>
读：007a/out/parts（外观，model.py / body_v10.py）、007a/out/exploded/whole（整机：外壳 + 骨架 + 电子 + 腿模块，exploded.py 给的拆开方向）。

打开 .blend 以后：
  - 两个集合：“外观（效果图）”= 装好的样子；“拆开看（外壳 + 骨架 + 电机）”= 工程件，默认藏着。在右上角大纲里点眼睛切换。
  - 拆开：选中“拆散控制”（机身上方的小球），按 N → 物体（Item）→ 属性里拖“拆散”（0 = 装好，1 = 全拆开）；时间轴按空格会自动拆开再装回。
  - 配色 B：外壳浅灰白、骨架深灰、青色灯。
"""
import json
import math
import os
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
OUT_BLEND = os.path.abspath(argv[0])
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.unit_settings.system = "METRIC"


def srgb(h):
    f = lambda x: x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4
    return tuple(f(int(h[i:i + 2], 16) / 255) for i in (1, 3, 5)) + (1.0,)


def principled(name, col, rough, metal=0.0, emit=None, strength=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = col
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    if emit:
        b.inputs["Emission Color"].default_value = emit
        b.inputs["Emission Strength"].default_value = strength
    m.diffuse_color = col                                              # 实体模式下也是这个颜色
    return m


MAT = dict(shell=principled("外壳·喷涂", srgb("#d9dbdd"), 0.42),
           skel=principled("骨架·阳极氧化", srgb("#3d4247"), 0.38, 0.35),
           skel_dk=principled("深灰", srgb("#25282c"), 0.45, 0.2),
           metal=principled("拉丝金属", srgb("#a3a9af"), 0.28, 1.0),
           rubber=principled("橡胶", srgb("#141516"), 0.85),
           glass=principled("烟黑玻璃", srgb("#0a0d10"), 0.05),
           cyan=principled("青色光", (0.0, 0.6, 1.0, 1), 0.3, emit=(0.0, 0.75, 1.0, 1), strength=4.0),
           eye=principled("眼睛", (0.3, 0.7, 1.0, 1), 0.3, emit=(0.47, 0.78, 1.0, 1), strength=3.0),
           red=principled("尾灯", (1.0, 0.05, 0.03, 1), 0.3, emit=(1.0, 0.04, 0.02, 1), strength=5.0),
           estop=principled("急停红", srgb("#c8102e"), 0.3),
           lamp=principled("灯", (0.95, 0.97, 1.0, 1), 0.15, emit=(1.0, 0.97, 0.9, 1), strength=2.5))


def collection(name):
    c = bpy.data.collections.new(name)
    scene.collection.children.link(c)
    return c


def load(path, name, color, col):
    bpy.ops.wm.stl_import(filepath=path)
    o = bpy.context.selected_objects[0]
    o.name = o.data.name = name
    o.scale = (0.001, 0.001, 0.001)
    o.rotation_euler = (0, 0, math.radians(90))                       # 机身 x 前 → Blender +Y 前（跟效果图一样）
    o.data.materials.clear()
    o.data.materials.append(MAT.get(color, MAT["skel"]))
    for c in list(o.users_collection):
        c.objects.unlink(o)
    col.objects.link(o)
    return o


# ---- 外观（装好的样子）
C_APP = collection("外观（效果图）")
src = os.path.join(HERE, "out", "parts")
for it in json.load(open(os.path.join(src, "manifest.json"), encoding="utf-8")):
    load(os.path.join(src, it["file"]), it["name"], it["color"], C_APP)

# ---- 拆开看：外壳 + 骨架 + 电子 + 腿模块，拆散控制
C_EXP = collection("拆开看（外壳 + 骨架 + 电机）")
ctrl = bpy.data.objects.new("拆散控制", None)
ctrl.empty_display_type = "SPHERE"
ctrl.empty_display_size = 0.04
ctrl.location = (0, 0, 0.75)
C_EXP.objects.link(ctrl)
ctrl["拆散"] = 0.0
ctrl.id_properties_ui("拆散").update(min=0.0, max=1.0, soft_min=0.0, soft_max=1.0, description="0 = 装好，1 = 全部拆开")
d = os.path.join(HERE, "out", "exploded", "whole")
for k, it in enumerate(json.load(open(os.path.join(d, "manifest.json"), encoding="utf-8"))):
    name = it.get("label") or os.path.splitext(os.path.basename(it["file"]))[0]
    o = load(os.path.normpath(os.path.join(d, it["file"])), f"{k:03d} {name}", it["color"], C_EXP)
    ox, oy, oz = [v / 1000 for v in it["offset"]]
    for i, v in enumerate((-oy, ox, oz)):                              # 跟上面的 90° 转一致
        if abs(v) < 1e-6:
            continue
        fc = o.driver_add("delta_location", i)
        fc.driver.type = "SCRIPTED"
        var = fc.driver.variables.new()
        var.name = "e"
        var.targets[0].id_type = "OBJECT"
        var.targets[0].id = ctrl
        var.targets[0].data_path = '["拆散"]'
        fc.driver.expression = f"e*{v:.5f}"
for vl in scene.view_layers:                                           # 默认藏着，只看外观
    vl.layer_collection.children[C_EXP.name].hide_viewport = True
C_EXP.hide_render = True

# ---- 拆散动画：0–60 装好 → 60–120 拆开 → 120–180 停 → 180–240 装回
scene.frame_start, scene.frame_end = 0, 240
for f, v in ((0, 0.0), (60, 0.0), (120, 1.0), (180, 1.0), (240, 0.0)):
    ctrl["拆散"] = v
    ctrl.keyframe_insert('["拆散"]', frame=f)
ctrl["拆散"] = 0.0

# ---- 地面、灯、相机
C_ENV = collection("环境（地面、灯、相机）")
bpy.ops.mesh.primitive_plane_add(size=30, location=(0, 0, 0))
floor = bpy.context.object
floor.name = "地面"
floor.data.materials.append(principled("地面", srgb("#8d9196"), 0.6))
for c in list(floor.users_collection):
    c.objects.unlink(floor)
C_ENV.objects.link(floor)
world = bpy.data.worlds.new("背景")
scene.world = world
world.use_nodes = True
world.node_tree.nodes["Background"].inputs[0].default_value = (0.82, 0.84, 0.87, 1)
world.node_tree.nodes["Background"].inputs[1].default_value = 0.35
ctr = Vector((0, 0, 0.22))
for nm, kind, e, dvec, loc, size in (("主光", "SUN", 3.2, (-0.5, -0.4, -0.75), None, 1),
                                      ("补光", "AREA", 260, (0.7, 0.5, -0.3), (-1.6, -1.2, 1.0), 2.5),
                                      ("轮廓光", "AREA", 180, (0.0, -0.8, -0.3), (0.0, 1.8, 1.2), 2.0)):
    L = bpy.data.objects.new(nm, bpy.data.lights.new(nm, kind))
    L.data.energy = e
    if kind == "AREA":
        L.data.size = size
    L.rotation_euler = Vector(dvec).normalized().to_track_quat("-Z", "Y").to_euler()
    L.location = Vector(loc) if loc else ctr - Vector(dvec).normalized() * 3
    C_ENV.objects.link(L)
cam = bpy.data.objects.new("相机（前 45°）", bpy.data.cameras.new("相机"))
cam.data.lens = 60
dv = Vector((1.35, 1.35, 0.55)).normalized()
cam.location = ctr + dv * 1.75
cam.rotation_euler = (ctr - cam.location).to_track_quat("-Z", "Y").to_euler()
C_ENV.objects.link(cam)
scene.camera = cam
scene.render.engine = "CYCLES"
scene.cycles.samples = 96
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = 1800, 1100
scene.view_settings.view_transform = "AgX"
os.makedirs(os.path.dirname(OUT_BLEND), exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=OUT_BLEND, compress=True)
print("存好：", OUT_BLEND)
