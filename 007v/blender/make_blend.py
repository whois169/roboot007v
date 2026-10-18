"""在 Blender 里搭好机械狗007v 的场景（Blender 5.x）：零件按运动部件挂好、转轴放对、带动画、能一键拆散，存成 .blend。

用法（不用打开 Blender 界面）：
  blender --background --factory-startup --python 007v/blender/make_blend.py -- <数据目录> <输出.blend> [预览图目录]
数据目录里要有 export_parts.py 导出的 parts.glb 和 scene.json。

打开 .blend 以后：
  - 空格键播放：展开 → 折叠 → 展开 → 抬头低头 → 拆散 → 装回 → 8 段仿真回放（时间轴上有标记，点标记就跳过去）
  - 拆散：选中“拆散控制”，按 N 打开侧栏 → 物体（Item）→ 属性里拖“拆散”（0 = 装好，1 = 全拆开）；
    时间轴上“拆散”那几段已经录好了，想自己拖就先到第一段“展开站着”
  - 转关节：选中“前左摆臂”之类的空物体，按 R X 输入角度
"""
import json
import math
import os
import sys

import bpy
from bpy_extras import anim_utils

argv = sys.argv[sys.argv.index("--") + 1:]
DATA, OUT_BLEND = argv[0], argv[1]
PREVIEW = argv[2] if len(argv) > 2 else None
D = json.load(open(os.path.join(DATA, "scene.json"), encoding="utf-8"))

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.fps = D["fps"]
scene.frame_start, scene.frame_end = 0, D["frames"] - 1
scene.unit_settings.system = "METRIC"


def collection(name, parent=None):
    c = bpy.data.collections.new(name)
    (parent or scene.collection).children.link(c)
    return c


C_ENV = collection("环境（地面、灯、相机）")
C_RIG = collection("关节（空物体，转它们零件就跟着动）")
C_PARTS = {k: collection(n) for k, n in (("body", "机身零件"), ("head", "屏幕头零件"), ("arm", "摆臂零件"), ("wheel", "麦轮零件"))}


def empty(name, parent=None, loc=(0, 0, 0), size=0.04, kind="PLAIN_AXES", col=None):
    o = bpy.data.objects.new(name, None)
    o.empty_display_type = kind
    o.empty_display_size = size
    (col or C_RIG).objects.link(o)
    if parent is not None:
        o.parent = parent
    o.location = loc
    return o


# ---------------------------------------------------------------- 关节层级
world = empty("世界（坡道时整体转）", size=0.1)
base = empty("机身（整体位置和姿态）", world, size=0.12, kind="ARROWS")
base.rotation_mode = "QUATERNION"
G = {"body": base}
for g in D["groups"]:
    if g["key"] == "body":
        continue
    G[g["key"]] = empty(g["name"] + "（转轴）", G[g["parent"]], g["loc"], 0.05, "CIRCLE" if g["key"] != "head" else "SINGLE_ARROW")
    if g["key"] != "head":
        G[g["key"]].rotation_euler = (0, 0, 0)
ctrl = empty("拆散控制", None, (0, 0, 0.75), 0.06, "SPHERE")
ctrl["拆散"] = 0.0
ctrl.id_properties_ui("拆散").update(min=0.0, max=1.0, soft_min=0.0, soft_max=1.0, description="0 = 装好，1 = 全部拆开")


def explode_driver(obj, vec):
    for i in range(3):
        if abs(vec[i]) < 1e-5:
            continue
        fc = obj.driver_add("delta_location", i)
        d = fc.driver
        d.type = "SCRIPTED"
        v = d.variables.new()
        v.name = "e"
        v.type = "SINGLE_PROP"
        v.targets[0].id_type = "OBJECT"
        v.targets[0].id = ctrl
        v.targets[0].data_path = '["拆散"]'
        d.expression = f"e*{vec[i]:.5f}"


for g in D["groups"]:
    if g["key"] != "body":
        explode_driver(G[g["key"]], g["explode"])

# ---------------------------------------------------------------- 导入零件，挂到各自的关节下面
before = set(bpy.data.objects)
bpy.ops.import_scene.gltf(filepath=os.path.join(DATA, "parts.glb"))
new = {o.name: o for o in bpy.data.objects if o not in before}
for o in list(new.values()):
    if o.type != "MESH" and not o.children:
        bpy.data.objects.remove(o)
by_node = {}
for o in bpy.data.objects:
    if o.type == "MESH":
        by_node[o.name.split(".")[0]] = o
missing = 0
for p in D["parts"]:
    o = by_node.get(p["node"])
    if o is None:
        missing += 1
        continue
    mw = o.matrix_world.copy()
    o.parent = G[p["group"]]
    o.matrix_parent_inverse.identity()
    o.matrix_basis = mw                        # 导入时零件就在自己关节的局部坐标里
    o.name = p["name"]
    o.data.name = p["name"]
    key = "body" if p["group"] == "body" else "head" if p["group"] == "head" else "wheel" if p["group"].endswith("w") else "arm"
    for c in list(o.users_collection):
        c.objects.unlink(o)
    C_PARTS[key].objects.link(o)
    explode_driver(o, p["explode"])
for o in [o for o in bpy.data.objects if o.type == "EMPTY" and o.name not in {x.name for x in C_RIG.objects} and o not in (ctrl,)]:
    if not o.children:
        bpy.data.objects.remove(o)

# ---------------------------------------------------------------- 地面、货、障碍物、灯、相机
def material(name, rgb, alpha=1.0, rough=0.8, metal=0.0, emit=None):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (*rgb, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    if alpha < 1:
        b.inputs["Alpha"].default_value = alpha
        m.surface_render_method = "BLENDED" if hasattr(m, "surface_render_method") else m.surface_render_method
    m.diffuse_color = (*rgb, alpha)
    return m


bpy.ops.mesh.primitive_plane_add(size=60)
ground = bpy.context.active_object
ground.name = "地面"
gm = bpy.data.materials.new("地面格子")
gm.use_nodes = True
nt = gm.node_tree
chk = nt.nodes.new("ShaderNodeTexChecker")
chk.inputs["Scale"].default_value = 300
chk.inputs["Color1"].default_value = (0.012, 0.015, 0.017, 1)
chk.inputs["Color2"].default_value = (0.02, 0.025, 0.028, 1)
nt.links.new(chk.outputs["Color"], nt.nodes["Principled BSDF"].inputs["Base Color"])
gm.diffuse_color = (0.04, 0.05, 0.06, 1)
ground.data.materials.append(gm)
for c in list(ground.users_collection):
    c.objects.unlink(ground)
C_ENV.objects.link(ground)
ground.parent = world

bpy.ops.mesh.primitive_cube_add(size=1)
cargo = bpy.context.active_object
cargo.name = "货物 10 kg"
cargo.scale = D["cargo"]["size"]
cargo.parent = base
cargo.location = D["cargo"]["loc"]
cargo.data.materials.append(material("货物", (0.9, 0.62, 0.22), 0.35))
cargo.display_type = "WIRE"
for c in list(cargo.users_collection):
    c.objects.unlink(cargo)
C_ENV.objects.link(cargo)

bumps = []
for i, ob in enumerate(D["obstacles"]):
    bpy.ops.mesh.primitive_cube_add(size=1)
    b = bpy.context.active_object
    b.name = f"门槛条 15 mm（{i + 1}）"
    b.scale = ob["size"]
    b.location = ob["loc"]
    b.parent = world
    b.data.materials.append(material("门槛", (0.9, 0.62, 0.22), rough=0.5))
    for c in list(b.users_collection):
        c.objects.unlink(b)
    C_ENV.objects.link(b)
    bumps.append(b)

sun = bpy.data.objects.new("太阳光", bpy.data.lights.new("太阳光", "SUN"))
sun.data.energy = 1.2
sun.rotation_euler = (math.radians(50), math.radians(10), math.radians(30))
C_ENV.objects.link(sun)
fill = bpy.data.objects.new("补光", bpy.data.lights.new("补光", "AREA"))
fill.data.energy = 120
fill.data.size = 2
fill.location = (-1.5, -1.5, 1.6)
fill.rotation_euler = (math.radians(55), 0, math.radians(-45))
C_ENV.objects.link(fill)
scene.world = bpy.data.worlds.new("背景")
scene.world.color = (0.02, 0.026, 0.03)
scene.world.use_nodes = True
scene.world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.02, 0.026, 0.03, 1)

follow = empty("相机跟随点", None, (0, 0, 0), 0.03, col=C_ENV)
cl = follow.constraints.new("COPY_LOCATION")
cl.target = base
cam = bpy.data.objects.new("相机", bpy.data.cameras.new("相机"))
cam.data.lens = 40
cam.parent = follow
cam.location = (0.86, -1.1, 0.42)
tt = cam.constraints.new("TRACK_TO")
tt.target = follow
tt.track_axis = "TRACK_NEGATIVE_Z"
tt.up_axis = "UP_Y"
C_ENV.objects.link(cam)
scene.camera = cam
aim = empty("相机看的点", follow, (0, 0, 0.06), 0.02, col=C_ENV)
tt.target = aim


# ---------------------------------------------------------------- 动画：一条时间轴
def track(obj, path, index, values, constant=False):
    if obj.animation_data is None:
        obj.animation_data_create()
    ad = obj.animation_data
    if ad.action is None:
        act = bpy.data.actions.new(obj.name + " 动作")
        ad.action = act
        slot = act.slots.new(id_type="OBJECT", name=obj.name)
        ad.action_slot = slot
    cb = anim_utils.action_ensure_channelbag_for_slot(ad.action, ad.action_slot)
    fc = cb.fcurves.new(path, index=index)
    n = len(values)
    fc.keyframe_points.add(n)
    co = [0.0] * (2 * n)
    co[0::2] = [float(i) for i in range(n)]
    co[1::2] = [float(v) for v in values]
    fc.keyframe_points.foreach_set("co", co)
    for kp in fc.keyframe_points:
        kp.interpolation = "CONSTANT" if constant else "LINEAR"
    fc.update()


T = D["tracks"]
track(base, "location", 0, T["bx"])
track(base, "location", 1, T["by"])
track(base, "location", 2, T["bz"])
for i, k in enumerate(("qw", "qx", "qy", "qz")):
    track(base, "rotation_quaternion", i, T[k])
for k, s in D["sign"].items():
    track(G[k], "rotation_euler", 0, [s * v for v in T["h" + k]])
    track(G[k + "w"], "rotation_euler", 0, T["w" + k])
track(G["head"], "rotation_euler", 0, T["head"])
track(world, "rotation_euler", 0, T["slope"], constant=True)
track(ctrl, '["拆散"]', 0, T["explode"])
track(cargo, "hide_viewport", 0, [1 - v for v in T["cargo"]], constant=True)
track(cargo, "hide_render", 0, [1 - v for v in T["cargo"]], constant=True)
for b in bumps:
    track(b, "hide_viewport", 0, [1 - v for v in T["bump"]], constant=True)
    track(b, "hide_render", 0, [1 - v for v in T["bump"]], constant=True)
for f, name in D["markers"]:
    scene.timeline_markers.new(name, frame=f)

# ---------------------------------------------------------------- 视图：材质预览，Z 向上
for scr in bpy.data.screens:
    for area in scr.areas:
        if area.type == "VIEW_3D":
            for sp in area.spaces:
                if sp.type == "VIEW_3D":
                    sp.shading.type = "MATERIAL"
                    sp.clip_end = 200
try:
    scene.render.engine = "BLENDER_EEVEE"
except TypeError:
    pass
scene.render.resolution_x, scene.render.resolution_y = 1280, 720
scene.frame_set(0)
print(f"零件 {len(D['parts']) - missing} 个挂好（缺 {missing}），{D['frames']} 帧，标记 {len(D['markers'])} 个")

# ---------------------------------------------------------------- 预览图（Workbench，快）
# ---------------------------------------------------------------- 栏板单独一个集合，默认隐藏（和效果图一样，要看就在大纲里点眼睛）
C_FENCE = collection("栏板（可拆，默认隐藏）")
for o in list(bpy.data.objects):
    if o.type == "MESH" and ("栏板" in o.name) and "压条" not in o.name or o.name.startswith("外壳·栏板压条"):
        for c in list(o.users_collection):
            c.objects.unlink(o)
        C_FENCE.objects.link(o)
C_FENCE.hide_render = True
for vl in scene.view_layers:
    lc = vl.layer_collection.children.get(C_FENCE.name)
    if lc:
        lc.hide_viewport = True

# ---------------------------------------------------------------- 圆边看起来更顺：按角度平滑着色 + 很细的倒角修改器（只影响显示）
for o in bpy.data.objects:
    if o.type != "MESH" or o.name in ("地面",):
        continue
    try:
        o.data.shade_smooth()
        o.data.set_sharp_from_angle(angle=math.radians(32))
    except Exception:
        pass
    if "滚子" in o.name:
        continue
    m = o.modifiers.new("细倒角", "BEVEL")
    m.width = 0.0008
    m.segments = 2
    m.limit_method = "ANGLE"
    m.angle_limit = math.radians(40)
    m.harden_normals = True

# ---------------------------------------------------------------- 好看一点：屏幕 HUD 眼睛、灯带发光、影棚灯
def hud_image():
    import numpy as np
    W, H = 640, 272
    a = np.zeros((H, W, 4), np.float32)
    a[..., 0], a[..., 1], a[..., 2], a[..., 3] = 0.004, 0.02, 0.03, 1.0
    a[::4, :, 1] += 0.012                                    # 扫描线
    yy, xx = np.mgrid[0:H, 0:W]
    for cx in (W * 0.36, W * 0.64):                          # 两只方眼睛（圆角）
        dx = np.maximum(np.abs(xx - cx) - 40, 0)
        dy = np.maximum(np.abs(yy - H * 0.5) - 40, 0)
        d = np.sqrt(dx * dx + dy * dy)
        m = d < 16
        a[m, 0], a[m, 1], a[m, 2] = 0.05, 0.85, 1.0
        g = (d >= 16) & (d < 40)
        k = (1 - (d[g] - 16) / 24) * 0.25
        a[g, 1] += k * 0.85
        a[g, 2] += k
    a[int(H * 0.88):int(H * 0.9), int(W * 0.1):int(W * 0.5), :3] = (0.05, 0.6, 0.7)   # 底部状态条
    img = bpy.data.images.new("HUD 眼睛", W, H, alpha=False)
    img.pixels.foreach_set(np.flipud(a).ravel())
    img.pack()
    return img


def screen_material():
    img = hud_image()
    m = bpy.data.materials.new("屏幕 HUD")
    nt = m.node_tree if m.node_tree else None
    if nt is None:
        m.use_nodes = True
        nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Strength"].default_value = 2.5
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = img
    tc = nt.nodes.new("ShaderNodeTexCoord")
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    com = nt.nodes.new("ShaderNodeCombineXYZ")
    nt.links.new(tc.outputs["Generated"], sep.inputs[0])
    nt.links.new(sep.outputs["X"], com.inputs["X"])           # 屏幕左右 = Blender X，上下 = Z
    nt.links.new(sep.outputs["Z"], com.inputs["Y"])
    nt.links.new(com.outputs[0], tex.inputs["Vector"])
    nt.links.new(tex.outputs["Color"], em.inputs["Color"])
    nt.links.new(em.outputs[0], out.inputs["Surface"])
    return m


glass = bpy.data.objects.get("屏幕玻璃")
if glass is not None:
    hud = screen_material()
    glass.data.materials.clear()
    glass.data.materials.append(hud)
for m in bpy.data.materials:                                  # 青色光环 / 灯带、白色照明灯、红色尾灯
    if m.name.split(".")[0] in ("cyan", "lamp_w", "lamp_r") and m.node_tree:
        b = m.node_tree.nodes.get("Principled BSDF")
        if b:
            b.inputs["Emission Strength"].default_value = 2.2
for nm, e, loc, rot, size in (("顶光", 60, (0, 0, 2.2), (0, 0, 0), 1.8), ("左前柔光", 45, (1.6, -1.4, 1.0), (55, 0, 50), 1.2),
                              ("右后轮廓光", 90, (-1.4, 1.6, 0.9), (60, 0, -140), 1.0)):
    L = bpy.data.objects.new(nm, bpy.data.lights.new(nm, "AREA"))
    L.data.energy, L.data.size = e, size
    L.location = loc
    L.rotation_euler = tuple(math.radians(v) for v in rot)
    C_ENV.objects.link(L)
L = bpy.data.objects.get("右后轮廓光")
if L:
    L.data.color = (0.6, 0.9, 1.0)

if PREVIEW:
    os.makedirs(PREVIEW, exist_ok=True)
    eng = scene.render.engine
    try:
        scene.render.engine = "BLENDER_EEVEE"
        scene.eevee.taa_render_samples = 64
    except Exception:
        scene.render.engine = "BLENDER_WORKBENCH"
    scene.render.resolution_x, scene.render.resolution_y = 1000, 640
    # 和效果图一样的三个角度
    keep = (cam.location.copy(), aim.location.copy())
    cam.data.lens = 70
    aim.location = (0, -0.02, 0.07)
    for nm, loc in (("00a_前斜", (0.75, -1.55, 0.2)), ("00b_侧面", (1.75, -0.12, 0.17)), ("00c_后斜", (-0.85, 1.6, 0.25))):
        cam.location = loc
        scene.frame_set(5)
        scene.render.filepath = os.path.join(PREVIEW, nm + ".png")
        bpy.ops.render.render(write_still=True)
    cam.location, aim.location = keep
    marks = {name: f for f, name in D["markers"]}
    shots = [("01_展开", 5), ("02_折叠", marks["折叠好了"] + 10), ("03_拆散", marks["拆散（停住看）"] + 30)]
    for f, name in D["markers"]:
        if "抬后左腿" in name and "直接" not in name:
            shots.append(("04_抬腿", f + 150))
        if "10% 坡" in name:
            shots.append(("05_坡", f + 200))
    for nm, f in shots:
        scene.frame_set(int(f))
        scene.render.filepath = os.path.join(PREVIEW, nm + ".png")
        bpy.ops.render.render(write_still=True)
    # 头部特写
    keep = (cam.location.copy(), aim.location.copy())
    cam.location, aim.location = (0.32, -0.5, 0.3), (0, -0.11, 0.2)
    scene.frame_set(5)
    scene.render.filepath = os.path.join(PREVIEW, "06_头部特写.png")
    bpy.ops.render.render(write_still=True)
    cam.location, aim.location = keep
    scene.render.engine = eng
    scene.render.resolution_x, scene.render.resolution_y = 1280, 720
    scene.frame_set(0)

bpy.ops.wm.save_as_mainfile(filepath=OUT_BLEND)
print("存好了：", OUT_BLEND)
