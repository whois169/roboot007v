"""配色渲染：打开 make_blend.py 做好的 .blend，只换材质和灯光（不动任何零件），出正面、侧面、前 45°；可加一张夜间效果图。

两套配色（全车不用金色；外壳都比骨架浅）：
- A 深色版：外壳石墨灰，骨架 / 连杆 / 电机近黑色
- B 浅色版：外壳浅灰白，骨架 / 连杆 / 电机深灰
材质：外壳哑光喷涂带细颗粒；骨架、连杆、电机护罩深色阳极氧化（轻微金属光泽）；螺栓、轴承座、轴毂拉丝金属；
      麦轮滚子、线缆黑色橡胶；屏幕、传感器窗口烟黑玻璃（有反光）
原来的琥珀色件：改成比所在部件深一级的灰色（看起来像一道凹槽线）；有功能的灯按下表点亮
灯（都有用处）：
- 表情屏：青色眼睛（交互）
- 头部转轴、麦轮轮毂盖、电机端面：青色光环（看得出哪个关节 / 轮子在动、是否通电）
- 机身两侧腰线：青色状态灯带（电量 / 状态，侧面的人也看得到）
- 前保险杠两个方框：白色照明灯（给深度相机补光、夜里照路）
- 尾部：两盏红色尾灯（后面的人看得到它、刹车 / 倒车提示）
- 原来的底光、前后腰线青灯、隐私灯：不点亮（不在这次的灯光清单里）
渲染设置和白模一样：单侧主光 + 很弱的补光，Cycles 光线追踪（缝里有环境光遮蔽）

用法：blender --background --factory-startup --python 007v/blender/render_color.py -- <机械狗007v.blend> <输出目录> <A|B> [只出哪几张，逗号分开] [night]
"""
import math
import os
import sys

import bpy
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:]
BLEND, OUT, SCHEME = os.path.abspath(argv[0]), os.path.abspath(argv[1]), argv[2].upper()
ONLY = argv[3].split(",") if len(argv) > 3 and argv[3] not in ("", "all") else None
NIGHT = len(argv) > 4 and argv[4] == "night"
bpy.ops.wm.open_mainfile(filepath=BLEND)
scene = bpy.context.scene


def srgb(h):
    """#rrggbb -> 线性 RGBA。"""
    def f(x):
        x /= 255
        return x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4
    return tuple(f(int(h[i:i + 2], 16)) for i in (1, 3, 5)) + (1.0,)


# 外壳 / 外壳深一级 / 骨架 / 骨架深一级（琥珀件改灰用）
PAL = dict(A=dict(shell="#5c6268", shell_dk="#43484d", skel="#1b1d20", skel_dk="#0e0f11"),
           B=dict(shell="#d9dbdd", shell_dk="#a6aaae", skel="#3d4247", skel_dk="#25282c"))[SCHEME]

SHELL_KEYS = ("shell", "deck", "armor", "printed")
SKEL_KEYS = ("anod", "dark", "motor", "profile", "pcb", "battery")
METAL_KEYS = ("alu", "steel", "trim", "silver")
RUBBER_KEYS = ("wheel", "black")
GLASS_KEYS = ("glass", "gloss", "visor")


def principled(name, color, rough, metal=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = color
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    return m, b


def paint(name, hexc):
    """哑光喷涂，表面带很细的颗粒（噪声 → 轻微凹凸 + 一点点明暗）。"""
    m, b = principled(name, srgb(hexc), 0.72)
    nt = m.node_tree
    tc = nt.nodes.new("ShaderNodeTexCoord")
    nz = nt.nodes.new("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = 900.0                 # 物体坐标是米：约 1 mm 一颗
    nz.inputs["Detail"].default_value = 2.0
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.06
    bump.inputs["Distance"].default_value = 0.0003
    nt.links.new(tc.outputs["Object"], nz.inputs["Vector"])
    nt.links.new(nz.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], b.inputs["Normal"])
    rr = nt.nodes.new("ShaderNodeMapRange")                  # 粗糙度 0.66–0.78 之间轻微变化
    rr.inputs["To Min"].default_value, rr.inputs["To Max"].default_value = 0.66, 0.78
    nt.links.new(nz.outputs["Fac"], rr.inputs["Value"])
    nt.links.new(rr.outputs["Result"], b.inputs["Roughness"])
    return m


def anodized(name, hexc):
    """深色阳极氧化铝：半金属、中等粗糙，轻微光泽。"""
    m, b = principled(name, srgb(hexc), 0.42, 0.65)
    return m


def brushed(name):
    """拉丝金属：全金属，各向异性 + 沿一个方向拉长的细纹。"""
    m, b = principled(name, srgb("#9aa0a6"), 0.3, 1.0)
    try:
        b.inputs["Anisotropic"].default_value = 0.6
    except KeyError:
        pass
    nt = m.node_tree
    tc = nt.nodes.new("ShaderNodeTexCoord")
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (1.0, 1.0, 60.0)      # 拉成细纹
    nz = nt.nodes.new("ShaderNodeTexNoise")
    nz.inputs["Scale"].default_value = 400.0
    rr = nt.nodes.new("ShaderNodeMapRange")
    rr.inputs["To Min"].default_value, rr.inputs["To Max"].default_value = 0.22, 0.38
    nt.links.new(tc.outputs["Object"], mp.inputs["Vector"])
    nt.links.new(mp.outputs["Vector"], nz.inputs["Vector"])
    nt.links.new(nz.outputs["Fac"], rr.inputs["Value"])
    nt.links.new(rr.outputs["Result"], b.inputs["Roughness"])
    return m


def rubber(name):
    m, b = principled(name, srgb("#141516"), 0.88)
    return m


def smoked(name):
    """烟黑玻璃：很暗、很光滑，有清漆层反光。"""
    m, b = principled(name, srgb("#0a0d10"), 0.06)
    for k, v in (("Coat Weight", 1.0), ("Coat Roughness", 0.03), ("Specular IOR Level", 0.7)):
        try:
            b.inputs[k].default_value = v
        except KeyError:
            pass
    return m


def lamp(name, rgb, strength):
    """灯：表面是磨砂灯罩（暗时也看得出颜色），开灯时自发光。"""
    m, b = principled(name, rgb + (1.0,), 0.35)
    b.inputs["Emission Color"].default_value = rgb + (1.0,)
    b.inputs["Emission Strength"].default_value = strength
    return m


K = 1.0                                                      # 夜里灯不调亮，靠关掉环境光
M = dict(shell=paint("外壳·喷涂", PAL["shell"]), shell_dk=paint("外壳·深一级", PAL["shell_dk"]),
         skel=anodized("骨架·阳极氧化", PAL["skel"]), skel_dk=paint("骨架·深一级（凹槽线）", PAL["skel_dk"]),
         metal=brushed("拉丝金属"), rubber=rubber("橡胶"), glass=smoked("烟黑玻璃"),
         seam=principled("分缝", srgb("#050607"), 0.9)[0],
         cyan=lamp("青色光环", (0.0, 0.75, 1.0), 3.0 * K), band=lamp("青色腰线灯带", (0.0, 0.75, 1.0), 3.5 * K),
         white=lamp("白色照明灯", (1.0, 0.97, 0.92), 9.0 * K), red=lamp("红色尾灯", (1.0, 0.03, 0.02), 5.0 * K),
         estop=principled("急停红", srgb("#c8102e"), 0.3)[0],
         floor=principled("地面", srgb("#8d9196") if not NIGHT else srgb("#5d6166"), 0.6)[0])

# 琥珀件在哪个部件上 → 改成那个部件深一级的灰
AMBER_TO = [("臂座黄箍", "shell_dk"), ("栏板反光条", None), ("摆臂黄条", "skel_dk"), ("摆臂边黄线", "skel_dk"),
            ("髋盖黄圈", "skel_dk"), ("隐私指示灯", "glass")]
NAME_RULES = [("屏幕玻璃", "hud"), ("腰线端头", "skel"), ("腰线灯", "band"), ("照明灯", "white"), ("尾灯", "red"),
              ("底部灯槽", "skel"), ("急停蘑菇头", "estop")]


def key_of(o):
    ks = [ms.material.name.split(".")[0] for ms in o.material_slots if ms.material]
    return ks[0] if ks else ""


fence_key = next((key_of(o) for o in bpy.data.objects if o.type == "MESH" and "栏板" in o.name and "压条" not in o.name), "shell")
fence_dk = "shell_dk" if fence_key in SHELL_KEYS else "skel_dk"


def pick(o):
    k = key_of(o)
    for sub, tgt in NAME_RULES:
        if sub in o.name:
            return tgt
    if k == "amber":
        for sub, tgt in AMBER_TO:
            if sub in o.name:
                return tgt or fence_dk
        print("琥珀件没归类，按外壳深一级：", o.name)
        return "shell_dk"
    if k == "cyan":
        return "cyan"
    if k in ("shell_dk", "skel_dk"):
        return k
    for keys, tgt in ((SHELL_KEYS, "shell"), (SKEL_KEYS, "skel"), (METAL_KEYS, "metal"), (RUBBER_KEYS, "rubber"),
                      (GLASS_KEYS, "glass"), (("seam",), "seam")):
        if k in keys:
            return tgt
    return "skel"


# 屏幕：保留 HUD 眼睛的自发光，叠一层烟黑玻璃反光
hud = None
glass_obj = bpy.data.objects.get("屏幕玻璃")
if glass_obj and glass_obj.material_slots and glass_obj.material_slots[0].material:
    hud = glass_obj.material_slots[0].material
    nt = hud.node_tree
    em = next(n for n in nt.nodes if n.type == "EMISSION")
    out = next(n for n in nt.nodes if n.type == "OUTPUT_MATERIAL")
    em.inputs["Strength"].default_value = 1.8
    gl = nt.nodes.new("ShaderNodeBsdfGlossy")
    gl.inputs["Color"].default_value = (0.05, 0.05, 0.06, 1)
    gl.inputs["Roughness"].default_value = 0.04
    add = nt.nodes.new("ShaderNodeAddShader")
    nt.links.new(em.outputs[0], add.inputs[0])
    nt.links.new(gl.outputs[0], add.inputs[1])
    nt.links.new(add.outputs[0], out.inputs["Surface"])

stat = {}
for o in bpy.data.objects:
    if o.type != "MESH":
        continue
    tgt = "floor" if o.name == "地面" else pick(o)
    if tgt == "hud":
        continue
    stat[tgt] = stat.get(tgt, 0) + 1
    o.data.materials.clear()
    o.data.materials.append(M[tgt])
print("材质分配：", stat)

# ---- 灯：影棚灯关掉，主光 + 补光和白模一样；夜里只留一点点月光
for o in bpy.data.objects:
    if o.type == "LIGHT":
        o.hide_render = True
world = scene.world or bpy.data.worlds.new("配色")
scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes.get("Background")
bg.inputs["Color"].default_value = (0.85, 0.87, 0.9, 1) if not NIGHT else (0.3, 0.4, 0.6, 1)
bg.inputs["Strength"].default_value = 0.12 if not NIGHT else 0.004


def add_light(name, kind, energy, direction, loc=None, **kw):
    L = bpy.data.objects.new(name, bpy.data.lights.new(name, kind))
    L.data.energy = energy
    for k, v in kw.items():
        setattr(L.data, k, v)
    d = Vector(direction).normalized()
    L.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    L.location = loc if loc is not None else -d * 3
    scene.collection.objects.link(L)
    return L


if not NIGHT:
    add_light("主光", "SUN", 4.5, (-0.5, 0.5, -0.71), angle=math.radians(2.0))
    add_light("补光", "SUN", 0.6, (0.6, -0.2, -0.45), angle=math.radians(15))
else:
    add_light("月光", "SUN", 0.05, (-0.5, 0.5, -0.71), angle=math.radians(2.0), color=(0.7, 0.8, 1.0))

# ---- Cycles，显卡优先（同白模）
scene.render.engine = "CYCLES"
try:
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "CUDA"
    prefs.get_devices()
    if any(d.type == "CUDA" for d in prefs.devices):
        for d in prefs.devices:
            d.use = d.type in ("CUDA", "CPU")
        scene.cycles.device = "GPU"
        print("用显卡渲染：CUDA")
except Exception as e:
    print("没有显卡渲染，用 CPU：", e)
scene.cycles.samples = 128 if not NIGHT else 256
scene.cycles.use_denoising = True
scene.render.resolution_x, scene.render.resolution_y = 1800, 1150
scene.view_settings.view_transform = "Standard"
scene.view_settings.look = "None"

cam = scene.camera
aim = next((c.target for c in cam.constraints if hasattr(c, "target") and c.target), None)
cam.data.lens = 70
scene.frame_set(5)

if NIGHT:                                                    # 白灯真的往前照路：在两盏灯的位置放两盏聚光灯
    for o in bpy.data.objects:
        if "照明灯" in o.name and o.type == "MESH":
            c = o.matrix_world @ (sum((Vector(v) for v in o.bound_box), Vector()) / 8)
            add_light(o.name + "·光束", "SPOT", 2.5, (0, -1, -0.75), loc=c + Vector((0, -0.01, 0)),
                      spot_size=math.radians(55), spot_blend=0.7, shadow_soft_size=0.01, color=(1.0, 0.97, 0.92))
        if "尾灯" in o.name and o.type == "MESH":
            c = o.matrix_world @ (sum((Vector(v) for v in o.bound_box), Vector()) / 8)
            add_light(o.name + "·红光", "SPOT", 0.6, (0, 1, -0.4), loc=c + Vector((0, 0.01, 0)),
                      spot_size=math.radians(100), spot_blend=0.8, shadow_soft_size=0.01, color=(1.0, 0.05, 0.03))

os.makedirs(OUT, exist_ok=True)
VIEWS = [("正面", (0.0, -1.85, 0.22), None), ("侧面", (1.85, -0.05, 0.2), None), ("前45度", (1.25, -1.3, 0.42), None),
         ("后45度", (-1.25, 1.3, 0.42), None)]
tag = f"方案{SCHEME}" + ("_夜间" if NIGHT else "")
for nm, loc, look_at in VIEWS:
    if ONLY and nm not in ONLY:
        continue
    cam.location = loc
    if aim:
        aim.location = look_at or (0, -0.02, 0.09)
    scene.render.filepath = os.path.join(OUT, f"{tag}_{nm}.png")
    try:
        bpy.ops.render.render(write_still=True)
    except RuntimeError as e:
        print("显卡渲染失败，改用 CPU：", e)
        scene.cycles.device = "CPU"
        bpy.ops.render.render(write_still=True)
print("配色渲染完成：", OUT)
