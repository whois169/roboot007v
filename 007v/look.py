"""零件颜色、材质和坐标换算：3D 查看页（build.py）、SolidWorks 导出、Blender 导出共用一套。"""

# 2026-10-18 配色定方案 B（浅色版）：外壳浅灰白，骨架 / 连杆 / 电机深灰，青色灯光；全车不用金色
COL = dict(armor="#d9dbdd", alu="#9aa0a6", profile="#3d4247", black="#141516", motor="#3d4247", steel="#a3a9af",
           trim="#9aa0a6", pcb="#1f6b48", battery="#2f5c9e", printed="#d9dbdd", glass="#0a0d10", wheel="#141516",
           shell="#d9dbdd", deck="#d9dbdd", dark="#3d4247", silver="#a3a9af", gloss="#0a0d10", seam="#050607",
           cyan="#00bfff", anod="#3d4247", visor="#0a0d10",
           shell_dk="#a6aaae", skel_dk="#25282c",             # 比所在部件深一级的灰（原来的金色线条 / 金色件）
           lamp_w="#fff7eb", lamp_r="#ff1a10")                # 白色照明灯、红色尾灯
EMISSIVE = dict(glass=(0.0, 0.35, 0.45), cyan=(0.0, 0.75, 1.0), lamp_w=(1.0, 0.97, 0.92), lamp_r=(1.0, 0.05, 0.03))
METAL = ("alu", "steel", "trim", "silver")                   # 拉丝金属：螺栓、轴承座、轴毂、轮子侧板
ANOD = ("anod", "profile", "dark", "motor")                   # 深色阳极氧化：骨架、连杆、电机护罩（轻微金属光泽）


def lin(v):
    """sRGB 0–255 → 线性 0–255。"""
    x = v / 255
    return round(255 * (x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4))


def rgb(c):
    """颜色名 → (r, g, b) 0–1，sRGB。"""
    h = COL[c]
    return tuple(int(h[i:i + 2], 16) / 255 for i in (1, 3, 5))


def gl(p):
    """机身坐标 (X前 Y左 Z上, mm) -> glTF (x=左, y=上, z=前, m)，是旋转不是镜像。"""
    return [p[1] / 1000.0, p[2] / 1000.0, p[0] / 1000.0]


def blender(p):
    """机身坐标 (X前 Y左 Z上, mm) -> Blender (X=左, Y=后, Z=上, m)：Blender 导入 glTF 后就是这个方向，机器人脸朝 -Y（前视图正对着你）。"""
    return [p[1] / 1000.0, -p[0] / 1000.0, p[2] / 1000.0]
