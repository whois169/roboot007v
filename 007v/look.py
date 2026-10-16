"""零件颜色、材质和坐标换算：3D 查看页（build.py）、SolidWorks 导出、Blender 导出共用一套。"""

COL = dict(armor="#4b5159", alu="#b9bfc7", profile="#9ea5ad", black="#1b1d20", motor="#2c3035", steel="#d3d7db",
           trim="#c8cdd3", pcb="#1f6b48", battery="#2f5c9e", printed="#6e747b", glass="#0c1a22", wheel="#26292d",
           shell="#353b40", deck="#4a5258", dark="#1f2327", silver="#cdd2d6", gloss="#06080a", seam="#050607",
           amber="#ffb000", cyan="#00e5ff", anod="#3b4147")   # anod = 深灰阳极氧化铝
EMISSIVE = dict(glass=(0.0, 0.35, 0.45), amber=(1.0, 0.63, 0.0), cyan=(0.0, 0.85, 1.0))
METAL = ("alu", "steel", "trim", "profile", "silver", "anod")


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
