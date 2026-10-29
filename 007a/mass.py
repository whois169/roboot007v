"""007A 重量和重心：骨架按体积 × 密度算，买来的零件按官方重量（没有的按估计，标出来）。

用法：python 007a/mass.py → 打印分项、合计、重心，并和 sizing.py 的估算比
"""
import json
import math
import os

import eng_body as B
import eng_leg as E
import model as M

RHO = dict(铝=2.70e-6, 钢=7.85e-6, 亚克力=1.19e-6, 打印=1.27e-6 * 0.45)   # kg/mm³；打印件按 PETG、填充约 45%

# 买来的零件：(名字, kg, x, y, z（离地）, 来源)
BOUGHT = [
    ("电池 6S3P P42A（18 节 × 70 g + 保护板 + 线）", 1.40, 0.0, 0.0, M.Z0 + 45, "估"),
    ("树莓派 5 + 散热", 0.06, 0.0, 67.0, B.TRAY_Z + 15, "官方约 46 g + 散热"),
    ("USB 转 4 路 485、集线器、降压×3、电平转换", 0.15, 0.0, -40.0, B.TRAY_Z + 12, "估"),
    ("IMU HI13", 0.011, 0.0, 0.0, B.TRAY_Z + 10, "官网 < 11 g"),
    ("继电器、保险丝盒、分流器、线束", 0.35, 0.0, 0.0, M.Z0 + 30, "估"),
    ("深度相机 Gemini 335", 0.097, M.L2 + 15, 0.0, M.Z0 + 33, "官网 97 g"),
    ("热像仪 P2 Pro", 0.04, M.L2 + 10, -101.0, M.Z0 + 32, "估"),
    ("探照灯 + 驱动", 0.06, M.L2 + 10, 101.0, M.Z0 + 32, "估"),
    ("激光雷达 LD19", 0.047, M.L2 - 46, -80.0, M.TOP + 20, "007v 资料"),
    ("气体检测 ZCE04B", 0.06, -M.L2 + 10, -94.0, M.Z0 + 90, "估"),
    ("屏幕板 4.3B + 抬头舵机 STS3215", 0.16, M.L2 + 15, 0.0, M.Z0 + 93, "估（舵机约 55 g）"),
    ("外壳（打印，上层 + 下层 + 腰 + 头 + 防撞杠）", 1.30, 0.0, 0.0, M.Z0 + 85, "估：按 2 mm 壁"),
    ("喇叭×2、功放、麦克风、急停、开关、充电口", 0.15, 0.0, 0.0, M.Z0 + 90, "估"),
]
MOTOR_KG = 0.53                                     # 宇树 GO-M8010-6 官方约 530 g


def solid_mass(s, rho):
    try:
        v = s.Volume()
        c = s.Center()
        return v * rho, (c.x, c.y, c.z)
    except Exception:
        return 0.0, (0, 0, 0)


# 减重目标（下一轮出图时照这个做减重孔 / 空心壳）：实心算出来的重量 × 系数
LIGHTEN = [("带轮", 0.34), ("大腿外层壳", 0.25), ("小腿", 0.6), ("大腿杯", 0.6), ("大腿外板", 0.6), ("髋座", 0.7)]   # 骨架铝板的减重孔已经画进去了（eng_body.finish），按实际算


def lighten(name):
    for k, f in LIGHTEN:
        if k in name:
            return f                                                # 带轮：钢 → 铝（2.7 / 7.85）
    return 1.0


def material(name):
    if "（铝" in name or "铝，CNC" in name or "铝 4" in name or "角码" in name:
        return "铝"
    if "钢" in name or "轴" in name or "带轮" in name or "轴承" in name or "立柱" in name:
        return "钢"
    if "亚克力" in name:
        return "亚克力"
    return "打印"


if __name__ == "__main__":
    items = []
    envs = [B.sweep_envelope(fx, sy) for fx in (1, -1) for sy in (1, -1)]
    for n, s, c in B.frame_parts(envs):
        m, p = solid_mass(s, RHO[material(n)])
        items.append(("骨架 " + n, m, p))
    for n, s, c in B.inside_parts():
        if "电池" in n and "托" not in n:
            continue                                                 # 电池在 BOUGHT 里
        if n in [b[0] for b in B.BOARDS] or any(n == sp[0] for sp in B.SIDE_PARTS):
            continue
        m, p = solid_mass(s, RHO[material(n)])
        items.append((n, m, p))
    legs_kg = 0.0
    for fx in (1, -1):
        for sy in (1, -1):
            for n, s, c, g in B.leg_parts_world(fx, sy):
                if "电机" in n and "座" not in n:
                    bb = s.BoundingBox()
                    items.append((f"腿{fx}{sy} {n}（宇树 530 g）", MOTOR_KG, (bb.center.x, bb.center.y, bb.center.z)))
                    legs_kg += MOTOR_KG
                    continue
                if "皮带" in n or "同步带" in n:
                    m, p = solid_mass(s, 1.2e-6)
                else:
                    m, p = solid_mass(s, RHO[material(n)] if "小腿" not in n else RHO["铝"] * 0.5)
                items.append((f"腿{fx}{sy} {n}", m, p))
                legs_kg += m
    for n, m, x, y, z, src in BOUGHT:
        items.append((n + f"（{src}）", m, (x, y, z)))
    solid = sum(m for _, m, _ in items)
    items = [(n, m * lighten(n), p) for n, m, p in items]
    tot = sum(m for _, m, _ in items)
    cx = sum(m * p[0] for _, m, p in items) / tot
    cy = sum(m * p[1] for _, m, p in items) / tot
    cz = sum(m * p[2] for _, m, p in items) / tot
    groups = {}
    for n, m, p in items:
        k = "腿（含 12 个电机）" if n.startswith("腿") else ("骨架铝板" if n.startswith("骨架") else n.split("（")[0])
        groups[k] = groups.get(k, 0) + m
    print("分项（kg）：")
    for k, v in sorted(groups.items(), key=lambda kv: -kv[1]):
        print(f"  {k:<36}{v:6.2f}")
    print(f"\n现在图上全按实心算：{solid:.2f} kg；按减重目标：{tot:.2f} kg（sizing.py 估的 14.4 kg）")
    print(f"重心：x {cx:+.0f}（正 = 偏前），y {cy:+.0f}，离地 {cz:.0f}（髋高 {M.HIP_Z:.0f}）")
    # 给仿真（sim/export_mujoco.py）：机身（不含 4 条腿）的重量和重心，腿在仿真里按零件逐个算
    body = [(m, p) for n, m, p in items if not n.startswith("腿")]
    bm = sum(m for m, _ in body)
    bc = [sum(m * p[i] for m, p in body) / bm for i in range(3)]
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "mass.json")
    json.dump(dict(total=tot, com=[cx, cy, cz], body=bm, body_com=bc, legs=tot - bm), open(out, "w", encoding="utf-8"), indent=1)
    print("→", out)
