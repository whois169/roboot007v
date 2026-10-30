"""007A 第 2 步出图：铝板 DXF（激光切割）+ STEP；CNC 件 STEP；打印件 STL；一张总览图（每块板的外形和尺寸）。

和 3D 模型、干涉检查用的是同一份几何（eng_body.py / eng_leg.py / eng_head.py），改尺寸改那边再跑这个。
用法：python 007a/mfg.py → 007a/out/mfg/，docs/机械狗007A/制造件总览.png、制造件清单.md
"""
import json
import os

import cadquery as cq

import eng_body as B
import eng_head as H
import eng_leg as E
from model import V

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "mfg")
DOC = os.path.join(os.path.dirname(HERE), "docs", "机械狗007A")


def flat(s):
    """板件：把最薄的方向转到 z，放到原点。返回 (实体, 厚, 长, 宽)。"""
    bb = s.BoundingBox()
    dims = {"x": bb.xlen, "y": bb.ylen, "z": bb.zlen}
    thin = min(dims, key=dims.get)
    if thin == "x":
        s = s.rotate(V(0, 0, 0), V(0, 1, 0), 90)
    elif thin == "y":
        s = s.rotate(V(0, 0, 0), V(1, 0, 0), 90)
    bb = s.BoundingBox()
    s = s.translate(V(-bb.xmin, -bb.ymin, -bb.zmin))
    bb = s.BoundingBox()
    return s, bb.zlen, bb.xlen, bb.ylen


def export_plate(code, name, s, qty, mat):
    f, t, L, W = flat(s)
    base = os.path.join(OUT, f"{code}_{name}")
    sec = cq.Workplane("XY").add(f).section(t / 2)
    cq.exporters.exportDXF(sec, base + ".dxf")
    cq.exporters.export(cq.Workplane().add(f), base + ".step")
    return dict(code=code, name=name, qty=qty, material=mat, process="激光切割", t=round(t, 1), size=f"{L:.0f} × {W:.0f}",
                files=[f"{code}_{name}.dxf", f"{code}_{name}.step"], solid=f)


def export_other(code, name, s, qty, mat, process):
    base = os.path.join(OUT, f"{code}_{name}")
    bb = s.BoundingBox()
    if process == "3D 打印":
        cq.exporters.export(cq.Workplane().add(s), base + ".stl", tolerance=0.05, angularTolerance=0.1)
        files = [f"{code}_{name}.stl"]
    else:
        cq.exporters.export(cq.Workplane().add(s), base + ".step")
        files = [f"{code}_{name}.step"]
    return dict(code=code, name=name, qty=qty, material=mat, process=process, t="—", size=f"{bb.xlen:.0f} × {bb.ylen:.0f} × {bb.zlen:.0f}", files=files)


def clean(n):
    return n.split("（")[0].replace(" ", "").replace("/", "-")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        os.remove(os.path.join(OUT, f))
    envs = [B.sweep_envelope(fx, sy) for fx in (1, -1) for sy in (1, -1)]
    frame = {n: s for n, s, c in B.frame_parts(envs)}
    parts = []
    # ---- 机身铝板（左右 / 前后一样的只出一个，数量写 2 或 4）
    pick = [("A01", "A01 底板（铝 3）", 1), ("A02", "A02 外摆隔板前（铝 5）", 2), ("A03", "A03 侧梁左（铝 3）", 2),
            ("A04", "A04 纵梁前左（铝 4）", 4), ("A05", "A05 前端板（铝 4）", 1), ("A06", "A06 后端板（铝 4）", 1), ("A07", "A07 顶板（铝 3）", 1)]
    for code, n, q in pick:
        parts.append(export_plate(code, clean(n), frame[n], q, "6061-T6 铝板"))
    # ---- 腿（左前腿的零件；右边的镜像，数量按 4 条腿）
    P, knee, foot = E.leg_module()
    leg = {n: s for n, s, c, g in P}
    parts.append(export_plate("L01", "大腿外板", leg["大腿外板（铝 4）"], 4, "6061-T6 铝板 4（左右各 2，镜像）"))
    parts.append(export_other("L02", "大腿杯和内板", leg["大腿杯 + 内板（铝，CNC）"], 4, "6061-T6", "CNC"))
    parts.append(export_other("L03", "髋座", leg["髋座（铝，CNC）"], 4, "6061-T6", "CNC"))
    parts.append(export_other("L04", "转接盘和轴", leg["转接盘 + 轴（钢，车削）"], 4, "45 钢", "车削"))
    parts.append(export_other("L05", "膝轴", leg["膝轴 Φ12"], 4, "45 钢", "车削"))
    parts.append(export_other("L06", "大腿外层壳", leg["大腿外层壳（打印，完整外形）"], 4, "PETG / ASA", "3D 打印"))
    parts.append(export_other("L07", "小腿", leg["小腿（铝 + 打印壳）"], 4, "PA12-CF（或铝管 + 打印壳）", "3D 打印"))
    parts.append(export_other("L08", "绕线环", leg["绕线环（打印 PETG）"], 4, "PETG", "3D 打印"))
    br = [s for n, s, c in B.frame_parts(envs) if n.startswith("L 角码")]
    parts.append(export_other("J01", "L角码", br[0], len(br), "不锈钢 / 铝（买成品，腿长 12、宽 12、厚 2，M3 孔）", "买"))
    # ---- 头支架、里面的打印件
    for n, s in H.body_side():
        if n.startswith("头支架"):
            parts.append(export_other("H01" if "左" in n else "H02", clean(n), s, 1, "5052 铝板 3（折弯）", "钣金折弯"))
    inside = {n: s for n, s, c in B.inside_parts()}
    parts.append(export_other("P01", "电池托", inside["电池托（打印 PETG，两道绑带槽）"], 1, "PETG", "3D 打印"))
    parts.append(export_other("P02", "托板插槽导轨", frame["托板插槽导轨左（打印）"], 2, "PETG", "3D 打印"))
    parts.append(export_plate("C01", "电子托板", inside["C01 电子托板（亚克力 3，插在侧梁槽里）"], 1, "黑色亚克力 3"))
    # ---- 总览图
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
    plates = [p for p in parts if "solid" in p]
    cols = 4
    rows = (len(plates) + cols - 1) // cols
    fig, axs = plt.subplots(rows, cols, figsize=(16, 4.2 * rows))
    for ax in axs.flat:
        ax.axis("off")
    for ax, p in zip(axs.flat, plates):
        sec = cq.Workplane("XY").add(p["solid"]).section(p["t"] / 2 if isinstance(p["t"], float) else 1)
        for e in sec.edges().vals():
            pts = [e.positionAt(i / 40) for i in range(41)]
            ax.plot([q.x for q in pts], [q.y for q in pts], "k-", lw=0.7)
        ax.set_aspect("equal")
        ax.set_title(f"{p['code']} {p['name']} ×{p['qty']}\n{p['material']}，厚 {p['t']}，{p['size']}", fontsize=9)
    plt.tight_layout()
    plt.savefig(os.path.join(DOC, "制造件总览.png"), dpi=100)
    # ---- 清单
    L = ["# 机械狗007A 制造件清单（第 4 步，v19 省钱版）", "", "> 由 `007a/mfg.py` 生成。文件在 `007a/out/mfg/`。铝板发 DXF 给激光切割厂；CNC / 车削件发 STEP；打印件是 STL。",
         "> 每个件一页的零件图（尺寸、孔表、三视图）在 `零件图.pdf`（`007a/drawings.py`）；外壳等打印件见 `打印件清单.md`（`007a/print_parts.py`）。",
         "> 孔位标 ★ 的（SteadyWin 电机安装孔和输出盘孔、舵机、IMU、USB 转 CAN）拿到 STEP / 到货核对后再改。", "",
         "| 编号 | 名字 | 数量 | 材料 | 加工 | 厚 | 外形 | 文件 |", "|---|---|---|---|---|---|---|---|"]
    for p in parts:
        L.append(f"| {p['code']} | {p['name']} | {p['qty']} | {p['material']} | {p['process']} | {p['t']} | {p['size']} | {', '.join(p['files'])} |")
    open(os.path.join(DOC, "制造件清单.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
    json.dump([{k: v for k, v in p.items() if k != "solid"} for p in parts], open(os.path.join(OUT, "parts.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"{len(parts)} 种制造件 → {OUT}")
    for p in parts:
        print(f"  {p['code']} {p['name']} ×{p['qty']}  {p['process']}  {p['size']}")
