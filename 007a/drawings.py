"""007A 第 4 步：零件图（给人看的，一个零件一页）→ docs/机械狗007A/零件图.pdf，外加每页一张 PNG（007a/out/drawings/）。

  - 铝板 / 亚克力（激光切割）：展开外形 + 总长宽 + 厚 + 孔表（按孔径分组：个数、每个孔中心坐标，原点 = 左下角）
  - CNC / 车削 / 折弯件：三视图（主视 / 俯视 / 左视，画棱线）+ 外形尺寸
  - 标题栏：编号、名字、材料、加工、数量、单位 mm、版本 v19（省钱版）、★ = 到货 / 拿到 STEP 后要核的
工厂加工用 007a/out/mfg/ 里的 DXF / STEP（mfg.py），这些图是核对、沟通用的。
用法：python 007a/drawings.py
"""
import collections
import os

import cadquery as cq
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

import eng_body as B  # noqa: E402
import eng_head as H  # noqa: E402
import eng_leg as E  # noqa: E402
from mfg import clean, flat  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "drawings")
DOC = os.path.join(os.path.dirname(HERE), "docs", "机械狗007A")
VERSION = "v19 省钱版（SteadyWin GIM8108-8）"


def title_block(fig, code, name, mat, proc, qty, note=""):
    ax = fig.add_axes([0.0, 0.0, 1.0, 0.09])
    ax.axis("off")
    ax.add_patch(plt.Rectangle((0.01, 0.08), 0.98, 0.84, fill=False, lw=1.2, transform=ax.transAxes))
    cells = [("编号", code), ("名字", name), ("材料", mat), ("加工", proc), ("数量", str(qty)), ("单位", "mm"), ("版本", VERSION)]
    x = 0.02
    for k, v in cells:
        w = max(0.07, 0.012 * (len(v) + 3))
        ax.text(x, 0.62, k, fontsize=7, color="#666", transform=ax.transAxes)
        ax.text(x, 0.22, v, fontsize=10, transform=ax.transAxes)
        x += w
    ax.text(0.98, 0.22, "机械狗007A", fontsize=10, ha="right", transform=ax.transAxes, weight="bold")
    if note:
        fig.text(0.02, 0.105, note, fontsize=8, color="#a33")


def dim(ax, p0, p1, off, text, horiz=True):
    """简单尺寸线：两端竖线 + 带箭头的线 + 文字。"""
    if horiz:
        y = p0[1] + off
        ax.plot([p0[0], p0[0]], [p0[1], y], "-", c="#36c", lw=0.5)
        ax.plot([p1[0], p1[0]], [p1[1], y], "-", c="#36c", lw=0.5)
        ax.annotate("", (p0[0], y), (p1[0], y), arrowprops=dict(arrowstyle="<->", color="#36c", lw=0.7))
        ax.text((p0[0] + p1[0]) / 2, y, text, ha="center", va="bottom" if off > 0 else "top", fontsize=8, color="#36c",
                bbox=dict(fc="white", ec="none", pad=0.5))
    else:
        x = p0[0] + off
        ax.plot([p0[0], x], [p0[1], p0[1]], "-", c="#36c", lw=0.5)
        ax.plot([p1[0], x], [p1[1], p1[1]], "-", c="#36c", lw=0.5)
        ax.annotate("", (x, p0[1]), (x, p1[1]), arrowprops=dict(arrowstyle="<->", color="#36c", lw=0.7))
        ax.text(x, (p0[1] + p1[1]) / 2, text, ha="right" if off < 0 else "left", va="center", fontsize=8, color="#36c", rotation=90,
                bbox=dict(fc="white", ec="none", pad=0.5))


def plate_page(pdf, code, name, s, qty, mat, note=""):
    f, t, L, W = flat(s)
    sec = cq.Workplane("XY").add(f).section(t / 2)
    holes, outline = [], []
    for e in sec.edges().vals():
        if e.geomType() == "CIRCLE" and abs(e.radius() - 0) > 0 and e.IsClosed():
            c = e.Center()
            holes.append((c.x, c.y, 2 * e.radius()))
        else:
            outline.append([(q.x, q.y) for q in (e.positionAt(i / 30) for i in range(31))])
    fig = plt.figure(figsize=(11.69, 8.27))
    ax = fig.add_axes([0.05, 0.16, 0.6, 0.76])
    for pts in outline:
        a = np.array(pts)
        ax.plot(a[:, 0], a[:, 1], "k-", lw=0.8)
    groups = collections.OrderedDict()
    for x, y, d in sorted(holes, key=lambda h: (-h[2], h[0], h[1])):
        groups.setdefault(round(d, 1), []).append((x, y))
    letters = "ABCDEFGHJKLMNPQRSTUVWXYZ"
    colors = plt.cm.tab10.colors
    for gi, (d, pts) in enumerate(groups.items()):
        for x, y in pts:
            ax.add_patch(plt.Circle((x, y), d / 2, fill=False, lw=0.8, ec=colors[gi % 10]))
        ax.plot([p[0] for p in pts], [p[1] for p in pts], "+", ms=3, c=colors[gi % 10], mew=0.5)
    dim(ax, (0, 0), (L, 0), -max(L, W) * 0.06, f"{L:.1f}")
    dim(ax, (0, 0), (0, W), -max(L, W) * 0.06, f"{W:.1f}", horiz=False)
    ax.set_aspect("equal")
    ax.margins(0.12)
    ax.axis("off")
    ax.set_title(f"{code}  {name}   厚 {t:.1f}   ×{qty}", fontsize=13, loc="left")
    # 孔表
    tx = fig.add_axes([0.67, 0.16, 0.31, 0.76])
    tx.axis("off")
    lines = ["孔表（原点 = 左下角，单位 mm）", ""]
    for gi, (d, pts) in enumerate(groups.items()):
        lines.append(f"{letters[gi % len(letters)]}  Φ{d:g}  × {len(pts)}")
        if len(pts) <= 12:
            lines += [f"     ({x:.1f}, {y:.1f})" for x, y in pts]
        else:
            lines.append(f"     坐标见 DXF（{len(pts)} 个）")
    total = sum(len(p) for p in groups.values())
    lines += ["", f"共 {total} 个孔", "", "未注倒角：去毛刺", "DXF 文件：007a/out/mfg/" + f"{code}_{clean(name)}.dxf"]
    y = 1.0
    for i, s_ in enumerate(lines):
        col = "k"
        if s_[:1] in letters and "Φ" in s_:
            col = colors[letters.index(s_[0]) % 10]
        tx.text(0, y, s_, fontsize=7.5 if s_.startswith("     ") else 9, color=col, va="top", transform=tx.transAxes, family="Microsoft YaHei")
        y -= 0.022 if s_.startswith("     ") else 0.03
        if y < 0:
            tx.text(0, y, "……（太多了，见 DXF）", fontsize=8, transform=tx.transAxes)
            break
    title_block(fig, code, name, mat, "激光切割", qty, note)
    pdf.savefig(fig)
    fig.savefig(os.path.join(OUT, f"{code}_{clean(name)}.png"), dpi=110)
    plt.close(fig)


def edges_of(s):
    """实体 → 所有棱线（B-rep 的边，圆弧也按真实形状取点），返回 N×2×3 的线段。"""
    seg = []
    for e in s.Edges():
        n = 24 if e.geomType() in ("CIRCLE", "BSPLINE", "ELLIPSE") else 1
        pts = [e.positionAt(i / n) for i in range(n + 1)]
        seg += [((p.x, p.y, p.z), (q.x, q.y, q.z)) for p, q in zip(pts, pts[1:])]
    return np.array(seg)


def solid_page(pdf, code, name, s, qty, mat, proc, note=""):
    bb = s.BoundingBox()
    s = s.translate(V(-bb.xmin, -bb.ymin, -bb.zmin))
    bb = s.BoundingBox()
    seg = edges_of(s)
    fig = plt.figure(figsize=(11.69, 8.27))
    views = [("主视（从前往后看，x–z）", (0, 2), [0.05, 0.52, 0.42, 0.38]), ("俯视（从上往下看，x–y）", (0, 1), [0.05, 0.13, 0.42, 0.36]),
             ("左视（从左往右看，y–z）", (1, 2), [0.53, 0.52, 0.42, 0.38])]
    size = (bb.xlen, bb.ylen, bb.zlen)
    for title, (i, j), rect in views:
        ax = fig.add_axes(rect)
        for a, b in seg:
            ax.plot([a[i], b[i]], [a[j], b[j]], "k-", lw=0.45)
        dim(ax, (0, 0), (size[i], 0), -max(size[i], size[j]) * 0.08, f"{size[i]:.1f}")
        dim(ax, (0, 0), (0, size[j]), -max(size[i], size[j]) * 0.08, f"{size[j]:.1f}", horiz=False)
        ax.set_aspect("equal")
        ax.margins(0.15)
        ax.axis("off")
        ax.set_title(title, fontsize=9, loc="left")
    # 等轴测
    ax = fig.add_axes([0.53, 0.13, 0.42, 0.36])
    R = np.array([[np.cos(np.radians(45)), -np.sin(np.radians(45)), 0], [np.sin(np.radians(45)) * 0.5, np.cos(np.radians(45)) * 0.5, 0.82]])
    for a, b in seg:
        p, q = R @ a, R @ b
        ax.plot([p[0], q[0]], [p[1], q[1]], "k-", lw=0.4)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title("轴测（看个样子）", fontsize=9, loc="left")
    fig.suptitle(f"{code}  {name}   外形 {bb.xlen:.1f} × {bb.ylen:.1f} × {bb.zlen:.1f}   ×{qty}", x=0.05, ha="left", fontsize=13)
    title_block(fig, code, name, mat, proc, qty, note)
    pdf.savefig(fig)
    fig.savefig(os.path.join(OUT, f"{code}_{clean(name)}.png"), dpi=110)
    plt.close(fig)


V = cq.Vector


def main():
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        os.remove(os.path.join(OUT, f))
    envs = [B.sweep_envelope(fx, sy) for fx in (1, -1) for sy in (1, -1)]
    frame = {n: s for n, s, c in B.frame_parts(envs)}
    inside = {n: s for n, s, c in B.inside_parts()}
    P, _, _ = E.leg_module()
    leg = {n: s for n, s, c, g in P}
    star_motor = "★ 电机安装孔、输出盘孔照宇树画的，拿到 SteadyWin GIM8108-8 的 STEP 后要改"
    with PdfPages(os.path.join(DOC, "零件图.pdf")) as pdf:
        # 封面
        fig = plt.figure(figsize=(11.69, 8.27))
        fig.text(0.06, 0.8, "机械狗007A 零件图", fontsize=30, weight="bold")
        fig.text(0.06, 0.72, VERSION + "    由 007a/drawings.py 生成", fontsize=12, color="#555")
        rows = [("A01–A07", "机身铝板（激光切割，6061-T6）"), ("L01", "大腿外板（激光切割）"), ("L02 / L03", "大腿杯 + 内板、髋座（CNC）"),
                ("L04 / L05", "转接盘 + 轴、膝轴（车削，45 钢）"), ("H01 / H02", "头支架（5052 铝板 3 折弯）"), ("C01", "电子托板（亚克力 3）"),
                ("打印件", "见 打印件清单.md（STL 在 007a/out/print/）")]
        for i, (a, b) in enumerate(rows):
            fig.text(0.08, 0.6 - i * 0.05, a, fontsize=13, weight="bold")
            fig.text(0.24, 0.6 - i * 0.05, b, fontsize=13)
        fig.text(0.06, 0.12, "工厂加工用 007a/out/mfg/ 里的 DXF（激光切割）/ STEP（CNC、车削、折弯）。这份图是核对、沟通用的。\n"
                             "★ = 到货或拿到卖家 STEP 后要核对的尺寸。未注公差按 GB/T 1804-m；铝件去毛刺；CNC 件阳极氧化深灰。", fontsize=10, color="#444")
        pdf.savefig(fig)
        plt.close(fig)
        for code, n, q in [("A01", "A01 底板（铝 3）", 1), ("A02", "A02 外摆隔板前（铝 5）", 2), ("A03", "A03 侧梁左（铝 3）", 2),
                           ("A04", "A04 纵梁前左（铝 4）", 4), ("A05", "A05 前端板（铝 4）", 1), ("A06", "A06 后端板（铝 4）", 1),
                           ("A07", "A07 顶板（铝 3）", 1)]:
            note = star_motor.replace("电机安装孔、输出盘孔", "外摆电机背面安装孔（R40 × 8）") if code == "A02" else ""
            plate_page(pdf, code, clean(n), frame[n], q, "6061-T6 铝板", note)
            print("  ", code)
        plate_page(pdf, "L01", "大腿外板", leg["大腿外板（铝 4）"], 4, "6061-T6 铝板 4（左右各 2，镜像）")
        for code, nm, key, mat, proc in (("L02", "大腿杯和内板", "大腿杯 + 内板（铝，CNC）", "6061-T6，阳极氧化深灰", "CNC"),
                                         ("L03", "髋座", "髋座（铝，CNC）", "6061-T6，阳极氧化深灰", "CNC"),
                                         ("L04", "转接盘和轴", "转接盘 + 轴（钢，车削）", "45 钢，发黑", "车削"),
                                         ("L05", "膝轴", "膝轴 Φ12", "45 钢，发黑", "车削")):
            solid_page(pdf, code, nm, leg[key], 4, mat, proc, star_motor if code in ("L02", "L03", "L04") else "")
            print("  ", code)
        for n, s in H.body_side():
            if n.startswith("头支架"):
                solid_page(pdf, "H01" if "左" in n else "H02", clean(n), s, 1, "5052 铝板 3", "钣金折弯", "★ 舵机孔按经销商资料，到货核")
        plate_page(pdf, "C01", "电子托板", inside["C01 电子托板（亚克力 3，插在侧梁槽里）"], 1, "黑色亚克力 3",
                   "★ 树莓派、USB 转 CAN、IMU 的孔到货核")
    print("→", os.path.join(DOC, "零件图.pdf"))


if __name__ == "__main__":
    main()
