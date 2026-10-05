"""把 out/robodog.glb、rig.json 和打印件清单塞进 template.html -> robodog_v0.1.html（文件名沿用，保持预览链接不变）"""
import base64, json, os, re
H = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(H, "..", "out")
glb = base64.b64encode(open(os.path.join(out, "robodog.glb"), "rb").read()).decode()
rows = []
for line in open(os.path.join(out, "report.txt"), encoding="utf-8"):
    m = re.match(r"^(\w+)\s+(\d+)\s+([\d.]+)×\s*([\d.]+)×\s*([\d.]+)\s+[\d.]+\s+(\S+)\s+(.*)$", line.strip())
    if m:
        dims = "×".join(str(round(float(v))) for v in m.group(3, 4, 5))
        rows.append(dict(name=m.group(1), qty=int(m.group(2)), dims=dims, flag=m.group(6), mat=m.group(7)))
last = None
for r in rows:   # 把“同上”展开成具体材料
    if r["mat"] == "同上":
        r["mat"] = last
    last = r["mat"]
html = open(os.path.join(H, "template.html"), encoding="utf-8").read()
rig = open(os.path.join(out, "rig.json"), encoding="utf-8").read()
rep_txt = open(os.path.join(out, "report.txt"), encoding="utf-8").read()
m = re.search(r"\[站立\] 机身原点离地 (\d+)mm.*?包络 长 (\d+) × 宽 (\d+) × 高 (\d+)", rep_txt, re.S)
mass = re.search(r"整机估重 ≈ ([\d.]+) kg", rep_txt).group(1)
FILL = [("__HIP__", m.group(1)), ("__LEN__", m.group(2)), ("__WID__", m.group(3)), ("__HGT__", m.group(4)),
        ("__MASS__", f"{float(mass):.1f}")]
html = html.replace("__PARTS_JSON__", json.dumps(rows, ensure_ascii=False)).replace("__RIG_JSON__", rig).replace("__GLB_B64__", glb)
for k, v in FILL:
    html = html.replace(k, v)
open(os.path.join(H, "robodog_v0.1.html"), "w", encoding="utf-8").write(html)
print(len(rows), "parts;", round(len(html) / 1e6, 2), "MB")
