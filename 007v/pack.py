"""把 3D 模型、关节参数、仿真回放打包成 007v/out/007v_data.js，给 007v/viewer.html 用（Artifact 不能直接放 .glb）。

build.py 和 sim/run_sim.py 跑完都会调用；也可以单独跑：python 007v/pack.py
"""
import base64
import json
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")


def pack_viewer():
    with open(os.path.join(OUT, "rig.json"), encoding="utf-8") as f:
        data = dict(rig=json.load(f))
    sim = os.path.join(OUT, "sim.json")
    if os.path.exists(sim):
        with open(sim, encoding="utf-8") as f:
            data["sim"] = json.load(f)
    with open(os.path.join(OUT, "007v.glb"), "rb") as f:
        data["glb"] = base64.b64encode(f.read()).decode("ascii")
    with open(os.path.join(OUT, "007v_data.js"), "w", encoding="utf-8") as f:
        f.write("window.DOG007V = " + json.dumps(data, ensure_ascii=False, separators=(",", ":")) + ";\n")


def pack_drawings():
    """图纸网页（007v/drawings.html）用的数据：零件图、清单、接线图 → 007v/out/mfg/dwg_data.js"""
    mfg = os.path.join(OUT, "mfg")
    data = {}
    for key, fn in (("mfg", "parts.json"), ("bom", "bom.json"), ("wiring", "wiring.json")):
        with open(os.path.join(mfg, fn), encoding="utf-8") as f:
            data[key] = json.load(f)
    with open(os.path.join(mfg, "dwg_data.js"), "w", encoding="utf-8") as f:
        f.write("window.DOG007V_DWG = " + json.dumps(data, ensure_ascii=False, separators=(",", ":")) + ";\n")


if __name__ == "__main__":
    pack_viewer()
    if os.path.exists(os.path.join(OUT, "mfg", "parts.json")):
        pack_drawings()
