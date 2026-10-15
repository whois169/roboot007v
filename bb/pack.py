"""把 3D 模型、关节参数、仿真回放打包成 bb/out/bb01_data.js，给 bb/viewer.html 用（Artifact 不能直接放 .glb）。

build.py 和 sim/run_sim.py 跑完都会调用；也可以单独跑：python bb/pack.py
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
    with open(os.path.join(OUT, "bb01.glb"), "rb") as f:
        data["glb"] = base64.b64encode(f.read()).decode("ascii")
    with open(os.path.join(OUT, "bb01_data.js"), "w", encoding="utf-8") as f:
        f.write("window.BB01 = " + json.dumps(data, ensure_ascii=False, separators=(",", ":")) + ";\n")


if __name__ == "__main__":
    pack_viewer()
