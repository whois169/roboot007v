"""把头部装配位置的打印件 + 外购件导成 STL（007v/out/head_print/_asm/），给 Blender 爆炸图用。先跑 head_print.py。"""
import os

import cadquery as cq

import head_print as hp

A = os.path.join(hp.OUT, "_asm")
os.makedirs(A, exist_ok=True)
for k, (code, name, *_r) in hp.PRINT.items():
    s = cq.importers.importStep(os.path.join(hp.OUT, f"{code}_{name}.step")).val()
    cq.exporters.export(cq.Workplane().add(s), os.path.join(A, f"{k}.stl"), tolerance=0.05, angularTolerance=0.15)
KEY = {"亚克力窗": "b_acrylic", "空心轴销": "b_pin", "喇叭": "b_speaker", "屏幕板0": "b_board", "舵机": "b_servo", "铜套": "b_bush",
       "雷达": "b_lidar", "摄像头镜头": "b_camlens", "多点测距": "b_tof", "隐私灯": "b_led", "麦克风1": "b_mic1", "麦克风2": "b_mic2",
       "光线": "b_bh1750", "毫米波": "b_ld2410", "ESP32": "b_esp32", "摄像头主板": "b_camboard", "功放": "b_max", "热成像": "b_mlx",
       "红外发射": "b_irtx", "红外接收": "b_irrx", "温湿度": "b_sht40"}
for n, s, f in hp.bought():
    k = next((v for kk, v in KEY.items() if n.startswith(kk)), None)
    if not k:
        continue
    s = hp.to_neck(s) if f == "head" else s
    cq.exporters.export(cq.Workplane().add(s), os.path.join(A, k + ".stl"), tolerance=0.1, angularTolerance=0.3)
print(sorted(os.listdir(A)))
