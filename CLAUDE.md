# RoboDog

开始任何工作前，先读 `docs/00_项目核心原则.md`。核心：我们做的是"科技机械狗"（不是宠物狗），也不是"装了个音箱的机器狗"，每个改动都要通过"音箱测试"，把差距放大。
做外观相关的零件前，先读 `docs/外观设计规范.md`（深灰壳板 + 银色件 + 黑色传感器窗 + 琥珀橙状态灯；缝线 0.8 宽 × 0.6 深；外观件不承力、能平放打印）。

## 2026-10-16 起改版为机械狗007v
外观推翻重做：轮足运输机器人（4 个髋 + 4 个麦轮，背上驮 5–10 kg，可折叠），外观定稿 `docs/概念/机械狗007v_外观v10.html`。树莓派 5、头部 ESP32-S3、大脑程序、语音服务、传感器清单不变。
新结构在 `007v/`：`config.py` 所有尺寸 → `model.py` 零件 → `build.py` 摆姿态 + 干涉检查 + 折叠轨迹检查 + 重心 + 导出；选型计算 `sizing.py` 和 `docs/机械狗007v/选型计算书.md`。改完跑 `python 007v/build.py`，`007v/out/report.txt` 里 4 个姿态都要 0 处、折叠全程不撞；3D 查看页 `007v/viewer.html`（用 `007v/out/007v_data.js`，Artifact https://claude.ai/artifact/Hr9vptZPw8y4d6ifZNE8aL）。
第 4 步出图：`python 007v/mfg.py`（加工件 DXF / STEP / STL + 尺寸图，输出 `007v/out/mfg/`）、`python 007v/bom.py`（零件清单 → `docs/机械狗007v/零件清单.md`、`采购清单.xlsx`）、`python 007v/wiring.py`（接线图 → `docs/机械狗007v/接线图.md`），再 `python 007v/pack.py` 打包给图纸网页 `007v/drawings.html`（Artifact https://claude.ai/artifact/Kaue9H9hpVhFuuNNoapkRt）。接口尺寸都在 `config.py` 的“制造件的接口”一节，标 ★ 的是网上查不到、等卖家图纸或到货实测的。
给人看 / 拆散 / 做动画：`python 007v/export_sw.py`（SolidWorks 装配体，按运动部件分组，带铰链标记）、`python 007v/blender/export_parts.py` 再用 Blender 5.2 跑 `007v/blender/make_blend.py`（生成 `007v/out/blender/机械狗007v.blend`，带动画和拆散滑块），说明见 `docs/机械狗007v/SolidWorks和Blender使用说明.md`。这台电脑只有 eDrawings，SolidWorks 主程序没装。
仿真在 `007v/sim/`：`python 007v/sim/run_sim.py --export`（改了 CAD 要加 --export），`007v/out/sim_report.txt` 的结论除“演示”外都要 ✓；说明和得出的控制规矩在 `docs/机械狗007v/仿真报告.md`（写底盘程序时照做）。
按步骤做，每步给用户看、点头再往下：①选型（完成）②骨架（完成）③仿真（完成）④打印件、采购表、接线、资料包（4a 初版图纸完成，等样品和卖家图纸再出 4b 定稿）。下面写的四足内容（`cad/`）是旧版存档。

## 项目是什么（旧版四足，存档）
四足机器狗：12 个达妙 DM-J4310 准直驱关节（膝关节同步带 1:1.8，104 齿皮带，大腿轴距 189.3mm），2 自由度脖子（GM3506 偏航 + GM4108 俯仰），2.25 寸长条彩屏 HUD 眼睛，头部有热成像和红外遥控收发。背部两根任务导轨（兼提手），可装激光雷达模块和任务箱，GX16 任务接口。
树莓派 5（上位机）+ 达妙 MC02（下位机，3 路 CAN）+ 头部 ESP32-S3。大脑是云端多模态大模型。3D 打印骨架 + 2020 型材。
用户说中文，文档用简单直白的中文写。

## 目录
- `cad/config.py`：所有关键尺寸（改尺寸改这里）；`head.py` 头和脖子；`leg.py` 腿（含皮带走线和长度计算）；`parts.py` 机身；`armor.py` 外观件、任务导轨、雷达模块、任务箱
- `cad/build.py`：生成 STL / STEP / GLB / rig.json / report.txt；`--check` 对站立、转头、坐下、趴下 4 个姿态做干涉检查
- `cad/export_sw.py`（SolidWorks 分组装配体）、`cad/export_mujoco.py`（MuJoCo 模型）、`cad/check_neck.py`（脖子全范围扫描）、`cad/check_haa.py`（外摆关节全范围扫描）、`cad/check_knee.py`（膝关节全行程扫描，v0.6）
- 腿的造型：`config.py` → `LEG_STYLE`（A 直 / B 后弓 / C 狗腿反弓，当前 C），也可以用环境变量 `LEG_STYLE=B python3 build.py` 临时切换；小腿外形在 `leg.py` 的 `shank_profile()`，大腿装甲外形在 `armor.py` 的 `_shield_outline()`
- `out/mujoco/check_poses.py`：PD 控制下的站立、坐下、趴下、起身仿真，看电机扭矩是否超限
- `viewer/make_viewer.py` + `template.html`：生成 3D 预览网页；`viewer/shots_v6.mjs`：渲染效果图（v0.6，浅灰摄影棚背景）（需要 node + playwright + three）
- `docs/`：核心原则、外观设计规范、P1 头部开发计划、设计说明书、电子系统与接线、行动清单、分阶段采购表（xlsx）、电机实测清单、零件尺寸收集清单、SolidWorks 说明

## 买来的零件尺寸
用户会把买来的零件尺寸（卖家 3D 模型、图纸截图或实测值）填进 `docs/零件尺寸收集清单.md`。按清单里"对应模型参数"一栏改模型，改完跑下面的验证流程。打印机孔径校准结果出来后，在模型里统一加孔径补偿。

## 坐标和约定
机身坐标 X 前、Y 左、Z 上，单位 mm。脖子俯仰角负 = 抬头。

## 改完设计以后的验证流程（每次都要跑）
```bash
cd cad
python3 build.py --check        # 4 个姿态都要 0 处干涉，约 15 分钟；report.txt 里的"皮带长度检查"必须是"通过"
python3 check_neck.py           # 改了头或脖子时跑，结果更新 head.py 的 NECK_LIMITS 和文档里的限位表
python3 check_haa.py            # 改了机身、髋部或大腿时跑，外摆关节至少要能摆 ±15°
python3 check_knee.py           # 改了大腿、小腿、膝盖或足端时跑，膝关节必须保持 -10°…150° 全程无碰撞
python3 export_sw.py && python3 export_mujoco.py
cd ../out/mujoco && python3 check_poses.py    # 电机扭矩不能长时间顶到上限
cd ../../viewer && python3 make_viewer.py
```
改了设计要同步更新：README 版本记录、docs 里相关文档、采购表。
改了大腿长度、带轮齿数或张紧轮位置，必须重新确认皮带长度检查（`leg.py` 的 `_belt_loop` 按实际走线算，不要用近似公式）。
大腿和小腿现在不一样长（189.3 / 190），逆运动学用 `build.py` 里的 `ik()`（余弦定理），不要假设两段等长。
