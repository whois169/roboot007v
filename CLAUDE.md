# RoboDog

开始任何工作前，先读 `docs/00_项目核心原则.md`。核心：我们做的是"科技机械狗"（不是宠物狗），也不是"装了个音箱的机器狗"，每个改动都要通过"音箱测试"，把差距放大。
做外观相关的零件前，先读 `docs/外观设计规范.md`（深灰壳板 + 银色件 + 黑色传感器窗 + 琥珀橙状态灯；缝线 0.8 宽 × 0.6 深；外观件不承力、能平放打印）。

## 项目是什么
四足机器狗：12 个达妙 DM-J4310 准直驱关节（膝关节同步带 1:1.8，104 齿皮带，大腿轴距 189.3mm），2 自由度脖子（GM3506 偏航 + GM4108 俯仰），2.25 寸长条彩屏 HUD 眼睛，头部有热成像和红外遥控收发。背部两根任务导轨（兼提手），可装激光雷达模块和任务箱，GX16 任务接口。
树莓派 4B（上位机）+ 达妙 MC02（下位机，3 路 CAN）+ 头部 ESP32-S3。大脑是云端多模态大模型。3D 打印骨架 + 2020 型材。
用户说中文，文档用简单直白的中文写。

## 目录
- `cad/config.py`：所有关键尺寸（改尺寸改这里）；`head.py` 头和脖子；`leg.py` 腿（含皮带走线和长度计算）；`parts.py` 机身；`armor.py` 外观件、任务导轨、雷达模块、任务箱
- `cad/build.py`：生成 STL / STEP / GLB / rig.json / report.txt；`--check` 对站立、转头、坐下、趴下 4 个姿态做干涉检查
- `cad/export_sw.py`（SolidWorks 分组装配体）、`cad/export_mujoco.py`（MuJoCo 模型）、`cad/check_neck.py`（脖子全范围扫描）、`cad/check_haa.py`（外摆关节全范围扫描）
- `out/mujoco/check_poses.py`：PD 控制下的站立、坐下、趴下、起身仿真，看电机扭矩是否超限
- `viewer/make_viewer.py` + `template.html`：生成 3D 预览网页；`viewer/shots_v5.mjs`：渲染效果图（v0.5，浅灰摄影棚背景）（需要 node + playwright + three）
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
python3 export_sw.py && python3 export_mujoco.py
cd ../out/mujoco && python3 check_poses.py    # 电机扭矩不能长时间顶到上限
cd ../../viewer && python3 make_viewer.py
```
改了设计要同步更新：README 版本记录、docs 里相关文档、采购表。
改了大腿长度、带轮齿数或张紧轮位置，必须重新确认皮带长度检查（`leg.py` 的 `_belt_loop` 按实际走线算，不要用近似公式）。
大腿和小腿现在不一样长（189.3 / 190），逆运动学用 `build.py` 里的 `ik()`（余弦定理），不要假设两段等长。
