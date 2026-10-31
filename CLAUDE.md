# 机械狗007A（主做）+ 机械狗007v（参考保留）

开始任何工作前，先读 `docs/00_项目核心原则.md`。核心：我们做的是"科技机械狗"（不是宠物狗），也不是"装了个音箱的机器狗"，每个改动都要通过"音箱测试"，把差距放大。
做外观相关的零件前，先读 `docs/外观设计规范.md`（缝线、外观件不承力、能平放打印这些规矩照旧）。
用户说中文，文档用简单直白的中文写。

## 2026-10-31 清理：只留 007A 和 007v
- **007A 是主做、要照着造的**：纯腿四足（每腿 3 个宇树 GO-M8010-6，膝 = 电机在髋上 + HTD5M 同步带 16/24 齿），巡检管道 / 电缆隧道、能自己上下楼梯。工程 `007a/`，资料 `docs/机械狗007A/`（零件清单、采购清单.xlsx、零件图.pdf、装配说明、仿真报告、孔位核对），厂家资料 `ref/`。
- 改完 007A 依次跑：`model.py` → `body_v10.py` → `eng_body.py`（站立 / 外摆 ±20° 都要 0 处干涉）→ `eng_head.py` → `print_parts.py`（孔位全对上）→ `mfg.py` → `mass.py` → `bom.py` → `drawings.py`；效果图 Blender `blender/render.py`；仿真 `sim/run_sim.py`。外观改前先打 git 标签存档。
- 007v（轮足运输版）封存在 git 标签 `007v-最终版`，`007v/` 里的大脑程序 `007v/brain/` 和底盘程序还在用 / 参考，下面一节是它的说明。
- 旧版四足（cad/、旧文档）和 `../旧版/` 已删，要找只能翻 git 历史。

## 2026-10-16 起改版为机械狗007v
外观推翻重做：轮足运输机器人（4 个髋 + 4 个麦轮，背上驮 5–10 kg，可折叠），外观效果图 `docs/机械狗007v/外观效果图_v10.html`；**当前外观定稿 v4（git tag 外观定稿-v4）**：底盘框架外露 + 叠层顶板、原版头 + 工业细节、Φ97 麦轮在腿外侧齐平、电机在腿内侧机身下方带护罩（兼支架）、轮轴单边两个 6801、不往上折叠（最多趴平）、最高 1.0 m/s；白模渲染 `blender/render_white.py`。树莓派 5、头部 ESP32-S3、大脑程序、语音服务、传感器清单不变。
新结构在 `007v/`：`config.py` 所有尺寸 → `model.py` 零件 → `build.py` 摆姿态 + 干涉检查 + 折叠轨迹检查 + 重心 + 导出；选型计算 `sizing.py` 和 `docs/机械狗007v/选型计算书.md`。改完跑 `python 007v/build.py`，`007v/out/report.txt` 里 4 个姿态都要 0 处、折叠全程不撞；3D 查看页 `007v/viewer.html`（用 `007v/out/007v_data.js`，Artifact https://claude.ai/artifact/Hr9vptZPw8y4d6ifZNE8aL）。
第 4 步出图：`python 007v/mfg.py`（加工件 DXF / STEP / STL + 尺寸图，输出 `007v/out/mfg/`）、`python 007v/bom.py`（零件清单 → `docs/机械狗007v/零件清单.md`、`采购清单.xlsx`）、`python 007v/wiring.py`（接线图 → `docs/机械狗007v/接线图.md`），再 `python 007v/pack.py` 打包给图纸网页 `007v/drawings.html`（Artifact https://claude.ai/artifact/Kaue9H9hpVhFuuNNoapkRt）。接口尺寸都在 `config.py` 的“制造件的接口”一节，标 ★ 的是网上查不到、等卖家图纸或到货实测的。
给人看 / 拆散 / 做动画：`python 007v/export_sw.py`（SolidWorks 装配体，按运动部件分组，带铰链标记）、`python 007v/blender/export_parts.py` 再用 Blender 5.2 跑 `007v/blender/make_blend.py`（生成 `007v/out/blender/机械狗007v.blend`，带动画和拆散滑块），说明见 `docs/机械狗007v/SolidWorks和Blender使用说明.md`。这台电脑只有 eDrawings，SolidWorks 主程序没装。
仿真在 `007v/sim/`：`python 007v/sim/run_sim.py --export`（改了 CAD 要加 --export），`007v/out/sim_report.txt` 的结论除“演示”外都要 ✓；说明和得出的控制规矩在 `docs/机械狗007v/仿真报告.md`（写底盘程序时照做）。
大脑程序在 `007v/brain/`（2026-10-17 挪进来；改完跑 `python tests/run_tests.py`，在 `007v/brain` 里跑；屏幕 HUD 是 800×440，截图 `python scripts/hud_capture.py`）。
底盘程序在 `007v/chassis/`（Python 参考实现 + 虚拟底盘，以后翻成 C++ 烧进底盘 ESP32）；改完跑 `python 007v/chassis/test_chassis.py`，说明在 `docs/机械狗007v/底盘程序说明.md`。大脑默认接虚拟底盘（`config.yaml` 的 `chassis.type`），`python main.py --text --view` 能边打字边看仿真里的车走。

## 坐标和约定
机身坐标 X 前、Y 左、Z 上，单位 mm。脖子俯仰角负 = 抬头。
