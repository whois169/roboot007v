"""
导出给 SolidWorks 做运动分析用的文件（零位、按刚体分组）：
  out/solidworks/robodog_v0.6_rigid_groups.step   一个装配体，15 个子装配 = 15 个刚体
  out/solidworks/groups/<刚体>.step                   每个刚体单独一个文件
  out/solidworks/关节表.csv                            关节轴线、转动范围、站立/坐下/趴下角度
  out/solidworks/刚体质量.csv                          每个刚体的估算质量和重心（零位、mm）
用法：cd cad && python3 export_sw.py        （会先完整跑一遍 build.py，约 2–3 分钟）
"""
import csv
import os
import sys

sys.argv = [sys.argv[0]] + (["--check"] if "--check" in sys.argv else [])   # 加 --check 会顺带做全量干涉检查
import build as B                 # noqa: E402  导入即生成全部常规输出
import cadquery as cq             # noqa: E402

SW = os.path.join(B.OUT, "solidworks")
os.makedirs(os.path.join(SW, "groups"), exist_ok=True)

GROUP_CN = {"body": "机身（固定件）", "neck_yaw": "脖子偏航", "neck_pitch": "头部（俯仰）"}
for code, sx, sy, cn in B.LEGS:
    GROUP_CN[f"{code}_haa"] = f"{cn}腿-髋支架（外摆）"
    GROUP_CN[f"{code}_hfe"] = f"{cn}腿-大腿（髋俯仰）"
    GROUP_CN[f"{code}_knee"] = f"{cn}腿-小腿（膝）"

# ---------------- 装配体 STEP（零位，每个刚体一个子装配）
top = cq.Assembly(name="RoboDog_v0.6_zero_pose")
for g, lst in B.groups.items():
    sub = cq.Assembly(name=g)
    one = cq.Assembly(name=g)
    for i, (nm, s, c) in enumerate(lst):
        if c == "keepout":
            continue
        sub.add(s, name=f"{g}_{i:02d}_{nm}", color=cq.Color(B.COL[c]))
        one.add(s, name=f"{g}_{i:02d}_{nm}", color=cq.Color(B.COL[c]))
    top.add(sub, name=g)
    one.export(os.path.join(SW, "groups", f"{g}.step"))
top.export(os.path.join(SW, "robodog_v0.6_rigid_groups.step"))

# ---------------- 关节表
RANGE = {"haa": ("-30", "+30", "未校核，建议先限制在 ±30°"),
         "hfe": ("-85", "+85", "站立/坐下/趴下三个姿态已校核"),
         "knee": ("-150", "+12", "已校核：膝内角 192°→30° 全程无碰撞（转角 = -β）"),
         "neck_yaw": ("-50", "+50", "已校核"),
         "neck_pitch": ("-15", "+25", "已校核；负 = 抬头")}
MATE = {"haa": "HAA电机外圆柱面（机身） ↔ HAA法兰圆柱面（髋支架组）：同轴心 + 法兰端面重合；或直接用铰链配合",
        "hfe": "HFE电机外圆柱面（髋支架组） ↔ HFE法兰圆柱面（大腿组）：同轴心 + 端面重合",
        "knee": "膝轴圆柱面（大腿组） ↔ 小腿轮毂内孔（小腿组）：同轴心 + 轮毂端面与大腿内板距离 4mm",
        "neck_yaw": "偏航电机定子（机身） ↔ 偏航电机转子（脖子偏航组）：同轴心 + 端面重合",
        "neck_pitch": "俯仰电机定子（脖子偏航组） ↔ 俯仰电机转子（头部组）：同轴心 + 端面重合"}
rows = []
for j, d in B.joints.items():
    kind = j.split("_", 1)[1] if j.startswith(("FL", "FR", "RL", "RR")) else j
    lo, hi, note = RANGE[kind]
    p, ax = d["point"], d["axis"]
    ang = {pn: B.joint_angles(pose)[j] for pn, pose in B.POSES.items()}
    rows.append([j, GROUP_CN[d["parent"]], GROUP_CN[j], f"{p[0]:.1f}", f"{p[1]:.1f}", f"{p[2]:.1f}",
                 f"{ax[0]:g}", f"{ax[1]:g}", f"{ax[2]:g}", lo, hi,
                 f"{ang['stand']:.1f}", f"{ang['look']:.1f}", f"{ang['sit']:.1f}", f"{ang['lie']:.1f}", MATE[kind], note])
with open(os.path.join(SW, "关节表.csv"), "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["关节", "父刚体", "子刚体", "轴上一点X(mm)", "轴上一点Y(mm)", "轴上一点Z(mm)", "轴方向X", "轴方向Y", "轴方向Z",
                "转角下限(°)", "转角上限(°)", "站立(°)", "转头(°)", "坐下(°)", "趴下(°)", "SolidWorks 配合建议", "说明"])
    w.writerows(rows)

# ---------------- 刚体质量
with open(os.path.join(SW, "刚体质量.csv"), "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["刚体", "名称", "估算质量(kg)", "重心X(mm)", "重心Y(mm)", "重心Z(mm)", "说明"])
    tot = 0.0
    for g, lst in B.groups.items():
        items = [(g, nm, s, c) for nm, s, c in lst]
        M, c = B.com(items)
        tot += M
        w.writerow([g, GROUP_CN[g], f"{M:.3f}", f"{c[0]:.1f}", f"{c[1]:.1f}", f"{c[2]:.1f}",
                    "零位坐标；外购件按公开参数，打印件按材料密度 × 填充率估算"])
    w.writerow(["合计", "", f"{tot:.3f}", "", "", "", "电池已按配平位置计入"])
print("SolidWorks 文件 ->", os.path.abspath(SW))
