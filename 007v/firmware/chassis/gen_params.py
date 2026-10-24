"""把 Python 底盘程序（007v/chassis/controller.py）和 007v/config.py 里的常数写成 C++ 头文件 src/params.h。

C++ 固件和 Python 参考程序用同一套数字：改了 Python 那边的常数，重新跑一次这个脚本就行。
用法：python 007v/firmware/chassis/gen_params.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))          # 007v/
sys.path.insert(0, ROOT)
from chassis import controller as C  # noqa: E402

NAMES = ["FW", "RATE", "STATUS_EVERY", "R", "FOLD_DEG", "TOUCH_DEG",
         "V_MAX", "VY_MAX", "WZ_MAX", "A_ACC", "A_BRAKE", "A_Y", "A_W", "A_ESTOP", "ESTOP_RAMP", "W_MAX",
         "DRIVE_TIMEOUT", "LINK_TIMEOUT", "HIP_SPEED", "HIP_ACC", "HIP_SLOW", "TOUCH_BAND", "HIP_STALL_DEG", "HIP_STALL_S",
         "MOVE_V", "MOVE_A", "MOVE_VY", "MOVE_WZ", "POS_TOL", "ANG_TOL", "BAT_LOW",
         "M_EMPTY", "LOAD_MIN_N", "LOAD_MIN_VAR", "LOAD_RESET_S",
         "STEP_SIN", "CLIFF_MM", "STEP_MM", "STEP_V", "STEP_SENSOR_V", "CLIFF_CLEAR_S", "STEP_HOLD_S", "CHG_A", "BRAKE_A", "EPS_T",
         "ARM", "HIP_X", "HIP_Z", "SKID_H", "STANCE_DEG", "WHEEL_D"]
ARRAYS = ["LEG_FX", "S_ROLL", "YW", "STEP_Z"]


def num(v):
    if isinstance(v, str):
        return f'"{v}"'
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    return repr(float(v)) + "f" if abs(v) < 1e30 else "0"


lines = ["// 自动生成，不要手改：python 007v/firmware/chassis/gen_params.py", "// 来源：007v/chassis/controller.py、007v/config.py",
         "#pragma once", ""]
for n in NAMES:
    v = getattr(C, n)
    t = "const char*" if isinstance(v, str) else "int" if isinstance(v, int) and not isinstance(v, bool) else "float"
    lines.append(f"constexpr {t} {n} = {num(v)};" if t != "const char*" else f"static const char* const {n} = {num(v)};")
for n in ARRAYS:
    v = getattr(C, n)
    t = "int" if all(isinstance(x, int) for x in v) else "float"
    lines.append(f"constexpr {t} {n}[4] = {{{', '.join(num(x) for x in v)}}};")
lines.append(f"static const char* const STEP_NAMES[4] = {{{', '.join(num(x) for x in C.STEP_NAMES)}}};")
lines.append(f"constexpr float PI_F = {math.pi!r}f;")
out = os.path.join(HERE, "src", "params.h")
open(out, "w", encoding="utf-8", newline="\n").write("\n".join(lines) + "\n")
print("写好了", out, f"（{len(NAMES)} 个常数、{len(ARRAYS) + 1} 个数组）")
