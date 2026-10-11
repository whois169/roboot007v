"""长条屏 HUD：渲染器 + 指令协议。按真屏分辨率 284×76 画。

画面思路：平时是一台在工作的机器，全屏的真实数据流（大脑推过来的读数、事件、耗时）用细小的字
一直平滑地往上流；有人和它打交道时（唤醒、聆听、思考、汇报、锁定目标），
数据流里的方块往中间飞、“拼”出两只眼睛；结束后眼睛“解体”成方块散开，回到数据流。

移植规则（以后照着改成 ESP32 的 C++）：
  - 渲染器只通过 Gfx 画图：点、线、矩形、实心凸多边形、圆、5×7 位图字体。TFT_eSPI 都有对应函数。
  - 没有透明混合、没有模糊/发光。“淡入淡出”全部用 mix(背景色, 颜色, 程度) 把颜色往背景缩放来做。
  - 所有动画只依赖“现在时间”和每帧的 dt，30 帧。

指令（每条一行 JSON，字段都可以省略，只更新给出的部分）：
  {"mode": "track", "gaze": [0.3, -0.1], "level": 0.6, "text": "E-NET", "hold": 2.5,
   "status": {"temp": 22.1, "sim": true, "bat": 87}, "log": "FACE 1 X-0.40"}
  mode   见 MODES
  gaze   [x, y]，-1–1。x 按狗自己的方向：-1 = 狗的左边（面对屏幕时眼睛往你的右边移），y：-1 上，+1 下
  level  0–1：聆听时是麦克风音量，汇报时是语音音量
  text   左下角的短文字（换模式时自动清掉）
  hold   这个模式至少保持几秒，期间来的模式切换先排队（唤醒默认 0.9 秒）
  status 边上显示的状态：temp 温度（sim=true 表示假数据，后面带 *），bat 电量（null = 不知道）
  log    往数据流里加一行（或一组行），最多 52 个字符，只能用字体里有的字符
"""
import math
import random
from collections import deque

W, H = 284, 76
CX, CY = W // 2, H // 2
EYE_DX = 58                   # 眼睛中心到屏幕中线的距离
GAZE_X, GAZE_Y = 22, 8        # gaze = ±1 时眼睛移动多少像素
TOP, BOTTOM = 11, 66          # 中间区域（上下两条留给边上的状态文字）
LOG_MAX = 52                  # 数据流一行最多几个字（小字体每字占 5 像素宽）
LINE_H = 6                    # 数据流行高（小字体 5 像素高 + 1 像素行距）
FLOW_MIN = 4.0                # 数据流最慢的流速（像素/秒）

BG = (2, 7, 10)
CYAN = (0, 229, 255)          # #00E5FF
AMBER = (255, 176, 0)
RED = (255, 48, 48)
WHITE = (225, 255, 255)

MODES = ("idle", "wake", "listen", "think", "speak", "track", "scan", "thermal",
         "alert", "error", "sleep", "low_bat")
LABEL = {"idle": "IDLE", "wake": "WAKE", "listen": "LISTEN", "think": "THINK", "speak": "SPEAK",
         "track": "TRACK", "scan": "SCAN", "thermal": "THERMAL", "alert": "ALERT", "error": "ERROR",
         "sleep": "SLEEP", "low_bat": "LOW BAT"}
DEFAULT_HOLD = {"wake": 0.9}

# 每个模式：眼睛形状（宽、高、切角、亮度、实心程度、准星程度）、眼睛出不出现、数据流亮度、颜色、叠加层
STYLE = {
    "idle":    dict(w=46, h=36, ch=9, bright=0.8, fill=1, ret=0, eyes=0, stream=1.0,  color=CYAN,  ov=None),
    "wake":    dict(w=46, h=36, ch=9, bright=1.0, fill=1, ret=0, eyes=1, stream=0.25, color=CYAN,  ov="online"),
    "listen":  dict(w=46, h=36, ch=9, bright=0.9, fill=1, ret=0, eyes=1, stream=0.2,  color=CYAN,  ov="bars"),
    "think":   dict(w=48, h=12, ch=5, bright=0.85, fill=1, ret=0, eyes=1, stream=0.3, color=CYAN,  ov="think"),
    "speak":   dict(w=46, h=30, ch=8, bright=0.9, fill=1, ret=0, eyes=1, stream=0.2,  color=CYAN,  ov="wave"),
    "track":   dict(w=40, h=40, ch=8, bright=1.0, fill=0, ret=1, eyes=1, stream=0.2,  color=CYAN,  ov=None),
    "scan":    dict(w=46, h=36, ch=9, bright=0.8, fill=1, ret=0, eyes=0, stream=0.45, color=CYAN,  ov="scan"),
    "thermal": dict(w=46, h=36, ch=9, bright=0.8, fill=1, ret=0, eyes=0, stream=0.0,  color=CYAN,  ov="thermal"),
    "alert":   dict(w=46, h=36, ch=9, bright=0.8, fill=1, ret=0, eyes=0, stream=0.5,  color=AMBER, ov="alert"),
    "error":   dict(w=46, h=36, ch=9, bright=0.8, fill=1, ret=0, eyes=0, stream=0.5,  color=RED,   ov="error"),
    "sleep":   dict(w=46, h=36, ch=9, bright=0.8, fill=1, ret=0, eyes=0, stream=0.0,  color=CYAN,  ov="sleep"),
    "low_bat": dict(w=46, h=36, ch=9, bright=0.8, fill=1, ret=0, eyes=0, stream=0.25, color=AMBER, ov="lowbat"),
}
OVERLAYS = ("online", "bars", "think", "wave", "scan", "thermal", "alert", "error", "sleep", "lowbat")
FROZEN = ("alert", "error")   # 警告、错误时数据流卡住不动


def mix(a, b, t):
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    return (int(a[0] + (b[0] - a[0]) * t), int(a[1] + (b[1] - a[1]) * t), int(a[2] + (b[2] - a[2]) * t))


# ---------------------------------------------------------------- 5×7 位图字体
_F = {
    "A": ".###.#...##...#######...##...##...#", "B": "####.#...##...#####.#...##...#####.",
    "C": ".###.#...##....#....#....#...#.###.", "D": "####.#...##...##...##...##...#####.",
    "E": "######....#....####.#....#....#####", "F": "######....#....####.#....#....#....",
    "G": ".###.#...##....#.####...##...#.####", "H": "#...##...##...#######...##...##...#",
    "I": ".###...#....#....#....#....#...###.", "J": "..###...#....#....#....#.#..#..##..",
    "K": "#...##..#.#.#..##...#.#..#..#.#...#", "L": "#....#....#....#....#....#....#####",
    "M": "#...###.###.#.##.#.##...##...##...#", "N": "#...##...###..##.#.##..###...##...#",
    "O": ".###.#...##...##...##...##...#.###.", "P": "####.#...##...#####.#....#....#....",
    "Q": ".###.#...##...##...##.#.##..#..##.#", "R": "####.#...##...#####.#.#..#..#.#...#",
    "S": ".#####....#.....###.....#....#####.", "T": "#####..#....#....#....#....#....#..",
    "U": "#...##...##...##...##...##...#.###.", "V": "#...##...##...##...##...#.#.#...#..",
    "W": "#...##...##...##.#.##.#.##.#.#.#.#.", "X": "#...##...#.#.#...#...#.#.#...##...#",
    "Y": "#...##...#.#.#...#....#....#....#..", "Z": "#####....#...#...#...#...#....#####",
    "0": ".###.#...##..###.#.###..##...#.###.", "1": "..#...##....#....#....#....#...###.",
    "2": ".###.#...#....#...#...#...#...#####", "3": "#####...#...#.....#.....##...#.###.",
    "4": "...#...##..#.#.#..#.#####...#....#.", "5": "######....####.....#....##...#.###.",
    "6": "..##..#...#....####.#...##...#.###.", "7": "#####....#...#...#...#....#....#...",
    "8": ".###.#...##...#.###.#...##...#.###.", "9": ".###.#...##...#.####....#...#..##..",
    " ": "...................................", ".": ".........................##...##...",
    "%": "##...##..#...#...#...#...#..##...##", "-": "...............#####...............",
    "+": "......#....#..#####..#....#........", ":": "......##...##........##...##.......",
    "/": ".........#...#...#...#...#.........", "!": "..#....#....#....#....#.........#..",
    "*": "......#..#.#.#.###.#.#.#..#........", "_": "..............................#####",
    "=": "".join([".....", ".....", "#####", ".....", "#####", ".....", "....."]),
    "<": "".join(["...#.", "..#..", ".#...", "#....", ".#...", "..#..", "...#."]),
    ">": "".join([".#...", "..#..", "...#.", "....#", "...#.", "..#..", ".#..."]),
    "(": "".join(["..#..", ".#...", "#....", "#....", "#....", ".#...", "..#.."]),
    ")": "".join(["..#..", "...#.", "....#", "....#", "....#", "...#.", "..#.."]),
    ",": "".join([".....", ".....", ".....", ".....", ".##..", "..#..", ".#..."]),
    "#": "".join([".#.#.", ".#.#.", "#####", ".#.#.", "#####", ".#.#.", ".#.#."]),
    "[": "".join([".###.", ".#...", ".#...", ".#...", ".#...", ".#...", ".###."]),
    "]": "".join([".###.", "...#.", "...#.", "...#.", "...#.", "...#.", ".###."]),
    "?": "".join([".###.", "#...#", "....#", "...#.", "..#..", ".....", "..#.."]),
}
GLYPHS = {k: [[v[r * 5 + c] == "#" for c in range(5)] for r in range(7)] for k, v in _F.items()}

# 数据流用的小字体：4 像素宽、5 像素高，笔画细，每字占 5×6
_S = {
    "A": ['.##.', '#..#', '####', '#..#', '#..#'], "B": ['###.', '#..#', '###.', '#..#', '###.'], "C": ['.###', '#...', '#...', '#...', '.###'],
    "D": ['###.', '#..#', '#..#', '#..#', '###.'], "E": ['####', '#...', '###.', '#...', '####'], "F": ['####', '#...', '###.', '#...', '#...'],
    "G": ['.###', '#...', '#.##', '#..#', '.###'], "H": ['#..#', '#..#', '####', '#..#', '#..#'], "I": ['###.', '.#..', '.#..', '.#..', '###.'],
    "J": ['..##', '...#', '...#', '#..#', '.##.'], "K": ['#..#', '#.#.', '##..', '#.#.', '#..#'], "L": ['#...', '#...', '#...', '#...', '####'],
    "M": ['#..#', '####', '####', '#..#', '#..#'], "N": ['#..#', '##.#', '#.##', '#..#', '#..#'], "O": ['.##.', '#..#', '#..#', '#..#', '.##.'],
    "P": ['###.', '#..#', '###.', '#...', '#...'], "Q": ['.##.', '#..#', '#..#', '#.#.', '.#.#'], "R": ['###.', '#..#', '###.', '#.#.', '#..#'],
    "S": ['.###', '#...', '.##.', '...#', '###.'], "T": ['###.', '.#..', '.#..', '.#..', '.#..'], "U": ['#..#', '#..#', '#..#', '#..#', '.##.'],
    "V": ['#..#', '#..#', '#..#', '.##.', '.##.'], "W": ['#..#', '#..#', '####', '####', '#..#'], "X": ['#..#', '.##.', '.##.', '.##.', '#..#'],
    "Y": ['#..#', '#..#', '.##.', '.#..', '.#..'], "Z": ['####', '...#', '.##.', '#...', '####'], "0": ['.##.', '#.##', '##.#', '#..#', '.##.'],
    "1": ['.#..', '##..', '.#..', '.#..', '###.'], "2": ['###.', '...#', '.##.', '#...', '####'], "3": ['###.', '...#', '.##.', '...#', '###.'],
    "4": ['#..#', '#..#', '####', '...#', '...#'], "5": ['####', '#...', '###.', '...#', '###.'], "6": ['.##.', '#...', '###.', '#..#', '.##.'],
    "7": ['####', '...#', '..#.', '.#..', '.#..'], "8": ['.##.', '#..#', '.##.', '#..#', '.##.'], "9": ['.##.', '#..#', '.###', '...#', '.##.'],
    " ": ['....', '....', '....', '....', '....'], ".": ['....', '....', '....', '....', '.#..'], "%": ['#..#', '...#', '..#.', '.#..', '#..#'],
    "-": ['....', '....', '###.', '....', '....'], "+": ['....', '.#..', '###.', '.#..', '....'], ":": ['....', '.#..', '....', '.#..', '....'],
    "/": ['...#', '..#.', '..#.', '.#..', '#...'], "!": ['.#..', '.#..', '.#..', '....', '.#..'], "*": ['....', '#.#.', '.#..', '#.#.', '....'],
    "_": ['....', '....', '....', '....', '####'], "=": ['....', '###.', '....', '###.', '....'], "<": ['..#.', '.#..', '#...', '.#..', '..#.'],
    ">": ['#...', '.#..', '..#.', '.#..', '#...'], "(": ['..#.', '.#..', '.#..', '.#..', '..#.'], ")": ['.#..', '..#.', '..#.', '..#.', '.#..'],
    ",": ['....', '....', '....', '.#..', '#...'], "#": ['.#.#', '####', '.#.#', '####', '.#.#'], "[": ['##..', '#...', '#...', '#...', '##..'],
    "]": ['##..', '.#..', '.#..', '.#..', '##..'], "?": ['###.', '...#', '.##.', '....', '.#..'],
}
SMALL = {k: [[row[c] == "#" for c in range(4)] for row in v] for k, v in _S.items()}


def text_width(s, scale=1):
    return max(0, len(s) * 6 * scale - scale)


def clean(s: str) -> str:
    """转成字体能显示的样子：大写，不认识的字符变空格。"""
    return "".join(ch if ch in GLYPHS else " " for ch in str(s).upper())[:LOG_MAX]


class Gfx:
    """画图接口：渲染器只能用这些（ESP32 上一一对应 TFT_eSPI 的函数）。"""

    def __init__(self, surface):
        import pygame
        self.pg, self.s = pygame, surface

    def clear(self, c):
        self.s.fill(c)

    def pixel(self, x, y, c):
        if 0 <= x < W and 0 <= y < H:
            self.s.set_at((int(x), int(y)), c)

    def line(self, x0, y0, x1, y1, c):
        self.pg.draw.line(self.s, c, (int(x0), int(y0)), (int(x1), int(y1)))

    def rect(self, x, y, w, h, c):
        self.pg.draw.rect(self.s, c, (int(x), int(y), max(1, int(w)), max(1, int(h))), 1)

    def fill_rect(self, x, y, w, h, c):
        self.pg.draw.rect(self.s, c, (int(x), int(y), max(1, int(w)), max(1, int(h))))

    def fill_poly(self, pts, c):        # 只用凸多边形（ESP32 上拆成三角形）
        self.pg.draw.polygon(self.s, c, [(int(x), int(y)) for x, y in pts])

    def poly(self, pts, c):
        self.pg.draw.lines(self.s, c, True, [(int(x), int(y)) for x, y in pts])

    def circle(self, x, y, r, c):
        self.pg.draw.circle(self.s, c, (int(x), int(y)), int(r), 1)

    def text_small(self, x, y, s, c, top=0, bottom=H):
        """数据流用的 4×5 小字体，超出 top–bottom 的部分不画（流到顶上就被切掉）。"""
        for i, ch in enumerate(s):
            g = SMALL.get(ch, SMALL[" "])
            ox = x + i * 5
            for r in range(5):
                yy = y + r
                if top <= yy < bottom:
                    for col in range(4):
                        if g[r][col]:
                            self.pixel(ox + col, yy, c)

    def text(self, x, y, s, c, scale=1):
        for i, ch in enumerate(s.upper()):
            g = GLYPHS.get(ch, GLYPHS[" "])
            ox = x + i * 6 * scale
            for r in range(7):
                for col in range(5):
                    if g[r][col]:
                        if scale == 1:
                            self.pixel(ox + col, y + r, c)
                        else:
                            self.fill_rect(ox + col * scale, y + r * scale, scale, scale, c)


def chamfer_pts(cx, cy, w, h, ch):
    """带切角的方块（八边形），和机身的斜切角一致。"""
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    c = max(0.0, min(ch, w / 2, h / 2))
    return [(x0 + c, y0), (x1 - c, y0), (x1, y0 + c), (x1, y1 - c), (x1 - c, y1), (x0 + c, y1), (x0, y1 - c), (x0, y0 + c)]


# ---------------------------------------------------------------- 指令检查（模拟器和 ESP32 都按这个来）
def check(cmd) -> str:
    if not isinstance(cmd, dict):
        return "指令不是 JSON 对象"
    bad = set(cmd) - {"mode", "gaze", "level", "text", "hold", "status", "log"}
    if bad:
        return f"不认识的字段：{sorted(bad)}"
    if "mode" in cmd and cmd["mode"] not in MODES:
        return f"没有这个模式：{cmd['mode']!r}"
    if "gaze" in cmd:
        g = cmd["gaze"]
        if not (isinstance(g, (list, tuple)) and len(g) == 2 and all(isinstance(v, (int, float)) for v in g)):
            return "gaze 要写成 [x, y]"
    for k, lo, hi in (("level", 0, 1), ("hold", 0, 30)):
        if k in cmd and not (isinstance(cmd[k], (int, float)) and lo <= cmd[k] <= hi):
            return f"{k} 应该在 {lo}–{hi}"
    if "text" in cmd and not (isinstance(cmd["text"], str) and len(cmd["text"]) <= 14):
        return "text 最多 14 个字符"
    if "log" in cmd:
        lines = cmd["log"] if isinstance(cmd["log"], list) else [cmd["log"]]
        if not all(isinstance(s, str) and len(s) <= LOG_MAX for s in lines):
            return f"log 每行最多 {LOG_MAX} 个字符"
    return ""


# ---------------------------------------------------------------- 渲染器
class HudRenderer:
    def __init__(self, gfx: Gfx, now=0.0, seed=None):
        self.g = gfx
        self.rng = random.Random(seed)
        self.now = now
        self.mode = "idle"
        self.mode_t = now
        self.hold_until = 0.0
        self.pending = None
        st = STYLE["idle"]
        keys = ("w", "h", "ch", "bright", "fill", "ret", "stream")
        self.cur = {k: float(st[k]) for k in keys}
        self.cur.update(gx=0.0, gy=0.0)
        self.tgt = dict(self.cur)
        self.color, self.tcolor = st["color"], st["color"]
        self.fade = {k: 0.0 for k in OVERLAYS}
        self.level, self.level_t = 0.0, 0.0
        self.text = ""
        self.status = {"temp": None, "sim": False, "bat": None}
        self.flash_t = self.lock_t = -9.0
        self.sweep_t, self.next_sweep = -9.0, now + 4
        self.bars = [0.0] * 7
        self.bar_rand = [1.0] * 7
        self.bar_next = 0.0
        # 眼睛出现/消失：pres 0 = 没有眼睛，1 = 完整的眼睛；中间是方块飞来飞去
        self.pres, self.pres_t = 0.0, 0.0
        self.particles = []                       # [(u, v, 起点 x, 起点 y, 延迟)]
        self._new_particles()
        # 数据流
        self.lines = []                           # 屏幕上的行：{"s": 文字, "y": 位置, "typed": 已经打出几个字}
        self.queue = deque(maxlen=40)             # 还没进屏幕的行
        self.thermal, self.thermal_sim = None, False   # 热成像画面（set_thermal）

    # ---------------- 收指令
    def apply(self, cmd: dict):
        if "status" in cmd and isinstance(cmd["status"], dict):
            self.status.update(cmd["status"])
        if "gaze" in cmd:
            gx, gy = cmd["gaze"]
            self.tgt["gx"], self.tgt["gy"] = max(-1.0, min(1.0, gx)), max(-1.0, min(1.0, gy))
        if "log" in cmd:
            for s in (cmd["log"] if isinstance(cmd["log"], list) else [cmd["log"]]):
                self.queue.append(clean(s))
        if "mode" in cmd:
            m = cmd["mode"]
            hold = cmd.get("hold", DEFAULT_HOLD.get(m, 0.0))
            if self.now < self.hold_until and m != self.mode:
                self.pending = (m, hold, cmd.get("text"))   # 正在保持的模式还没到时间：排队
            else:
                self._enter(m, hold)
        if "level" in cmd:
            self.level_t = float(cmd["level"])
        if "text" in cmd and not self.pending:
            self.text = clean(cmd["text"])

    def _enter(self, m, hold):
        if m != self.mode:
            self.mode_t = self.now
            self.text = ""                        # 换模式时清掉上一个模式的文字
        self.mode = m
        self.hold_until = self.now + hold
        st = STYLE[m]
        for k in ("w", "h", "ch", "bright", "fill", "ret", "stream"):
            self.tgt[k] = float(st[k])
        self.tcolor = st["color"]
        if st["eyes"] != self.pres_t:             # 眼睛要出现或要解体：重新撒一批方块
            self.pres_t = float(st["eyes"])
            self._new_particles()
        if m == "wake":
            self.flash_t = self.now
        if m == "track":
            self.lock_t = self.now
        if m not in ("listen", "speak"):
            self.level_t = 0.0

    def _new_particles(self):
        """眼睛由一格一格的方块组成；每个方块从数据流里随机一个位置飞过来（解体时飞回去）。"""
        self.particles = []
        for j in range(10):
            for i in range(12):
                u, v = (i + 0.5) / 12 * 2 - 1, (j + 0.5) / 10 * 2 - 1
                self.particles.append((u, v, self.rng.uniform(4, W - 8), self.rng.uniform(TOP, BOTTOM - 4),
                                       self.rng.uniform(0, 0.35)))

    # ---------------- 每帧
    def tick(self, now: float):
        dt = max(0.0, min(0.2, now - self.now))
        self.now = now
        if self.pending and now >= self.hold_until:
            m, hold, text = self.pending
            self.pending = None
            self._enter(m, hold)
            if text:
                self.text = clean(text)
        k = 1 - math.exp(-dt * 10)
        for key in self.cur:
            self.cur[key] += (self.tgt[key] - self.cur[key]) * k
        self.color = mix(self.color, self.tcolor, 1 - math.exp(-dt * 8))
        kf = 1 - math.exp(-dt * 8)
        active = STYLE[self.mode]["ov"]
        for ov in OVERLAYS:
            self.fade[ov] += ((1.0 if ov == active else 0.0) - self.fade[ov]) * kf
        self.level += (self.level_t - self.level) * (1 - math.exp(-dt * 15))
        old = self.pres                          # 眼睛拼合 0.6 秒、解体 0.5 秒（匀速，好控制节奏）
        step = dt / (0.6 if self.pres_t > self.pres else 0.5)
        self.pres = min(self.pres_t, self.pres + step) if self.pres_t > self.pres else max(self.pres_t, self.pres - step)
        if old < 1 <= self.pres:
            self.flash_t = now                   # 拼好的一瞬间闪一下
        if self.mode == "idle" and now > self.next_sweep:
            self.sweep_t = now
            self.next_sweep = now + self.rng.uniform(6, 10)
        self._update_stream(dt)
        self.draw()

    def _update_stream(self, dt):
        """数据流一直往上流：新的一行从屏幕底下进来；还有没进来的行就流得快一点，都进来了就慢慢停住。"""
        if self.mode in FROZEN:
            return
        rest = BOTTOM - LINE_H                                 # 最新一行“落定”的位置
        low = self.lines[-1]["y"] if self.lines else rest - LINE_H
        if self.queue and low <= rest:                          # 底下有空了：放下一行进来（先在屏幕外）
            self.lines.append({"s": self.queue.popleft(), "y": low + LINE_H, "typed": 0.0})
            low += LINE_H
        gap = low - rest
        if gap > 0:
            v = max(FLOW_MIN, gap * 2.5 + len(self.queue) * 4)
            step = min(gap, v * dt)
            for ln in self.lines:
                ln["y"] -= step
        for ln in self.lines:                                   # 逐字打出来
            if ln["typed"] < len(ln["s"]):
                ln["typed"] = min(len(ln["s"]), ln["typed"] + dt * (90 + 30 * len(self.queue)))
        self.lines = [ln for ln in self.lines if ln["y"] > TOP - LINE_H]

    # ---------------- 画
    def draw(self):
        g, c = self.g, self.color
        g.clear(BG)
        dx, dy = -self.cur["gx"] * GAZE_X, self.cur["gy"] * GAZE_Y
        ocx = CX + dx                                    # 两眼中间（跟着视线一起动）
        f = self.fade

        self._stream(c, self.cur["stream"])
        self._sweep(c)
        if f["scan"] > 0.02:
            self._scan(c, f["scan"])
        if f["thermal"] > 0.02:
            self._thermal(f["thermal"])
        if self.pres > 0.001:
            for side in (-1, 1):
                ex, ey = CX + side * EYE_DX + dx, CY + dy
                if self.pres >= 1:
                    self._eye(ex, ey, c)
                else:
                    self._eye_blocks(ex, ey, c)
        p = self.pres
        if f["online"] > 0.02:
            self._online(ocx, c, f["online"])
        if f["bars"] * p > 0.02:
            self._bars(ocx, c, f["bars"] * p)
        if f["think"] * p > 0.02:
            self._think(ocx, dx, dy, c, f["think"] * p)
        if f["wave"] * p > 0.02:
            self._wave(ocx, c, f["wave"] * p)
        if f["alert"] > 0.02:
            self._alert(c, f["alert"])
        if f["error"] > 0.02:
            self._error(c, f["error"])
        if f["sleep"] > 0.02:
            self._sleep(c, f["sleep"])
        if f["lowbat"] > 0.02:
            self._lowbat(c, f["lowbat"])
        self._edges(c)

    # ---- 数据流：细小的真实日志一直往上流
    def _stream(self, c, vis):
        """数据流：细小的字往上流，越往上越暗，到顶淡出；最新一行最亮，带打字光标。"""
        if vis < 0.02 or not self.lines:
            return
        g = self.g
        span = BOTTOM - TOP
        last = self.lines[-1]
        for ln in self.lines:
            y = ln["y"]
            height = (y - TOP) / span                           # 0 = 顶上，1 = 底下
            a = (0.15 + 0.55 * max(0.0, min(1.0, height)) ** 1.3) * vis
            if ln is last:
                a = 0.95 * vis
            shown = ln["s"][:int(ln["typed"])]
            g.text_small(12, round(y), shown, mix(BG, c, a), TOP, BOTTOM)
            if ln["typed"] < len(ln["s"]) and int(self.now * 10) % 2 == 0:
                g.fill_rect(12 + len(shown) * 5, round(y), 4, 5, mix(BG, c, a))   # 打字光标

    def _breath(self):
        if self.mode in ("idle", "wake", "listen", "track"):
            return 0.9 + 0.1 * math.sin(2 * math.pi * self.now / 4)
        return 1.0

    def _eye(self, cx, cy, c):
        g, p = self.g, self.cur
        flash = max(0.0, 1 - (self.now - self.flash_t) / 0.25)
        col = mix(c, WHITE, flash)
        bright = min(1.0, p["bright"] * self._breath() + flash)
        lit = mix(BG, col, bright)
        pts = chamfer_pts(cx, cy, p["w"], p["h"], p["ch"])
        if p["fill"] > 0.02:
            g.fill_poly(pts, mix(BG, lit, p["fill"]))
            if p["fill"] > 0.5 and p["h"] > 12:                            # 内圈细线，做出面板感
                g.poly(chamfer_pts(cx, cy, p["w"] - 7, p["h"] - 7, p["ch"] - 2), mix(BG, lit, 0.45))
        else:
            g.fill_poly(pts, BG)                                           # 空心准星框里挡住数据流
        g.poly(pts, lit)
        if p["ret"] > 0.02:                                                # 准星框
            r = p["ret"]
            lock = min(1.0, (self.now - self.lock_t) / 0.3)
            hw, hh = p["w"] / 2 + 4 + (1 - lock) * 12, p["h"] / 2 + 4 + (1 - lock) * 8
            rc = mix(BG, lit, r)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    x, y = cx + sx * hw, cy + sy * hh
                    g.line(x, y, x - sx * 7, y, rc)
                    g.line(x, y, x, y - sy * 7, rc)
            g.line(cx - 9, cy, cx - 3, cy, rc)
            g.line(cx + 3, cy, cx + 9, cy, rc)
            g.line(cx, cy - 9, cx, cy - 3, rc)
            g.line(cx, cy + 3, cx, cy + 9, rc)
            g.pixel(cx, cy, rc)

    def _eye_blocks(self, cx, cy, c):
        """眼睛拼合/解体的中间过程：一格一格的方块在数据流和眼睛之间飞。"""
        p, pres = self.cur, self.pres
        w, h = p["w"], p["h"]
        lim = w / 2 + h / 2 - p["ch"]
        hollow = p["fill"] < 0.5                                           # 准星框只拼外圈
        for u, v, sx, sy, delay in self.particles:
            tx, ty = cx + u * w / 2, cy + v * h / 2
            ax, ay = abs(u * w / 2), abs(v * h / 2)
            if ax + ay > lim:
                continue
            if hollow and ax < w / 2 - 4 and ay < h / 2 - 4 and ax + ay < lim - 4:
                continue
            q = max(0.0, min(1.0, (pres - delay) / 0.65))
            e = 1 - (1 - q) ** 3
            x, y = sx + (tx - sx) * e, sy + (ty - sy) * e
            self.g.fill_rect(x - 1, y - 1, 3, 3, mix(BG, c, 0.25 + 0.75 * q))

    def _sweep(self, c):
        p = (self.now - self.sweep_t) / 0.9
        if 0 <= p <= 1 and self.mode == "idle":
            x = p * W
            for i, a in enumerate((0.6, 0.3, 0.15)):
                self.g.line(x - i * 2, TOP - 1, x - i * 2, BOTTOM, mix(BG, c, a))

    def _scan(self, c, f):
        ph = ((self.now - self.mode_t) / 1.6) % 2
        x = (ph if ph < 1 else 2 - ph) * (W - 1)
        back = -1 if ph < 1 else 1
        for y in range(TOP + 1, BOTTOM, 6):                                # 淡淡的横向网格
            self.g.line(0, y, W, y, mix(BG, c, 0.1 * f))
        for i, a in enumerate((1.0, 0.45, 0.25, 0.12)):
            self.g.line(x + back * i * 2, TOP - 1, x + back * i * 2, BOTTOM, mix(BG, c, a * f))
        pct = int(((self.now - self.mode_t) / 3.2 * 100) % 100)
        s = f"SWEEP {pct:02d}%"
        self.g.fill_rect(CX - text_width(s) / 2 - 2, BOTTOM - 9, text_width(s) + 4, 9, BG)
        self.g.text(CX - text_width(s) / 2, BOTTOM - 8, s, mix(BG, c, f))

    def set_thermal(self, frame, sim=False):
        """热成像画面：h 行 × w 列的温度（°C），从左上角开始，列 0 = 狗的左边。
        ESP32 上是固件自己从 MLX90640 读到的那一帧；模拟器里是假房间。"""
        self.thermal, self.thermal_sim = frame, sim

    def _thermal(self, f):
        """热成像（全屏）：中间是热画面（每个点画成 3×2 像素），最热点用角标框出来；
        左边写最高、最低温度，右边是色标。画面左右镜像，像照镜子：狗左边的东西出现在你看到的右边。"""
        fr = self.thermal
        if not fr:
            s = "NO THERMAL DATA"
            self.g.text(CX - text_width(s) / 2, CY - 3, s, mix(BG, WHITE, f))
            return
        h, w = len(fr), len(fr[0])
        cw, chh = 3, 2
        x0, y0 = CX - w * cw // 2, CY - h * chh // 2 + 1
        lo = min(min(r) for r in fr)
        hi = max(max(r) for r in fr)
        span = max(hi - lo, 4.0)                                  # 温差太小时别把噪声放大成满屏花
        # 颜色按温度的平方根分配：有一个很烫的东西时，其他东西（人、热水）也还看得清
        hr = hc = 0
        for r in range(h):
            row = fr[r]
            for c in range(w):
                v = row[c]
                if v >= fr[hr][hc]:
                    hr, hc = r, c
                self.g.fill_rect(x0 + (w - 1 - c) * cw, y0 + r * chh, cw, chh, mix(BG, iron(((v - lo) / span) ** 0.5), f))
        bx, by = x0 + (w - 1 - hc) * cw + 1, y0 + hr * chh + 1      # 最热点：四个角标
        col = mix(BG, WHITE, f)
        for sx in (-1, 1):
            for sy in (-1, 1):
                cx, cy = bx + sx * 5, by + sy * 5
                self.g.line(cx, cy, cx - sx * 2, cy, col)
                self.g.line(cx, cy, cx, cy - sy * 2, col)
        lx = x0 - 8                                               # 左边：数字
        for k, (name, v) in enumerate((("MAX", hi), ("MIN", lo))):
            t = f"{name} {v:.1f}C"
            self.g.text(lx - text_width(t), CY - 10 + k * 11, t, mix(BG, WHITE if k == 0 else CYAN, f))
        if self.thermal_sim:
            self.g.text(lx - text_width("SIM"), CY + 12, "SIM", mix(BG, AMBER, f))
        rx = x0 + w * cw + 8                                      # 右边：色标
        top, bot = y0 + 2, y0 + h * chh - 2
        for y in range(top, bot):
            self.g.line(rx, y, rx + 3, y, mix(BG, iron((1 - (y - top) / (bot - top)) ** 0.5), f))
        self.g.text(rx + 7, top - 1, f"{hi:.0f}", mix(BG, WHITE, f))
        self.g.text(rx + 7, bot - 6, f"{lo:.0f}", mix(BG, CYAN, f))

    def _online(self, ocx, c, f):
        col = mix(BG, WHITE, f)
        w = text_width("ONLINE")
        self.g.fill_rect(ocx - w / 2 - 6, CY - 7, w + 12, 15, BG)
        self.g.text(ocx - w / 2, CY - 3, "ONLINE", col)
        for sx in (-1, 1):
            x = ocx + sx * (w / 2 + 4)
            self.g.line(x, CY - 6, x, CY + 6, mix(BG, c, f))
            self.g.line(x, CY - 6, x - sx * 3, CY - 6, mix(BG, c, f))
            self.g.line(x, CY + 6, x - sx * 3, CY + 6, mix(BG, c, f))

    def _bars(self, ocx, c, f):
        if self.now > self.bar_next:
            self.bar_rand = [self.rng.uniform(0.6, 1.0) for _ in self.bars]
            self.bar_next = self.now + 0.08
        shape = (0.45, 0.7, 0.9, 1.0, 0.9, 0.7, 0.45)
        for i in range(7):
            target = 2 + 26 * self.level * shape[i] * self.bar_rand[i]
            self.bars[i] += (target - self.bars[i]) * 0.5
            h = self.bars[i]
            self.g.fill_rect(ocx - 20 + i * 6, CY - h / 2, 4, h, mix(BG, c, f * (0.5 + 0.5 * shape[i])))

    def _think(self, ocx, dx, dy, c, f):
        k = int(self.now * 6) % 3
        for i in range(3):
            self.g.fill_rect(ocx - 9 + i * 7, CY - 2 + dy, 4, 4, mix(BG, c, f * (1.0 if i == k else 0.25)))
        p = self.cur
        for side in (-1, 1):                                               # 眼睛下面的进度条来回跑
            ex = CX + side * EYE_DX + dx
            x0, y = ex - p["w"] / 2, CY + dy + p["h"] / 2 + 5
            self.g.line(x0, y, x0 + p["w"], y, mix(BG, c, 0.25 * f))
            s = (self.now * 60) % (p["w"] + 12) - 12
            self.g.line(max(x0, x0 + s), y, min(x0 + p["w"], x0 + s + 12), y, mix(BG, c, f))

    def _wave(self, ocx, c, f):
        a = 2 + 13 * self.level
        pts = []
        for i in range(27):
            x = ocx - 26 + i * 2
            env = math.sin(math.pi * i / 26)
            y = CY + env * a * (0.7 * math.sin(x * 0.35 - self.now * 9) + 0.3 * math.sin(x * 0.9 + self.now * 5))
            pts.append((x, y))
        col = mix(BG, c, f)
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            self.g.line(x0, y0, x1, y1, col)

    def _box(self, w, h, c, f):
        """屏幕中间的提示框：挡住后面的数据流，带边框和切角。"""
        pts = chamfer_pts(CX, CY, w, h, 5)
        self.g.fill_poly(pts, BG)
        self.g.poly(pts, mix(BG, c, f))

    def _alert(self, c, f):
        on = (self.now * 2) % 1 < 0.6
        self._box(120, 34, c, f)
        col = mix(BG, c, f * (1.0 if on else 0.35))
        tri = [(CX - 38, CY - 10), (CX - 26, CY + 10), (CX - 50, CY + 10)]
        self.g.poly(tri, col)
        self.g.text(CX - 40, CY - 2, "!", col)
        msg = self.text or "WARNING"
        self.g.text(CX - 18, CY - 8, "WARNING", mix(BG, c, f))
        self.g.text(CX - 18, CY + 2, msg[:9], mix(BG, WHITE, f))
        off = int(self.now * 12) % 6                                       # 底部移动的警示斜纹
        for x in range(104 + off, 214, 6):
            self.g.line(x, H - 2, x + 3, H - 5, mix(BG, c, 0.6 * f))

    def _error(self, c, f):
        on = (self.now * 2) % 1 < 0.6
        self._box(110, 34, c, f)
        w = text_width("ERR", 2)
        self.g.text(CX - w / 2 - 22, CY - 7, "ERR", mix(BG, c, f * (1.0 if on else 0.3)), 2)
        self.g.text(CX + 14, CY - 3, (self.text or "SYS")[:6], mix(BG, WHITE, f))

    def _sleep(self, c, f):
        """休眠：数据流停了，中间只剩一个慢慢闪的光标和一条很暗的线。"""
        a = (0.3 + 0.25 * math.sin(2 * math.pi * self.now / 5)) * f
        self.g.line(CX - 60, CY, CX + 60, CY, mix(BG, c, a * 0.5))
        if int(self.now * 1.5) % 2 == 0:
            self.g.fill_rect(CX - 2, CY - 4, 5, 7, mix(BG, c, a))

    def _lowbat(self, c, f):
        on = self.now % 1 < 0.6
        bat = self.status.get("bat")
        lv = 0.1 if bat is None else max(0.0, min(1.0, bat / 100))
        self._box(96, 30, c, f)
        col = mix(BG, c, f)
        self.g.rect(CX - 38, CY - 6, 24, 12, col)
        self.g.fill_rect(CX - 14, CY - 2, 2, 4, col)
        if on:
            self.g.fill_rect(CX - 36, CY - 4, max(1, 20 * lv), 8, mix(BG, RED, f))
        s = "--%" if bat is None else f"{int(bat)}%"
        self.g.text(CX - 4, CY - 8, "CHARGE", mix(BG, c, f))
        self.g.text(CX - 4, CY + 2, s, mix(BG, WHITE, f))

    def _edges(self, c):
        g = self.g
        dim = 0.35 if self.mode == "sleep" else 1.0
        frame = mix(BG, c, 0.6 * dim)
        L = 8
        for x, sx in ((0, 1), (W - 1, -1)):                                # 四个角标
            for y, sy in ((0, 1), (H - 1, -1)):
                g.line(x, y, x + sx * L, y, frame)
                g.line(x, y, x, y + sy * L, frame)
        txt = mix(BG, c, 0.85 * dim)
        g.text(12, 2, LABEL[self.mode], txt)
        temp = self.status.get("temp")
        ts = "T --" if temp is None else f"T{temp:.1f}C" + ("*" if self.status.get("sim") else "")
        g.text(W - 12 - text_width(ts), 2, ts, txt)
        bat = self.status.get("bat")
        bs = "BAT --" if bat is None else f"BAT {int(bat)}%"
        g.text(W - 12 - text_width(bs), H - 9, bs, txt)
        if self.mode == "track":
            g.text(12, H - 9, f"X{self.cur['gx']:+.2f} Y{self.cur['gy']:+.2f}", txt)
        elif self.text and self.mode not in ("alert", "error"):
            g.text(12, H - 9, self.text, txt)
        if self.fade["alert"] < 0.5:                                       # 底部刻度尺
            for i, x in enumerate(range(104, 182, 6)):
                g.line(x, H - 2, x, H - (4 if i % 5 == 0 else 3), mix(BG, c, 0.35 * dim))


def iron(t):
    """铁红色伪彩：冷 = 黑紫，热 = 黄白。"""
    stops = ((0, 0, 0), (40, 0, 100), (170, 0, 140), (235, 70, 20), (255, 190, 0), (255, 255, 210))
    t = max(0.0, min(1.0, t)) * (len(stops) - 1)
    i = min(int(t), len(stops) - 2)
    return mix(stops[i], stops[i + 1], t - i)
