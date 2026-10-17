"""宽屏 HUD（机械狗007v）：渲染器 + 指令协议。按屏幕透过玻璃窗露出来的亮区 800×440 画。

屏幕：微雪 ESP32-S3-Touch-LCD-4.3B（800×480）。前脸玻璃窗 118×50 mm，屏幕亮区约 95×50 mm，
上下各有约 20 像素藏在框后面，所以只画中间 800×440（屏上 y = 20–460）。

画面思路（和旧长条屏一样）：平时是一台在工作的机器，全屏的真实数据流（大脑推过来的读数、事件、耗时）
一直平滑地往上流；有人和它打交道时（唤醒、聆听、思考、汇报、锁定目标），数据流里的方块往中间飞、
“拼”出两只眼睛；结束后眼睛“解体”成方块散开，回到数据流。
宽屏多出来的地方：上面一条写模式和电量，下面一条写速度、背上载重、温度（运输机器人更关心这些）。

移植规则（以后照着改成屏幕板上的 C++，LVGL 的 canvas 或直接画帧缓冲都行）：
  - 渲染器只通过 Gfx 画图：点、线（可带粗细）、矩形、实心凸多边形、圆、两套位图字体（可整数倍放大）。
  - 没有透明混合。“发光”是往外画几圈越来越暗的线；“淡入淡出”都用 mix(背景色, 颜色, 程度) 来做。
  - 所有动画只依赖“现在时间”和每帧的 dt，30 帧。

指令（每条一行 JSON，字段都可以省略，只更新给出的部分）：
  {"mode": "track", "gaze": [0.3, -0.1], "level": 0.6, "text": "E-NET", "hold": 2.5, "face": "happy",
   "status": {"temp": 22.1, "sim": true, "bat": 87, "spd": 0.8, "load": 6.5}, "log": "FACE 1 X-0.40"}
  mode   见 MODES
  gaze   [x, y]，-1–1。x 按机器人自己的方向：-1 = 它的左边（面对屏幕时眼睛往你的右边移），y：-1 上，+1 下
  level  0–1：聆听时是麦克风音量，汇报时是语音音量
  text   左下角的短文字（换模式时自动清掉）
  hold   这个模式至少保持几秒，期间来的模式切换先排队（唤醒默认 0.9 秒）
  face   表情（可省略，换模式时回到这个模式默认的）：normal 圆角方块 / happy 弯弯的 ∩ / angry 上沿往里斜（凶）
  status 上下两条显示的状态：temp 温度（sim=true 表示假数据，后面带 *）、bat 电量 %、spd 速度 m/s、load 背上载重 kg（null = 不知道）
  log    往数据流里加一行（或一组行），最多 72 个字符，只能用字体里有的字符
"""
import math
import random
from collections import deque

W, H = 800, 440
CX, CY = W // 2, H // 2
U = 3                         # 一个“格”多少像素：旧长条屏的尺寸 × 3
EYE_DX = 130                  # 眼睛中心到屏幕中线的距离（照效果图：两眼在屏幕 1/3 和 2/3 附近）
EYE_Y = CY - 22               # 眼睛中心的高度（比正中略高，下面留一行给音量条、声波、提示字）
OV_Y = CY + 96                # 眼睛下面那一行的中心
GAZE_X, GAZE_Y = 60, 24       # gaze = ±1 时眼睛移动多少像素
TOP, BOTTOM = 56, 384         # 中间区域（上下两条状态栏）
LOG_MAX = 72                  # 数据流一行最多几个字
SS = 2                        # 数据流小字体放大倍数（每字 10 像素宽、10 像素高）
LINE_H = 14                   # 数据流行高
FLOW_MIN = 12.0               # 数据流最慢的流速（像素/秒）
LOG_X = 30

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

# 每个模式：眼睛形状（宽、高、圆角、亮度、实心程度、准星程度、斜切 cut、弯弯 arc）、眼睛出不出现、数据流亮度、颜色、叠加层
# 眼睛照效果图：平时是发光的圆角方块；唤醒时弯成 ∩（打招呼）；警告时变琥珀色、上沿往里斜（凶）
_E = dict(w=120, h=104, r=24, bright=0.85, fill=1, ret=0, cut=0, arc=0)
STYLE = {
    "idle":    dict(_E, eyes=0, stream=1.0,  color=CYAN,  ov=None),
    "wake":    dict(_E, bright=1.0, arc=1, eyes=1, stream=0.25, color=CYAN, ov="online"),
    "listen":  dict(_E, bright=0.9, eyes=1, stream=0.2, color=CYAN, ov="bars"),
    "think":   dict(_E, w=124, h=30, r=14, eyes=1, stream=0.3, color=CYAN, ov="think"),
    "speak":   dict(_E, h=96, bright=0.9, eyes=1, stream=0.2, color=CYAN, ov="wave"),
    "track":   dict(_E, w=110, h=110, r=22, bright=1.0, fill=0, ret=1, eyes=1, stream=0.2, color=CYAN, ov=None),
    "scan":    dict(_E, eyes=0, stream=0.45, color=CYAN,  ov="scan"),
    "thermal": dict(_E, eyes=0, stream=0.0,  color=CYAN,  ov="thermal"),
    "alert":   dict(_E, bright=1.0, cut=1, eyes=1, stream=0.5, color=AMBER, ov="alert"),
    "error":   dict(_E, eyes=0, stream=0.5,  color=RED,   ov="error"),
    "sleep":   dict(_E, eyes=0, stream=0.0,  color=CYAN,  ov="sleep"),
    "low_bat": dict(_E, eyes=0, stream=0.25, color=AMBER, ov="lowbat"),
}
FACES = {"normal": dict(cut=0, arc=0), "happy": dict(cut=0, arc=1), "angry": dict(cut=1, arc=0)}
SHAPE = ("w", "h", "r", "bright", "fill", "ret", "cut", "arc", "stream")
OVERLAYS = ("online", "bars", "think", "wave", "scan", "thermal", "alert", "error", "sleep", "lowbat")
FROZEN = ("alert", "error")   # 警告、错误时数据流卡住不动


def mix(a, b, t):
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    return (int(a[0] + (b[0] - a[0]) * t), int(a[1] + (b[1] - a[1]) * t), int(a[2] + (b[2] - a[2]) * t))


def dim(c, a):
    """颜色往背景缩，亮度分 24 档（屏幕板上可以查表，模拟器可以缓存字形）。"""
    return mix(BG, c, round(max(0.0, min(1.0, a)) * 24) / 24)


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

# 数据流用的小字体：4 像素宽、5 像素高，放大 SS 倍画
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


def small_width(s, scale=SS):
    return max(0, len(s) * 5 * scale - scale)


def clean(s: str) -> str:
    """转成字体能显示的样子：大写，不认识的字符变空格。"""
    return "".join(ch if ch in GLYPHS else " " for ch in str(s).upper())[:LOG_MAX]


class Gfx:
    """画图接口：渲染器只能用这些（屏幕板上一一对应 LVGL canvas / 帧缓冲的画点、画线、填矩形、填三角形）。
    模拟器用 pygame；字形按 (字, 颜色, 倍数) 缓存成小图，免得每帧一个点一个点地画。"""

    def __init__(self, surface):
        import pygame
        self.pg, self.s = pygame, surface
        self._cache = {}

    def clear(self, c):
        self.s.fill(c)

    def pixel(self, x, y, c):
        if 0 <= x < W and 0 <= y < H:
            self.s.set_at((int(x), int(y)), c)

    def line(self, x0, y0, x1, y1, c, w=1):
        self.pg.draw.line(self.s, c, (int(x0), int(y0)), (int(x1), int(y1)), max(1, int(w)))

    def rect(self, x, y, w, h, c, t=1):
        self.pg.draw.rect(self.s, c, (int(x), int(y), max(1, int(w)), max(1, int(h))), max(1, int(t)))

    def fill_rect(self, x, y, w, h, c):
        self.pg.draw.rect(self.s, c, (int(x), int(y), max(1, int(w)), max(1, int(h))))

    def fill_poly(self, pts, c):        # 只用凸多边形（屏幕板上拆成三角形）
        self.pg.draw.polygon(self.s, c, [(int(x), int(y)) for x, y in pts])

    def poly(self, pts, c, w=1):
        self.pg.draw.lines(self.s, c, True, [(int(x), int(y)) for x, y in pts], max(1, int(w)))

    def circle(self, x, y, r, c, w=1):
        self.pg.draw.circle(self.s, c, (int(x), int(y)), int(r), max(1, int(w)))

    def _glyph(self, font, ch, c, scale):
        key = (id(font), ch, c, scale)
        img = self._cache.get(key)
        if img is None:
            g = font.get(ch, font[" "])
            img = self.pg.Surface((len(g[0]) * scale, len(g) * scale))
            img.set_colorkey((0, 0, 0))
            for r, row in enumerate(g):
                for col, on in enumerate(row):
                    if on:
                        img.fill(c, (col * scale, r * scale, scale, scale))
            if len(self._cache) > 6000:
                self._cache.clear()
            self._cache[key] = img
        return img

    def text_small(self, x, y, s, c, top=0, bottom=H, scale=SS):
        """数据流用的 4×5 小字体，超出 top–bottom 的部分不画（流到顶上就被切掉）。"""
        self.s.set_clip((0, int(top), W, int(bottom - top)))
        for i, ch in enumerate(s):
            if ch != " ":
                self.s.blit(self._glyph(SMALL, ch, c, scale), (int(x + i * 5 * scale), int(y)))
        self.s.set_clip(None)

    def text(self, x, y, s, c, scale=1):
        for i, ch in enumerate(s.upper()):
            if ch != " ":
                self.s.blit(self._glyph(GLYPHS, ch, c, scale), (int(x + i * 6 * scale), int(y)))


def chamfer_pts(cx, cy, w, h, ch):
    """带切角的方块（八边形），和机身、屏幕框的斜切角一致。"""
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    c = max(0.0, min(ch, w / 2, h / 2))
    return [(x0 + c, y0), (x1 - c, y0), (x1, y0 + c), (x1, y1 - c), (x1 - c, y1), (x0 + c, y1), (x0, y1 - c), (x0, y0 + c)]


def rrect_pts(cx, cy, w, h, r, n=6):
    """圆角方块（凸多边形，每个圆角 n 段），屏幕板上一样拆成三角形填。"""
    r = max(0.0, min(r, w / 2, h / 2))
    pts = []
    for qx, qy, a0 in ((1, -1, -90), (1, 1, 0), (-1, 1, 90), (-1, -1, 180)):
        ox, oy = cx + qx * (w / 2 - r), cy + qy * (h / 2 - r)
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    return pts


def clip_above(pts, x0, y0, x1, y1):
    """凸多边形切掉直线 (x0,y0)-(x1,y1) 上面的部分（结果还是凸多边形）。"""
    def side(p):
        return (x1 - x0) * (p[1] - y0) - (y1 - y0) * (p[0] - x0)
    out = []
    for i, a in enumerate(pts):
        b = pts[(i + 1) % len(pts)]
        sa, sb = side(a), side(b)
        if sa >= 0:
            out.append(a)
        if (sa >= 0) != (sb >= 0):
            t = sa / (sa - sb)
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return out


def eye_pts(cx, cy, w, h, r, cut, inner):
    """一只眼睛的外形。cut 0–1：上沿往鼻子那边斜下去（凶），inner = 鼻子在哪边（-1 左 / +1 右）。"""
    pts = rrect_pts(cx, cy, w, h, r)
    if cut > 0.01:
        drop = h * 0.42 * cut
        top = cy - h / 2
        a, b = (cx - inner * w / 2 - 2, top - 2), (cx + inner * w / 2 + 2, top + drop)
        pts = clip_above(pts, *a, *b) if inner > 0 else clip_above(pts, *b, *a)
    return pts


def arc_pts(cx, cy, w, h, n=16):
    """弯弯的 ∩ 眼睛的中心线（半个椭圆，开口朝下）。"""
    return [(cx - w / 2 * math.cos(math.pi * k / n), cy + h / 2 - h * math.sin(math.pi * k / n)) for k in range(n + 1)]


# ---------------------------------------------------------------- 指令检查（模拟器和屏幕板都按这个来）
def check(cmd) -> str:
    if not isinstance(cmd, dict):
        return "指令不是 JSON 对象"
    bad = set(cmd) - {"mode", "gaze", "level", "text", "hold", "status", "log", "face"}
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
    if "face" in cmd and cmd["face"] not in FACES:
        return f"face 只能是 {tuple(FACES)}"
    if "status" in cmd and not isinstance(cmd["status"], dict):
        return "status 要是 JSON 对象"
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
        self.cur = {k: float(st[k]) for k in SHAPE}
        self.cur.update(gx=0.0, gy=0.0)
        self.tgt = dict(self.cur)
        self.color, self.tcolor = st["color"], st["color"]
        self.fade = {k: 0.0 for k in OVERLAYS}
        self.level, self.level_t = 0.0, 0.0
        self.text = ""
        self.status = {"temp": None, "sim": False, "bat": None, "spd": None, "load": None}
        self.flash_t = self.lock_t = -9.0
        self.sweep_t, self.next_sweep = -9.0, now + 4
        self.bars = [0.0] * 9
        self.bar_rand = [1.0] * 9
        self.bar_next = 0.0
        # 眼睛出现/消失：pres 0 = 没有眼睛，1 = 完整的眼睛；中间是方块飞来飞去
        self.pres, self.pres_t = 0.0, 0.0
        self.particles = []                       # [(u, v, 起点 x, 起点 y, 延迟)]
        self._new_particles()
        # 数据流
        self.lines = []                           # 屏幕上的行：{"s": 文字, "y": 位置, "typed": 已经打出几个字}
        self.queue = deque(maxlen=60)             # 还没进屏幕的行
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
        if "face" in cmd:
            self.tgt.update({k: float(v) for k, v in FACES[cmd["face"]].items()})
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
        for k in SHAPE:
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
        for j in range(12):
            for i in range(15):
                u, v = (i + 0.5) / 15 * 2 - 1, (j + 0.5) / 12 * 2 - 1
                self.particles.append((u, v, self.rng.uniform(LOG_X, W - 30), self.rng.uniform(TOP, BOTTOM - 10),
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
            v = max(FLOW_MIN, gap * 2.5 + len(self.queue) * 10)
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

        self._grid(c)
        self._stream(c, self.cur["stream"])
        self._sweep(c)
        if f["scan"] > 0.02:
            self._scan(c, f["scan"])
        if f["thermal"] > 0.02:
            self._thermal(f["thermal"])
        if self.pres > 0.001:
            for side in (-1, 1):
                ex, ey = CX + side * EYE_DX + dx, EYE_Y + dy
                if self.pres >= 1:
                    self._eye(ex, ey, c, -side)
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

    def _grid(self, c):
        """很淡的背景点阵，让黑屏有“屏幕在工作”的质感（休眠时不画）。"""
        if self.mode == "sleep":
            return
        col = dim(c, 0.07)
        for y in range(TOP + 12, BOTTOM, 24):
            for x in range(24, W - 12, 24):
                self.g.fill_rect(x, y, 2, 2, col)

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
            a = (0.12 + 0.5 * max(0.0, min(1.0, height)) ** 1.3) * vis
            if ln is last:
                a = 0.95 * vis
            shown = ln["s"][:int(ln["typed"])]
            col = dim(c, a)
            if ln is last:                                      # 最新一行前面的小方块
                g.fill_rect(LOG_X - 12, round(y) + 3, 4, 4, col)
            g.text_small(LOG_X, round(y), shown, col, TOP, BOTTOM)
            if ln["typed"] < len(ln["s"]) and int(self.now * 10) % 2 == 0:
                g.fill_rect(LOG_X + len(shown) * 5 * SS, round(y), 4 * SS, 5 * SS, col)   # 打字光标

    def _breath(self):
        if self.mode in ("idle", "wake", "listen", "track"):
            return 0.9 + 0.1 * math.sin(2 * math.pi * self.now / 4)
        return 1.0

    def _glow(self, cx, cy, w, h, r, c, a, n=6, step=3, cut=0.0, inner=1):
        """发光：往外画几圈越来越暗的轮廓（不用透明，屏幕板上照样能画）。"""
        for i in range(n, 0, -1):
            self.g.poly(eye_pts(cx, cy, w + 2 * i * step, h + 2 * i * step, r + i * step, cut, inner),
                        dim(c, a * 0.32 * (1 - i / (n + 1)) ** 1.5), step)

    def _eye(self, cx, cy, c, inner):
        """一只眼睛。inner = 鼻子在哪边（斜切往那边低）。arc 过半就画成弯弯的 ∩，否则画圆角方块。"""
        g, p = self.g, self.cur
        flash = max(0.0, 1 - (self.now - self.flash_t) / 0.25)
        col = mix(c, WHITE, flash)
        bright = min(1.0, p["bright"] * self._breath() + flash)
        lit = dim(col, bright)
        arc = p["arc"]
        if arc > 0.5:                                                      # 开心：∩，从扁方块“弯”过来
            k = (arc - 0.5) * 2
            w, h, t = p["w"] * (0.9 + 0.2 * k), p["h"] * (0.2 + 0.25 * k), 6 + 7 * k
            pts = arc_pts(cx, cy + p["h"] * 0.1, w, h)
            for i in range(5, 0, -1):                                       # 发光：同一条弧，越外越粗越暗
                glow = dim(col, bright * 0.3 * (1 - i / 6) ** 1.5)
                for a, b in zip(pts, pts[1:]):
                    g.line(*a, *b, glow, t + i * 6)
            for a, b in zip(pts, pts[1:]):
                g.line(*a, *b, lit, t)
            for x, y in (pts[0], pts[-1]):                                  # 两头圆一点
                g.fill_poly(rrect_pts(x, y, t, t, t / 2, 3), lit)
            return
        squash = 1 - arc * 1.4                                              # 往 ∩ 变的前半段：先压扁
        w, h, r, cut = p["w"], max(10.0, p["h"] * squash), min(p["r"], p["h"] * squash / 2), p["cut"]
        pts = eye_pts(cx, cy, w, h, r, cut, inner)
        self._glow(cx, cy, w, h, r, col, bright, cut=cut, inner=inner)
        if p["fill"] > 0.02:
            g.fill_poly(pts, dim(lit, p["fill"]))
            if p["fill"] > 0.5 and h > 40:                                 # 上沿一条亮边，做出屏幕面板的光泽
                hi = clip_above(eye_pts(cx, cy, w - 16, h - 16, max(2, r - 8), cut, inner), cx + w, cy - h / 2 + 16, cx - w, cy - h / 2 + 16)   # 只留上沿一窄条
                if len(hi) >= 3:
                    g.fill_poly(hi, mix(lit, WHITE, 0.3))
        else:
            g.fill_poly(pts, BG)                                           # 空心准星框里挡住数据流
        g.poly(pts, lit, 3)
        if p["ret"] > 0.02:                                                # 准星框
            rt = p["ret"]
            lock = min(1.0, (self.now - self.lock_t) / 0.3)
            hw, hh = w / 2 + 12 + (1 - lock) * 36, h / 2 + 12 + (1 - lock) * 24
            rc = dim(lit, rt)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    x, y = cx + sx * hw, cy + sy * hh
                    g.line(x, y, x - sx * 21, y, rc, 3)
                    g.line(x, y, x, y - sy * 21, rc, 3)
            g.line(cx - 27, cy, cx - 9, cy, rc, 3)
            g.line(cx + 9, cy, cx + 27, cy, rc, 3)
            g.line(cx, cy - 27, cx, cy - 9, rc, 3)
            g.line(cx, cy + 9, cx, cy + 27, rc, 3)
            g.fill_rect(cx - 2, cy - 2, 5, 5, rc)

    def _eye_blocks(self, cx, cy, c):
        """眼睛拼合/解体的中间过程：一格一格的方块在数据流和眼睛之间飞（拼成圆角方块，拼好再变表情）。"""
        p, pres = self.cur, self.pres
        w, h, r = p["w"], max(p["h"], 40.0), p["r"]
        hollow = p["fill"] < 0.5                                           # 准星框只拼外圈
        for u, v, sx, sy, delay in self.particles:
            tx, ty = cx + u * w / 2, cy + v * h / 2
            ax, ay = abs(u * w / 2), abs(v * h / 2)
            ex, ey = ax - (w / 2 - r), ay - (h / 2 - r)
            if ex > 0 and ey > 0 and ex * ex + ey * ey > r * r:            # 圆角外面
                continue
            if hollow and ax < w / 2 - 12 and ay < h / 2 - 12:
                continue
            q = max(0.0, min(1.0, (pres - delay) / 0.65))
            e = 1 - (1 - q) ** 3
            x, y = sx + (tx - sx) * e, sy + (ty - sy) * e
            s = 4 + 3 * q
            self.g.fill_rect(x - s / 2, y - s / 2, s, s, dim(c, 0.25 + 0.75 * q))

    def _sweep(self, c):
        p = (self.now - self.sweep_t) / 1.1
        if 0 <= p <= 1 and self.mode == "idle":
            x = p * W
            for i, a in enumerate((0.6, 0.3, 0.15, 0.07)):
                self.g.line(x - i * 6, TOP, x - i * 6, BOTTOM, dim(c, a), 3)

    def _scan(self, c, f):
        ph = ((self.now - self.mode_t) / 1.6) % 2
        x = (ph if ph < 1 else 2 - ph) * (W - 1)
        back = -1 if ph < 1 else 1
        for y in range(TOP + 6, BOTTOM, 18):                               # 淡淡的横向网格
            self.g.line(0, y, W, y, dim(c, 0.1 * f))
        for i, a in enumerate((1.0, 0.45, 0.25, 0.12, 0.06)):
            self.g.line(x + back * i * 6, TOP, x + back * i * 6, BOTTOM, dim(c, a * f), 3)
        pct = int(((self.now - self.mode_t) / 3.2 * 100) % 100)
        s = f"SWEEP {pct:02d}%"
        tw = text_width(s, 3)
        self.g.fill_rect(CX - tw / 2 - 10, BOTTOM - 40, tw + 20, 34, BG)
        self.g.rect(CX - tw / 2 - 10, BOTTOM - 40, tw + 20, 34, dim(c, 0.5 * f), 2)
        self.g.text(CX - tw / 2, BOTTOM - 33, s, dim(c, f), 3)

    def set_thermal(self, frame, sim=False):
        """热成像画面：h 行 × w 列的温度（°C），从左上角开始，列 0 = 机器人的左边。
        屏幕板上是传感器板转过来的那一帧（或大脑转发）；模拟器里是假房间。"""
        self.thermal, self.thermal_sim = frame, sim

    def _thermal(self, f):
        """热成像（全屏）：中间是热画面（每个点画成 10×10 像素），最热点用角标框出来；
        左边写最高、最低温度，右边是色标。画面左右镜像，像照镜子：它左边的东西出现在你看到的右边。"""
        fr = self.thermal
        if not fr:
            s = "NO THERMAL DATA"
            self.g.text(CX - text_width(s, 3) / 2, CY - 10, s, dim(WHITE, f), 3)
            return
        h, w = len(fr), len(fr[0])
        cs = 10
        x0, y0 = CX - w * cs // 2, CY - h * cs // 2
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
                self.g.fill_rect(x0 + (w - 1 - c) * cs, y0 + r * cs, cs, cs, mix(BG, iron(((v - lo) / span) ** 0.5), f))
        self.g.rect(x0 - 4, y0 - 4, w * cs + 8, h * cs + 8, dim(CYAN, 0.5 * f), 2)
        bx, by = x0 + (w - 1 - hc) * cs + cs // 2, y0 + hr * cs + cs // 2      # 最热点：四个角标
        col = dim(WHITE, f)
        for sx in (-1, 1):
            for sy in (-1, 1):
                cx, cy = bx + sx * 16, by + sy * 16
                self.g.line(cx, cy, cx - sx * 8, cy, col, 3)
                self.g.line(cx, cy, cx, cy - sy * 8, col, 3)
        lx = x0 - 28                                              # 左边：数字
        for k, (name, v) in enumerate((("MAX", hi), ("MIN", lo))):
            t = f"{name} {v:.1f}C"
            self.g.text(lx - text_width(t, 3), CY - 34 + k * 38, t, dim(WHITE if k == 0 else CYAN, f), 3)
        if self.thermal_sim:
            self.g.text(lx - text_width("SIM", 2), CY + 50, "SIM", dim(AMBER, f), 2)
        rx = x0 + w * cs + 28                                     # 右边：色标
        top, bot = y0, y0 + h * cs
        for y in range(top, bot, 2):
            self.g.fill_rect(rx, y, 14, 2, mix(BG, iron((1 - (y - top) / (bot - top)) ** 0.5), f))
        self.g.text(rx + 24, top, f"{hi:.0f}", dim(WHITE, f), 3)
        self.g.text(rx + 24, bot - 21, f"{lo:.0f}", dim(CYAN, f), 3)

    def _online(self, ocx, c, f):
        col = dim(WHITE, f)
        w = text_width("ONLINE", 3)
        self.g.fill_rect(ocx - w / 2 - 18, OV_Y - 20, w + 36, 40, BG)
        self.g.text(ocx - w / 2, OV_Y - 10, "ONLINE", col, 3)
        for sx in (-1, 1):
            x = ocx + sx * (w / 2 + 12)
            self.g.line(x, OV_Y - 16, x, OV_Y + 16, dim(c, f), 3)
            self.g.line(x, OV_Y - 16, x - sx * 9, OV_Y - 16, dim(c, f), 3)
            self.g.line(x, OV_Y + 16, x - sx * 9, OV_Y + 16, dim(c, f), 3)

    def _bars(self, ocx, c, f):
        if self.now > self.bar_next:
            self.bar_rand = [self.rng.uniform(0.6, 1.0) for _ in self.bars]
            self.bar_next = self.now + 0.08
        shape = (0.35, 0.55, 0.75, 0.9, 1.0, 0.9, 0.75, 0.55, 0.35)
        n = len(shape)
        for i in range(n):
            target = 4 + 52 * self.level * shape[i] * self.bar_rand[i]
            self.bars[i] += (target - self.bars[i]) * 0.5
            h = self.bars[i]
            self.g.fill_rect(ocx - (n * 16 - 6) / 2 + i * 16, OV_Y - h / 2, 10, h, dim(c, f * (0.5 + 0.5 * shape[i])))

    def _think(self, ocx, dx, dy, c, f):
        k = int(self.now * 6) % 3
        for i in range(3):
            self.g.fill_rect(ocx - 27 + i * 21, EYE_Y - 6 + dy, 12, 12, dim(c, f * (1.0 if i == k else 0.25)))
        p = self.cur
        for side in (-1, 1):                                               # 眼睛下面的进度条来回跑
            ex = CX + side * EYE_DX + dx
            x0, y = ex - p["w"] / 2, EYE_Y + dy + p["h"] / 2 + 18
            self.g.line(x0, y, x0 + p["w"], y, dim(c, 0.25 * f), 3)
            s = (self.now * 180) % (p["w"] + 36) - 36
            self.g.line(max(x0, x0 + s), y, min(x0 + p["w"], x0 + s + 36), y, dim(c, f), 3)

    def _wave(self, ocx, c, f):
        a = 4 + 26 * self.level
        pts = []
        for i in range(81):
            x = ocx - 120 + i * 3
            env = math.sin(math.pi * i / 80)
            y = OV_Y + env * a * (0.7 * math.sin(x * 0.11 - self.now * 9) + 0.3 * math.sin(x * 0.3 + self.now * 5))
            pts.append((x, y))
        col = dim(c, f)
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            self.g.line(x0, y0, x1, y1, col, 3)

    def _box(self, w, h, c, f):
        """屏幕中间的提示框：挡住后面的数据流，带边框和切角。"""
        pts = chamfer_pts(CX, CY, w, h, 15)
        self._glow(CX, CY, w, h, 15, c, f, n=4)
        self.g.fill_poly(pts, BG)
        self.g.poly(pts, dim(c, f), 3)

    def _alert(self, c, f):
        """警告：琥珀色的“凶”眼睛（效果图那样），下面一行闪烁的警告三角 + 原因，底部移动的斜纹。"""
        on = (self.now * 2) % 1 < 0.6
        col = dim(c, f * (1.0 if on else 0.35))
        msg = (self.text or "WARNING")[:12]
        tw = text_width(msg, 3)
        x0 = CX - (tw + 56) / 2
        self.g.fill_rect(x0 - 14, OV_Y - 24, tw + 84, 48, BG)
        self.g.fill_poly([(x0 + 18, OV_Y - 18), (x0 + 38, OV_Y + 16), (x0 - 2, OV_Y + 16)], col)
        self.g.fill_rect(x0 + 16, OV_Y - 6, 4, 12, BG)
        self.g.fill_rect(x0 + 16, OV_Y + 9, 4, 4, BG)
        self.g.text(x0 + 56, OV_Y - 10, msg, dim(WHITE, f), 3)
        off = int(self.now * 36) % 24
        for x in range(CX - 200 + off, CX + 200, 24):
            self.g.fill_poly([(x, BOTTOM - 4), (x + 10, BOTTOM - 4), (x + 22, BOTTOM - 16), (x + 12, BOTTOM - 16)], dim(c, 0.6 * f))

    def _error(self, c, f):
        on = (self.now * 2) % 1 < 0.6
        self._box(380, 120, c, f)
        w = text_width("ERR", 6)
        self.g.text(CX - w / 2 - 70, CY - 21, "ERR", dim(c, f * (1.0 if on else 0.3)), 6)
        self.g.text(CX + 40, CY - 10, (self.text or "SYS")[:6], dim(WHITE, f), 3)

    def _sleep(self, c, f):
        """休眠：数据流停了，中间只剩一个慢慢闪的光标和一条很暗的线。"""
        a = (0.3 + 0.25 * math.sin(2 * math.pi * self.now / 5)) * f
        self.g.line(CX - 180, CY, CX + 180, CY, dim(c, a * 0.5), 2)
        if int(self.now * 1.5) % 2 == 0:
            self.g.fill_rect(CX - 6, CY - 12, 14, 21, dim(c, a))

    def _lowbat(self, c, f):
        on = self.now % 1 < 0.6
        bat = self.status.get("bat")
        lv = 0.1 if bat is None else max(0.0, min(1.0, bat / 100))
        self._box(320, 110, c, f)
        col = dim(c, f)
        self.g.rect(CX - 125, CY - 21, 84, 42, col, 3)
        self.g.fill_rect(CX - 41, CY - 8, 8, 16, col)
        if on:
            self.g.fill_rect(CX - 118, CY - 14, max(3, 70 * lv), 28, mix(BG, RED, f))
        s = "--%" if bat is None else f"{int(bat)}%"
        self.g.text(CX - 12, CY - 28, "CHARGE", dim(c, f), 3)
        self.g.text(CX - 12, CY + 6, s, dim(WHITE, f), 3)

    # ---- 上下两条状态栏和四角
    def _edges(self, c):
        g = self.g
        dm = 0.35 if self.mode == "sleep" else 1.0
        frame = dim(c, 0.6 * dm)
        L = 26
        for x, sx in ((1, 1), (W - 2, -1)):                                # 四个角标
            for y, sy in ((1, 1), (H - 2, -1)):
                g.line(x, y, x + sx * L, y, frame, 3)
                g.line(x, y, x, y + sy * L, frame, 3)
        rule = dim(c, 0.22 * dm)                                           # 上下分隔线：两头斜切
        for y, sy in ((TOP - 8, 1), (BOTTOM + 6, -1)):
            g.line(40, y, W - 40, y, rule)
            g.line(40, y, 30, y - sy * 10, rule)
            g.line(W - 40, y, W - 30, y - sy * 10, rule)
        txt = dim(c, 0.9 * dm)
        sub = dim(c, 0.5 * dm)
        # 上：模式（大字）、名字、电量
        g.fill_poly(chamfer_pts(36, 25, 10, 18, 3), txt)
        g.text(50, 15, LABEL[self.mode], txt, 3)
        name = "007V"
        g.text(CX - text_width(name, 2) / 2, 18, name, sub, 2)
        for i in range(-3, 4):                                             # 名字两边的小刻度
            if i:
                x = CX + i * 14 + (30 if i > 0 else -30)
                g.line(x, 24 - (3 if i % 3 == 0 else 1), x, 26 + (3 if i % 3 == 0 else 1), sub)
        bat = self.status.get("bat")
        bs = "--%" if bat is None else f"{int(bat)}%"
        bx = W - 30 - text_width(bs, 3)
        g.text(bx, 15, bs, txt, 3)
        ix = bx - 58                                                       # 电池图标
        g.rect(ix, 15, 44, 21, txt, 2)
        g.fill_rect(ix + 44, 21, 4, 9, txt)
        lv = 0 if bat is None else max(0.0, min(1.0, bat / 100))
        for k in range(5):
            if lv > k / 5 + 0.02:
                g.fill_rect(ix + 4 + k * 8, 19, 6, 13, AMBER if lv < 0.2 else txt)
        # 下：左边文字或坐标，右边速度、载重、温度
        if self.mode == "track":
            g.text(30, H - 38, f"X{self.cur['gx']:+.2f} Y{self.cur['gy']:+.2f}", txt, 3)
        elif self.text and self.mode not in ("alert", "error"):
            g.text(30, H - 38, self.text, txt, 3)
        spd, load, temp = self.status.get("spd"), self.status.get("load"), self.status.get("temp")
        items = [("SPD", "--" if spd is None else f"{spd:.1f}", "M/S"),
                 ("LOAD", "--" if load is None else f"{load:.1f}", "KG"),
                 ("T", "--" if temp is None else f"{temp:.1f}" + ("*" if self.status.get("sim") else ""), "C")]
        x = W - 30
        for label, val, unit in reversed(items):
            x -= small_width(unit)
            g.text_small(x, H - 26, unit, sub, scale=SS)
            x -= 6 + text_width(val, 3)
            g.text(x, H - 38, val, txt, 3)
            x -= 8 + small_width(label)
            g.text_small(x, H - 26, label, sub, scale=SS)
            x -= 26
        if self.fade["alert"] < 0.5:                                       # 底部刻度尺
            for i, xx in enumerate(range(CX - 150, CX - 30, 10)):
                g.line(xx, H - 6, xx, H - (14 if i % 5 == 0 else 10), dim(c, 0.35 * dm))


def iron(t):
    """铁红色伪彩：冷 = 黑紫，热 = 黄白。"""
    stops = ((0, 0, 0), (40, 0, 100), (170, 0, 140), (235, 70, 20), (255, 190, 0), (255, 255, 210))
    t = max(0.0, min(1.0, t)) * (len(stops) - 1)
    i = min(int(t), len(stops) - 2)
    return mix(stops[i], stops[i + 1], t - i)
