"""宽屏 HUD（机械狗007v）：渲染器 + 指令协议。按屏幕透过玻璃窗露出来的亮区 800×440 画。

屏幕：微雪 ESP32-S3-Touch-LCD-4.3B（800×480）。前脸玻璃窗 118×50 mm，屏幕亮区约 95×50 mm，
上下各有约 20 像素藏在框后面，所以只画中间 800×440（屏上 y = 20–460）。

画面思路（和旧长条屏一样）：平时是一台在工作的机器，全屏的真实数据流（大脑推过来的读数、事件、耗时）
一直平滑地往上流；有人和它打交道时（唤醒、聆听、思考、汇报、锁定目标），数据流里的方块往中间飞、
“拼”出两只眼睛；结束后眼睛“解体”成方块散开，回到数据流。
宽屏多出来的地方：上面一条写模式和电量，下面一条写速度、背上载重、温度（运输机器人更关心这些）。

风格（2026-10-24 改，参考车机屏）：深蓝灰到黑的渐变底、干净的细字体（数字大、单位小）、冰蓝色眼睛带柔和光晕；
电量绿、警告橙、错误红。旧的像素风存在 git 标签“屏幕旧风格存档”。界面文字有中文、英文两套（lang 指令切换），数据流日志一直是英文。

移植规则（以后照着改成屏幕板上的 C++，用 LVGL）：
  - 渲染器只通过 Gfx 画图：渐变底图、线、矩形、抗锯齿凸多边形、字（两种字体：窄体西文 + 中文黑体，LVGL 字库）、光晕。
  - 光晕是把形状模糊以后“加”到画面上（LVGL 的阴影或预先模糊好的贴图）；其余“淡入淡出”用 mix(背景色, 颜色, 程度)。
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
  log    往数据流里加一行（或一组行），最多 62 个字符，只用大写字母、数字和常用符号（机器日志）
  lang   界面文字用哪种：en 英文 / cn 中文（模式名、下栏标签、提示卡里的字）
"""
import math
import os
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
LOG_MAX = 62                  # 数据流一行最多几个字
LINE_H = 19                   # 数据流行高
LOG_SIZE = 15                 # 数据流字号
FLOW_MIN = 12.0               # 数据流最慢的流速（像素/秒）
LOG_X = 30

BG_MID, BG_EDGE = (34, 42, 60), (6, 8, 12)   # 底：中间深蓝灰、四周渐黑
BG = (14, 18, 26)             # 淡入淡出往这个颜色缩（底的平均色）
PANEL = (22, 27, 38)          # 提示卡片的底
CYAN = (120, 200, 255)        # 主色：冰蓝（配色 A；名字沿用 CYAN）
COOL = (140, 170, 255)        # 低温数字
AMBER = (255, 170, 40)
RED = (255, 70, 70)
GREEN = (64, 210, 120)
WHITE = (240, 246, 255)
TEXT = (232, 238, 246)        # 主要文字
SUB = (132, 146, 168)         # 次要文字、单位、标签
LINE = (58, 72, 96)           # 分隔线、刻度

MODES = ("idle", "wake", "listen", "think", "speak", "track", "scan", "thermal",
         "alert", "error", "sleep", "low_bat")
LABEL = {"idle": "IDLE", "wake": "WAKE", "listen": "LISTEN", "think": "THINK", "speak": "SPEAK",
         "track": "TRACK", "scan": "SCAN", "thermal": "THERMAL", "alert": "ALERT", "error": "ERROR",
         "sleep": "SLEEP", "low_bat": "LOW BAT"}
DEFAULT_HOLD = {"wake": 0.9}
LANGS = ("en", "cn")
UI = {"en": dict(LABEL, spd="SPD", load="LOAD", temp="TEMP", online="ONLINE", sweep="SWEEP", warning="WARNING",
                 error="ERROR", charge="CHARGE", max="MAX", min="MIN", nothermal="NO THERMAL DATA"),
      "cn": dict(idle="待机", wake="唤醒", listen="聆听", think="思考", speak="汇报", track="跟踪", scan="扫描",
                 thermal="热成像", alert="警告", error="故障", sleep="休眠", low_bat="电量低",
                 spd="速度", load="载重", temp="温度", online="已上线", sweep="扫描", warning="注意",
                 charge="请充电", max="最高", min="最低", nothermal="没有热成像数据")}

# 每个模式：眼睛形状（宽、高、圆角、亮度、实心程度、准星程度、斜切 cut、弯弯 arc）、眼睛出不出现、数据流亮度、颜色、叠加层
# 眼睛照效果图：平时是发光的圆角方块；唤醒时弯成 ∩（打招呼）；警告时变琥珀色、上沿往里斜（凶）
_E = dict(w=120, h=104, r=24, bright=0.85, fill=1, ret=0, cut=0, arc=0)
STYLE = {
    "idle":    dict(_E, eyes=0, stream=1.0,  color=CYAN,  ov=None),
    "wake":    dict(_E, bright=1.0, arc=1, eyes=1, stream=0.4, color=CYAN, ov="online"),
    "listen":  dict(_E, bright=0.9, eyes=1, stream=0.4, color=CYAN, ov="bars"),
    "think":   dict(_E, w=124, h=30, r=14, eyes=1, stream=0.45, color=CYAN, ov="think"),
    "speak":   dict(_E, h=96, bright=0.9, eyes=1, stream=0.4, color=CYAN, ov="wave"),
    "track":   dict(_E, w=110, h=110, r=22, bright=1.0, fill=0, ret=1, eyes=1, stream=0.4, color=CYAN, ov=None),
    "scan":    dict(_E, eyes=0, stream=0.6, color=CYAN,  ov="scan"),
    "thermal": dict(_E, eyes=0, stream=0.0,  color=CYAN,  ov="thermal"),
    "alert":   dict(_E, bright=1.0, cut=1, eyes=1, stream=0.55, color=AMBER, ov="alert"),
    "error":   dict(_E, eyes=0, stream=0.5,  color=RED,   ov="error"),
    "sleep":   dict(_E, eyes=0, stream=0.0,  color=CYAN,  ov="sleep"),
    "low_bat": dict(_E, eyes=0, stream=0.4, color=AMBER, ov="lowbat"),
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


# ---------------------------------------------------------------- 字体
# 模拟器用电脑上已有的字体（按顺序找，找到哪个用哪个）；屏幕板上换成能免费商用的同类字体（DIN 一类的窄体 + 中文黑体），
# 用 LVGL 的字体转换工具做成精简字库（中文只放用得到的字）。
FONT_FILES = {
    "light": ("C:/Windows/Fonts/DINNextLTPro-Light.ttf", "C:/Windows/Fonts/bahnschrift.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    "regular": ("C:/Windows/Fonts/DINNextLTPro-Regular.ttf", "C:/Windows/Fonts/bahnschrift.ttf",
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    "medium": ("C:/Windows/Fonts/DINNextLTPro-Medium.ttf", "C:/Windows/Fonts/bahnschrift.ttf",
               "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"),
    "cn": ("C:/Windows/Fonts/msyh.ttc", "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
           "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"),
}
LOG_CHARS = set("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 .%-+:/!*_=<>(),#[]?")


def clean(s: str) -> str:
    """数据流的一行：大写，只留机器日志用的字符（不认识的变空格）。"""
    return "".join(ch if ch in LOG_CHARS else " " for ch in str(s).upper())[:LOG_MAX]


def clean_text(s: str) -> str:
    """提示文字（text 字段）：中英文都可以，去掉换行和控制字符。"""
    return "".join(ch for ch in str(s) if ch.isprintable())[:14]


class Gfx:
    """画图接口：渲染器只能用这些（屏幕板上一一对应 LVGL：渐变底图、抗锯齿多边形、线、圆角矩形、字体、叠加发光）。
    模拟器用 pygame。"""

    def __init__(self, surface):
        import pygame
        import pygame.gfxdraw
        self.pg, self.gd, self.s = pygame, pygame.gfxdraw, surface
        pygame.font.init()
        self._fonts = {}
        self._bg = None

    # ---- 底：中间深蓝灰、四周渐黑（参考车机屏）；只算一次
    def background(self, k=1.0):
        if self._bg is None:
            import numpy as np
            y, x = np.mgrid[0:H, 0:W].astype(np.float32)
            d = np.sqrt(((x - W / 2) / (W * 0.62)) ** 2 + ((y - H * 0.52) / (H * 0.8)) ** 2)
            t = np.clip(d, 0, 1) ** 1.35
            img = np.array(BG_MID)[None, None, :] * (1 - t[..., None]) + np.array(BG_EDGE)[None, None, :] * t[..., None]
            self._bg = self.pg.surfarray.make_surface(img.transpose(1, 0, 2).astype(np.uint8))
            self._bg_dark = self._bg.copy()
            self._bg_dark.fill((90, 90, 90), special_flags=self.pg.BLEND_RGB_MULT)
        self.s.blit(self._bg if k > 0.5 else self._bg_dark, (0, 0))

    def clear(self, c):
        self.s.fill(c)

    def line(self, x0, y0, x1, y1, c, w=1):
        if w <= 1:
            self.pg.draw.aaline(self.s, c, (x0, y0), (x1, y1))
        else:
            self.pg.draw.line(self.s, c, (int(x0), int(y0)), (int(x1), int(y1)), int(w))

    def rect(self, x, y, w, h, c, t=1):
        self.pg.draw.rect(self.s, c, (int(x), int(y), max(1, int(w)), max(1, int(h))), max(1, int(t)))

    def fill_rect(self, x, y, w, h, c):
        self.pg.draw.rect(self.s, c, (int(x), int(y), max(1, int(w)), max(1, int(h))))

    def fill_poly(self, pts, c):        # 凸多边形，边缘抗锯齿
        p = [(int(round(x)), int(round(y))) for x, y in pts]
        if len(p) >= 3:
            self.gd.filled_polygon(self.s, p, c)
            self.gd.aapolygon(self.s, p, c)

    def poly(self, pts, c, w=1):
        if w <= 1:
            self.pg.draw.aalines(self.s, c, True, pts)
        else:
            self.pg.draw.lines(self.s, c, True, [(int(x), int(y)) for x, y in pts], int(w))

    def circle(self, x, y, r, c, w=1):
        self.pg.draw.circle(self.s, c, (int(x), int(y)), int(r), max(1, int(w)))

    def glow(self, shapes, c, radius=16, k=0.6):
        """柔和的光晕：把形状（("poly", 点) / ("line", x0, y0, x1, y1, 粗细)）画到一张小图上、模糊、再“加”到画面上。
        屏幕板上：LVGL 的阴影（shadow）或预先模糊好的光晕贴图，用叠加模式画。"""
        pg = self.pg
        xs, ys = [], []
        for sh in shapes:
            if sh[0] == "poly":
                xs += [p[0] for p in sh[1]]; ys += [p[1] for p in sh[1]]
            else:
                xs += [sh[1], sh[3]]; ys += [sh[2], sh[4]]
        pad = radius * 2 + 8
        x0, y0 = int(min(xs)) - pad, int(min(ys)) - pad
        w, h = int(max(xs)) + pad - x0, int(max(ys)) + pad - y0
        lay = pg.Surface((w, h))
        col = tuple(int(v * k) for v in c)
        for sh in shapes:
            if sh[0] == "poly":
                pg.draw.polygon(lay, col, [(x - x0, y - y0) for x, y in sh[1]])
            else:
                pg.draw.line(lay, col, (sh[1] - x0, sh[2] - y0), (sh[3] - x0, sh[4] - y0), int(sh[5]))
        f = max(2, radius // 3)
        small = pg.transform.smoothscale(lay, (max(1, w // f), max(1, h // f)))
        small = pg.transform.smoothscale(small, (max(1, w // (f * 2)), max(1, h // (f * 2))))
        lay = pg.transform.smoothscale(small, (w, h))
        self.s.blit(lay, (x0, y0), special_flags=pg.BLEND_RGB_ADD)

    # ---- 字
    def font(self, size, weight="regular"):
        key = (size, weight)
        f = self._fonts.get(key)
        if f is None:
            path = next((p for p in FONT_FILES[weight] if os.path.exists(p)), None)
            f = self.pg.font.Font(path, size)
            self._fonts[key] = f
        return f

    def _pick(self, s, size, weight):
        return self.font(size, "cn" if not s.isascii() else weight)

    def measure(self, s, size, weight="regular"):
        return self._pick(s, size, weight).size(s)[0] if s else 0

    def text(self, x, y, s, c, size=18, weight="regular", anchor="lt", clip=None):
        """写字。anchor：第 1 个字母 l / m / r 横向，第 2 个 t / m / b 纵向。返回字宽。"""
        if not s:
            return 0
        f = self._pick(s, size, weight)
        img = f.render(s, True, c)
        w, h = img.get_size()
        ax, ay = anchor[0], anchor[1]
        x -= w / 2 if ax == "m" else w if ax == "r" else 0
        y -= h / 2 if ay == "m" else h if ay == "b" else 0
        if clip:
            self.s.set_clip((0, int(clip[0]), W, int(clip[1] - clip[0])))
        self.s.blit(img, (int(x), int(y)))
        if clip:
            self.s.set_clip(None)
        return w


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
    bad = set(cmd) - {"mode", "gaze", "level", "text", "hold", "status", "log", "face", "lang"}
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
    if "lang" in cmd and cmd["lang"] not in LANGS:
        return f"lang 只能是 {LANGS}"
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
        self.lang = "en"
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
        if cmd.get("lang") in LANGS:
            self.lang = cmd["lang"]
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
            self.text = clean_text(cmd["text"])

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
                self.text = clean_text(text)
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
    def ui(self, key):
        return UI[self.lang].get(key, UI["en"][key])

    def draw(self):
        g, c = self.g, self.color
        g.background(0.0 if self.mode == "sleep" else 1.0)
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
        """很淡的背景点阵，让屏幕有“在工作”的质感（休眠时不画）。"""
        if self.mode == "sleep":
            return
        col = mix(BG, SUB, 0.12)
        for y in range(TOP + 12, BOTTOM, 24):
            for x in range(24, W - 12, 24):
                self.g.fill_rect(x, y, 2, 2, col)

    # ---- 数据流：细小的真实日志一直往上流
    def _stream(self, c, vis):
        """数据流：字往上流，越往上越淡，到顶淡出；最新一行最亮，前面一个小方块，带打字光标。"""
        if vis < 0.02 or not self.lines:
            return
        g = self.g
        span = BOTTOM - TOP
        last = self.lines[-1]
        for ln in self.lines:
            y = ln["y"]
            height = max(0.0, min(1.0, (y - TOP) / span))           # 0 = 顶上，1 = 底下
            a = (0.12 + 0.7 * height ** 1.4) * vis
            if ln is last:
                a = 0.95 * vis
            shown = ln["s"][:int(ln["typed"])]
            col = mix(BG, SUB if ln is not last else TEXT, a)
            if ln is last:
                g.fill_rect(LOG_X - 12, round(y) + 6, 5, 5, mix(BG, c, a))
            w = g.text(LOG_X, round(y), shown, col, LOG_SIZE, "light", clip=(TOP, BOTTOM))
            if ln["typed"] < len(ln["s"]) and int(self.now * 10) % 2 == 0:
                g.fill_rect(LOG_X + w + 2, round(y) + 3, 7, LOG_SIZE - 3, col)   # 打字光标

    def _breath(self):
        if self.mode in ("idle", "wake", "listen", "track"):
            return 0.9 + 0.1 * math.sin(2 * math.pi * self.now / 4)
        return 1.0

    def _eye(self, cx, cy, c, inner):
        """一只眼睛。inner = 鼻子在哪边（斜切往那边低）。arc 过半就画成弯弯的 ∩，否则画圆角方块。"""
        g, p = self.g, self.cur
        flash = max(0.0, 1 - (self.now - self.flash_t) / 0.25)
        col = mix(c, WHITE, flash)
        bright = min(1.0, p["bright"] * self._breath() + flash)
        lit = dim(col, 0.7 + 0.3 * bright)                                 # 亮度只在 70–100% 之间呼吸，眼睛不发灰
        arc = p["arc"]
        if arc > 0.5:                                                      # 开心：∩，从扁方块“弯”过来
            k = (arc - 0.5) * 2
            w, h, t = p["w"] * (0.9 + 0.2 * k), p["h"] * (0.2 + 0.25 * k), 6 + 7 * k
            pts = arc_pts(cx, cy + p["h"] * 0.1, w, h)
            g.glow([("line", *a, *b, t + 4) for a, b in zip(pts, pts[1:])], col, 18, 0.55 * bright)
            for a, b in zip(pts, pts[1:]):
                g.line(*a, *b, lit, t)
            for x, y in (pts[0], pts[-1]):                                  # 两头圆一点
                g.fill_poly(rrect_pts(x, y, t, t, t / 2, 3), lit)
            return
        squash = 1 - arc * 1.4                                              # 往 ∩ 变的前半段：先压扁
        w, h, r, cut = p["w"], max(10.0, p["h"] * squash), min(p["r"], p["h"] * squash / 2), p["cut"]
        pts = eye_pts(cx, cy, w, h, r, cut, inner)
        g.glow([("poly", pts)], col, 20, 0.55 * bright)
        if p["fill"] > 0.02:
            g.fill_poly(pts, dim(lit, p["fill"]))
            if p["fill"] > 0.5 and h > 40:                                 # 上沿一道细高光（屏幕面板的光泽）
                top = clip_above(eye_pts(cx, cy, w - 32, h - 16, max(2, r - 10), cut, inner), cx + w, cy - h / 2 + 13, cx - w, cy - h / 2 + 13)
                if len(top) >= 3:
                    g.fill_poly(top, mix(lit, WHITE, 0.45))
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
                    g.line(x, y, x - sx * 21, y, rc, 2)
                    g.line(x, y, x, y - sy * 21, rc, 2)
            g.line(cx - 27, cy, cx - 9, cy, rc, 2)
            g.line(cx + 9, cy, cx + 27, cy, rc, 2)
            g.line(cx, cy - 27, cx, cy - 9, rc, 2)
            g.line(cx, cy + 9, cx, cy + 27, rc, 2)
            g.fill_poly(rrect_pts(cx, cy, 5, 5, 2, 2), rc)

    def _eye_blocks(self, cx, cy, c):
        """眼睛拼合/解体的中间过程：一格一格的小方块在数据流和眼睛之间飞（拼成圆角方块，拼好再变表情）。"""
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
            s = 3 + 4 * q
            self.g.fill_rect(x - s / 2, y - s / 2, s, s, dim(c, 0.25 + 0.75 * q))

    def _sweep(self, c):
        p = (self.now - self.sweep_t) / 1.1
        if 0 <= p <= 1 and self.mode == "idle":
            x = p * W
            for i, a in enumerate((0.45, 0.22, 0.1, 0.05)):
                self.g.line(x - i * 6, TOP, x - i * 6, BOTTOM, dim(c, a), 2)

    def _panel(self, cx, cy, w, h, c, f, r=12):
        """提示卡片：深色圆角底（挡住后面的数据流）+ 细边框 + 一圈很淡的光。"""
        pts = rrect_pts(cx, cy, w, h, r)
        self.g.glow([("poly", pts)], c, 14, 0.25 * f)
        self.g.fill_poly(pts, mix(BG, PANEL, f))
        self.g.poly(pts, dim(c, f), 2)

    def _scan(self, c, f):
        ph = ((self.now - self.mode_t) / 1.6) % 2
        x = (ph if ph < 1 else 2 - ph) * (W - 1)
        back = -1 if ph < 1 else 1
        for y in range(TOP + 6, BOTTOM, 18):                               # 淡淡的横向网格
            self.g.line(0, y, W, y, dim(c, 0.08 * f))
        for i, a in enumerate((1.0, 0.45, 0.25, 0.12, 0.06)):
            self.g.line(x + back * i * 6, TOP, x + back * i * 6, BOTTOM, dim(c, a * f), 2)
        pct = int(((self.now - self.mode_t) / 3.2 * 100) % 100)
        lab, num = self.ui("sweep"), f"{pct:02d}%"
        tw = self.g.measure(lab, 15) + 10 + self.g.measure(num, 24, "medium")
        self._panel(CX, BOTTOM - 26, tw + 36, 40, c, f, 10)
        x0 = CX - tw / 2
        x0 += self.g.text(x0, BOTTOM - 26, lab, mix(BG, SUB, f), 15, anchor="lm") + 10
        self.g.text(x0, BOTTOM - 26, num, mix(BG, TEXT, f), 24, "medium", anchor="lm")

    def set_thermal(self, frame, sim=False):
        """热成像画面：h 行 × w 列的温度（°C），从左上角开始，列 0 = 机器人的左边。
        屏幕板上是传感器板转过来的那一帧（或大脑转发）；模拟器里是假房间。"""
        self.thermal, self.thermal_sim = frame, sim

    def _thermal(self, f):
        """热成像（全屏）：中间是热画面（每个点画成 9×9 像素），最热点用角标框出来；
        左边大字写最高、最低温度，右边是色标。画面左右镜像，像照镜子：它左边的东西出现在你看到的右边。"""
        g = self.g
        fr = self.thermal
        if not fr:
            g.text(CX, CY, self.ui("nothermal"), mix(BG, SUB, f), 20, anchor="mm")
            return
        h, w = len(fr), len(fr[0])
        cs = 9
        x0, y0 = CX - w * cs // 2, CY - h * cs // 2 + 2
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
                g.fill_rect(x0 + (w - 1 - c) * cs, y0 + r * cs, cs, cs, mix(BG, iron(((v - lo) / span) ** 0.5), f))
        fc = mix(BG, TEXT, 0.8 * f)                               # 画面四个角的取景框
        for sx, sy, x, y in ((1, 1, x0 - 6, y0 - 6), (-1, 1, x0 + w * cs + 6, y0 - 6), (1, -1, x0 - 6, y0 + h * cs + 6), (-1, -1, x0 + w * cs + 6, y0 + h * cs + 6)):
            g.line(x, y, x + sx * 16, y, fc, 2)
            g.line(x, y, x, y + sy * 16, fc, 2)
        bx, by = x0 + (w - 1 - hc) * cs + cs // 2, y0 + hr * cs + cs // 2      # 最热点：四个角标
        col = mix(BG, WHITE, f)
        for sx in (-1, 1):
            for sy in (-1, 1):
                cx, cy = bx + sx * 15, by + sy * 15
                g.line(cx, cy, cx - sx * 7, cy, col, 2)
                g.line(cx, cy, cx, cy - sy * 7, col, 2)
        g.text(bx + 20, by - 15, f"{hi:.1f}°", col, 14, "medium")
        lx = x0 - 46                                              # 左边：最高、最低（标签小、数字大）
        for k, (key, v, vc) in enumerate((("max", hi, AMBER), ("min", lo, COOL))):
            y = CY - 46 + k * 74
            g.text(lx, y, self.ui(key), mix(BG, SUB, f), 13, anchor="rt")
            vw = g.measure("°C", 14)
            g.text(lx - vw - 4, y + 18, f"{v:.1f}", mix(BG, vc, f), 32, "medium", anchor="rt")
            g.text(lx, y + 32, "°C", mix(BG, SUB, f), 14, anchor="rt")
        if self.thermal_sim:
            g.text(lx, CY + 76, "SIM", mix(BG, AMBER, f), 13, "medium", anchor="rt")
        rx = x0 + w * cs + 30                                     # 右边：色标
        top, bot = y0, y0 + h * cs
        for y in range(top, bot, 2):
            g.fill_rect(rx, y, 8, 2, mix(BG, iron((1 - (y - top) / (bot - top)) ** 0.5), f))
        g.text(rx + 16, top, f"{hi:.0f}", mix(BG, SUB, f), 14)
        g.text(rx + 16, bot, f"{lo:.0f}", mix(BG, SUB, f), 14, anchor="lb")

    def _online(self, ocx, c, f):
        s = self.ui("online")
        w = self.g.measure(s, 20, "medium")
        self._panel(ocx, OV_Y, w + 44, 40, c, f, 10)
        self.g.fill_poly(rrect_pts(ocx - w / 2 - 8, OV_Y, 6, 6, 3, 3), mix(BG, GREEN, f))
        self.g.text(ocx + 6, OV_Y, s, mix(BG, TEXT, f), 20, "medium", anchor="mm")

    def _bars(self, ocx, c, f):
        if self.now > self.bar_next:
            self.bar_rand = [self.rng.uniform(0.6, 1.0) for _ in self.bars]
            self.bar_next = self.now + 0.08
        shape = (0.35, 0.55, 0.75, 0.9, 1.0, 0.9, 0.75, 0.55, 0.35)
        n = len(shape)
        for i in range(n):
            target = 6 + 50 * self.level * shape[i] * self.bar_rand[i]
            self.bars[i] += (target - self.bars[i]) * 0.5
            h = self.bars[i]
            x = ocx - (n * 15 - 15) / 2 + i * 15
            self.g.fill_poly(rrect_pts(x, OV_Y, 6, h, 3, 3), dim(c, f * (0.5 + 0.5 * shape[i])))

    def _think(self, ocx, dx, dy, c, f):
        k = int(self.now * 6) % 3
        for i in range(3):
            self.g.fill_poly(rrect_pts(ocx - 20 + i * 20, EYE_Y + dy, 9, 9, 4.5, 4), dim(c, f * (1.0 if i == k else 0.25)))
        p = self.cur
        for side in (-1, 1):                                               # 眼睛下面的进度条来回跑
            ex = CX + side * EYE_DX + dx
            x0, y = ex - p["w"] / 2, EYE_Y + dy + p["h"] / 2 + 18
            self.g.line(x0, y, x0 + p["w"], y, dim(c, 0.2 * f), 2)
            s = (self.now * 180) % (p["w"] + 36) - 36
            self.g.line(max(x0, x0 + s), y, min(x0 + p["w"], x0 + s + 36), y, dim(c, f), 2)

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
            self.g.line(x0, y0, x1, y1, col, 2)

    def _warn_icon(self, x, y, s, col, back):
        self.g.fill_poly([(x, y - s * 0.55), (x + s * 0.6, y + s * 0.45), (x - s * 0.6, y + s * 0.45)], col)
        self.g.fill_rect(x - 1.5, y - s * 0.22, 3, s * 0.38, back)
        self.g.fill_rect(x - 1.5, y + s * 0.24, 3, 3, back)

    def _alert(self, c, f):
        """警告：琥珀色的“凶”眼睛，下面一张提示卡（闪烁的警告三角 + 原因），底部移动的斜纹。"""
        on = (self.now * 2) % 1 < 0.6
        msg = self.text or self.ui("warning")
        tw = self.g.measure(msg, 20, "medium")
        self._panel(CX, OV_Y, tw + 90, 46, c, f, 10)
        x0 = CX - (tw + 40) / 2
        self._warn_icon(x0 + 10, OV_Y, 24, dim(c, f * (1.0 if on else 0.35)), mix(BG, PANEL, f))
        self.g.text(x0 + 40, OV_Y, msg, mix(BG, TEXT, f), 20, "medium", anchor="lm")
        off = int(self.now * 36) % 24
        for x in range(CX - 200 + off, CX + 200, 24):
            self.g.fill_poly([(x, BOTTOM - 4), (x + 10, BOTTOM - 4), (x + 22, BOTTOM - 16), (x + 12, BOTTOM - 16)], dim(c, 0.5 * f))

    def _error(self, c, f):
        on = (self.now * 2) % 1 < 0.6
        self._panel(CX, CY, 380, 120, c, f, 14)
        self._warn_icon(CX - 130, CY, 40, dim(c, f * (1.0 if on else 0.3)), mix(BG, PANEL, f))
        self.g.text(CX - 92, CY - 16, self.ui("error"), mix(BG, SUB, f), 15, anchor="lm")
        self.g.text(CX - 92, CY + 12, self.text or "SYS", mix(BG, TEXT, f), 28, "medium", anchor="lm")

    def _sleep(self, c, f):
        """休眠：数据流停了、底变暗，中间只剩一个慢慢闪的光标和一条很暗的线。"""
        a = (0.3 + 0.25 * math.sin(2 * math.pi * self.now / 5)) * f
        self.g.line(CX - 180, CY, CX + 180, CY, dim(c, a * 0.5), 1)
        if int(self.now * 1.5) % 2 == 0:
            self.g.fill_poly(rrect_pts(CX, CY - 2, 10, 20, 2, 2), dim(c, a))

    def _lowbat(self, c, f):
        on = self.now % 1 < 0.6
        bat = self.status.get("bat")
        lv = 0.1 if bat is None else max(0.0, min(1.0, bat / 100))
        self._panel(CX, CY, 330, 110, c, f, 14)
        col = mix(BG, SUB, f)
        bx, by = CX - 128, CY - 20
        self.g.fill_poly(rrect_pts(bx + 40, by + 20, 80, 40, 6), col)
        self.g.fill_poly(rrect_pts(bx + 40, by + 20, 74, 34, 4), mix(BG, PANEL, f))
        self.g.fill_rect(bx + 81, by + 13, 5, 14, col)
        if on:
            self.g.fill_poly(rrect_pts(bx + 7 + max(4, 66 * lv) / 2, by + 20, max(4, 66 * lv), 28, 3), mix(BG, RED, f))
        s = "--" if bat is None else f"{int(bat)}"
        self.g.text(CX - 18, CY - 16, self.ui("charge"), mix(BG, SUB, f), 15, anchor="lm")
        w = self.g.text(CX - 18, CY + 14, s, mix(BG, TEXT, f), 30, "medium", anchor="lm")
        self.g.text(CX - 18 + w + 3, CY + 19, "%", mix(BG, SUB, f), 15, anchor="lm")

    # ---- 上下两条状态栏和四角
    def _edges(self, c):
        g = self.g
        dm = 0.35 if self.mode == "sleep" else 1.0
        acc = dim(c, 0.9 * dm)
        L = 18
        for x, sx in ((6, 1), (W - 7, -1)):                                # 四个角标
            for y, sy in ((6, 1), (H - 7, -1)):
                g.line(x, y, x + sx * L, y, acc, 2)
                g.line(x, y, x, y + sy * L, acc, 2)
        rule = mix(BG, LINE, dm)                                           # 上下分隔线：两头斜切
        for y, sy in ((TOP - 2, 1), (BOTTOM + 2, -1)):
            g.line(40, y, W - 40, y, rule)
            g.line(40, y, 30, y - sy * 8, rule)
            g.line(W - 40, y, W - 30, y - sy * 8, rule)
            g.line(30, y - sy * 8, 14, y - sy * 8, rule)
            g.line(W - 30, y - sy * 8, W - 14, y - sy * 8, rule)
        txt = mix(BG, TEXT, dm)
        sub = mix(BG, SUB, dm)
        # 上：模式（竖条 + 大字）、名字、电量
        g.fill_poly(rrect_pts(29, 27, 6, 20, 3, 3), acc)
        g.text(42, 27, self.ui(self.mode), txt, 22, "medium", anchor="lm")
        name = "007V"
        g.text(CX, 27, name, sub, 15, "medium", anchor="mm")
        for i in range(-3, 4):                                             # 名字两边的小刻度
            if i:
                x = CX + i * 8 + (36 if i > 0 else -36)
                g.line(x, 23, x, 31, mix(BG, LINE, dm))
        bat = self.status.get("bat")
        bs = "--" if bat is None else f"{int(bat)}"
        pw = g.measure("%", 13)
        g.text(W - 30, 30, "%", sub, 13, anchor="rm")
        bw = g.text(W - 30 - pw - 2, 27, bs, txt, 22, "medium", anchor="rm")
        ix = W - 30 - pw - 2 - bw - 52                                     # 电池图标
        lv = 0 if bat is None else max(0.0, min(1.0, bat / 100))
        g.fill_poly(rrect_pts(ix + 20, 27, 40, 18, 4), sub)
        g.fill_poly(rrect_pts(ix + 20, 27, 37, 15, 3), mix(BG, BG_MID, 0.7))
        g.fill_rect(ix + 40, 23, 3, 8, sub)
        if lv > 0.02:
            g.fill_poly(rrect_pts(ix + 3 + 34 * lv / 2, 27, 34 * lv, 11, 2), mix(BG, AMBER if lv < 0.2 else GREEN, dm))
        # 下：左边文字或坐标，右边速度、载重、温度（标签小、数字大、单位小）
        if self.mode == "track":
            g.text(30, H - 27, f"X {self.cur['gx']:+.2f}   Y {self.cur['gy']:+.2f}", txt, 18, anchor="lm")
        elif self.text and self.mode not in ("alert", "error"):
            g.text(30, H - 27, self.text, txt, 18, anchor="lm")
        spd, load, temp = self.status.get("spd"), self.status.get("load"), self.status.get("temp")
        items = [(self.ui("spd"), "--" if spd is None else f"{spd:.1f}", "m/s"),
                 (self.ui("load"), "--" if load is None else f"{load:.1f}", "kg"),
                 (self.ui("temp"), "--" if temp is None else f"{temp:.1f}" + ("*" if self.status.get("sim") else ""), "°C")]
        x = W - 30
        for label, val, unit in reversed(items):
            x -= g.text(x, H - 24, unit, sub, 13, anchor="rm") + 4
            x -= g.text(x, H - 27, val, txt, 26, "medium", anchor="rm") + 8
            x -= g.text(x, H - 24, label, sub, 12, anchor="rm") + 30
        if self.fade["alert"] < 0.5:                                       # 底部刻度尺
            for i, xx in enumerate(range(CX - 150, CX - 60, 6)):
                g.line(xx, H - 8, xx, H - (16 if i % 5 == 0 else 12), mix(BG, LINE, dm))


def iron(t):
    """铁红色伪彩：冷 = 黑紫，热 = 黄白。"""
    stops = ((0, 0, 0), (40, 0, 100), (170, 0, 140), (235, 70, 20), (255, 190, 0), (255, 255, 210))
    t = max(0.0, min(1.0, t)) * (len(stops) - 1)
    i = min(int(t), len(stops) - 2)
    return mix(stops[i], stops[i + 1], t - i)
