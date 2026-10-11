"""热成像：头部 MLX90640（32×24 个温度点），ESP32 每秒发一帧给大脑，大脑在这里分析。

一帧在串口上的样子（一行 JSON，约 1.1 KB）：
  {"ev":"thermal","w":32,"h":24,"min":21.3,"max":55.2,"px":"<base64>"}
px 是 w×h 个字节（从左上角开始一行一行排），每个字节 0–255 表示在 min 和 max 之间的位置，
所以精度是 (max - min) / 255，一般 0.1–0.3 °C，够用。

分析：找最热、最冷，把比周围热一截的区域圈成“热源”，算出它在哪个方向，再按温度粗分类（只是猜测）。
模拟器里没有真模块时，用 FakeScene 造一个假房间：走来走去的人、一杯热水、一扇冷窗户，按 H 打开“灶台”。
"""
import base64
import math

import numpy as np

W, H = 32, 24


# ---------------------------------------------------------------- 编码 / 解码（ESP32 固件照着 encode 写）
def encode(frame: np.ndarray) -> dict:
    lo, hi = float(frame.min()), float(frame.max())
    span = max(hi - lo, 0.01)
    px = np.clip(np.round((frame - lo) / span * 255), 0, 255).astype(np.uint8)
    return {"ev": "thermal", "w": int(frame.shape[1]), "h": int(frame.shape[0]),
            "min": round(lo, 2), "max": round(hi, 2), "px": base64.b64encode(px.tobytes()).decode("ascii")}


def decode(ev: dict) -> np.ndarray:
    w, h = ev["w"], ev["h"]
    px = np.frombuffer(base64.b64decode(ev["px"]), np.uint8).astype(np.float32).reshape(h, w)
    return ev["min"] + px / 255.0 * (ev["max"] - ev["min"])


def check(ev: dict) -> str:
    """协议检查（protocol.check_event 会调用）。"""
    w, h = ev.get("w"), ev.get("h")
    if not (isinstance(w, int) and isinstance(h, int) and 4 <= w <= 64 and 4 <= h <= 64):
        return "thermal 的 w、h 要是 4–64 的整数"
    lo, hi = ev.get("min"), ev.get("max")
    if not (isinstance(lo, (int, float)) and isinstance(hi, (int, float)) and -40 <= lo <= hi <= 400):
        return "thermal 的 min、max 要是 -40–400 的数字，且 min ≤ max"
    try:
        n = len(base64.b64decode(ev.get("px", ""), validate=True))
    except Exception:
        return "thermal 的 px 不是合法的 base64"
    if n != w * h:
        return f"thermal 的 px 应该是 {w * h} 个字节，收到 {n} 个"
    return ""


# ---------------------------------------------------------------- 分析
def direction(ax: float, ay: float) -> str:
    """角度 -> 中文方向。ax：负 = 狗的左边；ay：负 = 上。"""
    if abs(ax) < 8:
        h = "正前方"
    else:
        h = f"{'左' if ax < 0 else '右'}前方约 {abs(ax):.0f}°"
    if ay < -8:
        h += "偏上"
    elif ay > 8:
        h += "偏下"
    return h


def guess(peak: float, size: int) -> str:
    if peak >= 60:
        return "高温，可能是灶台、取暖器、电热水壶这类"
    if peak >= 40:
        return "比人体热，可能是热水、热饭、电器"
    if 28 <= peak < 40 and size >= 6:
        return "温度和大小像人或动物（体表温度）"
    return "比周围热一些的东西"


def analyze(frame: np.ndarray, fov=(55.0, 35.0), mirror=False, min_rise=4.0, max_spots=3) -> dict:
    """返回 {max, min, ambient, hot: [热源...], cold: 冷点 or None}。
    热源：比环境（中位数）高 min_rise 度以上、连在一起的一片，按最高温度排序。"""
    if mirror:
        frame = frame[:, ::-1]
    h, w = frame.shape
    amb = float(np.median(frame))

    def angles(r, c):
        return (c + 0.5 - w / 2) / w * fov[0], (r + 0.5 - h / 2) / h * fov[1]

    mask = frame > amb + min_rise
    seen = np.zeros_like(mask)
    spots = []
    for r0 in range(h):
        for c0 in range(w):
            if not mask[r0, c0] or seen[r0, c0]:
                continue
            stack, cells = [(r0, c0)], []
            seen[r0, c0] = True
            while stack:                                   # 把连在一起的一片找出来
                r, c = stack.pop()
                cells.append((r, c))
                for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                    if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        stack.append((rr, cc))
            pr, pc = max(cells, key=lambda rc: frame[rc])
            ax, ay = angles(pr, pc)
            peak = float(frame[pr, pc])
            spots.append({"peak": round(peak, 1), "size": len(cells), "row": pr, "col": pc,
                          "ax": round(ax, 1), "ay": round(ay, 1), "where": direction(ax, ay),
                          "guess": guess(peak, len(cells))})
    spots.sort(key=lambda s: -s["peak"])
    cr, cc = np.unravel_index(int(np.argmin(frame)), frame.shape)
    lo = float(frame[cr, cc])
    cold = None
    if lo < amb - min_rise:
        ax, ay = angles(cr, cc)
        cold = {"temp": round(lo, 1), "where": direction(ax, ay), "row": int(cr), "col": int(cc)}
    hr, hc = np.unravel_index(int(np.argmax(frame)), frame.shape)
    return {"max": round(float(frame.max()), 1), "min": round(lo, 1), "ambient": round(amb, 1),
            "max_at": (int(hr), int(hc)), "hot": spots[:max_spots], "cold": cold}


def describe(a: dict) -> str:
    """分析结果 -> 给大模型看的一段话。"""
    parts = [f"画面最高 {a['max']:.1f}°C、最低 {a['min']:.1f}°C，环境大约 {a['ambient']:.1f}°C"]
    if a["hot"]:
        spots = [f"{s['where']} {s['peak']:.1f}°C（占 {s['size']} 格，{s['guess']}）" for s in a["hot"]]
        parts.append("热源：" + "；".join(spots))
    else:
        parts.append("没有明显热源")
    if a["cold"]:
        parts.append(f"冷点：{a['cold']['where']} {a['cold']['temp']:.1f}°C（可能是窗户、空调出风口）")
    return "。".join(parts)


# ---------------------------------------------------------------- 假房间（模拟器用）
class FakeScene:
    """一个假的 32×24 热画面：环境 23°C，左上冷窗户，右下一杯热水，一个人在画面里慢慢走来走去。"""

    def __init__(self, seed=None):
        self.rng = np.random.default_rng(seed)
        self.stove = False              # 按 H 切换：右边出现一块 90°C 以上的“灶台”
        yy, xx = np.mgrid[0:H, 0:W]
        self.yy, self.xx = yy.astype(np.float32), xx.astype(np.float32)

    def frame(self, t: float) -> np.ndarray:
        yy, xx = self.yy, self.xx
        f = 23.0 + 0.8 * (1 - yy / H) + self.rng.normal(0, 0.15, (H, W))
        f = np.where((xx < 6) & (yy < 9), 17.5, f)                                  # 冷窗户
        px = 16 + 9 * math.sin(t * 2 * math.pi / 30)                                # 人：30 秒走一个来回
        body = ((xx - px) / 3.2) ** 2 + ((yy - 15) / 7) ** 2 < 1
        head = ((xx - px) / 2.0) ** 2 + ((yy - 6) / 2.2) ** 2 < 1
        f = np.where(body, 31.5, f)
        f = np.where(head, 34.5, f)
        f = np.where((np.abs(xx - 26) < 1.5) & (np.abs(yy - 19) < 1.0), 52.0, f)      # 热水杯
        if self.stove:
            f = np.where((np.abs(xx - 28) < 2.5) & (np.abs(yy - 12) < 1.5), 95.0 + 5 * math.sin(t), f)
        return f.astype(np.float32)
