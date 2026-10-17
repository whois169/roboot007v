"""注意力：在后台盯着摄像头和时钟，发现值得说的事就主动开口。

现在会主动说的（都基于真实数据）：
  arrive   你离开一阵（默认 5 分钟以上）又回来，走到它面前看着它时，先开口报一两句有用的信息
  first    今天第一次见到你：同上，做个简短的“今日播报”
  sit      摄像头连续很久（默认 60 分钟）一直看到你，提醒你起来活动

频率限制（都在 config.yaml 的 attention 里）：
  两次主动开口至少隔 min_gap_minutes；每小时最多 max_per_hour 次；同一类事有自己的冷却时间；
  夜里 quiet_hours 不主动说；用户让它“安静一会儿”（quiet 技能）期间不说。
真正说什么由大模型决定：不值得说它会输出 ignore，一个字都不说。
"""
import logging
import time
from collections import deque
from datetime import datetime

log = logging.getLogger("attention")


class Attention:
    def __init__(self, cfg: dict, sight=None, clock=time.time):
        self.cfg = cfg
        self.sight = sight
        self.clock = clock
        now = clock()
        self.started = now
        self.present = False
        self.present_since = None          # 这次连续在场从什么时候开始
        self.absent_since = now            # 从什么时候开始没人（刚开机当作“一直没人”）
        self.pending = None                # 等你走近、看着它时要说的事：(类型, 原因, 失效时间)
        self.last_spoke = 0.0
        self.spoke_times = deque()
        self.kind_last = {}
        self.greet_day = ""                # 哪天已经打过招呼
        self.quiet_until = 0.0             # 由大脑的 quiet 技能设置（time.time）

    # ------------------------------------------------ 频率限制
    def blocked(self, kind: str) -> str:
        """不能开口就返回原因，能开口返回空字符串。"""
        now = self.clock()
        c = self.cfg
        if now < self.quiet_until:
            return "用户让我安静"
        h0, h1 = c.get("quiet_hours", [23, 7])
        hour = datetime.fromtimestamp(now).hour
        if (h0 > h1 and (hour >= h0 or hour < h1)) or (h0 < h1 and h0 <= hour < h1):
            return f"夜间（{h0}–{h1} 点）不主动说"
        if now - self.last_spoke < c.get("min_gap_minutes", 10) * 60:
            return "离上次主动开口太近"
        while self.spoke_times and now - self.spoke_times[0] > 3600:
            self.spoke_times.popleft()
        if len(self.spoke_times) >= c.get("max_per_hour", 3):
            return "这一小时说得够多了"
        cool = c.get("cooldown_minutes", {}).get(kind, 30) * 60
        if now - self.kind_last.get(kind, -1e18) < cool:
            return f"「{kind}」还在冷却"
        return ""

    def spoke(self, kind: str):
        now = self.clock()
        self.last_spoke = now
        self.spoke_times.append(now)
        self.kind_last[kind] = now
        if kind in ("arrive", "first"):
            self.greet_day = datetime.fromtimestamp(now).strftime("%Y-%m-%d")

    # ------------------------------------------------ 每秒更新一次在场情况
    def update(self):
        now = self.clock()
        here = bool(getattr(self.sight, "present", False))
        if here and not self.present:                         # 刚出现
            away = now - self.absent_since
            today = datetime.fromtimestamp(now).strftime("%Y-%m-%d")
            if self.greet_day != today:
                self.pending = ("first", "今天第一次见到用户（现在用户就在你面前）", now + 120)
            elif away >= self.cfg.get("arrive_after_minutes", 5) * 60 and now - self.started > 60:
                self.pending = ("arrive", f"用户离开了大约 {away / 60:.0f} 分钟，刚回到你面前（是用户回来了，你一直在原地）", now + 120)
            self.present_since = now
        elif not here and self.present:                       # 刚离开
            self.absent_since = now
            self.present_since = None
            self.pending = None
        self.present = here

    def greeting(self):
        """你走近、看着它的这一刻：有要说的事并且不受限制，返回 (类型, 原因)；否则 None。"""
        if not self.pending or self.clock() > self.pending[2]:
            self.pending = None
            return None
        kind, why, _ = self.pending
        if self.blocked(kind):
            log.debug("想打招呼但不说：%s", self.blocked(kind))
            return None
        self.pending = None
        return kind, why

    def periodic(self, comfort: str = ""):
        """不需要你看着它的提醒（坐太久、屋里太热太潮……）。返回 (类型, 原因) 或 None。
        comfort：大脑算好的“屋里持续不舒服”的原因，没有就是空字符串。只在有人的时候说。"""
        if not self.present or self.present_since is None:
            return None
        if comfort and not self.blocked("comfort"):
            return "comfort", comfort
        mins = (self.clock() - self.present_since) / 60
        if mins >= self.cfg.get("sitting_minutes", 60) and not self.blocked("sit"):
            return "sit", f"摄像头已经连续 {mins:.0f} 分钟看到用户在你面前，用户一直没离开过。这是到点的久坐提醒，必须说：直接提醒用户起来活动"
        return None
