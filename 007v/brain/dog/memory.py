"""长期记忆：存在 data/memory/<家的名字>.db（SQLite），每个家一个文件。

三张表：
  events  事件日志：什么时间发生了什么（对话、有人进出、看图描述……）
  facts   用户告诉它的事实：名字、习惯、东西放在哪、约定……（大模型负责把过时的 forget 掉）
  daily   每日小结：前一天发生了什么，由大模型写（没有大模型就写次数统计）

聊天时 recall(用户的话) 按“字词重合度”挑出相关的几条，放进提示词的【我的记忆】。
不联网、不用额外的库。删掉某个家的 .db 文件，就是把那个家的记忆全部清掉。
"""
import asyncio
import logging
import os
import re
import sqlite3
import time
from datetime import date, datetime, timedelta

from . import config

log = logging.getLogger("memory")

# 问到这些词时，除了按相关性挑，再把最近发生的事带上
TIME_WORDS = ("刚才", "刚刚", "今天", "昨天", "前天", "上次", "最近", "什么时候", "几点", "谁来", "有人", "发生")
STOP_GRAMS = {"什么", "一下", "这个", "那个", "我们", "你们", "他们", "怎么", "是不", "不是", "知道", "现在"}


def grams(s: str) -> set:
    """一句话 -> 相邻两个字的组合（中文没有空格，用“两字一组”来比较相似）。"""
    s = re.sub(r"[\W_]+", "", s.lower())
    if len(s) < 2:
        return {s} if s else set()
    return {s[i:i + 2] for i in range(len(s) - 1)} - STOP_GRAMS


def similarity(a: str, b: str) -> float:
    ga, gb = grams(a), grams(b)
    if not ga or not gb:
        return 0.0
    return len(ga & gb) / (len(ga) * len(gb)) ** 0.5


def when(ts: float) -> str:
    """时间戳 -> 给大模型看的时间：今天 14:20 / 昨天 09:05 / 10-03 18:30。"""
    d = datetime.fromtimestamp(ts)
    days = (date.today() - d.date()).days
    head = "今天" if days == 0 else "昨天" if days == 1 else "前天" if days == 2 else d.strftime("%m-%d")
    return f"{head} {d:%H:%M}"


class Memory:
    def __init__(self, home="home_main", folder="data/memory", fact_merge=0.8, max_facts=6, max_events=6, **_):
        self.home = home
        self.path = os.path.join(config.path(folder), f"{home}.db")
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.fact_merge, self.max_facts, self.max_events = fact_merge, max_facts, max_events
        self.db = sqlite3.connect(self.path)
        self.db.executescript("""
            CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY, ts REAL, kind TEXT, text TEXT);
            CREATE TABLE IF NOT EXISTS facts  (id INTEGER PRIMARY KEY, ts REAL, text TEXT, active INTEGER DEFAULT 1);
            CREATE TABLE IF NOT EXISTS daily  (day TEXT PRIMARY KEY, summary TEXT, ts REAL);
            CREATE INDEX IF NOT EXISTS events_ts ON events(ts);
        """)
        self.db.commit()

    # ------------------------------------------------ 写
    def add_event(self, kind: str, text: str, ts: float = None):
        self.db.execute("INSERT INTO events (ts, kind, text) VALUES (?, ?, ?)", (ts or time.time(), kind, text[:300]))
        self.db.commit()

    def add_fact(self, text: str) -> str:
        """记一条事实。和已有的某条几乎一样（重复记），就只更新时间。返回 'new' 或 'updated'。
        “同一件事的新情况”（比如钥匙换了地方）字面上不一定像，由大模型先 forget 旧的再 remember 新的。"""
        best, best_id = 0.0, None
        for fid, old in self.db.execute("SELECT id, text FROM facts WHERE active = 1"):
            s = similarity(text, old)
            if s > best:
                best, best_id = s, fid
        if best_id is not None and best >= self.fact_merge:
            self.db.execute("UPDATE facts SET text = ?, ts = ? WHERE id = ?", (text, time.time(), best_id))
            self.db.commit()
            return "updated"
        self.db.execute("INSERT INTO facts (ts, text) VALUES (?, ?)", (time.time(), text))
        self.db.commit()
        return "new"

    def forget(self, text: str) -> str:
        """忘掉和 text 最像的那条事实（不是真删，标记为不用了）。返回被忘掉的内容，没找到返回空字符串。"""
        rows = [(similarity(text, t), fid, t) for fid, t in self.db.execute("SELECT id, text FROM facts WHERE active = 1")]
        rows = [r for r in rows if r[0] >= 0.3]
        if not rows:
            return ""
        _, fid, t = max(rows)
        self.db.execute("UPDATE facts SET active = 0 WHERE id = ?", (fid,))
        self.db.commit()
        return t

    # ------------------------------------------------ 读
    def facts(self):
        return list(self.db.execute("SELECT ts, text FROM facts WHERE active = 1 ORDER BY ts DESC"))

    def events(self, since: float = 0, limit: int = 2000):
        return list(self.db.execute("SELECT ts, kind, text FROM events WHERE ts >= ? ORDER BY ts DESC LIMIT ?",
                                    (since, limit)))

    def recall(self, query: str) -> str:
        """【我的记忆】：和这句话相关的事实、事件、小结。"""
        lines = []
        facts = self.facts()
        scored = sorted(((similarity(query, t), ts, t) for ts, t in facts), reverse=True)
        picked = [x for x in scored if x[0] > 0.05][:self.max_facts]
        if len(picked) < 3:                                    # 相关的不多：补上最近记的几条，总共不超过 3 条
            picked += [(0, ts, t) for ts, t in facts if t not in [p[2] for p in picked]][:3 - len(picked)]
        for _, ts, t in picked:
            lines.append(f"- 用户告诉我的（{when(ts)} 记下）：{t}")

        week = self.events(since=time.time() - 7 * 86400)
        ev = sorted(((similarity(query, t), ts, k, t) for ts, k, t in week), reverse=True)
        chosen = [x for x in ev if x[0] > 0.12][:self.max_events]
        if any(w in query for w in TIME_WORDS):              # 问到时间：再带上最近发生的事
            recent = [(0, ts, k, t) for ts, k, t in week[:self.max_events]]
            chosen += [r for r in recent if r[3] not in [c[3] for c in chosen]]
        for _, ts, k, t in sorted(chosen, key=lambda x: x[1])[-self.max_events * 2:]:
            lines.append(f"- {when(ts)}（{'对话记录' if k == 'chat' else '事件'}）{t}")

        for day, summary in self.db.execute("SELECT day, summary FROM daily ORDER BY day DESC LIMIT 7"):
            d = date.fromisoformat(day)
            label = "昨天" if (date.today() - d).days == 1 else "前天" if (date.today() - d).days == 2 else day
            if label == "昨天" or similarity(query, summary) > 0.1 or label in query or day in query:
                lines.append(f"- {label}（{day}）小结：{summary}")
        return "\n".join(lines)

    def stats(self) -> str:
        n_f = self.db.execute("SELECT COUNT(*) FROM facts WHERE active = 1").fetchone()[0]
        n_e = self.db.execute("SELECT COUNT(*) FROM events").fetchone()[0]
        n_d = self.db.execute("SELECT COUNT(*) FROM daily").fetchone()[0]
        return f"{n_f} 条事实、{n_e} 条事件、{n_d} 天小结"

    # ------------------------------------------------ 每日小结
    def days_to_summarize(self):
        """今天以前、有事件、还没写小结的日子。"""
        today = date.today().isoformat()
        rows = self.db.execute("SELECT DISTINCT date(ts, 'unixepoch', 'localtime') FROM events")
        done = {d for (d,) in self.db.execute("SELECT day FROM daily")}
        return sorted(d for (d,) in rows if d and d < today and d not in done)

    def day_events(self, day: str):
        start = datetime.fromisoformat(day).timestamp()
        return list(self.db.execute("SELECT ts, kind, text FROM events WHERE ts >= ? AND ts < ? ORDER BY ts",
                                    (start, start + 86400)))

    async def summarize(self, llm=None):
        for day in self.days_to_summarize():
            evs = self.day_events(day)
            summary = ""
            if llm is not None and hasattr(llm, "complete"):
                lines = "\n".join(f"{datetime.fromtimestamp(ts):%H:%M} [{k}] {t}" for ts, k, t in evs[-200:])
                try:
                    summary = await llm.complete([
                        {"role": "system", "content": "你是一台家用轮足机器人（机械狗007v）的记忆整理程序。根据这一天的事件日志，"
                                                      "用 2–4 句简短的中文写当天小结：发生了哪些值得记住的事、用户交代了什么、"
                                                      "有没有异常。只写日志里有的，不要编。不要写标题。"},
                        {"role": "user", "content": f"{day} 的事件日志：\n{lines}"}])
                except Exception as e:
                    log.warning("写 %s 的小结失败（%s），先用次数统计", day, e)
            if not summary.strip():
                kinds = {}
                for _, k, _ in evs:
                    kinds[k] = kinds.get(k, 0) + 1
                names = {"chat": "对话", "person_appeared": "有人出现", "person_left": "有人离开", "sight": "看图"}
                summary = "，".join(f"{names.get(k, k)} {n} 次" for k, n in kinds.items()) + "（自动统计）"
            self.db.execute("INSERT OR REPLACE INTO daily (day, summary, ts) VALUES (?, ?, ?)",
                            (day, summary.strip()[:400], time.time()))
            self.db.commit()
            log.info("写好了 %s 的小结：%s", day, summary.strip()[:60])

    async def summarize_loop(self, llm=None, every_minutes=60):
        while True:
            try:
                await self.summarize(llm)
            except Exception as e:
                log.warning("每日小结出错：%s", e)
            await asyncio.sleep(every_minutes * 60)

    def close(self):
        self.db.close()
