"""红外遥控码库：学到的码按“家电 + 动作”存，和记忆放在同一个文件里（data/memory/<家>.db 的 ir_codes 表）。

去别人家（换 memory.home）时码库也跟着换，不会拿自己家空调的码去按别人家的。
红外码本身由头部 ESP32 学和发，大脑只负责存、找、交给 ESP32。码的格式见 protocol.check_ir_code()。
"""
import json
import logging
import re
import sqlite3
import time

from . import protocol

log = logging.getLogger("ir")

# 动作的不同说法统一成一种，免得学的时候说“关机”、用的时候说“关掉”就找不到
ACTION_SYNONYMS = {"开": ("打开", "开启", "开机", "启动", "开一下"),
                   "关": ("关闭", "关机", "关掉", "关上", "关一下"),
                   "开关": ("电源", "电源键", "开关键"),
                   "音量加": ("音量+", "声音大一点", "大声点", "调大音量"),
                   "音量减": ("音量-", "声音小一点", "小声点", "调小音量")}
_ACTION = {alt: std for std, alts in ACTION_SYNONYMS.items() for alt in alts}


def norm_device(s: str) -> str:
    return re.sub(r"[\W_]+|的", "", s.strip().lower())


def norm_action(s: str) -> str:
    s = re.sub(r"[\W_]+", "", s.strip().lower()).replace("度", "")
    return _ACTION.get(s, s)


class IrCodes:
    def __init__(self, db):
        """db：记忆的数据库连接（和记忆共用一个连接，一起关），或者单独的 .db 文件路径（测试用）。"""
        self.own = isinstance(db, str)
        self.db = sqlite3.connect(db) if self.own else db
        self.db.execute("""CREATE TABLE IF NOT EXISTS ir_codes (
            device TEXT, action TEXT, code TEXT, learned REAL, used INTEGER DEFAULT 0, last_used REAL,
            PRIMARY KEY (device, action))""")
        self.db.commit()

    def save(self, device: str, action: str, code: dict) -> str:
        """存一个码；同名的覆盖（重新学一次）。返回 new / updated。"""
        err = protocol.check_ir_code(code)
        if err:
            raise ValueError(err)
        d, a = norm_device(device), norm_action(action)
        old = self.db.execute("SELECT 1 FROM ir_codes WHERE device=? AND action=?", (d, a)).fetchone()
        self.db.execute("INSERT OR REPLACE INTO ir_codes (device, action, code, learned, used) VALUES (?,?,?,?,0)",
                        (d, a, json.dumps(code, ensure_ascii=False), time.time()))
        self.db.commit()
        return "updated" if old else "new"

    def find(self, device: str, action: str):
        """找码：返回 (家电, 动作, code)，找不到返回 None。
        家电名先精确找；找不到时，如果只有一个已学的家电名和它互相包含（“空调”和“客厅空调”），就用那个。"""
        d, a = norm_device(device), norm_action(action)
        rows = self.db.execute("SELECT device, action, code FROM ir_codes WHERE action=?", (a,)).fetchall()
        hit = [r for r in rows if r[0] == d] or [r for r in rows if d and (d in r[0] or r[0] in d)]
        if len(hit) != 1:
            return None
        dev, act, code = hit[0]
        self.db.execute("UPDATE ir_codes SET used=used+1, last_used=? WHERE device=? AND action=?", (time.time(), dev, act))
        self.db.commit()
        return dev, act, json.loads(code)

    def forget(self, device: str, action: str = "") -> int:
        """忘掉一个码；action 为空就忘掉这个家电的全部码。返回删了几个。"""
        d = norm_device(device)
        if action:
            cur = self.db.execute("DELETE FROM ir_codes WHERE device=? AND action=?", (d, norm_action(action)))
        else:
            cur = self.db.execute("DELETE FROM ir_codes WHERE device=?", (d,))
        self.db.commit()
        return cur.rowcount

    def devices(self) -> dict:
        """{家电: [动作, ...]}"""
        out = {}
        for d, a in self.db.execute("SELECT device, action FROM ir_codes ORDER BY device, learned"):
            out.setdefault(d, []).append(a)
        return out

    def text(self) -> str:
        """给提示词用：已经学会的家电和按键。"""
        return "\n".join(f"- {d}：{'、'.join(acts)}" for d, acts in self.devices().items())

    def close(self):
        if self.own:
            self.db.close()
