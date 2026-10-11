"""认人：登记过的人和他们的脸部特征，存在记忆同一个文件里（data/memory/<家>.db 的 people、faces 两张表）。

只存 SFace 算出来的 128 个数字（脸部特征），不存照片，也认不回照片。
删掉某个人 = 删掉他的名字和全部特征。换一个家（memory.home）就是另一套人。

FaceID   查一张脸是谁：和每个人登记的特征比相似度（余弦），最像的那个超过阈值才算认出
Tracker  同一个人连续几帧的结果投票，免得一帧认错就叫错名字
"""
import time
from collections import Counter, deque

import numpy as np

RELATIONS = ("用户本人", "家人", "朋友", "访客")


class People:
    def __init__(self, db, max_faces=20):
        self.db = db
        self.max_faces = max_faces          # 每个人最多存多少组特征（多了删最旧的）
        db.executescript("""
            CREATE TABLE IF NOT EXISTS people (id INTEGER PRIMARY KEY, name TEXT UNIQUE, relation TEXT, created REAL,
                                               last_seen REAL);
            CREATE TABLE IF NOT EXISTS faces (id INTEGER PRIMARY KEY, person INTEGER, vec BLOB, ts REAL);""")
        db.commit()
        self._cache = None

    def add(self, name: str, relation: str, vecs) -> str:
        """登记（或补充）一个人的脸。返回 new / updated。"""
        name = name.strip()
        row = self.db.execute("SELECT id FROM people WHERE name=?", (name,)).fetchone()
        now = time.time()
        if row:
            pid, how = row[0], "updated"
            self.db.execute("UPDATE people SET relation=? WHERE id=?", (relation, pid))
        else:
            pid = self.db.execute("INSERT INTO people (name, relation, created, last_seen) VALUES (?,?,?,?)",
                                  (name, relation, now, now)).lastrowid
            how = "new"
        for v in vecs:
            self.db.execute("INSERT INTO faces (person, vec, ts) VALUES (?,?,?)",
                            (pid, unit(v).astype(np.float32).tobytes(), now))
        self.db.execute("""DELETE FROM faces WHERE person=? AND id NOT IN
                           (SELECT id FROM faces WHERE person=? ORDER BY ts DESC, id DESC LIMIT ?)""",
                        (pid, pid, self.max_faces))
        self.db.commit()
        self._cache = None
        return how

    def forget(self, name: str) -> bool:
        row = self.db.execute("SELECT id FROM people WHERE name=?", (name.strip(),)).fetchone()
        if not row:
            return False
        self.db.execute("DELETE FROM faces WHERE person=?", (row[0],))
        self.db.execute("DELETE FROM people WHERE id=?", (row[0],))
        self.db.commit()
        self._cache = None
        return True

    def seen(self, name: str):
        self.db.execute("UPDATE people SET last_seen=? WHERE name=?", (time.time(), name))
        self.db.commit()

    def all(self):
        """[(名字, 关系, 几组特征)]"""
        return self.db.execute("""SELECT p.name, p.relation, COUNT(f.id) FROM people p LEFT JOIN faces f ON f.person=p.id
                                  GROUP BY p.id ORDER BY p.created""").fetchall()

    def text(self) -> str:
        return "、".join(f"{n}（{r}）" for n, r, _ in self.all())

    def table(self):
        """(名字列表, 关系列表, 特征矩阵 N×128, 每行属于第几个人)，缓存起来，登记或删除时清掉。"""
        if self._cache is None:
            rows = self.db.execute("SELECT p.name, p.relation, f.vec FROM faces f JOIN people p ON p.id=f.person").fetchall()
            names, rels, idx, vecs = [], [], [], []
            for n, r, v in rows:
                if n not in names:
                    names.append(n)
                    rels.append(r)
                idx.append(names.index(n))
                vecs.append(np.frombuffer(v, np.float32))
            self._cache = (names, rels, np.array(vecs, np.float32).reshape(-1, 128), np.array(idx, int))
        return self._cache


def unit(v) -> np.ndarray:
    v = np.asarray(v, np.float32).ravel()
    return v / (np.linalg.norm(v) or 1.0)


class FaceID:
    """一张脸 -> (名字, 关系, 相似度) 或 (None, None, 最高相似度)。
    SFace 官方建议余弦相似度 ≥ 0.363 算同一个人；我们默认 0.42，宁可说“不认识”也不叫错名字。"""

    def __init__(self, people: People, threshold=0.42):
        self.people, self.threshold = people, threshold

    def identify(self, vec):
        names, rels, mat, idx = self.people.table()
        if not len(mat):
            return None, None, 0.0
        sims = mat @ unit(vec)
        best = {}
        for i, s in zip(idx, sims):
            best[i] = max(best.get(i, -1.0), float(s))
        i, s = max(best.items(), key=lambda kv: kv[1])
        return (names[i], rels[i], s) if s >= self.threshold else (None, None, s)


class Tracker:
    """最大那张脸最近几次的识别结果投票：同一个名字至少 need 票才算认出；连续几次都不认识才算陌生人。"""

    def __init__(self, window=5, need=3):
        self.votes = deque(maxlen=window)
        self.need = need

    def add(self, name):
        self.votes.append(name or "?")

    def reset(self):
        self.votes.clear()

    def who(self):
        """返回 (名字 or "?"=陌生人 or None=还说不准)。"""
        if not self.votes:
            return None
        name, n = Counter(self.votes).most_common(1)[0]
        return name if n >= self.need else None
