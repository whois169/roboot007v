"""看：本地一直看（人脸检测 + 动静检测），画面不出电脑。
只在“有事”的时候截一张 512px 的图交给看图模型：有人出现、有人叫它、空闲太久。

Camera  后台线程：读摄像头、检测人脸和动静，只保存“最新一帧”和检测结果；
        开了认人时，还给够大、够正的脸算 SFace 脸部特征（128 个数字，不存照片）
Sight   asyncio 这边：根据检测结果判断事件、认出是谁（dog/people.py）、让眼睛跟着人看、截图看图、
        给大模型写“我看到了什么”
"""
import asyncio
import logging
import os
import sys
import threading
import time

import cv2
import numpy as np

from . import config
from .people import Tracker

log = logging.getLogger("vision")
FACE_MODEL = "models/face_detection_yunet_2023mar.onnx"
RECOG_MODEL = "models/face_recognition_sface_2021dec.onnx"
DETECT_W = 320                 # 检测时把画面缩到这个宽度，又快又够用

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
except AttributeError:
    pass


class Camera(threading.Thread):
    def __init__(self, index=0, detect_fps=5, face_min_score=0.8, recognize=True, recog_fps=2, recog_min_width=0.08):
        super().__init__(daemon=True, name="camera")
        self.index, self.period, self.min_score = index, 1 / detect_fps, face_min_score
        self.recognize = recognize and os.path.exists(config.rel(RECOG_MODEL))
        self.recog_period, self.recog_min_width = 1 / recog_fps, recog_min_width
        self.feats = []                # 和 faces 一一对应：脸部特征（128 个数）或 None（太小、太歪、这一轮没算）
        self.feats_t = 0.0             # 特征是什么时候算的
        self.lock = threading.Lock()
        self.frame = None              # 最新一帧（BGR），只在内存里
        self.faces = []                # [(x 中心 0–1, y 中心 0–1, 宽 0–1, 分数, 正对程度 0–1)]，x 按画面从左到右
        self.motion = 0.0              # 画面里在动的部分占多少（0–1）
        self.updated = 0.0             # 检测结果的时间（time.monotonic）
        self.error = ""
        self.running = True
        self.ready = threading.Event()

    def run(self):
        backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY   # Windows 上 DSHOW 打开快很多
        cap = cv2.VideoCapture(self.index, backend)
        if not cap.isOpened():
            self.error = f"打不开摄像头 {self.index}"
            self.ready.set()
            return
        det = cv2.FaceDetectorYN.create(config.rel(FACE_MODEL), "", (DETECT_W, 240), self.min_score)
        rec = cv2.FaceRecognizerSF.create(config.rel(RECOG_MODEL), "") if self.recognize else None
        prev = None
        last_detect = last_recog = 0.0
        self.ready.set()
        while self.running:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            now = time.monotonic()
            with self.lock:
                self.frame = frame
            if now - last_detect < self.period:
                continue
            last_detect = now
            h, w = frame.shape[:2]
            small = cv2.resize(frame, (DETECT_W, round(h * DETECT_W / w)))
            sh, sw = small.shape[:2]
            det.setInputSize((sw, sh))
            _, found = det.detect(small)
            faces = [] if found is None else [
                ((f[0] + f[2] / 2) / sw, (f[1] + f[3] / 2) / sh, f[2] / sw, float(f[-1]), facing(f)) for f in found]
            feats, feats_t = None, 0.0
            if rec is not None and found is not None and now - last_recog >= self.recog_period:
                last_recog, feats_t = now, now
                feats = []
                k = w / sw                                   # 在小图上找脸，回到原图上抠脸算特征（更准）
                for f, info in zip(found, faces):
                    if info[2] < self.recog_min_width or info[4] < 0.3 or sum(v is not None for v in feats) >= 3:
                        feats.append(None)
                        continue
                    big = f.copy()
                    big[:14] *= k
                    try:
                        feats.append(rec.feature(rec.alignCrop(frame, big)).ravel().copy())
                    except cv2.error:
                        feats.append(None)
            gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (9, 9), 0)
            motion = 0.0 if prev is None else float(np.mean(cv2.absdiff(gray, prev) > 25))
            prev = gray
            with self.lock:
                self.faces, self.motion, self.updated = faces, motion, now
                if feats is not None:
                    self.feats, self.feats_t = feats, feats_t
                elif len(self.feats) != len(faces):
                    self.feats, self.feats_t = [None] * len(faces), 0.0
        cap.release()

    def snapshot(self, size=512) -> bytes | None:
        """最新一帧缩到长边 size 像素，编码成 JPEG。"""
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            return None
        h, w = frame.shape[:2]
        k = size / max(h, w)
        frame = cv2.resize(frame, (round(w * k), round(h * k)), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes() if ok else None

    def latest(self):
        with self.lock:
            return list(self.faces), self.motion, self.updated, (None if self.frame is None else self.frame.copy())

    def features(self):
        """(人脸列表, 对应的特征列表, 特征算出来的时间)"""
        with self.lock:
            return list(self.faces), list(self.feats), self.feats_t

    def stop(self):
        self.running = False


def facing(f) -> float:
    """脸有多正对着摄像头（0–1）：看鼻子是不是在两只眼睛正中间。
    YuNet 每张脸给 5 个点：右眼、左眼、鼻尖、右嘴角、左嘴角（f[4]–f[13]）。"""
    ex1, ex2, nx = f[4], f[6], f[8]
    if abs(ex2 - ex1) < 1:
        return 0.0
    r = (nx - ex1) / (ex2 - ex1)          # 0.5 = 正中间
    return float(max(0.0, 1 - abs(r - 0.5) * 4))


def face_direction(x: float) -> str:
    """画面左边 = 摄像头（狗）自己的左边。人在左边三分之一就往左看。"""
    return "left" if x < 0.36 else "right" if x > 0.64 else "center"


def ago(t: float) -> str:
    s = time.monotonic() - t
    return f"{s:.0f} 秒前" if s < 90 else f"{s / 60:.0f} 分钟前"


class Sight:
    def __init__(self, cfg: dict, eyes, vlm=None):
        self.cfg = cfg
        self.eyes = eyes
        self.vlm = vlm                       # 看图模型；None = 没配，截图不上传
        self.cam = Camera(cfg.get("camera_index", 0), cfg.get("detect_fps", 5), cfg.get("face_min_score", 0.8),
                          cfg.get("recognize", True), cfg.get("recog_fps", 2), cfg.get("recog_min_width", 0.08))
        self.faceid = None                   # 认人（大脑在 setup 时给）：people.FaceID
        self.tracker = Tracker()
        self.who = None                      # 现在面前最大那张脸是谁：名字 / "?" 陌生人 / None 还说不准或没人
        self.face_ids = []                   # 和 faces 一一对应：(名字 or None, 关系, 相似度) 或 None（没算）
        self._feats_seen = 0.0
        self.may_move = lambda: True         # 大脑告诉我：现在能不能自己转眼睛（说话时别乱动）
        self.on_event = None                 # 有事时回调 on_event(名字, 说明)，第⑥块“主动开口”用
        self.present = False                 # 现在是不是有人
        self.last_face = 0.0                 # 最后一次看到人脸的时间
        self.last_active = time.monotonic()  # 最后一次有互动或有人出现
        self.tracking = False                # 屏幕现在是不是我切的 TRACK
        self.paused = False                  # 隐私开关打开时暂停（真硬件上相机已经断电）
        self.att_since = None                # 从什么时候开始一直看着我
        self.att_last = 0.0                  # 最后一次看着我的时间
        self.sight = None                    # 最近一次看图结果：(描述, 时间, 原因)
        self._snap_lock = asyncio.Lock()

    async def start(self):
        self.cam.start()
        await asyncio.to_thread(self.cam.ready.wait, 10)
        if self.cam.error:
            log.warning("%s，“看”这块先关掉", self.cam.error)
            return False
        for _ in range(30):                  # 等第一帧检测结果出来（最多 3 秒），免得一开口就说“看不见”
            if self.cam.updated:
                break
            await asyncio.sleep(0.1)
        log.info("摄像头已打开（本地检测，画面不上传）。看图模型：%s",
                 self.vlm.name if self.vlm else "没配，截图不上传，只用本地检测结果")
        asyncio.create_task(self._watch())
        return True

    def attending(self) -> bool:
        """有没有人在看着我：最近的那张脸够近、正对着我，并且已经持续 attend_seconds 秒。"""
        faces, _, _, _ = self.cam.latest()
        now = time.monotonic()
        ok = False
        if faces:
            big = max(faces, key=lambda f: f[2])
            ok = big[2] >= self.cfg.get("near_face_width", 0.12) and big[4] >= self.cfg.get("facing_min", 0.6)
        if not ok:
            self.att_since = None
            return False
        self.att_last = now
        if self.att_since is None:
            self.att_since = now
        return now - self.att_since >= self.cfg.get("attend_seconds", 1.0)

    def looking_now(self) -> bool:
        """这一刻是不是正对着我（不要求持续多久，说话过程中用来统计）。"""
        faces, _, _, _ = self.cam.latest()
        if not faces:
            return False
        big = max(faces, key=lambda f: f[2])
        ok = big[2] >= self.cfg.get("near_face_width", 0.12) * 0.8 and big[4] >= self.cfg.get("facing_min", 0.6) * 0.8
        if ok:
            self.att_last = time.monotonic()                 # 聆听期间也要刷新，不然会以为人扭头了
        return ok

    def looked_away_for(self) -> float:
        return time.monotonic() - self.att_last if self.att_last else 1e9

    def touch(self):
        """有互动（被叫、聊天）时调用，用来判断“空闲太久”。"""
        self.last_active = time.monotonic()

    # ------------------------------------------------ 后台：每 0.2 秒看一眼检测结果
    async def _watch(self):
        gone = self.cfg.get("person_gone_seconds", 10)
        idle = self.cfg.get("idle_look_minutes", 10) * 60
        last_gaze = (0.0, 0.0)
        self.tracking = False
        while True:
            await asyncio.sleep(0.2)
            if self.paused:
                continue
            faces, motion, updated, _ = self.cam.latest()
            now = time.monotonic()
            if faces:
                self.last_face = now
                if not self.present:
                    self.present = True
                    self.touch()
                    await self._event("person_appeared", f"有人出现了（{len(faces)} 张脸）")
                    self.snapshot_later("有人出现")
            elif self.present and now - self.last_face > gone:
                self.present = False
                await self._event("person_left", f"人走了（{gone} 秒没看到人脸）")
            await self._recognize(faces, now)

            # 视线跟着最大的那张脸（任何模式下都跟，说话时也看着人）；闲着时屏幕切到 TRACK 锁定框
            if self.cfg.get("follow_face", True):
                if faces:
                    x, y = max(faces, key=lambda f: f[2])[:2]
                    g = (x * 2 - 1, y * 2 - 1)        # 画面左边 = 狗自己的左边 = gaze 负方向
                    if abs(g[0] - last_gaze[0]) + abs(g[1] - last_gaze[1]) > 0.04:
                        last_gaze = g
                        self.eyes.gaze(*g)
                elif now - self.last_face > 1.5 and last_gaze != (0.0, 0.0):
                    last_gaze = (0.0, 0.0)
                    self.eyes.gaze(0, 0)
                if not self.may_move():
                    self.tracking = False              # 大脑在用屏幕（聆听、思考、汇报），不抢
                elif faces and not self.tracking:
                    self.tracking = True
                    self.eyes.mode("track")
                elif self.tracking and now - self.last_face > 1.5:
                    self.tracking = False
                    self.eyes.mode("idle")

            if now - self.last_active > idle and (not self.sight or now - self.sight[1] > idle):
                self.touch()
                self.snapshot_later("空闲巡视")

    # ------------------------------------------------ 认人
    async def _recognize(self, faces, now):
        """有新算出来的特征就认一遍；最大那张脸的结果投票，定下来是谁（或陌生人）时发事件。"""
        if not faces:
            if self.who is not None and now - self.last_face > 1.5:
                self.who = None
                self.tracker.reset()
            self.face_ids = []
            return
        if not self.faceid:
            return
        fs, feats, t = self.cam.features()
        if t <= self._feats_seen or len(feats) != len(fs):
            return
        self._feats_seen = t
        self.face_ids = [None if v is None else self.faceid.identify(v) for v in feats]
        i = max(range(len(fs)), key=lambda k: fs[k][2])
        if self.face_ids[i] is None:
            return
        self.tracker.add(self.face_ids[i][0])
        who = self.tracker.who()
        if who is None or who == self.who:
            return
        self.who = who
        if who == "?":
            if self.faceid.people.all():                  # 一个人都没登记时，谁都是“陌生人”，就不报了
                where = {"left": "我左前方", "right": "我右前方", "center": "我正前方"}[face_direction(fs[i][0])]
                await self._event("stranger", f"看到一个没登记过的人，在{where}")
        else:
            rel = next((r for n, r, _ in self.faceid.people.all() if n == who), "")
            self.faceid.people.seen(who)
            await self._event("person_recognized", f"认出了：{who}（{rel}）")

    async def collect(self, n=5, timeout=8.0) -> list:
        """登记用：收集最大那张脸的 n 组特征（要够近、够正），超时返回已经收到的。"""
        got, last = [], 0.0
        end = time.monotonic() + timeout
        while time.monotonic() < end and len(got) < n:
            await asyncio.sleep(0.1)
            fs, feats, t = self.cam.features()
            if not fs or t <= last or len(feats) != len(fs):
                continue
            i = max(range(len(fs)), key=lambda k: fs[k][2])
            if feats[i] is not None and fs[i][2] >= self.cfg.get("near_face_width", 0.12) * 0.8 and fs[i][4] >= 0.5:
                got.append(feats[i])
                last = t
        return got

    async def _event(self, name, desc):
        log.info("[看] %s", desc)
        self.eyes.log({"person_appeared": "EVT PERSON IN", "person_left": "EVT PERSON OUT",
                       "stranger": "EVT UNKNOWN PERSON", "person_recognized": "EVT ID MATCH"}.get(name, "EVT " + name))
        if self.on_event:
            await self.on_event(name, desc)

    # ------------------------------------------------ 截图交给看图模型
    def snapshot_later(self, reason) -> asyncio.Task | None:
        """后台去截图看图，不挡住别的事。没配看图模型就什么都不做。"""
        if not self.vlm:
            return None
        return asyncio.create_task(self.snapshot(reason))

    async def snapshot(self, reason: str):
        if not self.vlm:
            return None
        async with self._snap_lock:
            jpg = self.cam.snapshot(self.cfg.get("snapshot_size", 512))
            if not jpg:
                return None
            if self.cfg.get("save_snapshots"):
                d = config.path("data/snapshots")
                os.makedirs(d, exist_ok=True)
                with open(os.path.join(d, time.strftime("%Y%m%d-%H%M%S") + ".jpg"), "wb") as f:
                    f.write(jpg)
            t0 = time.perf_counter()
            try:
                desc = await self.vlm.describe(jpg)
            except Exception as e:
                log.warning("看图失败（%s）：%s", type(e).__name__, e)
                return None
            self.sight = (desc, time.monotonic(), reason)
            if self.on_event:
                await self.on_event("sight", f"因为「{reason}」拍了一张照片，看到：{desc}")
            log.info("[看图] 因为「%s」截了一张 %dKB 的图，%.1f 秒后看图模型说：%s",
                     reason, len(jpg) // 1024, time.perf_counter() - t0, desc)
            return desc

    # ------------------------------------------------ 放到感觉板上的读数
    def readings(self):
        """给感觉板的读数（board.add_provider(sight.readings)）。"""
        from .senses import Reading
        faces, motion, updated, _ = self.cam.latest()
        loc = "头部前方摄像头"
        if self.paused:
            return [Reading("camera.faces", "看到的人", "隐私开关打开，摄像头断电，我现在看不见", source="隐私开关",
                            location=loc, confidence=1.0)]
        if not updated:
            return [Reading("camera.faces", "看到的人", "没有画面，我现在看不见", source="摄像头", location=loc, confidence=1.0)]
        wall = lambda mono: time.time() - (time.monotonic() - mono)      # 单调时钟 -> 墙上时间
        where = {"left": "我左前方", "right": "我右前方", "center": "我正前方"}
        if faces:
            big = max(faces, key=lambda f: f[2])
            near = "很近" if big[2] > 0.25 else "不远" if big[2] > 0.12 else "比较远"
            value = f"{len(faces)} 张人脸，最近的一个在{where[face_direction(big[0])]}，离我{near}" + \
                    ("，正对着我" if big[4] >= self.cfg.get("facing_min", 0.6) else "，没有正对着我")
            value += self._who_text(faces.index(big), len(faces))
        else:
            seen = f"（上次看到人脸是 {ago(self.last_face)}）" if self.last_face else ""
            value = f"没有人脸{seen}"
        out = [Reading("camera.faces", "看到的人", value, source="本地人脸检测", location=loc, confidence=0.9,
                       note="" if faces else "人背对我或离得太远时也会测不到", max_age=10, t=wall(updated))]
        if motion > 0.02:
            out.append(Reading("camera.motion", "画面动静", "有东西在动", source="本地动静检测", location=loc,
                               confidence=0.3, note="只说明有东西在动，不一定是人", max_age=10, t=wall(updated)))
        if self.sight:
            desc, t, reason = self.sight
            out.append(Reading("camera.sight", "看图描述", desc, source=f"看图模型（因为「{reason}」拍的一张照片）",
                               location=loc, confidence=0.6, note="可能看错", max_age=1800, t=wall(t)))
        return out

    def _who_text(self, i, n) -> str:
        """认人的结果，接在“看到的人”后面。"""
        if not self.faceid:
            return ""
        if not self.faceid.people.all():
            return "；还没有登记过任何人，认不出是谁"
        if self.who and self.who != "?":
            rel = next((r for nm, r, _ in self.faceid.people.all() if nm == self.who), "")
            s = f"；认出是 {self.who}（{rel}）"
        elif self.who == "?":
            s = "；这个人没登记过（陌生人）"
        else:
            s = "；还没认清是谁（离近一点、正对我会认得更快）"
        others = [x for k, x in enumerate(self.face_ids) if k != i and x is not None]
        if n > 1 and others:
            known = [x[0] for x in others if x[0]]
            unknown = len(others) - len(known)
            s += "；另外还有：" + "、".join(known + ([f"{unknown} 个没登记过的人"] if unknown else []))
        return s

    def stop(self):
        self.cam.stop()


async def preview(sight: Sight):
    """调试窗口：显示摄像头画面和检测框。按 s 截图看图，按 q 关掉这个窗口。"""
    while True:
        faces, motion, _, frame = sight.cam.latest()
        if frame is not None:
            h, w = frame.shape[:2]
            for x, y, fw, score, _ in faces:
                bw, bh = fw * w, fw * w * 1.2
                p1 = (int(x * w - bw / 2), int(y * h - bh / 2))
                p2 = (int(x * w + bw / 2), int(y * h + bh / 2))
                cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
                cv2.putText(frame, f"{score:.2f}", (p1[0], p1[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"faces {len(faces)}  motion {motion:.2f}  [s] snapshot  [q] close",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow("RoboDog camera (debug)", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            cv2.destroyAllWindows()
            return
        if k == ord("s"):
            if sight.vlm:
                sight.snapshot_later("手动按 s")
            else:
                log.info("没配看图模型，截图不上传。现在大模型能拿到的是：\n%s",
                         "\n".join(r.line() for r in sight.readings()))
        await asyncio.sleep(1 / 20)
