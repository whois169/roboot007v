"""大脑主流程：唤醒 → 马上“嗯？” → 听一句话 → 识别 → 大模型（流式）→ 边想边说。

状态：idle 等唤醒词 / listening 听你说话 / thinking 识别和想 / speaking 说话
自己出声的时候不听麦克风（半双工），免得把自己的声音当成唤醒词。
"""
import asyncio
import difflib
import json
import logging
import math
import os
import re
import time

import numpy as np

from . import config
from .memory import Memory
from .ir import IrCodes
from .people import People, FaceID
from .attention import Attention
from . import protocol
from . import thermal as thermal_mod
from . import llm as llm_mod
from . import prompt, skills
from .audio import Microphone, Speaker
from . import eyes as eyes_mod
from . import chassis as chassis_mod
from . import tts as tts_mod
from . import senses as senses_mod
from .lookup import Weather

log = logging.getLogger("brain")

FALLBACK = "云端连接失败，请稍后再试。"


class Brain:
    def __init__(self, cfg: dict, force_mock=False, eyes=None):
        self.cfg = cfg
        self.persona = cfg["persona"]
        self.llm = llm_mod.make(cfg["llm"], force_mock)
        self.voice = tts_mod.make(cfg["tts"])
        self.speaker = Speaker(cfg["audio"]["output_device"], cfg["audio"]["mute_tail_seconds"])
        self.eyes = eyes or eyes_mod.make(cfg.get("eyes", {}), self.persona["name"])
        self.eyes.on_event = self.on_head_event   # 头部发来的事件：屏幕触摸、传感器、隐私开关、心跳
        self.chassis = chassis_mod.make(cfg.get("chassis", {}))   # 底盘：virtual 仿真 / serial 真车 / none
        self.chassis.on_event = self.on_chassis_event
        self.motions = []              # 这次回答里大模型要做的动作（按顺序做）
        self.motion_job = None         # 正在做动作的任务
        self.history: list = []
        self.memory = Memory(**cfg.get("memory", {}))   # 第⑤块：长期记忆（data/memory/<家>.db）
        self.ir = IrCodes(self.memory.db)               # 第⑧块：红外遥控码库（和记忆同一个文件、同一个连接）
        self.ir_pending = None         # 大模型让学的按键：(家电, 动作)，等这次回答说完再开始学
        self.people = People(self.memory.db)            # 第⑪块：认人（登记过的人和脸部特征，和记忆同一个文件）
        self.enroll_pending = None     # 大模型让登记的人：(名字, 关系)，等这次回答说完再开始看脸
        self.ir_wait = None            # 正在学：等 ESP32 回 learned / timeout / fail
        self.ir_sent = []              # 这次回答发了哪些红外按键
        self.thermal = None            # 第⑨块：最近一帧热成像的分析结果（dog/thermal.py 的 analyze）
        self.hot_since = None          # 高温点从什么时候开始一直在
        self.hot_alerted = 0.0         # 上次高温提醒的时间
        self.comfort_since = {}        # 第⑩块：屋里不舒服（太热 / 太冷 / 太潮 / 太干）从什么时候开始
        self.quiet_until = 0.0         # quiet 技能设置的“安静到什么时候”
        self.attention = None          # 第⑥块：主动开口（语音模式、开了摄像头才有）
        self.privacy = False           # 头上的隐私开关拨到“隐私”了（相机和麦克风硬件断电）
        self.interrupted = ""          # 说话时被点了屏幕：listen = 停下来听你说 / idle = 停下来待机
        self.head_online = None        # 头部 ESP32 连着没有（只有真串口才检查心跳）
        self.state = "idle"
        self.via = ""                   # 这次是怎么开始听的：look 看着我 / wake 唤醒词 / follow 接着说
        self.att_frames = [0, 0]        # 说话期间：总帧数、看着我的帧数
        self.recent_said = []           # 最近自己说过的话：[(过期时间, 去掉标点的文字)]，用来认出回声
        self.last_reply = []            # 上一次回答说了哪几句（写决策日志用）
        self.utt = {}                   # 当前这句话的信息（写决策日志用）
        self.log_dir = "data/logs"      # 决策日志放哪（测试时换到别的目录）
        self.look_talk = self.use_wake = False
        self.ack = None
        self.sight = None              # 第③块：看
        self._sight_job = None         # 被叫时在后台截图看图的任务
        self.board = senses_mod.SenseBoard()                   # 第④块：感觉板
        self.started = time.time()
        self.board.add_provider(senses_mod.clock_provider(time.time()))
        self.sound = senses_mod.SoundLevel(self.board)
        self.weather = Weather(**cfg["weather"]) if cfg.get("weather") else None

    # ------------------------------------------------------------ 启动
    async def setup(self, voice_input=True):
        log.info("大模型：%s", self.llm.name)
        log.info("语音合成：%s", self.voice.name)
        weather_job = asyncio.create_task(self.weather.start()) if self.weather else None   # 和下面的加载一起进行
        await self.eyes.start()
        self.eyes.send({"lang": self.cfg.get("eyes", {}).get("lang", "en")})   # 屏幕界面文字：en 英文（默认）/ cn 中文
        th = self.cfg.get("thermal", {})
        if th.get("hz", 1):
            self.eyes.send_thermal({"hz": th.get("hz", 1)})       # 让头部每秒发热画面过来
        self.eyes.log("SYS BOOT 007V BRAIN")                        # 开机自检：屏幕上逐行打出真实的初始化过程
        await self.chassis.start()
        self.board.add_provider(self.chassis.readings)
        self.eyes.log(f"BASE {self.chassis.kind.upper()} " + ("OK" if self.chassis.online else "OFF" if self.chassis.kind == "none" else "NO LINK"))
        react = self.cfg["react"]
        self.ack = ((tts_mod.beep(), 24000) if react.get("ack", "beep") == "beep"
                    else await self.voice.cached(react.get("ack_text", "在。")))
        self.chirp = (tts_mod.chirp(), 24000)
        h = await self.voice.health()
        if h:
            log.info("语音合成服务在线：音色 %s，机械感 %s", h.get("default_speaker"), h.get("effect"))
        self.eyes.log("TTS QWEN3 OK" if h else "TTS OFFLINE")
        await self.setup_vision()
        self.eyes.log("VIS CAM OK FACE YUNET" if self.sight else "VIS CAM OFF")
        self.setup_senses()
        asyncio.create_task(self.status_loop())
        asyncio.create_task(self.memory.summarize_loop(None if isinstance(self.llm, llm_mod.MockLLM) else self.llm))
        log.info("记忆：%s（%s）", self.memory.home, self.memory.stats())
        self.eyes.log(f"MEM {self.memory.home} {len(self.memory.facts())} FACTS")
        if voice_input:
            from .hearing import Listener, Recognizer, WakeWord   # 文字模式不用加载语音模型
            a, w, li = self.cfg["audio"], self.cfg["wake"], self.cfg["listen"]
            log.info("加载语音模型……")
            self.wake = WakeWord(w["keywords"], w["threshold"], w["boost"], a["sample_rate"])
            self.listener = Listener(li["wait_seconds"], li["max_speech_seconds"],
                                     li["end_silence_seconds"], a["sample_rate"])
            self.asr = await asyncio.to_thread(Recognizer, **self.cfg["asr"])
            self.mic = Microphone(a["sample_rate"], a["input_device"])
            self.eyes.log("ASR SENSEVOICE OK KWS OK")
        if weather_job:
            try:                                                   # 第一次查天气最多再等 5 秒，查不到也照样启动
                await asyncio.wait_for(asyncio.shield(weather_job), 5)
            except asyncio.TimeoutError:
                log.warning("天气还没查到，先启动，后台接着查")
            self.eyes.log("NET " + (self.weather.short() if self.weather.data else "WX PENDING"))
        self.eyes.log("LLM MOCK" if isinstance(self.llm, llm_mod.MockLLM) else f"LLM {self.llm.model} READY")

    async def setup_vision(self):
        vc = self.cfg.get("vision", {})
        if not vc.get("enabled", False):
            log.info("摄像头：关（config.yaml 里 vision.enabled）")
            return
        from .vision import Sight
        att = {k: v for k, v in self.cfg.get("listen", {}).items()          # “看着我”的判断标准在 listen 里
               if k in ("near_face_width", "facing_min", "attend_seconds")}
        sight = Sight(dict(vc, **att), self.eyes, llm_mod.make_vision(vc.get("model", {})))
        sight.may_move = lambda: self.state == "idle"          # 说话、想事情的时候眼睛听大模型的
        sight.on_event = self.on_sight_event                  # 有人进出、看图结果记进事件日志
        sight.faceid = FaceID(self.people, vc.get("match_threshold", 0.42))
        if not sight.cam.recognize:
            log.info("认人：关（没开或没有 SFace 模型，先运行 python scripts/download_models.py）")
            sight.faceid = None
        if await sight.start():
            self.sight = sight
            self.board.add_provider(sight.readings)

    def setup_senses(self):
        if not self.sight:
            self.board.put(senses_mod.Reading("camera.faces", "看到的人", "摄像头关着，我现在看不见",
                                              source="摄像头", location="头部前方", max_age=1e9))
        sc = self.cfg.get("senses", {})
        if sc.get("temp_hum", "fake") == "fake":
            sensor = senses_mod.FakeTempHum(sc.get("fake_temp", 22.0), sc.get("fake_humidity", 55.0))
            asyncio.create_task(senses_mod.temp_hum_loop(self.board, sensor, sc.get("temp_hum_seconds", 30)))

    async def status_loop(self):
        """隔几秒把温度、电量刷到屏幕边上，并往数据流里推一行真实读数（几种轮流来）。
        电量现在测不到，显示 BAT --；假数据后面带 *。"""
        n = 0
        while True:
            r = self.board.get("room.temp")
            st = self.chassis.status if self.chassis.online else None
            self.eyes.status(temp=None if r is None else r.value, sim=bool(r and r.confidence <= 0),
                             bat=st.get("bat") if st else None, load=st.get("load") if st else None,
                             spd=round(math.hypot(st["v"][0], st["v"][1]), 2) if st else None)
            if self.eyes.is_real:
                online = time.monotonic() - self.eyes.last_hb < 3
                if online != self.head_online:
                    self.head_online = online
                    (log.info if online else log.warning)("头部 ESP32：%s", "连上了" if online else "3 秒没收到心跳，可能掉线了")
                    self.board.put(senses_mod.Reading("head.link", "头部连接", "正常" if online else "断开",
                                                      source="心跳", location="头部 ESP32", max_age=1e9))
            line = self.telemetry(n) if self.state == "idle" else ""   # 对话时不推读数，屏幕上只看大脑每一步在干什么
            if line:
                self.eyes.log(line)
            n += 1
            await asyncio.sleep(self.cfg.get("eyes", {}).get("status_seconds", 3))

    def telemetry(self, n: int) -> str:
        """感觉板上的读数 -> 一行英文缩写。"""
        def val(key, fmt="{}"):
            r = self.board.get(key)
            if r is None:
                return "--"
            return fmt.format(r.value) + ("*" if r.confidence <= 0 else "")
        kind = 3 if n % 15 == 14 else n % 4 if self.chassis.online else n % 3     # 天气变化慢，大约每 30 秒推一次
        if kind == 0:
            snd = self.board.get("sound.level")
            db = re.match(r"-?\d+", str(snd.value)).group() if snd else "--"
            return f"SNS T{val('room.temp')}C RH{val('room.humidity', '{:.0f}')} SND {db}DB"
        if kind == 1 and self.sight:
            faces, motion, _, _ = self.sight.cam.latest()
            return f"VIS FACE {len(faces)} MOT {motion:.2f}"
        if kind == 2:
            up = (time.time() - self.started) / 60
            return f"SYS UP {up:.0f}M STATE {self.state.upper()} HIST {len(self.history) // 2}"
        if kind == 3 and n % 15 != 14 and self.chassis.online:
            st = self.chassis.status
            x, y, h = st["odo"]
            return f"BASE {st['st'][:5].upper()} V{st['v'][0]:+.2f} X{x:+.2f} Y{y:+.2f} H{h:+.0f}"
        if kind == 3 and self.weather:
            return "NET " + self.weather.short()
        return ""

    async def on_sight_event(self, name: str, desc: str):
        """摄像头的事件（有人出现、离开、认出是谁、陌生人、看图描述）记进事件日志；陌生人要主动汇报。"""
        self.memory.add_event(name, desc)
        if name == "stranger" and self.attention and self.state == "idle" and not self.privacy:
            why = self.attention.blocked("stranger")
            if why:
                log.info("看到陌生人，但先不说：%s", why)
                return
            self.state = "thinking"
            asyncio.create_task(self.proactive(
                "stranger", f"{desc}。这是安全汇报，必须说：报告发现一名未登记人员和方位；"
                            "如果【我自己感觉到的】里同时认出了登记过的人，就问他要不要登记这位（说“记住他，他叫××”）"))

    def look_when_called(self):
        """被叫到：记一下有互动，同时在后台截一张图去看（没配看图模型就不截）。"""
        if self.sight:
            self.sight.touch()
            self._sight_job = self.sight.snapshot_later("有人叫我")

    async def sensed(self) -> str:
        """【我自己感觉到的】= 感觉板上的全部读数。被叫时截的图如果还没看完，最多再等一小会儿。"""
        job, self._sight_job = self._sight_job, None
        if job and not job.done():
            try:
                await asyncio.wait_for(asyncio.shield(job), self.cfg["vision"].get("wait_on_wake_seconds", 1.0))
            except asyncio.TimeoutError:
                log.info("看图还没回来，先不等了（回来以后下一句能用上）")
        return self.board.text()

    def looked_up(self, text: str) -> str:
        """【我查到的】：话题和天气有关才放天气，免得每句话都扯天气；有屋里的读数时再附上算好的室内外对比。"""
        if self.weather and self.weather.relevant(text):
            cmp = self.compare_text()
            return self.weather.text() + (f"\n{cmp}" if cmp else "")
        return ""

    # ------------------------------------------------------------ 第⑩块：室内外对比、屋里舒不舒服
    def put_room(self, name: str, value):
        """头部 SHT40 的读数 -> 感觉板上的“室内温度 / 湿度”。温度加上校准值（头里比屋里略热，到货后对着温度计校准）。"""
        sc = self.cfg.get("senses", {})
        real = self.eyes.is_real
        if name == "room_temp":
            key, human, unit, v = "room.temp", "室内温度", "°C", round(float(value) + sc.get("temp_offset", 0.0), 1)
        else:
            key, human, unit, v = "room.humidity", "室内湿度", "%", round(float(value) + sc.get("humidity_offset", 0.0))
        self.board.put(senses_mod.Reading(
            key, human, v, unit,
            source="头部温湿度传感器（SHT40）" if real else "模拟器的假温湿度（SHT40 还没装）",
            location="头壳后下方通风口，测的是我所在位置的空气", confidence=0.9 if real else 0,
            note="" if real else "这是模拟数据，不是真的测量值；提到时要说明是模拟的", max_age=120))

    def compare_text(self) -> str:
        """程序算好的室内外对比（免得大模型自己算错）。屋里或外面的数缺一个就不给。"""
        rt, rh = self.board.get("room.temp"), self.board.get("room.humidity")
        if not (rt and self.weather and self.weather.data):
            return ""
        c = self.weather.data["current"]
        out_t, out_h = c["temperature_2m"], c["relative_humidity_2m"]
        d = rt.value - out_t
        s = (f"- 室内外对比（程序算好的，直接用这些数）：屋里 {rt.value:.1f} °C（我自己测的），外面现在 {out_t:g} °C（预报的实况），"
             + (f"屋里比外面高 {d:.1f} 度" if d > 0.4 else f"屋里比外面低 {-d:.1f} 度" if d < -0.4 else "里外差不多"))
        if rh:
            s += f"；湿度 屋里 {rh.value:.0f}%、外面 {out_h}%"
        if rt.confidence <= 0:
            s += "｜注意：屋里的读数是模拟数据"
        return s

    def comfort_issue(self) -> str:
        """屋里持续不舒服就返回原因（给主动开口用），否则返回空字符串。"""
        cc = self.cfg.get("comfort", {})
        if not cc.get("enabled", True):
            return ""
        rt, rh = self.board.get("room.temp"), self.board.get("room.humidity")
        now, issues = time.time(), {}
        if rt and now - rt.t < rt.max_age:
            if rt.value >= cc.get("hot", 28):
                issues["hot"] = f"屋里偏热（{rt.value:.1f} °C，超过 {cc.get('hot', 28)} °C）"
            elif rt.value <= cc.get("cold", 16):
                issues["cold"] = f"屋里偏冷（{rt.value:.1f} °C，低于 {cc.get('cold', 16)} °C）"
        if rh and now - rh.t < rh.max_age:
            if rh.value >= cc.get("humid", 75):
                issues["humid"] = f"屋里偏潮（湿度 {rh.value:.0f}%，超过 {cc.get('humid', 75)}%）"
            elif rh.value <= cc.get("dry", 30):
                issues["dry"] = f"屋里偏干（湿度 {rh.value:.0f}%，低于 {cc.get('dry', 30)}%）"
        self.comfort_since = {k: self.comfort_since.get(k, now) for k in issues}     # 没了的就清掉，重新计时
        lasted = [issues[k] for k, t0 in self.comfort_since.items() if now - t0 >= cc.get("minutes", 10) * 60]
        if not lasted:
            return ""
        return ("、".join(lasted) + f"，已经持续 {cc.get('minutes', 10)} 分钟以上。"
                "这是到点的环境提醒，必须说：先报屋里的读数和外面的天气，再给一个具体建议"
                "（外面比屋里舒服就建议开窗；外面也不行就建议开空调或加湿 / 除湿；"
                "【我能遥控的家电】里有能用的按键，就问用户要不要你帮他开，不要自己直接发）")

    # ------------------------------------------------------------ 语音模式
    async def run_voice(self):
        """不用唤醒词：有人看着它（脸够近、正对、持续 1 秒）就开始听，说完就回答。
        唤醒词留着当备用（看不见你的时候喊一声）。"""
        await self.setup(voice_input=True)
        li = self.cfg["listen"]
        self.look_talk = li.get("look_to_talk", True) and self.sight is not None
        self.use_wake = li.get("wake_word", True)
        how = "看着我直接说话" if self.look_talk else ""
        if self.use_wake:
            how += ("，或者" if how else "") + f"说「{self.cfg['wake']['keywords'][0]}」"
        log.info("准备好了：%s（Ctrl+C 退出）", how or "（看着说话和唤醒词都关了，config.yaml 里 listen 打开一个）")
        ac = self.cfg.get("attention", {})
        if ac.get("enabled", True) and self.sight:
            self.attention = Attention(ac, self.sight)
            asyncio.create_task(self.attention_loop())
            log.info("主动开口：开（每小时最多 %s 次，至少隔 %s 分钟）", ac.get("max_per_hour", 3), ac.get("min_gap_minutes", 10))
        async for frame in self.mic.frames():
            if self.privacy:
                continue                                   # 隐私开关打开：不听（真硬件上麦克风已经断电）
            if self.speaker.busy or self.state in ("thinking", "speaking"):
                continue                                   # 自己在说话或在想：不听
            self.sound.feed(frame)                         # 只算音量放到感觉板上，不录音
            if self.state == "idle":
                if self.use_wake and (hit := self.wake.feed(frame)):
                    self.on_wake(hit)
                elif self.look_talk and self.sight.attending():
                    first = self.attention.greeting() if self.attention else None
                    if first:                              # 你回来了 / 今天第一次见：它先开口，说完接着听
                        self.state = "thinking"
                        asyncio.create_task(self.proactive(*first, then_listen=True))
                    else:
                        self.start_listen("look")
            elif self.state == "listening":
                self.eyes.level(mic_level(frame))          # 屏幕上的音量条
                status, seg = self.listener.feed(frame)
                looking = bool(self.sight and self.sight.looking_now())
                if self.listener.heard_speech:             # 说话期间统计有多少时间是看着我的
                    self.att_frames[0] += 1
                    self.att_frames[1] += looking
                if status == "done":
                    self.on_utterance(seg)
                elif status == "timeout":
                    log.info("没听到说话，回去待机")
                    self.to_idle()
                elif (self.via in ("look", "follow") and not self.listener.heard_speech and self.sight
                      and self.sight.looked_away_for() > li.get("look_away_seconds", 1.5)):
                    log.debug("你没在看我了，停止聆听")    # 没开口就扭头、走开：不听了
                    self.to_idle()

    def is_echo(self, core: str) -> bool:
        """识别出来的话，和自己最近 30 秒说过的某句话大部分重合，就当成回声（扬声器的声音被麦克风听到了）。
        按拼音比（不看声调）：识别常把“天气晴”听成“填气情”这种同音字；数字先统一成中文读法（22 = 二十二）。"""
        heard = sounds(core)
        for until, said in self.recent_said:
            if not said or time.monotonic() > until:
                continue
            mine = sounds(said)
            m = difflib.SequenceMatcher(None, heard, mine, autojunk=False).find_longest_match(0, len(heard), 0, len(mine))
            if m.size >= max(3, 0.6 * len(heard)):
                return True
        return False

    def record(self, decision: str, **info):
        """决策日志：每句话理没理、为什么，一行一条写进 data/logs/decisions-日期.jsonl（只记文字，不录音）。
        用几天以后拿它来调 config.yaml 里 listen 的参数。"""
        row = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "decision": decision, **self.utt, **info}
        try:
            d = config.path(self.log_dir)
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, time.strftime("decisions-%Y-%m-%d.jsonl")), "a", encoding="utf-8") as f:
                f.write(json.dumps(row, ensure_ascii=False) + "\n")
        except OSError as e:
            log.warning("写决策日志失败：%s", e)

    async def attention_loop(self):
        """每秒更新一次“有没有人、在了多久”；闲着时检查要不要提醒（比如坐太久）。"""
        while True:
            await asyncio.sleep(1)
            self.attention.update()
            comfort = self.comfort_issue()
            if self.state == "idle" and not self.privacy and (p := self.attention.periodic(comfort)):
                self.state = "thinking"
                asyncio.create_task(self.proactive(*p))

    async def proactive(self, kind: str, why: str, then_listen=False):
        """主动开口：把“为什么想说”交给大模型，值得说才说（不值得它会 ignore）。"""
        if self.attention:
            self.attention.spoke(kind)              # 不管最后说没说，都算一次，免得反复问大模型
        log.info("想主动开口（%s）：%s", kind, why)
        self.eyes.log(f"ATTN {kind.upper()}")
        self.utt = {"via": "proactive", "kind": kind}
        try:
            looked = self.weather.text() if self.weather else ""
            if (cmp := self.compare_text()):
                looked += f"\n{cmp}"
            said = await self.respond(f"（没有人在问你。你想主动开口的原因：{why}）", via="proactive", looked_up=looked)
            self.record("proactive" if said else "proactive_skip", why=why, reply=self.last_reply)
            if not said:
                log.info("大模型觉得不值得说，保持安静")
        finally:
            if self.after_interrupt():
                pass
            elif then_listen and self.sight and self.sight.attending():
                self.start_listen("look")
            else:
                self.to_idle()

    # ------------------------------------------------------------ 头部 ESP32 发来的事件
    async def on_head_event(self, ev: dict):
        kind = ev["ev"]
        if kind == "touch":
            self.on_touch(ev["zone"], ev["action"])
        elif kind == "sensor" and ev["name"] in ("room_temp", "room_humidity"):
            self.put_room(ev["name"], ev["value"])
        elif kind == "sensor":
            name = ev["name"]
            human, unit, where = protocol.SENSORS[name]
            value = ev["value"]
            if name == "presence":
                value = "有人" if value else "没人"
            self.board.put(senses_mod.Reading(f"head.{name}", human, value, ev.get("unit") or unit,
                                              source="头部 ESP32 传感器", location=where, confidence=0.8, max_age=30))
        elif kind == "privacy":
            self.set_privacy(ev["on"])
        elif kind == "ir":
            self.on_ir_event(ev)
        elif kind == "thermal":
            self.on_thermal(ev)
        elif kind == "hello":
            log.info("头部 ESP32 连上了：固件 %s，功能 %s", ev.get("fw", "?"), "、".join(ev.get("caps", [])) or "未说明")
            self.eyes.log(f"HEAD FW {ev.get('fw', '?')}")
        elif kind == "error":
            log.warning("头部 ESP32 报错：%s %s", ev.get("code", ""), ev.get("text", ""))
            self.eyes.log(f"HEAD ERR {ev.get('code', '')}")

    def on_touch(self, zone: str, action: str):
        """点屏幕：平时 = 唤醒并确认（闪 ONLINE + 提示音 + 开始听）；它在想、在说、在走、在学遥控器时 = 马上停下来听你说。
        长按先只记一笔，以后再定用途。（2026-10-17 取消了头顶、下巴触摸，只剩屏幕。）"""
        if self.privacy or action == "release":
            return
        if action == "hold":
            log.info("长按了屏幕（暂时没有用途）")
            self.eyes.log("TOUCH SCREEN HOLD")
            return
        self.eyes.log("TOUCH SCREEN")
        if self.state in ("thinking", "speaking", "learning"):
            self.interrupt("listen")
        else:
            if self.motion_job and not self.motion_job.done():   # 正在走：先停
                self.halt()
            self.touch_wake()

    def touch_wake(self):
        """点屏幕唤醒并确认：马上响提示音、屏幕闪 ONLINE，然后开始听（不用看着它、不用喊唤醒词）。"""
        log.info("点了屏幕：唤醒")
        self.speaker.play(*self.ack)
        self.eyes.mode("wake")
        if hasattr(self, "listener"):                    # 文字模式没有麦克风，只做屏幕确认
            self.start_listen("touch", wait=self.cfg["listen"].get("wait_seconds", 5))
        self.look_when_called()

    def interrupt(self, then: str):
        """它正在想、在说的时候被点了屏幕：马上停，后面的话都不说了。"""
        log.info("被打断了：%s", "停下来听你说" if then == "listen" else "停下来待机")
        self.eyes.log("TOUCH STOP")
        self.interrupted = then
        self.speaker.stop()
        self.ir_pending = None
        self.halt()                                      # 正在走：马上停
        if self.ir_wait and not self.ir_wait.done():     # 正在学遥控器：不学了
            self.ir_wait.set_result({"ev": "ir", "op": "cancelled"})

    def after_interrupt(self) -> bool:
        """回答结束时如果是被打断的，按打断的意思收尾，返回 True。"""
        then, self.interrupted = self.interrupted, ""
        if then == "listen":
            self.touch_wake()
            return True
        if then == "idle":
            self.to_idle()
            return True
        return False

    def set_privacy(self, on: bool):
        """隐私开关：拨到“隐私”= 不听、不看、不主动说（真硬件上相机和麦克风已经断电）；拨回来恢复。"""
        if on == self.privacy:
            return
        self.privacy = on
        if self.sight:
            self.sight.paused = on
        if on:
            log.info("隐私开关打开：不听、不看")
            if self.state in ("thinking", "speaking", "learning"):
                self.interrupt("idle")
            elif self.state == "listening":
                self.to_idle()
            self.eyes.log("PRIVACY ON")
            self.eyes.mode("sleep", text="PRIVACY")
        else:
            log.info("隐私开关关上：恢复")
            self.eyes.log("PRIVACY OFF")
            self.to_idle()

    def start_listen(self, via: str, wait=None):
        """进入聆听。via：look 看着我 / wake 唤醒词 / follow 刚聊完接着说。"""
        self.via = via
        self.att_frames = [0, 0]
        self.listener.start(wait=3600 if via == "look" else wait)   # 看着我的时候一直听，靠“扭头”来结束
        self.state = "listening"
        self.eyes.mode("listen")
        if via == "look":
            log.info("你在看着我，开始听")
            if self.cfg["react"].get("listen_beep", True):
                self.speaker.play(*self.chirp)
            self.eyes.log("ATTN FACE LOCK LISTEN OPEN")
            self.look_when_called()

    def on_wake(self, word):
        """唤醒词（备用）：本地先反应，马上出声 + 屏幕 ONLINE。"""
        t0 = time.perf_counter()
        self.speaker.play(*self.ack)
        self.eyes.mode("wake")                           # 闪一下 + ONLINE，0.9 秒后自动进入聆听
        log.info("听到唤醒词「%s」，%.0f ms 内开始出声", word, (time.perf_counter() - t0) * 1000)
        self.eyes.log("KWS WAKE")
        self.start_listen("wake")
        self.look_when_called()

    def to_idle(self):
        self.state = "idle"
        self.via = ""
        if hasattr(self, "wake"):
            self.wake.reset()
        if self.sight:
            self.sight.att_since = None                  # 要重新看满 1 秒才会再开始听
        self.eyes.mode("idle")

    def on_utterance(self, seg):
        """说完一句。看着我触发的，先过两道关：时长够不够、说话时是不是一直看着我。"""
        li = self.cfg["listen"]
        dur = len(seg) / 16000
        looked = self.att_frames[1] / max(1, self.att_frames[0])
        self.utt = {"via": self.via, "seconds": round(dur, 2), "looked": round(looked, 2)}
        self.eyes.log(f"VAD SEG {dur:.2f}S LOOK {looked * 100:.0f}% VIA {self.via.upper()}")
        if self.via in ("look", "follow"):
            if dur < li.get("min_speech_seconds", 0.5):
                return self.skip(f"太短（{dur:.1f} 秒）", "SHORT")
            if self.via == "look" and looked < li.get("attention_ratio", 0.6):
                return self.skip(f"说话时只有 {looked:.0%} 的时间看着我", "NOT FACING")
        self.state = "thinking"
        self.eyes.mode("think")
        asyncio.create_task(self.handle_speech(seg))

    def skip(self, why: str, code: str, text: str = ""):
        """这句不理：回到聆听（人还看着我）或者待机。"""
        log.info("不理这句：%s", why)
        self.eyes.log(f"IGN {code}")
        self.record("ignore", reason=code, why=why, text=text)
        if self.sight and self.sight.attending():
            self.start_listen("look")
        else:
            self.to_idle()

    async def handle_speech(self, samples):
        t0 = time.perf_counter()
        text = await asyncio.to_thread(self.asr.transcribe, samples)
        log.info("识别用时 %.2f 秒（%.1f 秒录音）", time.perf_counter() - t0, len(samples) / 16000)
        self.eyes.log(f"ASR SENSEVOICE {len(samples) / 16000:.1f}S AUDIO -> {len(text)}CH {time.perf_counter() - t0:.2f}S")
        core = re.sub(r"[\W_]+", "", text)
        li = self.cfg["listen"]
        if not core:                                     # 只有标点或空：大概是噪音
            return self.skip("识别结果为空", "EMPTY")
        if self.via in ("look", "follow") and (len(core) < li.get("min_chars", 2) or core in li.get("fillers", [])):
            return self.skip(f"「{text}」只是语气词", "FILLER", text)
        if self.via in ("look", "follow") and self.is_echo(core):
            return self.skip(f"「{text}」是我自己刚说过的话（回声）", "ECHO", text)
        print(f"\n你：{text}")
        answered = await self.respond(text, via=self.via)
        if self.after_interrupt():
            return
        if not answered:
            return self.skip("大模型判断这句不是对我说的", "NOT FOR ME", text)
        self.record("answer", text=text, reply=self.last_reply, resp_seconds=self.utt.get("resp"))
        if self.sight and self.look_talk and self.sight.attending():
            self.start_listen("look")                    # 还看着我：接着听
            return
        follow = li["follow_up_seconds"]
        if follow > 0:
            log.info("%d 秒内可以直接接着说", follow)
            self.start_listen("follow", wait=follow)
        else:
            self.to_idle()

    # ------------------------------------------------------------ 文字模式（不用麦克风，测大模型和语音合成）
    async def run_text(self):
        await self.setup(voice_input=False)
        print("文字模式：直接打字和它说话，回车发送；输入 q 退出。")
        while True:
            try:
                text = (await asyncio.to_thread(input, "\n你：")).strip()
            except EOFError:
                break
            if text.lower() in ("q", "quit", "exit"):
                break
            if text:
                self.speaker.play(*self.ack)
                self.eyes.mode("wake")
                self.look_when_called()
                await self.respond(text, via="text")
                self.state = "idle"
                self.eyes.mode("idle")

    # ------------------------------------------------------------ 想 + 说
    async def respond(self, text: str, via: str = "", looked_up: str = None) -> bool:
        """想 + 说。返回 False 表示大模型判断这句不是对它说的（ignore），什么都没说。"""
        self.state = "thinking"
        self.interrupted = ""
        self.ir_sent = []                                # 这次回答发了哪些红外按键
        t0 = self._t0 = time.perf_counter()
        first_audio = []
        self.eyes.mode("think")
        say_q: asyncio.Queue = asyncio.Queue()           # 里面放的是“已经开始要声音”的句子，按说话顺序排

        async def play_worker():                          # 每句一出来就去要声音（服务那边排队合成），播放按顺序、边收边播
            while (u := await say_q.get()) is not None:
                if self.interrupted:                       # 被点了屏幕：后面的都不说了
                    u.cancel()
                    continue
                if await u.first() is None or self.interrupted:
                    continue
                if not first_audio:
                    first_audio.append(time.perf_counter() - t0)
                    log.info("从说完到开口：%.2f 秒", first_audio[0])
                    self.eyes.log(f"TTS 1ST AUDIO {first_audio[0]:.2f}S")
                self.state = "speaking"
                self.eyes.mode("speak")
                self.utt["resp"] = round(first_audio[0], 2)
                await self.speaker.play_stream(u.chunks(), u.sr, on_chunk=self.level_chunk(u.sr))
                self.recent_said = [x for x in self.recent_said if time.monotonic() < x[0]]   # 只留 30 秒内的

        def say(s):
            say_q.put_nowait(self.voice.say(s))

        workers = [asyncio.create_task(play_worker())]
        context = {"sensed": await self.sensed(),
                   "looked_up": self.looked_up(text) if looked_up is None else looked_up,
                   "memory": self.memory.recall(text),
                   "devices": self.ir.text(),
                   "via": via}
        messages = prompt.build(self.persona, self.history, text, can_move=self.chassis.kind != "none", **context)
        log.debug("这次发给大模型的三栏：\n%s", messages[-1]["content"])
        n_sense = context["sensed"].count("\n") + 1 if context["sensed"] else 0
        n_mem = context["memory"].count("\n") + 1 if context["memory"] else 0
        self.eyes.log(f"CTX SENSE {n_sense} MEM {n_mem} HIST {len(self.history) // 2} WX {'Y' if context['looked_up'] else 'N'}",
                      f"LLM REQ {getattr(self.llm, 'model', 'MOCK')} {sum(len(m['content']) for m in messages)}CH")
        raw, said = [], []
        ignored = False
        try:
            async for line in llm_mod.lines(self.llm.stream(messages)):
                if self.interrupted:
                    break
                call, err = skills.parse(line)
                if err:
                    log.warning("忽略一行不合规的输出：%s", err)
                if call and call["skill"] == "ignore":
                    if not said:                    # 还没开口就说不理：整句不回答
                        ignored = True
                        break
                    continue
                if call:
                    raw.append(line.strip())
                    if await self.run_skill(call, say, said) == "stop":   # 比如要发的遥控码没学过：后面的话不说了
                        break
            self.eyes.log(f"LLM END {len(raw)} CALLS {time.perf_counter() - t0:.2f}S" + (" IGNORE" if ignored else ""))
        except Exception as e:
            log.error("大模型调用失败（%s）：%s", type(e).__name__, e)
            self.eyes.log(f"LLM ERR {type(e).__name__[:20]}")
            self.eyes.mode("error", text="E-NET", hold=2.5)
            say(FALLBACK)
            said.append(FALLBACK)
        if not said and self.ir_sent and not ignored:   # 发了红外码却一句没说：补一句，让人知道发了
            msg = "已发送：" + "、".join(self.ir_sent) + "。"
            print(f"{self.persona['name']}：{msg}")
            say(msg)
            said.append(msg)
        elif not said and not raw and not ignored:
            say("未生成回答。")
        await say_q.put(None)
        await asyncio.gather(*workers)
        if self.ir_pending and not self.interrupted and not ignored:   # 话说完了，开始等遥控器
            msg = await self.ir_learn_flow(*self.ir_pending)
            if msg:
                said.append(msg)
                raw.append(json.dumps({"skill": "say", "text": msg}, ensure_ascii=False))
        self.ir_pending = None
        if self.enroll_pending and not self.interrupted and not ignored:   # 话说完了，开始看脸登记
            msg = await self.enroll_flow(*self.enroll_pending)
            if msg:
                said.append(msg)
                raw.append(json.dumps({"skill": "say", "text": msg}, ensure_ascii=False))
        self.enroll_pending = None
        if raw:
            self.history += [{"role": "user", "content": text}, {"role": "assistant", "content": "\n".join(raw)}]
            self.history = self.history[-2 * self.cfg["llm"]["history_turns"]:]
        if ignored:
            log.info("大模型判断不是对我说的，不回答")
        self.last_reply = said
        if said and not ignored:                     # 这轮对话记进事件日志
            if via == "proactive":
                self.memory.add_event("proactive", f"我主动说：{''.join(said)}")
            else:
                self.memory.add_event("chat", f"用户说：{text}｜我回答：{''.join(said)}")
        return not ignored

    async def run_skill(self, call: dict, say, said: list):
        name = call["skill"]
        if name == "say":
            if not said:
                log.debug("大模型第一句话到达：%.2f 秒", time.perf_counter() - self._t0)
                self.eyes.log(f"LLM 1ST TOKEN {time.perf_counter() - self._t0:.2f}S")
            print(f"{self.persona['name']}：{call['text']}")
            self.eyes.log(f"SKL SAY {len(call['text'])}CH")
            said.append(call["text"])
            say(call["text"])
            self.recent_said.append((time.monotonic() + 30, re.sub(r"[\W_]+", "", call["text"])))
        elif name in skills.MOTION:
            self.motion_skill(call)
        elif name == "display":
            m = call["mode"]
            self.eyes.mode(m, hold=0 if m == "idle" else 6)          # 扫描、热成像、警告至少显示 6 秒
            self.eyes.log(f"SKL DISPLAY {m}")
        elif name == "look":
            self.eyes.gaze(*eyes_mod.GAZE[call["direction"]])
            self.eyes.log(f"SKL LOOK {call['direction']}")
            hold = self.cfg.get("eyes", {}).get("look_hold_seconds", 3)
            asyncio.get_running_loop().call_later(hold, self.eyes.gaze, 0, 0)
        elif name == "remember":
            how = self.memory.add_fact(call["fact"])
            self.eyes.log("MEM +1 FACT" if how == "new" else "MEM FACT UPDATED")
            log.info("%s：%s", "记住了" if how == "new" else "更新了一条记忆", call["fact"])
        elif name == "forget":
            gone = self.memory.forget(call["fact"])
            self.eyes.log("MEM -1 FACT" if gone else "MEM FORGET MISS")
            log.info("忘掉了：%s" if gone else "没找到要忘掉的：%s", gone or call["fact"])
        elif name == "quiet":
            self.quiet_until = time.monotonic() + call["minutes"] * 60
            if self.attention:
                self.attention.quiet_until = time.time() + call["minutes"] * 60
            self.eyes.log(f"QUIET {call['minutes']}M")
            log.info("安静 %s 分钟：这段时间不主动开口，被叫到还会回答", call["minutes"])
        elif name == "ir_send":
            hit = self.ir.find(call["device"], call["action"])
            if not hit:
                msg = f"还没学过{call['device']}的“{call['action']}”键。要学的话，说“学一下{call['device']}的{call['action']}”。"
                log.info("要发的遥控码没学过：%s %s", call["device"], call["action"])
                self.eyes.log("IR TX MISS")
                print(f"{self.persona['name']}：{msg}")
                said.append(msg)
                say(msg)
                return "stop"
            dev, act, code = hit
            self.eyes.send_ir({"op": "send", "code": code})
            self.eyes.log(f"IR TX {code.get('proto', 'RAW')}")
            log.info("红外发出：%s %s", dev, act)
            self.ir_sent.append(f"{dev} {act}")
            self.memory.add_event("ir", f"用红外发出：{dev} {act}")
        elif name == "enroll":
            self.enroll_pending = (call["name"].strip(), call["relation"])
            self.eyes.log("FACE ENROLL REQ")
        elif name == "forget_person":
            gone = self.people.forget(call["name"])
            self.eyes.log("FACE FORGET" if gone else "FACE FORGET MISS")
            log.info("删掉了登记的人：%s" if gone else "要删的人没登记过：%s", call["name"])
        elif name == "ir_learn":
            self.ir_pending = (call["device"], call["action"])
            self.eyes.log("IR LEARN REQ")
        elif name == "ir_forget":
            every = call["action"] in ("全部", "所有", "全部按键", "所有按键")
            n = self.ir.forget(call["device"], "" if every else call["action"])
            self.eyes.log(f"IR FORGET {n}")
            log.info("忘掉了 %d 个遥控按键：%s %s", n, call["device"], "全部" if every else call["action"])

    # ------------------------------------------------------------ 底盘：动作
    def motion_skill(self, call: dict):
        """大模型的动作技能：stop 马上执行；其余排队，在后台按顺序做（和说话同时进行）。"""
        name = call["skill"]
        if name == "stop":
            self.halt()
            self.eyes.log("SKL BASE STOP")
            log.info("停下")
            return
        self.motions.append(call)
        self.eyes.log("SKL BASE " + {"move": f"{call.get('direction', '')[:4].upper()} {call.get('meters', 0):.2f}M",
                                     "turn": f"TURN {call.get('degrees', 0):+.0f}", "posture": call.get("pose", "").upper()}[name])
        if not self.motion_job or self.motion_job.done():
            self.motion_job = asyncio.create_task(self.motion_run())

    async def motion_run(self):
        """动作在后台做，不挡着听（走的时候还能听到“停”）。没走成：等它空下来再说原因。"""
        msg = await self.motion_flow()
        if not msg:
            return
        for _ in range(150):                                # 正在说话或想事情：最多等 15 秒
            if self.state == "idle":
                break
            await asyncio.sleep(0.1)
        if self.state == "idle" and not self.privacy:
            self.state = "speaking"
            await self.speak_now(msg)
            self.to_idle()
        else:
            log.info("没走成，但一直没空说：%s", msg)

    def halt(self):
        """马上停：清掉排队的动作，让底盘刹车。"""
        self.motions.clear()
        if self.chassis.kind != "none":
            self.chassis.stop()

    async def motion_flow(self) -> str:
        """按顺序做排队的动作。全做完返回空字符串（响一声提示音）；做不成返回要说的话（被叫停的不用说，也返回空）。"""
        mc = self.cfg.get("chassis", {})
        v = mc.get("speed", 0.4)
        did = []
        while self.motions:
            call = self.motions.pop(0)
            name = call["skill"]
            if self.chassis.kind == "none":
                self.motions.clear()
                return "底盘没有接上，我现在不能移动。"
            if name in ("move", "turn") and self.chassis.state == "folded":
                log.info("趴着：先站起来")
                ok, why = await self.chassis.pose("stand")
                if not ok:
                    return self.motion_fail("站起来", why)
            if name == "posture":
                what = "站起来" if call["pose"] == "stand" else "趴下"
                ok, why = await self.chassis.pose(call["pose"])
            elif name == "turn":
                what = f"{'左' if call['degrees'] > 0 else '右'}转 {abs(call['degrees']):g} 度"
                ok, why = await self.chassis.move(dth=call["degrees"], v=v)
                if not ok and why == "FOLDED":                       # 状态还没刷新（刚趴下）：先站起来再来一次
                    ok, why = await self.chassis.pose("stand")
                    if ok:
                        ok, why = await self.chassis.move(dth=call["degrees"], v=v)
            else:
                d = call["direction"]
                m = min(call["meters"], mc.get("max_meters", 3.0))
                what = {"forward": "往前", "backward": "往后", "left": "往左", "right": "往右"}[d] + f"走 {m:g} 米"
                dx = m if d == "forward" else -m if d == "backward" else 0.0
                dy = m if d == "left" else -m if d == "right" else 0.0
                ok, why = await self.guarded_move(dx, dy, v)
                if not ok and why == "FOLDED":
                    ok, why = await self.chassis.pose("stand")
                    if ok:
                        ok, why = await self.guarded_move(dx, dy, v)
            log.info("%s：%s", what, "完成" if ok else f"没成（{why}）")
            self.eyes.log(f"BASE DONE" if ok else f"BASE FAIL {why}")
            if not ok:
                self.motions.clear()
                return self.motion_fail(what, why)
            did.append(what)
            self.memory.add_event("move", f"移动：{what}")
        if did:
            self.speaker.play(*self.chirp)
        return ""

    def motion_fail(self, what, why) -> str:
        if why == "STOPPED":                                # 是用户让停的（stop 技能、点屏幕打断）：不用解释
            return ""
        if why.startswith("OBSTACLE"):
            return f"前方 {why.split(':')[1]} 厘米有障碍，已停下。"
        return f"没能{what}：{chassis_mod.WHY_CN.get(why, why)}。"

    async def guarded_move(self, dx, dy, v):
        """走一段；往前走时盯着下巴的测距，前面太近就停。"""
        stop_mm = self.cfg.get("chassis", {}).get("stop_distance", 0.35) * 1000
        job = asyncio.create_task(self.chassis.move(dx=dx, dy=dy, v=v))
        while not job.done():
            await asyncio.sleep(0.1)
            r = self.board.get("head.distance")
            if (dx > 0 and r is not None and r.confidence > 0 and time.time() - r.t < 2
                    and isinstance(r.value, (int, float)) and r.value < stop_mm):
                self.chassis.stop()
                await job
                return False, f"OBSTACLE:{r.value / 10:.0f}"
        return job.result()

    async def on_chassis_event(self, ev: dict):
        """底盘的提醒和故障：记日志、上屏幕；急停、故障、电量低换屏幕模式。"""
        kind, code = ev["ev"], ev.get("code", "")
        if kind == "hello":
            log.info("底盘连上了：固件 %s", ev.get("fw", "?"))
            self.eyes.log(f"BASE FW {ev.get('fw', '?')}")
            return
        (log.warning if kind == "fault" else log.info)("底盘%s：%s %s", "故障" if kind == "fault" else "提醒", code, ev.get("text", ""))
        self.eyes.log(f"BASE {code}")
        self.memory.add_event("base", f"底盘{'故障' if kind == 'fault' else '提醒'}：{code} {ev.get('text', '')}")
        if code in ("ESTOP_BTN", "ESTOP_CMD"):
            self.eyes.mode("alert", text="E-STOP", hold=3)
        elif kind == "fault":
            self.eyes.mode("error", text=code[:6], hold=3)
        elif code == "BAT_LOW":
            self.eyes.mode("low_bat", hold=3)
        elif code == "CLIFF":
            self.eyes.mode("alert", text="CLIFF", hold=3)

    # ------------------------------------------------------------ 热成像
    def on_thermal(self, ev: dict):
        """收到一帧热画面：分析，放上感觉板；有持续的高温点就主动提醒。"""
        th = self.cfg.get("thermal", {})
        frame = thermal_mod.decode(ev)
        a = self.thermal = thermal_mod.analyze(frame, fov=tuple(th.get("fov", (55, 35))), mirror=th.get("mirror", False),
                                               min_rise=th.get("min_rise", 4.0))
        real = self.eyes.is_real
        self.board.put(senses_mod.Reading(
            "thermal", "热成像", thermal_mod.describe(a),
            source="头部热成像（MLX90640）" if real else "模拟器的假房间（热成像模块还没装）",
            location="右肩，视野约 55°×35°，只看得到正前方", confidence=0.8 if real else 0, max_age=5,
            note="测的是物体表面温度，不是空气温度；热源是什么是按温度和大小猜的，不一定对"))
        now = time.monotonic()
        limit = th.get("alert_temp", 60)
        if a["max"] < limit:
            if a["max"] < limit - 5:                       # 降下来了：下次再热起来可以再提醒
                self.hot_since = None
            return
        if self.hot_since is None:
            self.hot_since = now
            self.eyes.log(f"TH HOT {a['max']:.0f}C")
        lasted = now - self.hot_since
        if (lasted >= th.get("alert_seconds", 5) and now - self.hot_alerted >= th.get("alert_repeat_minutes", 10) * 60
                and self.state == "idle" and not self.privacy):
            self.hot_alerted = now
            spot = a["hot"][0] if a["hot"] else {"where": "画面里", "peak": a["max"], "guess": ""}
            log.warning("热成像发现高温：%s %.1f°C，已经 %.0f 秒", spot["where"], spot["peak"], lasted)
            self.state = "thinking"
            self.eyes.mode("alert", text=f"HOT {spot['peak']:.0f}C", hold=2)
            why = (f"热成像发现高温点：{spot['where']} {spot['peak']:.1f}°C，已经持续 {lasted:.0f} 秒（{spot['guess']}）。"
                   "这是安全提醒，必须说：报出温度和方向，提醒用户去确认是不是忘了关火或关电器")
            self.memory.add_event("thermal", f"热成像发现高温点：{spot['where']} {spot['peak']:.1f}°C")
            asyncio.create_task(self.proactive("hot", why))

    # ------------------------------------------------------------ 红外遥控
    async def ir_learn_flow(self, device: str, action: str) -> str:
        """学一个遥控按键：让 ESP32 等信号，屏幕显示 SCAN + IR LEARN；学到就存，然后汇报。返回汇报的话。"""
        timeout = self.cfg.get("ir", {}).get("learn_seconds", 15)
        self.state = "learning"
        self.ir_wait = asyncio.get_running_loop().create_future()
        self.eyes.mode("scan", text="IR LEARN")
        self.eyes.log("IR LEARN WAIT")
        self.eyes.send_ir({"op": "learn", "timeout": timeout})
        log.info("等遥控器信号：%s %s（最多 %d 秒）", device, action, timeout)
        try:
            ev = await asyncio.wait_for(self.ir_wait, timeout + 2)    # ESP32 自己会报超时，多等 2 秒是防它没回
        except asyncio.TimeoutError:
            ev = {"op": "timeout"}
        finally:
            self.ir_wait = None
        op = ev["op"]
        if op == "cancelled":
            self.eyes.send_ir({"op": "cancel"})
            log.info("红外学习被打断了")
            return ""
        if op == "learned":
            try:
                how = self.ir.save(device, action, ev["code"])
            except ValueError as e:
                op, ev = "fail", {"text": str(e)}
            else:
                self.speaker.play(*self.chirp)
                self.eyes.log("IR LEARNED " + ev["code"].get("proto", "RAW"))
                self.memory.add_event("ir", f"学会了遥控按键：{device} {action}")
                log.info("学会了：%s %s", device, action)
                msg = f"{'已更新' if how == 'updated' else '已学会'}：{device} {action}。"
        if op == "timeout":
            self.eyes.send_ir({"op": "cancel"})
            self.eyes.log("IR LEARN TIMEOUT")
            log.info("%d 秒内没收到红外信号", timeout)
            msg = "没有收到红外信号，学习已取消。把遥控器靠近我的右肩再试一次。"
        elif op == "fail":
            self.eyes.log("IR LEARN FAIL")
            log.warning("红外学习出错：%s", ev.get("text", ""))
            msg = "红外学习出错，没有存下来。请再试一次。"
        await self.speak_now(msg)
        return msg

    # ------------------------------------------------------------ 认人：登记
    async def enroll_flow(self, name: str, relation: str) -> str:
        """看着面前最大那张脸收集 5 组特征，存起来。返回汇报的话。"""
        if not (self.sight and self.sight.faceid):
            msg = "摄像头或认人功能没开，无法登记。"
            await self.speak_now(msg)
            return msg
        self.state = "learning"
        self.eyes.mode("scan", text="FACE SCAN")
        self.eyes.log("FACE ENROLL WAIT")
        log.info("登记 %s（%s）：看着面前最大那张脸收集特征", name, relation)
        vecs = await self.sight.collect(5, self.cfg.get("vision", {}).get("enroll_seconds", 8))
        if self.interrupted:
            return ""
        if len(vecs) < 3:
            msg = "没有看清脸，登记取消。请正对我、离近一点再试一次。"
            self.eyes.log("FACE ENROLL FAIL")
        else:
            other, _, sim = self.sight.faceid.identify(np.mean(vecs, axis=0))
            if other and other != name and sim >= 0.55:
                msg = f"这张脸已经登记过，是{other}。要改名就先让我忘掉{other}。"
                self.eyes.log("FACE ENROLL DUP")
            else:
                how = self.people.add(name, relation, vecs)
                self.sight.tracker.reset()                       # 马上按新登记的重新认
                self.sight.who = None
                self.speaker.play(*self.chirp)
                self.eyes.log("FACE ENROLLED")
                self.memory.add_event("people", f"登记了人脸：{name}（{relation}）")
                msg = f"{'已更新' if how == 'updated' else '已登记'}：{name}，{relation}。"
        log.info(msg)
        await self.speak_now(msg)
        return msg

    def on_ir_event(self, ev: dict):
        op = ev["op"]
        if op in ("learned", "timeout", "fail") and self.ir_wait and not self.ir_wait.done():
            self.ir_wait.set_result(ev)
        elif op == "sent":
            self.eyes.log("IR TX OK")
        elif op == "fail":
            log.warning("红外发送出错：%s", ev.get("text", ""))
            self.eyes.log("IR TX FAIL")
        else:
            log.info("收到红外事件 %s，但现在不在学习，不管它", op)

    async def speak_now(self, text: str):
        """不经过大模型，直接说一句（比如红外学习的结果）。"""
        print(f"{self.persona['name']}：{text}")
        u = self.voice.say(text)
        if await u.first() is None or self.interrupted:
            return
        self.state = "speaking"
        self.eyes.mode("speak")
        self.recent_said.append((time.monotonic() + 30, re.sub(r"[\W_]+", "", text)))
        await self.speaker.play_stream(u.chunks(), u.sr, on_chunk=self.level_chunk(u.sr))

    def level_chunk(self, sr):
        """边收边播时，每段开始播就把这一段的音量（每 0.05 秒一个）排好时间发给屏幕，声波跟着动。"""
        loop = asyncio.get_running_loop()

        def on_chunk(pcm):
            for i, v in enumerate(envelope(pcm, sr)):
                loop.call_later(i * 0.05, self.eyes.level, v)
        return on_chunk


_CN_DIGITS = "零一二三四五六七八九"


def read_number(num: str) -> str:
    """阿拉伯数字 -> 中文读法：22 -> 二十二，23.5 -> 二十三点五，2026 -> 二零二六（四位以上按位读）。"""
    whole, _, frac = num.partition(".")
    n = int(whole) if whole else 0
    if n >= 10000 or (len(whole) >= 4 and not frac):
        out = "".join(_CN_DIGITS[int(d)] for d in whole)
    elif n == 0:
        out = "零"
    else:
        out, units = "", ["", "十", "百", "千"]
        digits = list(map(int, str(n)))
        for i, d in enumerate(digits):
            pos = len(digits) - 1 - i
            if d:
                out += ("" if (d == 1 and pos == 1 and i == 0) else _CN_DIGITS[d]) + units[pos]
            elif out and not out.endswith("零") and any(digits[i:]):
                out += "零"
        out = out.rstrip("零")
    return out + ("点" + "".join(_CN_DIGITS[int(d)] for d in frac) if frac else "")


def sounds(text: str) -> list:
    """一句话 -> 拼音音节列表（不带声调），用来比较两句话“听起来”是不是一样。"""
    from pypinyin import lazy_pinyin
    text = re.sub(r"\d+(?:\.\d+)?", lambda m: read_number(m.group()), text)
    return [p for p in lazy_pinyin(text) if p.strip()]


def mic_level(frame) -> float:
    """麦克风一小段声音 -> 0–1 的音量（-55 dBFS 算 0，-20 dBFS 算 1）。"""
    db = 20 * math.log10(float(np.sqrt(np.mean(frame ** 2))) + 1e-9)
    return max(0.0, min(1.0, (db + 55) / 35))


def envelope(pcm, sr, step=0.05):
    """合成好的语音 -> 每 step 秒一个 0–1 的音量。"""
    n = max(1, int(sr * step))
    rms = [float(np.sqrt(np.mean(pcm[i:i + n] ** 2))) for i in range(0, len(pcm), n)]
    top = max(0.02, float(np.percentile(rms, 95))) if rms else 1.0
    return [min(1.0, v / top) for v in rms]

