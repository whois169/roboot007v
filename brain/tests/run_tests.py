"""一键回归测试：每次改完代码跑一遍，确认没改坏。

用法（在 brain 文件夹里）：
    python tests/run_tests.py          全部测试（要联网调用大模型，花几分钱，约 2–3 分钟）
    python tests/run_tests.py --quick  只跑不联网的部分（约 30 秒）

不出声：扬声器会被换成“假扬声器”。需要摄像头的地方用“假的看着我”信号代替。
"""
import asyncio
import json
import logging
import os
import sys
import time
import traceback

BRAIN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(BRAIN)
sys.path.insert(0, BRAIN)
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
for s in (sys.stdout, sys.stderr):
    if hasattr(s, "reconfigure"):
        s.reconfigure(encoding="utf-8", errors="replace")
logging.basicConfig(level=logging.WARNING, format="      [%(name)s] %(message)s")

import numpy as np  # noqa: E402

from dog import config, hud, prompt, skills  # noqa: E402

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print(f"  {'通过' if ok else '失败'}  {name}" + (f"（{detail}）" if detail else ""), flush=True)


SKIPPED = []


def skip(name, why):
    SKIPPED.append(name)
    print(f"  跳过  {name}（{why}）", flush=True)


def section(title):
    print(f"\n== {title}", flush=True)


# ---------------------------------------------------------------- 不联网：基础部分
def test_basics():
    section("基础：配置、技能检查、提示词")
    cfg = config.load()
    check("config.yaml 能读", "persona" in cfg and "listen" in cfg)
    ok_lines = ['{"skill":"say","text":"收到。"}', '{"skill":"display","mode":"scan"}', '{"skill":"ignore"}',
                '{"skill":"look","direction":"left"}', '{"skill":"quiet","minutes":10}']
    check("合法技能都能通过", all(skills.parse(x)[0] for x in ok_lines))
    bad_lines = ['{"skill":"display","mode":"happy"}', '{"skill":"quiet","minutes":500}', '{"skill":"motor","speed":9}',
                 '{"skill":"say","text":"x","extra":1}', 'hello']
    check("不合法技能都被拦下", all(skills.parse(x)[0] is None for x in bad_lines))
    sp = prompt.system_prompt(cfg["persona"])
    check("系统提示词里没有宠物化说法", not any(w in sp for w in ("主人", "小狗跟", "撒娇吧")), "")
    check("系统提示词写了 ignore 规则", '{"skill":"ignore"}' in sp)
    um = prompt.user_message("你好", sensed="- x", via="look")
    check("用户消息带触发方式和三栏", all(k in um for k in ("【触发方式】", "【我自己感觉到的】", "【我查到的】", "【我的记忆】", "【用户说】")))


def test_hud():
    section("HUD：指令检查、12 个状态都能画")
    import pygame
    good = [{"mode": "track", "gaze": [0.3, -0.1], "level": 0.6}, {"log": ["FACE 1 MOT 0.02"]},
            {"status": {"temp": 22.1, "sim": True, "bat": None}}, {"mode": "error", "text": "E-NET", "hold": 2.5}]
    check("合法指令通过", all(hud.check(c) == "" for c in good))
    bad = [{"mode": "happy"}, {"gaze": [1]}, {"level": 3}, {"foo": 1}, {"log": "X" * 99}]
    check("不合法指令被拦下", all(hud.check(c) for c in bad))
    check("两套字体字形都完整", all(len(g) == 7 and all(len(r) == 5 for r in g) for g in hud.GLYPHS.values())
          and all(len(g) == 5 and all(len(r) == 4 for r in g) for g in hud.SMALL.values()))
    r = hud.HudRenderer(hud.Gfx(pygame.Surface((hud.W, hud.H))), now=0.0, seed=1)
    t, err = 0.0, ""
    try:
        for m in hud.MODES:
            r.apply(json.loads(json.dumps({"mode": m, "log": [f"TEST {m.upper()}"], "level": 0.5, "gaze": [0.2, 0.1]})))
            for _ in range(30):
                t += 1 / 30
                r.tick(t)
    except Exception as e:
        err = f"{type(e).__name__}: {e}"
    check("12 个状态各跑 1 秒不出错", not err, err)
    check("眼睛在聆听等状态会出现、待机会消失", STYLE_OK())


def STYLE_OK():
    return all(hud.STYLE[m]["eyes"] == 1 for m in ("wake", "listen", "think", "speak", "track")) and hud.STYLE["idle"]["eyes"] == 0


def test_brain_units():
    section("大脑：回声识别、决策日志")
    from dog.brain import Brain
    from dog.eyes import ConsoleEyes
    b = Brain(config.load(), force_mock=True, eyes=ConsoleEyes())
    b.recent_said = [(time.monotonic() + 30, "外部温度二十二度天气晴")]
    check("自己刚说过的话被认成回声", b.is_echo("外部温度二十二度"))
    check("识别成数字和同音字也能认出回声", b.is_echo("外部温度22度填气情"))
    check("别人的话不会被当成回声", not b.is_echo("帮我扫描一下客厅"))
    b.recent_said = [(time.monotonic() - 1, "外部温度二十二度天气晴")]
    check("30 秒以前说的话不再算回声", not b.is_echo("外部温度二十二度"))
    b.log_dir = "data/test_logs"                       # 测试写到单独的目录，不弄脏真实的决策日志
    path = config.path(time.strftime("data/test_logs/decisions-%Y-%m-%d.jsonl"))
    before = os.path.getsize(path) if os.path.exists(path) else 0
    b.utt = {"via": "test"}
    b.record("ignore", reason="TEST", text="回归测试")
    check("决策日志能写", os.path.exists(path) and os.path.getsize(path) > before)


# ---------------------------------------------------------------- 记忆
async def test_memory():
    section("记忆：存、取、忘、每日小结、按家分开（不联网）")
    from dog.memory import Memory
    m = Memory(home="unit_test", folder="data/test_memory")
    m.db.executescript("DELETE FROM events; DELETE FROM facts; DELETE FROM daily;")
    m.add_fact("用户叫阿峰")
    m.add_fact("钥匙放在门口鞋柜上")
    m.add_fact("手机放在门口鞋柜上")
    check("字面很像的两件事不会被合并", len(m.facts()) == 3)
    check("一模一样的重复记录只留一条", m.add_fact("用户叫阿峰") == "updated" and len(m.facts()) == 3)
    r = m.recall("我钥匙放哪了")
    check("问钥匙能取出钥匙那条", "钥匙放在门口鞋柜上" in r)
    y = time.time() - 86400
    m.add_event("person_appeared", "有人出现了（1 张脸）", ts=y)
    m.add_event("chat", "用户说：帮我扫描一下客厅｜我回答：开始扫描客厅。", ts=y + 60)
    check("问“刚才/昨天”会带上最近的事件", "扫描一下客厅" in m.recall("刚才发生了什么"))
    await m.summarize(None)
    check("前一天自动写小结（没大模型时写次数统计）", "小结" in m.recall("昨天干了什么"))
    check("忘掉指定的一条", m.forget("钥匙放哪") == "钥匙放在门口鞋柜上" and len(m.facts()) == 2)
    other = Memory(home="unit_test_other", folder="data/test_memory")
    check("不同的家记忆分开", other.facts() == [] and other.path != m.path)
    for x in (m, other):
        path = x.path
        x.close()
        os.remove(path)


async def test_memory_llm():
    section("记忆 + 大模型：记下来、重启以后还记得、换地方会更新")
    from dog.brain import Brain
    from dog.eyes import ConsoleEyes

    def make():
        cfg = config.load()
        cfg["memory"] = dict(cfg.get("memory", {}), home="e2e_test", folder="data/test_memory")
        b = Brain(cfg, eyes=ConsoleEyes())
        b.speaker.play = lambda pcm, sr: asyncio.ensure_future(asyncio.sleep(0))
        return b

    b = make()
    b.memory.db.executescript("DELETE FROM events; DELETE FROM facts; DELETE FROM daily;")
    await b.respond("记一下，我叫阿峰，我的钥匙放在门口鞋柜上", via="text")
    facts = [t for _, t in b.memory.facts()]
    check("一句话里的两件事分开记", len(facts) == 2 and any("阿峰" in t for t in facts) and any("鞋柜" in t for t in facts), str(facts))
    await b.respond("钥匙我换地方了，现在放在餐桌上", via="text")
    facts = [t for _, t in b.memory.facts()]
    check("换地方：旧位置忘掉、新位置记下、名字还在",
          any("餐桌" in t for t in facts) and not any("鞋柜" in t for t in facts) and any("阿峰" in t for t in facts), str(facts))
    b.memory.close()
    b2 = make()                                          # 重启：短期对话记录清空，只剩长期记忆
    await b2.respond("我钥匙放哪了？", via="text")
    check("重启以后还记得钥匙在餐桌", "餐桌" in "".join(b2.last_reply), "".join(b2.last_reply))
    await b2.respond("我的手机放哪了？", via="text")
    reply = "".join(b2.last_reply)
    check("没记过的不编（手机）：说没有记录", any(w in reply for w in ("没有记录", "没记", "没有手机", "不知道")), reply)
    path = b2.memory.path
    b2.memory.close()
    os.remove(path)


# ---------------------------------------------------------------- 主动开口
def test_attention():
    section("主动开口：什么时候开口、频率限制（不联网，假时钟）")
    import datetime
    from dog.attention import Attention
    from dog.tts import speakable

    class S:
        present = False

    t = [datetime.datetime(2026, 10, 9, 9, 0).timestamp()]
    s = S()
    a = Attention(config.load()["attention"], s, clock=lambda: t[0])

    def step(sec, here):
        s.present = here
        for _ in range(int(sec)):
            t[0] += 1
            a.update()

    step(120, False)
    step(5, True)
    g = a.greeting()
    check("今天第一次见到人：要打招呼", g is not None and g[0] == "first")
    a.spoke(g[0])
    step(60, False)
    step(5, True)
    check("离开 1 分钟就回来：不算“回来了”", a.greeting() is None)
    step(400, False)
    step(5, True)
    check("离开 6 分钟回来，但离上次开口不到 10 分钟：先不说", a.greeting() is None)
    step(600, False)
    step(5, True)
    g = a.greeting()
    check("离开 10 分钟回来：要说", g is not None and g[0] == "arrive")
    a.spoke("arrive")
    step(3600, True)
    check("连续坐了 1 小时：提醒活动", (a.periodic() or ("",))[0] == "sit")
    a.quiet_until = t[0] + 1800
    check("让它安静的时候不说", a.periodic() is None)
    a.quiet_until = 0
    t[0] = datetime.datetime(2026, 10, 9, 23, 30).timestamp()
    check("夜里 23:30 不主动说", a.blocked("sit") != "")
    check("本地合成前把 % 和时间转成能念的中文",
          speakable("降水概率59%，现在22:14，外面20°C") == "降水概率百分之59，现在22点14分，外面20度")


async def test_proactive_llm():
    section("主动开口 + 大模型：说的话有用、不寒暄")
    from dog.attention import Attention
    from dog.brain import Brain
    from dog.eyes import ConsoleEyes

    class FS:
        present = True

        def attending(self):
            return False

        def readings(self):
            return []

    cfg = config.load()
    cfg["memory"] = dict(cfg.get("memory", {}), home="e2e_proactive", folder="data/test_memory")
    b = Brain(cfg, eyes=ConsoleEyes())
    b.speaker.play = lambda pcm, sr: asyncio.ensure_future(asyncio.sleep(0))
    b.log_dir = "data/test_logs"
    b.memory.db.executescript("DELETE FROM facts; DELETE FROM events; DELETE FROM daily;")
    b.memory.add_fact("用户每天早上七点出门")
    b.sight = FS()
    b.wake = type("W", (), {"reset": lambda self: None})()
    for kind, why in (("arrive", "用户离开了大约 40 分钟，刚回到你面前（是用户回来了，你一直在原地）"), ("sit", "摄像头已经连续 75 分钟看到用户在你面前，用户一直没离开过。这是到点的久坐提醒，必须说：直接提醒用户起来活动")):
        b.attention = Attention(cfg["attention"], b.sight)
        await b.proactive(kind, why)
        said = "".join(b.last_reply)
        check(f"主动开口（{kind}）：说了话，简短，不寒暄、不自我介绍、不说“汪”",
              said and len(b.last_reply) <= 3 and "汪" not in said and "我是" not in said and "有什么可以帮" not in said, said)
    path = b.memory.path
    b.memory.close()
    os.remove(path)


# ---------------------------------------------------------------- 头部：触摸、隐私开关、传感器、协议
async def test_head_link():
    section("头部 ESP32 事件：协议检查、摸头唤醒、摸一下打断、隐私开关、传感器（不联网，假回答）")
    from dog import protocol
    from dog.brain import Brain
    from dog.eyes import ConsoleEyes
    good = [{"ev": "hello", "fw": "0.1.0", "caps": ["touch"]}, {"ev": "hb", "up": 12},
            {"ev": "touch", "zone": "head", "action": "tap"}, {"ev": "sensor", "name": "light", "value": 120, "unit": "lux"},
            {"ev": "sensor", "name": "presence", "value": True}, {"ev": "privacy", "on": True}, {"ev": "error", "code": "I2C"}]
    check("合法的头部事件都通过", all(protocol.check_event(e) == "" for e in good))
    bad = [{"ev": "touch", "zone": "tail", "action": "tap"}, {"ev": "touch", "zone": "head", "action": "poke"},
           {"ev": "sensor", "name": "laser", "value": 1}, {"ev": "privacy", "on": "yes"}, {"ev": "boom"}, [1, 2]]
    check("不合法的头部事件都被拦下", all(protocol.check_event(e) for e in bad))

    class FakeListener:
        heard_speech = False

        def start(self, wait=None):
            self.wait = wait

    b = Brain(config.load(), force_mock=True, eyes=ConsoleEyes())
    b.log_dir = "data/test_logs"
    b.listener, b.ack, b.chirp = FakeListener(), (np.zeros(10, np.float32), 24000), (np.zeros(10, np.float32), 24000)
    b.wake = type("W", (), {"reset": lambda self: None})()
    plays, stops = [], []
    b.speaker.play = lambda pcm, sr: plays.append(1) or asyncio.ensure_future(asyncio.sleep(0.4))
    b.speaker.stop = lambda: stops.append(1)

    async def feel(ev):                                  # 模拟 ESP32 发来一行，等大脑处理完
        b.eyes.receive_line(json.dumps(ev))
        await asyncio.sleep(0.05)

    await feel({"ev": "touch", "zone": "head", "action": "tap"})
    check("待机时摸头顶：唤醒并开始听", b.state == "listening" and b.via == "touch" and b.eyes.mode_now in ("wake", "listen"))
    await feel({"ev": "touch", "zone": "chin", "action": "tap"})
    check("在听的时候摸下巴：不听了", b.state == "idle")

    plays.clear()
    task = asyncio.create_task(b.respond("帮我扫描一下客厅", via="touch"))   # 假回答固定说两句
    for _ in range(100):
        if b.state == "speaking":
            break
        await asyncio.sleep(0.02)
    await feel({"ev": "touch", "zone": "head", "action": "tap"})
    await task
    check("说话时摸头顶：马上停，后面的不说了", stops and len(plays) == 1, f"停了 {len(stops)} 次，说了 {len(plays)} 句")
    check("被摸头打断后：停下来听你说", b.after_interrupt() and b.state == "listening" and b.via == "touch")

    b.to_idle()
    await feel({"ev": "privacy", "on": True})
    check("隐私开关拨下：不听不看，屏幕显示 PRIVACY", b.privacy and b.eyes.mode_now == "sleep")
    await feel({"ev": "touch", "zone": "head", "action": "tap"})
    check("隐私开关打开时摸头不唤醒", b.state == "idle")
    await feel({"ev": "privacy", "on": False})
    check("隐私开关拨回来：恢复待机", not b.privacy and b.state == "idle" and b.eyes.mode_now == "idle")

    await feel({"ev": "sensor", "name": "light", "value": 120.5, "unit": "lux"})
    await feel({"ev": "sensor", "name": "presence", "value": True})
    r1, r2 = b.board.get("head.light"), b.board.get("head.presence")
    check("头部传感器读数进了感觉板", r1 and r1.value == 120.5 and r2 and r2.value == "有人")
    await feel({"ev": "hb", "up": 5})
    check("收到心跳会记下时间", b.eyes.last_hb > 0)


# ---------------------------------------------------------------- 不联网：红外遥控
IR_NEC = {"proto": "NEC", "bits": 32, "value": "0x20DF10EF"}


def fresh_ir_db(name):
    """测试用的码库文件（每次先删掉，从空的开始）。"""
    from dog.ir import IrCodes
    os.makedirs("data/test_logs", exist_ok=True)
    path = f"data/test_logs/{name}.db"
    if os.path.exists(path):
        os.remove(path)
    return IrCodes(path)


async def test_ir():
    section("红外遥控：协议、码库、学按键、发码、没学过的不发（不联网，脚本回答）")
    from dog import protocol
    from dog.brain import Brain
    from dog.eyes import ConsoleEyes
    good = [IR_NEC, {"proto": "GREE", "state": "0x" + "ab" * 8}, {"raw": [9000, 4500, 560], "khz": 38}]
    check("合法的红外码都通过", all(protocol.check_ir_code(c) == "" for c in good))
    bad = [{"proto": "NEC", "bits": 32, "value": "20DF"}, {"proto": "NEC", "bits": 32, "value": "0xZZ"},
           {"proto": "NEC", "value": "0x20"}, {"raw": [0, 5]}, {"raw": [100, 100], "khz": 5}, {}, "x"]
    check("不合法的红外码都被拦下", all(protocol.check_ir_code(c) for c in bad))
    check("红外事件和指令的检查", protocol.check_event({"ev": "ir", "op": "learned", "code": IR_NEC}) == ""
          and protocol.check_event({"ev": "ir", "op": "boom"}) and protocol.check_ir_cmd({"op": "send", "code": IR_NEC}) == ""
          and protocol.check_ir_cmd({"op": "fire"}) and protocol.check_ir_cmd({"op": "learn", "timeout": 999}))

    db = fresh_ir_db("ir_unit")
    db.save("客厅空调", "关机", IR_NEC)
    hit = db.find("空调", "关掉")
    check("码库：说法不同也能找到（“空调 关掉”找到“客厅空调 关”）", hit and hit[0] == "客厅空调" and hit[1] == "关")
    check("码库：没学过的找不到", db.find("电视", "开") is None)
    check("码库：同一个按键重新学是覆盖", db.save("客厅空调", "关闭", IR_NEC) == "updated" and db.devices() == {"客厅空调": ["关"]})
    db.close()

    cfg = config.load()
    cfg["memory"] = dict(cfg.get("memory", {}), home="ir_flow", folder="data/test_logs")
    path = "data/test_logs/ir_flow.db"
    if os.path.exists(path):
        os.remove(path)
    b = Brain(cfg, force_mock=True, eyes=ConsoleEyes())
    b.log_dir = "data/test_logs"

    class Listener:
        heard_speech = False

        def start(self, wait=None):
            pass

    b.listener, b.ack, b.chirp = Listener(), (np.zeros(10, np.float32), 24000), (np.zeros(10, np.float32), 24000)
    b.wake = type("W", (), {"reset": lambda self: None})()
    b.speaker.play = lambda pcm, sr: asyncio.ensure_future(asyncio.sleep(0.05))
    b.speaker.stop = lambda: None

    async def fake_synth(s):
        return np.zeros(100, np.float32), 24000
    b.voice.synth = fake_synth
    sent = []
    real_send_ir = b.eyes.send_ir
    b.eyes.send_ir = lambda ir: sent.append(ir) or real_send_ir(ir)

    class Script:                                        # 按脚本“回答”，每行一个技能
        name = "脚本"
        lines, seen = [], ""

        async def stream(self, messages):
            Script.seen = messages[-1]["content"]
            for obj in Script.lines:
                yield json.dumps(obj, ensure_ascii=False) + "\n"
    b.llm = Script()

    async def feel(ev):
        b.eyes.receive_line(json.dumps(ev, ensure_ascii=False))
        await asyncio.sleep(0.05)

    async def until_learning():
        for _ in range(100):
            if b.state == "learning":
                return True
            await asyncio.sleep(0.02)
        return False

    # 1. 学会一个按键
    Script.lines = [{"skill": "ir_learn", "device": "电视", "action": "开关"},
                    {"skill": "say", "text": "请把遥控器对准我的前脸，按一下开关键。"}]
    task = asyncio.create_task(b.respond("学一下电视的开关", via="text"))
    learning = await until_learning()
    check("学按键：先说完话，再进入等遥控器（屏幕 SCAN、给 ESP32 发 learn）",
          learning and b.eyes.mode_now == "scan" and sent and sent[-1]["op"] == "learn")
    await feel({"ev": "ir", "op": "learned", "code": IR_NEC})
    await asyncio.wait_for(task, 5)
    check("学按键：收到码以后存进码库并汇报“已学会”", b.ir.devices() == {"电视": ["开关"]}
          and "已学会：电视 开关。" in b.last_reply, f"说了 {b.last_reply}")

    # 2. 没按遥控器：超时
    Script.lines = [{"skill": "ir_learn", "device": "风扇", "action": "开关"}, {"skill": "say", "text": "请按一下风扇的开关键。"}]
    sent.clear()
    task = asyncio.create_task(b.respond("学一下风扇的开关", via="text"))
    await until_learning()
    await feel({"ev": "ir", "op": "timeout"})
    await asyncio.wait_for(task, 5)
    check("学按键超时：汇报没收到信号，不存", "风扇" not in b.ir.devices() and any("没有收到红外信号" in s for s in b.last_reply)
          and sent[-1]["op"] == "cancel")

    # 3. 学的时候摸头顶：不学了，转去听你说
    Script.lines = [{"skill": "ir_learn", "device": "风扇", "action": "开关"}, {"skill": "say", "text": "请按一下。"}]
    task = asyncio.create_task(b.respond("学一下风扇的开关", via="text"))
    await until_learning()
    await feel({"ev": "touch", "zone": "head", "action": "tap"})
    await asyncio.wait_for(task, 5)
    check("学的时候摸头顶：取消学习，转去听你说", "风扇" not in b.ir.devices() and b.after_interrupt() and b.state == "listening")
    b.to_idle()

    # 4. 发码：说法不同（电源）也能找到学过的（开关）
    Script.lines = [{"skill": "ir_send", "device": "电视", "action": "电源"}, {"skill": "say", "text": "已发送：电视 开关。"}]
    sent.clear()
    await b.respond("把电视打开", via="text")
    check("大模型能看到已经学会的按键", "【我能遥控的家电】\n- 电视：开关" in Script.seen)
    check("发码：找到学过的码发给 ESP32", sent and sent[-1] == {"op": "send", "code": IR_NEC} and b.last_reply == ["已发送：电视 开关。"])

    Script.lines = [{"skill": "ir_send", "device": "电视", "action": "开关"}]
    await b.respond("关电视", via="text")
    check("发了码但大模型一句没说：补一句“已发送”", b.last_reply == ["已发送：电视 开关。"], f"说了 {b.last_reply}")

    # 5. 没学过的按键：不发，后面的“已发送”也不说
    Script.lines = [{"skill": "ir_send", "device": "空调", "action": "关"}, {"skill": "say", "text": "已发送：空调 关。"}]
    sent.clear()
    await b.respond("把空调关了", via="text")
    check("没学过的按键：不发，告诉你还没学过", not sent and len(b.last_reply) == 1 and "还没学过" in b.last_reply[0],
          f"说了 {b.last_reply}")

    Script.lines = [{"skill": "ir_forget", "device": "电视", "action": "全部"}, {"skill": "say", "text": "已删除。"}]
    await b.respond("把电视的遥控都忘了", via="text")
    check("忘掉一个家电的全部按键", b.ir.devices() == {})
    b.ir.close()
    b.memory.close()


async def test_ir_llm():
    section("大模型：什么时候发红外、什么时候先学（联网）")
    from dog import llm as llm_mod
    cfg = config.load()
    llm = llm_mod.make(cfg["llm"])
    devices = "- 空调：关、制冷26\n- 电视：开关、音量加"

    async def ask(text):
        msgs = prompt.build(cfg["persona"], [], text, devices=devices, via="text")
        calls = []
        async for line in llm_mod.lines(llm.stream(msgs)):
            c, _ = skills.parse(line)
            if c:
                calls.append(c)
        return calls

    def find(calls, name):
        return [c for c in calls if c["skill"] == name]

    c = await ask("帮我把空调关掉")
    s = find(c, "ir_send")
    check("“把空调关掉” → 发 空调 关", len(s) == 1 and s[0]["device"] == "空调" and s[0]["action"] == "关", str(c)[:200])
    c = await ask("电视声音大一点")
    s = find(c, "ir_send")
    check("“电视声音大一点” → 发 电视 音量加", s and s[0]["device"] == "电视" and s[0]["action"] == "音量加", str(c)[:200])
    c = await ask("把风扇打开")
    check("没学过的风扇 → 不发码，说还没学", not find(c, "ir_send") and not find(c, "ir_learn")
          and any("学" in x.get("text", "") for x in find(c, "say")), str(c)[:200])
    c = await ask("学一下风扇的开关")
    s = find(c, "ir_learn")
    check("“学一下风扇的开关” → ir_learn 风扇", s and s[0]["device"] == "风扇", str(c)[:200])


# ---------------------------------------------------------------- 不联网：热成像
def thermal_brain(**th):
    from dog.brain import Brain
    from dog.eyes import ConsoleEyes
    cfg = config.load()
    cfg["thermal"] = dict(cfg.get("thermal", {}), **th)
    cfg["memory"] = dict(cfg.get("memory", {}), home="thermal_test", folder="data/test_logs")
    b = Brain(cfg, force_mock=True, eyes=ConsoleEyes())
    b.log_dir = "data/test_logs"
    return b


async def test_thermal():
    section("热成像：帧格式、分析、感觉板、屏幕、高温主动提醒（不联网）")
    import pygame
    from dog import protocol, thermal
    scene = thermal.FakeScene(seed=1)
    fr = scene.frame(3.0)
    ev = thermal.encode(fr)
    back = thermal.decode(json.loads(json.dumps(ev)))
    check("一帧热画面编码再解码，误差小于 0.2°C，一行不到 1.2 KB",
          float(np.abs(back - fr).max()) < 0.2 and len(json.dumps(ev)) < 1200 and protocol.check_event(ev) == "")
    bad = [dict(ev, px=ev["px"][:-8]), dict(ev, px="@@@"), dict(ev, min=60, max=20), dict(ev, w=100)]
    check("不合法的热画面被拦下", all(protocol.check_event(e) for e in bad))
    check("热成像指令检查", protocol.check_thermal_cmd({"hz": 1}) == "" and protocol.check_thermal_cmd({"hz": 99})
          and protocol.check_thermal_cmd({"fps": 1}))

    a = thermal.analyze(back)
    cup = [s for s in a["hot"] if 50 <= s["peak"] <= 54]
    person = [s for s in a["hot"] if 33 <= s["peak"] <= 36]
    check("分析：找到热水杯（右前方偏下、比人体热）", cup and "右前方" in cup[0]["where"] and "偏下" in cup[0]["where"]
          and "热水" in cup[0]["guess"], str(a["hot"])[:200])
    check("分析：找到人（大小和温度像人）", person and "像人" in person[0]["guess"])
    check("分析：找到冷窗户（左前方偏上）", a["cold"] and "左前方" in a["cold"]["where"] and a["cold"]["temp"] < 19)
    m = thermal.analyze(back, mirror=True)
    check("装反了可以镜像（mirror）：热水杯变到左边", "左前方" in [s for s in m["hot"] if s["peak"] > 50][0]["where"])
    flat = thermal.analyze(np.full((24, 32), 23.0, np.float32) + np.random.default_rng(0).normal(0, 0.2, (24, 32)))
    check("全是室温：没有热源、没有冷点", not flat["hot"] and flat["cold"] is None and "没有明显热源" in thermal.describe(flat))

    surf = pygame.Surface((hud.W, hud.H))
    r = hud.HudRenderer(hud.Gfx(surf), now=0)
    r.apply({"mode": "thermal"})
    r.tick(0.5)
    r.set_thermal(fr.tolist(), sim=True)
    r.tick(1.0)
    check("屏幕：没有数据和有数据时都能画热成像", r.mode == "thermal")

    b = thermal_brain(alert_seconds=0)
    called = []

    async def fake_proactive(kind, why, then_listen=False):
        called.append((kind, why))
        b.to_idle()
    b.proactive = fake_proactive

    async def feel(frame):
        b.eyes.receive_line(json.dumps(thermal.encode(frame)))
        await asyncio.sleep(0.05)

    await feel(fr)
    rd = b.board.get("thermal")
    check("热画面进了感觉板（模拟数据标成假数据）", rd and "52" in rd.value and rd.confidence == 0 and "表面温度" in rd.note,
          rd.value[:80] if rd else "")
    check("没有高温：不提醒", not called)
    scene.stove = True
    await feel(scene.frame(3.0))
    check("发现 60°C 以上的高温点：屏幕 ALERT，主动提醒", len(called) == 1 and called[0][0] == "hot"
          and "必须说" in called[0][1] and b.eyes.mode_now in ("alert", "idle"), str(called)[:120])
    await feel(scene.frame(4.0))
    check("同一处高温 10 分钟内不重复提醒", len(called) == 1)
    b.hot_alerted = -1e9
    await b.on_head_event({"ev": "privacy", "on": True})
    await feel(scene.frame(5.0))
    check("隐私开关打开时不提醒", len(called) == 1)
    b.memory.close()

    b = thermal_brain(alert_seconds=3)
    b.proactive = fake_proactive
    called.clear()
    await feel(scene.frame(3.0))
    check("高温要持续 alert_seconds 秒才提醒（一闪而过不算）", not called and b.hot_since is not None)
    b.memory.close()


async def test_thermal_llm():
    section("大模型：用热成像回答（联网）")
    from dog import llm as llm_mod
    from dog import senses as senses_mod
    from dog import thermal
    cfg = config.load()
    llm = llm_mod.make(cfg["llm"])
    a = thermal.analyze(thermal.FakeScene(seed=1).frame(3.0))
    board = senses_mod.SenseBoard()
    board.put(senses_mod.Reading("thermal", "热成像", thermal.describe(a), source="模拟器的假房间（热成像模块还没装）",
                                 location="前脸，视野约 55°×35°，只看得到正前方", confidence=0, max_age=5,
                                 note="测的是物体表面温度，不是空气温度；热源是什么是按温度和大小猜的，不一定对"))
    board.put(senses_mod.Reading("room.temp", "室内温度", 22.0, "°C", source="温湿度传感器（还没装，现在是假数据）",
                                 location="屋里（模拟）", confidence=0))

    async def ask(text):
        msgs = prompt.build(cfg["persona"], [], text, sensed=board.text(), via="text")
        calls = []
        async for line in llm_mod.lines(llm.stream(msgs)):
            c, _ = skills.parse(line)
            if c:
                calls.append(c)
        return calls, "".join(c.get("text", "") for c in calls if c["skill"] == "say")

    calls, said = await ask("看看前面哪里最热")
    check("“哪里最热” → 切到热成像，报出 52 度和方向，说明是模拟数据",
          any(c["skill"] == "display" and c["mode"] == "thermal" for c in calls) and "52" in said and "右" in said
          and any(w in said for w in ("模拟", "假数据", "未安装", "尚未安装")), said[:120])
    calls, said = await ask("屋里现在多少度")
    check("问室温：用温湿度的读数，不拿热成像的 52 度回答", "22" in said and "52" not in said, said[:120])


# ---------------------------------------------------------------- 不联网：语音模型
VOICE = None


async def speech(text):
    """用本地合成造一段测试语音（16k）。"""
    global VOICE
    from dog.audio import resample
    from dog.tts import LocalVoice
    VOICE = VOICE or LocalVoice("models/vits-melo-tts-zh_en", speed=1.0)
    pcm, sr = await VOICE.synth(text)
    return resample(pcm, sr, 16000)


async def test_speech():
    section("语音模型：唤醒词、说话检测、识别、合成（都在本机）")
    from dog.hearing import Listener, Recognizer, WakeWord
    cfg = config.load()
    w = cfg["wake"]
    kws = WakeWord(w["keywords"], w["threshold"], w["boost"])
    sil = np.zeros(8000, np.float32)
    from dog.audio import resample
    from dog.tts import EdgeVoice
    out = await EdgeVoice("zh-CN-YunxiNeural").synth(w["keywords"][0])
    name = f"备用唤醒词「{w['keywords'][0]}」能唤醒"
    if out is None:
        skip(name, "连不上 edge-tts；本地合成的声音唤醒模型认不出，只能用 edge-tts 的声音或真人测")
    else:
        audio = np.concatenate([sil, resample(out[0], out[1], 16000), sil])
        check(name, any(kws.feed(audio[i:i + 1600]) for i in range(0, len(audio), 1600)))
    kws2 = WakeWord(w["keywords"], w["threshold"], w["boost"])
    other = await speech("今天天气不错，我们出去走走吧")
    check("普通句子不会误唤醒", not any(kws2.feed(other[i:i + 1600]) for i in range(0, len(other), 1600)))
    li = Listener(5, 15, cfg["listen"]["end_silence_seconds"])
    li.start()
    seg = None
    a = np.concatenate([sil, await speech("帮我扫描一下客厅"), np.zeros(24000, np.float32)])
    a = np.concatenate([a, np.zeros(-len(a) % 1600, np.float32)])
    for x in a.reshape(-1, 1600):
        st, s = li.feed(x)
        if st == "done":
            seg = s
            break
    check("说完自动收声", seg is not None)
    asr = Recognizer(**cfg["asr"])
    text = asr.transcribe(seg) if seg is not None else ""
    check("本地识别出内容", "扫描" in text, text)


# ---------------------------------------------------------------- 联网：完整流程（看着它说话）
class FakeSight:
    present = False

    def __init__(self, att=True, look=True):
        self.att, self.look, self.att_since = att, look, None

    def attending(self):
        return self.att

    def looking_now(self):
        return self.look

    def looked_away_for(self):
        return 0 if self.att else 99

    def touch(self):
        pass

    def snapshot_later(self, r):
        return None

    def readings(self):
        return []


async def run_flow(text, att=True, look=True, via_echo=False, mock=False, touch=False):
    """跑一遍：语音里说 text，返回 (决定, 原因)。"""
    from dog.brain import Brain
    from dog.eyes import ConsoleEyes
    from dog.hearing import Listener, Recognizer, WakeWord
    cfg = config.load()
    b = Brain(cfg, force_mock=mock, eyes=ConsoleEyes())
    decisions = []
    b.record = lambda d, **kw: decisions.append((d, kw.get("reason", "")))
    sil = np.zeros(16000, np.float32)
    audio = np.concatenate([sil, await speech(text), sil])

    class Mic:
        async def frames(self):
            if touch:                                   # 先摸一下头顶再说话
                b.eyes.receive_line('{"ev":"touch","zone":"head","action":"tap"}')
                await asyncio.sleep(0.05)
            for i in range(0, len(audio), 1600):
                yield audio[i:i + 1600]
                await asyncio.sleep(0.003)
            for _ in range(250):                      # 再给 25 秒等大模型回答
                if decisions:
                    return
                yield np.zeros(1600, np.float32)
                await asyncio.sleep(0.1)

    async def setup(voice_input=True):
        b.sight = FakeSight(att, look)
        w, li = cfg["wake"], cfg["listen"]
        b.wake = WakeWord(w["keywords"], w["threshold"], w["boost"])
        b.listener = Listener(li["wait_seconds"], li["max_speech_seconds"], li["end_silence_seconds"])
        b.asr = Recognizer(**cfg["asr"])
        b.mic = Mic()
        b.ack = b.chirp = (np.zeros(10, np.float32), 24000)
        if via_echo:
            import re
            b.recent_said = [(time.monotonic() + 30, re.sub(r"[\W_]+", "", text))]

    b.setup = setup
    b.speaker.play = lambda pcm, sr: asyncio.ensure_future(asyncio.sleep(0))   # 不出声
    errors = []

    class Catch(logging.Handler):                       # 过程中出现任何报错都记下来
        def emit(self, rec):
            if rec.levelno >= logging.ERROR:
                errors.append(rec.getMessage())

    h = Catch()
    logging.getLogger().addHandler(h)
    try:
        await asyncio.wait_for(b.run_voice(), 60)
    finally:
        logging.getLogger().removeHandler(h)
    got = decisions[0] if decisions else ("none", "")
    return (got[0], got[1], errors)


async def test_flow(online: bool):
    section("完整流程：看着它说话（假的“看着我”信号 + 合成语音）")
    cases = [
        ("没看着它说话 → 不进入聆听", dict(text="现在外面多少度", att=False, look=False, mock=True), ("none", "")),
        ("说话时扭头了 → 不理", dict(text="现在外面多少度", look=False, mock=True), ("ignore", "NOT FACING")),
        ("听到自己刚说的话 → 当回声不理", dict(text="外部温度二十二度天气晴", via_echo=True, mock=True), ("ignore", "ECHO")),
        ("看着它说话 → 回答两句，过程不报错（假回答）", dict(text="帮我扫描一下客厅", mock=True), ("answer", "")),
        ("没看着它，但先摸了头顶再说 → 回答（假回答）", dict(text="帮我扫描一下客厅", att=False, look=False, mock=True, touch=True), ("answer", "")),
    ]
    if online:
        cases += [
            ("看着它问问题 → 回答", dict(text="帮我扫描一下客厅"), ("answer", "")),
            ("看着它但在打电话 → 大模型判断不理", dict(text="喂，张总，那个合同我下午发给你"), ("ignore", "NOT FOR ME")),
        ]
    for name, kw, want in cases:
        try:
            d, reason, errors = await run_flow(**kw)
            check(name, (d, reason) == want and not errors,
                  (f"结果 {d} {reason}".strip() + (f"；过程报错：{errors[0]}" if errors else "")))
        except Exception as e:
            check(name, False, f"{type(e).__name__}: {e}")
            traceback.print_exc()


async def test_llm_judgement():
    section("大模型：判断是不是对它说的（直接问，不走语音）")
    from dog import llm as llm_mod
    cfg = config.load()
    llm = llm_mod.make(cfg["llm"])
    cases = [("现在外面多少度？", False), ("你帮我扫描一下房间", False), ("喂，张总，那个合同我下午发你", True),
             ("第三条，乙方应当在十五日内付款", True)]
    for text, want_ignore in cases:
        msgs = prompt.build(cfg["persona"], [], text, sensed="- 看到的人：1 张人脸，在我正前方，离我不远，正对着我", via="look")
        out = ""
        async for d in llm.stream(msgs):
            out += d
        ign = '"ignore"' in out
        check(f"「{text}」→ {'不理' if want_ignore else '回答'}", ign == want_ignore)
        if not want_ignore:
            check(f"「{text}」的回答是汇报式语气（没有“汪”、没有“结论：”标签）",
                  "汪" not in out and "结论：" not in out and "结论:" not in out, out.replace("\n", " ")[:80])


async def main():
    quick = "--quick" in sys.argv
    online = not quick and bool(config.load()["llm"].get("api_key"))
    t0 = time.time()
    for fn in (test_basics, test_hud, test_brain_units, test_attention):
        try:
            fn()
        except Exception as e:
            check(fn.__name__, False, f"{type(e).__name__}: {e}")
            traceback.print_exc()
    try:
        await test_head_link()
    except Exception as e:
        check("test_head_link", False, f"{type(e).__name__}: {e}")
        traceback.print_exc()
    try:
        await test_memory()
    except Exception as e:
        check("test_memory", False, f"{type(e).__name__}: {e}")
        traceback.print_exc()
    try:
        await test_ir()
    except Exception as e:
        check("test_ir", False, f"{type(e).__name__}: {e}")
        traceback.print_exc()
    try:
        await test_thermal()
    except Exception as e:
        check("test_thermal", False, f"{type(e).__name__}: {e}")
        traceback.print_exc()
    try:
        await test_speech()
    except Exception as e:
        check("test_speech", False, f"{type(e).__name__}: {e}")
        traceback.print_exc()
    await test_flow(online)
    if online:
        await test_llm_judgement()
        await test_memory_llm()
        await test_proactive_llm()
        await test_ir_llm()
        await test_thermal_llm()
    else:
        print("\n（--quick 或没有密钥：跳过要联网的大模型测试）")
    bad = [r for r in RESULTS if not r[1]]
    print(f"\n共 {len(RESULTS)} 项，通过 {len(RESULTS) - len(bad)} 项，失败 {len(bad)} 项，跳过 {len(SKIPPED)} 项，"
          f"用时 {time.time() - t0:.0f} 秒")
    for name, _, detail in bad:
        print(f"  失败：{name} {detail}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    asyncio.run(main())
