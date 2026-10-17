"""机械狗007v 大脑入口。

python main.py            语音模式：叫唤醒词跟它说话
python main.py --text     文字模式：打字跟它说话（不用麦克风，测大模型和语音合成）
python main.py --mock     不调用大模型，用假回答（没有密钥时会自动进入）
python main.py --devices  列出麦克风和扬声器
python main.py --eyes     只开眼睛窗口，自动轮流演示所有表情（不加载语音和大模型）
python main.py --see      摄像头调试：画面 + 检测框 + 眼睛跟着人看
python main.py --memory   看它记住了什么（事实、最近的事件、每日小结）
python main.py --text --view   打字跟它说话，同时开虚拟底盘的仿真窗口，看它按你说的走
"""
import argparse
import asyncio
import logging
import os
import sys

from dog import config


def main():
    ap = argparse.ArgumentParser(description="机械狗007v 大脑")
    ap.add_argument("--text", action="store_true", help="文字模式")
    ap.add_argument("--mock", action="store_true", help="用假回答，不调用大模型")
    ap.add_argument("--devices", action="store_true", help="列出声音设备")
    ap.add_argument("--eyes", action="store_true", help="眼睛演示")
    ap.add_argument("--see", action="store_true", help="摄像头调试窗口")
    ap.add_argument("--memory", action="store_true", help="查看长期记忆")
    ap.add_argument("--debug", action="store_true", help="打印更多日志")
    ap.add_argument("--view", action="store_true", help="虚拟底盘开仿真窗口，看车怎么动")
    args = ap.parse_args()
    os.chdir(config.BRAIN_DIR)           # 从哪里运行都行；有的库要用相对路径（见 config.rel）

    for s in (sys.stdout, sys.stderr):   # Windows 控制台也按 UTF-8 输出中文
        if hasattr(s, "reconfigure"):
            s.reconfigure(encoding="utf-8", errors="replace")
    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO,
                        format="%(asctime)s.%(msecs)03d [%(name)s] %(message)s", datefmt="%H:%M:%S")
    for noisy in ("httpx", "httpcore", "openai", "asyncio"):
        logging.getLogger(noisy).setLevel(logging.WARNING)

    if args.devices:
        from dog.audio import list_devices
        list_devices()
        return

    cfg = config.load()
    if args.memory:
        show_memory(cfg)
        return
    if args.eyes or args.see:
        try:
            asyncio.run(see_demo(cfg) if args.see else eyes_demo(cfg))
        except KeyboardInterrupt:
            pass
        return

    if args.view:
        cfg.setdefault("chassis", {}).setdefault("virtual", {})["view"] = True
    from dog.brain import Brain
    brain = Brain(cfg, force_mock=args.mock)
    brain.eyes.on_quit = bye             # 关掉眼睛窗口 = 退出程序
    try:
        asyncio.run(brain.run_text() if args.text else brain.run_voice())
    except KeyboardInterrupt:
        print("\n再见！")


def bye():
    # 文字模式下有个线程卡在等键盘输入，正常退出会一直等它，所以直接结束进程
    print("\n再见！", flush=True)
    os._exit(0)


def show_memory(cfg):
    """打印这个家的记忆：事实、最近 30 条事件、每日小结。"""
    import datetime
    from dog.memory import Memory
    m = Memory(**cfg.get("memory", {}))
    print(f"记忆文件：{m.path}")
    print(f"共 {m.stats()}\n")
    print("【用户告诉我的事实】")
    for ts, t in m.facts():
        print(f"  {datetime.datetime.fromtimestamp(ts):%m-%d %H:%M}  {t}")
    print("\n【最近 30 条事件】")
    for ts, k, t in reversed(m.events(limit=30)):
        print(f"  {datetime.datetime.fromtimestamp(ts):%m-%d %H:%M}  [{k}] {t}")
    print("\n【每日小结】")
    for day, summary in m.db.execute("SELECT day, summary FROM daily ORDER BY day DESC LIMIT 14"):
        print(f"  {day}  {summary}")


async def eyes_demo(cfg):
    """HUD 演示：12 个模式轮流来一遍，聆听/汇报时模拟音量，锁定目标时模拟人脸移动。"""
    import math
    from dog import eyes as eyes_mod, hud
    eyes = eyes_mod.make(cfg.get("eyes", {}), cfg["persona"]["name"])
    eyes.on_quit = bye
    await eyes.start()
    eyes.status(temp=22.1, sim=True, bat=87, spd=0.0, load=6.5)
    extra = {"alert": {"text": "DOOR OPEN"}, "error": {"text": "E-NET"}, "speak": {"face": "happy"}}
    print("HUD 演示中：每个模式 3 秒，循环播放。Ctrl+C 或关掉窗口退出")
    while True:
        for m in hud.MODES:
            eyes.status(bat=8 if m == "low_bat" else 87)
            eyes.mode(m, **extra.get(m, {}))
            eyes.log(f"DEMO MODE {hud.LABEL[m]}")          # 演示里推的是演示说明，不是真读数
            t0 = asyncio.get_running_loop().time()
            while (u := asyncio.get_running_loop().time() - t0) < 3:
                if m == "listen":
                    eyes.level(0.35 + 0.45 * abs(math.sin(u * 3.1)))
                elif m == "speak":
                    eyes.level(0.2 + 0.7 * abs(math.sin(u * 5.3) * math.sin(u * 1.7)))
                elif m == "track":
                    eyes.gaze(0.6 * math.sin(u * 1.4), 0.4 * math.sin(u * 2.1))
                await asyncio.sleep(0.05)
            eyes.gaze(0, 0)


async def see_demo(cfg):
    """摄像头调试：显示画面和检测框，眼睛跟着人看，打印事件和“我看到了什么”。"""
    from dog import eyes as eyes_mod, llm as llm_mod
    from dog.vision import Sight, preview
    eyes = eyes_mod.make(cfg.get("eyes", {}), cfg["persona"]["name"])
    eyes.on_quit = bye
    await eyes.start()
    vc = dict(cfg.get("vision", {}), idle_look_minutes=1)     # 调试时 1 分钟就算“空闲太久”
    sight = Sight(vc, eyes, llm_mod.make_vision(vc.get("model", {})))
    if not await sight.start():
        return
    print("摄像头调试中：在摄像头前出现、离开、左右走动，看日志和眼睛。按 s 截图看图，按 q 关掉画面，Ctrl+C 退出")
    await preview(sight)
    while True:                       # 关掉画面以后继续看日志
        await asyncio.sleep(1)


if __name__ == "__main__":
    main()
