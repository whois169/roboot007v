"""大脑（树莓派）↔ 底盘 ESP32-S3 的通信协议：一根 USB 串口，两个方向都是“一行一条 JSON”（UTF-8，结尾 \\n），115200。

大脑 → 底盘（都可以带 "id"，带了的会回 ack；move / pose 做完还会回 done）：
  {"drive":[0.4,0,0]}                         按速度走：[前后 m/s, 左右 m/s（左为正）, 转 rad/s（左转为正）]。要一直发（≥ 每 0.5 秒一次），断了就自己慢慢停
  {"move":{"dx":1.0,"dy":0,"dth":0,"v":0.4},"id":7}   走一段：以现在的位置为准，往前 dx 米、往左 dy 米、左转 dth 度；v 最高速度（可省，默认 0.4）
  {"pose":"stand","id":8}                    站起来（折叠趴着 → 展开）；"fold" = 趴下折叠。只能停稳了做
  {"stop":true}                               正常刹车停下（取消正在走的 move / drive）
  {"estop":true}                              软急停：4 m/s² 刹停，之后不动，要 reset 才能再动（和按红色急停按钮一样）
  {"reset":true}                              清除急停 / 故障（急停按钮还按着时清不掉）
  {"limit":{"v":0.3}}                         限速（前面有门槛、人多的地方）；{"limit":{"v":null}} 取消
  {"hb":1}                                    心跳：大脑每 0.5 秒发一次；底盘 2 秒收不到任何一行就当断线，自己停下
  {"calib":"step","id":9}                     台阶测距标定：站着、停稳、在平地上发一次，底盘记住现在的读数当"平地"
  {"light":{"front":60}}                      前面白灯亮度 0–100（0 = 关）；{"light":{"dim":30}} 所有灯的总亮度（默认 100）。两个可以一起写

底盘 → 大脑（都带 "ev"）：
  {"ev":"hello","fw":"0.1.0","caps":[...]}                  开机或重连时发一次
  {"ev":"chassis","st":"standing","v":[0.40,0,0],"odo":[1.23,0.01,2.0],"hip":[55,55,55,55],
   "bat":87,"vbat":24.1,"lim":null,"load":6.5,"busy":7,
   "step":[812,806,795,801],"cliff":null,"chg":false,"light":{"front":0,"dim":100}}
                                                              每 0.1 秒一次（同时当心跳）。v 实测速度；odo 里程 [x m, y m, 朝向 度]；
                                                              load 估出来的背上载重 kg（还没估出来是 null）；busy 正在做的 move / pose 的 id；
                                                              step 台阶测距（前左 前右 后左 后右，mm；-1 = 测不到地面，null = 测距板没回应）；
                                                              cliff 因为地面没了锁住的方向 "front" / "rear" / null；chg 正在充电
  {"ev":"ack","id":7,"ok":true}                               收到带 id 的指令；ok=false 时带 why（FOLDED / MOVING / ESTOP / FAULT / BAD）
  {"ev":"done","id":7,"ok":true}                              move / pose 做完；ok=false 时带 why（STOPPED 被取消 / TIMEOUT / ESTOP / FAULT / CLIFF 地面没了）
  {"ev":"warn","code":"LINK_LOST","text":"..."}               提醒：LINK_LOST 断线自己停了 / FOLDED 趴着不能走 / BAT_LOW 电量低 / LIMIT 被限速
  {"ev":"warn","code":"CLIFF","where":"front","text":"..."}   地面没了：已经刹车，那个方向锁住（只能往反方向走），读数回到平地 0.3 秒后自动解锁
  {"ev":"warn","code":"STEP","where":"front","h":15,"text":"..."}   前面有坎（h 估算高度 mm），自动限速 0.3
  {"ev":"warn","code":"STEP_SENSOR","where":"rear_left","text":"..."}   台阶测距板没回应，限速 0.2
  {"ev":"fault","code":"HIP_STALL","text":"..."}              故障，停下不动，要 reset：ESTOP_BTN 急停按钮 / HIP_STALL 髋卡住 / WHEEL_LOST 轮子电机没回应
"""

STATES = ("folded", "unfolding", "standing", "folding", "estop", "fault")
POSES = ("stand", "fold")
WHY = ("FOLDED", "MOVING", "ESTOP", "FAULT", "BAD", "STOPPED", "TIMEOUT", "BUSY", "CLIFF", "NODATA")
WARNS = ("LINK_LOST", "FOLDED", "BAT_LOW", "LIMIT", "CLIFF", "STEP", "STEP_SENSOR")
WHERE = ("front", "rear", "front_left", "front_right", "rear_left", "rear_right")
FAULTS = ("ESTOP_BTN", "ESTOP_CMD", "HIP_STALL", "WHEEL_LOST")
LIMITS = dict(dx=5.0, dy=2.0, dth=360.0, v=1.1)          # move 一次最多走多远、转多少


def _num(v, lo, hi):
    return not isinstance(v, bool) and isinstance(v, (int, float)) and lo <= v <= hi


def check_cmd(c) -> str:
    """大脑 → 底盘的一条指令，合法返回空字符串，否则返回原因。"""
    if not isinstance(c, dict):
        return "不是 JSON 对象"
    keys = set(c) - {"id"}
    if "id" in c and not (isinstance(c["id"], int) and not isinstance(c["id"], bool) and 0 <= c["id"] < 2 ** 31):
        return "id 要是非负整数"
    if len(keys) != 1:
        return "一条只能有一个指令"
    k = keys.pop()
    v = c[k]
    if k == "drive":
        if not (isinstance(v, list) and len(v) == 3 and _num(v[0], -1.1, 1.1) and _num(v[1], -0.5, 0.5) and _num(v[2], -1.6, 1.6)):
            return "drive 要写 [前后 -1.1–1.1, 左右 -0.5–0.5, 转 -1.6–1.6]"
    elif k == "move":
        if not isinstance(v, dict) or set(v) - {"dx", "dy", "dth", "v"}:
            return "move 只有 dx dy dth v"
        for a in ("dx", "dy", "dth", "v"):
            if a in v and not _num(v[a], -LIMITS[a] if a != "v" else 0.05, LIMITS[a]):
                return f"move.{a} 超出范围"
    elif k == "pose":
        if v not in POSES:
            return f"pose 只能是 {POSES}"
    elif k in ("stop", "estop", "reset", "hb"):
        if v not in (True, 1):
            return f"{k} 写 true"
    elif k == "limit":
        if not (isinstance(v, dict) and set(v) == {"v"} and (v["v"] is None or _num(v["v"], 0.05, 1.1))):
            return "limit 写 {\"v\": 0.05–1.1 或 null}"
    elif k == "calib":
        if v != "step":
            return "calib 只能是 \"step\""
    elif k == "light":
        if not (isinstance(v, dict) and v and not set(v) - {"front", "dim"} and all(_num(x, 0, 100) for x in v.values())):
            return "light 写 {\"front\": 0–100, \"dim\": 0–100}（写一个或两个）"
    else:
        return f"不认识的指令：{k}"
    return ""


def check_event(e) -> str:
    """底盘 → 大脑的一条事件，合法返回空字符串，否则返回原因。"""
    if not isinstance(e, dict):
        return "不是 JSON 对象"
    k = e.get("ev")
    if k == "hello":
        return "" if isinstance(e.get("fw", ""), str) else "hello 的 fw 要是文字"
    if k == "chassis":
        if e.get("st") not in STATES:
            return f"st 只能是 {STATES}"
        for a, n in (("v", 3), ("odo", 3), ("hip", 4)):
            if not (isinstance(e.get(a), list) and len(e[a]) == n and all(_num(x, -1e6, 1e6) for x in e[a])):
                return f"{a} 要是 {n} 个数"
        for a in ("bat", "vbat", "lim", "load", "busy"):
            if e.get(a) is not None and not _num(e[a], -1e6, 1e9):
                return f"{a} 要是数字或 null"
        if "step" in e and not (isinstance(e["step"], list) and len(e["step"]) == 4 and all(x is None or _num(x, -1, 1e5) for x in e["step"])):
            return "step 要是 4 个数（或 null）"
        if e.get("cliff") not in (None, "front", "rear"):
            return "cliff 只能是 front / rear / null"
        return ""
    if k in ("ack", "done"):
        if not isinstance(e.get("id"), int) or not isinstance(e.get("ok"), bool):
            return f"{k} 要有 id 和 ok"
        if not e["ok"] and e.get("why") not in WHY:
            return f"why 只能是 {WHY}"
        return ""
    if k == "warn":
        if e.get("code") not in WARNS:
            return f"warn 的 code 只能是 {WARNS}"
        if e["code"] in ("CLIFF", "STEP", "STEP_SENSOR") and e.get("where") not in WHERE:
            return f"{e['code']} 要写 where（{WHERE}）"
        return ""
    if k == "fault":
        return "" if e.get("code") in FAULTS else f"fault 的 code 只能是 {FAULTS}"
    return f"不认识的事件：{k!r}"
