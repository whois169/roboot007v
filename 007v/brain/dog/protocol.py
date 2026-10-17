"""大脑 ↔ 头部两块板的通信协议：每块板一根 USB 串口，两个方向都是“一行一条 JSON”（UTF-8，结尾 \\n）。
机械狗007v 头里有两块板：屏幕板（微雪 4.3B，收屏幕指令、发屏幕触摸）和传感器板（旧 ESP32-S3，红外、热成像、传感器、隐私开关）。
两块板的事件格式完全一样，大脑不管是从哪根线来的；只接一根线时，所有指令都走这一根（和以前一样）。

大脑 → ESP32：屏幕指令（格式和检查在 dog/hud.py：mode / gaze / level / text / hold / status / log）
              红外指令（带 "ir" 字段，格式见 check_ir_cmd()）：
  {"ir":{"op":"learn","timeout":15}}                         开始学：等遥控器对着红外接收头按一下，最多等 timeout 秒
  {"ir":{"op":"send","code":{"proto":"NEC","bits":32,"value":"0x20DF10EF"}}}   发一个码
  {"ir":{"op":"cancel"}}                                     不学了
              热成像指令：
  {"thermal":{"hz":1}}                                       每秒发几帧热画面过来（0 = 不发，最多 8）
ESP32 → 大脑：事件，都带 "ev" 字段，格式见 EVENTS，进来先过 check_event()：

  {"ev":"hello","fw":"0.1.0","caps":["light","ir"]}         开机或重连时发一次：固件版本、有哪些功能
  {"ev":"hb","up":1234}                                       心跳，每秒一次；up = 开机多少秒。3 秒收不到就算掉线
  {"ev":"touch","zone":"screen","action":"tap"}              点屏幕（屏幕板自带的触摸屏，2026-10-17 起不再做头顶、下巴触摸）；action = tap 点一下 / hold 按住 1 秒以上 / release 松开
  {"ev":"sensor","name":"light","value":120.5,"unit":"lux"}   传感器读数，name 见 SENSORS
  {"ev":"privacy","on":true}                                  隐私开关：on = true 表示拨到“隐私”（相机和麦克风已经硬件断电）
  {"ev":"error","code":"I2C","text":"BH1750 no ack"}          ESP32 自己的错误
  {"ev":"ir","op":"learned","code":{...}}                     红外学到了一个码（code 格式见 check_ir_code）
  {"ev":"ir","op":"timeout"}                                  学习时一直没收到信号
  {"ev":"ir","op":"sent"}                                     码发完了
  {"ev":"ir","op":"fail","text":"..."}                        学习或发送出错
  {"ev":"thermal","w":32,"h":24,"min":21.3,"max":55.2,"px":"<base64>"}   一帧热画面（格式见 dog/thermal.py）
完整说明见 robodog/docs/头部通信协议.md。
"""
from . import thermal


ZONES = ("screen",)
ACTIONS = ("tap", "hold", "release")
# 传感器：名字 -> (给人看的名字, 单位, 装在哪)
SENSORS = {
    "light":    ("环境光照", "lux", "头顶光照窗（BH1750）"),
    "distance": ("前方最近物体距离", "mm", "下巴测距窗（VL53L5CX）"),
    "presence": ("毫米波检测到有人", "", "屏幕上方（LD2410，人背对着、不动也能测到）"),
    "presence_distance": ("毫米波测到的人的距离", "cm", "屏幕上方（LD2410）"),
    "head_temp": ("头内温度", "°C", "头壳里（ESP32 旁边，测的是头里不是屋里）"),
    "room_temp": ("室内温度", "°C", "头壳后下方通风口（SHT40）"),
    "room_humidity": ("室内湿度", "%", "头壳后下方通风口（SHT40）"),
}


def check_event(ev) -> str:
    """合法返回空字符串，否则返回原因。"""
    if not isinstance(ev, dict):
        return "不是 JSON 对象"
    kind = ev.get("ev")
    if kind == "hello":
        if not isinstance(ev.get("fw", ""), str) or not isinstance(ev.get("caps", []), list):
            return "hello 的 fw 要是文字、caps 要是列表"
    elif kind == "hb":
        if not isinstance(ev.get("up", 0), (int, float)):
            return "hb 的 up 要是数字"
    elif kind == "touch":
        if ev.get("zone") not in ZONES:
            return f"touch 的 zone 只能是 {ZONES}"
        if ev.get("action") not in ACTIONS:
            return f"touch 的 action 只能是 {ACTIONS}"
    elif kind == "sensor":
        if ev.get("name") not in SENSORS:
            return f"不认识的传感器：{ev.get('name')!r}"
        if not isinstance(ev.get("value"), (bool, int, float, str)):
            return "sensor 的 value 要是数字、true/false 或文字"
    elif kind == "privacy":
        if not isinstance(ev.get("on"), bool):
            return "privacy 的 on 要是 true / false"
    elif kind == "error":
        if not isinstance(ev.get("code", ""), str):
            return "error 的 code 要是文字"
    elif kind == "thermal":
        return thermal.check(ev)
    elif kind == "ir":
        if ev.get("op") not in IR_EVENTS:
            return f"ir 事件的 op 只能是 {IR_EVENTS}"
        if ev["op"] == "learned":
            return check_ir_code(ev.get("code"))
    else:
        return f"不认识的事件：{kind!r}"
    return ""


# ---------------------------------------------------------------- 红外
IR_EVENTS = ("learned", "timeout", "sent", "fail")
IR_OPS = ("learn", "send", "cancel")


def check_ir_code(code) -> str:
    """红外码有三种写法（ESP32 用 IRremoteESP8266 库，认得出协议就用前两种，认不出就用原始波形）：
      {"proto":"NEC","bits":32,"value":"0x20DF10EF"}        电视、风扇这类：协议名 + 位数 + 码值（16 进制文字）
      {"proto":"GREE","state":"0x..."}                       空调这类：一整串状态（温度、模式都在里面，所以“开 26 度”和“关”是不同的码）
      {"raw":[9000,4500,560,...],"khz":38}                  原始波形：亮、灭交替的时长（微秒），载波频率 kHz
    """
    if not isinstance(code, dict):
        return "红外码要是 JSON 对象"
    if "raw" in code:
        raw, khz = code.get("raw"), code.get("khz", 38)
        if not (isinstance(raw, list) and 2 <= len(raw) <= 1024
                and all(isinstance(v, int) and not isinstance(v, bool) and 1 <= v <= 65535 for v in raw)):
            return "raw 要是 2–1024 个 1–65535 的整数（微秒）"
        if not (isinstance(khz, int) and 30 <= khz <= 60):
            return "khz 要是 30–60 的整数"
        return ""
    proto = code.get("proto")
    if not (isinstance(proto, str) and 1 <= len(proto) <= 24):
        return "红外码要有 proto（协议名）或 raw（原始波形）"
    hexes = [code.get(k) for k in ("value", "state") if k in code]
    if len(hexes) != 1 or not _is_hex(hexes[0]):
        return "红外码要有 value 或 state 之一，写成 0x 开头的 16 进制文字"
    if "value" in code and not (isinstance(code.get("bits"), int) and 1 <= code["bits"] <= 64):
        return "value 码要写 bits（1–64）"
    return ""


def _is_hex(s) -> bool:
    return (isinstance(s, str) and s[:2].lower() == "0x" and 3 <= len(s) <= 2 + 2 * 64
            and all(c in "0123456789abcdefABCDEF" for c in s[2:]))


def check_thermal_cmd(th) -> str:
    """大脑发给 ESP32 的热成像指令（{"thermal": ...} 里面那一层）。"""
    if not isinstance(th, dict) or set(th) != {"hz"}:
        return "thermal 指令只有 hz 一个字段"
    if isinstance(th["hz"], bool) or not isinstance(th["hz"], (int, float)) or not 0 <= th["hz"] <= 8:
        return "thermal 的 hz 要是 0–8"
    return ""


def check_ir_cmd(ir) -> str:
    """大脑发给 ESP32 的红外指令（{"ir": ...} 里面那一层）。"""
    if not isinstance(ir, dict) or ir.get("op") not in IR_OPS:
        return f"ir 指令的 op 只能是 {IR_OPS}"
    if ir["op"] == "learn" and not (isinstance(ir.get("timeout", 15), (int, float)) and 1 <= ir.get("timeout", 15) <= 60):
        return "learn 的 timeout 要是 1–60 秒"
    if ir["op"] == "send":
        return check_ir_code(ir.get("code"))
    return ""
