"""大脑 ↔ 头部 ESP32 的通信协议：一根 USB 串口，两个方向都是“一行一条 JSON”（UTF-8，结尾 \\n）。

大脑 → ESP32：屏幕指令（格式和检查在 dog/hud.py：mode / gaze / level / text / hold / status / log）
ESP32 → 大脑：事件，都带 "ev" 字段，格式见 EVENTS，进来先过 check_event()：

  {"ev":"hello","fw":"0.1.0","caps":["touch","light"]}       开机或重连时发一次：固件版本、有哪些功能
  {"ev":"hb","up":1234}                                       心跳，每秒一次；up = 开机多少秒。3 秒收不到就算掉线
  {"ev":"touch","zone":"head","action":"tap"}                触摸：zone = head 头顶 / chin 下巴；action = tap 点一下 / hold 按住 1 秒以上 / release 松开
  {"ev":"sensor","name":"light","value":120.5,"unit":"lux"}   传感器读数，name 见 SENSORS
  {"ev":"privacy","on":true}                                  隐私开关：on = true 表示拨到“隐私”（相机和麦克风已经硬件断电）
  {"ev":"error","code":"I2C","text":"BH1750 no ack"}          ESP32 自己的错误
完整说明见 docs/头部通信协议.md。
"""

ZONES = ("head", "chin")
ACTIONS = ("tap", "hold", "release")
# 传感器：名字 -> (给人看的名字, 单位, 装在哪)
SENSORS = {
    "light":    ("环境光照", "lux", "头顶光照窗（BH1750）"),
    "distance": ("前方最近物体距离", "mm", "前脸测距窗（VL53L5CX）"),
    "presence": ("毫米波检测到有人", "", "屏幕上方（LD2410，人背对着、不动也能测到）"),
    "presence_distance": ("毫米波测到的人的距离", "cm", "屏幕上方（LD2410）"),
    "head_temp": ("头内温度", "°C", "头壳里（ESP32 旁边，测的是头里不是屋里）"),
    "ir_code": ("学到的红外码", "", "前脸红外接收头"),
    "thermal_max": ("热成像最高温度", "°C", "前脸热成像（MLX90640）"),
    "thermal_min": ("热成像最低温度", "°C", "前脸热成像（MLX90640）"),
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
    else:
        return f"不认识的事件：{kind!r}"
    return ""
