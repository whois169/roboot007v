"""技能菜单：大模型只能输出这些技能的 JSON，执行前逐项检查参数。

每个参数的规则：("str", 最短, 最长) / ("enum", [可选值]) / ("num", 最小, 最大)
"""
import json

from .eyes import DIRECTIONS

# 大模型能主动切的屏幕模式（唤醒、聆听、思考、汇报、锁定目标这些由程序自动切）
DISPLAY_MODES = ("scan", "thermal", "alert", "idle")

SKILLS = {
    "say":      {"desc": "说一句话。每次只放一句短话（不超过 40 字），想说多句就分成多行",
                 "args": {"text": ("str", 1, 80)}},
    "display":  {"desc": "切换屏幕模式：scan 扫描、thermal 热成像（屏幕显示热画面）、alert 警告、idle 待机",
                 "args": {"mode": ("enum", list(DISPLAY_MODES))}},
    "look":     {"desc": "看向某个方向",
                 "args": {"direction": ("enum", list(DIRECTIONS))}},
    "remember": {"desc": "记住用户告诉你的一件事（名字、习惯、喜好、约定、东西放在哪等）。一次只记一件事，一句话里有几件就分几行",
                 "args": {"fact": ("str", 2, 120)}},
    "forget":   {"desc": "忘掉以前记下的一件事（用户说“忘了吧”“那个不用记了”时）",
                 "args": {"fact": ("str", 2, 120)}},
    "quiet":    {"desc": "安静一会儿：这段时间不主动开口（被叫到还是会回答）",
                 "args": {"minutes": ("num", 1, 120)}},
    "ir_send":  {"desc": "用红外遥控家电：发出已经学会的那个按键（只能用【我能遥控的家电】里有的）",
                 "args": {"device": ("str", 1, 12), "action": ("str", 1, 12)}},
    "ir_learn": {"desc": "学一个遥控器按键：说完话以后等用户拿遥控器对着你的前脸按一下，最多等 15 秒，学到或没学到程序会自己汇报",
                 "args": {"device": ("str", 1, 12), "action": ("str", 1, 12)}},
    "ir_forget": {"desc": "忘掉学过的遥控按键（action 写“全部”就忘掉这个家电的所有按键）",
                  "args": {"device": ("str", 1, 12), "action": ("str", 1, 12)}},
    "ignore":   {"desc": "不回答：这句话不是对你说的（只在【触发方式】是“看着我说话”时用，整次只输出这一行）",
                 "args": {}},
}

EXAMPLES = {"say": {"text": "收到。外部温度 22 度，晴。"}, "display": {"mode": "scan"},
            "look": {"direction": "left"}, "remember": {"fact": "用户叫小明，钥匙一般放在门口鞋柜上"},
            "forget": {"fact": "钥匙放在鞋柜上"}, "quiet": {"minutes": 30},
            "ir_send": {"device": "空调", "action": "关"}, "ir_learn": {"device": "电视", "action": "开关"},
            "ir_forget": {"device": "电视", "action": "全部"}, "ignore": {}}


def menu_text() -> str:
    """给系统提示词用的技能说明。"""
    lines = []
    for name, s in SKILLS.items():
        ex = json.dumps({"skill": name, **EXAMPLES[name]}, ensure_ascii=False)
        rules = []
        for arg, rule in s["args"].items():
            if rule[0] == "enum":
                rules.append(f"{arg} 只能是 {'/'.join(rule[1])}")
            elif rule[0] == "num":
                rules.append(f"{arg} 是 {rule[1]}–{rule[2]} 的数字")
        lines.append(f"- {ex}  {s['desc']}" + (f"（{'；'.join(rules)}）" if rules else ""))
    return "\n".join(lines)


def check(call) -> str:
    """参数合法返回空字符串，否则返回原因。"""
    if not isinstance(call, dict):
        return "不是 JSON 对象"
    name = call.get("skill")
    if name not in SKILLS:
        return f"没有这个技能：{name}"
    for arg, rule in SKILLS[name]["args"].items():
        v = call.get(arg)
        if rule[0] == "str":
            if not isinstance(v, str) or not rule[1] <= len(v.strip()) <= rule[2]:
                return f"{name}.{arg} 应该是 {rule[1]}–{rule[2]} 个字的文字"
        elif rule[0] == "enum":
            if v not in rule[1]:
                return f"{name}.{arg} 只能是 {rule[1]}，收到 {v!r}"
        elif rule[0] == "num":
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not rule[1] <= v <= rule[2]:
                return f"{name}.{arg} 应该是 {rule[1]}–{rule[2]} 的数字，收到 {v!r}"
    extra = set(call) - {"skill"} - set(SKILLS[name]["args"])
    if extra:
        return f"{name} 多了不认识的参数：{sorted(extra)}"
    return ""


def parse(line: str):
    """解析大模型输出的一行。返回 (技能调用 or None, 错误原因)。空行和代码块标记直接忽略。"""
    line = line.strip().rstrip(",")
    if not line or line.startswith("```"):
        return None, ""
    try:
        call = json.loads(line)
    except json.JSONDecodeError:
        return None, f"不是合法 JSON：{line[:60]}"
    err = check(call)
    return (None, err) if err else (call, "")
