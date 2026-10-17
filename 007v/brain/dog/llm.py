"""大模型适配层：任何 OpenAI 兼容接口（DeepSeek、通义千问、豆包、智谱……）都走这里。
没有密钥时用 MockLLM 给假回答，方便先测流程。
"""
import asyncio
import json
import logging

from openai import AsyncOpenAI

log = logging.getLogger("llm")


class ChatLLM:
    def __init__(self, base_url, model, api_key, temperature=0.8, max_tokens=400, timeout_seconds=20, **_):
        self.client = AsyncOpenAI(base_url=base_url, api_key=api_key, timeout=timeout_seconds, max_retries=1)
        self.model, self.temperature, self.max_tokens = model, temperature, max_tokens
        self.name = f"{model} @ {base_url}"

    async def stream(self, messages):
        """流式产出文字片段。"""
        resp = await self.client.chat.completions.create(
            model=self.model, messages=messages, stream=True,
            temperature=self.temperature, max_tokens=self.max_tokens)
        async for chunk in resp:
            if chunk.choices and chunk.choices[0].delta.content:
                yield chunk.choices[0].delta.content

    async def complete(self, messages) -> str:
        """一次性拿到完整回答（写每日小结这种不用边想边说的场合）。"""
        out = ""
        async for d in self.stream(messages):
            out += d
        return out


class MockLLM:
    """假回答：照样按“每行一个技能 JSON”的格式、一点点吐出来，模拟网络延迟。"""
    name = "模拟模式（假回答）"

    async def stream(self, messages):
        said = messages[-1]["content"].rsplit("【用户说】", 1)[-1].strip()
        lines = [{"skill": "look", "direction": "center"},
                 {"skill": "say", "text": f"收到：{said[:30]}"},
                 {"skill": "say", "text": "当前为模拟模式，未连接大模型。"}]
        if "记住" in said:
            lines.append({"skill": "remember", "fact": said[:60]})
        await asyncio.sleep(0.6)
        for obj in lines:
            text = json.dumps(obj, ensure_ascii=False) + "\n"
            for i in range(0, len(text), 8):
                await asyncio.sleep(0.03)
                yield text[i:i + 8]


def make(cfg_llm: dict, force_mock=False):
    mode = cfg_llm.get("mock", "auto")
    if force_mock or mode is True:
        return MockLLM()
    if not cfg_llm.get("api_key"):
        if mode is False:
            raise SystemExit(f"config.yaml 里 llm.mock 是 false，但 .env 里没有 {cfg_llm.get('api_key_env')}")
        log.warning("没有找到 API 密钥，使用模拟模式")
        return MockLLM()
    return ChatLLM(**cfg_llm)


VISION_PROMPT = ("你是一台轮足机器人（机械狗007v）的眼睛，这是你头上摄像头刚拍的一张照片。"
                 "用一两句中文客观描述：有没有人、几个人、在做什么、离镜头远近、表情和姿势；没人就说看到了什么。"
                 "不要猜测人的身份，不要编造看不清的细节。")


class VisionLLM:
    """看图模型（OpenAI 兼容接口，图片用 base64 放在消息里）。只在有事的时候调用一次。"""

    def __init__(self, base_url, model, api_key, timeout_seconds=15, max_tokens=150, **_):
        self.client = AsyncOpenAI(base_url=base_url, api_key=api_key, timeout=timeout_seconds, max_retries=0)
        self.model, self.max_tokens = model, max_tokens
        self.name = f"{model} @ {base_url}"

    async def describe(self, jpeg: bytes, prompt: str = VISION_PROMPT) -> str:
        import base64
        url = "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()
        resp = await self.client.chat.completions.create(
            model=self.model, max_tokens=self.max_tokens,
            messages=[{"role": "user", "content": [
                {"type": "image_url", "image_url": {"url": url}},
                {"type": "text", "text": prompt}]}])
        return (resp.choices[0].message.content or "").strip()


def make_vision(cfg_model: dict):
    """看图模型没配全（地址、模型名、密钥缺一个）就返回 None：截图不上传，只用本地检测。"""
    import os
    key = os.environ.get(cfg_model.get("api_key_env", "VISION_API_KEY"), "").strip()
    if not (cfg_model.get("base_url") and cfg_model.get("model") and key):
        return None
    return VisionLLM(api_key=key, **{k: v for k, v in cfg_model.items() if k != "api_key_env"})


async def lines(deltas):
    """把流式文字片段按换行切成完整的行，一行一产出。"""
    buf = ""
    async for d in deltas:
        buf += d
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            yield line
    if buf.strip():
        yield buf
