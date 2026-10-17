"""我查到的：从网上查来的信息。现在只有天气（Open-Meteo，免费、不用密钥）。

后台每 30 分钟刷新一次存起来，聊到相关话题时直接拿，不用现查。
“查到的”和“感觉到的”分开放：预报说的是“外面”，传感器测的是“屋里”。
"""
import asyncio
import json
import logging
import os
import time

import httpx

from . import config

log = logging.getLogger("lookup")

WMO = {0: "晴", 1: "大致晴", 2: "多云", 3: "阴", 45: "雾", 48: "雾凇", 51: "毛毛雨", 53: "毛毛雨", 55: "较大毛毛雨",
       56: "冻毛毛雨", 57: "冻毛毛雨", 61: "小雨", 63: "中雨", 65: "大雨", 66: "冻雨", 67: "冻雨", 71: "小雪", 73: "中雪",
       75: "大雪", 77: "米雪", 80: "阵雨", 81: "较强阵雨", 82: "强阵雨", 85: "阵雪", 86: "强阵雪", 95: "雷阵雨",
       96: "雷阵雨伴冰雹", 99: "强雷阵雨伴冰雹"}


class Weather:
    def __init__(self, city="北京", refresh_minutes=30, keywords=(), **_):
        self.city = city
        self.refresh = refresh_minutes * 60
        self.keywords = list(keywords)
        self.data = None          # 最近一次查到的结果
        self.fetched = 0.0        # 查的时间（时间戳）
        self.error = ""

    async def start(self):
        await self.update()
        asyncio.create_task(self._loop())

    async def _loop(self):
        while True:
            await asyncio.sleep(self.refresh)
            await self.update()

    async def _geo(self, client):
        """城市名 -> 经纬度。查一次存到 data/geo.json，以后不用再查。"""
        f = config.path("data/geo.json")
        cache = json.load(open(f, encoding="utf-8")) if os.path.exists(f) else {}
        if self.city not in cache:
            r = await client.get("https://geocoding-api.open-meteo.com/v1/search",
                                 params={"name": self.city, "count": 1, "language": "zh"})
            res = r.json().get("results")
            if not res:
                raise ValueError(f"找不到城市「{self.city}」，改一下 config.yaml 里 weather.city")
            g = res[0]
            cache[self.city] = dict(lat=g["latitude"], lon=g["longitude"],
                                    name="".join(x for x in (g.get("admin1", ""), g["name"]) if x))
            os.makedirs(os.path.dirname(f), exist_ok=True)
            json.dump(cache, open(f, "w", encoding="utf-8"), ensure_ascii=False)
        return cache[self.city]

    async def update(self):
        try:
            async with httpx.AsyncClient(timeout=10) as client:
                geo = await self._geo(client)
                r = await client.get("https://api.open-meteo.com/v1/forecast", params={
                    "latitude": geo["lat"], "longitude": geo["lon"], "timezone": "auto", "forecast_days": 2,
                    "current": "temperature_2m,relative_humidity_2m,apparent_temperature,weather_code,wind_speed_10m",
                    "daily": "temperature_2m_max,temperature_2m_min,precipitation_probability_max,weather_code"})
                r.raise_for_status()
                self.data = dict(r.json(), place=geo["name"])
                self.fetched = time.time()
                self.error = ""
                log.info("天气已更新：%s", self.text().split("\n")[0])
        except Exception as e:
            self.error = f"{type(e).__name__}: {e}"
            log.warning("查天气失败：%s", self.error)

    def short(self) -> str:
        """给屏幕数据流用的一行英文缩写，例如 WX OUT 21.2C RH77 CLOUDY。"""
        if not self.data:
            return "WX NO DATA"
        c = self.data["current"]
        code = c["weather_code"]
        sky = ("CLEAR" if code <= 1 else "CLOUDY" if code <= 3 else "FOG" if code <= 48 else "DRIZZLE" if code <= 57
               else "RAIN" if code <= 67 else "SNOW" if code <= 77 else "SHOWER" if code <= 82 else "SNOW" if code <= 86
               else "STORM")
        return f"WX OUT {c['temperature_2m']:g}C RH{c['relative_humidity_2m']} {sky}"

    def relevant(self, text: str) -> bool:
        return any(k in text for k in self.keywords)

    def text(self) -> str:
        if not self.data:
            return f"- 天气预报：没查到（{self.error or '还没查'}）"
        c, d = self.data["current"], self.data["daily"]
        age = (time.time() - self.fetched) / 60
        src = f"｜来源：Open-Meteo 天气预报｜位置：{self.data['place']}，室外｜查询时间：{age:.0f} 分钟前"
        days = ["今天", "明天"]
        lines = [f"- 外面现在：{WMO.get(c['weather_code'], '未知')}，{c['temperature_2m']:g} °C（体感 {c['apparent_temperature']:g} °C），"
                 f"湿度 {c['relative_humidity_2m']}%，风速 {c['wind_speed_10m']:g} km/h{src}"]
        for i, day in enumerate(days[:len(d["time"])]):
            lines.append(f"- {day}预报：{WMO.get(d['weather_code'][i], '未知')}，{d['temperature_2m_min'][i]:g}–"
                         f"{d['temperature_2m_max'][i]:g} °C，降水概率 {d['precipitation_probability_max'][i]}%{src}")
        return "\n".join(lines)
