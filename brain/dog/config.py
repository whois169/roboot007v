"""读取 config.yaml 和 .env。路径都相对 brain/ 目录。"""
import os

import yaml
from dotenv import load_dotenv

BRAIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def path(p: str) -> str:
    """把配置里的相对路径变成 brain/ 下的绝对路径。"""
    return p if os.path.isabs(p) else os.path.join(BRAIN_DIR, p)


def rel(p: str) -> str:
    """相对 brain/ 的路径。有些 C++ 库在 Windows 上打不开中文路径，
    所以 main.py 启动时先切到 brain/，再用相对路径给这些库。"""
    return os.path.relpath(path(p), os.getcwd())


def load(config_file: str = "config.yaml") -> dict:
    load_dotenv(path(".env"))
    with open(path(config_file), encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    llm = cfg["llm"]
    llm["api_key"] = os.environ.get(llm.get("api_key_env", "LLM_API_KEY"), "").strip()
    return cfg
