# -*- coding: utf-8 -*-
"""密钥管理模块 - 安全加载和管理API密钥

支持从环境变量和.env.local文件加载敏感配置
确保密钥不会意外提交到版本控制
"""
import os
import sys
from pathlib import Path
from typing import Optional
from dataclasses import dataclass, field

from common.logging import get_logger

logger = get_logger(__name__)


# 项目根目录（software/）
PROJECT_ROOT = Path(__file__).parent.parent.parent


def _load_env_file(env_path: Path) -> None:
    """加载.env文件到环境变量
    
    Args:
        env_path: .env文件路径
    """
    if not env_path.exists():
        return
    
    try:
        with open(env_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                # 跳过注释和空行
                if not line or line.startswith('#'):
                    continue
                # 解析KEY=VALUE
                if '=' in line:
                    key, value = line.split('=', 1)
                    key = key.strip()
                    value = value.strip().strip('"\'')  # 去除引号
                    # 只设置尚未存在的环境变量（允许用户通过系统环境变量覆盖）
                    if key not in os.environ:
                        os.environ[key] = value
        logger.debug(f"已加载环境变量文件: {env_path}")
    except Exception as e:
        logger.warning(f"加载环境变量文件失败: {e}")


def _load_all_env_files() -> None:
    """按优先级加载所有环境变量文件"""
    # 优先级：.env.local > .env.development > .env.production > .env
    env_files = [
        PROJECT_ROOT / ".env.local",
        PROJECT_ROOT / ".env.development",
        PROJECT_ROOT / ".env.production",
        PROJECT_ROOT / ".env",
    ]
    
    for env_file in env_files:
        if env_file.exists():
            _load_env_file(env_file)


# 模块加载时自动执行
_load_all_env_files()


@dataclass
class TTSSecrets:
    """TTS 密钥配置"""
    appid: str = ""
    access_token: str = ""
    resource_id: str = "seed-tts-2.0"
    voice_type: str = "zh_female_vv_uranus_bigtts"


@dataclass
class LLMSecrets:
    """LLM 密钥配置"""
    api_key: str = ""
    api_url: str = "https://ark.cn-beijing.volces.com/api/v3"  # 默认使用火山Ark
    model: str = ""  # 火山Ark需要填写模型ID，如 ep-20250324123456-abcdef


@dataclass
class VisionSecrets:
    """图片理解/Vision API 密钥配置"""
    provider: str = "deepseek"  # deepseek/qwen/openai
    api_key: str = ""
    api_url: str = ""
    model: str = ""


@dataclass
class Secrets:
    """所有密钥配置"""
    tts: TTSSecrets = field(default_factory=TTSSecrets)
    llm: LLMSecrets = field(default_factory=LLMSecrets)
    vision: VisionSecrets = field(default_factory=VisionSecrets)


# 全局密钥实例
_secrets_instance: Optional[Secrets] = None


def _get_env(key: str, default: str = "") -> str:
    """安全获取环境变量
    
    Args:
        key: 环境变量名
        default: 默认值
        
    Returns:
        环境变量值或默认值
    """
    return os.environ.get(key, default)


def _mask_key(key: str, visible_start: int = 4, visible_end: int = 4) -> str:
    """脱敏显示API密钥
    
    Args:
        key: 原始密钥
        visible_start: 开头显示的字符数
        visible_end: 结尾显示的字符数
        
    Returns:
        脱敏后的密钥，如 sk-****1234
    """
    if len(key) <= visible_start + visible_end:
        return "*" * len(key) if key else "(未设置)"
    return f"{key[:visible_start]}****{key[-visible_end:]}"


def load_secrets() -> Secrets:
    """加载所有密钥配置
    
    从环境变量加载敏感配置，支持多种命名方式
    
    Returns:
        Secrets: 密钥配置对象
    """
    # TTS 配置 - 支持多种环境变量名
    tts = TTSSecrets(
        appid=_get_env("VOLCANO_APPID", _get_env("TTS_APPID", "")),
        access_token=_get_env("VOLCANO_ACCESS_TOKEN", _get_env("TTS_ACCESS_TOKEN", "")),
        resource_id=_get_env("VOLCANO_RESOURCE_ID", "seed-tts-2.0"),
        voice_type=_get_env("VOLCANO_VOICE_TYPE", "zh_female_vv_uranus_bigtts"),
    )
    
    # LLM 配置 - 优先使用火山Ark，兼容DeepSeek
    llm = LLMSecrets(
        api_key=_get_env("ARK_API_KEY", _get_env("VOLCANO_API_KEY", _get_env("DEEPSEEK_API_KEY", _get_env("LLM_API_KEY", "")))),
        api_url=_get_env("ARK_API_URL", _get_env("LLM_API_URL", "https://ark.cn-beijing.volces.com/api/v3")),
        model=_get_env("ARK_MODEL_ID", _get_env("VOLCANO_MODEL_ID", _get_env("DEEPSEEK_MODEL", _get_env("LLM_MODEL", "")))),
    )
    
    # Vision 配置
    vision_provider = _get_env("VISION_PROVIDER", "deepseek")
    
    # 根据provider获取对应的配置
    vision_api_key = _get_env("VISION_API_KEY", "")
    vision_api_url = _get_env("VISION_API_URL", "")
    vision_model = _get_env("VISION_MODEL", "")
    
    # 如果没有单独设置VISION配置，使用LLM的配置
    if not vision_api_key and vision_provider == "deepseek":
        vision_api_key = llm.api_key
        vision_api_url = llm.api_url or "https://api.deepseek.com/v1"
        vision_model = vision_model or "deepseek-chat"
    
    vision = VisionSecrets(
        provider=vision_provider,
        api_key=vision_api_key,
        api_url=vision_api_url,
        model=vision_model,
    )
    
    return Secrets(tts=tts, llm=llm, vision=vision)


def get_secrets() -> Secrets:
    """获取全局密钥实例
    
    Returns:
        Secrets: 全局密钥配置实例
    """
    global _secrets_instance
    if _secrets_instance is None:
        _secrets_instance = load_secrets()
    return _secrets_instance


def reload_secrets() -> Secrets:
    """重新加载密钥配置
    
    用于在运行时重新读取环境变量
    
    Returns:
        Secrets: 新的密钥配置实例
    """
    global _secrets_instance
    _load_all_env_files()
    _secrets_instance = load_secrets()
    return _secrets_instance


def check_secrets(verbose: bool = True) -> dict:
    """检查密钥配置状态
    
    检查各项密钥是否已配置，并返回状态报告
    
    Args:
        verbose: 是否打印详细信息
        
    Returns:
        dict: 各服务的配置状态
    """
    secrets = get_secrets()
    
    status = {
        "tts": {
            "configured": bool(secrets.tts.appid and secrets.tts.access_token),
            "appid": secrets.tts.appid[:6] + "..." if secrets.tts.appid else "(未设置)",
            "access_token": _mask_key(secrets.tts.access_token),
        },
        "llm": {
            "configured": bool(secrets.llm.api_key),
            "api_key": _mask_key(secrets.llm.api_key),
            "model": secrets.llm.model,
        },
        "vision": {
            "configured": bool(secrets.vision.api_key),
            "provider": secrets.vision.provider,
            "api_key": _mask_key(secrets.vision.api_key),
        },
    }
    
    if verbose:
        print("\n" + "=" * 50)
        print("HomeBot API Key Configuration Status")
        print("=" * 50)
        
        # TTS 状态
        tts_ok = status["tts"]["configured"]
        status_icon = "[OK]" if tts_ok else "[MISSING]"
        print(f"\n[TTS] 火山引擎 TTS: {status_icon}")
        if tts_ok:
            print(f"   AppID: {status['tts']['appid']}")
            print(f"   Access Token: {status['tts']['access_token']}")
        else:
            print("   [提示] 设置 VOLCANO_APPID 和 VOLCANO_ACCESS_TOKEN 环境变量")
        
        # LLM 状态
        llm_ok = status["llm"]["configured"]
        status_icon = "[OK]" if llm_ok else "[MISSING]"
        print(f"\n[LLM] 火山Ark LLM: {status_icon}")
        if llm_ok:
            print(f"   API Key: {status['llm']['api_key']}")
            print(f"   Model: {status['llm']['model'] if status['llm']['model'] else '(未设置，必填)'}")
        else:
            print("   [提示] 设置 ARK_API_KEY 和 ARK_MODEL_ID 环境变量")
            print("   ARK_MODEL_ID 格式: ep-20250324123456-abcdef")
        
        # Vision 状态
        vision_ok = status["vision"]["configured"]
        status_icon = "[OK]" if vision_ok else "[MISSING]"
        print(f"\n[Vision] 图片理解: {status_icon}")
        print(f"   Provider: {status['vision']['provider']}")
        if vision_ok:
            print(f"   API Key: {status['vision']['api_key']}")
        elif status["vision"]["provider"] == "deepseek" and llm_ok:
            print("   [提示] 将复用 DeepSeek LLM 的配置")
            status["vision"]["configured"] = True  # 复用LLM配置
        else:
            print("   [提示] 设置 VISION_API_KEY 环境变量")
        
        # 配置文件路径提示
        local_env = PROJECT_ROOT / ".env.local"
        print(f"\n[文件] 配置文件路径: {local_env}")
        if not local_env.exists():
            example_env = PROJECT_ROOT / ".env.example"
            print(f"[提示] 复制 {example_env.name} 为 {local_env.name} 并填入密钥")
        
        print("=" * 50 + "\n")
    
    return status


def require_secrets(service: str) -> None:
    """检查特定服务的密钥是否已配置
    
    如果未配置，打印帮助信息并退出程序
    
    Args:
        service: 服务名称 (tts/llm/vision)
    """
    secrets = get_secrets()
    
    if service == "tts":
        if not (secrets.tts.appid and secrets.tts.access_token):
            logger.error("火山引擎 TTS 密钥未配置")
            print("\n[错误] 火山引擎 TTS 密钥未配置")
            print("\n请设置以下环境变量之一:")
            print("  1. VOLCANO_APPID and VOLCANO_ACCESS_TOKEN")
            print("  2. TTS_APPID and TTS_ACCESS_TOKEN")
            print(f"\n或创建 {PROJECT_ROOT / '.env.local'} 文件，格式如下:")
            print("  VOLCANO_APPID=your_appid")
            print("  VOLCANO_ACCESS_TOKEN=your_token")
            sys.exit(1)
    
    elif service == "llm":
        if not secrets.llm.api_key:
            logger.error("火山Ark LLM API Key 未配置")
            print("\n[错误] 火山Ark LLM API Key 未配置")
            print("\n请设置以下环境变量:")
            print("  ARK_API_KEY=your_api_key")
            print("  ARK_MODEL_ID=ep-your_model_id")
            print(f"\n或创建 {PROJECT_ROOT / '.env.local'} 文件，格式如下:")
            print("  ARK_API_KEY=your_api_key")
            print("  ARK_MODEL_ID=ep-20250324123456-abcdef")
            sys.exit(1)
        if not secrets.llm.model:
            logger.error("火山Ark 模型ID未配置")
            print("\n[错误] 火山Ark 模型ID未配置")
            print("\n请在 .env.local 文件中设置 ARK_MODEL_ID:")
            print("  ARK_MODEL_ID=ep-20250324123456-abcdef")
            print("\n注意: 需要在火山方舟控制台创建推理接入点并复制模型ID")
            sys.exit(1)
    
    elif service == "vision":
        # Vision 可以复用 DeepSeek 的配置
        if not secrets.vision.api_key:
            if secrets.llm.api_key and secrets.vision.provider == "deepseek":
                return  # 允许复用LLM配置
            logger.error("Vision API Key 未配置")
            print("\n[错误] Vision API Key 未配置")
            print("\n请设置以下环境变量:")
            print("  VISION_API_KEY")
            print(f"\n或创建 {PROJECT_ROOT / '.env.local'} 文件")
            sys.exit(1)


if __name__ == "__main__":
    # 命令行检查密钥配置
    check_secrets(verbose=True)
