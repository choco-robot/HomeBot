"""LLM 连接测试工具

测试火山 Ark / DeepSeek 等 LLM 配置是否正确，并给出明确的诊断建议。
"""
import os
import sys

# 把 software/src 加入路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from configs.config import get_config
from configs.secrets import get_secrets, require_secrets
from openai import OpenAI, AuthenticationError, APIConnectionError


def mask_key(key: str, visible: int = 6) -> str:
    if not key:
        return "(未设置)"
    if len(key) <= visible * 2:
        return "*" * len(key)
    return f"{key[:visible]}****{key[-visible:]}"


def main():
    print("=" * 60)
    print("HomeBot LLM 连接诊断")
    print("=" * 60)

    config = get_config()
    secrets = get_secrets()

    print(f"\n[配置]")
    print(f"  provider: {config.llm.provider}")
    print(f"  api_url:  {config.llm.api_url}")
    print(f"  model:    {config.llm.model}")
    print(f"  api_key:  {mask_key(config.llm.api_key)}")

    # 检查是否配置完整
    if not config.llm.api_key:
        print("\n[错误] LLM API Key 未配置")
        print("  请在 software/.env.local 中设置 ARK_API_KEY 或 DEEPSEEK_API_KEY")
        return 1

    if not config.llm.model:
        print("\n[错误] LLM 模型未配置")
        print("  火山 Ark: 设置 ARK_MODEL_ID=ep-xxxxxxxx")
        print("  DeepSeek: 设置 DEEPSEEK_MODEL=deepseek-chat")
        return 1

    # 格式检查
    key = config.llm.api_key
    is_ark_endpoint = "volces.com" in config.llm.api_url or "ark" in config.llm.api_url
    is_deepseek_endpoint = "deepseek.com" in config.llm.api_url

    if is_ark_endpoint and key.startswith("sk-"):
        print("\n[警告] 当前使用火山 Ark 端点，但 API Key 以 'sk-' 开头")
        print("  火山 Ark 的 API Key 通常不是 'sk-' 格式")
        print("  你可能把 DeepSeek 的 Key 填到了 ARK_API_KEY")
        print("  解决方式：")
        print("    1. 使用火山 Ark 的 Key（从 https://console.volcengine.com/ark/ 获取）")
        print("    2. 或改为使用 DeepSeek：provider=deepseek, api_url=https://api.deepseek.com/v1")

    if is_deepseek_endpoint and not key.startswith("sk-"):
        print("\n[警告] 当前使用 DeepSeek 端点，但 API Key 不是 'sk-' 开头")
        print("  请确认 DEEPSEEK_API_KEY 是否正确")

    # 尝试调用
    print("\n[测试] 正在发送测试请求...")
    client = OpenAI(api_key=config.llm.api_key, base_url=config.llm.api_url)

    try:
        response = client.chat.completions.create(
            model=config.llm.model,
            messages=[
                {"role": "system", "content": "你是一个简洁的助手"},
                {"role": "user", "content": "你好，请回复'LLM测试成功'"},
            ],
            temperature=0.1,
            max_tokens=50,
        )
        content = response.choices[0].message.content
        print(f"\n[成功] LLM 响应: {content}")
        return 0

    except AuthenticationError as e:
        print(f"\n[失败] 认证错误: {e}")
        err_body = getattr(e, "body", {}) or {}
        err_detail = err_body.get("error", {})
        if err_detail:
            print(f"  错误码: {err_detail.get('code')}")
            print(f"  错误信息: {err_detail.get('message')}")
        print("\n  常见原因:")
        print("    1. API Key 错误、过期或格式不匹配当前端点")
        print("    2. 火山 Ark 需要使用 ep- 开头的推理接入点 ID 作为 model")
        print("    3. 账号未开通对应模型或余额不足")
        return 1

    except APIConnectionError as e:
        print(f"\n[失败] 连接错误: {e}")
        print("  请检查网络、代理设置或 api_url 是否正确")
        return 1

    except Exception as e:
        print(f"\n[失败] 未知错误: {type(e).__name__}: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
