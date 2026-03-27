#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""密钥配置向导

交互式配置 HomeBot 的 API 密钥

用法:
    python tools/setup_secrets.py
"""
import sys
import os
from pathlib import Path


def main():
    print("\n" + "=" * 60)
    print("       HomeBot API Key Setup Wizard")
    print("=" * 60)
    print()
    
    software_dir = Path(__file__).parent.parent
    env_file = software_dir / ".env.local"
    example_file = software_dir / ".env.example"
    
    # 检查是否已存在
    if env_file.exists():
        print(f"[!] 配置文件已存在: {env_file}")
        response = input("   是否覆盖? (y/N): ").strip().lower()
        if response != 'y':
            print("\n已取消，保留现有配置。")
            return 0
        print()
    
    # 读取示例文件作为模板
    template = ""
    if example_file.exists():
        with open(example_file, 'r', encoding='utf-8') as f:
            template = f.read()
    
    print("[TTS] 火山引擎 TTS 配置")
    print("-" * 40)
    print("获取地址: https://console.volcengine.com/speech/service")
    print()
    
    volcano_appid = input("AppID: ").strip()
    volcano_token = input("Access Token: ").strip()
    
    print()
    print("[LLM] DeepSeek LLM 配置")
    print("-" * 40)
    print("获取地址: https://platform.deepseek.com/api_keys")
    print()
    
    deepseek_key = input("API Key: ").strip()
    
    print()
    print("[Vision] 图片理解配置（可选）")
    print("-" * 40)
    print("留空将使用 DeepSeek 配置")
    print()
    
    vision_key = input("Vision API Key (可选): ").strip()
    
    # 生成配置文件
    env_content = f"""# HomeBot API 密钥配置
# 生成时间: 自动

# ============================================
# 火山引擎 TTS 配置
# ============================================
VOLCANO_APPID={volcano_appid}
VOLCANO_ACCESS_TOKEN={volcano_token}

# ============================================
# DeepSeek LLM 配置
# ============================================
DEEPSEEK_API_KEY={deepseek_key}
"""
    
    if vision_key:
        env_content += f"""
# ============================================
# 图片理解/Vision API 配置
# ============================================
VISION_PROVIDER=deepseek
VISION_API_KEY={vision_key}
"""
    
    # 写入文件
    try:
        with open(env_file, 'w', encoding='utf-8') as f:
            f.write(env_content)
        
        print()
        print("=" * 60)
        print(f"✅ 配置已保存到: {env_file}")
        print("=" * 60)
        print()
        print("📝 配置摘要:")
        print(f"   火山引擎 AppID: {'*' * 8} (已设置)")
        print(f"   火山引擎 Token: {'*' * 8} (已设置)")
        print(f"   DeepSeek Key: {'*' * 8} (已设置)")
        if vision_key:
            print("   Vision Key: ******** (已设置)")
        print()
        print("💡 提示: 如需修改，直接编辑 .env.local 文件或重新运行此脚本")
        print()
        
        return 0
        
    except Exception as e:
        print(f"\n[错误] 保存配置失败: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
