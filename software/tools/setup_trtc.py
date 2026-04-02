#!/usr/bin/env python3
"""
TRTC 配置向导

帮助用户配置腾讯云 TRTC 音视频通话服务
"""

import sys
import os
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from configs.config import get_config
from configs.secrets import get_secrets


def print_header(title):
    print("")
    print("=" * 60)
    print(f"  {title}")
    print("=" * 60)


def check_current_config():
    """检查当前 TRTC 配置状态"""
    config = get_config()
    secrets = get_secrets()
    
    print_header("当前配置状态")
    
    sdk_app_id = config.trtc.sdk_app_id
    secret_key = secrets.trtc.secret_key
    room_id = config.trtc.room_id
    
    # 检查是否为默认/测试值
    is_default_sdk = sdk_app_id == 0 or sdk_app_id == 1400718166
    has_secret = bool(secret_key and len(secret_key) > 10)
    
    sdk_status = '(默认测试值)' if is_default_sdk else '[OK] 已配置'
    key_status = '[OK] 已配置' if has_secret else '[X] 未配置'
    print(f"SDKAppID:    {sdk_app_id} {sdk_status}")
    print(f"SecretKey:   {key_status}")
    print(f"RoomID:      {room_id}")
    
    return is_default_sdk or not has_secret


def show_setup_guide():
    """显示配置步骤指南"""
    print_header("TRTC 配置步骤")
    
    print("""
第一步：注册腾讯云账号
  1. 访问 https://cloud.tencent.com/
  2. 注册/登录腾讯云账号
  3. 完成实名认证（个人或企业）

第二步：开通 TRTC 服务
  1. 进入控制台 → 云产品 → 实时音视频 (TRTC)
  2. 点击"创建应用"
  3. 应用名称：HomeBot-Mahjong
  4. 创建完成后进入应用详情页

第三步：获取密钥信息
  在应用详情页 → 基本信息 中找到：
  
  ┌─────────────────────────────────────────┐
  │  SDKAppID:  1400xxxxxx                  │
  │  SDKSecretKey: xxxxxxxxxxxxxxxxxxxx...  │
  └─────────────────────────────────────────┘

第四步：配置到本项目
  将上述信息填入以下文件：
""")
    
    env_file = Path(__file__).parent.parent / ".env.local"
    print(f"  文件路径: {env_file}")
    print()
    
    # 显示需要添加的配置内容
    print("  需要添加以下内容到 .env.local：")
    print("  " + "-" * 50)
    print("  # 腾讯云 TRTC 配置")
    print("  TRTC_SDK_APP_ID=你的_SDKAppID")
    print("  TRTC_SECRET_KEY=你的_SecretKey")
    print("  " + "-" * 50)


def update_config():
    """交互式更新配置"""
    print_header("输入 TRTC 配置")
    
    sdk_app_id = input("请输入 SDKAppID: ").strip()
    secret_key = input("请输入 SecretKey: ").strip()
    
    if not sdk_app_id or not secret_key:
        print("错误：SDKAppID 和 SecretKey 不能为空")
        return False
    
    # 写入 .env.local
    env_file = Path(__file__).parent.parent / ".env.local"
    
    try:
        # 读取现有内容
        existing = ""
        if env_file.exists():
            existing = env_file.read_text(encoding='utf-8')
        
        # 移除旧的 TRTC 配置
        lines = existing.split('\n')
        new_lines = []
        skip_next = False
        for line in lines:
            if skip_next:
                skip_next = False
                continue
            if 'TRTC' in line:
                continue
            if '# 腾讯云 TRTC' in line:
                continue
            new_lines.append(line)
        
        # 添加新的配置
        config_section = f"""
# ============================================
# 腾讯云 TRTC 音视频配置
# ============================================
TRTC_SDK_APP_ID={sdk_app_id}
TRTC_SECRET_KEY={secret_key}
"""
        
        new_content = '\n'.join(new_lines) + config_section
        env_file.write_text(new_content, encoding='utf-8')
        
        print(f"\n[OK] 配置已保存到 {env_file}")
        return True
        
    except Exception as e:
        print(f"保存配置失败: {e}")
        return False


def verify_config():
    """验证配置是否正确"""
    print_header("验证配置")
    
    # 重新加载 secrets
    from configs.secrets import reload_secrets
    secrets = reload_secrets()
    
    if secrets.trtc.secret_key:
        print("[OK] SecretKey 已加载")
        print(f"  长度: {len(secrets.trtc.secret_key)} 字符")
        print(f"  预览: {secrets.trtc.secret_key[:8]}...{secrets.trtc.secret_key[-8:]}")
    else:
        print("[X] SecretKey 未加载")
        return False
    
    # 测试生成 UserSig
    try:
        config = get_config()
        from applications.mahjong_bot.web_server import gen_trtc_usersig
        test_sig = gen_trtc_usersig(
            config.trtc.sdk_app_id,
            secrets.trtc.secret_key,
            "test_user",
            86400
        )
        print("\n[OK] UserSig 生成测试通过")
        print(f"  UserSig 长度: {len(test_sig)} 字符")
        return True
    except Exception as e:
        print(f"\n[X] UserSig 生成失败: {e}")
        return False


def main():
    print_header("HomeBot TRTC 配置向导")
    print("\n本向导将帮助你配置腾讯云 TRTC 音视频通话服务")
    
    needs_setup = check_current_config()
    
    if not needs_setup:
        print("\n[OK] TRTC 已配置，无需操作")
        choice = input("\n是否重新配置? (y/N): ").strip().lower()
        if choice != 'y':
            return
    
    show_setup_guide()
    
    input("\n按回车键继续配置...")
    
    if update_config():
        verify_config()
        print_header("配置完成")
        print("\n现在可以启动服务测试 TRTC 功能：")
        print("  1. 启动麻将 Web 服务")
        print("  2. 在浏览器中打开 http://localhost:5100/mahjong")
        print("  3. 点击'加入通话'测试音视频")
    else:
        print("\n配置未完成，请重试")


if __name__ == "__main__":
    main()
