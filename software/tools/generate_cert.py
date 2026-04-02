#!/usr/bin/env python3
"""
生成自签名 HTTPS 证书，用于局域网访问

使用方法:
    python tools/generate_cert.py
    
生成的文件:
    - certs/server.crt (证书)
    - certs/server.key (私钥)
"""

import os
import sys
from pathlib import Path

# 项目根目录
PROJECT_ROOT = Path(__file__).parent.parent
CERT_DIR = PROJECT_ROOT / "certs"

def generate_cert():
    """使用 OpenSSL 生成自签名证书"""
    
    # 确保证书目录存在
    CERT_DIR.mkdir(exist_ok=True)
    
    key_file = CERT_DIR / "server.key"
    cert_file = CERT_DIR / "server.crt"
    
    # 检查是否已存在
    if key_file.exists() and cert_file.exists():
        print("证书已存在:")
        print(f"  私钥: {key_file}")
        print(f"  证书: {cert_file}")
        return True
    
    # 使用 OpenSSL 生成证书
    # 包含 SAN (Subject Alternative Name) 支持 IP 地址访问
    san_config = """
[SAN]
subjectAltName=DNS:localhost,DNS:*.local,IP:127.0.0.1,IP:0.0.0.0
"""
    
    config_file = CERT_DIR / "san.cnf"
    config_file.write_text(san_config)
    
    cmd = f'''
    openssl req -x509 -nodes -days 365 -newkey rsa:2048 \\
        -keyout "{key_file}" \\
        -out "{cert_file}" \\
        -subj "/C=CN/ST=State/L=City/O=HomeBot/OU=Dev/CN=localhost" \\
        -reqexts SAN -extensions SAN \\
        -config "{config_file}"
    '''
    
    print("生成自签名证书...")
    print(f"命令: {cmd}")
    
    result = os.system(cmd)
    
    # 清理临时配置文件
    config_file.unlink(missing_ok=True)
    
    if result == 0:
        print("\n证书生成成功!")
        print(f"  私钥: {key_file}")
        print(f"  证书: {cert_file}")
        print("\n使用方式:")
        print("  python -m applications.mahjong_bot --host 0.0.0.0 --port 5100 --ssl")
        return True
    else:
        print("\n证书生成失败，请确保已安装 OpenSSL")
        print("下载地址: https://slproweb.com/products/Win32OpenSSL.html")
        return False


if __name__ == "__main__":
    generate_cert()
