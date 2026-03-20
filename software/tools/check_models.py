#!/usr/bin/env python3
"""
诊断语音模型文件

检查模型文件是否存在、大小是否正确
"""
import sys
from pathlib import Path

# 添加 src 到路径
script_dir = Path(__file__).parent.parent
sys.path.insert(0, str(script_dir / "src"))

from configs.config import get_config

def check_file(path: Path, desc: str):
    """检查文件"""
    if path.exists():
        size = path.stat().st_size
        size_mb = size / 1024 / 1024
        print(f"  [OK] {desc}")
        print(f"       路径: {path}")
        print(f"       大小: {size_mb:.2f} MB ({size} bytes)")
        return True
    else:
        print(f"  [缺失] {desc}")
        print(f"       路径: {path}")
        return False

def main():
    config = get_config()
    
    # 计算模型目录
    base_path = script_dir
    wakeup_dir = base_path / config.speech.wakeup_model_path
    asr_dir = base_path / config.speech.asr_model_path
    
    print("=" * 60)
    print("HomeBot 语音模型诊断工具")
    print("=" * 60)
    print()
    
    print(f"项目根目录: {base_path}")
    print()
    
    # 检查唤醒模型
    print("唤醒模型目录:", wakeup_dir)
    wakeup_files = {
        "encoder": wakeup_dir / config.speech.wakeup_encoder_file,
        "decoder": wakeup_dir / config.speech.wakeup_decoder_file,
        "joiner": wakeup_dir / config.speech.wakeup_joiner_file,
        "tokens": wakeup_dir / "tokens.txt",
        "keywords": wakeup_dir / config.speech.wakeup_keyword_file,
    }
    
    wakeup_ok = True
    for name, path in wakeup_files.items():
        if not check_file(path, name):
            wakeup_ok = False
        print()
    
    # 检查ASR模型
    print("ASR模型目录:", asr_dir)
    asr_files = {
        "encoder": asr_dir / config.speech.asr_encoder_file,
        "decoder": asr_dir / config.speech.asr_decoder_file,
        "joiner": asr_dir / config.speech.asr_joiner_file,
        "tokens": asr_dir / "tokens.txt",
    }
    
    asr_ok = True
    for name, path in asr_files.items():
        if not check_file(path, name):
            asr_ok = False
        print()
    
    # 列出目录内容
    print("=" * 60)
    print("目录内容检查")
    print("=" * 60)
    print()
    
    if wakeup_dir.exists():
        print(f"唤醒目录 ({wakeup_dir}):")
        for f in sorted(wakeup_dir.iterdir()):
            if f.is_file():
                print(f"  - {f.name} ({f.stat().st_size / 1024:.1f} KB)")
    else:
        print(f"唤醒目录不存在: {wakeup_dir}")
    print()
    
    if asr_dir.exists():
        print(f"ASR目录 ({asr_dir}):")
        for f in sorted(asr_dir.iterdir()):
            if f.is_file():
                print(f"  - {f.name} ({f.stat().st_size / 1024:.1f} KB)")
    else:
        print(f"ASR目录不存在: {asr_dir}")
    print()
    
    # 总结
    print("=" * 60)
    if wakeup_ok and asr_ok:
        print("[OK] 所有模型文件已就绪")
        
        # 尝试加载模型验证
        print()
        print("正在验证模型文件格式...")
        try:
            import sherpa_onnx
            
            # 测试唤醒模型
            print("测试唤醒模型...")
            detector = sherpa_onnx.KeywordSpotter(
                tokens=str(wakeup_files["tokens"]),
                encoder=str(wakeup_files["encoder"]),
                decoder=str(wakeup_files["decoder"]),
                joiner=str(wakeup_files["joiner"]),
                keywords_file=str(wakeup_files["keywords"]),
                keywords_score=1,
                keywords_threshold=0.3,
                num_threads=1,
                provider="cpu",
            )
            print("[OK] 唤醒模型加载成功")
            
            # 测试ASR模型
            print("测试ASR模型...")
            recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
                tokens=str(asr_files["tokens"]),
                encoder=str(asr_files["encoder"]),
                decoder=str(asr_files["decoder"]),
                joiner=str(asr_files["joiner"]),
                num_threads=1,
                provider="cpu",
                sample_rate=16000,
                feature_dim=80,
            )
            print("[OK] ASR模型加载成功")
            
        except Exception as e:
            print(f"[错误] 模型加载失败: {e}")
            import traceback
            traceback.print_exc()
            
    else:
        print("[错误] 部分模型文件缺失")
        print()
        print("建议:")
        print("  1. 删除 models/ 目录重新下载")
        print("  2. 运行: python tools/download_speech_models.py")
    print("=" * 60)

if __name__ == "__main__":
    main()
