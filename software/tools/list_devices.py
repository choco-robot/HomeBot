# -*- coding: utf-8 -*-
"""
设备信息查看工具

用于列出系统中可用的串口设备、摄像头设备和麦克风设备，
方便用户快速了解设备端口号并修改配置。

使用方法:
    python software/tools/list_devices.py
    
    可选参数:
        --test-camera    测试摄像头是否可以正常打开
        --test-mic       测试麦克风是否可以正常录音
"""

import sys
import os
import argparse
from typing import List, Dict, Any, Optional

# 添加 src 目录到路径以导入配置
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../src'))

try:
    from configs import get_config
    HAS_CONFIG = True
except ImportError:
    HAS_CONFIG = False


def print_section(title: str):
    """打印分节标题"""
    print("\n" + "=" * 60)
    print(f" {title}")
    print("=" * 60)


def print_config_hint(config_key: str, config_value: str, config_file: str = "software/src/configs/config.py"):
    """打印配置提示"""
    print(f"   💡 配置提示: 修改 {config_file} 中的 {config_key} = \"{config_value}\"")


def list_serial_ports() -> List[Dict[str, Any]]:
    """列出所有串口设备"""
    devices = []
    try:
        from serial.tools import list_ports
        
        ports = list_ports.comports()
        if not ports:
            print("   未找到串口设备")
            return devices
            
        for port in sorted(ports):
            device_info = {
                "port": port.device,
                "description": port.description or "未知",
                "hwid": port.hwid or "未知",
                "manufacturer": getattr(port, "manufacturer", None),
                "vid": port.vid,
                "pid": port.pid,
            }
            devices.append(device_info)
            
            # 打印设备信息
            print(f"\n   📟 串口: {port.device}")
            print(f"      描述: {port.description or 'N/A'}")
            print(f"      硬件ID: {port.hwid or 'N/A'}")
            if port.vid and port.pid:
                print(f"      VID/PID: {port.vid:04X}:{port.pid:04X}")
                
    except ImportError:
        print("   ❌ 未安装 pyserial，请先安装: pip install pyserial")
    except Exception as e:
        print(f"   ❌ 获取串口信息失败: {e}")
        
    return devices


def list_cameras(test_open: bool = False) -> List[Dict[str, Any]]:
    """列出所有摄像头设备"""
    devices = []
    try:
        import cv2
        
        # 尝试检测前10个摄像头索引
        print("   正在扫描摄像头设备...")
        found_any = False
        
        for index in range(10):
            cap = cv2.VideoCapture(index, cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2)
            if cap.isOpened():
                found_any = True
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                backend = cap.getBackendName() if hasattr(cap, 'getBackendName') else "未知"
                
                device_info = {
                    "index": index,
                    "width": width,
                    "height": height,
                    "fps": fps,
                    "backend": backend,
                }
                devices.append(device_info)
                
                # 打印设备信息
                status = "✅ 可打开" if test_open else ""
                print(f"\n   📷 摄像头索引: {index} {status}")
                print(f"      分辨率: {width}x{height}")
                print(f"      帧率: {fps:.1f} fps" if fps > 0 else "      帧率: 未知")
                print(f"      后端: {backend}")
                
                # 如果请求测试，显示预览
                if test_open:
                    print(f"      正在测试预览，按 'q' 键继续...")
                    test_count = 0
                    while test_count < 150:  # 最多显示5秒 (30fps * 5)
                        ret, frame = cap.read()
                        if ret:
                            cv2.imshow(f"Camera {index} Test", frame)
                            if cv2.waitKey(33) & 0xFF == ord('q'):
                                break
                            test_count += 1
                        else:
                            print(f"      ⚠️ 无法读取帧")
                            break
                    cv2.destroyAllWindows()
                    
            cap.release()
            
        if not found_any:
            print("   未找到摄像头设备")
            
    except ImportError:
        print("   ❌ 未安装 opencv-python，请先安装: pip install opencv-python")
    except Exception as e:
        print(f"   ❌ 获取摄像头信息失败: {e}")
        
    return devices


def list_microphones() -> List[Dict[str, Any]]:
    """列出所有麦克风设备"""
    devices = []
    
    # 方法1: 使用 pyaudio
    try:
        import pyaudio
        
        p = pyaudio.PyAudio()
        print(f"   使用 PyAudio (版本: {pyaudio.get_portaudio_version_text()})")
        
        mic_count = 0
        for i in range(p.get_device_count()):
            info = p.get_device_info_by_index(i)
            # 只显示输入通道数大于0的设备（麦克风）
            if info.get('maxInputChannels', 0) > 0:
                mic_count += 1
                device_info = {
                    "index": i,
                    "name": info.get('name', '未知'),
                    "channels": info.get('maxInputChannels', 0),
                    "sample_rate": info.get('defaultSampleRate', 0),
                    "api": "pyaudio",
                }
                devices.append(device_info)
                
                # 打印设备信息
                print(f"\n   🎤 麦克风索引: {i}")
                print(f"      名称: {info.get('name', 'N/A')}")
                print(f"      输入通道: {info.get('maxInputChannels', 'N/A')}")
                print(f"      默认采样率: {int(info.get('defaultSampleRate', 0))} Hz")
                
        if mic_count == 0:
            print("   未找到麦克风设备")
            
        p.terminate()
        
    except ImportError:
        print("   ⚠️ 未安装 pyaudio，尝试使用 sounddevice...")
        
        # 方法2: 使用 sounddevice
        try:
            import sounddevice as sd
            
            print(f"   使用 SoundDevice")
            devices_list = sd.query_devices()
            
            mic_count = 0
            for i, info in enumerate(devices_list):
                # 只显示输入设备（麦克风）
                if info.get('max_input_channels', 0) > 0:
                    mic_count += 1
                    device_info = {
                        "index": i,
                        "name": info.get('name', '未知'),
                        "channels": info.get('max_input_channels', 0),
                        "sample_rate": info.get('default_samplerate', 0),
                        "api": "sounddevice",
                    }
                    devices.append(device_info)
                    
                    # 打印设备信息
                    print(f"\n   🎤 麦克风索引: {i}")
                    print(f"      名称: {info.get('name', 'N/A')}")
                    print(f"      输入通道: {info.get('max_input_channels', 'N/A')}")
                    print(f"      默认采样率: {int(info.get('default_samplerate', 0))} Hz")
                    
            if mic_count == 0:
                print("   未找到麦克风设备")
                
        except ImportError:
            print("   ❌ 未安装 pyaudio 或 sounddevice")
            print("      请安装: pip install pyaudio 或 pip install sounddevice")
            
    except Exception as e:
        print(f"   ❌ 获取麦克风信息失败: {e}")
        
    return devices


def test_microphone(device_index: Optional[int] = None, duration: float = 3.0):
    """测试麦克风录音"""
    try:
        import pyaudio
        import wave
        import tempfile
        
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        RECORD_SECONDS = int(duration)
        
        p = pyaudio.PyAudio()
        
        # 如果没有指定设备，尝试找到默认输入设备
        if device_index is None:
            for i in range(p.get_device_count()):
                info = p.get_device_info_by_index(i)
                if info.get('maxInputChannels', 0) > 0:
                    device_index = i
                    print(f"   使用默认麦克风 (索引: {i}): {info.get('name')}")
                    break
        else:
            info = p.get_device_info_by_index(device_index)
            print(f"   测试麦克风 (索引: {device_index}): {info.get('name')}")
            
        if device_index is None:
            print("   ❌ 未找到可用的麦克风")
            return False
            
        stream = p.open(format=FORMAT,
                       channels=CHANNELS,
                       rate=RATE,
                       input=True,
                       input_device_index=device_index,
                       frames_per_buffer=CHUNK)
        
        print(f"   🎙️  开始录音 {RECORD_SECONDS} 秒，请说话...")
        
        frames = []
        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
            
        print("   ✅ 录音完成")
        
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        # 保存到临时文件
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            temp_path = f.name
            
        wf = wave.open(temp_path, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        
        print(f"   录音文件已保存: {temp_path}")
        
        # 播放录音（可选）
        try:
            import simpleaudio as sa
            print("   🔊 播放录音...")
            wave_obj = sa.WaveObject.from_wave_file(temp_path)
            play_obj = wave_obj.play()
            play_obj.wait_done()
        except ImportError:
            print("   ℹ️  安装 simpleaudio 可播放录音: pip install simpleaudio")
            
        return True
        
    except ImportError:
        print("   ❌ 未安装 pyaudio，无法测试麦克风")
        print("      请安装: pip install pyaudio")
        return False
    except Exception as e:
        print(f"   ❌ 测试麦克风失败: {e}")
        return False


def print_summary(serial_devices: List[Dict], camera_devices: List[Dict], mic_devices: List[Dict]):
    """打印配置摘要"""
    print("\n" + "=" * 60)
    print(" 配置摘要 - 请根据上述信息修改 software/src/configs/config.py")
    print("=" * 60)
    
    # 获取当前配置
    current_config = {}
    if HAS_CONFIG:
        try:
            config = get_config()
            current_config = {
                "chassis_port": config.chassis.serial_port,
                "arm_port": config.arm.serial_port,
                "camera_id": config.camera.device_id,
                "mic_index": config.speech.mic_index,
            }
        except Exception:
            pass
    
    # 串口配置建议
    print("\n📟 串口配置 (ChassisConfig / ArmConfig):")
    if current_config:
        print(f"   当前配置: chassis.serial_port = \"{current_config.get('chassis_port')}\"")
        print(f"   当前配置: arm.serial_port = \"{current_config.get('arm_port')}\"")
    if serial_devices:
        for dev in serial_devices:
            port = dev.get("port", "")
            desc = dev.get("description", "")
            if "CH340" in desc or "USB-SERIAL" in desc or "Arduino" in desc or "tty" in port or "COM" in port:
                print(f"\n   👉 推荐串口 (舵机/底盘): {port}")
                print(f"      修改: chassis.serial_port = \"{port}\"")
                print(f"      修改: arm.serial_port = \"{port}\"")
    else:
        print("   未检测到串口设备")
        
    # 摄像头配置建议
    print("\n📷 摄像头配置 (CameraConfig):")
    if current_config:
        print(f"   当前配置: camera.device_id = {current_config.get('camera_id')}")
    if camera_devices:
        for dev in camera_devices:
            idx = dev.get("index", 0)
            print(f"\n   👉 可用摄像头: 索引 {idx}")
            print(f"      修改: camera.device_id = {idx}")
    else:
        print("   未检测到摄像头设备")
        
    # 麦克风配置建议
    print("\n🎤 麦克风配置 (SpeechConfig):")
    if current_config:
        print(f"   当前配置: speech.mic_index = {current_config.get('mic_index')}")
    if mic_devices:
        for dev in mic_devices:
            idx = dev.get("index", 0)
            name = dev.get("name", "未知")
            print(f"\n   👉 可用麦克风: 索引 {idx} - {name}")
            print(f"      修改: speech.mic_index = {idx}")
    else:
        print("   未检测到麦克风设备")


def main():
    parser = argparse.ArgumentParser(
        description="列出系统中的串口、摄像头和麦克风设备",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python list_devices.py                    # 列出所有设备
  python list_devices.py --test-camera      # 列出并测试摄像头
  python list_devices.py --test-mic         # 测试默认麦克风录音
  python list_devices.py --test-mic --mic-index 1  # 测试指定麦克风
        """
    )
    parser.add_argument("--test-camera", action="store_true", help="测试摄像头是否可以正常打开")
    parser.add_argument("--test-mic", action="store_true", help="测试麦克风录音")
    parser.add_argument("--mic-index", type=int, default=None, help="指定要测试的麦克风索引")
    parser.add_argument("--mic-duration", type=float, default=3.0, help="录音时长（秒）")
    
    args = parser.parse_args()
    
    print("\n🔍 HomeBot 设备信息查看工具")
    print("   用于快速了解系统设备信息并修改配置\n")
    
    # 串口设备
    print_section("📟 串口设备 (Serial Ports)")
    serial_devices = list_serial_ports()
    
    # 摄像头设备
    print_section("📷 摄像头设备 (Cameras)")
    camera_devices = list_cameras(test_open=args.test_camera)
    
    # 麦克风设备
    print_section("🎤 麦克风设备 (Microphones)")
    mic_devices = list_microphones()
    
    # 测试麦克风
    if args.test_mic:
        print_section("🎙️  麦克风录音测试")
        test_microphone(device_index=args.mic_index, duration=args.mic_duration)
    
    # 配置摘要
    print_summary(serial_devices, camera_devices, mic_devices)
    
    print("\n" + "=" * 60)
    print(" 完成！请根据以上信息修改配置文件。")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    main()
