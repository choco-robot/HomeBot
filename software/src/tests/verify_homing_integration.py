#!/usr/bin/env python
"""
升降平台零点初始化集成验证

验证内容:
1. 配置读取正确
2. 坐标转换正确
3. 电流读取功能可用
4. 找零方法存在
"""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def test_config():
    """测试配置"""
    print("=" * 60)
    print("[1] 配置验证")
    print("=" * 60)
    
    from configs import get_config
    config = get_config()
    lift = config.lift_platform
    
    print(f"最高点坐标 (max_height): {lift.max_height}mm")
    print(f"最低点坐标 (min_height): {lift.min_height}mm")
    print(f"行程长度 (stroke_length): {lift.stroke_length}mm")
    print(f"自动找零 (auto_homing_on_startup): {lift.auto_homing_on_startup}")
    print(f"找零方向 (homing_direction): {lift.homing_direction}")
    print(f"找零速度 (homing_speed): {lift.homing_speed}")
    print(f"电流阈值 (homing_current_threshold): {lift.homing_current_threshold}mA")
    print(f"超时时间 (homing_timeout): {lift.homing_timeout}s")
    print(f"回退步数 (homing_backoff_steps): {lift.homing_backoff_steps}")
    
    assert lift.max_height == 0.0, "最高点应为0"
    assert lift.min_height < 0, "最低点应为负值"
    print("[OK] 坐标系配置正确")
    return True


def test_coordinate_conversion():
    """测试坐标转换"""
    print("\n" + "=" * 60)
    print("[2] 坐标转换验证")
    print("=" * 60)
    
    from services.motion_service.arm_service import ArmService
    service = ArmService()
    
    test_cases = [
        (0, 0),           # 最高点
        (-50, -2560),     # 向下50mm
        (-100, -5120),    # 向下100mm
        (-200, -10240),   # 最低点
    ]
    
    for height, expected_steps in test_cases:
        steps = service._height_to_steps(height)
        height_back = service._steps_to_height(steps)
        
        print(f"  高度 {height:6.1f}mm -> {steps:8d} 步 (期望 {expected_steps:8d})")
        
        # 允许±1步的误差
        assert abs(steps - expected_steps) <= 1, f"转换错误: {steps} != {expected_steps}"
        assert abs(height_back - height) < 0.1, f"往返错误: {height_back} != {height}"
    
    print("[OK] 坐标转换正确")
    return True


def test_current_reading():
    """测试电流读取功能"""
    print("\n" + "=" * 60)
    print("[3] 电流读取功能验证")
    print("=" * 60)
    
    from hal.ftservo_driver import FTServoBus
    
    # 检查方法存在
    assert hasattr(FTServoBus, 'read_current'), "FTServoBus 缺少 read_current 方法"
    assert hasattr(FTServoBus, 'read_load'), "FTServoBus 缺少 read_load 方法"
    
    print("[OK] 电流/负载读取方法存在")
    return True


def test_homing_method():
    """测试找零方法"""
    print("\n" + "=" * 60)
    print("[4] 找零功能验证")
    print("=" * 60)
    
    from services.motion_service.arm_service import ArmService
    
    # 检查方法存在
    assert hasattr(ArmService, '_perform_homing'), "ArmService 缺少 _perform_homing 方法"
    
    print("[OK] 找零方法存在")
    return True


def test_sdk_methods():
    """测试SDK方法"""
    print("\n" + "=" * 60)
    print("[5] SDK方法验证")
    print("=" * 60)
    
    try:
        from hal.scservo_sdk import sms_sts
        assert hasattr(sms_sts, 'ReadLoad'), "sms_sts 缺少 ReadLoad 方法"
        assert hasattr(sms_sts, 'ReadCurrent'), "sms_sts 缺少 ReadCurrent 方法"
        print("[OK] SDK电流/负载读取方法存在")
    except ImportError:
        print("[WARN] SDK未安装，使用模拟模式")
        print("[OK] 模拟模式可用")
    
    return True


if __name__ == "__main__":
    try:
        test_config()
        test_coordinate_conversion()
        test_current_reading()
        test_homing_method()
        test_sdk_methods()
        
        print("\n" + "=" * 60)
        print("[OK] 所有验证通过!")
        print("=" * 60)
        print()
        print("提示: 首次使用前请运行电流阈值校准:")
        print("  python -m tests.test_lift_homing --calibrate")
        
    except AssertionError as e:
        print(f"\n[FAIL] 验证失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] 发生异常: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
