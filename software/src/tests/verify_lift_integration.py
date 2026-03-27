#!/usr/bin/env python
"""
验证升降平台集成是否正确
"""
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_config():
    """测试配置"""
    print("=" * 60)
    print("测试配置")
    print("=" * 60)
    
    from configs import get_config
    config = get_config()
    
    print(f"舵机ID1: {config.lift_platform.servo_id_1}")
    print(f"舵机ID2: {config.lift_platform.servo_id_2}")
    print(f"导程: {config.lift_platform.lead}mm")
    print(f"传动比: {config.lift_platform.gear_ratio}")
    print(f"角度分辨率: {config.lift_platform.angle_resolution}")
    print(f"行程: {config.lift_platform.min_height}mm - {config.lift_platform.max_height}mm")
    
    # 验证高度到步数的转换
    test_height = 100.0
    lead = config.lift_platform.lead
    ratio = config.lift_platform.gear_ratio
    a_res = config.lift_platform.angle_resolution
    steps = int(-test_height * 4096 / lead / ratio / a_res)
    print(f"\n验证: 高度 {test_height}mm = {steps} 步")
    print(f"公式: -{test_height} * 4096 / {lead} / {ratio} / {a_res} = {steps}")
    
    return True


def test_dataclasses():
    """测试数据结构"""
    print("\n" + "=" * 60)
    print("测试数据结构")
    print("=" * 60)
    
    import time
    from services.motion_service.arm_service import ArmCommand, ArmResponse
    
    # 测试 ArmCommand
    cmd = ArmCommand(
        joint_angles={'base': 0},
        speed=1000,
        source='test',
        priority=1,
        timestamp=time.time(),
        query=False,
        lift_height=50.0
    )
    print(f"ArmCommand.lift_height: {cmd.lift_height}")
    
    # 测试 ArmResponse
    resp = ArmResponse(
        success=True,
        message='test',
        current_owner='test',
        current_priority=1,
        joint_states={'base': 0},
        lift_height=50.0
    )
    print(f"ArmResponse.lift_height: {resp.lift_height}")
    
    return True


def test_lift_conversion():
    """测试升降平台高度转换"""
    print("\n" + "=" * 60)
    print("测试高度/步数转换")
    print("=" * 60)
    
    from configs import get_config
    config = get_config()
    
    lead = config.lift_platform.lead
    ratio = config.lift_platform.gear_ratio
    a_res = config.lift_platform.angle_resolution
    
    test_heights = [0, 50, 100, 150, 200]
    
    for height in test_heights:
        steps = int(-height * 4096 / lead / ratio / a_res)
        # 反向计算
        height_back = -steps * lead * ratio * a_res / 4096
        print(f"  高度 {height:6.1f}mm -> {steps:8d} 步 -> {height_back:6.1f}mm")
    
    return True


def test_arbiter_client():
    """测试仲裁器客户端"""
    print("\n" + "=" * 60)
    print("测试仲裁器客户端")
    print("=" * 60)
    
    from services.motion_service.chassis_arbiter import ArmArbiterClient, ArmResponse
    
    # 检查 send_lift_command 方法是否存在
    assert hasattr(ArmArbiterClient, 'send_lift_command'), "ArmArbiterClient 缺少 send_lift_command 方法"
    print("[OK] ArmArbiterClient.send_lift_command 方法存在")
    
    # 检查 ArmResponse 是否有 lift_height 字段
    resp = ArmResponse(
        success=True,
        message='test',
        current_owner='test',
        current_priority=1,
        lift_height=100.0
    )
    assert hasattr(resp, 'lift_height'), "ArmResponse 缺少 lift_height 字段"
    print(f"[OK] ArmResponse.lift_height: {resp.lift_height}")
    
    return True


if __name__ == "__main__":
    try:
        test_config()
        test_dataclasses()
        test_lift_conversion()
        test_arbiter_client()
        
        print("\n" + "=" * 60)
        print("[OK] 所有测试通过!")
        print("=" * 60)
    except Exception as e:
        print(f"\n[FAIL] 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
