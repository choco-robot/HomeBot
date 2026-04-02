"""
机械臂3D运动学测试工具

用于验证和调试5DOF机械臂的3DOF逆运动学控制

使用方法:
    cd software
    python tools/test_kinematics.py

功能:
    1. 正逆运动学一致性验证
    2. 工作空间边界测试
    3. 末端方向保持验证
    4. 可视化工作空间（生成点云图）
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import math
import numpy as np
from hal.arm.Kinematics import Arm3DKinematics, ArmKinematics
from common.logging import get_logger

logger = get_logger(__name__)


def test_forward_inverse_consistency(kin: Arm3DKinematics, num_tests: int = 10):
    """
    测试正逆运动学一致性
    
    随机生成关节角度 -> 正运动学 -> 逆运动学 -> 验证位置精度
    
    注意：逆运动学可能存在多解，因此只验证位置精度，不验证角度一致性
    """
    logger.info("=" * 60)
    logger.info("测试1: 正逆运动学一致性验证")
    logger.info("=" * 60)
    
    passed = 0
    failed = 0
    
    # 从kin获取关节限制
    limits = kin.joint_limits
    base_min, base_max = limits.get('base', (-90, 90))
    shoulder_min, shoulder_max = limits.get('shoulder', (0, 180))
    elbow_min, elbow_max = limits.get('elbow', (0, 180))
    
    for i in range(num_tests):
        # 随机生成关节角度（在限制范围内，且确保 r >= 0）
        base = np.random.uniform(base_min, base_max)
        
        # 限制 shoulder 和 elbow 范围以确保 r >= 0
        # 根据测试，shoulder 在 0~60° 且 elbow 不太大时 r 通常为正
        max_attempts = 100
        for attempt in range(max_attempts):
            shoulder = np.random.uniform(max(0, shoulder_min), min(60, shoulder_max))
            elbow = np.random.uniform(max(0, elbow_min), min(120, elbow_max))
            
            # 检查是否产生正的 r
            r_planar, _ = kin.planar_kin.forward_kinematics(shoulder, elbow)
            if r_planar >= 0:
                break
        else:
            # 找不到有效角度，使用安全默认值
            shoulder = 30.0
            elbow = 45.0
        
        # 正运动学
        x, y, z = kin.forward_kinematics(base, shoulder, elbow)
        
        # 获取所有逆运动学解
        solutions = kin.inverse_kinematics_all(x, y, z)
        
        if not solutions:
            logger.warning(f"测试{i+1}: 逆运动学无解 (base={base:.1f}, shoulder={shoulder:.1f}, elbow={elbow:.1f})")
            failed += 1
            continue
        
        # 检查原始角度是否在解集中（考虑数值误差）
        def angle_diff(a, b):
            diff = abs(a - b)
            return min(diff, 360 - diff)
        
        found_match = False
        best_pos_error = float('inf')
        
        for sol_base, sol_shoulder, sol_elbow in solutions:
            # 验证这个解的正运动学
            x_check, y_check, z_check = kin.forward_kinematics(sol_base, sol_shoulder, sol_elbow)
            pos_error = math.sqrt((x-x_check)**2 + (y-y_check)**2 + (z-z_check)**2)
            best_pos_error = min(best_pos_error, pos_error)
            
            # 检查是否匹配原始角度（允许一定误差）
            base_err = angle_diff(base, sol_base)
            # base允许180度歧义（因为 r = sqrt(x²+y²) 是正值，base和base+180可能都有效）
            base_err = min(base_err, abs(base_err - 180))
            shoulder_err = angle_diff(shoulder, sol_shoulder)
            elbow_err = angle_diff(elbow, sol_elbow)
            
            # 如果找到匹配的角度解，标记为找到
            if base_err < 5.0 and shoulder_err < 5.0 and elbow_err < 5.0:
                found_match = True
        
        # 关键指标：位置精度必须高
        if best_pos_error < 0.01:  # 0.01mm容差
            passed += 1
            match_info = "(找到角度匹配)" if found_match else "(多解，位置正确)"
            logger.info(f"✓ 测试{i+1}: 通过 {match_info} (位置误差={best_pos_error:.6f}mm)")
        else:
            failed += 1
            logger.error(f"✗ 测试{i+1}: 失败 - 位置精度过低")
            logger.error(f"  原始: base={base:.1f}°, shoulder={shoulder:.1f}°, elbow={elbow:.1f}°")
            logger.error(f"  正向: x={x:.1f}, y={y:.1f}, z={z:.1f}")
            logger.error(f"  最佳位置误差: {best_pos_error:.6f}mm")
    
    logger.info(f"\n结果: {passed}/{num_tests} 通过, {failed}/{num_tests} 失败")
    return failed == 0


def test_workspace_boundaries(kin: Arm3DKinematics):
    """
    测试工作空间边界
    """
    logger.info("\n" + "=" * 60)
    logger.info("测试2: 工作空间边界测试")
    logger.info("=" * 60)
    
    workspace = kin.get_workspace()
    r_min, r_max = workspace['r_min'], workspace['r_max']
    
    logger.info(f"工作空间: r∈[{r_min:.1f}, {r_max:.1f}]mm")
    
    # 测试点
    test_cases = [
        # (x, y, z, 描述, 期望是否可达)
        (200, 0, 100, "典型前伸位置", True),
        (50, 0, 150, "近距离高位", True),
        (r_max + 50, 0, 0, "超出最大距离", False),
        (r_min - 10, 0, 100, "小于最小距离", False),
        (0, 200, 100, "左侧位置", True),
        (0, -200, 100, "右侧位置", True),
        (150, 150, 100, "左前方", True),
        (100, 0, -50, "地面以下", False),
        (kin.L1 + kin.L2, 0, 0, "最大水平前伸", True),
        (0, 0, kin.L1 + kin.L2, "正上方最高点", True),
    ]
    
    passed = 0
    for x, y, z, desc, expected_reachable in test_cases:
        reachable = kin.is_reachable(x, y, z)
        result = kin.inverse_kinematics(x, y, z)
        
        has_solution = result is not None
        
        if has_solution == expected_reachable:
            passed += 1
            status = "✓"
        else:
            status = "✗"
        
        logger.info(f"{status} {desc}: ({x}, {y}, {z}) -> {'可达' if has_solution else '不可达'} (期望: {'可达' if expected_reachable else '不可达'})")
    
    logger.info(f"\n结果: {passed}/{len(test_cases)} 通过")
    return passed == len(test_cases)


def test_orientation_keeping(kin: Arm3DKinematics):
    """
    测试末端方向保持
    
    验证 wrist_flex 和 wrist_roll 计算是否正确保持末端方向
    """
    logger.info("\n" + "=" * 60)
    logger.info("测试3: 末端方向保持验证")
    logger.info("=" * 60)
    
    # 测试手腕角度计算
    test_cases = [
        # (shoulder, elbow, target_orientation, 期望wrist_flex)
        (30, 45, 0, -75),      # 保持水平
        (45, 45, 0, -90),      # 保持水平
        (0, 90, 0, -90),       # 保持水平
        (30, 45, -30, -105),   # 向下倾斜30度
    ]
    
    passed = 0
    for shoulder, elbow, target, expected in test_cases:
        wrist = kin.compute_wrist_flex(shoulder, elbow, target)
        error = abs(wrist - expected)
        
        if error < 1.0:
            passed += 1
            status = "✓"
        else:
            status = "✗"
        
        logger.info(f"{status} shoulder={shoulder}°, elbow={elbow}°, target={target}° -> wrist_flex={wrist:.1f}° (期望: {expected}°)")
    
    # 测试完整求解
    logger.info("\n完整位置求解测试:")
    result = kin.solve_for_position(x=200, y=50, z=100, 
                                    target_orientation=0.0, 
                                    target_yaw=0.0)
    if result:
        logger.info(f"目标位置 (200, 50, 100):")
        for joint, angle in result.items():
            logger.info(f"  {joint}: {angle:.1f}°")
        
        # 验证
        x_check, y_check, z_check = kin.forward_kinematics(
            result['base'], result['shoulder'], result['elbow']
        )
        error = math.sqrt((200-x_check)**2 + (50-y_check)**2 + (100-z_check)**2)
        logger.info(f"  位置验证误差: {error:.3f}mm")
        if error < 1.0:
            passed += 1
    
    logger.info(f"\n结果: {passed}/{len(test_cases)+1} 通过")
    return passed >= len(test_cases)


def generate_workspace_pointcloud(kin: Arm3DKinematics, filename: str = "workspace.txt"):
    """
    生成工作空间点云数据
    
    用于外部可视化分析
    """
    logger.info("\n" + "=" * 60)
    logger.info("生成工作空间点云")
    logger.info("=" * 60)
    
    points = []
    
    # 采样关节空间
    base_range = np.linspace(-90, 90, 20)
    shoulder_range = np.linspace(0, 120, 15)
    elbow_range = np.linspace(0, 150, 15)
    
    for base in base_range:
        for shoulder in shoulder_range:
            for elbow in elbow_range:
                x, y, z = kin.forward_kinematics(base, shoulder, elbow)
                points.append((x, y, z))
    
    # 保存到文件
    output_path = Path(filename)
    with open(output_path, 'w') as f:
        f.write("# x y z\n")
        for x, y, z in points:
            f.write(f"{x:.2f} {y:.2f} {z:.2f}\n")
    
    logger.info(f"已保存 {len(points)} 个点到 {output_path}")
    logger.info("可用matplotlib或cloudcompare可视化")
    
    # 输出统计
    points = np.array(points)
    logger.info(f"\n工作空间范围:")
    logger.info(f"  X: [{points[:,0].min():.1f}, {points[:,0].max():.1f}] mm")
    logger.info(f"  Y: [{points[:,1].min():.1f}, {points[:,1].max():.1f}] mm")
    logger.info(f"  Z: [{points[:,2].min():.1f}, {points[:,2].max():.1f}] mm")


def interactive_test(kin: Arm3DKinematics):
    """
    交互式测试
    
    用户输入目标位置，输出关节角度
    """
    logger.info("\n" + "=" * 60)
    logger.info("交互式逆运动学测试")
    logger.info("=" * 60)
    logger.info("输入目标位置 (x y z)，或 'q' 退出")
    logger.info("示例: 200 0 100")
    
    while True:
        try:
            user_input = input("\n目标位置 (x y z): ").strip()
            if user_input.lower() == 'q':
                break
            
            parts = user_input.split()
            if len(parts) != 3:
                print("请输入3个数字: x y z")
                continue
            
            x, y, z = map(float, parts)
            
            # 求解
            result = kin.solve_for_position(x, y, z)
            
            if result is None:
                print(f"❌ 位置 ({x}, {y}, {z}) 不可达")
            else:
                print(f"✓ 位置 ({x}, {y}, {z}):")
                print(f"  base:       {result['base']:.1f}°")
                print(f"  shoulder:   {result['shoulder']:.1f}°")
                print(f"  elbow:      {result['elbow']:.1f}°")
                print(f"  wrist_flex: {result['wrist_flex']:.1f}°")
                print(f"  wrist_roll: {result['wrist_roll']:.1f}°")
                
                # 验证
                x_check, y_check, z_check = kin.forward_kinematics(
                    result['base'], result['shoulder'], result['elbow']
                )
                error = math.sqrt((x-x_check)**2 + (y-y_check)**2 + (z-z_check)**2)
                print(f"  验证误差: {error:.3f}mm")
                
        except ValueError as e:
            print(f"输入错误: {e}")
        except KeyboardInterrupt:
            break
    
    print("退出交互模式")


def main():
    parser = argparse.ArgumentParser(
        description="机械臂3D运动学测试工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 运行所有测试
  python tools/test_kinematics.py
  
  # 仅运行特定测试
  python tools/test_kinematics.py --test consistency
  
  # 生工作空间点云
  python tools/test_kinematics.py --pointcloud
  
  # 交互式测试
  python tools/test_kinematics.py --interactive
        """
    )
    
    parser.add_argument("--test", type=str, choices=['consistency', 'workspace', 'orientation', 'all'],
                        default='all', help="选择测试类型 (默认: all)")
    parser.add_argument("--num-tests", type=int, default=10,
                        help="一致性测试的随机测试次数 (默认: 10)")
    parser.add_argument("--pointcloud", action="store_true",
                        help="生成工作空间点云文件")
    parser.add_argument("--interactive", "-i", action="store_true",
                        help="交互式测试模式")
    parser.add_argument("--L1", type=float, default=215.0,
                        help="大臂长度 mm (默认: 215)")
    parser.add_argument("--L2", type=float, default=230.0,
                        help="小臂长度 mm (默认: 230)")
    
    args = parser.parse_args()
    
    # 初始化运动学
    kin = Arm3DKinematics(L1=args.L1, L2=args.L2)
    logger.info(f"机械臂参数: L1={args.L1}mm, L2={args.L2}mm")
    
    # 交互模式
    if args.interactive:
        interactive_test(kin)
        return
    
    # 点云模式
    if args.pointcloud:
        generate_workspace_pointcloud(kin)
        return
    
    # 运行测试
    all_passed = True
    
    if args.test in ['consistency', 'all']:
        if not test_forward_inverse_consistency(kin, args.num_tests):
            all_passed = False
    
    if args.test in ['workspace', 'all']:
        if not test_workspace_boundaries(kin):
            all_passed = False
    
    if args.test in ['orientation', 'all']:
        if not test_orientation_keeping(kin):
            all_passed = False
    
    logger.info("\n" + "=" * 60)
    if all_passed:
        logger.info("✓ 所有测试通过!")
    else:
        logger.error("✗ 部分测试失败")
    logger.info("=" * 60)
    
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
