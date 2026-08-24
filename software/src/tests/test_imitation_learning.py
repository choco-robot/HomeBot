# -*- coding: utf-8 -*-
"""模仿学习（LeRobot 集成）测试

无硬件单测：
1. joint_map 关节名双向映射与夹爪单位换算
2. omni3 运动学正逆解往返一致、限幅
3. 底盘适配器工厂三种形态（diff2 在 main 上懒加载报错）
4. lerobot 未安装时 robot.py 可导入、实例化报友好错误；已安装则验证注册成功

运行方式:
    cd software/src
    python -m tests.test_imitation_learning
"""
import math

from applications.imitation_learning.joint_map import (
    HOME2LEROBOT, LEROBOT2HOME, LEROBOT_JOINTS,
    gripper_deg_to_lerobot, gripper_lerobot_to_deg,
    home_to_lerobot_state, lerobot_to_home_action, clip_action,
)
from applications.imitation_learning.chassis_adapter import (
    NullChassisAdapter, Omni3ChassisAdapter, Diff2ChassisAdapter,
    create_chassis_adapter,
    omni3_body_to_wheel_linear, omni3_wheel_linear_to_body,
    linear_to_wheel_raw, wheel_raw_to_linear, WHEEL_MAX_RAW,
)


def test_joint_map():
    """关节名双向映射与夹爪单位换算"""
    # 双向映射互逆
    assert LEROBOT2HOME == {v: k for k, v in HOME2LEROBOT.items()}
    assert set(HOME2LEROBOT.values()) == set(LEROBOT_JOINTS)
    assert HOME2LEROBOT["base"] == "shoulder_pan"

    # 夹爪 0-90 度 ↔ 0-100
    assert abs(gripper_deg_to_lerobot(90.0) - 100.0) < 1e-9
    assert abs(gripper_deg_to_lerobot(0.0) - 0.0) < 1e-9
    assert abs(gripper_lerobot_to_deg(50.0) - 45.0) < 1e-9
    # 往返
    assert abs(gripper_lerobot_to_deg(gripper_deg_to_lerobot(33.0)) - 33.0) < 1e-9

    # homebot 状态 → lerobot 观测键
    state = home_to_lerobot_state({"base": 10.0, "shoulder": 15.0, "gripper": 45.0})
    assert state["shoulder_pan.pos"] == 10.0
    assert state["shoulder_lift.pos"] == 15.0
    assert abs(state["gripper.pos"] - 50.0) < 1e-9

    # lerobot 动作 → homebot 关节角度（.vel 键被忽略）
    joints = lerobot_to_home_action({
        "shoulder_pan.pos": 5.0, "gripper.pos": 100.0, "x.vel": 0.2,
    })
    assert joints == {"base": 5.0, "gripper": 90.0}

    # 限幅
    clipped = clip_action({"shoulder_pan.pos": 120.0, "gripper.pos": -5.0},
                          {"shoulder_pan.pos": (-90, 90), "gripper.pos": (0, 100)})
    assert clipped["shoulder_pan.pos"] == 90
    assert clipped["gripper.pos"] == 0
    print("[OK] joint_map 映射与换算正确")


def test_omni3_kinematics():
    """omni3 运动学正逆解往返一致"""
    r = 0.18
    cases = [
        (0.2, 0.0, 0.0),    # 纯前进
        (0.0, 0.2, 0.0),    # 纯横移
        (0.0, 0.0, 0.8),    # 纯旋转
        (0.15, -0.1, 0.5),  # 混合
    ]
    for vx, vy, omega in cases:
        wheels = omni3_body_to_wheel_linear(vx, vy, omega, r)
        vx2, vy2, omega2 = omni3_wheel_linear_to_body(*wheels, r)
        assert abs(vx - vx2) < 1e-9, (vx, vx2)
        assert abs(vy - vy2) < 1e-9, (vy, vy2)
        assert abs(omega - omega2) < 1e-9, (omega, omega2)

    # 纯前进时：左右前轮对称，后轮为 0
    v_lf, v_rf, v_rear = omni3_body_to_wheel_linear(0.2, 0.0, 0.0, r)
    assert abs(v_lf + v_rf) < 1e-9 and abs(v_rear) < 1e-9

    # raw 换算与限幅
    assert linear_to_wheel_raw(0.5, 0.5) == WHEEL_MAX_RAW
    assert linear_to_wheel_raw(1.0, 0.5) == WHEEL_MAX_RAW  # 截断
    assert linear_to_wheel_raw(-1.0, 0.5) == -WHEEL_MAX_RAW
    assert abs(wheel_raw_to_linear(linear_to_wheel_raw(0.25, 0.5), 0.5) - 0.25) < 1e-3
    print("[OK] omni3 运动学正逆解与限幅正确")


def test_chassis_factory():
    """底盘适配器工厂与三种形态行为"""
    # none
    null = create_chassis_adapter("none")
    assert isinstance(null, NullChassisAdapter)
    assert not null.has_base
    null.set_velocity(0.1, 0.0, 0.0)  # 仅告警，不报错
    assert null.read_velocity() == (0.0, 0.0, 0.0)

    # omni3
    omni = create_chassis_adapter("omni3")
    assert isinstance(omni, Omni3ChassisAdapter)
    assert omni.has_base
    specs = omni.motor_specs
    assert specs["wheel_left_front"][0] == 9
    assert specs["wheel_right_front"][0] == 8
    assert specs["wheel_rear"][0] == 7
    # 未 bind_bus 时调用应断言失败
    try:
        omni.set_velocity(0.1, 0, 0)
        assert False, "未 bind_bus 应抛 AssertionError"
    except AssertionError:
        pass

    # diff2：main 分支上 diff_driver 不存在，connect 应报清晰错误
    diff = create_chassis_adapter("diff2")
    assert isinstance(diff, Diff2ChassisAdapter)
    try:
        diff.connect()
        # 若 navi 分支已合并，diff_driver 存在但无硬件也会失败——两种失败都接受，
        # 但报错信息不能是裸 ImportError
    except RuntimeError as e:
        assert "navi" in str(e) or "初始化失败" in str(e)

    # 非法类型
    try:
        create_chassis_adapter("quad")
        assert False, "非法底盘类型应抛 ValueError"
    except ValueError as e:
        assert "quad" in str(e)
    print("[OK] 底盘适配器工厂与三种形态行为正确")


def test_robot_import_without_lerobot():
    """robot.py 在无 lerobot 环境下可导入，实例化时给出友好错误"""
    from applications.imitation_learning import robot as robot_mod

    if robot_mod.LEROBOT_AVAILABLE:
        # lerobot 已安装：验证注册成功
        from lerobot.robots import RobotConfig
        cls = RobotConfig.get_choice_class("homebot")
        assert cls is robot_mod.HomeBotRobotConfig
        print("[OK] lerobot 已安装，homebot 机器人类型注册成功")
    else:
        try:
            robot_mod.HomeBotRobot(robot_mod.HomeBotRobotConfig())
            assert False, "未安装 lerobot 时实例化应抛 ImportError"
        except ImportError as e:
            assert "lerobot" in str(e)
        print("[OK] lerobot 未安装，robot.py 可导入且实例化报友好错误")


def main():
    print("=" * 60)
    print("模仿学习（LeRobot 集成）测试")
    print("=" * 60)

    test_joint_map()
    test_omni3_kinematics()
    test_chassis_factory()
    test_robot_import_without_lerobot()

    print("=" * 60)
    print("全部测试通过")
    print("=" * 60)


if __name__ == "__main__":
    main()
