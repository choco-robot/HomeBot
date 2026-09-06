# -*- coding: utf-8 -*-
"""策略推理部署（服务级）

在 GPU 机器或机器人本机运行训练好的 ACT / SmolVLA 策略，通过 HomeBot 现有
服务通道下发动作（经仲裁器，source="auto"，保留急停与超时保护）：

    图像:  vision_service PUB (5560)  → VisionSubscriber
    关节:  arm_service REP (5557) query 读取当前角度
    动作:  arm_service REP (5557) 下发 / chassis_service REP (5556) 下发速度

policy 加载基于 lerobot 当前主线 API（PreTrainedConfig + get_policy_class +
make_pre_post_processors），不同 lerobot 版本可能需微调 load_policy()。
"""
import json
import logging
import time
from typing import Any, Dict, Optional, Tuple

from common.bus import ZMQRequestClient
from common.logging import get_logger
from .joint_map import home_to_lerobot_state, lerobot_to_home_action

logger = get_logger(__name__)


def load_policy(policy_path: str, device: str = "cuda"):
    """加载训练好的策略及其前/后处理管线

    Returns:
        (policy, preprocessor, postprocessor, policy_cfg)
    """
    try:
        import torch  # noqa: F401
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors
    except ImportError as e:
        raise ImportError(
            "policy_runner 需要安装 torch 和 lerobot（建议独立环境，见 "
            "requirements-lerobot.txt）"
        ) from e

    policy_cfg = PreTrainedConfig.from_pretrained(policy_path)
    policy_cfg.pretrained_path = policy_path
    policy_cls = get_policy_class(policy_cfg.type)
    policy = policy_cls.from_pretrained(policy_path)
    policy.to(device)
    policy.eval()

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg, pretrained_path=policy_path
    )
    logger.info("策略已加载: %s (type=%s, device=%s)", policy_path, policy_cfg.type, device)
    return policy, preprocessor, postprocessor, policy_cfg


def detect_image_key(policy_cfg) -> str:
    """从策略配置中推断期望的图像特征名（去掉 observation.images. 前缀）"""
    image_keys = [
        k.removeprefix("observation.images.")
        for k in getattr(policy_cfg, "input_features", {})
        if k.startswith("observation.images.")
    ]
    if len(image_keys) == 1:
        return image_keys[0]
    if not image_keys:
        raise ValueError("策略配置中没有图像输入特征")
    raise ValueError(f"策略有多个图像输入 {image_keys}，请用 --camera-key 指定其一")


class PolicyRunner:
    """策略推理循环：观测 → 推理 → 经仲裁器下发"""

    def __init__(self, robot_host: str = "localhost", camera_key: Optional[str] = None,
                 enable_chassis: bool = False, timeout_ms: int = 500):
        from configs import get_config

        config = get_config()
        self._arm = ZMQRequestClient(
            config.zmq.arm_service_addr.replace("*", robot_host), timeout_ms=timeout_ms
        )
        self._chassis = ZMQRequestClient(
            config.zmq.chassis_service_addr.replace("*", robot_host), timeout_ms=timeout_ms
        )

        from services.vision_service.vision import VisionSubscriber

        self._vision = VisionSubscriber(config.zmq.vision_pub_addr.replace("*", robot_host))
        self._camera_key = camera_key
        self._enable_chassis = enable_chassis
        # 底盘无回读，用最近一次下发的速度作为观测
        self._last_base_vel = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}

    def _read_arm_state(self) -> Dict[str, float]:
        """从 arm_service 查询当前关节角度，转换为 LeRobot 观测键值"""
        resp = self._arm.request({
            "source": "auto", "joints": {}, "query": True, "timestamp": time.time(),
        })
        if resp is None or not resp.get("success"):
            raise RuntimeError(f"机械臂状态查询失败: {resp}")
        return home_to_lerobot_state(resp["joint_states"])

    def _send_arm_action(self, action: Dict[str, float]) -> None:
        """LeRobot 动作 → HomeBot 关节角度，经仲裁器下发"""
        joints = lerobot_to_home_action(action)
        resp = self._arm.request({
            "source": "auto", "joints": joints, "priority": 3, "timestamp": time.time(),
        })
        if resp is None or not resp.get("success"):
            logger.warning("机械臂动作被拒绝: %s", resp)

    def _send_base_action(self, action: Dict[str, float]) -> None:
        """底盘速度动作（theta.vel 为 deg/s，底盘服务 omega 为 rad/s）"""
        import math

        vx = action.get("x.vel", 0.0)
        vy = action.get("y.vel", 0.0)
        omega = math.radians(action.get("theta.vel", 0.0))
        self._last_base_vel = {"x.vel": vx, "y.vel": vy, "theta.vel": action.get("theta.vel", 0.0)}
        resp = self._chassis.request({
            "source": "auto", "vx": vx, "vy": vy, "vz": omega,
            "priority": 3, "timestamp": time.time(),
        })
        if resp is None or not resp.get("success"):
            logger.warning("底盘动作被拒绝: %s", resp)

    def get_observation(self) -> Dict[str, Any]:
        obs = self._read_arm_state()
        if self._enable_chassis:
            obs.update(self._last_base_vel)
        frame_id, frame = self._vision.read_frame()
        if frame is None:
            raise RuntimeError("未收到图像帧，请确认 vision_service 已启动")
        obs[self._camera_key] = frame
        return obs

    def stop_robot(self) -> None:
        """停止底盘（安全收尾）"""
        if self._enable_chassis:
            self._chassis.request({
                "source": "auto", "vx": 0.0, "vy": 0.0, "vz": 0.0,
                "priority": 3, "timestamp": time.time(),
            })

    def close(self) -> None:
        self._vision.stop()
        self._arm.close()
        self._chassis.close()


def run_policy(policy_path: str, robot_host: str = "localhost", fps: float = 30.0,
               task: Optional[str] = None, camera_key: Optional[str] = None,
               enable_chassis: bool = False, device: str = "cuda",
               dry_run: bool = False) -> None:
    """运行策略推理主循环

    Args:
        policy_path: 训练输出目录或 HF hub id
        robot_host: 机器人地址（服务地址中的 * 会替换为该值）
        fps: 控制频率
        task: 语言指令（SmolVLA 等 VLA 模型需要）
        camera_key: 图像键名（须与录制数据一致），默认从策略配置自动推断
        enable_chassis: 是否启用底盘动作（动作空间含 x.vel/y.vel/theta.vel 的模型）
        device: 推理设备（cuda / cpu）
        dry_run: 只组装观测并打印动作，不下发到机器人
    """
    policy, preprocessor, postprocessor, policy_cfg = load_policy(policy_path, device)
    if camera_key is None:
        camera_key = detect_image_key(policy_cfg)

    runner = PolicyRunner(robot_host=robot_host, camera_key=camera_key,
                          enable_chassis=enable_chassis)
    runner._vision.start()

    logger.info("策略运行中 (%.0f Hz, camera_key=%s, chassis=%s)，Ctrl+C 退出",
                fps, camera_key, enable_chassis)
    period = 1.0 / fps
    try:
        while True:
            t0 = time.perf_counter()

            obs = runner.get_observation()
            if task is not None:
                obs["task"] = task

            batch = preprocessor(obs)
            action = policy.select_action(batch)
            action = postprocessor(action)

            # action 可能是 tensor 或 dict，统一为 {key: float}
            if not isinstance(action, dict):
                raise TypeError(f"未预期的动作类型: {type(action)}")
            action = {k: float(v) for k, v in action.items()}

            if dry_run:
                print(json.dumps(action, ensure_ascii=False))
            else:
                runner._send_arm_action(action)
                if enable_chassis:
                    runner._send_base_action(action)

            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        logger.info("退出，停止机器人")
    finally:
        if not dry_run:
            runner.stop_robot()
        runner.close()
