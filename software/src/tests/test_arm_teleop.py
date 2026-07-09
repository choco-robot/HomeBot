"""
机械臂 WLAN 遥操作应用单元测试

运行方式:
    cd software/src
    python -m tests.test_arm_teleop
"""
import os
import tempfile
import threading
import time
import unittest
from unittest.mock import patch, MagicMock

import zmq

from configs import ArmTeleopConfig, ArmConfig
from hal.arm.driver import ArmConfig as HalArmConfig
from applications.arm_teleop.app import ArmTeleopApp, build_master_arm_config
from applications.arm_teleop.slave_client import SlaveArmClient
from applications.arm_teleop.recorder import TrajectoryRecorder, TrajectoryPlayer


class TestBuildMasterConfig(unittest.TestCase):
    """测试从全局配置构建 HAL 主臂配置"""

    def test_joint_ids_mapping(self):
        arm_cfg = ArmConfig(
            base_id=1,
            shoulder_id=2,
            elbow_id=3,
            wrist_flex_id=4,
            wrist_roll_id=5,
            gripper_id=6,
        )
        hal_cfg = build_master_arm_config(arm_cfg)
        self.assertEqual(hal_cfg.joint_ids["base"], 1)
        self.assertEqual(hal_cfg.joint_ids["gripper"], 6)
        self.assertEqual(hal_cfg.port, arm_cfg.serial_port)


class TestMappingAndClamping(unittest.TestCase):
    """测试关节映射与限幅"""

    def _make_app(self, mapping=None):
        teleop_cfg = ArmTeleopConfig(
            slave_arm_addr="tcp://127.0.0.1:5557",
            joint_mapping=mapping or {
                "base": ("base", 1),
                "shoulder": ("shoulder", 1),
                "elbow": ("elbow", 1),
                "wrist_flex": ("wrist_flex", 1),
                "wrist_roll": ("wrist_roll", 1),
                "gripper": ("gripper", 1),
            },
        )
        hal_cfg = HalArmConfig(
            joint_ids={
                "base": 1, "shoulder": 2, "elbow": 3,
                "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
            },
            joint_limits={
                "base": (-180, 180),
                "shoulder": (0, 180),
                "elbow": (0, 180),
                "wrist_flex": (-90, 90),
                "wrist_roll": (-180, 180),
                "gripper": (0, 90),
            },
        )

        with patch("applications.arm_teleop.app.MasterArmReader") as MockReader, \
             patch("applications.arm_teleop.app.SlaveArmClient") as MockClient:
            MockReader.return_value = MagicMock()
            MockClient.return_value = MagicMock()
            app = ArmTeleopApp(teleop_cfg, hal_cfg)
            return app

    def test_identity_mapping(self):
        app = self._make_app()
        master = {
            "base": 10, "shoulder": 45, "elbow": 90,
            "wrist_flex": 0, "wrist_roll": -30, "gripper": 45,
        }
        result = app._map_and_clamp(master)
        self.assertEqual(result, master)

    def test_sign_inversion(self):
        app = self._make_app(mapping={
            "base": ("base", -1),
            "elbow": ("elbow", 1),
        })
        master = {"base": 45, "elbow": 90}
        result = app._map_and_clamp(master)
        self.assertEqual(result["base"], -45)
        self.assertEqual(result["elbow"], 90)

    def test_clamping(self):
        app = self._make_app()
        master = {"shoulder": 200, "gripper": -10}
        result = app._map_and_clamp(master)
        self.assertEqual(result["shoulder"], 180)
        self.assertEqual(result["gripper"], 0)


class TestSpeedComputation(unittest.TestCase):
    """测试速度自适应计算"""

    def test_speed_within_bounds(self):
        teleop_cfg = ArmTeleopConfig(
            default_speed=500,
            min_speed=100,
            max_speed=2000,
            speed_scale=15.0,
            send_rate=30.0,
        )
        hal_cfg = HalArmConfig()
        with patch("applications.arm_teleop.app.MasterArmReader") as MockReader, \
             patch("applications.arm_teleop.app.SlaveArmClient") as MockClient:
            MockReader.return_value = MagicMock()
            MockClient.return_value = MagicMock()
            app = ArmTeleopApp(teleop_cfg, hal_cfg)
            app._last_sent_angles = {"base": 0}
            speed = app._compute_speed({"base": 10})
            self.assertGreaterEqual(speed, teleop_cfg.min_speed)
            self.assertLessEqual(speed, teleop_cfg.max_speed)

    def test_fixed_speed_when_scale_zero(self):
        teleop_cfg = ArmTeleopConfig(
            default_speed=700,
            speed_scale=0.0,
            send_rate=30.0,
        )
        hal_cfg = HalArmConfig()
        with patch("applications.arm_teleop.app.MasterArmReader") as MockReader, \
             patch("applications.arm_teleop.app.SlaveArmClient") as MockClient:
            MockReader.return_value = MagicMock()
            MockClient.return_value = MagicMock()
            app = ArmTeleopApp(teleop_cfg, hal_cfg)
            app._last_sent_angles = {"base": 0}
            speed = app._compute_speed({"base": 20})
            self.assertEqual(speed, 700)


class TestSlaveClientQuery(unittest.TestCase):
    """测试从臂客户端查询功能"""

    def test_query(self):
        addr = "inproc://test_slave_query"
        context = zmq.Context.instance()
        rep = context.socket(zmq.REP)
        rep.bind(addr)

        states = {"base": 10.0, "shoulder": 20.0}
        stop_event = threading.Event()

        def server():
            while not stop_event.is_set():
                try:
                    req = rep.recv_json(flags=zmq.NOBLOCK)
                    if req.get("query"):
                        rep.send_json({
                            "success": True,
                            "message": "查询成功",
                            "current_owner": "teleop",
                            "current_priority": 3,
                            "joint_states": states,
                        })
                        stop_event.set()
                except zmq.Again:
                    time.sleep(0.01)

        t = threading.Thread(target=server, daemon=True)
        t.start()

        client = SlaveArmClient(addr, timeout_ms=500)
        result = client.send_query()
        self.assertEqual(result, states)

        client.close()
        rep.close()
        stop_event.set()


class TestPlaybackLifecycle(unittest.TestCase):
    """测试回放生命周期"""

    def test_pending_playback_started_in_run(self):
        """CLI 指定的回放应在 run() 启动后再真正开始"""
        teleop_cfg = ArmTeleopConfig()
        hal_cfg = HalArmConfig()

        with patch("applications.arm_teleop.app.MasterArmReader") as MockReader, \
             patch("applications.arm_teleop.app.SlaveArmClient") as MockClient, \
             patch.object(ArmTeleopApp, "_main_loop", lambda self: None), \
             patch.object(ArmTeleopApp, "stop", lambda self: None):
            app = ArmTeleopApp(teleop_cfg, hal_cfg)
            app._pending_playback = ("test.json", 1.0, 2)
            with patch.object(app, "start_playback") as mock_start:
                app.run()
                mock_start.assert_called_once_with("test.json", 1.0, 2)


class TestTrajectoryRecorder(unittest.TestCase):
    """测试轨迹录制与加载"""

    def test_record_save_and_load(self):
        recorder = TrajectoryRecorder()
        recorder.start()
        recorder.record({"base": 0.0, "shoulder": 45.0})
        recorder.record({"base": 1.0, "shoulder": 46.0})
        recorder.stop()

        with tempfile.NamedTemporaryFile("w+", suffix=".json", delete=False) as f:
            path = f.name

        try:
            self.assertTrue(recorder.save(path))
            loaded = TrajectoryRecorder.load(path)
            self.assertEqual(len(loaded), 2)
            self.assertIn("base", loaded[0]["angles"])
            self.assertEqual(loaded[0]["angles"]["shoulder"], 45.0)
        finally:
            os.unlink(path)


class TestTrajectoryPlayer(unittest.TestCase):
    """测试轨迹回放"""

    def _make_player(self):
        app = MagicMock()
        app.cfg.default_speed = 100
        app.cfg.min_speed = 10
        app.cfg.max_speed = 1000
        app.cfg.speed_scale = 10.0
        app._running = True
        app.client = MagicMock()
        app.client.send_joint_angles.return_value = True
        player = TrajectoryPlayer(app)
        return player, app

    @patch("applications.arm_teleop.recorder.time.sleep")
    def test_play_once(self, mock_sleep):
        player, app = self._make_player()
        app._map_and_clamp = lambda a: a
        frames = [
            {"t": 0.000, "angles": {"base": 0.0}},
            {"t": 0.010, "angles": {"base": 5.0}},
            {"t": 0.020, "angles": {"base": 10.0}},
        ]
        player._play_once(frames, speed=1.0)
        self.assertEqual(app.client.send_joint_angles.call_count, 3)

    @patch("applications.arm_teleop.recorder.time.sleep")
    def test_play_loop(self, mock_sleep):
        player, app = self._make_player()
        app._map_and_clamp = lambda a: a
        frames = [
            {"t": 0.000, "angles": {"base": 0.0}},
            {"t": 0.010, "angles": {"base": 5.0}},
        ]
        player.play(frames, speed=1.0, loop=2)
        self.assertEqual(app.client.send_joint_angles.call_count, 4)


if __name__ == "__main__":
    unittest.main()
