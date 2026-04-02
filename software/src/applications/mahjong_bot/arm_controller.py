"""
麻将机械臂控制器

整合：
- GameStateManager: 管理牌状态
- CoordinateTransformer: 坐标转换
- MotionPlanner: 运动规划
- ArmClient: 机械臂通信

提供高层次的出牌控制接口
"""

from typing import Optional, Dict, Callable
import time

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.logging import get_logger
from configs.config import get_config

from .game_state_manager import GameStateManager, MahjongTileState
from .coordinate_transformer import CoordinateTransformer
from .motion_planner import MotionPlanner
from .arm_client import ArmServiceClient

logger = get_logger(__name__)


class MahjongArmController:
    """
    麻将机械臂控制器
    
    高层次的控制接口，整合所有组件
    """
    
    def __init__(self, arm_service_addr: str = "tcp://localhost:5557"):
        """初始化控制器"""
        # 初始化各组件
        self.game_state = GameStateManager()
        self.coordinate_transformer = CoordinateTransformer()
        self.motion_planner = MotionPlanner()
        self.arm_client = ArmServiceClient(arm_service_addr)
        
        # 状态
        self.is_calibrated = self.coordinate_transformer.is_calibrated
        self.is_arm_connected = False
        
        logger.info("MahjongArmController 初始化完成")
        logger.info(f"  标定状态: {'已标定' if self.is_calibrated else '未标定'}")
    
    def connect_arm(self) -> bool:
        """连接机械臂服务"""
        if self.arm_client.connect():
            self.is_arm_connected = True
            logger.info("机械臂已连接")
            return True
        else:
            logger.error("机械臂连接失败")
            return False
    
    def disconnect_arm(self):
        """断开机械臂连接"""
        self.arm_client.disconnect()
        self.is_arm_connected = False
        logger.info("机械臂已断开")
    
    def update_hand_detection(self, detected_tiles: list):
        """
        更新手牌检测结果
        
        Args:
            detected_tiles: 检测到的牌列表
        """
        self.game_state.initialize_hand(detected_tiles)
        
        # 如果有坐标转换器，更新每张牌的物理位置
        if self.coordinate_transformer.is_calibrated:
            for tile in self.game_state.hand:
                arm_pos = self.coordinate_transformer.image_to_arm(
                    tile.image_x, tile.image_y, tile.table_z
                )
                if arm_pos:
                    arm_x, arm_y, arm_z = arm_pos
                    # 转换回牌桌坐标（因为game_state使用牌桌坐标）
                    table_pos = self.coordinate_transformer.arm_to_table(arm_x, arm_y, arm_z)
                    self.game_state.update_tile_position(
                        tile.tile_id, table_pos[0], table_pos[1], table_pos[2]
                    )
    
    def select_tile(self, tile_id: int) -> bool:
        """
        选中一张牌
        
        Args:
            tile_id: 牌ID
        
        Returns:
            是否成功
        """
        return self.game_state.select_tile(tile_id)
    
    def select_tile_by_image_position(self, image_x: float, image_y: float) -> Optional[int]:
        """
        根据图像位置选择最近的牌
        
        Args:
            image_x: 图像X坐标
            image_y: 图像Y坐标
        
        Returns:
            选中的牌ID，如果没有找到返回None
        """
        # 找到最近的牌
        min_dist = float('inf')
        nearest_tile = None
        
        for tile in self.game_state.hand:
            dist = ((tile.image_x - image_x)**2 + (tile.image_y - image_y)**2) ** 0.5
            if dist < min_dist:
                min_dist = dist
                nearest_tile = tile
        
        # 距离阈值（像素）
        if nearest_tile and min_dist < 100:
            if self.select_tile(nearest_tile.tile_id):
                return nearest_tile.tile_id
        
        return None
    
    def discard_selected_tile(self, 
                             on_step: Optional[Callable[[str, int, int], None]] = None) -> bool:
        """
        打出选中的牌
        
        完整的出牌流程：
        1. 获取选中牌的位置
        2. 规划运动序列
        3. 执行机械臂动作
        
        Args:
            on_step: 步骤回调函数 (step_name, current_step, total_steps)
        
        Returns:
            是否成功
        """
        # 获取选中的牌
        selected_tile = self.game_state.get_selected_tile()
        if selected_tile is None:
            logger.error("没有选中的牌")
            return False
        
        logger.info(f"开始出牌: [{selected_tile.tile_id}] {selected_tile.class_name}")
        
        # 获取牌的物理位置（转换为机械臂坐标）
        arm_pos = self.coordinate_transformer.image_to_arm(
            selected_tile.image_x, selected_tile.image_y, selected_tile.table_z
        )
        
        if arm_pos is None:
            # 如果没有标定，使用预设的牌槽位置
            logger.warning("未标定，使用预设牌槽位置")
            # 找到牌在哪个槽位
            for slot_idx, tile_id in self.game_state.slot_to_tile.items():
                if tile_id == selected_tile.tile_id:
                    table_pos = self.game_state.hand_region.get_slot_position(slot_idx)
                    arm_pos = self.coordinate_transformer.table_to_arm(
                        table_pos[0], table_pos[1], table_pos[2]
                    )
                    break
            else:
                logger.error("无法确定牌的位置")
                return False
        
        arm_x, arm_y, arm_z = arm_pos
        logger.info(f"目标位置: 机械臂坐标 ({arm_x:.1f}, {arm_y:.1f}, {arm_z:.1f})")
        
        # 检查可达性
        if not self.motion_planner.is_position_reachable(arm_x, arm_y, arm_z):
            logger.error(f"位置 ({arm_x}, {arm_y}, {arm_z}) 不可达")
            return False
        
        # 规划运动序列
        sequence = self.motion_planner.plan_pick_and_place(
            tile_x=arm_x,
            tile_y=arm_y,
            tile_height=arm_z
        )
        
        if not sequence:
            logger.error("运动规划失败")
            return False
        
        # 执行运动序列
        logger.info(f"执行运动序列: {len(sequence)} 步")
        
        def step_callback(step, idx):
            if on_step:
                on_step(step.name, idx + 1, len(sequence))
        
        success = self.motion_planner.execute_sequence(
            self.arm_client,
            on_step_start=step_callback
        )
        
        if success:
            # 更新游戏状态
            self.game_state.discard_selected()
            logger.info("出牌完成")
        else:
            logger.error("出牌失败")
        
        return success
    
    def move_to_rest(self) -> bool:
        """移动到休息位"""
        config = get_config()
        rest_pos = config.arm.rest_position
        
        logger.info("移动到休息位")
        return self.arm_client.move_joints(rest_pos, speed=500)
    
    def calibrate_from_points(self, calibration_points: list) -> float:
        """
        使用标定点进行标定
        
        Args:
            calibration_points: 标定点列表
        
        Returns:
            标定误差
        """
        error = self.coordinate_transformer.calibrate_from_points(calibration_points)
        self.is_calibrated = True
        
        # 更新手牌位置
        self._update_hand_positions()
        
        return error
    
    def _update_hand_positions(self):
        """更新手牌的物理位置（标定后调用）"""
        for tile in self.game_state.hand:
            arm_pos = self.coordinate_transformer.image_to_arm(
                tile.image_x, tile.image_y, tile.table_z
            )
            if arm_pos:
                table_pos = self.coordinate_transformer.arm_to_table(*arm_pos)
                self.game_state.update_tile_position(
                    tile.tile_id, table_pos[0], table_pos[1], table_pos[2]
                )
    
    def get_hand_state(self) -> Dict:
        """获取手牌状态"""
        return self.game_state.get_state_summary()
    
    def is_ready(self) -> bool:
        """检查是否准备好出牌"""
        return self.is_arm_connected and len(self.game_state.hand) > 0
    
    def get_workspace_info(self) -> Dict:
        """获取工作空间信息"""
        return self.motion_planner.get_workspace_info()


# 简单的命令行测试接口
if __name__ == "__main__":
    print("=" * 60)
    print("麻将机械臂控制器测试")
    print("=" * 60)
    
    controller = MahjongArmController()
    
    # 模拟手牌检测
    print("\n1. 模拟手牌检测")
    detected_tiles = [
        {'class_name': '1-wan', 'image_x': 200, 'image_y': 400, 'confidence': 0.95},
        {'class_name': '5-wan', 'image_x': 400, 'image_y': 400, 'confidence': 0.92},
        {'class_name': 'dong', 'image_x': 600, 'image_y': 400, 'confidence': 0.88},
    ]
    controller.update_hand_detection(detected_tiles)
    
    # 显示状态
    print("\n2. 当前手牌状态")
    state = controller.get_hand_state()
    print(f"手牌数量: {state['hand_count']}")
    for tile in state['hand_tiles']:
        print(f"  [{tile['id']}] {tile['name']} @ {tile['position']}")
    
    # 选中牌
    print("\n3. 选中牌 ID=1")
    controller.select_tile(1)
    selected = controller.game_state.get_selected_tile()
    if selected:
        print(f"已选中: [{selected.tile_id}] {selected.class_name}")
    
    # 模拟标定
    print("\n4. 模拟标定（使用默认单位矩阵）")
    print("标定状态:", "已标定" if controller.is_calibrated else "未标定")
    
    # 显示工作空间
    print("\n5. 工作空间信息")
    workspace = controller.get_workspace_info()
    print(f"  半径范围: {workspace['r_min']:.0f} ~ {workspace['r_max']:.0f} mm")
    print(f"  高度范围: {workspace['z_min']:.0f} ~ {workspace['z_max']:.0f} mm")
    
    # 规划运动（不实际执行）
    print("\n6. 运动规划")
    if selected:
        arm_pos = controller.coordinate_transformer.image_to_arm(
            selected.image_x, selected.image_y, selected.table_z
        )
        if arm_pos:
            sequence = controller.motion_planner.plan_pick_and_place(*arm_pos)
            print(f"规划了 {len(sequence)} 个运动步骤:")
            for i, step in enumerate(sequence[:4]):  # 只显示前4步
                print(f"  {i+1}. {step.name}: {step.description}")
            if len(sequence) > 4:
                print(f"  ... 还有 {len(sequence)-4} 步")
    
    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)
