"""
麻将游戏状态管理器

管理：
1. 手牌状态（13张牌）
2. 虚拟牌与物理牌的映射
3. 出牌记录
4. 手牌区域坐标管理
"""

from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from common.logging import get_logger

logger = get_logger(__name__)


class TileStatus(Enum):
    """牌的状态"""
    IN_HAND = "in_hand"      # 在手牌中
    SELECTED = "selected"    # 被选中
    MOVING = "moving"        # 移动中
    DISCARDED = "discarded"  # 已打出


@dataclass
class MahjongTileState:
    """单张牌的状态"""
    tile_id: int                    # 牌ID（0-12，对应手牌位置）
    class_name: str                 # 牌的类别（如 "5-wan", "dong"）
    status: TileStatus = TileStatus.IN_HAND
    
    # 物理位置（牌桌坐标系 mm）
    table_x: float = 0.0
    table_y: float = 0.0
    table_z: float = 30.0
    
    # 图像坐标（用于显示）
    image_x: float = 0.0
    image_y: float = 0.0
    
    # 检测置信度
    confidence: float = 0.0


@dataclass
class HandRegion:
    """手牌区域定义"""
    # 牌桌坐标系下的区域范围（mm）
    x_min: float = 100.0
    x_max: float = 500.0
    y_min: float = 100.0
    y_max: float = 150.0
    z_height: float = 30.0
    
    # 13张牌的预设位置（用于简化控制）
    slot_spacing: float = 30.0  # 牌槽间距
    
    def get_slot_position(self, slot_index: int) -> Tuple[float, float, float]:
        """
        获取第n个牌槽的位置
        
        Args:
            slot_index: 牌槽索引 0-12
        
        Returns:
            (x, y, z) 牌桌坐标
        """
        # 从左到右排列13张牌
        x = self.x_min + slot_index * self.slot_spacing
        y = (self.y_min + self.y_max) / 2  # 区域中心
        z = self.z_height
        return x, y, z


@dataclass
class DiscardSlot:
    """出牌槽定义 - 支持双机械臂"""
    table_x: float = 200.0
    table_y: float = 0.0
    table_z: float = 30.0
    
    def __init__(self, arm_id: int = 1):
        """
        初始化出牌槽
        
        Args:
            arm_id: 机械臂ID，1=第一套(arm)，2=第二套(arm2)
        """
        from configs.config import get_config
        config = get_config()
        
        if arm_id == 2:
            self.table_x = config.mahjong.arm2_discard_slot_x
            self.table_y = config.mahjong.arm2_discard_slot_y
            self.table_z = config.mahjong.arm2_discard_slot_z
        else:
            self.table_x = config.mahjong.discard_slot_x
            self.table_y = config.mahjong.discard_slot_y
            self.table_z = config.mahjong.discard_slot_z


class GameStateManager:
    """
    麻将游戏状态管理器
    
    管理手牌状态，建立虚拟牌与物理牌的映射
    支持双机械臂配置
    """
    
    def __init__(self, arm_id: int = 1):
        """
        初始化游戏状态管理器
        
        Args:
            arm_id: 机械臂ID，1=第一套(arm)，2=第二套(arm2)
        """
        self.arm_id = arm_id
        arm_name = "arm2" if arm_id == 2 else "arm"
        
        # 手牌（最多13张）
        self.hand: List[MahjongTileState] = []
        
        # 出牌记录
        self.discarded: List[MahjongTileState] = []
        
        # 手牌区域
        self.hand_region = HandRegion()
        
        # 出牌槽 - 根据 arm_id 选择配置
        self.discard_slot = DiscardSlot(arm_id)
        
        # 当前选中的牌ID
        self.selected_tile_id: Optional[int] = None
        
        # 虚拟牌到物理位置的映射
        # 简化版本：假设牌按顺序放在13个固定位置
        self.slot_to_tile: Dict[int, int] = {}  # slot_index -> tile_id
        
        logger.info(f"GameStateManager 初始化完成 [{arm_name}]")
    
    def initialize_hand(self, detected_tiles: List[Dict]):
        """
        初始化手牌
        
        Args:
            detected_tiles: 检测到的牌列表，每个包含：
                - class_name: 牌的类别
                - image_x, image_y: 图像坐标
                - confidence: 置信度
        """
        self.hand = []
        self.slot_to_tile = {}
        
        # 按图像x坐标排序（从左到右）
        sorted_tiles = sorted(detected_tiles, key=lambda t: t.get('image_x', 0))
        
        for i, tile_data in enumerate(sorted_tiles[:13]):  # 最多13张
            tile = MahjongTileState(
                tile_id=i,
                class_name=tile_data.get('class_name', 'unknown'),
                status=TileStatus.IN_HAND,
                image_x=tile_data.get('image_x', 0),
                image_y=tile_data.get('image_y', 0),
                confidence=tile_data.get('confidence', 0)
            )
            
            # 预设物理位置（简化版本：使用固定牌槽）
            x, y, z = self.hand_region.get_slot_position(i)
            tile.table_x = x
            tile.table_y = y
            tile.table_z = z
            
            self.hand.append(tile)
            self.slot_to_tile[i] = i
        
        logger.info(f"手牌初始化完成: {len(self.hand)} 张牌")
        # for tile in self.hand:
        #     logger.info(f"  [{tile.tile_id}] {tile.class_name} @ "
        #                f"({tile.table_x:.0f}, {tile.table_y:.0f})")
    
    def get_tile_by_id(self, tile_id: int) -> Optional[MahjongTileState]:
        """根据ID获取牌状态"""
        for tile in self.hand:
            if tile.tile_id == tile_id:
                return tile
        return None
    
    def select_tile(self, tile_id: int) -> bool:
        """
        选中一张牌
        
        Args:
            tile_id: 牌ID
        
        Returns:
            是否成功选中
        """
        tile = self.get_tile_by_id(tile_id)
        if tile is None:
            logger.warning(f"选中的牌ID {tile_id} 不存在")
            return False
        
        if tile.status != TileStatus.IN_HAND:
            logger.warning(f"牌 {tile_id} 状态为 {tile.status.value}，无法选中")
            return False
        
        # 取消之前的选中
        if self.selected_tile_id is not None:
            prev_tile = self.get_tile_by_id(self.selected_tile_id)
            if prev_tile:
                prev_tile.status = TileStatus.IN_HAND
        
        # 选中新牌
        tile.status = TileStatus.SELECTED
        self.selected_tile_id = tile_id
        
        logger.info(f"选中牌 {tile_id}: {tile.class_name}")
        return True
    
    def get_selected_tile(self) -> Optional[MahjongTileState]:
        """获取当前选中的牌"""
        if self.selected_tile_id is None:
            return None
        return self.get_tile_by_id(self.selected_tile_id)
    
    def discard_selected(self) -> Optional[MahjongTileState]:
        """
        打出选中的牌
        
        Returns:
            被打出的牌，如果没有选中的牌则返回None
        """
        tile = self.get_selected_tile()
        if tile is None:
            logger.warning("没有选中的牌，无法出牌")
            return None
        
        # 更新状态
        tile.status = TileStatus.DISCARDED
        self.selected_tile_id = None
        
        # 从手牌移除，添加到出牌记录
        self.hand.remove(tile)
        self.discarded.append(tile)
        
        # 更新牌槽映射
        self._update_slot_mapping()
        
        logger.info(f"出牌完成: {tile.class_name}")
        return tile
    
    def _update_slot_mapping(self):
        """更新牌槽映射（出牌后重新排列）"""
        # 简化版本：保持原有映射，空缺位置留空
        # 实际项目中可能需要重新排列牌的位置
        self.slot_to_tile = {}
        for i, tile in enumerate(self.hand):
            self.slot_to_tile[i] = tile.tile_id
    
    def get_hand_tiles(self) -> List[MahjongTileState]:
        """获取所有手牌"""
        return self.hand.copy()
    
    def get_discarded_tiles(self) -> List[MahjongTileState]:
        """获取所有打出的牌"""
        return self.discarded.copy()
    
    def get_tile_position(self, tile_id: int) -> Optional[Tuple[float, float, float]]:
        """
        获取牌的物理位置
        
        Args:
            tile_id: 牌ID
        
        Returns:
            (x, y, z) 牌桌坐标，如果牌不存在返回None
        """
        tile = self.get_tile_by_id(tile_id)
        if tile is None:
            return None
        return (tile.table_x, tile.table_y, tile.table_z)
    
    def update_tile_position(self, tile_id: int, 
                            table_x: float, table_y: float, table_z: float):
        """
        更新牌的物理位置（用于标定后更新）
        
        Args:
            tile_id: 牌ID
            table_x, table_y, table_z: 新的牌桌坐标
        """
        tile = self.get_tile_by_id(tile_id)
        if tile:
            tile.table_x = table_x
            tile.table_y = table_y
            tile.table_z = table_z
            logger.debug(f"更新牌 {tile_id} 位置: ({table_x:.0f}, {table_y:.0f}, {table_z:.0f})")
    
    def get_slot_positions(self) -> List[Tuple[int, float, float, float]]:
        """
        获取所有牌槽的位置
        
        Returns:
            [(slot_index, x, y, z), ...]
        """
        positions = []
        for i in range(13):
            x, y, z = self.hand_region.get_slot_position(i)
            positions.append((i, x, y, z))
        return positions
    
    def get_discard_slot_position(self) -> Tuple[float, float, float]:
        """获取出牌槽位置"""
        return (self.discard_slot.table_x, 
                self.discard_slot.table_y, 
                self.discard_slot.table_z)
    
    def reset(self):
        """重置游戏状态"""
        self.hand = []
        self.discarded = []
        self.selected_tile_id = None
        self.slot_to_tile = {}
        logger.info("游戏状态已重置")
    
    def get_state_summary(self) -> Dict:
        """获取状态摘要"""
        return {
            'hand_count': len(self.hand),
            'discarded_count': len(self.discarded),
            'selected_tile': self.selected_tile_id,
            'hand_tiles': [
                {
                    'id': t.tile_id,
                    'name': t.class_name,
                    'status': t.status.value,
                    'position': (t.table_x, t.table_y, t.table_z)
                }
                for t in self.hand
            ]
        }


if __name__ == "__main__":
    # 测试游戏状态管理器
    print("=" * 60)
    print("游戏状态管理器测试")
    print("=" * 60)
    
    manager = GameStateManager()
    
    # 测试1: 初始化手牌
    print("\n测试1: 初始化手牌")
    detected_tiles = [
        {'class_name': '1-wan', 'image_x': 100, 'confidence': 0.95},
        {'class_name': '2-wan', 'image_x': 200, 'confidence': 0.92},
        {'class_name': '3-wan', 'image_x': 300, 'confidence': 0.88},
        {'class_name': '4-wan', 'image_x': 400, 'confidence': 0.91},
        {'class_name': '5-wan', 'image_x': 500, 'confidence': 0.93},
        {'class_name': 'dong', 'image_x': 600, 'confidence': 0.89},
        {'class_name': 'nan', 'image_x': 700, 'confidence': 0.90},
        {'class_name': 'xi', 'image_x': 800, 'confidence': 0.87},
        {'class_name': 'bei', 'image_x': 900, 'confidence': 0.94},
        {'class_name': 'zhong', 'image_x': 1000, 'confidence': 0.86},
        {'class_name': 'fa', 'image_x': 1100, 'confidence': 0.88},
        {'class_name': 'bai', 'image_x': 1200, 'confidence': 0.91},
        {'class_name': '1-tiao', 'image_x': 1300, 'confidence': 0.90},
    ]
    
    manager.initialize_hand(detected_tiles)
    
    # 测试2: 选中牌
    print("\n测试2: 选中牌")
    manager.select_tile(5)
    selected = manager.get_selected_tile()
    if selected:
        print(f"当前选中: [{selected.tile_id}] {selected.class_name}")
    
    # 测试3: 出牌
    print("\n测试3: 出牌")
    discarded = manager.discard_selected()
    if discarded:
        print(f"打出: [{discarded.tile_id}] {discarded.class_name}")
    
    # 测试4: 获取状态摘要
    print("\n测试4: 状态摘要")
    summary = manager.get_state_summary()
    print(f"手牌数量: {summary['hand_count']}")
    print(f"已出牌数: {summary['discarded_count']}")
    print(f"剩余手牌: {[t['name'] for t in summary['hand_tiles']]}")
    
    # 测试5: 获取牌槽位置
    print("\n测试5: 牌槽位置")
    slot_positions = manager.get_slot_positions()
    for i, x, y, z in slot_positions[:5]:  # 只显示前5个
        print(f"  槽位{i}: ({x:.0f}, {y:.0f}, {z:.0f})")
