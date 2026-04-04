import math
from typing import Optional, Tuple, List, Dict


def kinematics(L1, L2, alpha, beta):
    """
    正运动学
    alpha: 大臂角度（相对水平线，度数）
    beta: 小臂相对角度（相对大臂，度数），beta=0表示伸直
    返回: (x, y) 末端坐标
    """
    alpha_rad = math.radians(alpha)
    beta_rad = math.radians(beta)
    theta = alpha_rad + beta_rad  # 小臂绝对角度
    
    x = L1 * math.cos(alpha_rad) + L2 * math.cos(theta)
    y = L1 * math.sin(alpha_rad) + L2 * math.sin(theta)
    return x, y


def inverse_kinematics(L1, L2, target, elbow_up=True):
    """
    逆运动学（修正版）
    target: (tx, ty) 目标坐标
    elbow_up: True为肘部向上构型，False为肘部向下构型
    返回: (alpha, beta) 单位度，无解时返回None
    """
    tx, ty = target
    dist_sq = tx**2 + ty**2
    dist = math.sqrt(dist_sq)
    
    # 修正1: 工作空间检查使用abs(L1-L2)，避免L2>L1时错误
    if dist > L1 + L2:
        print(f"超出外极限: {dist:.1f} > {L1+L2:.1f}")
        return None
    if dist < abs(L1 - L2):
        print(f"超出内极限: {dist:.1f} < {abs(L1-L2):.1f}")
        return None
    
    # 修正2: 使用atan2代替atan，正确处理所有象限
    phi = math.atan2(ty, tx)
    
    # 计算beta（肘关节角度）
    cos_beta = (dist_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_beta = max(-1.0, min(1.0, cos_beta))  # 数值截断
    
    if elbow_up:
        beta_rad = math.acos(cos_beta)
    else:
        beta_rad = -math.acos(cos_beta)
    
    # 修正3: alpha计算使用标准公式
    # alpha = phi - atan2(L2*sin(beta), L1+L2*cos(beta))
    k1 = L1 + L2 * math.cos(beta_rad)
    k2 = L2 * math.sin(beta_rad)
    alpha_rad = phi - math.atan2(k2, k1)
    
    # 或使用几何法（等价）:
    # cos_alpha_offset = (L1**2 + dist_sq - L2**2) / (2 * L1 * dist)
    # cos_alpha_offset = max(-1.0, min(1.0, cos_alpha_offset))
    # alpha_offset = math.acos(cos_alpha_offset)
    # alpha_rad = phi - alpha_offset if elbow_up else phi + alpha_offset
    
    alpha_deg = math.degrees(alpha_rad)
    beta_deg = math.degrees(beta_rad)
    
    # 规范化到[-180, 180]
    alpha_deg = (alpha_deg + 180) % 360 - 180
    beta_deg = (beta_deg + 180) % 360 - 180
    
    return alpha_deg, beta_deg


def inverse_kinematics_all(L1, L2, target):
    """
    返回所有可行解（最多2组）
    返回: [(alpha1, beta1), (alpha2, beta2)] 或空列表
    """
    tx, ty = target
    dist_sq = tx**2 + ty**2
    dist = math.sqrt(dist_sq)
    solutions = []
    
    if dist > L1 + L2 or dist < abs(L1 - L2):
        return solutions
    
    cos_beta = (dist_sq - L1**2 - L2**2) / (2 * L1 * L2)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    
    # 两种构型：肘部向上(beta>0)和肘部向下(beta<0)
    for beta_rad in [math.acos(cos_beta), -math.acos(cos_beta)]:
        k1 = L1 + L2 * math.cos(beta_rad)
        k2 = L2 * math.sin(beta_rad)
        alpha_rad = math.atan2(ty, tx) - math.atan2(k2, k1)
        
        alpha_deg = math.degrees(alpha_rad)
        beta_deg = math.degrees(beta_rad)
        
        # 规范化
        alpha_deg = (alpha_deg + 180) % 360 - 180
        beta_deg = (beta_deg + 180) % 360 - 180
        
        solutions.append((alpha_deg, beta_deg))
    
    return solutions


class ArmKinematics:
    """
    机械臂运动学类 - 面向对象的2DOF平面机械臂运动学
    
    适用于 shoulder + elbow 两个关节控制的平面机械臂
    坐标系：r 为水平距离（前伸方向为正），z 为垂直高度（向上为正）
    """
    
    def __init__(self, L1: float = 120.0, L2: float = 100.0):
        """
        初始化运动学
        
        Args:
            L1: 大臂长度（上臂），单位 mm
            L2: 小臂长度（前臂），单位 mm
        """
        self.L1 = L1
        self.L2 = L2
    
    def forward_kinematics(self, shoulder_angle: float, elbow_angle: float) -> Tuple[float, float]:
        """
        正运动学：关节角度 -> 末端位置
        
        Args:
            shoulder_angle: 肩关节角度，相对水平线，度
            elbow_angle: 肘关节角度，相对大臂，度
        
        Returns:
            (r, z) 末端位置，单位 mm
            r: 水平距离（前伸方向为正）
            z: 垂直高度（向上为正）
        """
        shoulder_rad = math.radians(shoulder_angle)
        elbow_abs_rad = math.radians(shoulder_angle + elbow_angle)
        
        # 计算末端位置
        r = self.L1 * math.cos(shoulder_rad) + self.L2 * math.cos(elbow_abs_rad)
        z = self.L1 * math.sin(shoulder_rad) + self.L2 * math.sin(elbow_abs_rad)
        
        return r, z
    
    def inverse_kinematics(self, r: float, z: float, elbow_up: bool = True) -> Optional[Tuple[float, float]]:
        """
        逆运动学：末端位置 -> 关节角度
        
        Args:
            r: 目标水平距离，单位 mm
            z: 目标垂直高度，单位 mm
            elbow_up: True 为肘部向上构型，False 为肘部向下构型
        
        Returns:
            (shoulder角度, elbow角度) 单位度，无解时返回 None
        """
        dist_sq = r**2 + z**2
        dist = math.sqrt(dist_sq)
        
        # 工作空间检查
        if dist > self.L1 + self.L2:
            return None
        if dist < abs(self.L1 - self.L2):
            return None
        
        # 计算角度
        phi = math.atan2(z, r)
        cos_beta = (dist_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        
        # 选择构型
        if elbow_up:
            beta_rad = math.acos(cos_beta)
        else:
            beta_rad = -math.acos(cos_beta)
        
        k1 = self.L1 + self.L2 * math.cos(beta_rad)
        k2 = self.L2 * math.sin(beta_rad)
        alpha_rad = phi - math.atan2(k2, k1)
        
        shoulder_deg = math.degrees(alpha_rad)
        elbow_deg = math.degrees(beta_rad)
        
        # 规范化到 [-180, 180]
        shoulder_deg = (shoulder_deg + 180) % 360 - 180
        elbow_deg = (elbow_deg + 180) % 360 - 180
        
        return shoulder_deg, elbow_deg
    
    def inverse_kinematics_all(self, r: float, z: float) -> List[Tuple[float, float]]:
        """
        返回所有可行解（最多2组）
        
        Args:
            r: 目标水平距离，单位 mm
            z: 目标垂直高度，单位 mm
        
        Returns:
            [(shoulder1, elbow1), (shoulder2, elbow2)] 或空列表
        """
        dist_sq = r**2 + z**2
        dist = math.sqrt(dist_sq)
        solutions = []
        
        if dist > self.L1 + self.L2 or dist < abs(self.L1 - self.L2):
            return solutions
        
        cos_beta = (dist_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        
        # 两种构型：肘部向上(beta>0)和肘部向下(beta<0)
        for beta_rad in [math.acos(cos_beta), -math.acos(cos_beta)]:
            k1 = self.L1 + self.L2 * math.cos(beta_rad)
            k2 = self.L2 * math.sin(beta_rad)
            alpha_rad = math.atan2(z, r) - math.atan2(k2, k1)
            
            shoulder_deg = math.degrees(alpha_rad)
            elbow_deg = math.degrees(beta_rad)
            
            # 规范化
            shoulder_deg = (shoulder_deg + 180) % 360 - 180
            elbow_deg = (elbow_deg + 180) % 360 - 180
            
            solutions.append((shoulder_deg, elbow_deg))
        
        return solutions
    
    def compute_wrist_flex(self, shoulder_angle: float, elbow_angle: float, 
                          target_orientation: float = 0.0) -> float:
        """
        计算腕关节角度，使末端保持水平
        
        Args:
            shoulder_angle: 肩关节角度，度
            elbow_angle: 肘关节角度，度
            target_orientation: 目标末端方向，0表示水平（默认）
        
        Returns:
            wrist_flex 角度，度
        """
        # 手腕保持水平：wrist_flex = 180 - shoulder - elbow
        # 注意：这是基于当前机械臂构型的几何关系
        wrist_flex = target_orientation + 180.0 - shoulder_angle - elbow_angle
        return wrist_flex
    
    def is_reachable(self, r: float, z: float) -> bool:
        """
        检查目标位置是否可达
        
        Args:
            r: 目标水平距离，单位 mm
            z: 目标垂直高度，单位 mm
        
        Returns:
            True 如果位置可达，否则 False
        """
        dist_sq = r**2 + z**2
        dist = math.sqrt(dist_sq)
        
        if dist > self.L1 + self.L2:
            return False
        if dist < abs(self.L1 - self.L2):
            return False
        return True
    
    def get_workspace_radius(self) -> Tuple[float, float]:
        """
        获取工作空间半径范围
        
        Returns:
            (min_radius, max_radius) 单位 mm
        """
        return abs(self.L1 - self.L2), self.L1 + self.L2


class Arm3DKinematics:
    """
    3D机械臂运动学 - 基于ikpy和SO-101 URDF的5DOF运动学
    
    坐标系定义:
        - 原点: 机械臂基座中心
        - x轴: 向前（机械臂正前方）
        - y轴: 向左
        - z轴: 向上
    
    关节定义:
        - base: 基座旋转角度（绕z轴），0度时朝向x轴正方向
        - shoulder: 肩关节角度
        - elbow: 肘关节角度
        - wrist_flex: 腕关节屈伸
        - wrist_roll: 腕关节旋转
    """
    
    # 默认关节限制（度）
    DEFAULT_JOINT_LIMITS = {
        'base': (-180, 180),
        'shoulder': (-90, 180),
        'elbow': (-160, 0),
        'wrist_flex': (-90, 90),
        'wrist_roll': (-90, 90),
        'gripper': (0, 90),
    }
    
    def __init__(self, L1: float = 215.0, L2: float = 230.0,
                 joint_limits: dict = None,
                 base_shoulder_offset: float = 35.0,
                 urdf_path: str = None):
        """
        初始化3D运动学（基于ikpy + SO-101 URDF）
        
        Args:
            L1: 大臂长度（上臂），单位 mm，保留用于兼容和降级
            L2: 小臂长度（前臂），单位 mm，保留用于兼容和降级
            joint_limits: 关节角度限制字典
            base_shoulder_offset: 保留参数用于兼容
            urdf_path: URDF文件路径，None则自动查找
        """
        self.L1 = L1
        self.L2 = L2
        self.base_shoulder_offset = base_shoulder_offset
        
        # 尝试加载 ikpy 和 URDF
        self._ikpy_available = False
        self.chain = None
        self.active_links_mask = None
        
        try:
            from ikpy.chain import Chain
            import numpy as np
            import os
            import warnings
            
            # 忽略 IKPy 关于 fixed 链接的警告
            warnings.filterwarnings("ignore", message=".*fixed.*active_links_mask.*", category=UserWarning)
            warnings.filterwarnings("ignore", message=".*fixed.*axis.*", category=UserWarning)
            
            if urdf_path is None:
                urdf_path = self._find_urdf_path()
            
            self.urdf_path = urdf_path
            
            if urdf_path and os.path.exists(urdf_path):
                self.chain = Chain.from_urdf_file(urdf_path)
                n_links = len(self.chain.links)
                
                # SO-101: 7 links, 5 active joints
                # [base(fixed), shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper_frame(fixed)]
                self.active_links_mask = [False] + [True] * 5 + [False]
                if n_links != 7:
                    self.active_links_mask = [getattr(link, 'joint_type', 'fixed') != 'fixed' for link in self.chain.links]
                    if len(self.active_links_mask) >= 2:
                        self.active_links_mask[0] = False
                        self.active_links_mask[-1] = False
                
                self.chain.active_links_mask = self.active_links_mask
                self._ikpy_available = True
                self._np = np
            else:
                print(f"[Arm3DKinematics] URDF未找到: {urdf_path}")
        except Exception as e:
            print(f"[Arm3DKinematics] ikpy加载失败: {e}")
        
        # 如果 ikpy 不可用，保留几何回退
        self.planar_kin = ArmKinematics(L1, L2)
        
        # 加载关节限制
        if joint_limits is not None:
            self.joint_limits = joint_limits
        else:
            try:
                from configs.config import get_config
                config = get_config()
                if hasattr(config, 'arm') and hasattr(config.arm, 'joint_limits'):
                    self.joint_limits = config.arm.joint_limits
                else:
                    self.joint_limits = self.DEFAULT_JOINT_LIMITS.copy()
            except:
                self.joint_limits = self.DEFAULT_JOINT_LIMITS.copy()
    
    def _find_urdf_path(self) -> Optional[str]:
        """自动查找默认URDF路径"""
        import os
        current_dir = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            # 从 software/src/hal/arm 向上回溯到项目根目录，再进入 hardware/structure/URDF/SO101
            os.path.join(current_dir, "..", "..", "..", "..", "..", "hardware", "structure", "URDF", "SO101", "so101_plus.urdf"),
            # 备用：从 software/src 回溯
            os.path.join(current_dir, "..", "..", "..", "..", "hardware", "structure", "URDF", "SO101", "so101_plus.urdf"),
        ]
        for p in candidates:
            p = os.path.abspath(p)
            if os.path.exists(p):
                return p
        return None
    
    def _build_full_joints(self, angles_deg: List[float]) -> List[float]:
        """将度数的关节角度转换为ikpy所需的完整关节数组（弧度）"""
        full = [0.0] * len(self.chain.links)
        active_idx = 0
        for i, is_active in enumerate(self.active_links_mask):
            if is_active and active_idx < len(angles_deg):
                full[i] = math.radians(angles_deg[active_idx])
                active_idx += 1
        return full
    
    def _extract_angles(self, full_joints_rad: List[float]) -> List[float]:
        """从ikpy完整关节数组中提取活动关节角度（弧度）"""
        return [full_joints_rad[i] for i, active in enumerate(self.active_links_mask) if active]
    
    def _check_joint_limits(self, joints: Dict[str, float]) -> bool:
        """检查关节角度是否在限制范围内"""
        for joint_name, angle in joints.items():
            if joint_name in self.joint_limits:
                min_val, max_val = self.joint_limits[joint_name]
                if not (min_val <= angle <= max_val):
                    return False
        return True
    
    def _normalize_angle(self, angle: float) -> float:
        """将角度规范化到 [-180, 180]"""
        return (angle + 180) % 360 - 180
    
    def forward_kinematics(self, *args) -> Tuple[float, float, float]:
        """
        正运动学：关节角度 -> 3D末端位置
        
        支持两种调用方式:
            forward_kinematics(base, shoulder, elbow)  # 兼容旧接口
            forward_kinematics([base, shoulder, elbow, wrist_flex, wrist_roll])  # 新接口
        
        Returns:
            (x, y, z) 末端位置，单位 mm
        """
        if len(args) == 3:
            angles = list(args) + [0.0, 0.0]
        elif len(args) == 1 and hasattr(args[0], '__len__'):
            angles = list(args[0])
            if len(angles) < 5:
                angles = angles + [0.0] * (5 - len(angles))
        else:
            raise ValueError("forward_kinematics 参数错误，期望 (base, shoulder, elbow) 或 [joints...]")
        
        if self._ikpy_available:
            joints = self._build_full_joints(angles)
            transform = self.chain.forward_kinematics(joints)
            x, y, z = transform[:3, 3] * 1000.0  # m -> mm
            return float(x), float(y), float(z)
        else:
            return self._fallback_forward(*angles[:3])
    
    def _fallback_forward(self, base: float, shoulder: float, elbow: float) -> Tuple[float, float, float]:
        """回退：简化几何正运动学"""
        r_arm, z = self.planar_kin.forward_kinematics(shoulder, elbow)
        base_rad = math.radians(base)
        x = self.base_shoulder_offset + r_arm * math.cos(base_rad)
        y = r_arm * math.sin(base_rad)
        return x, y, z
    
    def inverse_kinematics(self, x: float, y: float, z: float,
                          elbow_up: bool = True) -> Optional[Tuple[float, float, float]]:
        """
        逆运动学：3D目标位置 -> 关节角度 (base, shoulder, elbow)
        
        Args:
            x, y, z: 目标位置，单位 mm
            elbow_up: True为肘部向上构型
        
        Returns:
            (base, shoulder, elbow) 单位度，无解时返回 None
        """
        if self._ikpy_available:
            target_pos = [x / 1000.0, y / 1000.0, z / 1000.0]
            
            # 使用不同的初始猜测来引导到不同构型
            if elbow_up:
                initial = [0, 0, math.radians(45), math.radians(-45), 0, 0, 0]
            else:
                initial = [0, 0, math.radians(-45), math.radians(45), 0, 0, 0]
            
            try:
                ik = self.chain.inverse_kinematics(
                    target_position=target_pos,
                    initial_position=initial
                )
                angles = self._extract_angles(ik)
                base_deg = math.degrees(angles[0])
                shoulder_deg = math.degrees(angles[1])
                elbow_deg = math.degrees(angles[2])
                
                # 根据 elbow_up 筛选，如果不符合则尝试另一组初始猜测
                is_up = elbow_deg > -5
                if is_up != elbow_up:
                    initial_alt = [0, 0, math.radians(-45), math.radians(45), 0, 0, 0] if elbow_up else [0, 0, math.radians(45), math.radians(-45), 0, 0, 0]
                    ik = self.chain.inverse_kinematics(
                        target_position=target_pos,
                        initial_position=initial_alt
                    )
                    angles = self._extract_angles(ik)
                    base_deg = math.degrees(angles[0])
                    shoulder_deg = math.degrees(angles[1])
                    elbow_deg = math.degrees(angles[2])
                
                return (
                    self._normalize_angle(base_deg),
                    self._normalize_angle(shoulder_deg),
                    self._normalize_angle(elbow_deg)
                )
            except Exception as e:
                print(f"[Arm3DKinematics] IK求解失败: {e}")
        
        return self._fallback_inverse(x, y, z, elbow_up)
    
    def _fallback_inverse(self, x: float, y: float, z: float,
                         elbow_up: bool = True) -> Optional[Tuple[float, float, float]]:
        """回退：简化几何逆运动学"""
        shoulder_x = self.base_shoulder_offset
        dx = x - shoulder_x
        dy = y
        r_arm = math.sqrt(dx**2 + dy**2)
        if r_arm < 0.1:
            target_angle = 0.0
        else:
            target_angle = math.atan2(dy, dx)
        
        r_for_planar = -r_arm if not elbow_up else r_arm
        result = self.planar_kin.inverse_kinematics(r_for_planar, z, elbow_up)
        if result is None:
            return None
        
        shoulder_deg, elbow_deg = result
        if not elbow_up:
            base_deg = math.degrees(target_angle + math.pi)
        else:
            base_deg = math.degrees(target_angle)
        
        return self._normalize_angle(base_deg), shoulder_deg, elbow_deg
    
    def inverse_kinematics_all(self, x: float, y: float, z: float) -> List[Tuple[float, float, float]]:
        """
        返回所有可行解，并检查关节限制
        
        Args:
            x, y, z: 目标位置，单位 mm
        
        Returns:
            [(base, shoulder, elbow), ...] 或空列表
        """
        solutions = []
        
        for elbow_up in [True, False]:
            result = self.inverse_kinematics(x, y, z, elbow_up=elbow_up)
            if result is not None:
                base, shoulder, elbow = result
                joints = {'base': base, 'shoulder': shoulder, 'elbow': elbow}
                if self._check_joint_limits(joints):
                    sol = (base, shoulder, elbow)
                    if sol not in solutions:
                        solutions.append(sol)
        
        return solutions
    
    def compute_wrist_flex(self, shoulder: float, elbow: float,
                          target_orientation: float = 0.0) -> float:
        """
        计算腕关节角度，使末端保持指定方向
        
        原理: wrist_flex = target_orientation + 180 - shoulder - elbow
        """
        wrist_flex = target_orientation + 180.0 - shoulder - elbow
        wrist_flex = (wrist_flex + 180) % 360 - 180
        return wrist_flex
    
    def compute_wrist_roll(self, base: float, target_yaw: float = 0.0) -> float:
        """
        计算腕旋转角度，使夹爪保持指定朝向
        """
        wrist_roll = target_yaw - base
        wrist_roll = (wrist_roll + 180) % 360 - 180
        return wrist_roll
    
    def solve_for_position(self, x: float, y: float, z: float,
                          target_orientation: float = 0.0,
                          target_yaw: float = 0.0,
                          elbow_up: bool = False) -> Optional[Dict[str, float]]:
        """
        完整求解：目标位置 + 末端方向 -> 所有关节角度
        
        优先使用 ikpy 的带姿态控制逆运动学，失败时回退到几何法。
        
        Args:
            x, y, z: 目标位置，单位 mm
            target_orientation: 末端俯仰角，0表示水平，度
            target_yaw: 夹爪偏航角，0表示朝x轴正方向，度
            elbow_up: 是否优先使用肘部向上构型
        
        Returns:
            {
                'base': base角度,
                'shoulder': shoulder角度,
                'elbow': elbow角度,
                'wrist_flex': wrist_flex角度,
                'wrist_roll': wrist_roll角度
            } 或 None（无解时）
        """
        # 主路径：ikpy 带姿态 IK
        if self._ikpy_available:
            target_pos = [x / 1000.0, y / 1000.0, z / 1000.0]
            
            # 根据 iktest.py 的映射：
            # 水平姿态 (target_orientation=0) 对应 orientation_mode='Y' 且 R[1] = pi/2
            # target_orientation 是相对水平的俯仰角
            # 0°水平 -> Y轴旋转 pi/2
            # 90°垂直向下 -> Y轴旋转 0
            # -90°垂直向上 -> Y轴旋转 pi
            orient_y = math.radians(target_orientation)
            target_orient = [0, orient_y, math.radians(target_yaw)]
            

            try:
                ik = self.chain.inverse_kinematics(
                    target_position=target_pos,
                    target_orientation=target_orient,
                    orientation_mode='Y',
                )
                angles = self._extract_angles(ik)
                joints = {
                    'base': self._normalize_angle(math.degrees(angles[0])),
                    'shoulder': self._normalize_angle(math.degrees(angles[1])),
                    'elbow': self._normalize_angle(math.degrees(angles[2])),
                    'wrist_flex': self._normalize_angle(math.degrees(angles[3])),
                    'wrist_roll': self._normalize_angle(math.degrees(angles[4]))
                }
                
                if self._check_joint_limits(joints):
                    return joints
            except Exception:
                print(f"[Arm3DKinematics] 带姿态IK求解失败，尝试纯位置IK + 几何wrist")
            
            # 带姿态求解失败，尝试纯位置 IK + 几何 wrist
            result = self.inverse_kinematics(x, y, z, elbow_up=elbow_up)
            if result is not None:
                base, shoulder, elbow = result
                wrist_flex = self.compute_wrist_flex(shoulder, elbow, target_orientation)
                wrist_roll = self.compute_wrist_roll(base, target_yaw)
                joints = {
                    'base': base,
                    'shoulder': shoulder,
                    'elbow': elbow,
                    'wrist_flex': self._normalize_angle(wrist_flex),
                    'wrist_roll': self._normalize_angle(wrist_roll)
                }
                if self._check_joint_limits(joints):
                    return joints
        
        # 回退路径：纯几何法
        solutions = self.inverse_kinematics_all(x, y, z)
        for base, shoulder, elbow in solutions:
            if elbow_up:
                if elbow < -5:
                    continue
            else:
                if elbow > 5:
                    continue
            
            wrist_flex = self.compute_wrist_flex(shoulder, elbow, target_orientation)
            wrist_roll = self.compute_wrist_roll(base, target_yaw)
            joints = {
                'base': base,
                'shoulder': shoulder,
                'elbow': elbow,
                'wrist_flex': self._normalize_angle(wrist_flex),
                'wrist_roll': self._normalize_angle(wrist_roll)
            }
            if self._check_joint_limits(joints):
                return joints
        
        return None
    
    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """检查目标位置是否可达"""
        dx = x - self.base_shoulder_offset
        dy = y
        r_arm = math.sqrt(dx**2 + dy**2)
        return self.planar_kin.is_reachable(r_arm, z)
    
    def get_workspace(self) -> Dict[str, float]:
        """获取工作空间范围"""
        r_min, r_max = self.planar_kin.get_workspace_radius()
        z_max = self.L1 + self.L2
        z_min = max(0, abs(self.L1 - self.L2) - 50)
        
        return {
            'r_min': r_min,
            'r_max': r_max,
            'z_min': z_min,
            'z_max': z_max
        }


# ========== 验证测试 ==========
if __name__ == "__main__":
    kin=Arm3DKinematics()
    
