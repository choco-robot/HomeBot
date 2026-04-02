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
    3DOF机械臂运动学 - base + shoulder + elbow
    
    坐标系定义:
        - 原点: 机械臂基座中心
        - x轴: 向前（机械臂正前方）
        - y轴: 向左
        - z轴: 向上
    
    关节定义:
        - base: 基座旋转角度（绕z轴），0度时朝向x轴正方向
        - shoulder: 肩关节角度（相对水平面）
        - elbow: 肘关节角度（相对大臂）
    
    简化假设:
        - shoulder和elbow构成平面2连杆机构，在垂直于base的平面内运动
        - wrist_flex用于保持末端方向（如水平）
    """
    
    # 默认关节限制（度）
    DEFAULT_JOINT_LIMITS = {
        'base': (-180, 180),
        'shoulder': (0, 180),
        'elbow': (0, 180),
        'wrist_flex': (-90, 90),
        'wrist_roll': (-180, 180),
        'gripper': (0, 90),
    }
    
    def __init__(self, L1: float = 215.0, L2: float = 230.0, joint_limits: dict = None):
        """
        初始化3D运动学
        
        Args:
            L1: 大臂长度（上臂），单位 mm
            L2: 小臂长度（前臂），单位 mm
            joint_limits: 关节角度限制字典，如 {'base': (-180, 180), ...}
                         如果为None，则尝试从config读取，否则使用默认值
        """
        self.L1 = L1
        self.L2 = L2
        self.planar_kin = ArmKinematics(L1, L2)
        
        # 加载关节限制
        if joint_limits is not None:
            self.joint_limits = joint_limits
        else:
            # 尝试从config读取
            try:
                from configs.config import get_config
                config = get_config()
                if hasattr(config, 'arm') and hasattr(config.arm, 'joint_limits'):
                    self.joint_limits = config.arm.joint_limits
                else:
                    self.joint_limits = self.DEFAULT_JOINT_LIMITS.copy()
            except:
                self.joint_limits = self.DEFAULT_JOINT_LIMITS.copy()
        
    def _check_joint_limits(self, joints: Dict[str, float]) -> bool:
        """
        检查关节角度是否在限制范围内
        
        Args:
            joints: 关节角度字典，如 {'base': 10, 'shoulder': 45, ...}
        
        Returns:
            True 如果所有关节都在限制范围内
        """
        for joint_name, angle in joints.items():
            if joint_name in self.joint_limits:
                min_val, max_val = self.joint_limits[joint_name]
                if not (min_val <= angle <= max_val):
                    return False
        return True
    
    def _normalize_angle(self, angle: float) -> float:
        """
        将角度规范化到 [-180, 180]
        
        Args:
            angle: 输入角度，度
        
        Returns:
            规范化后的角度，度
        """
        return (angle + 180) % 360 - 180
    
    def forward_kinematics(self, base: float, shoulder: float, elbow: float) -> Tuple[float, float, float]:
        """
        正运动学：关节角度 -> 3D末端位置
        
        Args:
            base: 基座旋转角度，度，0度朝向x轴正方向
            shoulder: 肩关节角度，度，相对水平面
            elbow: 肘关节角度，度，相对大臂
        
        Returns:
            (x, y, z) 末端位置，单位 mm
            x: 向前距离
            y: 向左距离  
            z: 向上高度
        """
        # 1. 计算平面运动学得到 (r, z)
        # r: 水平面内距离原点的距离
        # z: 垂直高度
        r, z = self.planar_kin.forward_kinematics(shoulder, elbow)
        
        # 2. base旋转将 r 映射到 x-y 平面
        base_rad = math.radians(base)
        x = r * math.cos(base_rad)
        y = r * math.sin(base_rad)
        
        return x, y, z
    
    def inverse_kinematics(self, x: float, y: float, z: float, 
                          elbow_up: bool = True) -> Optional[Tuple[float, float, float]]:
        """
        逆运动学：3D目标位置 -> 关节角度
        
        Args:
            x: 目标x坐标（向前），单位 mm
            y: 目标y坐标（向左），单位 mm
            z: 目标z坐标（向上），单位 mm
            elbow_up: True为肘部向上构型，False为肘部向下
        
        Returns:
            (base, shoulder, elbow) 单位度，无解时返回 None
        """
        # 1. 计算水平距离 r
        r = math.sqrt(x**2 + y**2)
        
        # 2. 计算base角度
        # 注意：当r=0时，base可以是任意值，此时选择0
        if r < 0.1:  # 避免数值问题
            base_deg = 0.0
        else:
            base_deg = math.degrees(math.atan2(y, x))
        
        # 3. 使用2D逆运动学求解 shoulder 和 elbow
        result = self.planar_kin.inverse_kinematics(-r, z, elbow_up)
        if result is None:
            return None
        
        shoulder_deg, elbow_deg = result
        return base_deg, shoulder_deg, elbow_deg
    
    def inverse_kinematics_all(self, x: float, y: float, z: float) -> List[Tuple[float, float, float]]:
        """
        返回所有可行解（最多4组：2个平面解 × 2个base方向），并检查关节限制
        
        Args:
            x, y, z: 目标位置，单位 mm
        
        Returns:
            [(base1, shoulder1, elbow1), ...] 或空列表
            只返回符合关节限制的解
        """
        # 1. 计算水平距离和base角度
        r = math.sqrt(x**2 + y**2)
        
        if r < 0.1:
            # r接近0时，base可以是任意值，尝试限制范围内的值
            base_candidates = [0.0]
            base_min, base_max = self.joint_limits.get('base', (-180, 180))
            if base_min <= 0 <= base_max:
                base_candidates = [0.0]
            else:
                base_candidates = [(base_min + base_max) / 2]
        else:
            base_base = math.degrees(math.atan2(y, x))
            base_candidates = [
                self._normalize_angle(base_base),
                self._normalize_angle(base_base + 180)
            ]
        
        # 2. 获取平面运动学的所有解（肘部向上/向下）
        planar_solutions = self.planar_kin.inverse_kinematics_all(-r, z)
        
        # 3. 组合成3D解并检查关节限制
        solutions = []
        for shoulder_deg, elbow_deg in planar_solutions:
            # 规范化角度
            shoulder_deg = self._normalize_angle(shoulder_deg)
            elbow_deg = self._normalize_angle(elbow_deg)
            
            # 尝试不同的base角度
            for base_deg in base_candidates:
                joints = {'base': base_deg, 'shoulder': shoulder_deg, 'elbow': elbow_deg}
                if self._check_joint_limits(joints):
                    # 避免重复解
                    sol = (base_deg, shoulder_deg, elbow_deg)
                    if sol not in solutions:
                        solutions.append(sol)
        
        return solutions
    
    def compute_wrist_flex(self, shoulder: float, elbow: float, 
                          target_orientation: float = 0.0) -> float:
        """
        计算腕关节角度，使末端保持指定方向
        
        原理: wrist_flex = target_orientation - shoulder - elbow
        当target_orientation=0时，末端保持水平
        
        Args:
            shoulder: 肩关节角度，度
            elbow: 肘关节角度，度
            target_orientation: 目标末端方向，0表示水平（默认）
        
        Returns:
            wrist_flex角度，度
        """
        wrist_flex = target_orientation - shoulder - elbow
        # 规范化到 [-180, 180]
        wrist_flex = (wrist_flex + 180) % 360 - 180
        return wrist_flex
    
    def compute_wrist_roll(self, base: float, target_yaw: float = 0.0) -> float:
        """
        计算腕旋转角度，使夹爪保持指定朝向
        
        原理: 当base旋转时，如果不调整wrist_roll，夹爪会跟着旋转。
        通过 wrist_roll = target_yaw - base 可以保持夹爪绝对朝向不变。
        
        Args:
            base: 基座旋转角度，度
            target_yaw: 目标夹爪朝向（相对世界坐标系），0表示朝x轴正方向
        
        Returns:
            wrist_roll角度，度
        """
        wrist_roll = target_yaw - base
        # 规范化到 [-180, 180]
        wrist_roll = (wrist_roll + 180) % 360 - 180
        return wrist_roll
    
    def solve_for_position(self, x: float, y: float, z: float,
                          target_orientation: float = 0.0,
                          target_yaw: float = 0.0,
                          elbow_up: bool = True) -> Optional[Dict[str, float]]:
        """
        完整求解：目标位置 + 末端方向 -> 所有关节角度
        
        会尝试所有可行解，返回第一个符合关节限制的解
        
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
        # 获取所有可行解（已经过滤了不符合base/shoulder/elbow限制的）
        solutions = self.inverse_kinematics_all(x, y, z)
        
        for base, shoulder, elbow in solutions:
            # 计算腕关节补偿
            wrist_flex = self.compute_wrist_flex(shoulder, elbow, target_orientation)
            wrist_roll = self.compute_wrist_roll(base, target_yaw)
            
            # 规范化
            wrist_flex = self._normalize_angle(wrist_flex)
            wrist_roll = self._normalize_angle(wrist_roll)
            
            # 检查所有关节限制
            joints = {
                'base': base,
                'shoulder': shoulder,
                'elbow': elbow,
                'wrist_flex': wrist_flex,
                'wrist_roll': wrist_roll
            }
            
            if self._check_joint_limits(joints):
                return joints
        
        # 如果没有找到符合所有限制的解，返回None
        return None
    
    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """
        检查目标位置是否可达
        
        Args:
            x, y, z: 目标位置，单位 mm
        
        Returns:
            True 如果位置可达
        """
        r = math.sqrt(x**2 + y**2)
        return self.planar_kin.is_reachable(r, z)
    
    def get_workspace(self) -> Dict[str, float]:
        """
        获取工作空间范围
        
        Returns:
            {
                'r_min': 最小水平距离,
                'r_max': 最大水平距离,
                'z_min': 最小高度（近似）,
                'z_max': 最大高度（近似）
            }
        """
        r_min, r_max = self.planar_kin.get_workspace_radius()
        
        # 高度范围近似估计
        # 最大高度：当shoulder=90°（垂直向上）时，z = L1 + L2
        z_max = self.L1 + self.L2
        # 最小高度：当shoulder=-90°（垂直向下）时，z = -(L1 + L2)
        # 但实际中机械臂不能穿过地面，所以通常是0或某个最小值
        z_min = max(0, abs(self.L1 - self.L2) - 50)  # 保守估计
        
        return {
            'r_min': r_min,
            'r_max': r_max,
            'z_min': z_min,
            'z_max': z_max
        }


# ========== 验证测试 ==========
if __name__ == "__main__":
    L1, L2 = 100.0, 80.0
    
    # 测试1: 正逆运动学一致性验证
    alpha_test, beta_test = 30.0, 45.0
    x, y = kinematics(L1, L2, alpha_test, beta_test)
    print(f"测试角度: α={alpha_test}°, β={beta_test}°")
    print(f"正运动学: x={x:.2f}, y={y:.2f}")
    
    # 逆解
    sol = inverse_kinematics_all(L1, L2, (x, y))
    print(f"逆运动学解: {sol}")
    
    # 验证反推
    for i, (a, b) in enumerate(sol):
        x_check, y_check = kinematics(L1, L2, a, b)
        print(f"  解{i+1}: α={a:.1f}°, β={b:.1f}° -> x={x_check:.2f}, y={y_check:.2f}")
    
    # 测试2: 特殊位置（第二象限）
    print("\n测试目标点(-50, 120):")
    sol = inverse_kinematics_all(L1, L2, (-50, 120))
    print(f"可行解: {sol}")
    
    # 测试3: ArmKinematics 类
    print("\n" + "="*50)
    print("ArmKinematics 类测试")
    print("="*50)
    
    kin = ArmKinematics(L1=120.0, L2=100.0)
    
    # 正运动学测试
    shoulder, elbow = 30.0, 45.0
    r, z = kin.forward_kinematics(shoulder, elbow)
    print(f"\n正运动学: shoulder={shoulder}°, elbow={elbow}°")
    print(f"末端位置: r={r:.1f}mm, z={z:.1f}mm")
    
    # 逆运动学测试
    ik_result = kin.inverse_kinematics(r, z)
    print(f"\n逆运动学: r={r:.1f}mm, z={z:.1f}mm")
    print(f"解: shoulder={ik_result[0]:.1f}°, elbow={ik_result[1]:.1f}°")
    
    # 手腕角度计算
    wrist = kin.compute_wrist_flex(ik_result[0], ik_result[1], target_orientation=0.0)
    print(f"手腕角度(保持水平): {wrist:.1f}°")
    
    # 可达性测试
    print(f"\n工作空间半径: {kin.get_workspace_radius()}")
    print(f"位置 (150, 100) 是否可达: {kin.is_reachable(150, 100)}")
    print(f"位置 (300, 300) 是否可达: {kin.is_reachable(300, 300)}")
