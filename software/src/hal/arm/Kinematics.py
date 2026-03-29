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
