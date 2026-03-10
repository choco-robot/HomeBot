import math

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