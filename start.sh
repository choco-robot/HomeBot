#!/bin/bash
# start.sh - 启动 HomeBot 服务并记录 PID
sleep 3

set -e

# 配置
PID_DIR="/tmp/homebot_pids"
VENV_PATH="$HOME/homebot/.venv"
SRC_PATH="$HOME/homebot/software/src"

# 清理旧 PID 文件
rm -rf "$PID_DIR"
mkdir -p "$PID_DIR"

# 激活虚拟环境
source "$VENV_PATH/bin/activate"

cd "$SRC_DIR"

echo "========================================"
echo "   启动 HomeBot 服务"
echo "========================================"

# 定义服务列表
declare -A SERVICES=(
    ["motion_service"]="services.motion_service.arm_service"
    # ["vision_service"]="services.vision_service"
    # ["gamepad_control"]="applications.gamepad_control"
    # ["remote_control"]="applications.remote_control"
)

# 启动每个服务并记录 PID
for name in "${!SERVICES[@]}"; do
    module="${SERVICES[$name]}"
    echo "  ▶ 启动 $name ..."
    
    nohup python -m "$module" > "/tmp/homebot_${name}.log" 2>&1 &
    pid=$!
    
    echo $pid > "$PID_DIR/${name}.pid"
    echo "    PID: $pid | 日志: /tmp/homebot_${name}.log"
done

echo ""
echo "========================================"
echo "  所有服务已启动"
echo "  PID 目录: $PID_DIR"
echo ""
echo "  查看状态:  ./status.sh"
echo "  停止服务:  ./stop.sh"
echo "========================================"