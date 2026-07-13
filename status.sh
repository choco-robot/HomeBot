#!/bin/bash
# status.sh - 查看服务运行状态

PID_DIR="/tmp/homebot_pids"

echo "========================================"
echo "   HomeBot 服务状态"
echo "========================================"

if [ ! -d "$PID_DIR" ]; then
    echo "  服务未启动"
    exit 0
fi

for pid_file in "$PID_DIR"/*.pid; do
    [ -e "$pid_file" ] || continue
    
    name=$(basename "$pid_file" .pid)
    pid=$(cat "$pid_file")
    
    if kill -0 "$pid" 2>/dev/null; then
        # 获取 CPU/内存
        cpu_mem=$(ps -p "$pid" -o %cpu,%mem --no-headers 2>/dev/null | xargs)
        echo "  ● $name | PID: $pid | 运行中 | CPU/MEM: $cpu_mem"
    else
        echo "  ✗ $name | PID: $pid | 已停止"
    fi
done

echo "========================================"