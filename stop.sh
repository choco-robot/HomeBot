#!/bin/bash
# stop.sh - 统一停止所有 HomeBot 服务

PID_DIR="/tmp/homebot_pids"

if [ ! -d "$PID_DIR" ]; then
    echo "没有找到 PID 记录，服务可能未启动或已手动停止"
    exit 1
fi

echo "========================================"
echo "   停止 HomeBot 服务"
echo "========================================"

for pid_file in "$PID_DIR"/*.pid; do
    [ -e "$pid_file" ] || continue
    
    name=$(basename "$pid_file" .pid)
    pid=$(cat "$pid_file")
    
    if kill -0 "$pid" 2>/dev/null; then
        echo "  ■ 停止 $name (PID: $pid) ..."
        kill "$pid"
        
        # 等待进程退出（最多 5 秒）
        for i in {1..5}; do
            if ! kill -0 "$pid" 2>/dev/null; then
                echo "    ✓ 已停止"
                break
            fi
            sleep 1
        done
        
        # 强制终止
        if kill -0 "$pid" 2>/dev/null; then
            echo "    ! 强制终止"
            kill -9 "$pid"
        fi
    else
        echo "  ○ $name 已不在运行"
    fi
done

# 清理
rm -rf "$PID_DIR"
echo ""
echo "========================================"
echo "  所有服务已停止"
echo "========================================"