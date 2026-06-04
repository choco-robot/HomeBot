#!/bin/bash
# macOS 临时符号链接脚本（udev 替代方案）
# udev 是 Linux 特有的，macOS 不支持 udev 规则。
# 由于 macOS SIP 保护，无法在 /dev 下创建文件，因此使用 /usr/local/dev 作为替代。
# 注意：macOS 重启后若设备路径变化，可能需要重新运行。

set -e

LIDAR_SRC="/dev/cu.usbserial-14210"
SERVO_SRC="/dev/cu.usbmodem5A7C1210751"
DEVDIR="/usr/local/dev"

if [ ! -c "$LIDAR_SRC" ]; then
    echo "错误: 激光雷达设备未找到: $LIDAR_SRC"
    exit 1
fi

if [ ! -c "$SERVO_SRC" ]; then
    echo "错误: 舵机驱动设备未找到: $SERVO_SRC"
    exit 1
fi

sudo mkdir -p "$DEVDIR"
sudo rm -f "$DEVDIR/lidar" "$DEVDIR/servobus"
sudo ln -s "$LIDAR_SRC" "$DEVDIR/lidar"
sudo ln -s "$SERVO_SRC" "$DEVDIR/servobus"

echo "已创建符号链接:"
ls -la "$DEVDIR/lidar" "$DEVDIR/servobus"

echo ""
echo "提示: macOS 上请使用以下路径访问设备："
echo "  激光雷达 -> $DEVDIR/lidar"
echo "  舵机驱动 -> $DEVDIR/servobus"
echo ""
echo "如需在代码中统一兼容 Linux/macOS，可配置为："
echo '  lidar_port = "/dev/lidar" if os.path.exists("/dev/lidar") else "/usr/local/dev/lidar"'
