#!/bin/bash

# 雷达仿真环境设置脚本
# 确保正确source工作空间并提供清晰的启动指令

echo "=========================================="
echo "雷达仿真环境设置工具"
echo "=========================================="

# 检查当前目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

echo "检测到工作空间目录: $WORKSPACE_DIR"
echo ""

# 检查是否是正确的工作空间
if [[ "$WORKSPACE_DIR" != *"ros2_simu_ws"* ]]; then
    echo "❌ 错误: 此脚本不在ros2_simu_ws工作空间中"
    echo "请确保您位于正确的雷达仿真工作空间中"
    exit 1
fi

echo "✅ 工作空间验证通过"
echo ""

# 设置环境
echo "正在设置ROS2环境..."
source /opt/ros/humble/setup.bash
echo "✅ ROS2 Humble环境已加载"

echo ""
echo "正在加载雷达仿真工作空间..."
cd "$WORKSPACE_DIR"
source install/setup.bash
echo "✅ 雷达仿真工作空间已加载"

echo ""
echo "检查radar_sim包..."
if ! ros2 pkg list | grep -q "radar_sim"; then
    echo "❌ 错误: radar_sim包未找到"
    echo "请先编译工作空间:"
    echo "  cd $WORKSPACE_DIR"
    echo "  colcon build"
    exit 1
fi
echo "✅ radar_sim包已找到"

echo ""
echo "=========================================="
echo "环境设置完成！"
echo ""
echo "启动雷达仿真，请运行:"
echo "  ros2 launch radar_sim radar_sim.launch.py"
echo ""
echo "如果点云显示不完整，请检查:"
echo "1. RViz中的Fixed Frame是否设置为'world'"
echo "2. PointCloud2插件的话题是否为'/livox/lidar/points'"
echo "3. 运行调试脚本: ./scripts/check_radar_sim.sh"
echo "=========================================="