#!/bin/bash

# 雷达仿真调试脚本
# 用于检查TF树、点云话题和RViz状态

echo "=========================================="
echo "雷达仿真调试工具"
echo "=========================================="

# 1. 检查TF树完整性
echo ""
echo "1. 检查TF树完整性..."
echo "正在生成TF树图，请稍候..."
ros2 run tf2_tools view_frames
echo "TF树图已生成为 frames.pdf，请查看确认从world到雷达frame的路径是否完整"

# 2. 检查点云话题
echo ""
echo "2. 检查点云话题状态..."
echo "检查点云话题 /livox/lidar/points:"
ros2 topic info /livox/lidar/points 2>/dev/null || echo "错误: 点云话题不存在或无法访问"

echo ""
echo "检查最近5秒的点云消息:"
ros2 topic echo /livox/lidar/points --once 2>/dev/null || echo "错误: 无法接收点云消息"

# 3. 检查仿真时间
echo ""
echo "3. 检查仿真时间..."
echo "检查 /clock 话题:"
ros2 topic info /clock 2>/dev/null || echo "警告: /clock 话题不存在，可能未使用仿真时间"

echo ""
echo "检查当前仿真时间:"
ros2 topic echo /clock --once 2>/dev/null || echo "错误: 无法获取仿真时间"

# 4. 检查TF发布状态
echo ""
echo "4. 检查TF发布状态..."
echo "检查 world 坐标系:"
ros2 run tf2_ros tf2_echo world radar_station_blue/livox_hap/livox_hap_lidar 2>/dev/null || echo "错误: 无法从world转换到雷达坐标系"

echo ""
echo "检查所有TF变换:"
ros2 run tf2_ros tf2_monitor --ros-args -p use_sim_time:=true 2>/dev/null || echo "警告: 无法监控TF变换"

# 5. 检查RViz状态
echo ""
echo "5. 检查RViz节点状态..."
echo "检查RViz节点是否运行:"
pgrep -f rviz2 > /dev/null && echo "RViz节点正在运行" || echo "警告: RViz节点未运行"

echo ""
echo "=========================================="
echo "调试完成！"
echo "如果发现问题，请检查上述输出中的错误信息"
echo "=========================================="