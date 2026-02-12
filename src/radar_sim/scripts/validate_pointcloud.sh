#!/bin/bash

# 点云数据验证脚本
# 用于检查点云数据的质量和覆盖范围

echo "=========================================="
echo "点云数据验证工具"
echo "=========================================="

# 检查点云话题
echo ""
echo "1. 检查点云话题状态..."
if ros2 topic info /livox/lidar/points 2>/dev/null; then
    echo "✅ 点云话题存在"
else
    echo "❌ 错误: 点云话题不存在"
    exit 1
fi

# 检查点云数据
echo ""
echo "2. 检查点云数据质量..."
echo "监听点云数据(5秒)..."
timeout 5s ros2 topic echo /livox/lidar/points --once 2>/dev/null > /tmp/pointcloud_test.txt

if [ -s /tmp/pointcloud_test.txt ]; then
    echo "✅ 点云数据接收成功"
    
    # 提取点云大小信息
    POINT_COUNT=$(grep -o '"width": [0-9]*' /tmp/pointcloud_test.txt | head -1 | cut -d' ' -f2)
    if [ ! -z "$POINT_COUNT" ]; then
        echo "点云包含 $POINT_COUNT 个点"
        
        if [ "$POINT_COUNT" -lt 100 ]; then
            echo "⚠️  警告: 点云数量较少，可能扫描范围不完整"
        else
            echo "✅ 点云数量充足"
        fi
    fi
else
    echo "❌ 错误: 无法接收点云数据"
fi

# 检查点云频率
echo ""
echo "3. 检查点云发布频率..."
echo "监听点云话题频率(10秒)..."
timeout 10s ros2 topic hz /livox/lidar/points 2>/dev/null > /tmp/pointcloud_hz.txt

if [ -s /tmp/pointcloud_hz.txt ]; then
    AVG_HZ=$(grep "average:" /tmp/pointcloud_hz.txt | cut -d' ' -f2)
    if [ ! -z "$AVG_HZ" ]; then
        echo "平均频率: ${AVG_HZ} Hz"
        if (( $(echo "$AVG_HZ < 5" | bc -l) )); then
            echo "⚠️  警告: 点云频率较低"
        else
            echo "✅ 点云频率正常"
        fi
    fi
else
    echo "⚠️  无法测量点云频率"
fi

# 检查RViz配置
echo ""
echo "4. RViz配置建议..."
echo "请在RViz中检查以下设置:"
echo "1. Fixed Frame: world"
echo "2. PointCloud2插件:"
echo "   - Topic: /livox/lidar/points"
echo "   - Color Transformer: AxisColor 或 FlatColor"
echo "   - Size (Pixels): 3-5"
echo "   - Style: Points"

# 清理临时文件
rm -f /tmp/pointcloud_test.txt /tmp/pointcloud_hz.txt

echo ""
echo "=========================================="
echo "验证完成！"
echo ""
echo "如果点云仍然显示不完整，请尝试:"
echo "1. 重新编译工作空间: colcon build"
echo "2. 重新启动仿真"
echo "3. 调整雷达倾斜角度(在model.sdf中修改pitch值)"
echo "=========================================="