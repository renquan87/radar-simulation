# RMUC2026 雷达仿真系统

基于 ROS2 和 Ignition Gazebo 的 RoboMaster 2026 赛场雷达仿真系统，用于开发空中机器人定位。

##  项目介绍
参考自 https://github.com/robomaster-oss/rmoss_gazebo/tree/humble?tab=readme-ov-file

主要包括：

-  RMUC2026 赛场环境仿真
-  Livox HAP 激光雷达仿真（包括点云节点发布和rviz可视化）
-  工业相机仿真
-  红蓝双方各 3 个机器人模型

目前只能实现最基础功能，仍在开发中...

##  环境要求

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble |
| Gazebo | Ignition Fortress |
| 显卡 | 支持 OpenGL 3.3+ |

##  快速开始

### 1. 安装依赖

```bash
sudo apt-get install -y \
  ignition-fortress \
  ros-humble-ros-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-image \
  ros-humble-tf2-ros
```

### 2. 编译项目

```bash
cd /data/projects/radar/simulation/radar-simulation
source /opt/ros/humble/setup.bash
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```
推荐使用小鱼的rosdepc

### 3. 启动仿真

```bash
# 一键启动（包含 RViz）
ros2 launch radar_sim radar_sim.launch.py

# 仅启动 Gazebo
ros2 launch radar_sim radar_sim.launch.py use_rviz:=false
```

### 4. 验证运行

```bash
# 检查点云数据
ros2 topic hz /livox/lidar/points

# 检查 TF 树
ros2 run tf2_tools view_frames

# 查看 RViz 中的点云显示
```

##  项目结构

```
radar-simulation/
├── src/
│   ├──────radar_sim/           # 雷达仿真启动包
|   |   ├── config/             # 配置文件 
│   │   ├── launch/             # 启动文件
│   │   ├── rviz/               # RViz 配置
│   │   ├── worlds/             # 世界文件
│   │   └── scripts/            # 工具脚本
│   ├── rmoss_gazebo/           # Gazebo 接口
│   ├── rmoss_gz_resources/     # 模型资源
│   └── rmoss_interfaces/       # 消息定义
├── 使用说明.md                  # 使用文档
└── README.md                   # readme
```

##  传感器数据

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/livox/lidar/points` | `sensor_msgs/PointCloud2` | 10 Hz | 激光雷达点云 |
| `/radar/camera/image` | `sensor_msgs/Image` | 30 Hz | 相机图像 |
| `/radar/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | 相机参数 |
| `/tf_static` | `tf2_msgs/TFMessage` | - | 坐标变换 |

##  常用命令

```bash
# 查看所有话题
ros2 topic list

# 查看点云数据
ros2 topic echo /livox/lidar/points --once

# 查看相机图像
ros2 topic echo /radar/camera/image --once | head -10

# 检查 TF 变换
ros2 run tf2_ros tf2_echo world livox_hap_lidar

# 在 RViz 中手动添加点云显示
# Topic: /livox/lidar/points
# Fixed Frame: world
```

##  故障排除

### RViz 中看不到点云？
1. 确认话题有数据：`ros2 topic hz /livox/lidar/points`
2. 检查 TF 是否正常：`ros2 topic echo /tf_static --once`
3. 检查激光雷达是否被障碍物遮挡



