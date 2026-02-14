# RMUC2026 雷达仿真系统

基于 ROS2 和 Ignition Gazebo 的 RoboMaster 2026 赛场雷达仿真系统。

## 项目介绍

参考自 https://github.com/robomaster-oss/rmoss_gazebo/tree/humble

主要组件：
- **RMUC2026 赛场地图**（STL 导入，尺寸 ~29m × 16m）
- **红蓝双方各 3 个机器人 + 2 个空中机器人**
- **雷达站**（暂时删除了基座和支架），搭载：
  - **Livox HAP 激光雷达**（GPU Lidar，250°×90° FOV @10Hz）
  - **工业相机 CS060**（3072×2048，FOV ~61°，30Hz）
- 通过 `ros_gz_bridge` 将传感器数据桥接到 ROS2 话题
- 发布完整的 TF 坐标变换树

目前只能实现最基础功能，仍在开发中...

## 环境要求

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble |
| Gazebo | Ignition Fortress |
| 显卡 | 支持 OpenGL 3.3+ |

## 快速开始

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

推荐使用小鱼的 rosdepc。

### 2. 编译项目

```bash
cd /data/projects/radar/simulation/radar-simulation
source /opt/ros/humble/setup.bash
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

### 3. 一键启动

```bash
# 启动完整仿真（包含 RViz）
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

## 雷达站配置

### 位置参数

配置文件：`src/radar_sim/config/sim_config.py`

- **世界位置**: (-14.5, 0, 0) - 赛场宽边边缘中心
- **LiDAR 高度**: 2.5m + 0.5m（距地面）
- **相机高度**: 2.555m + 0.5m
- **LiDAR 俯仰角**: 15°向下（0.2618 rad）
- **相机俯仰角**: 20°向下（0.3491 rad）

修改 `sim_config.py` 后运行 `generate_world.py` 即可重新生成世界文件。

### Livox HAP 传感器参数

| 参数 | 值 |
|------|-----|
| 水平 FOV | 250°（±125°） |
| 垂直 FOV | 90°（±45°） |
| 探测范围 | 0.2 - 150m |
| 扫描频率 | 10 Hz |
| 水平采样 | 2500 |
| 垂直采样 | 360 |

### CS060 相机参数

| 参数 | 值 |
|------|-----|
| 分辨率 | 3072 × 2048 |
| 水平 FOV | ~61°（6mm 镜头） |
| 帧率 | 30 Hz |
| 格式 | R8G8B8 |

## ROS2 话题

### 传感器数据

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/livox/lidar/points` | `sensor_msgs/PointCloud2` | 10 Hz | 激光雷达点云 |
| `/radar/camera` | `sensor_msgs/Image` | 30 Hz | 相机图像 |
| `/radar/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | 相机内参 |
| `/tf_static` | `tf2_msgs/TFMessage` | - | 坐标变换 |

### TF 坐标变换

| 父坐标系 | 子坐标系 | 说明 |
|----------|----------|------|
| `world` | `radar_station_base` | 雷达站底座 |
| `world` | `radar_station_blue/livox_hap/livox_hap_lidar` | LiDAR（Ignition scoped name） |
| `world` | `radar_station_blue/camera_link/radar_camera` | 相机（Ignition scoped name） |
| 上述 scoped name | `livox_hap_lidar` | LiDAR 简短别名 |
| 上述 scoped name | `radar_camera` | 相机简短别名 |

## 工具脚本

### 保存点云 demo

运行仿真时保存几帧点云为 PCD 文件，用于离线开发：

```bash
# 保存 5 帧点云
python3 src/radar_sim/scripts/save_pointcloud_demo.py --frames 5 --output demo_pcds

# 同时保存点云 + 相机图像
python3 src/radar_sim/scripts/save_pointcloud_demo.py --frames 5 --save-image --output demo_pcds
```

## 与算法项目对接

### 对接 air_target_radar 项目

```bash
# 终端1: 启动仿真
ros2 launch radar_sim radar_sim.launch.py use_rviz:=false

# 终端2: 启动点云处理节点
ros2 launch air_target_clustering clustering.launch.py \
  lidar_topic:=/livox/lidar/points \
  use_sim_time:=true
```

## 常用命令

```bash
# 查看所有话题
ros2 topic list

# 查看点云频率
ros2 topic hz /livox/lidar/points

# 查看点云数据
ros2 topic echo /livox/lidar/points --once

# 检查 TF 变换
ros2 run tf2_ros tf2_echo world livox_hap_lidar
```

## 项目结构

```
radar-simulation/
├── src/
│   ├── radar_sim/              # 雷达仿真启动包
│   │   ├── config/             # 配置文件 (sim_config.py, generate_world.py)
│   │   ├── launch/             # 启动文件
│   │   ├── rviz/               # RViz 配置
│   │   ├── worlds/             # 世界文件
│   │   └── scripts/            # 工具脚本
│   ├── rmoss_gazebo/           # Gazebo 接口
│   ├── rmoss_gz_resources/     # 模型资源
│   └── rmoss_interfaces/       # 消息定义
├── .gitattributes              # Git LFS 配置
├── .gitignore                  # Git 忽略文件
└── README.md                   # 项目说明
```

## 故障排除

### RViz 中看不到点云？
1. 检查 Fixed Frame 是否为 `world`
2. 确认话题有数据：`ros2 topic hz /livox/lidar/points`
3. 检查 TF 是否正常：`ros2 topic echo /tf_static --once`
4. 等待足够时间让所有节点完全启动
5. 其实最可能的原因是障碍物遮挡或者没有启动gazebo

### 点云显示不完整？
1. 确认雷达位置正确（位于赛场边缘）
2. 检查俯仰角配置是否合理
3. 验证 `sim_config.py` 中的参数

### Gazebo 启动慢？
确保有 GPU 驱动：
```bash
glxinfo | grep "OpenGL renderer"
# 应显示 GPU 型号，而非 llvmpipe
```

### EGL 警告？
正常的警告，不影响仿真。如需软件渲染：
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

## 开发者说明

### Git LFS
本项目使用 Git LFS 管理大文件（`.stp`, `.pcd`, `.dae`）。首次克隆后：
```bash
git lfs install
git lfs pull
```

### 参数调整
雷达站位置和传感器大多数参数统一在 `src/radar_sim/config/sim_config.py` 中配置，修改后运行：
```bash
cd src/radar_sim/config
python3 generate_world.py
colcon build --packages-select radar_sim
