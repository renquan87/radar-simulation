# RMUC2026 雷达仿真 - 配置与参数修改指南

## 概述

本仿真环境基于 Gazebo Ignition (Fortress)，模拟 RoboMaster 2026 超级对抗赛 (RMUC) 的雷达站视角。所有可调参数集中在 `sim_config.py` 中，修改后运行 `generate_world.py` 即可重新生成世界文件。

## 快速开始

```bash
# 1. 修改参数
vim src/radar_sim/config/sim_config.py

# 2. 重新生成世界文件
python3 src/radar_sim/config/generate_world.py

# 3. 重新构建（如果修改了模型文件）
colcon build --symlink-install --packages-select radar_sim rmoss_gz_resources

# 4. 启动仿真
source install/setup.bash
ros2 launch radar_sim radar_sim.launch.py
```

---

## 文件结构

```
src/
├── radar_sim/
│   ├── config/
│   │   ├── sim_config.py          # ★ 主配置文件（修改这里）
│   │   ├── generate_world.py      # 世界文件生成器
│   │   └── README.md              # 本文件
│   ├── worlds/
│   │   └── rmuc2026_radar_sim.sdf # 生成的世界文件（不要手动编辑）
│   └── launch/
│       └── radar_sim.launch.py    # ROS2 启动文件
├── rmoss_gz_resources/
│   └── resource/models/           # Gazebo 模型库
│       ├── radar_station/         # 雷达站模型
│       ├── rmua19_standard_robot/ # 地面机器人模型
│       ├── aerial_robot/          # 空中机器人模型（四旋翼）
│       └── rmuc2026_competition/  # 赛场地图模型
└── rmoss_gazebo/
    └── worlds/                    # 世界文件同步副本
```

---

## sim_config.py 参数详解

### 1. 赛场参数 (`FIELD`)

```python
FIELD = {
    "model_uri": "model://rmuc2026_competition",
    "x_offset": 0,    # 赛场X方向偏移 (米)
    "y_offset": 0,    # 赛场Y方向偏移 (米)
    "z_offset": 0,    # 赛场Z方向偏移 (米), 地面已校正为Z=0
}
```

- 赛场范围约 X: [-14.5, 14.5]m, Y: [-8, 8]m
- 赛场地面在 Z=0，高地最高约 1.3m
- 一般不需要修改偏移值

### 2. 雷达站参数 (`RADAR_STATION`)

```python
RADAR_STATION = {
    "model_uri": "model://radar_station",
    "name": "radar_station_blue",
    # 世界坐标
    "x": -15.0,       # 赛场X边缘外侧
    "y": 0.5,         # 略向左偏移
    "z": 0.0,         # 基座底部在地面
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,       # 面向+X (面向赛场)
    # 传感器相对位置
    "lidar_local_x": 0.5,
    "lidar_local_y": 0.0,
    "lidar_local_z": 3.70,
    "lidar_pitch": 0.35,    # ~20° 向下俯仰
    "camera_local_x": 0.5,
    "camera_local_y": 0.0,
    "camera_local_z": 3.78,
    "camera_pitch": 0.35,
}
```

#### 坐标系说明

雷达站面向 +X 方向（面向赛场）：

```
        +Y (左)
         ↑
         |
  -X ----+---→ +X (面向赛场)
  (后方)  |
         ↓
        -Y (右)
```

#### 常用调整

| 操作 | 修改参数 | 说明 |
|------|---------|------|
| 前后移动 | `x` | 增大=靠近赛场，减小=远离赛场 |
| 左右移动 | `y` | 增大=向左，减小=向右 |
| 调整高度 | `z` | 一般保持0（地面） |
| 旋转朝向 | `yaw` | 弧度值，0=面向+X |
| LiDAR俯仰 | `lidar_pitch` | 增大=更向下看，0.35≈20° |
| 相机俯仰 | `camera_pitch` | 通常与LiDAR保持一致 |

### 3. 机器人参数 (`ROBOTS`)

每个机器人是一个字典：

```python
{
    "name": "blue_robot_1",                    # 唯一名称
    "model_uri": "model://rmua19_standard_robot",  # 模型URI
    "x": -12.0, "y": 0.0, "z": 0.0,          # 世界坐标
    "roll": 0, "pitch": 0, "yaw": 0.0,        # 姿态 (弧度)
    "team": "blue",                            # 队伍标识
    "comment": "蓝方启动区 (平地)",              # 备注
}
```

#### 注意事项

- 地面机器人 `z` 设为 0（模型内部已有 0.15m 偏移）
- 空中机器人 `z` 设为飞行高度（如 5.0m）
- 位置必须避开赛场高地和墙壁
- 安全区域：赛场两端启动区附近（|X| > 10m 的区域通常是平地）

#### 赛场安全区域参考

```
  Y
  ↑
  8 ┌─────────────────────────────────────┐
    │                                     │
  4 │  ★蓝方安全区    ⚠高地区域   ★红方安全区│
    │  X∈[-13,-9]     X∈[-5,5]   X∈[9,13] │
  0 │  Y∈[-3,3]                  Y∈[-3,3] │
    │                                     │
 -4 │                                     │
    │                                     │
 -8 └─────────────────────────────────────┘
   -14                 0                  14  → X
```

#### 添加/删除机器人

直接在 `ROBOTS` 列表中增删字典即可。支持的模型：

| 模型 | URI | 说明 |
|------|-----|------|
| 地面步兵 | `model://rmua19_standard_robot` | 麦克纳姆轮底盘 + 云台 |
| 空中机器人 | `model://aerial_robot` | 四旋翼无人机（静态） |

### 4. 物理引擎参数 (`PHYSICS`)

```python
PHYSICS = {
    "max_step_size": 0.004,        # 仿真步长 (秒)
    "real_time_factor": 1.0,       # 实时倍率 (1.0=实时)
    "real_time_update_rate": 250,  # 更新频率 (Hz)
}
```

- 降低 `real_time_factor` 可在性能不足时减慢仿真
- 增大 `max_step_size` 可提高性能但降低精度

### 5. GUI 相机参数 (`GUI_CAMERA`)

```python
GUI_CAMERA = {
    "x": -20.0, "y": 0.0, "z": 12.0,  # 初始视角位置
    "roll": 0.0, "pitch": 0.7, "yaw": 0.0,  # 初始视角姿态
}
```

---

## 模型修改指南

### 雷达站模型

文件：`src/rmoss_gz_resources/resource/models/radar_station/model.sdf`

结构层级：
```
base_link (原点，地面)
├── pillar_fl/fr/bl/br (4根支撑柱，高2.5m)
├── platform (平台面板，高1.525m)
│   └── tripod_center (中心支柱)
│       └── livox_hap (LiDAR，高2.5m)
│           └── camera_link (相机，高2.58m)
```

#### 调整传感器参数

LiDAR 参数在 `livox_hap` link 的 `<sensor>` 中：

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `update_rate` | 10 Hz | 扫描频率 |
| `horizontal/samples` | 600 | 水平采样点数 |
| `horizontal/min_angle` | -1.0472 (-60°) | 水平FOV左边界 |
| `horizontal/max_angle` | 1.0472 (60°) | 水平FOV右边界 |
| `vertical/samples` | 64 | 垂直采样线数 |
| `vertical/min_angle` | -0.3927 (-22.5°) | 垂直FOV下边界 |
| `vertical/max_angle` | 0.3927 (22.5°) | 垂直FOV上边界 |
| `range/max` | 150.0 m | 最大探测距离 |
| `noise/stddev` | 0.01 m | 噪声标准差 |

相机参数在 `camera_link` 的 `<sensor>` 中：

| 参数 | 当前值 | 说明 |
|------|--------|------|
| `update_rate` | 30 Hz | 帧率 |
| `horizontal_fov` | 1.2 rad (69°) | 水平视场角 |
| `image/width` | 1280 | 图像宽度 |
| `image/height` | 1024 | 图像高度 |

### 赛场地图模型

文件：`src/rmoss_gz_resources/resource/models/rmuc2026_competition/model.sdf`

- 使用 STL 网格文件，单位毫米，缩放 0.001
- Z 偏移 1.441m 使行走地面在 Z=0
- Y 偏移 -1.625m 使赛场 Y 轴中心在 0
- 如需更换赛场 STL，替换同目录下的 `RMUC2026.stl` 文件

### 空中机器人模型

文件：`src/rmoss_gz_resources/resource/models/aerial_robot/model.sdf`

- 静态模型（`<static>true</static>`），不受物理引擎影响
- 尺寸：轴距约 0.55m，整体约 0.55m × 0.55m × 0.18m
- 包含：机身、4个旋翼臂、电机座、螺旋桨圆盘、起落架、底部相机、LED指示灯

---

## ROS2 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | LiDAR 点云 |
| `/radar/camera` | `sensor_msgs/Image` | 相机图像 |

---

## 常见问题

### 机器人嵌入地形
调整 `sim_config.py` 中对应机器人的 `x`, `y` 坐标，移到平地区域。参考上方安全区域图。

### 仿真卡顿
- 减少机器人数量
- 降低 LiDAR 采样点数（`samples`）
- 降低相机分辨率
- 设置 `real_time_factor` < 1.0

### 模型修改后不生效
```bash
colcon build --symlink-install --packages-select rmoss_gz_resources
source install/setup.bash
```

### 需要重新生成世界文件
```bash
python3 src/radar_sim/config/generate_world.py
