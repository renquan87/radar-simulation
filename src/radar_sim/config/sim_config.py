#!/usr/bin/env python3
"""
RMUC2026 雷达仿真 - 参数配置文件

修改此文件中的参数后, 运行 generate_world.py 即可重新生成世界文件
或直接运行本文件: python3 sim_config.py (会打印当前配置)
"""

import math

# ==================== 赛场参数 ====================
FIELD = {
    "model_uri": "model://rmuc2026_competition",
    # STL偏移 (原始STL在mm单位, scale后的修正值)
    "x_offset": 0,
    "y_offset": 0,
    "z_offset": 0,  # 地面已校正为Z=0
}

# ==================== 雷达站参数 ====================
# 按规则手册 V1.3.0:
#   - 赛场宽边边缘中心, 平台3.4m×1.16m, 高2.5m
#   - 蓝方雷达在X负方向边缘
RADAR_STATION = {
    "model_uri": "model://radar_station",
    "name": "radar_station_blue",
    # 世界坐标系中的位置
    "x": -14.5,       # 赛场X边缘 (赛场范围约 ±14.575m)
    "y": 0.0,          # 与战场中心轴线对齐
    "z": 0.0,          # 基座底部在地面
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,        # 面向+X (面向赛场)
    # 传感器在模型内的相对位置 (用于TF发布)
    "lidar_local_x": 0.5,
    "lidar_local_y": 0.0,
    "lidar_local_z": 3.70,
    "lidar_pitch": 0.35,    # ~20° 向下
    "camera_local_x": 0.5,
    "camera_local_y": 0.0,
    "camera_local_z": 3.78,
    "camera_pitch": 0.35,   # 与LiDAR同角度
}

# ==================== 机器人参数 ====================
# z=0: 机器人模型内部已有z=0.15偏移, 轮子底部恰好在地面
ROBOTS = [
    # ---- 蓝方 ----
    {
        "name": "blue_robot_1",
        "model_uri": "model://rmua19_standard_robot",
        "x": -10.0, "y": 3.0, "z": 0.0,
        "roll": 0, "pitch": 0, "yaw": 0.5,
        "team": "blue", "comment": "蓝方半场前侧",
    },
    {
        "name": "blue_robot_2",
        "model_uri": "model://rmua19_standard_robot",
        "x": -3.0, "y": -2.0, "z": 0.0,
        "roll": 0, "pitch": 0, "yaw": 0.3,
        "team": "blue", "comment": "中路偏蓝方",
    },
    {
        "name": "blue_robot_3",
        "model_uri": "model://rmua19_standard_robot",
        "x": -7.0, "y": -5.0, "z": 0.0,
        "roll": 0, "pitch": 0, "yaw": 1.0,
        "team": "blue", "comment": "蓝方半场侧路",
    },
    # ---- 红方 ----
    {
        "name": "red_robot_1",
        "model_uri": "model://rmua19_standard_robot",
        "x": 10.0, "y": -3.0, "z": 0.0,
        "roll": 0, "pitch": 0, "yaw": -2.6,
        "team": "red", "comment": "红方半场前侧",
    },
    {
        "name": "red_robot_2",
        "model_uri": "model://rmua19_standard_robot",
        "x": 3.0, "y": 2.0, "z": 0.0,
        "roll": 0, "pitch": 0, "yaw": -2.8,
        "team": "red", "comment": "中路偏红方",
    },
    {
        "name": "red_robot_3",
        "model_uri": "model://rmua19_standard_robot",
        "x": 7.0, "y": 5.0, "z": 0.0,
        "roll": 0, "pitch": 0, "yaw": -2.0,
        "team": "red", "comment": "红方半场侧路",
    },
]

# ==================== 物理引擎参数 ====================
PHYSICS = {
    "max_step_size": 0.004,
    "real_time_factor": 1.0,
    "real_time_update_rate": 250,
}

# ==================== GUI相机参数 ====================
GUI_CAMERA = {
    "x": -20.0, "y": 0.0, "z": 12.0,
    "roll": 0.0, "pitch": 0.7, "yaw": 0.0,
}


def get_lidar_world_pose():
    """计算LiDAR在世界坐标系中的位置"""
    rs = RADAR_STATION
    return {
        "x": rs["x"] + rs["lidar_local_x"],
        "y": rs["y"] + rs["lidar_local_y"],
        "z": rs["z"] + rs["lidar_local_z"],
        "roll": rs["roll"],
        "pitch": rs["lidar_pitch"],
        "yaw": rs["yaw"],
    }


def get_camera_world_pose():
    """计算Camera在世界坐标系中的位置"""
    rs = RADAR_STATION
    return {
        "x": rs["x"] + rs["camera_local_x"],
        "y": rs["y"] + rs["camera_local_y"],
        "z": rs["z"] + rs["camera_local_z"],
        "roll": rs["roll"],
        "pitch": rs["camera_pitch"],
        "yaw": rs["yaw"],
    }


def print_config():
    """打印当前配置摘要"""
    print("=" * 60)
    print("  RMUC2026 雷达仿真配置")
    print("=" * 60)
    
    rs = RADAR_STATION
    print(f"\n📡 雷达站: ({rs['x']}, {rs['y']}, {rs['z']})")
    print(f"   朝向: yaw={rs['yaw']:.2f} rad ({math.degrees(rs['yaw']):.1f}°)")
    
    lp = get_lidar_world_pose()
    print(f"\n🔴 LiDAR 世界坐标: ({lp['x']}, {lp['y']}, {lp['z']})")
    print(f"   俯仰角: {lp['pitch']:.2f} rad ({math.degrees(lp['pitch']):.1f}°)")
    
    cp = get_camera_world_pose()
    print(f"\n📷 Camera 世界坐标: ({cp['x']}, {cp['y']}, {cp['z']})")
    print(f"   俯仰角: {cp['pitch']:.2f} rad ({math.degrees(cp['pitch']):.1f}°)")
    
    print(f"\n🤖 机器人 ({len(ROBOTS)} 个):")
    for r in ROBOTS:
        team_icon = "🔵" if r["team"] == "blue" else "🔴"
        print(f"   {team_icon} {r['name']}: ({r['x']}, {r['y']}, {r['z']}) - {r['comment']}")
    
    print(f"\n⚙️  物理: step={PHYSICS['max_step_size']}s, "
          f"RTF={PHYSICS['real_time_factor']}, "
          f"rate={PHYSICS['real_time_update_rate']}Hz")
    print("=" * 60)


if __name__ == "__main__":
    print_config()
