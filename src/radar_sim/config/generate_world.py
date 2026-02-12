#!/usr/bin/env python3
"""
RMUC2026 雷达仿真 - 世界文件生成器

根据 sim_config.py 中的参数生成 Gazebo Ignition 世界文件 (.sdf)

用法:
  python3 generate_world.py                 # 生成到默认位置
  python3 generate_world.py -o output.sdf   # 生成到指定文件
  python3 generate_world.py --print         # 打印到标准输出
"""

import argparse
import os
import sys
import textwrap

# 导入配置
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_config import (
    FIELD, RADAR_STATION, ROBOTS, PHYSICS, GUI_CAMERA,
    get_lidar_world_pose, get_camera_world_pose, print_config,
)


def pose_str(x, y, z, roll=0, pitch=0, yaw=0):
    """生成 SDF pose 字符串"""
    return f"{x} {y} {z} {roll} {pitch} {yaw}"


def generate_world_sdf():
    """根据配置生成完整的世界SDF"""

    # 机器人部分
    robots_xml = ""
    for r in ROBOTS:
        team_label = "蓝方" if r["team"] == "blue" else "红方"
        robots_xml += f"""
        <!-- {team_label} - {r['comment']} -->
        <include>
            <uri>{r['model_uri']}</uri>
            <pose>{pose_str(r['x'], r['y'], r['z'], r['roll'], r['pitch'], r['yaw'])}</pose>
            <name>{r['name']}</name>
        </include>
"""

    rs = RADAR_STATION
    sdf = f"""<?xml version="1.0" ?>
<!--
  RMUC2026 雷达仿真世界 - 由 generate_world.py 自动生成
  
  赛场尺寸: ~29m(X) × 16m(Y), 中心在原点
  赛场地面: Z=0
  
  雷达站: 按规则手册V1.3.0
    - 位于蓝方宽边(X={rs['x']})边缘中心(Y={rs['y']})
    - 基座底部在地面(Z={rs['z']}), 平台高2.5m
    - 传感器面向赛场(+X方向)
  
  机器人: {len(ROBOTS)}个, 站在赛场地面(Z=0)上
-->
<sdf version="1.7">
    <world name="rmuc2026_radar_sim">

        <!-- ==================== 系统插件 ==================== -->
        <plugin filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics"></plugin>
        <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster"></plugin>
        <plugin filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands"></plugin>
        <plugin filename="libignition-gazebo-sensors-system.so"
            name="ignition::gazebo::systems::Sensors">
            <render_engine>ogre2</render_engine>
        </plugin>
        <plugin filename="libignition-gazebo-imu-system.so"
            name="ignition::gazebo::systems::Imu"></plugin>
        <plugin filename="libignition-gazebo-contact-system.so"
            name="ignition::gazebo::systems::Contact"></plugin>

        <!-- ==================== 物理引擎 ==================== -->
        <physics name="default_physics" type="ode">
            <max_step_size>{PHYSICS['max_step_size']}</max_step_size>
            <real_time_factor>{PHYSICS['real_time_factor']}</real_time_factor>
            <real_time_update_rate>{PHYSICS['real_time_update_rate']}</real_time_update_rate>
        </physics>

        <!-- ==================== 场景 ==================== -->
        <scene>
            <ambient>0.6 0.6 0.6 1</ambient>
            <background>0.7 0.8 0.95 1</background>
            <shadows>true</shadows>
        </scene>

        <!-- ==================== GUI ==================== -->
        <gui fullscreen="0">
            <plugin filename="GzScene3D" name="3D View">
                <ignition-gui>
                    <title>3D View</title>
                    <property type="bool" key="showTitleBar">false</property>
                    <property type="string" key="state">docked</property>
                </ignition-gui>
                <engine>ogre2</engine>
                <scene>scene</scene>
                <camera_pose>{pose_str(GUI_CAMERA['x'], GUI_CAMERA['y'], GUI_CAMERA['z'], GUI_CAMERA['roll'], GUI_CAMERA['pitch'], GUI_CAMERA['yaw'])}</camera_pose>
            </plugin>
            <plugin filename="WorldControl" name="World control">
                <ignition-gui>
                    <title>World control</title>
                    <property type="bool" key="showTitleBar">false</property>
                    <property type="bool" key="resizable">false</property>
                    <property type="double" key="height">72</property>
                    <property type="double" key="width">121</property>
                    <property type="double" key="z">1</property>
                    <property type="string" key="state">floating</property>
                    <anchors target="3D View">
                        <line own="left" target="left"/>
                        <line own="bottom" target="bottom"/>
                    </anchors>
                </ignition-gui>
                <play_pause>true</play_pause>
                <step>true</step>
                <start_paused>false</start_paused>
            </plugin>
            <plugin filename="WorldStats" name="World stats">
                <ignition-gui>
                    <title>World stats</title>
                    <property type="bool" key="showTitleBar">false</property>
                    <property type="bool" key="resizable">false</property>
                    <property type="double" key="height">110</property>
                    <property type="double" key="width">290</property>
                    <property type="double" key="z">1</property>
                    <property type="string" key="state">floating</property>
                    <anchors target="3D View">
                        <line own="right" target="right"/>
                        <line own="bottom" target="bottom"/>
                    </anchors>
                </ignition-gui>
                <sim_time>true</sim_time>
                <real_time>true</real_time>
                <real_time_factor>true</real_time_factor>
                <iterations>true</iterations>
            </plugin>
            <plugin filename="EntityTree" name="Entity tree">
                <ignition-gui>
                    <property type="string" key="state">docked</property>
                </ignition-gui>
            </plugin>
        </gui>

        <!-- ==================== 光照 ==================== -->
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 30 0 0 0</pose>
            <diffuse>0.9 0.9 0.9 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.3</constant>
                <linear>0.005</linear>
                <quadratic>0.0001</quadratic>
            </attenuation>
            <direction>-0.5 0.3 -0.9</direction>
        </light>
        <light type="directional" name="fill_light">
            <cast_shadows>false</cast_shadows>
            <pose>0 0 25 0 0 0</pose>
            <diffuse>0.4 0.4 0.4 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <direction>0.5 -0.3 -0.7</direction>
        </light>

        <!-- ==================== 地面 ==================== -->
        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                    <geometry><plane><normal>0 0 1</normal></plane></geometry>
                </collision>
                <visual name="visual">
                    <geometry><plane><normal>0 0 1</normal><size>40 25</size></plane></geometry>
                    <material>
                        <ambient>0.55 0.55 0.55 1</ambient>
                        <diffuse>0.6 0.6 0.6 1</diffuse>
                    </material>
                </visual>
            </link>
        </model>

        <!-- ==================== 赛场地图 ==================== -->
        <include>
            <uri>{FIELD['model_uri']}</uri>
            <pose>{pose_str(FIELD['x_offset'], FIELD['y_offset'], FIELD['z_offset'])}</pose>
        </include>

        <!-- ==================== 机器人 ==================== -->
{robots_xml}
        <!-- ==================== 雷达站 ==================== -->
        <include>
            <uri>{rs['model_uri']}</uri>
            <pose>{pose_str(rs['x'], rs['y'], rs['z'], rs['roll'], rs['pitch'], rs['yaw'])}</pose>
            <name>{rs['name']}</name>
        </include>

    </world>
</sdf>
"""
    return sdf


def main():
    parser = argparse.ArgumentParser(
        description="RMUC2026 雷达仿真世界文件生成器"
    )
    parser.add_argument(
        "-o", "--output",
        help="输出文件路径 (默认: rmoss_gazebo/worlds/rmuc2026_radar_sim.sdf)",
    )
    parser.add_argument(
        "--print", action="store_true", dest="print_only",
        help="仅打印到标准输出, 不写入文件",
    )
    parser.add_argument(
        "--config", action="store_true",
        help="打印当前配置信息",
    )
    args = parser.parse_args()

    if args.config:
        print_config()
        return

    sdf_content = generate_world_sdf()

    if args.print_only:
        print(sdf_content)
        return

    # 默认输出路径
    if args.output:
        output_path = args.output
    else:
        # 输出到 radar_sim/worlds/ 目录
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # radar_sim/config/ → 上一级到 radar_sim/
        package_dir = os.path.dirname(script_dir)
        output_path = os.path.join(
            package_dir, "worlds", "rmuc2026_radar_sim.sdf"
        )

    # 确保输出目录存在
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with open(output_path, "w") as f:
        f.write(sdf_content)

    print(f"✅ 世界文件已生成: {output_path}")

    # 同步到 rmoss_gazebo/worlds/ (如果存在)
    src_dir = os.path.dirname(os.path.dirname(script_dir))
    alt_path = os.path.join(
        src_dir, "rmoss_gazebo", "worlds", "rmuc2026_radar_sim.sdf"
    )
    if os.path.isdir(os.path.dirname(alt_path)):
        with open(alt_path, "w") as f:
            f.write(sdf_content)
        print(f"✅ 同步副本: {alt_path}")
    print()
    print_config()
    print()
    print("下次修改机器人/雷达位置, 只需编辑 sim_config.py 然后重新运行此脚本")


if __name__ == "__main__":
    main()
