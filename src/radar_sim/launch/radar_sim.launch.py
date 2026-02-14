"""
RMUC2026 雷达仿真 - 一键启动
启动 Gazebo Ignition, ros_gz_bridge, 静态TF, RViz2

传感器话题:
  - /livox/lidar/points  (PointCloud2)
  - /radar/camera/image  (Image)
  - /radar/camera/camera_info (CameraInfo)

坐标系:
  - world: 世界坐标系 (Gazebo)
  - radar_station_base: 雷达站基座 (世界文件中的pose)
  - livox_hap_lidar: 激光雷达坐标系
  - radar_camera: 相机坐标系

注意: Ignition Gazebo中传感器的frame_id为scoped name格式:
  radar_station_blue::livox_hap::livox_hap_lidar
  radar_station_blue::camera_link::radar_camera
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ======================== 参数声明 ========================
    use_rviz = LaunchConfiguration("use_rviz")
    world_name = LaunchConfiguration("world_name")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="是否启动 RViz"
    )
    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value="rmuc2026_radar_sim",
        description="Gazebo世界名称",
    )

    # ======================== 路径 ========================
    radar_sim_dir = get_package_share_directory("radar_sim")
    rviz_config = os.path.join(radar_sim_dir, "rviz", "radar_sim.rviz")
    world_file = os.path.join(
        radar_sim_dir, "worlds", "rmuc2026_radar_sim.sdf"
    )

    # ======================== 清理残留 Gazebo 进程 ========================
    # 杀掉可能残留的 ign/gz 进程，避免黑屏和 Start 按钮卡死
    cleanup_gz = ExecuteProcess(
        cmd=["bash", "-c",
             "killall -9 ign gazebo ruby gz 2>/dev/null; sleep 1; echo '[cleanup] Done'"],
        output="screen",
    )

    # ======================== Gazebo Ignition ========================
    # Run server and GUI as separate processes to work around an
    # ign-gazebo-6 (Fortress) bug where the GUI's SceneManager crashes
    # with "Another item already exists" when both run in the same process.
    # Server starts immediately after cleanup; GUI connects after a short delay.
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": "-s " + world_file}.items(),
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": "-g"}.items(),
    )

    # ======================== ros_gz_bridge ========================
    # Ignition topic → ROS2 topic 桥接
    # 传感器SDF中设置了显式 <topic>, Ignition直接使用, 无world/model前缀
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            # 激光雷达点云
            "/livox/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            # 相机图像 (Ignition topic: /radar/camera)
            "/radar/camera@sensor_msgs/msg/Image[ignition.msgs.Image",
            # 相机内参 (Ignition topic: /radar/camera/camera_info)
            "/radar/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        output="screen",
    )

    # ======================== 静态 TF ========================
    # 雷达站在世界中的位置 (来自世界文件中的pose)
    # radar_station_blue 基座: (-14.5, 0, 0), 朝向+X
    #
    # Ignition中传感器frame_id为scoped name:
    #   radar_station_blue::livox_hap::livox_hap_lidar
    #   radar_station_blue::camera_link::radar_camera
    #
    # 我们发布TF链:
    #   world → radar_station_base → livox_hap_lidar (用简短名)
    #                               → radar_camera
    #
    # 同时为Ignition的scoped frame_id建立别名TF:
    #   world → radar_station_blue::livox_hap::livox_hap_lidar
    #   world → radar_station_blue::camera_link::radar_camera

    # --- 雷达站基座 (世界坐标系下) ---
    radar_station_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="radar_station_tf",
        arguments=[
            "--x", "-14.5",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "radar_station_base",
        ],
        output="screen",
    )

    # --- Ignition LiDAR frame (scoped name) → 世界坐标系 ---
    # LiDAR在雷达站模型内: pose (1.6, 0, 2.5), pitch=0rad, yaw=0.262rad
    # 世界坐标系: base(-14.575,5.33,0) + local(1.6,0,2.5) = (-12.975, 5.33, 2.5)
    # 注意: Ignition Gazebo中frame_id为斜杠格式: radar_station_blue/livox_hap/livox_hap_lidar
    lidar_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_frame_tf",
        arguments=[
            "--x", "-12.975",
            "--y", "5.33",
            "--z", "2.5",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0.262",
            "--frame-id", "world",
            "--child-frame-id", "radar_station_blue/livox_hap/livox_hap_lidar",
        ],
        output="screen",
    )

    # --- Ignition Camera frame (scoped name) → 世界坐标系 ---
    # Camera在雷达站模型内: pose (1.6, 0, 2.58), pitch=0rad, yaw=0.262rad
    # 世界坐标系: base(-14.575,5.33,0) + local(1.6,0,2.58) = (-12.975, 5.33, 2.58)
    # 注意: Ignition Gazebo中frame_id为斜杠格式: radar_station_blue/camera_link/radar_camera
    camera_frame_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_frame_tf",
        arguments=[
            "--x", "-12.975",
            "--y", "5.33",
            "--z", "2.58",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0.262",
            "--frame-id", "world",
            "--child-frame-id", "radar_station_blue/camera_link/radar_camera",
        ],
        output="screen",
    )

    # --- 简短名称别名 TF (方便后续算法使用) ---
    lidar_alias_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_alias_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "radar_station_blue/livox_hap/livox_hap_lidar",
            "--child-frame-id", "livox_hap_lidar",
        ],
        output="screen",
    )

    camera_alias_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_alias_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "radar_station_blue/camera_link/radar_camera",
            "--child-frame-id", "radar_camera",
        ],
        output="screen",
    )

    # ======================== RViz2 ========================
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
        output="screen",
        # 确保RViz使用仿真时间，避免时间同步问题
        parameters=[{"use_sim_time": True}],
    )

    # ======================== 延迟启动桥接和TF ========================
    # Server 从 2s 启动, GUI 从 6s, 桥接从 12s (给传感器足够初始化时间)
    delayed_bridge_and_tf = TimerAction(
        period=12.0,
        actions=[
            bridge,
            radar_station_tf,
            lidar_frame_tf,
            camera_frame_tf,
            lidar_alias_tf,
            camera_alias_tf,
        ],
    )

    # RViz最后启动, 确保TF树完全建立
    delayed_rviz = TimerAction(
        period=16.0,
        actions=[rviz2],
    )

    # Server 在清理后延迟启动
    delayed_server = TimerAction(
        period=2.0,
        actions=[gazebo_server],
    )

    # GUI 在 server 之后启动
    delayed_gui_after_server = TimerAction(
        period=6.0,
        actions=[gazebo_gui],
    )

    return LaunchDescription(
        [
            declare_use_rviz,
            declare_world_name,
            cleanup_gz,
            delayed_server,
            delayed_gui_after_server,
            delayed_bridge_and_tf,
            delayed_rviz,
        ]
    )
