#!/usr/bin/env python3
"""
保存仿真点云为 PCD 文件，用于离线开发（如空中机器人识别）

用法:
  # 先启动仿真
  ros2 launch radar_sim radar_sim.launch.py

  # 另一个终端运行此脚本，默认保存5帧
  python3 save_pointcloud_demo.py

  # 指定保存帧数和输出目录
  python3 save_pointcloud_demo.py --frames 10 --output ./demo_pcds

  # 也可以同时保存相机图像
  python3 save_pointcloud_demo.py --frames 5 --save-image

输出:
  demo_pcds/
    frame_000.pcd
    frame_001.pcd
    ...
    frame_000.png  (如果 --save-image)
"""

import argparse
import os
import sys
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField, Image


def pointcloud2_to_xyz(msg: PointCloud2):
    """从 PointCloud2 消息中提取 (x, y, z) 点列表"""
    # 找到 x, y, z 字段的偏移
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ("x", "y", "z")):
        return []

    ox = field_map["x"].offset
    oy = field_map["y"].offset
    oz = field_map["z"].offset
    point_step = msg.point_step
    data = msg.data

    points = []
    for i in range(msg.width * msg.height):
        base = i * point_step
        x = struct.unpack_from("f", data, base + ox)[0]
        y = struct.unpack_from("f", data, base + oy)[0]
        z = struct.unpack_from("f", data, base + oz)[0]
        # 过滤无效点
        if x == 0.0 and y == 0.0 and z == 0.0:
            continue
        import math
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            continue
        if math.isinf(x) or math.isinf(y) or math.isinf(z):
            continue
        points.append((x, y, z))
    return points


def save_pcd(filepath: str, points: list):
    """保存点列表为 ASCII PCD 文件"""
    with open(filepath, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def save_image_png(filepath: str, msg: Image):
    """保存 ROS Image 消息为 PNG（需要 cv_bridge 或手动处理）"""
    try:
        import cv2
        import numpy as np
        # 根据编码解析
        if msg.encoding in ("rgb8", "RGB8"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding in ("bgr8", "BGR8"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding in ("mono8", "MONO8"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        else:
            print(f"  [warn] 不支持的图像编码: {msg.encoding}, 跳过保存")
            return False
        cv2.imwrite(filepath, img)
        return True
    except ImportError:
        print("  [warn] 需要 opencv-python 才能保存图像: pip install opencv-python")
        return False


class PointCloudSaver(Node):
    def __init__(self, max_frames: int, output_dir: str, save_image: bool):
        super().__init__("pointcloud_saver")
        self.max_frames = max_frames
        self.output_dir = output_dir
        self.save_image = save_image
        self.pc_count = 0
        self.img_count = 0
        self.latest_image = None

        os.makedirs(output_dir, exist_ok=True)

        # 使用 best effort QoS 匹配 Gazebo 传感器输出
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pc_sub = self.create_subscription(
            PointCloud2,
            "/livox/lidar/points",
            self.pc_callback,
            qos,
        )

        if save_image:
            self.img_sub = self.create_subscription(
                Image,
                "/radar/camera",
                self.img_callback,
                qos,
            )

        self.get_logger().info(
            f"等待点云数据... (将保存 {max_frames} 帧到 {output_dir}/)"
        )

    def img_callback(self, msg: Image):
        self.latest_image = msg

    def pc_callback(self, msg: PointCloud2):
        if self.pc_count >= self.max_frames:
            return

        points = pointcloud2_to_xyz(msg)
        if len(points) == 0:
            self.get_logger().warn("收到空点云，跳过")
            return

        # 保存 PCD
        pcd_path = os.path.join(self.output_dir, f"frame_{self.pc_count:03d}.pcd")
        save_pcd(pcd_path, points)
        self.get_logger().info(
            f"[{self.pc_count + 1}/{self.max_frames}] 保存 {len(points)} 个点 → {pcd_path}"
        )

        # 保存对应图像
        if self.save_image and self.latest_image is not None:
            img_path = os.path.join(self.output_dir, f"frame_{self.pc_count:03d}.png")
            if save_image_png(img_path, self.latest_image):
                self.get_logger().info(f"  保存图像 → {img_path}")

        self.pc_count += 1

        if self.pc_count >= self.max_frames:
            self.get_logger().info(
                f"已保存 {self.max_frames} 帧点云到 {self.output_dir}/，完成！"
            )
            # 延迟一点再退出，确保日志输出
            self.create_timer(0.5, lambda: rclpy.shutdown())


def main():
    parser = argparse.ArgumentParser(description="保存仿真点云为 PCD 文件")
    parser.add_argument(
        "--frames", type=int, default=5, help="保存帧数 (默认: 5)"
    )
    parser.add_argument(
        "--output", type=str, default="demo_pcds", help="输出目录 (默认: demo_pcds)"
    )
    parser.add_argument(
        "--save-image", action="store_true", help="同时保存相机图像"
    )
    args = parser.parse_args()

    rclpy.init()
    node = PointCloudSaver(args.frames, args.output, args.save_image)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"中断，已保存 {node.pc_count} 帧")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
