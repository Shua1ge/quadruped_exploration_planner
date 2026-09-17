#!/usr/bin/env python3
"""Publish SDF collision geometry as a transient global point cloud."""

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from explorer_core.sdf_world_geometry import load_sdf_world


class SdfMapPublisher(Node):
    def __init__(self):
        super().__init__("sdf_map_pub")
        world_file = str(self.declare_parameter("world_file", "").value)
        frame_id = str(self.declare_parameter("frame_id", "world").value)
        mode = str(self.declare_parameter("mode", "planar").value)
        spacing = float(self.declare_parameter("resolution", 0.20).value)
        max_slope = float(self.declare_parameter("max_slope_degrees", 15.0).value)
        obstacle_height = float(self.declare_parameter("obstacle_height", 2.5).value)
        vertical_spacing = float(self.declare_parameter("vertical_spacing", 0.20).value)
        recenter = bool(self.declare_parameter("recenter", True).value)
        if not world_file or not Path(world_file).is_file():
            raise RuntimeError("world_file must reference an existing SDF world")
        self.frame_id = frame_id
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            PointCloud2, "/map_generator/global_cloud", qos)
        self.get_logger().info(f"Loading SDF collision geometry from {world_file}")
        cloud = load_sdf_world(Path(world_file), spacing, mode, max_slope,
                               obstacle_height, vertical_spacing, recenter)
        header = Header()
        header.frame_id = frame_id
        self.message = point_cloud2.create_cloud_xyz32(header, cloud.points)
        bounds = cloud.source_bounds
        self.get_logger().info(
            f"SDF map ready: {len(cloud.points)} points, "
            f"triangles={cloud.triangle_count}, boxes={cloud.box_count}, "
            f"source_bounds={bounds}, translation={cloud.translation}")
        self.publish_map()
        self.timer = self.create_timer(5.0, self.publish_map)

    def publish_map(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = SdfMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
