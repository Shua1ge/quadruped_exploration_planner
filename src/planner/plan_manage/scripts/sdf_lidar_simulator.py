#!/usr/bin/env python3
"""Simulate local LiDAR returns directly from SDF collision geometry.

The SDF/DAE world is loaded into a private voxel acceleration structure.  The
node publishes only local ray hits; no global truth cloud or PCD representation
is exposed to the planning stack.
"""

import math
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header

from explorer_core.sdf_world_geometry import load_sdf_world


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


class SdfLidarSimulator(Node):
    def __init__(self):
        super().__init__("sdf_lidar_simulator")
        world_file = str(self.declare_parameter("world_file", "").value)
        self.frame_id = str(self.declare_parameter("frame_id", "world").value)
        self.resolution = float(self.declare_parameter("resolution", 0.20).value)
        self.max_range = float(self.declare_parameter("max_range", 7.5).value)
        self.min_range = float(self.declare_parameter("min_range", 0.35).value)
        self.horizontal_rays = int(
            self.declare_parameter("horizontal_rays", 720).value)
        self.vertical_layers = int(
            self.declare_parameter("vertical_layers", 7).value)
        self.vertical_fov = math.radians(float(
            self.declare_parameter("vertical_fov_degrees", 24.0).value))
        self.sensor_height = float(
            self.declare_parameter("sensor_height", 0.0).value)
        sensing_rate = float(self.declare_parameter("sensing_rate", 10.0).value)
        recenter = bool(self.declare_parameter("recenter", True).value)
        self.collision_enabled = bool(
            self.declare_parameter("collision_check_enable", True).value)
        self.collision_radius = float(
            self.declare_parameter("collision_radius", 0.35).value)
        self.collision_min_relative_z = float(
            self.declare_parameter("collision_min_relative_z", -0.15).value)
        self.collision_max_relative_z = float(
            self.declare_parameter("collision_max_relative_z", 0.55).value)
        if not world_file or not Path(world_file).is_file():
            raise RuntimeError("world_file must reference an existing SDF world")
        if self.resolution <= 0.0 or self.max_range <= self.min_range:
            raise ValueError("invalid LiDAR resolution or range")
        if self.horizontal_rays < 1 or self.vertical_layers < 1:
            raise ValueError("LiDAR ray counts must be positive")

        self.get_logger().info(
            f"Loading private SDF mesh acceleration map from {world_file}")
        world = load_sdf_world(
            Path(world_file), spacing=self.resolution,
            mode="surface", recenter=recenter)
        self.occupied = {
            (round(x / self.resolution), round(y / self.resolution),
             round(z / self.resolution))
            for x, y, z in world.points}
        self.get_logger().info(
            f"SDF mesh ready: voxels={len(self.occupied)}, "
            f"triangles={world.triangle_count}, boxes={world.box_count}, "
            "global truth remains private")

        self.pose: Optional[Tuple[float, float, float, float]] = None
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/quad_0/cloud", qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(
            Odometry, "/quad_0/lidar_pose", qos_profile_sensor_data)
        self.collision_pub = self.create_publisher(
            Bool, "/simulation/collision", 10)
        self.create_subscription(
            Odometry, "/quad_0/body_pose", self.pose_callback,
            qos_profile_sensor_data)
        self.create_timer(1.0 / max(0.1, sensing_rate), self.render_scan)

    def pose_callback(self, message: Odometry):
        pose = message.pose.pose
        orientation = pose.orientation
        self.pose = (
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z),
            yaw_from_quaternion(
                orientation.x, orientation.y, orientation.z, orientation.w))
        lidar_pose = Odometry()
        lidar_pose.header = message.header
        lidar_pose.header.frame_id = self.frame_id
        lidar_pose.child_frame_id = "lidar"
        lidar_pose.pose = message.pose
        lidar_pose.twist = message.twist
        lidar_pose.pose.pose.position.z += self.sensor_height
        self.pose_pub.publish(lidar_pose)

    def voxel_occupied(self, x: float, y: float, z: float) -> bool:
        key = (round(x / self.resolution), round(y / self.resolution),
               round(z / self.resolution))
        return key in self.occupied

    def cast_ray(self, origin, azimuth: float, elevation: float):
        horizontal = math.cos(elevation)
        direction = (horizontal * math.cos(azimuth),
                     horizontal * math.sin(azimuth), math.sin(elevation))
        step = 0.5 * self.resolution
        distance = self.min_range
        while distance < self.max_range:
            point = (origin[0] + distance * direction[0],
                     origin[1] + distance * direction[1],
                     origin[2] + distance * direction[2])
            if self.voxel_occupied(*point):
                return point
            distance += step
        # A point beyond both consumers' configured range encodes a no-return
        # beam.  Explorer ignores it as an obstacle and clears its conservative
        # near field; GridMap clips it to max_ray_length and records free space.
        distance = self.max_range + self.resolution
        return (origin[0] + distance * direction[0],
                origin[1] + distance * direction[1],
                origin[2] + distance * direction[2])

    def collision_detected(self, x: float, y: float, z: float) -> bool:
        if not self.collision_enabled:
            return False
        cells = int(math.ceil(self.collision_radius / self.resolution))
        min_dz = math.ceil(self.collision_min_relative_z / self.resolution)
        max_dz = math.floor(self.collision_max_relative_z / self.resolution)
        cx, cy, cz = (round(x / self.resolution),
                      round(y / self.resolution),
                      round(z / self.resolution))
        radius_sq = (self.collision_radius / self.resolution) ** 2
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                if dx * dx + dy * dy > radius_sq:
                    continue
                for dz in range(min_dz, max_dz + 1):
                    if (cx + dx, cy + dy, cz + dz) in self.occupied:
                        return True
        return False

    def render_scan(self):
        if self.pose is None:
            return
        x, y, z, yaw = self.pose
        origin = (x, y, z + self.sensor_height)
        if self.vertical_layers == 1:
            elevations = (0.0,)
        else:
            elevations = tuple(
                -0.5 * self.vertical_fov
                + self.vertical_fov * layer / (self.vertical_layers - 1)
                for layer in range(self.vertical_layers))
        hits = []
        for ray in range(self.horizontal_rays):
            azimuth = yaw - math.pi + 2.0 * math.pi * ray / self.horizontal_rays
            for elevation in elevations:
                hit = self.cast_ray(origin, azimuth, elevation)
                hits.append(hit)
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, hits))
        collision = Bool()
        collision.data = self.collision_detected(x, y, z)
        self.collision_pub.publish(collision)


def main():
    rclpy.init()
    node = SdfLidarSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
