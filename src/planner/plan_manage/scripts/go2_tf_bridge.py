#!/usr/bin/env python3
"""Static TF publisher to connect Go2 TF tree for RViz visualization."""

import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class Go2TFBridge(Node):
    def __init__(self):
        super().__init__('go2_tf_bridge')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static transforms to connect the TF tree
        transforms = []
        
        # Connect go2/base_footprint to base (zero offset)
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'go2/base_footprint'
        t1.child_frame_id = 'base'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.w = 1.0
        transforms.append(t1)

        lidar = TransformStamped()
        lidar.header.stamp = self.get_clock().now().to_msg()
        lidar.header.frame_id = 'base'
        lidar.child_frame_id = 'go2/base/exploration_lidar'
        with open(os.path.join(get_package_share_directory('go2_description'),
                               'config', 'mid360.yaml'), encoding='utf-8') as stream:
            mount = yaml.safe_load(stream)['mount']
        lidar.transform.translation.x = mount['x']
        lidar.transform.translation.y = mount['y']
        lidar.transform.translation.z = mount['z']
        # Unitree Go2 kit has only a +13deg pitch mounting rotation.
        lidar.transform.rotation.y = math.sin(mount['pitch'] / 2.)
        lidar.transform.rotation.w = math.cos(mount['pitch'] / 2.)
        transforms.append(lidar)
        
        # Connect world to go2/odom (fixed at origin for now)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'world'
        t2.child_frame_id = 'go2/odom'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0
        transforms.append(t2)
        
        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            'Published static TF: world→go2/odom, go2/base_footprint→base, '
            'base→go2/base/exploration_lidar')


def main(args=None):
    rclpy.init(args=args)
    node = Go2TFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
