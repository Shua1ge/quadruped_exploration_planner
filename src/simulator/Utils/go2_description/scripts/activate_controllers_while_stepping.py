#!/usr/bin/env python3
"""Activate ros2_control controllers while advancing a paused Gazebo world."""

import argparse
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from controller_manager_msgs.srv import SwitchController


def _world_name(world_file: str) -> str:
    root = ET.parse(world_file).getroot()
    world = root if root.tag == "world" else root.find("world")
    if world is None or not world.get("name"):
        raise ValueError(f"no named <world> element in {world_file}")
    return world.get("name")


def _step_world(world_name: str, steps: int) -> bool:
    result = subprocess.run(
        [
            "ign", "service",
            "-s", f"/world/{world_name}/control",
            "--reqtype", "ignition.msgs.WorldControl",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "5000",
            "--req", f"pause: true multi_step: {steps}",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    return result.returncode == 0 and "data: true" in result.stdout.lower()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", required=True, help="Gazebo SDF world file")
    parser.add_argument("--controller", action="append", required=True)
    parser.add_argument("--service-timeout", type=float, default=30.0)
    parser.add_argument("--switch-timeout", type=float, default=15.0)
    parser.add_argument("--step-batch", type=int, default=25)
    args, ros_args = parser.parse_known_args()

    try:
        world_name = _world_name(args.world)
    except (OSError, ET.ParseError, ValueError) as error:
        print(f"invalid Gazebo world file: {error}", file=sys.stderr)
        return 1

    # launch_ros appends ``--ros-args`` (and may add remaps or parameters) to
    # every Node executable.  Keep the helper's arguments separate and pass
    # the remaining ROS arguments to rclpy instead of rejecting them.
    rclpy.init(args=ros_args)
    node = rclpy.create_node("go2_controller_activator")
    client = node.create_client(
        SwitchController, "/controller_manager/switch_controller")
    try:
        if not client.wait_for_service(timeout_sec=args.service_timeout):
            node.get_logger().error("controller switch service did not become available")
            return 1

        request = SwitchController.Request()
        request.activate_controllers = args.controller
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = int(args.switch_timeout)
        request.timeout.nanosec = int(
            (args.switch_timeout - int(args.switch_timeout)) * 1_000_000_000)

        # call_async queues the request before the first Gazebo step.  The
        # controller manager then gets update cycles in bounded batches until
        # it applies the switch and completes the service response.
        future = client.call_async(request)
        deadline = time.monotonic() + args.switch_timeout + 2.0
        while rclpy.ok() and not future.done():
            rclpy.spin_once(node, timeout_sec=0.05)
            if future.done():
                break
            if time.monotonic() >= deadline:
                node.get_logger().error("controller activation timed out")
                return 1
            if not _step_world(world_name, args.step_batch):
                node.get_logger().error(
                    f"failed to advance paused Gazebo world '{world_name}'")
                return 1

        response = future.result()
        if response is None or not response.ok:
            node.get_logger().error("controller manager rejected controller activation")
            return 1
        node.get_logger().info(
            "controllers activated while Gazebo remained paused")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
