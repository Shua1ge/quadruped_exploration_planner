#!/usr/bin/env python3
"""Drive a staircase of constant forward velocity commands and log base motion.

Determines the minimum effective forward speed of the ONNX locomotion policy:
each level holds vx constant for a fixed duration while base pose and IMU are
logged to CSV.  Designed for the Gazebo Fortress physics branch of
run.launch.py (go2_onnx_policy_node subscribes /quad_0/cmd_vel with a 0.5 s
timeout, so commands are republished continuously).

Usage:
  python3 probe_run.py --levels 0.05,0.1,0.15,0.2,0.3,0.4,0.5 \
      --drive 10.0 --out /tmp/min_speed_probe.csv
"""

import argparse
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class MinSpeedProbe(Node):
    SETTLE = "settle"
    DRIVE = "drive"
    REST = "rest"
    DONE = "done"

    def __init__(self, args):
        super().__init__("min_speed_probe", parameter_overrides=[
            Parameter("use_sim_time", value=args.use_sim_time)])
        self.levels = args.levels
        self.drive_sec = args.drive
        self.settle_sec = args.settle
        self.rest_sec = args.rest
        self.out_path = args.out
        self.rate_hz = args.rate
        self.stand_z_min = args.stand_z_min
        self.use_sim_time = args.use_sim_time
        self.require_ready = args.require_actuation_ready
        self.wait_timeout = args.wait_timeout
        self.started_wall = time.monotonic()
        self.ready_wall = 0.0
        self.actuation_ready = False
        self.abort_reason = None
        if self.require_ready:
            from scan_planner_msgs.msg import LocomotionState
            self.create_subscription(
                LocomotionState, "/robot/locomotion_state", self.ready_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.pose = None
        self.imu_w = (0.0, 0.0, 0.0)
        self.create_subscription(
            Odometry, args.pose_topic, self.pose_callback, qos_profile_sensor_data)
        self.create_subscription(
            Imu, args.imu_topic, self.imu_callback, qos_profile_sensor_data)

        self.rows = []
        self.level_idx = -1
        self.phase = "wait_odom"
        self.phase_start = self.sample_time()
        self.fallen = False
        # Keep readiness/timeout checks alive even if Gazebo pauses /clock.
        # Phase durations and recorded timestamps still use sample_time().
        self.create_timer(1.0 / self.rate_hz, self.tick,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def pose_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def imu_callback(self, msg):
        self.imu_w = (msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z)

    def sample_time(self):
        if self.use_sim_time:
            return self.get_clock().now().nanoseconds * 1e-9
        return time.monotonic()

    def ready_callback(self, msg):
        self.actuation_ready = bool(msg.actuation_ready)
        self.ready_wall = time.monotonic()

    def abort(self, reason):
        self.abort_reason = reason
        self.get_logger().error(reason)
        self.send_cmd(0.0)
        self.begin_phase(self.DONE)

    def send_cmd(self, vx):
        msg = Twist()
        msg.linear.x = vx
        self.cmd_pub.publish(msg)

    def begin_phase(self, name):
        self.phase = name
        self.phase_start = self.sample_time()

    def tick(self):
        now = self.sample_time()
        if self.phase == self.DONE:
            self.send_cmd(0.0)
            return
        if self.require_ready and (not self.actuation_ready
                                   or time.monotonic() - self.ready_wall > 1.0):
            self.send_cmd(0.0)
            if self.phase not in ("wait_odom", "stand_check"):
                self.abort("controller lost actuation readiness; aborting probe")
            elif time.monotonic() - self.started_wall > self.wait_timeout:
                self.abort("controller did not become actuation-ready before timeout")
            else:
                self.get_logger().info("waiting for controller actuation_ready ...",
                                       throttle_duration_sec=2.0)
            return
        if self.pose is None:
            self.get_logger().info("waiting for pose on body topic ...",
                                   throttle_duration_sec=2.0)
            return
        x, y, z = self.pose[0], self.pose[1], self.pose[2]
        roll, pitch, _ = quat_to_rpy(*self.pose[3:7])

        if self.phase == "wait_odom":
            if z >= self.stand_z_min:
                self.begin_phase("stand_check")
            else:
                self.get_logger().info(
                    "robot not standing (z=%.2f < %.2f); run getup first" %
                    (z, self.stand_z_min), throttle_duration_sec=2.0)
                return
        elif self.phase == "stand_check":
            if z < self.stand_z_min:
                self.phase = "wait_odom"
                return
            if now - self.phase_start >= 1.0:
                self.level_idx = 0
                self.begin_phase(self.SETTLE)
                self.get_logger().info("standing, start level 0: vx=%.2f"
                                       % self.levels[0])
        elif self.phase == self.SETTLE:
            self.send_cmd(0.0)
            if now - self.phase_start >= self.settle_sec:
                self.begin_phase(self.DRIVE)
        elif self.phase == self.DRIVE:
            cmd = self.levels[self.level_idx]
            self.send_cmd(cmd)
            if z < self.stand_z_min or abs(roll) > 0.6 or abs(pitch) > 0.8:
                self.get_logger().error(
                    "fall detected at level %.2f, aborting remaining levels"
                    % cmd)
                self.fallen = True
                self.begin_phase(self.DONE)
            elif now - self.phase_start >= self.drive_sec:
                self.level_idx += 1
                if self.level_idx >= len(self.levels):
                    self.begin_phase(self.DONE)
                else:
                    self.begin_phase(self.REST)
        elif self.phase == self.REST:
            self.send_cmd(0.0)
            if now - self.phase_start >= self.rest_sec:
                self.begin_phase(self.SETTLE)
        elif self.phase == self.DONE:
            self.send_cmd(0.0)

        self.rows.append([
            now, self.phase, self.level_idx,
            self.levels[self.level_idx] if (
                self.phase == self.DRIVE
                and 0 <= self.level_idx < len(self.levels))
            else 0.0,
            x, y, z, roll, pitch,
            quat_to_rpy(*self.pose[3:7])[2],
            self.imu_w[0], self.imu_w[1], self.imu_w[2],
            1 if self.fallen else 0,
        ])

    def save(self):
        header = ["t", "phase", "level_idx", "cmd_vx", "x", "y", "z",
                  "roll", "pitch", "yaw", "wx", "wy", "wz", "fallen"]
        with open(self.out_path, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            writer.writerows(self.rows)
        self.get_logger().info("wrote %d rows to %s"
                               % (len(self.rows), self.out_path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--levels", default="0.05,0.1,0.15,0.2,0.3,0.4,0.5,0.6",
                        help="comma-separated constant vx levels [m/s]")
    parser.add_argument("--drive", type=float, default=10.0,
                        help="seconds per level")
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--rest", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=50.0,
                        help="command publish / log rate [Hz]")
    parser.add_argument("--stand-z-min", type=float, default=0.20)
    parser.add_argument("--use-sim-time", action="store_true",
                        help="measure speed per simulation second, not Gazebo wall-clock speed")
    parser.add_argument("--require-actuation-ready", action="store_true",
                        help="wait for fresh supervisor readiness; abort if it is lost")
    parser.add_argument("--wait-timeout", type=float, default=60.0,
                        help="wall seconds allowed for controller readiness")
    parser.add_argument("--cmd-topic", default="/planning/cmd_vel_raw",
                        help="supervisor 前门命令话题(经站立/健康门控)")
    parser.add_argument("--pose-topic", default="/quad_0/body_pose")
    parser.add_argument("--imu-topic", default="/go2/imu")
    parser.add_argument("--out", default="/tmp/min_speed_probe.csv")
    args = parser.parse_args()
    args.levels = [float(v) for v in args.levels.split(",")]

    rclpy.init()
    node = MinSpeedProbe(args)
    try:
        while rclpy.ok() and node.phase != "done":
            rclpy.spin_once(node, timeout_sec=0.1)
        deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_cmd(0.0)
        node.save()
        node.destroy_node()
        rclpy.shutdown()
    return 2 if node.fallen or node.abort_reason else 0


if __name__ == "__main__":
    sys.exit(main())
