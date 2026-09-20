#!/usr/bin/env python3
"""Run the ONNX position policy against Gazebo's trajectory controller."""

from collections import deque
import os

import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GO2OnnxPolicyNode(Node):
    def __init__(self):
        super().__init__("go2_onnx_policy_node")
        model_path = str(self.declare_parameter(
            "model_path",
            "/home/t1an/ros2_ws/scan_planner_ws/parkour_moe_full_model.onnx",
        ).value)
        if not os.path.isfile(model_path):
            raise FileNotFoundError(model_path)
        self.session = ort.InferenceSession(model_path)
        self.num_obs = 45
        self.num_actions = 12
        self.history_len = 10
        self.control_dt = 0.02
        self.action_scale = 0.25
        self.cmd_timeout = 0.5
        self.default_dof_pos = np.array(
            [0.1, 0.8, -1.5, -0.1, 0.8, -1.5,
             0.1, 1.0, -1.5, -0.1, 1.0, -1.5], dtype=np.float32)
        self.joint_names = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        ]
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self.dof_pos = self.default_dof_pos.copy()
        self.dof_vel = np.zeros(self.num_actions, dtype=np.float32)
        self.cmd_vel = np.zeros(3, dtype=np.float32)
        self.last_action = np.zeros(self.num_actions, dtype=np.float32)
        self.have_joint_state = False
        self.have_imu = False
        self.last_cmd_time = None
        self.obs_history = deque(maxlen=self.history_len)
        self.depth_camera = np.zeros((1, 2, 58, 87), dtype=np.float32)
        self.use_vision = np.array([0.0], dtype=np.float32)
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(
            Imu, "/go2/imu", self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, "/quad_0/cmd_vel", self.cmd_vel_callback, 10)
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)
        for _ in range(self.history_len):
            self.obs_history.append(self.compute_obs())
        inputs = {item.name: item.shape for item in self.session.get_inputs()}
        expected = {
            "proprio_history": [1, 450], "depth_camera": [1, 2, 58, 87],
            "obs_now": [1, 45], "use_vision": [1],
        }
        if inputs != expected:
            raise RuntimeError(f"Unexpected ONNX input signature: {inputs}")
        self.get_logger().info(
            "ONNX position policy ready: 50 Hz, vision disabled, "
            "waiting for joint_states + imu + cmd_vel")

    def joint_state_callback(self, msg):
        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            if name in self.joint_names:
                index = self.joint_names.index(name)
                self.dof_pos[index] = position
                self.dof_vel[index] = velocity
        self.have_joint_state = True

    def imu_callback(self, msg):
        self.base_ang_vel[:] = [
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        q = msg.orientation
        self.projected_gravity[:] = [
            2.0 * (q.x * q.z - q.w * q.y),
            2.0 * (q.y * q.z + q.w * q.x),
            q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z,
        ]
        self.have_imu = True

    def cmd_vel_callback(self, msg):
        self.cmd_vel[:] = np.clip(
            [msg.linear.x, msg.linear.y, msg.angular.z], [-2.0, -1.0, -2.0], [2.0, 1.0, 2.0])
        self.last_cmd_time = self.get_clock().now().nanoseconds

    def compute_obs(self):
        obs = np.zeros(self.num_obs, dtype=np.float32)
        obs[0:3] = self.base_ang_vel * 0.25
        obs[3:6] = self.projected_gravity
        obs[6:9] = self.cmd_vel * np.array([2.0, 2.0, 0.25], dtype=np.float32)
        obs[9:21] = self.dof_pos - self.default_dof_pos
        obs[21:33] = self.dof_vel * 0.05
        obs[33:45] = self.last_action
        return obs

    def publish_hold(self):
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.default_dof_pos.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.control_dt * 1e9)
        message.points = [point]
        self.joint_cmd_pub.publish(message)

    def control_loop(self):
        now = self.get_clock().now().nanoseconds
        if (not self.have_joint_state or not self.have_imu or
                self.last_cmd_time is None or
                (now - self.last_cmd_time) / 1e9 > self.cmd_timeout):
            self.publish_hold()
            return
        obs = self.compute_obs()
        self.obs_history.append(obs)
        inputs = {
            "proprio_history": np.asarray(self.obs_history, dtype=np.float32).reshape(1, -1),
            "depth_camera": self.depth_camera,
            "obs_now": obs.reshape(1, -1),
            "use_vision": self.use_vision,
        }
        try:
            action = np.asarray(self.session.run(["action"], inputs)[0], dtype=np.float32).reshape(-1)
            if action.shape != (self.num_actions,):
                raise RuntimeError(f"unexpected action shape {action.shape}")
        except Exception as error:
            self.get_logger().error(f"ONNX inference failed: {error}")
            self.publish_hold()
            return
        target = self.default_dof_pos + self.action_scale * action
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = target.tolist()
        point.time_from_start.nanosec = int(self.control_dt * 1e9)
        message.points = [point]
        self.joint_cmd_pub.publish(message)
        self.last_action = action


def main(args=None):
    rclpy.init(args=args)
    node = GO2OnnxPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
