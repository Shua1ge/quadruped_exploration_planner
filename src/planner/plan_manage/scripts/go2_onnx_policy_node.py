#!/usr/bin/env python3
"""GO2 ONNX Policy Node - Deploys Parkour MoE model in Gazebo simulation."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import onnxruntime as ort
from collections import deque
import os


class GO2OnnxPolicyNode(Node):
    def __init__(self):
        super().__init__('go2_onnx_policy_node')
        
        # Load ONNX model
        model_path = self.declare_parameter('model_path', 
            '/home/t1an/ros2_ws/scan_planner_ws/parkour_moe_full_model.onnx').value
        self.get_logger().info(f'Loading ONNX model from {model_path}')
        
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            raise FileNotFoundError(f'Model file not found: {model_path}')
        
        self.session = ort.InferenceSession(model_path)
        self.get_logger().info('ONNX model loaded successfully')
        
        # Configuration from pt文件输入输出说明.md
        self.num_obs = 45
        self.num_actions = 12
        self.history_len = 10
        self.control_dt = 0.02  # 50Hz policy
        
        # Scaling factors
        self.ang_vel_scale = 0.25
        self.dof_pos_scale = 1.0
        self.dof_vel_scale = 0.05
        self.cmd_scale = np.array([2.0, 2.0, 0.25])
        self.action_scale = 0.25
        
        # Default joint positions (from doc line 471-476)
        self.default_dof_pos = np.array([
            0.1, 0.8, -1.5,   # FL
            -0.1, 0.8, -1.5,  # FR
            0.1, 1.0, -1.5,   # RL
            -0.1, 1.0, -1.5   # RR
        ])
        
        # Joint names in Gazebo order
        self.joint_names = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint'
        ]
        
        # State buffers
        self.obs_history = deque(maxlen=self.history_len)
        self.last_action = np.zeros(self.num_actions)
        self.cmd_vel = np.array([0.0, 0.0, 0.0])  # [vx, vy, yaw_rate]
        
        # Current state
        self.base_ang_vel = np.zeros(3)
        self.projected_gravity = np.array([0.0, 0.0, -1.0])
        self.dof_pos = self.default_dof_pos.copy()
        self.dof_vel = np.zeros(self.num_actions)
        
        # Dummy depth camera (zeros for now, will add real depth later)
        self.depth_camera = np.zeros((1, 2, 58, 87), dtype=np.float32)
        self.use_vision = np.array([0.0], dtype=np.float32)  # Disable vision for now
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/go2/imu', self.imu_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/quad_0/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publisher for joint trajectory commands
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Control timer (50Hz)
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)
        
        # Initialize observation history
        for _ in range(self.history_len):
            self.obs_history.append(self._compute_obs())
        
        self.get_logger().info('GO2 ONNX Policy Node initialized, control frequency: 50Hz')
    
    def joint_state_callback(self, msg):
        """Update joint positions and velocities from Gazebo."""
        for i, name in enumerate(self.joint_names):
            try:
                idx = msg.name.index(name)
                self.dof_pos[i] = msg.position[idx]
                self.dof_vel[i] = msg.velocity[idx]
            except (ValueError, IndexError):
                pass
    
    def imu_callback(self, msg):
        """Update IMU data (angular velocity and gravity)."""
        self.base_ang_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Compute projected gravity from orientation
        qw, qx, qy, qz = (msg.orientation.w, msg.orientation.x, 
                          msg.orientation.y, msg.orientation.z)
        # Rotate gravity vector [0, 0, -1] by inverse quaternion
        gx = 2 * (qx * qz - qw * qy)
        gy = 2 * (qy * qz + qw * qx)
        gz = qw*qw - qx*qx - qy*qy + qz*qz
        self.projected_gravity = np.array([gx, gy, gz])
    
    def cmd_vel_callback(self, msg):
        """Update command velocity from planner."""
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        # Clip to max command
        self.cmd_vel = np.clip(self.cmd_vel, [-2.0, -1.0, -2.0], [2.0, 1.0, 2.0])
    
    def _compute_obs(self):
        """Compute 45-dim observation vector (doc line 110-119)."""
        obs = np.zeros(self.num_obs, dtype=np.float32)
        
        # 0:3 - base angular velocity (scaled)
        obs[0:3] = self.base_ang_vel * self.ang_vel_scale
        
        # 3:6 - projected gravity
        obs[3:6] = self.projected_gravity
        
        # 6:9 - command (scaled)
        obs[6:9] = self.cmd_vel * self.cmd_scale
        
        # 9:21 - joint position offset (scaled)
        obs[9:21] = (self.dof_pos - self.default_dof_pos) * self.dof_pos_scale
        
        # 21:33 - joint velocity (scaled)
        obs[21:33] = self.dof_vel * self.dof_vel_scale
        
        # 33:45 - last action
        obs[33:45] = self.last_action
        
        return obs
    
    def control_loop(self):
        """Main control loop at 50Hz."""
        # Update observation history
        obs_now = self._compute_obs()
        self.obs_history.append(obs_now)
        
        # Prepare ONNX inputs (doc line 430-438)
        proprio_history = np.array(self.obs_history, dtype=np.float32).flatten()
        proprio_history = proprio_history.reshape(1, -1)  # [1, 450]
        
        obs_now_input = obs_now.reshape(1, -1)  # [1, 45]
        
        # Run inference
        try:
            outputs = self.session.run(None, {
                'proprio_history': proprio_history,
                'depth_camera': self.depth_camera,
                'obs_now': obs_now_input,
                'use_vision': self.use_vision
            })
            
            # Output is [1, 12] action (doc line 441)
            action = outputs[0].flatten()
            
        except Exception as e:
            self.get_logger().error(f'ONNX inference failed: {e}')
            action = np.zeros(self.num_actions)
        
        # Convert action to target joint positions (doc line 454-457)
        target_dof_pos = self.default_dof_pos + action * self.action_scale
        
        # Publish joint commands
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = self.joint_names
        cmd_msg.position = target_dof_pos.tolist()
        cmd_msg.velocity = [0.0] * self.num_actions
        self.joint_cmd_pub.publish(cmd_msg)
        
        # Update last action
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


if __name__ == '__main__':
    main()

