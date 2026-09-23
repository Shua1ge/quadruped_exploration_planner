#!/usr/bin/env python3
"""Single owner for Go2 motion capability and actuator outputs."""

from collections import deque
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scan_planner_msgs.msg import LocomotionState
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from go2_locomotion_logic import (
    DEFAULT_DOF_POS, GO2_COMMAND_LIMITS, GO2_TORQUE_LIMITS, GRAVITY_WORLD,
    JOINT_NAMES,
    advance_limited_command,
    body_motion_stable, interpolate_joint_target, joint_motion_stable, pd_effort,
    planar_pose_valid, posture_error as compute_posture_error,
    projected_gravity_from_quaternion, tilt_cos,
)


class Go2LocomotionSupervisor(Node):
    def __init__(self):
        super().__init__("go2_locomotion_supervisor")
        self.mode_name = str(self.declare_parameter(
            "locomotion_mode", "planning_kinematic").value)
        if self.mode_name not in ("planning_kinematic", "rl_effort"):
            raise ValueError(f"unsupported locomotion_mode: {self.mode_name}")
        self.mode = (LocomotionState.MODE_PLANNING_KINEMATIC
                     if self.mode_name == "planning_kinematic"
                     else LocomotionState.MODE_RL_EFFORT)
        self.state = LocomotionState.STATE_STAND
        self.reason = "WAITING_FOR_JOINT_STATE"
        self.epoch = int(time.time_ns() & ((1 << 64) - 1))
        # Match the model's recorded MuJoCo deployment contract exactly: the
        # joint PD is recomputed at the 2 ms physics step and ONNX refreshes the
        # target every ten steps.  A 5 ms inner loop changes the closed-loop
        # plant even when the policy itself still runs at 50 Hz.
        self.control_dt = float(self.declare_parameter("control_dt", 0.002).value)
        self.policy_dt = float(self.declare_parameter("policy_dt", 0.02).value)
        ratio = self.policy_dt / self.control_dt
        self.policy_decimation = int(round(ratio))
        if (self.control_dt <= 0.0 or self.policy_decimation < 1
                or not math.isclose(ratio, self.policy_decimation,
                                    rel_tol=0.0, abs_tol=1e-9)):
            raise ValueError("policy_dt must be a positive integer multiple of control_dt")
        self.state_timeout = float(self.declare_parameter("state_timeout", 0.5).value)
        # The policy's verified, motionless stance settles with the calf joints
        # up to 0.368 rad and the hip joints up to 0.511 rad from
        # DEFAULT_DOF_POS, so a limit near this learned equilibrium's own edge
        # turns the check into a coin flip: height, tilt, body motion and joint
        # motion are what physically verify the stand.  What must stay rejected
        # is a collapsed leg, which reads 1.22 rad at the calf and 2.7 rad at
        # the thigh -- any limit well below 1.0 still detects it decisively.
        self.stand_error_limit = float(self.declare_parameter(
            "stand_error_limit", 0.70).value)
        self.fault_cycles_required = int(self.declare_parameter("fault_cycles", 40).value)
        self.stable_cycles_required = int(self.declare_parameter("stable_cycles", 25).value)
        # Original deployment contract of this exact Go2 Parkour MoE export.
        self.kp = float(self.declare_parameter("kp", 40.0).value)
        self.kd = float(self.declare_parameter("kd", 1.0).value)
        self.getup_kp = float(self.declare_parameter("getup_kp", 60.0).value)
        self.getup_kd = float(self.declare_parameter("getup_kd", 5.0).value)
        self.torque_limits = np.asarray(self.declare_parameter(
            "torque_limits",
            GO2_TORQUE_LIMITS.tolist()).value,
            dtype=np.float64)
        self.getup_torque_limits = np.asarray(self.declare_parameter(
            "getup_torque_limits",
            [23.0, 23.0, 35.55, 23.0, 23.0, 35.55,
             23.0, 23.0, 35.55, 23.0, 23.0, 35.55]).value,
            dtype=np.float64)
        self.action_scale = float(self.declare_parameter("action_scale", 0.25).value)
        self.cmd_timeout = float(self.declare_parameter("cmd_timeout", 0.5).value)
        self.max_tilt_cos = math.cos(math.radians(float(
            self.declare_parameter("max_tilt_deg", 55.0).value)))
        self.min_body_height = float(self.declare_parameter(
            "min_body_height", 0.15).value)
        self.stand_min_body_height = float(self.declare_parameter(
            "stand_min_body_height", 0.24).value)
        self.stand_max_tilt_cos = math.cos(math.radians(float(
            self.declare_parameter("stand_max_tilt_deg", 30.0).value)))
        self.stand_max_linear_speed = float(self.declare_parameter(
            "stand_max_linear_speed", 0.08).value)
        self.stand_max_angular_speed = float(self.declare_parameter(
            "stand_max_angular_speed", 0.25).value)
        self.stand_capture_max_linear_speed = float(self.declare_parameter(
            "stand_capture_max_linear_speed", 0.20).value)
        self.stand_capture_max_angular_speed = float(self.declare_parameter(
            "stand_capture_max_angular_speed", 1.0).value)
        self.stand_capture_max_joint_speed = float(self.declare_parameter(
            "stand_capture_max_joint_speed", 2.0).value)
        self.getup_prepose_duration = float(self.declare_parameter(
            "getup_prepose_duration", 1.0).value)
        self.getup_stand_duration = float(self.declare_parameter(
            "getup_stand_duration", 2.0).value)
        self.policy_handoff_duration = float(self.declare_parameter(
            "policy_handoff_duration", 0.30).value)
        # "policy" gives the lying robot straight to the zero-command ONNX
        # policy, which is the architecture that reached a verified stance from
        # a folded spawn.  "deterministic" keeps the time-scheduled
        # PREPOSE/STAND interpolation, which finishes on the wall clock rather
        # than on the robot actually standing: it has been observed completing
        # at 0.16 m body height with a calf joint already at its velocity limit.
        self.getup_mode = str(self.declare_parameter(
            "getup_mode", "policy").value)
        if self.getup_mode not in ("policy", "deterministic"):
            raise ValueError("getup_mode must be 'policy' or 'deterministic'")
        if min(self.getup_prepose_duration, self.getup_stand_duration,
               self.policy_handoff_duration) <= 0.0:
            raise ValueError("get-up and policy handoff durations must be positive")
        self.rl_command_limits = np.asarray(self.declare_parameter(
            "rl_command_limits", GO2_COMMAND_LIMITS.tolist()).value,
            dtype=np.float64)
        self.rl_command_slew_rates = np.asarray(self.declare_parameter(
            "rl_command_slew_rates", [0.50, 0.40, 1.00]).value,
            dtype=np.float64)
        self.require_navigation_enable = bool(self.declare_parameter(
            "require_navigation_enable", True).value)
        self.navigation_enabled = not (
            self.mode_name == "rl_effort" and self.require_navigation_enable)
        if self.rl_command_limits.shape != (3,):
            raise ValueError("rl_command_limits must contain [vx, vy, yaw_rate]")
        if self.rl_command_slew_rates.shape != (3,):
            raise ValueError(
                "rl_command_slew_rates must contain [vx, vy, yaw_rate]")
        if self.torque_limits.shape != (12,) or np.any(self.torque_limits <= 0.0):
            raise ValueError("torque_limits must contain twelve positive values")
        if (self.getup_torque_limits.shape != (12,)
                or np.any(self.getup_torque_limits <= 0.0)):
            raise ValueError(
                "getup_torque_limits must contain twelve positive values")
        self.q = DEFAULT_DOF_POS.copy()
        self.dq = np.zeros(12, dtype=np.float64)
        self.gravity_body = GRAVITY_WORLD.copy()
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.base_lin_vel = np.zeros(3, dtype=np.float32)
        self.raw_cmd = np.zeros(3, dtype=np.float32)
        self.filtered_cmd = np.zeros(3, dtype=np.float64)
        self.last_action = np.zeros(12, dtype=np.float32)
        self.have_joint_state = False
        self.have_imu = False
        self.last_joint_ns = 0
        self.last_imu_ns = 0
        self.last_cmd_ns = 0
        self.last_odom_ns = 0
        self.body_height = 0.0
        self.odom_pose_valid = False
        self.stable_cycles = 0
        self.fault_cycles = 0
        self.getup_phase = "IDLE"
        self.getup_elapsed = 0.0
        self.getup_start = DEFAULT_DOF_POS.copy()
        self.getup_prepose = np.array(
            [0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
             0.0, 1.36, -2.65, 0.0, 1.36, -2.65], dtype=np.float64)
        self.desired_target = DEFAULT_DOF_POS.copy()
        self.policy_cycle = 0
        self.policy_handoff_elapsed = 0.0
        self.history = deque(maxlen=10)
        self.onnx_session = None
        self.policy_lock = threading.Lock()
        self.policy_event = threading.Event()
        self.policy_stop = threading.Event()
        self.policy_request = None
        self.policy_result = None
        self.policy_error = None
        self.policy_thread = None

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            LocomotionState, "/robot/locomotion_state", state_qos)
        self.enter_navigation_service = self.create_service(
            Trigger, "/robot/enter_navigation", self.enter_navigation_callback)
        self.create_subscription(JointState, "/joint_states", self.joint_callback, 20)
        self.create_subscription(
            Twist, "/planning/cmd_vel_raw", self.cmd_callback, 20)
        self.create_subscription(
            Odometry, "body_pose", self.odom_callback, qos_profile_sensor_data)

        if self.mode_name == "planning_kinematic":
            # Legs are fixed in this mode: there is no joint controller, so
            # publishing joint commands would create a second, unread writer.
            self.joint_pub = None
            self.base_cmd_pub = self.create_publisher(Twist, "/quad_0/cmd_vel", 20)
            self.effort_pub = None
            self.diagnostics_pub = None
        else:
            import onnxruntime as ort
            model_path = str(self.declare_parameter(
                "model_path",
                "/home/t1an/ros2_ws/scan_planner_ws/parkour_moe_full_model.onnx").value)
            if not os.path.isfile(model_path):
                raise FileNotFoundError(model_path)
            self.onnx_session = ort.InferenceSession(model_path)
            inputs = {item.name: item.shape for item in self.onnx_session.get_inputs()}
            expected = {
                "proprio_history": [1, 450], "depth_camera": [1, 2, 58, 87],
                "obs_now": [1, 45], "use_vision": [1],
            }
            if inputs != expected:
                raise RuntimeError(f"unexpected ONNX input signature: {inputs}")
            self.create_subscription(Imu, "/go2/imu", self.imu_callback, qos_profile_sensor_data)
            self.effort_pub = self.create_publisher(
                Float64MultiArray, "/joint_group_effort_controller/commands", 10)
            self.diagnostics_pub = self.create_publisher(
                Float64MultiArray, "/robot/locomotion_diagnostics", 10)
            self.joint_pub = None
            self.base_cmd_pub = None
            self.policy_thread = threading.Thread(
                target=self.policy_worker, name="go2_onnx_50hz", daemon=True)
            self.policy_thread.start()

        self.timer = self.create_timer(self.control_dt, self.control_loop)
        if self.mode_name == "planning_kinematic":
            self.state = LocomotionState.STATE_RUN
            self.reason = "KINEMATIC_PROXY"
        elif not self.navigation_enabled:
            self.reason = "WAITING_FOR_NAVIGATION_ENABLE"
        self.publish_state()
        self.get_logger().info(
            f"locomotion supervisor mode={self.mode_name} "
            f"getup={self.getup_mode} "
            f"control={1.0 / self.control_dt:.0f}Hz "
            f"policy={1.0 / self.policy_dt:.0f}Hz "
            f"kp={self.kp:.1f} kd={self.kd:.1f}")
        if self.mode_name == "planning_kinematic":
            self.get_logger().info(
                "[KINEMATIC_PROXY] motion is provided by the model-level velocity "
                "actuator; leg pose is cosmetic and is not a dynamics result")
        elif not self.navigation_enabled:
            self.get_logger().info(
                f"[NAVIGATION_DISARMED] robot remains passive; call "
                f"'/robot/enter_navigation' to start the {self.getup_mode} get-up")

    def enter_navigation_callback(self, _request, response):
        if self.mode_name != "rl_effort":
            response.success = True
            response.message = "planning_kinematic is already navigation-ready"
            return response
        if self.state == LocomotionState.STATE_FAULT:
            response.success = False
            response.message = (
                f"locomotion fault is latched ({self.reason}); restart required")
            return response
        if self.navigation_enabled:
            response.success = True
            response.message = "navigation is already enabled"
            return response
        now = self.get_clock().now().nanoseconds
        if (not self.have_joint_state or not self.have_imu
                or not self.odom_fresh(now)):
            response.success = False
            response.message = (
                "initialization incomplete: waiting for joint state, IMU, "
                "and odometry")
            return response
        self.navigation_enabled = True
        self.stable_cycles = 0
        self.fault_cycles = 0
        self.filtered_cmd.fill(0.0)
        self.last_action.fill(0.0)
        if self.getup_mode == "policy":
            self.begin_policy_phase()
            self.set_state(
                LocomotionState.STATE_STAND, "POLICY_GETUP")
            self.get_logger().info(
                "[NAVIGATION_ARMED] ONNX policy owns the get-up with zero "
                "motion command; planner commands remain gated until the "
                "stance is verified")
            response.message = (
                "policy get-up started; navigation waits for verified stance")
        else:
            self.history.clear()
            self.getup_phase = "PREPOSE"
            self.getup_elapsed = 0.0
            self.getup_start = self.q.copy()
            self.desired_target = self.q.copy()
            self.policy_cycle = 0
            self.policy_handoff_elapsed = 0.0
            self.set_state(LocomotionState.STATE_STAND, "GETUP_PREPOSE")
            self.get_logger().info(
                "[NAVIGATION_ARMED] deterministic 3s get-up started; planner "
                "commands remain gated until zero-command ONNX stance is "
                "verified")
            response.message = (
                "deterministic get-up started; navigation waits for verified "
                "stance")
        response.success = True
        return response

    def set_state(self, state, reason):
        if self.state != state or self.reason != reason:
            self.state = state
            self.reason = reason
            self.publish_state()

    def capabilities(self, now_ns: int):
        """Derive the two capability facts from the current authority state.

        ``actuation_ready`` gates motion commands; ``perception_pose_valid``
        gates map fusion.  They are published from this single owner so no
        consumer has to reinterpret the human-facing STAND/RUN/FAULT enum.
        """
        if self.mode_name == "planning_kinematic":
            # The proxy does not use leg posture, but its planar pose is still
            # a hard mapping contract.  Never call a tipped / fallen model
            # healthy merely because odometry is fresh.
            ready = (self.state == LocomotionState.STATE_RUN
                     and self.odom_fresh(now_ns)
                     and self.odom_pose_valid)
            return ready, ready
        ready = (self.state == LocomotionState.STATE_RUN
                 and self.odom_fresh(now_ns)
                 and self.odom_pose_valid
                 and self.have_joint_state
                 and self.have_imu
                 and (now_ns - self.last_joint_ns) * 1e-9 <= self.state_timeout
                 and (now_ns - self.last_imu_ns) * 1e-9 <= self.state_timeout)
        return ready, ready

    def publish_state(self):
        now_ns = self.get_clock().now().nanoseconds
        actuation_ready, perception_pose_valid = self.capabilities(now_ns)
        msg = LocomotionState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.epoch = self.epoch
        msg.mode = self.mode
        msg.state = self.state
        msg.actuation_ready = actuation_ready
        msg.perception_pose_valid = perception_pose_valid
        msg.reason = self.reason
        self.state_pub.publish(msg)

    def joint_callback(self, msg):
        values = {name: (position, velocity)
                  for name, position, velocity in zip(msg.name, msg.position, msg.velocity)}
        if not all(name in values for name in JOINT_NAMES):
            return
        self.q[:] = [values[name][0] for name in JOINT_NAMES]
        self.dq[:] = [values[name][1] for name in JOINT_NAMES]
        now = self.get_clock().now().nanoseconds
        if not self.have_joint_state:
            self.history.clear()
        self.have_joint_state = True
        self.last_joint_ns = now

    def imu_callback(self, msg):
        self.base_ang_vel[:] = [
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        q = msg.orientation
        try:
            self.gravity_body = projected_gravity_from_quaternion(q.x, q.y, q.z, q.w)
        except ValueError:
            self.set_state(LocomotionState.STATE_FAULT, "IMU_QUATERNION_INVALID")
            return
        self.have_imu = True
        self.last_imu_ns = self.get_clock().now().nanoseconds

    def cmd_callback(self, msg):
        self.raw_cmd[:] = np.clip(
            [msg.linear.x, msg.linear.y, msg.angular.z],
            [-2.0, -1.0, -2.0], [2.0, 1.0, 2.0])
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def odom_callback(self, msg):
        self.last_odom_ns = self.get_clock().now().nanoseconds
        self.body_height = float(msg.pose.pose.position.z)
        self.base_lin_vel[:] = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        q = msg.pose.pose.orientation
        try:
            self.odom_pose_valid = planar_pose_valid(
                q.x, q.y, q.z, q.w, self.body_height,
                self.max_tilt_cos, self.min_body_height)
        except ValueError:
            self.odom_pose_valid = False

    def odom_fresh(self, now_ns: int) -> bool:
        return (self.last_odom_ns > 0
                and (now_ns - self.last_odom_ns) * 1e-9 <= self.state_timeout)

    def publish_joint_target(self, target):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = target.tolist()
        point.time_from_start.nanosec = int(self.control_dt * 1e9)
        msg.points = [point]
        self.joint_pub.publish(msg)

    def publish_effort(self, effort):
        msg = Float64MultiArray()
        msg.data = effort.astype(float).tolist()
        self.effort_pub.publish(msg)

    def zero_output(self):
        if self.base_cmd_pub is not None:
            self.base_cmd_pub.publish(Twist())
        if self.effort_pub is not None:
            self.publish_effort(np.zeros(12, dtype=np.float64))

    def posture_error(self, target=None) -> float:
        return compute_posture_error(
            self.q, DEFAULT_DOF_POS if target is None else target)

    def compute_observation(self, command=None):
        obs = np.zeros(45, dtype=np.float32)
        obs[0:3] = self.base_ang_vel * 0.25
        obs[3:6] = self.gravity_body.astype(np.float32)
        cmd = (self.filtered_cmd if command is None
               else np.asarray(command, dtype=np.float64))
        obs[6:9] = cmd * np.array([2.0, 2.0, 0.25], dtype=np.float32)
        obs[9:21] = (self.q - DEFAULT_DOF_POS).astype(np.float32)
        obs[21:33] = self.dq.astype(np.float32) * 0.05
        obs[33:45] = self.last_action
        return np.clip(obs, -100.0, 100.0)

    def evaluate_policy(self, command):
        """Queue one observation for the independent ONNX inference worker."""
        obs = self.compute_observation(command)
        self.history.append(obs)
        if len(self.history) != self.history.maxlen:
            return False
        inputs = {
            "proprio_history": np.asarray(
                self.history, dtype=np.float32).reshape(1, -1),
            "depth_camera": np.zeros((1, 2, 58, 87), dtype=np.float32),
            "obs_now": obs.reshape(1, -1),
            "use_vision": np.array([0.0], dtype=np.float32),
        }
        with self.policy_lock:
            # Keep only the newest request if inference ever takes longer than
            # one 20 ms policy period; the 200 Hz PD loop must never block.
            self.policy_request = inputs
        self.policy_event.set()
        return True

    def policy_worker(self):
        """Run ONNX off the actuator timer so PD feedback remains at 200 Hz."""
        while not self.policy_stop.is_set():
            self.policy_event.wait(timeout=0.1)
            self.policy_event.clear()
            if self.policy_stop.is_set():
                break
            with self.policy_lock:
                inputs = self.policy_request
                self.policy_request = None
            if inputs is None:
                continue
            try:
                action = np.asarray(
                    self.onnx_session.run(["action"], inputs)[0],
                    dtype=np.float32).reshape(-1)
                if action.shape != (12,) or not np.all(np.isfinite(action)):
                    raise RuntimeError(f"invalid action {action}")
                with self.policy_lock:
                    self.policy_result = np.clip(action, -100.0, 100.0)
            except Exception as error:  # reported by the ROS thread
                with self.policy_lock:
                    self.policy_error = str(error)

    def consume_policy_result(self):
        """Apply the newest asynchronous action without blocking control."""
        with self.policy_lock:
            error = self.policy_error
            self.policy_error = None
            action = self.policy_result
            self.policy_result = None
        if error is not None:
            self.set_state(LocomotionState.STATE_FAULT, "ONNX_INFERENCE_FAILED")
            self.get_logger().error(error)
            self.zero_output()
            return False
        if action is None:
            return True
        self.last_action = action
        policy_target = DEFAULT_DOF_POS + self.action_scale * action
        if self.policy_handoff_elapsed < self.policy_handoff_duration:
            self.policy_handoff_elapsed += self.policy_dt
            progress = min(
                self.policy_handoff_elapsed / self.policy_handoff_duration, 1.0)
            self.desired_target = (
                DEFAULT_DOF_POS + progress * (policy_target - DEFAULT_DOF_POS))
        else:
            self.desired_target = policy_target
        return True

    def shutdown(self):
        self.policy_stop.set()
        self.policy_event.set()
        if self.policy_thread is not None:
            self.policy_thread.join(timeout=2.0)

    def publish_target_effort(self, target, kp, kd, torque_limits=None):
        torque_limits = (self.torque_limits if torque_limits is None
                         else torque_limits)
        unclipped = kp * (np.asarray(target) - self.q) - kd * self.dq
        effort = pd_effort(
            target, self.q, self.dq, kp, kd, torque_limits)
        self.publish_effort(effort)
        if self.diagnostics_pub is not None and self.policy_cycle == 0:
            # Fixed layout, all in policy/controller leg-major order:
            # cmd(3), q(12), dq(12), action(12), q_target(12),
            # tau_unclipped(12), tau_command(12), saturation(12).
            diagnostics = Float64MultiArray()
            saturated = np.abs(unclipped) >= (np.asarray(torque_limits) - 1e-9)
            diagnostics.data = np.concatenate((
                self.filtered_cmd, self.q, self.dq, self.last_action,
                np.asarray(target), unclipped, effort,
                saturated.astype(np.float64))).astype(float).tolist()
            self.diagnostics_pub.publish(diagnostics)

    def begin_policy_phase(self):
        """Hand actuator authority to the zero-command ONNX policy.

        Both entry points into the policy share this so they present identical
        target, phase and observation history: the arm path in "policy" mode,
        and the end of the deterministic get-up.
        """
        self.getup_phase = "POLICY"
        self.desired_target = DEFAULT_DOF_POS.copy()
        self.policy_cycle = 0
        self.policy_handoff_elapsed = 0.0
        self.stable_cycles = 0
        self.history.clear()
        observation = self.compute_observation(np.zeros(3, dtype=np.float64))
        self.history.extend(observation.copy() for _ in range(self.history.maxlen))

    def run_getup(self):
        """Run the proven two-stage RL-SAR get-up using the 200 Hz PD loop."""
        self.getup_elapsed += self.control_dt
        if self.getup_phase == "PREPOSE":
            target, progress = interpolate_joint_target(
                self.getup_start, self.getup_prepose,
                self.getup_elapsed, self.getup_prepose_duration)
            self.publish_target_effort(
                target, self.getup_kp, self.getup_kd,
                self.getup_torque_limits)
            if progress >= 1.0:
                self.getup_phase = "STAND"
                self.getup_elapsed = 0.0
                self.set_state(LocomotionState.STATE_STAND, "GETUP_TO_DEFAULT")
            return
        target, progress = interpolate_joint_target(
            self.getup_prepose, DEFAULT_DOF_POS,
            self.getup_elapsed, self.getup_stand_duration)
        self.publish_target_effort(
            target, self.getup_kp, self.getup_kd,
            self.getup_torque_limits)
        if progress >= 1.0:
            self.begin_policy_phase()
            self.set_state(
                LocomotionState.STATE_STAND, "ZERO_COMMAND_POLICY_HANDOFF")
            self.get_logger().info(
                "[GETUP_COMPLETE] default stance reached; the correctly tuned "
                "zero-command ONNX policy now absorbs residual get-up motion")

    def control_loop(self):
        now = self.get_clock().now().nanoseconds
        self.publish_state()
        if self.state == LocomotionState.STATE_FAULT:
            self.zero_output()
            return
        if self.mode_name == "planning_kinematic":
            if not self.odom_fresh(now):
                self.zero_output()
                return
            if not self.odom_pose_valid:
                self.fault_cycles += 1
                self.zero_output()
                if self.fault_cycles >= self.fault_cycles_required:
                    self.set_state(
                        LocomotionState.STATE_FAULT,
                        "KINEMATIC_POSE_INVALID_MAP_RESTART_REQUIRED")
                    self.get_logger().error(
                        f"[KINEMATIC_POSE_FAULT] height={self.body_height:.3f}m; "
                        "roll/pitch or height violated the planar mapping "
                        "contract; restart the run to discard the invalid map")
                return
            self.fault_cycles = 0
            if self.have_joint_state:
                self.publish_joint_target(DEFAULT_DOF_POS)
            cmd = Twist()
            if self.last_cmd_ns and (now - self.last_cmd_ns) * 1e-9 <= self.cmd_timeout:
                cmd.linear.x, cmd.linear.y, cmd.angular.z = map(float, self.raw_cmd)
            self.base_cmd_pub.publish(cmd)
            return
        if not self.have_joint_state or (now - self.last_joint_ns) * 1e-9 > self.state_timeout:
            self.set_state(LocomotionState.STATE_STAND, "WAITING_FOR_JOINT_STATE")
            self.zero_output()
            return
        if not self.navigation_enabled:
            # Deliberately passive until the user authorizes get-up.
            self.filtered_cmd.fill(0.0)
            self.last_action.fill(0.0)
            self.history.clear()
            self.set_state(
                LocomotionState.STATE_STAND, "WAITING_FOR_NAVIGATION_ENABLE")
            self.zero_output()
            return
        if not self.have_imu or (now - self.last_imu_ns) * 1e-9 > self.state_timeout:
            self.set_state(LocomotionState.STATE_FAULT, "IMU_STALE")
            self.zero_output()
            return

        if self.getup_phase in ("PREPOSE", "STAND"):
            self.filtered_cmd.fill(0.0)
            self.run_getup()
            return

        command_fresh = (self.last_cmd_ns
                         and (now - self.last_cmd_ns) * 1e-9 <= self.cmd_timeout)
        requested = (self.raw_cmd if command_fresh
                     else np.zeros(3, dtype=np.float32))
        policy_tick = self.policy_cycle == 0
        self.policy_cycle = (self.policy_cycle + 1) % self.policy_decimation
        if self.state == LocomotionState.STATE_STAND:
            # Fixed default-pose PD cannot absorb the residual get-up momentum:
            # live data showed immediate tumbling with saturated efforts.  Give
            # the policy stabilization authority, but keep its command at zero
            # and keep Explorer gated until the physical window passes.
            if policy_tick and not self.evaluate_policy(
                    np.zeros(3, dtype=np.float64)):
                return
            if not self.consume_policy_result():
                return
            self.publish_target_effort(self.desired_target, self.kp, self.kd)
            if not policy_tick:
                return
            error = self.posture_error()
            handoff_complete = (
                self.policy_handoff_elapsed >= self.policy_handoff_duration)
            stable = (handoff_complete
                      and error <= self.stand_error_limit
                      and self.odom_fresh(now)
                      and self.body_height >= self.stand_min_body_height
                      and tilt_cos(self.gravity_body) >= self.stand_max_tilt_cos
                      and body_motion_stable(
                          self.base_lin_vel, self.base_ang_vel,
                          self.stand_max_linear_speed,
                          self.stand_max_angular_speed)
                      and joint_motion_stable(
                          self.dq, self.stand_capture_max_joint_speed))
            self.stable_cycles = self.stable_cycles + 1 if stable else 0
            self.set_state(
                LocomotionState.STATE_STAND, "ZERO_COMMAND_SETTLING")
            if self.stable_cycles >= self.stable_cycles_required:
                self.set_state(LocomotionState.STATE_RUN, "STAND_VERIFIED")
                self.get_logger().info(
                    f"[STAND_VERIFIED] correctly tuned zero-command ONNX "
                    f"stable for "
                    f"{self.stable_cycles * self.policy_dt:.2f}s; "
                    f"height={self.body_height:.3f}m "
                    f"max_joint_error={error:.3f}rad; Explorer may plan")
            else:
                self.get_logger().info(
                    f"[ZERO_COMMAND_SETTLING] height={self.body_height:.3f}m "
                    f"tilt_cos={tilt_cos(self.gravity_body):.3f} "
                    f"max_joint_error={error:.3f}rad "
                    f"linear_speed={np.linalg.norm(self.base_lin_vel):.3f}m/s "
                    f"angular_speed={np.linalg.norm(self.base_ang_vel):.3f}rad/s "
                    f"joint_speed={np.max(np.abs(self.dq)):.3f}rad/s "
                    f"stable={self.stable_cycles}/{self.stable_cycles_required}",
                    throttle_duration_sec=1.0)
            return

        if self.state == LocomotionState.STATE_RUN:
            body_bad = (not self.odom_fresh(now)
                        or not self.odom_pose_valid
                        or tilt_cos(self.gravity_body) < self.max_tilt_cos)
            self.fault_cycles = self.fault_cycles + 1 if body_bad else 0
            if self.fault_cycles >= self.fault_cycles_required:
                self.set_state(
                    LocomotionState.STATE_FAULT,
                    "BODY_POSE_INVALID_MAP_RESTART_REQUIRED")
                self.get_logger().error(
                    f"[BODY_POSE_FAULT] height={self.body_height:.3f}m "
                    f"tilt_cos={tilt_cos(self.gravity_body):.3f}; restart "
                    "the run to discard the map built from the invalid pose")
                self.zero_output()
                return

        if policy_tick:
            self.filtered_cmd = advance_limited_command(
                self.filtered_cmd, requested, self.rl_command_limits,
                self.rl_command_slew_rates, self.policy_dt)
            if not self.evaluate_policy(self.filtered_cmd):
                return

        if not self.consume_policy_result():
            return
        # High-rate inner loop: ONNX refreshes the target at 50 Hz while fresh
        # q/dq feedback recomputes the deployment-contract PD effort at 200 Hz.
        self.publish_target_effort(self.desired_target, self.kp, self.kd)


def main(args=None):
    rclpy.init(args=args)
    node = Go2LocomotionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
