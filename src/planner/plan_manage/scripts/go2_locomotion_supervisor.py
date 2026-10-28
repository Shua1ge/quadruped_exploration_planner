#!/usr/bin/env python3
"""Single owner for Go2 motion capability and actuator outputs."""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scan_planner_msgs.msg import LocomotionState
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from go2_locomotion_logic import (
    DEFAULT_DOF_POS, GO2_COMMAND_LIMITS, GO2_TORQUE_LIMITS, GRAVITY_WORLD,
    JOINT_NAMES,
    advance_limited_command,
    body_motion_stable, interpolate_joint_target, joint_motion_stable, pd_effort,
    planar_pose_valid, posture_error as compute_posture_error,
    projected_gravity_from_quaternion, tilt_cos,
    leg_extension_height, fault_support_mode, hold_effort,
    message_stamp_ns, diagnostic_age_ms, summarize_policy_timing,
)


class Go2LocomotionSupervisor(Node):
    def __init__(self, policy_session_factory=None):
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
        # Parkour's legacy/stateful policy uses [2,2,0.25]; go2_rl_lab uses
        # [0.5,1,1].  The backend selection below replaces this for the
        # experiment policy before the first observation is built.
        self.policy_command_scale = np.array([2.0, 2.0, 0.25], dtype=np.float32)
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
        # Source/sample times are diagnostic only. Receipt-based safety gates
        # remain unchanged until measurements justify a control change.
        self.sensor_source_ns = dict(joint=None, imu=None, odom=None)
        self.body_height = 0.0
        self.support_height = 0.0
        self.low_extension_cycles = 0
        self.low_extension_active = False
        self.low_extension_trace_recorded = False
        self.fault_trace_recorded = False
        self.trace_directory = os.path.abspath(os.path.expanduser(str(
            self.declare_parameter("trace_directory", "/tmp/scan_locomotion_traces").value)))
        # At most two jobs per run: first sustained low reach and first FAULT.
        # Disk I/O and JSON encoding must not run on the actuator callback.
        self.trace_writer = ThreadPoolExecutor(max_workers=1, thread_name_prefix="go2_trace")
        self.motion_trace = deque(maxlen=100)
        self.last_effort = np.zeros(12, dtype=np.float64)
        self.fault_hold_target = None
        self.fault_hold_feedforward = None
        self.fault_output_mode = None
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
        self.policy_sequence = 0
        self.policy_episode = 0
        self.policy_last_installed_sequence = 0
        self.policy_last_observation_ros_ns = None
        self.policy_latest_timing = None
        self.policy_timing_records = deque(maxlen=512)
        self.policy_inference_records = deque(maxlen=512)
        self.policy_inflight = None
        self.policy_timing_counts = dict(
            requested=0, inferred=0, installed=0, request_overwrites=0,
            result_overwrites=0, errors=0)
        self.policy_timing_report_counts = self.policy_timing_counts.copy()
        self.policy_timing_report_wall_ns = time.monotonic_ns()

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            LocomotionState, "/robot/locomotion_state", state_qos)
        self.fault_trace_pub = self.create_publisher(
            String, "/robot/locomotion_fault_trace", state_qos)
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
            from ament_index_python.packages import get_package_share_directory
            from go2_stateful_policy import StatefulPolicyAdapter

            self.locomotion_policy = str(self.declare_parameter(
                "locomotion_policy", "legacy").value)
            if self.locomotion_policy not in (
                    "legacy", "parkour_moe_stateful", "go2_rl_lab"):
                raise ValueError(
                    "locomotion_policy must be 'legacy', 'parkour_moe_stateful' "
                    "or 'go2_rl_lab'")
            policy_dir = os.path.join(get_package_share_directory("scan_planner"),
                                      "config", "policies", "parkour_moe_stateful")
            if self.locomotion_policy == "legacy":
                default_model = "/home/t1an/ros2_ws/scan_planner_ws/parkour_moe_full_model.onnx"
            elif self.locomotion_policy == "parkour_moe_stateful":
                default_model = os.path.join(policy_dir, "parkour_moe_full_stateful.onnx")
            else:
                # Experiment-only backend: TorchScript policy exported by
                # wty-yy/go2_rl_lab.  Keep it in the isolated dyn workspace.
                default_model = ("/home/t1an/ros2_ws/scan_planner_dyn_ws/"
                                 "models/go2_rl_lab_policy.pt")
            model_path = str(self.declare_parameter(
                "model_path", default_model).value) or default_model
            actor_model_path = str(self.declare_parameter(
                "actor_model_path", os.path.join(policy_dir, "actor.onnx")).value)
            actor_model_path = actor_model_path or os.path.join(policy_dir, "actor.onnx")
            refresh_every = self.declare_parameter("estimator_refresh_every", 5).value
            if not os.path.isfile(model_path):
                raise FileNotFoundError(model_path)
            if policy_session_factory is not None:
                self.onnx_session = policy_session_factory(model_path)
            elif self.locomotion_policy == "parkour_moe_stateful":
                if not os.path.isfile(actor_model_path):
                    raise FileNotFoundError(actor_model_path)
                self.onnx_session = StatefulPolicyAdapter(
                    model_path, actor_model_path, refresh_every)
            elif self.locomotion_policy == "go2_rl_lab":
                from go2_rl_lab_policy import Go2RlLabPolicyAdapter
                self.kp = float(self.declare_parameter("go2_rl_lab_kp", 25.0).value)
                self.kd = float(self.declare_parameter("go2_rl_lab_kd", 0.5).value)
                self.policy_command_scale = np.array([0.5, 1.0, 1.0], dtype=np.float32)
                self.rl_command_limits = np.array([2.0, 1.0, 3.0], dtype=np.float64)
                self.onnx_session = Go2RlLabPolicyAdapter(
                    model_path,
                    threads=int(self.declare_parameter("go2_rl_lab_threads", 1).value))
            else:
                session_options = ort.SessionOptions()
                session_options.intra_op_num_threads = 2
                session_options.inter_op_num_threads = 1
                session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
                self.onnx_session = ort.InferenceSession(
                    model_path, sess_options=session_options,
                    providers=["CPUExecutionProvider"])
            inputs = {item.name: item.shape for item in self.onnx_session.get_inputs()}
            expected = ({"obs_now": [1, 45]} if self.locomotion_policy == "go2_rl_lab" else {
                "proprio_history": [1, 450], "depth_camera": [1, 2, 58, 87],
                "obs_now": [1, 45], "use_vision": [1],
            })
            if inputs != expected:
                raise RuntimeError(f"unexpected ONNX input signature: {inputs}")
            self.get_logger().info(
                f"[LOCOMOTION_POLICY] backend={self.locomotion_policy} "
                f"model={model_path} blind=true actor={1.0/self.policy_dt:.0f}Hz "
                f"estimator_refresh_every={refresh_every if self.locomotion_policy != 'legacy' else 1}")
            if self.locomotion_policy == "go2_rl_lab":
                self.get_logger().warning(
                    "[POLICY_CONTRACT] go2_rl_lab experiment backend: "
                    f"kp={self.kp:.1f} kd={self.kd:.1f} action_scale={self.action_scale:.2f} "
                    f"command_scale={self.policy_command_scale.tolist()} "
                    f"command_limits={self.rl_command_limits.tolist()} "
                    f"torch_threads={getattr(self.onnx_session, 'thread_count', 1)}")
            if self.locomotion_policy == "parkour_moe_stateful":
                self.get_logger().warning(
                    "[POLICY_CONTRACT] using the flat-ground-tested observation order, "
                    f"action_scale={self.action_scale}, kp={self.kp}, kd={self.kd}; "
                    "new training/deployment configuration has not been supplied")
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
            self.policy_timing_pub = self.create_publisher(
                String, "/robot/locomotion_policy_timing", state_qos)
            # Separate low-rate reporting, including while simulation is
            # paused. No JSON serialization/logging in the 500 Hz loop.
            self.policy_timing_timer = self.create_timer(
                1.0, self.publish_policy_timing,
                clock=Clock(clock_type=ClockType.STEADY_TIME))

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
            first_fault = (state == LocomotionState.STATE_FAULT
                           and self.state != LocomotionState.STATE_FAULT)
            self.state = state
            self.reason = reason
            self.publish_state()
            if first_fault and hasattr(self, 'motion_trace'):
                self.record_posture_event("FAULT", reason)

    def record_posture_event(self, event, reason):
        """Snapshot immutable samples; the writer retains them after exit."""
        flag = "fault_trace_recorded" if event == "FAULT" else "low_extension_trace_recorded"
        if getattr(self, flag):
            return
        setattr(self, flag, True)
        def numbers(values):
            return [float(v) if math.isfinite(float(v)) else None for v in values]
        trace = {
            "event": event, "reason": reason, "epoch": self.epoch,
            "stamp_ns": self.get_clock().now().nanoseconds,
            "state": int(self.state), "policy_backend": getattr(self, "locomotion_policy", None),
            "body_world_z": numbers([self.body_height])[0],
            "leg_extension_height": numbers([self.support_height])[0],
            "trigger": {name: numbers(value) for name, value in (
                ("q", self.q), ("dq", self.dq), ("gravity", self.gravity_body),
                ("base_velocity", self.base_lin_vel), ("angular_velocity", self.base_ang_vel),
                ("cmd_raw", self.raw_cmd), ("cmd_filtered", self.filtered_cmd),
                ("action", self.last_action), ("target", self.desired_target),
                ("effort", self.last_effort))},
            "sensor_source_ns": self.sensor_source_ns.copy(),
            "policy_timing": getattr(self, "policy_latest_timing", None),
            "samples": list(self.motion_trace),
        }
        self.trace_writer.submit(self.persist_posture_trace, trace)

    def persist_posture_trace(self, trace):
        """Off-control-thread evidence export; failure must not affect effort."""
        filename = os.path.join(self.trace_directory,
                                f"go2_{trace['epoch']}_{trace['event'].lower()}.json")
        try:
            payload = json.dumps(trace, allow_nan=False)
        except (ValueError, TypeError) as error:
            self.get_logger().error(f"[LOCOMOTION_TRACE_FAILED] event={trace['event']} error={error}")
            return
        try:
            os.makedirs(self.trace_directory, exist_ok=True)
            with open(filename, "x", encoding="utf-8") as output:
                output.write(payload)
            self.get_logger().info(f"[LOCOMOTION_TRACE_SAVED] event={trace['event']} file={filename}")
        except OSError as error:
            self.get_logger().error(f"[LOCOMOTION_TRACE_FAILED] file={filename} error={error}")
        self.fault_trace_pub.publish(String(data=payload))

    def observe_leg_extension_risk(self):
        """Low FK reach is evidence, not ground clearance or a fall verdict."""
        low = self.support_height < self.min_body_height
        self.low_extension_cycles = self.low_extension_cycles + 1 if low else 0
        if not low:
            if self.low_extension_active:
                self.get_logger().info("[LEG_EXTENSION_RECOVERED] policy support remained active")
            self.low_extension_active = False
        elif self.low_extension_cycles >= self.fault_cycles_required:
            if not self.low_extension_active:
                self.record_posture_event("LOW_EXTENSION_RISK", "LOW_LEG_EXTENSION_ONLY")
            self.low_extension_active = True
            self.get_logger().warning(
                f"[LEG_EXTENSION_RISK] reach={self.support_height:.3f}m "
                f"threshold={self.min_body_height:.3f}m; not measured ground clearance; "
                "policy continues, no pose/map invalidation",
                throttle_duration_sec=1.0)

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
        if not all(math.isfinite(v) for name in JOINT_NAMES for v in values[name]):
            self.have_joint_state = False
            self.set_state(LocomotionState.STATE_FAULT, 'JOINT_FEEDBACK_INVALID')
            return
        self.q[:] = [values[name][0] for name in JOINT_NAMES]
        self.dq[:] = [values[name][1] for name in JOINT_NAMES]
        now = self.get_clock().now().nanoseconds
        if not self.have_joint_state:
            self.history.clear()
        self.have_joint_state = True
        self.last_joint_ns = now
        self.sensor_source_ns['joint'] = message_stamp_ns(msg)

    def imu_callback(self, msg):
        if not all(math.isfinite(v) for v in (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)):
            self.have_imu = False
            self.set_state(LocomotionState.STATE_FAULT, 'IMU_FEEDBACK_INVALID')
            return
        self.base_ang_vel[:] = [
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        q = msg.orientation
        try:
            self.gravity_body = projected_gravity_from_quaternion(q.x, q.y, q.z, q.w)
        except ValueError:
            self.have_imu = False
            self.set_state(LocomotionState.STATE_FAULT, "IMU_QUATERNION_INVALID")
            return
        self.have_imu = True
        self.last_imu_ns = self.get_clock().now().nanoseconds
        self.sensor_source_ns['imu'] = message_stamp_ns(msg)

    def cmd_callback(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            self.raw_cmd.fill(0.)
            self.set_state(LocomotionState.STATE_FAULT, 'MOTION_COMMAND_INVALID')
            return
        self.raw_cmd[:] = np.clip(
            [msg.linear.x, msg.linear.y, msg.angular.z],
            [-2.0, -1.0, -2.0], [2.0, 1.0, 2.0])
        self.last_cmd_ns = self.get_clock().now().nanoseconds

    def odom_callback(self, msg):
        if not all(math.isfinite(v) for v in (
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)):
            self.last_odom_ns = 0
            self.odom_pose_valid = False
            self.set_state(LocomotionState.STATE_FAULT, 'ODOM_FEEDBACK_INVALID')
            return
        self.last_odom_ns = self.get_clock().now().nanoseconds
        self.sensor_source_ns['odom'] = message_stamp_ns(msg)
        self.body_height = float(msg.pose.pose.position.z)
        self.base_lin_vel[:] = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        q = msg.pose.pose.orientation
        try:
            gravity = projected_gravity_from_quaternion(q.x, q.y, q.z, q.w)
            if self.mode_name == 'planning_kinematic':
                self.odom_pose_valid = planar_pose_valid(
                    q.x, q.y, q.z, q.w, self.body_height,
                    self.max_tilt_cos, self.min_body_height)
            else:
                # World elevation is not clearance above the ground. Physical
                # collapse is checked from fresh leg/IMU posture in control_loop.
                self.odom_pose_valid = (math.isfinite(self.body_height)
                                        and tilt_cos(gravity) >= self.max_tilt_cos)
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

    def fault_output(self, now):
        if self.mode_name != 'rl_effort':
            self.zero_output()
            return
        fresh = (self.have_joint_state and self.have_imu and self.odom_fresh(now)
                 and 0 <= (now-self.last_joint_ns)*1e-9 <= self.state_timeout
                 and 0 <= (now-self.last_imu_ns)*1e-9 <= self.state_timeout)
        finite = all(np.isfinite(v).all() for v in
                     (self.q, self.dq, self.gravity_body, self.base_lin_vel, self.base_ang_vel))
        extension = leg_extension_height(self.q, self.gravity_body) if finite else 0.
        mode = fault_support_mode(self.reason, fresh, finite, extension,
                                  tilt_cos(self.gravity_body), float(np.linalg.norm(self.base_lin_vel)),
                                  float(np.linalg.norm(self.base_ang_vel)), float(np.max(np.abs(self.dq))))
        # Once a hold loses its validity, never re-capture a different pose
        # automatically. Navigation remains latched and requires an explicit restart.
        if self.fault_output_mode in ('DAMPING_ONLY', 'DISABLED') and mode == 'SUPPORT_HOLD':
            mode = 'DAMPING_ONLY'
        if mode != self.fault_output_mode:
            self.fault_output_mode = mode
            self.get_logger().warning(
                f'[FAULT_ACTUATION] mode={mode} reason={self.reason}; navigation remains disabled')
        if mode == 'SUPPORT_HOLD':
            if self.fault_hold_target is None:
                self.fault_hold_target = self.q.copy()
                self.fault_hold_feedforward = np.clip(
                    self.last_effort, -.25*self.torque_limits, .25*self.torque_limits)
            self.publish_effort(hold_effort(self.fault_hold_target, self.fault_hold_feedforward,
                                self.q, self.dq, 20., 4., .5*self.torque_limits))
        elif mode == 'DAMPING_ONLY':
            self.publish_effort(np.clip(-2.*self.dq, -.25*self.torque_limits, .25*self.torque_limits))
        else:
            self.zero_output()

    def posture_error(self, target=None) -> float:
        return compute_posture_error(
            self.q, DEFAULT_DOF_POS if target is None else target)

    def compute_observation(self, command=None):
        obs = np.zeros(45, dtype=np.float32)
        obs[0:3] = self.base_ang_vel * 0.25
        obs[3:6] = self.gravity_body.astype(np.float32)
        cmd = (self.filtered_cmd if command is None
               else np.asarray(command, dtype=np.float64))
        obs[6:9] = cmd * self.policy_command_scale
        obs[9:21] = (self.q - DEFAULT_DOF_POS).astype(np.float32)
        obs[21:33] = self.dq.astype(np.float32) * 0.05
        obs[33:45] = self.last_action
        return np.clip(obs, -100.0, 100.0)

    def evaluate_policy(self, command):
        """Queue one observation for the independent ONNX inference worker."""
        observation_wall_ns = time.monotonic_ns()
        observation_ros_ns = self.get_clock().now().nanoseconds
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
            # one 20 ms policy period; the 500 Hz PD loop must never block.
            self.policy_sequence += 1
            timing = {
                'sequence': self.policy_sequence,
                'episode': self.policy_episode,
                'previous_action_sequence': self.policy_last_installed_sequence,
                'command': np.asarray(command).tolist(),
                'observation_ros_ns': observation_ros_ns,
                'observation_wall_ns': observation_wall_ns,
                'observation_interval_ros_ms': diagnostic_age_ms(
                    observation_ros_ns, self.policy_last_observation_ros_ns),
                'sensor_source_ns': self.sensor_source_ns.copy(),
                'sensor_receipt_ns': dict(
                    joint=self.last_joint_ns if self.have_joint_state else None,
                    imu=self.last_imu_ns if self.have_imu else None,
                    odom=self.last_odom_ns or None),
                'queued_wall_ns': time.monotonic_ns(),
            }
            self.policy_last_observation_ros_ns = observation_ros_ns
            self.policy_timing_counts['requested'] += 1
            self.policy_timing_counts['request_overwrites'] += int(self.policy_request is not None)
            self.policy_request = dict(inputs=inputs, timing=timing)
        self.policy_event.set()
        return True

    def policy_worker(self):
        """Run ONNX off the actuator timer so PD feedback remains at 500 Hz."""
        last_estimator = None
        while not self.policy_stop.is_set():
            self.policy_event.wait(timeout=0.1)
            self.policy_event.clear()
            if self.policy_stop.is_set():
                break
            with self.policy_lock:
                request = self.policy_request
                self.policy_request = None
                self.policy_inflight = (request['timing'] if request else None)
            if request is None:
                continue
            try:
                timing = request['timing'].copy()
                statistics = getattr(self.onnx_session, 'statistics', None)
                before = statistics()[1] if statistics is not None else None
                start_wall_ns = time.monotonic_ns()
                start_ros_ns = self.get_clock().now().nanoseconds
                action = np.asarray(
                    self.onnx_session.run(["action"], request['inputs'])[0],
                    dtype=np.float32).reshape(-1)
                end_wall_ns = time.monotonic_ns()
                if action.shape != (12,) or not np.all(np.isfinite(action)):
                    raise RuntimeError(f"invalid action {action}")
                after = statistics()[1] if statistics is not None else None
                # Legacy has no exposed estimator/cache counters: unknown,
                # not falsely reported as a measured estimator frequency.
                refreshed = after > before if before is not None else None
                timing.update(
                    inference_start_wall_ns=start_wall_ns,
                    inference_end_wall_ns=end_wall_ns,
                    queue_wall_ms=diagnostic_age_ms(start_wall_ns, timing['queued_wall_ns']),
                    inference_wall_ms=diagnostic_age_ms(end_wall_ns, start_wall_ns),
                    estimator_refreshed=refreshed, estimator_refresh_count=after)
                if last_estimator is not None and last_estimator['episode'] != timing['episode']:
                    last_estimator = None
                if refreshed:
                    timing.update(
                        estimator_observation_interval_ros_ms=diagnostic_age_ms(
                            timing['observation_ros_ns'],
                            last_estimator['observation_ros_ns'] if last_estimator else None),
                        estimator_execution_interval_ros_ms=diagnostic_age_ms(
                            start_ros_ns, last_estimator['start_ros_ns'] if last_estimator else None),
                        estimator_interval_wall_ms=diagnostic_age_ms(
                            start_wall_ns, last_estimator['start_wall_ns'] if last_estimator else None),
                        estimator_sequence_gap=(timing['sequence'] - last_estimator['sequence']
                                                if last_estimator else None))
                    last_estimator = dict(
                        episode=timing['episode'], sequence=timing['sequence'],
                        observation_ros_ns=timing['observation_ros_ns'],
                        start_ros_ns=start_ros_ns, start_wall_ns=start_wall_ns)
                with self.policy_lock:
                    timing['result_ready_wall_ns'] = time.monotonic_ns()
                    self.policy_timing_counts['inferred'] += 1
                    self.policy_timing_counts['result_overwrites'] += int(self.policy_result is not None)
                    self.policy_result = dict(action=np.clip(action, -100.0, 100.0), timing=timing)
                    self.policy_inference_records.append(timing)
                    self.policy_inflight = None
            except Exception as error:  # reported by the ROS thread
                with self.policy_lock:
                    self.policy_timing_counts['errors'] += 1
                    self.policy_error = str(error)
                    self.policy_inflight = None

    def consume_policy_result(self):
        """Apply the newest asynchronous action without blocking control."""
        with self.policy_lock:
            error = self.policy_error
            self.policy_error = None
            result = self.policy_result
            self.policy_result = None
        if error is not None:
            self.set_state(LocomotionState.STATE_FAULT, "ONNX_INFERENCE_FAILED")
            self.get_logger().error(error)
            self.zero_output()
            return False
        if result is None:
            return True
        action = result['action']
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
        # "Installed" means the desired target has been assigned; transport
        # delivery to Gazebo/the actuator is a separate, unmeasured boundary.
        install_ros_ns = self.get_clock().now().nanoseconds
        install_wall_ns = time.monotonic_ns()
        timing = result['timing'].copy()
        timing.update(
            install_ros_ns=install_ros_ns, install_wall_ns=install_wall_ns,
            result_wait_wall_ms=diagnostic_age_ms(install_wall_ns, timing['result_ready_wall_ns']),
            observation_to_install_wall_ms=diagnostic_age_ms(install_wall_ns, timing['observation_wall_ns']),
            observation_to_install_ros_ms=diagnostic_age_ms(install_ros_ns, timing['observation_ros_ns']),
            newer_requests=self.policy_sequence - timing['sequence'],
            episode_matches=timing['episode'] == self.policy_episode)
        for sensor, source_ns in timing['sensor_source_ns'].items():
            timing[f'{sensor}_sample_age_ms'] = diagnostic_age_ms(install_ros_ns, source_ns)
            timing[f'{sensor}_receipt_age_ms'] = diagnostic_age_ms(
                install_ros_ns, timing['sensor_receipt_ns'][sensor])
        with self.policy_lock:
            self.policy_last_installed_sequence = timing['sequence']
            self.policy_timing_counts['installed'] += 1
            self.policy_latest_timing = timing
            self.policy_timing_records.append(timing)
        return True

    def publish_policy_timing(self):
        """Report time association without changing action acceptance or HOLD."""
        now_ros_ns = self.get_clock().now().nanoseconds
        now_wall_ns = time.monotonic_ns()
        with self.policy_lock:
            records = list(self.policy_timing_records)
            self.policy_timing_records.clear()
            inferences = list(self.policy_inference_records)
            self.policy_inference_records.clear()
            totals = self.policy_timing_counts.copy()
            window = {key: value - self.policy_timing_report_counts[key]
                      for key, value in totals.items()}
            self.policy_timing_report_counts = totals
            latest = self.policy_latest_timing
            pending = self.policy_request
            inflight = self.policy_inflight
        duration = diagnostic_age_ms(now_wall_ns, self.policy_timing_report_wall_ns)
        self.policy_timing_report_wall_ns = now_wall_ns
        summary = summarize_policy_timing(records)
        # Inference/estimator statistics must include results overwritten
        # before installation. They are not execution/installed samples.
        summary.update(summarize_policy_timing(inferences))
        report = dict(
            schema_version=1, epoch=self.epoch, episode=self.policy_episode,
            stamp_ros_ns=now_ros_ns, stamp_wall_ns=now_wall_ns,
            state=int(self.state), reason=self.reason, blind=True,
            window_wall_ms=duration, window_counts=window, totals=totals,
            installed_samples=len(records), summary=summary, latest=latest,
            last_install_age_ros_ms=diagnostic_age_ms(
                now_ros_ns, latest['install_ros_ns'] if latest else None),
            last_install_age_wall_ms=diagnostic_age_ms(
                now_wall_ns, latest['install_wall_ns'] if latest else None),
            pending_sequence=pending['timing']['sequence'] if pending else None,
            pending_age_wall_ms=diagnostic_age_ms(
                now_wall_ns, pending['timing']['queued_wall_ns'] if pending else None),
            inflight_sequence=inflight['sequence'] if inflight else None,
            inflight_observation_age_wall_ms=diagnostic_age_ms(
                now_wall_ns, inflight['observation_wall_ns'] if inflight else None),
            current_sensor_sample_age_ms={
                name: diagnostic_age_ms(now_ros_ns, stamp)
                for name, stamp in self.sensor_source_ns.items()},
            current_sensor_receipt_age_ms={
                name: diagnostic_age_ms(now_ros_ns, stamp) for name, stamp in dict(
                    joint=self.last_joint_ns if self.have_joint_state else None,
                    imu=self.last_imu_ns if self.have_imu else None,
                    odom=self.last_odom_ns or None).items()})
        self.policy_timing_pub.publish(String(data=json.dumps(report, allow_nan=False)))
        def maximum(field):
            value = summary.get(field, {}).get('max')
            return f"{value:.3f}" if value is not None else 'unknown'
        self.get_logger().info(
            f"[POLICY_TIMING] requested/inferred/installed="
            f"{window['requested']}/{window['inferred']}/{window['installed']} "
            f"overwritten_request/result={window['request_overwrites']}/{window['result_overwrites']} "
            f"queue_wall_max_ms={maximum('queue_wall_ms')} "
            f"inference_wall_max_ms={maximum('inference_wall_ms')} "
            f"install_wall_max_ms={maximum('observation_to_install_wall_ms')} "
            f"estimator_obs_ros_max_ms={maximum('estimator_observation_interval_ros_ms')} "
            f"joint/imu_sample_ros_max_ms={maximum('joint_sample_age_ms')}/{maximum('imu_sample_age_ms')} "
            f"last_seq={latest['sequence'] if latest else None} state={self.reason}")

    def shutdown(self):
        self.policy_stop.set()
        self.policy_event.set()
        if self.policy_thread is not None:
            self.policy_thread.join(timeout=2.0)
        self.trace_writer.shutdown(wait=True)

    def publish_target_effort(self, target, kp, kd, torque_limits=None):
        torque_limits = (self.torque_limits if torque_limits is None
                         else torque_limits)
        unclipped = kp * (np.asarray(target) - self.q) - kd * self.dq
        effort = pd_effort(
            target, self.q, self.dq, kp, kd, torque_limits)
        self.publish_effort(effort)
        self.last_effort = effort.copy()
        if self.diagnostics_pub is not None and self.policy_cycle == 0:
            self.motion_trace.append({
                'stamp_ns': self.get_clock().now().nanoseconds,
                'body_world_z': self.body_height, 'leg_extension_height': self.support_height,
                'cmd_raw': self.raw_cmd.astype(float).tolist(),
                'cmd_filtered': self.filtered_cmd.astype(float).tolist(),
                'gravity': self.gravity_body.astype(float).tolist(),
                'base_velocity': self.base_lin_vel.astype(float).tolist(),
                'q': self.q.astype(float).tolist(), 'dq': self.dq.astype(float).tolist(),
                'action': self.last_action.astype(float).tolist(),
                'target': np.asarray(target).astype(float).tolist(),
                'effort': effort.astype(float).tolist()})
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
        # Reset the recurrent state only on entry into a new policy episode,
        # never on zero command, HOLD, a new trajectory or a new Explorer goal.
        reset_policy = getattr(self.onnx_session, "reset", None)
        if reset_policy is not None:
            reset_policy()
        self.getup_phase = "POLICY"
        self.desired_target = DEFAULT_DOF_POS.copy()
        self.policy_cycle = 0
        self.policy_handoff_elapsed = 0.0
        self.stable_cycles = 0
        self.history.clear()
        observation = self.compute_observation(np.zeros(3, dtype=np.float64))
        self.history.extend(observation.copy() for _ in range(self.history.maxlen))
        self.policy_episode = getattr(self, 'policy_episode', 0) + 1
        self.policy_last_observation_ros_ns = None

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
            self.fault_output(now)
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

        self.support_height = leg_extension_height(self.q, self.gravity_body)

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
                      and self.support_height >= self.stand_min_body_height
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
            self.observe_leg_extension_risk()
            body_bad = (not self.odom_fresh(now)
                        or not self.odom_pose_valid
                        or tilt_cos(self.gravity_body) < self.max_tilt_cos)
            self.fault_cycles = self.fault_cycles + 1 if body_bad else 0
            if self.fault_cycles >= self.fault_cycles_required:
                self.set_state(
                    LocomotionState.STATE_FAULT,
                    "BODY_POSE_INVALID_MAP_RESTART_REQUIRED")
                self.get_logger().error(
                    f"[BODY_POSE_FAULT] body_world_z={self.body_height:.3f}m "
                    f"leg_extension_height={self.support_height:.3f}m "
                    f"tilt_cos={tilt_cos(self.gravity_body):.3f}; restart "
                    "the run to discard the map built from the invalid pose")
                self.fault_output(now)
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
