#!/usr/bin/env python3
"""Drive the Go2 in Gazebo with a trained locomotion policy.

The policy is a TorchScript checkpoint exported from an rl_sar-style training
pipeline (HIMLoco, robot_lab, ...), and it is loaded together with the YAML that
was used to train it.  The YAML is the single source of truth for the interface:
observation order and scales, history length, action scale, default joint pose,
joint mapping and the PD gains of each phase.  Nothing about the interface is
hardcoded here, because a checkpoint cannot be used correctly without the
matching specification.

The node closes the loop at 50 Hz (dt * decimation of the YAML):

    IMU + joint states + cmd_vel -> observation -> policy -> joint torques
                                         ^                        |
                                         +------------------------+

Torques are published to an effort controller, not as position targets:
gz_ros2_control turns a position command into a velocity servo with a fixed
proportional gain, which cannot reproduce the torque-limited PD that a
locomotion policy was trained against.
"""

import math
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import torch
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String

GRAVITY_WORLD = np.array([0.0, 0.0, -1.0])

# The index space of `joint_mapping` in an rl_sar config: the Go2 motor order,
# which is also the order the real robot's SDK uses.  It is NOT the order the
# controller expects, and confusing the two silently swaps the left and right
# front legs.
MOTOR_JOINT_ORDER = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]

# Order of the command array published to the effort controller; it must match
# the controller's joint table (ros2_controllers.yaml).
COMMAND_JOINT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]


def quat_to_rotation(x, y, z, w):
    """Quaternion (body -> world) as a 3x3 rotation matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class PolicyConfig:
    """Interface specification of one checkpoint, read from an rl_sar YAML."""

    def __init__(self, path: str, key: str, policy_override: str = ""):
        raw = yaml.safe_load(Path(path).read_text())
        if key not in raw:
            raise RuntimeError(
                f"key '{key}' not in {path}; available: {sorted(raw.keys())}")
        cfg = raw[key]

        self.key = key
        self.policy_file = policy_override or str(Path(path).parent / cfg["model_name"])
        self.obs_terms = list(cfg["observations"])
        self.obs_history = list(cfg["observations_history"])
        self.num_obs = int(cfg["num_observations"])
        self.clip_obs = float(cfg.get("clip_obs", 100.0))

        self.n_dofs = int(cfg["num_of_dofs"])
        self.default_dof_pos = np.array(cfg["default_dof_pos"], dtype=np.float64)
        self.action_scale = np.array(cfg["action_scale"], dtype=np.float64)
        self.joint_mapping = [int(i) for i in cfg["joint_mapping"]]
        self.torque_limits = np.array(cfg["torque_limits"], dtype=np.float64)

        self.rl_kp = np.array(cfg["rl_kp"], dtype=np.float64)
        self.rl_kd = np.array(cfg["rl_kd"], dtype=np.float64)
        self.fixed_kp = np.array(cfg.get("fixed_kp", cfg["rl_kp"]), dtype=np.float64)
        self.fixed_kd = np.array(cfg.get("fixed_kd", cfg["rl_kd"]), dtype=np.float64)

        self.scale = {
            "ang_vel": float(cfg["ang_vel_scale"]),
            "dof_pos": float(cfg["dof_pos_scale"]),
            "dof_vel": float(cfg["dof_vel_scale"]),
        }
        self.commands_scale = np.array(cfg["commands_scale"], dtype=np.float64)
        # rl_sar uses the body-frame angular velocity for the ROS 2 backend.
        self.ang_vel_in_body_frame = True

        unknown = set(self.obs_terms) - {
            "commands", "ang_vel", "gravity_vec", "dof_pos", "dof_vel", "actions"}
        if unknown:
            raise RuntimeError(f"unsupported observation terms: {sorted(unknown)}")

        for name, arr in (("default_dof_pos", self.default_dof_pos),
                          ("action_scale", self.action_scale),
                          ("rl_kp", self.rl_kp), ("rl_kd", self.rl_kd)):
            if arr.shape != (self.n_dofs,):
                raise RuntimeError(f"{name} has shape {arr.shape}, expected ({self.n_dofs},)")
        if sorted(self.joint_mapping) != list(range(self.n_dofs)):
            raise RuntimeError(f"joint_mapping is not a permutation: {self.joint_mapping}")


class Go2PolicyNode(Node):
    def __init__(self):
        super().__init__("go2_policy_node")

        default_config = str(
            Path.home() / "cave_team_ws/src/rl_sar/policy/go2/himloco/config.yaml")
        self.declare_parameter("config_file", default_config)
        self.declare_parameter("policy_key", "go2/himloco")
        self.declare_parameter("policy_file", "")
        self.declare_parameter("control_dt", 0.02)          # dt * decimation
        self.declare_parameter("getup_duration", 2.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("max_tilt", 1.0)             # rad before passive
        self.declare_parameter("auto_start", True)
        self.declare_parameter("motor_joint_names", MOTOR_JOINT_ORDER)
        self.declare_parameter("command_joint_names", COMMAND_JOINT_ORDER)
        # "odom" (default) or "imu": the base orientation and angular velocity.
        # The odometry comes from the model's odometry publisher, so it does not
        # depend on the world providing a Sensors system.
        self.declare_parameter("orientation_source", "odom")
        self.declare_parameter("debug_period", 0.0)
        # use_sim_time is declared by rclpy itself.

        cfg = PolicyConfig(
            self.get_parameter("config_file").value,
            self.get_parameter("policy_key").value,
            self.get_parameter("policy_file").value)
        self.cfg = cfg

        self.model = torch.jit.load(cfg.policy_file, map_location="cpu")
        self.model.eval()

        self.dt = float(self.get_parameter("control_dt").value)
        self.getup_duration = float(self.get_parameter("getup_duration").value)
        self.cmd_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.max_tilt = float(self.get_parameter("max_tilt").value)
        self.motor_names = [str(n) for n in self.get_parameter("motor_joint_names").value]
        command_names = [str(n) for n in self.get_parameter("command_joint_names").value]
        for label, names in (("motor_joint_names", self.motor_names),
                             ("command_joint_names", command_names)):
            if len(names) != cfg.n_dofs:
                raise RuntimeError(
                    f"{label} has {len(names)} entries, expected {cfg.n_dofs}")
        if sorted(self.motor_names) != sorted(command_names):
            raise RuntimeError("motor_joint_names and command_joint_names differ")
        # joint states arrive by name; keep q/dq in motor order (the index space
        # of joint_mapping) and permute once when publishing.
        self.motor_index = {name: i for i, name in enumerate(self.motor_names)}
        self.motor_to_command = [command_names.index(n) for n in self.motor_names]

        self._lock = threading.Lock()
        self.q = np.zeros(cfg.n_dofs)                 # controller joint order
        self.dq = np.zeros(cfg.n_dofs)
        self.quat = np.array([0.0, 0.0, 0.0, 1.0])    # IMU frame -> world, x y z w
        self.ang_vel = np.zeros(3)                    # IMU, body frame
        self.odom_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.odom_ang_vel = np.zeros(3)
        self.command = np.zeros(3)                    # vx, vy, wz
        self.last_cmd_time = None
        self.joint_names = None

        self.history = deque(maxlen=max(cfg.obs_history) + 1 if cfg.obs_history else 1)
        self.last_action = np.zeros(cfg.n_dofs)
        self.phase = "WAIT"
        self.phase_start = time.monotonic()
        self.q_getup_start = self.cfg.default_dof_pos.copy()
        self.hold_target = self.cfg.default_dof_pos.copy()
        self.stand_attempts = 0
        self.max_stand_attempts = 3
        self.hold_locked = False
        self.have_odom = False
        self.tilt_since = None
        self.have_joint_state = False

        imu_qos = QoSProfile(depth=10)
        imu_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.orientation_source = str(self.get_parameter("orientation_source").value)
        self.debug_period = float(self.get_parameter("debug_period").value)
        self.last_debug = 0.0
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        self.create_subscription(Imu, "imu", self._on_imu, imu_qos)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.command_pub = self.create_publisher(Float64MultiArray, "joint_effort_command", 10)
        self.status_pub = self.create_publisher(String, "go2_policy/status", 10)

        if self.orientation_source not in ("odom", "imu"):
            raise RuntimeError("orientation_source must be 'odom' or 'imu'")

        self.create_timer(self.dt, self._control_step)
        self.get_logger().info(
            f"policy '{cfg.key}' loaded from {Path(cfg.policy_file).name}: "
            f"obs={cfg.num_obs}x{len(cfg.obs_history) or 1} "
            f"terms={','.join(cfg.obs_terms)} "
            f"kp={cfg.rl_kp[0]:.0f} kd={cfg.rl_kd[0]:.1f} "
            f"mode={'auto' if self.get_parameter('auto_start').value else 'manual'}")

    # ---------------------------------------------------------------- inputs
    def _on_joint_state(self, msg: JointState):
        # Index by name, not by message order: joint_mapping refers to the
        # controller joint table, and a reordered JointState must not silently
        # scramble the observation.
        with self._lock:
            for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
                if name in self.motor_index:
                    i = self.motor_index[name]
                    self.q[i] = pos
                    self.dq[i] = vel
            self.have_joint_state = True

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        with self._lock:
            self.quat = np.array([q.x, q.y, q.z, q.w])
            self.ang_vel = np.array([msg.angular_velocity.x,
                                     msg.angular_velocity.y,
                                     msg.angular_velocity.z])

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        w = msg.twist.twist.angular
        with self._lock:
            self.odom_quat = np.array([q.x, q.y, q.z, q.w])
            self.odom_ang_vel = np.array([w.x, w.y, w.z])
            self.have_odom = True

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self.command = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
            self.last_cmd_time = time.monotonic()

    # ------------------------------------------------------------ observation
    def _base_state(self):
        """(orientation, angular velocity) of the base, in the configured source."""
        if self.orientation_source == "imu":
            return self.quat.copy(), self.ang_vel.copy()
        return self.odom_quat.copy(), self.odom_ang_vel.copy()

    def _build_observation(self) -> np.ndarray:
        with self._lock:
            q = self.q.copy()
            dq = self.dq.copy()
            quat, ang_vel = self._base_state()
            command = self.command.copy()
            last_cmd = self.last_cmd_time

        if last_cmd is None or time.monotonic() - last_cmd > self.cmd_timeout:
            command = np.zeros(3)

        # policy joint order i  <->  controller joint index joint_mapping[i]
        q_policy = q[self.cfg.joint_mapping]
        dq_policy = dq[self.cfg.joint_mapping]

        rot = quat_to_rotation(*quat)
        gravity_body = rot.T @ GRAVITY_WORLD
        ang_vel_body = ang_vel if self.cfg.ang_vel_in_body_frame else rot.T @ ang_vel

        values = {
            "commands": command * self.cfg.commands_scale,
            "ang_vel": ang_vel_body * self.cfg.scale["ang_vel"],
            "gravity_vec": gravity_body,
            "dof_pos": (q_policy - self.cfg.default_dof_pos) * self.cfg.scale["dof_pos"],
            "dof_vel": dq_policy * self.cfg.scale["dof_vel"],
            "actions": self.last_action,
        }
        frame = np.concatenate([values[term] for term in self.cfg.obs_terms])
        return np.clip(frame, -self.cfg.clip_obs, self.cfg.clip_obs)

    def _policy_action(self) -> np.ndarray:
        frame = self._build_observation()
        self.history.appendleft(frame)                      # index 0 = latest
        while len(self.history) < (max(self.cfg.obs_history) + 1):
            self.history.append(frame)

        if self.cfg.obs_history:
            stacked = np.concatenate([self.history[i] for i in self.cfg.obs_history])
        else:
            stacked = frame

        with torch.no_grad():
            action = self.model(torch.tensor(stacked, dtype=torch.float32).unsqueeze(0))
        action = np.asarray(action.squeeze(0).cpu().numpy(), dtype=np.float64)
        if action.shape != (self.cfg.n_dofs,):
            raise RuntimeError(f"policy returned shape {action.shape}, expected {(self.cfg.n_dofs,)}")
        self.last_action = action.copy()
        return action

    # -------------------------------------------------------------- control
    def _to_motor_order(self, tau_policy: np.ndarray) -> np.ndarray:
        """Scatter a policy-order torque vector into motor order."""
        tau_motor = np.zeros(self.cfg.n_dofs)
        tau_motor[self.cfg.joint_mapping] = tau_policy
        return tau_motor

    def _publish_torque(self, tau_motor: np.ndarray):
        tau_motor = np.clip(tau_motor, -self.cfg.torque_limits, self.cfg.torque_limits)
        tau_command = np.zeros(self.cfg.n_dofs)
        for motor_idx, command_idx in enumerate(self.motor_to_command):
            tau_command[command_idx] = tau_motor[motor_idx]
        msg = Float64MultiArray()
        msg.data = [float(v) for v in tau_command]
        self.command_pub.publish(msg)

    def _tilt_guard(self, rot):
        tilt = math.acos(max(-1.0, min(1.0, float(rot[2, 2]))))
        now = time.monotonic()
        if tilt > self.max_tilt:
            self.tilt_since = self.tilt_since or now
            if now - self.tilt_since > 0.5:
                return True
        else:
            self.tilt_since = None
        return False

    def _set_phase(self, phase: str):
        self.phase = phase
        self.phase_start = time.monotonic()
        self.history.clear()
        self.last_action = np.zeros(self.cfg.n_dofs)
        if phase == "HOLD":
            with self._lock:
                self.hold_target = self.q[self.cfg.joint_mapping].copy()
        elif phase == "GETUP":
            with self._lock:
                self.q_getup_start = self.q[self.cfg.joint_mapping].copy()
        msg = String()
        msg.data = phase
        self.status_pub.publish(msg)
        self.get_logger().info(f"phase -> {phase}")

    def _control_step(self):
        with self._lock:
            q = self.q.copy()
            dq = self.dq.copy()
            quat, _ = self._base_state()
            have_joint_state = self.have_joint_state
        rot = quat_to_rotation(*quat)
        q_policy = q[self.cfg.joint_mapping]
        dq_policy = dq[self.cfg.joint_mapping]

        if self.phase == "WAIT":
            # Do not torque the joints before the first joint state arrives:
            # the robot spawns standing and pushing zeros would drop it.
            if not have_joint_state or not self.get_parameter("auto_start").value:
                self._publish_torque(np.zeros(self.cfg.n_dofs))
                return
            self._set_phase("GETUP")
            return

        if self._tilt_guard(rot) and self.phase == "RL":
            self.stand_attempts += 1
            if self.stand_attempts > self.max_stand_attempts:
                self.hold_locked = True
                self.get_logger().error(
                    f"base tilted beyond {self.max_tilt:.2f} rad "
                    f"{self.stand_attempts} times; holding the current pose")
            self._set_phase("HOLD")
            return

        if self.phase == "HOLD":
            # Hold the pose the robot had when the phase started.  Going limp
            # here would drop a standing robot onto its legs before the policy
            # ever runs.
            tau = (self.cfg.fixed_kp * (self.hold_target - q_policy)
                   - self.cfg.fixed_kd * dq_policy)
            self._publish_torque(self._to_motor_order(tau))
            if not self.hold_locked and time.monotonic() - self.phase_start > 1.0:
                self._set_phase("GETUP")
            return

        if self.phase == "GETUP":
            t = time.monotonic() - self.phase_start
            alpha = min(1.0, t / max(1e-3, self.getup_duration))
            target = (1 - alpha) * self.q_getup_start + alpha * self.cfg.default_dof_pos
            tau = (self.cfg.fixed_kp * (target - q_policy)
                   - self.cfg.fixed_kd * dq_policy)
            self._publish_torque(self._to_motor_order(tau))
            if alpha >= 1.0:
                self._set_phase("RL")
            return

        # RL locomotion
        try:
            action = self._policy_action()
        except Exception as exc:                                   # noqa: BLE001
            self.get_logger().error(f"policy inference failed: {exc}")
            self._set_phase("HOLD")
            return
        target = self.cfg.default_dof_pos + self.cfg.action_scale * action
        tau = self.cfg.rl_kp * (target - q_policy) - self.cfg.rl_kd * dq_policy
        self._publish_torque(self._to_motor_order(tau))

        if (self.debug_period > 0.0
                and time.monotonic() - self.last_debug >= self.debug_period):
            self.last_debug = time.monotonic()
            with self._lock:
                command = self.command.copy()
            self.get_logger().info(
                f"debug: source={self.orientation_source} "
                f"gravity_body={np.round(rot.T @ GRAVITY_WORLD, 2).tolist()} "
                f"command={np.round(command, 2).tolist()} "
                f"action_max={np.abs(action).max():.2f} "
                f"tau_max={np.abs(tau).max():.1f} "
                f"q_err_max={np.abs(target - q_policy).max():.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = Go2PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
