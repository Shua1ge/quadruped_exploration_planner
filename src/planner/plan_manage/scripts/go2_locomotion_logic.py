"""Pure locomotion logic for the Go2 supervisor.

Kept free of ROS imports so the regression tests can run as a plain
``unittest`` process under ``ctest`` without sourcing the ROS environment.
"""

import math

import numpy as np

JOINT_NAMES = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
)
DEFAULT_DOF_POS = np.array(
    [0.1, 0.8, -1.5, -0.1, 0.8, -1.5,
     0.1, 1.0, -1.5, -0.1, 1.0, -1.5], dtype=np.float64)
GRAVITY_WORLD = np.array([0.0, 0.0, -1.0], dtype=np.float64)

# The policy, JointState name lookup, and effort controller all use this same
# leg-major order.  Keep the actuator limits in that order as part of the
# deployment contract instead of using one synthetic limit for every motor.
GO2_TORQUE_LIMITS = np.array(
    [23.7, 23.7, 35.55,
     23.7, 23.7, 35.55,
     23.7, 23.7, 35.55,
     23.7, 23.7, 35.55], dtype=np.float64)

# The exported Go2 policy was deployed with a 1.2 m/s initial forward command.
# Clamping it to 0.25 m/s produced a repeatable static three-leg equilibrium:
# the RR calf action stayed near -2.43 for an entire eight-second capture while
# the planner requested about 0.75 m/s.  These limits admit the trained command
# range; advance_limited_command still provides the start/stop slew guard.
GO2_COMMAND_LIMITS = np.array([1.2, 0.5, 1.0], dtype=np.float64)


def interpolate_joint_target(start, target, elapsed, duration):
    """Linearly interpolate a twelve-joint target with clamped progress."""
    start = np.asarray(start, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if start.shape != (12,) or target.shape != (12,):
        raise ValueError("start and target must contain twelve joints")
    if duration <= 0.0:
        raise ValueError("duration must be positive")
    progress = float(np.clip(float(elapsed) / float(duration), 0.0, 1.0))
    return start + progress * (target - start), progress


def projected_gravity_from_quaternion(x, y, z, w):
    """Return world gravity expressed in the body frame.

    A level robot must report ``[0, 0, -1]``: the observation convention used by
    the trained policy expects gravity pointing down the body z axis.
    """
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        raise ValueError("IMU quaternion must be finite")
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("IMU quaternion has invalid norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
    return rotation.T @ GRAVITY_WORLD


def pd_effort(target, position, velocity, kp, kd, torque_limit):
    """Torque-limited joint PD, matching the training deployment contract."""
    effort = kp * (target - position) - kd * velocity
    return np.clip(effort, -torque_limit, torque_limit)


def hold_effort(target, feedforward, position, velocity,
                kp, kd, torque_limit):
    """Torque-limited hold PD around the support effort learned in a window."""
    effort = (np.asarray(feedforward, dtype=np.float64)
              + kp * (np.asarray(target) - np.asarray(position))
              - kd * np.asarray(velocity))
    return np.clip(effort, -torque_limit, torque_limit)


def estimate_hold(position_samples, velocity_samples, effort_samples):
    """Return robust per-joint position and support-effort window estimates."""
    positions = np.asarray(position_samples, dtype=np.float64)
    velocities = np.asarray(velocity_samples, dtype=np.float64)
    efforts = np.asarray(effort_samples, dtype=np.float64)
    if (positions.ndim != 2 or velocities.ndim != 2 or efforts.ndim != 2
            or positions.shape != velocities.shape
            or positions.shape != efforts.shape
            or positions.shape[0] == 0 or positions.shape[1] != 12):
        raise ValueError("q/dq/effort samples must be matching non-empty [N, 12] arrays")
    return np.median(positions, axis=0), np.median(efforts, axis=0)


def blend_effort(policy_effort, hold_command, progress):
    """Smoothly blend policy and hold effort without a command discontinuity."""
    progress = float(np.clip(progress, 0.0, 1.0))
    alpha = progress * progress * (3.0 - 2.0 * progress)
    return ((1.0 - alpha) * np.asarray(policy_effort, dtype=np.float64)
            + alpha * np.asarray(hold_command, dtype=np.float64))


def posture_error(position, default=None):
    """Largest per-joint-group stance error, in radians.

    Hip, thigh and calf are compared separately.  A folded leg must not be
    masked by another joint whose error happens to wrap to a smaller value.
    """
    default = DEFAULT_DOF_POS if default is None else default
    return float(max(
        np.max(np.abs(default[offset::3] - position[offset::3]))
        for offset in range(3)))


def tilt_cos(gravity_body):
    """Cosine of the body tilt angle inferred from body-frame gravity."""
    return -float(gravity_body[2])


def planar_pose_valid(x, y, z, w, body_height, max_tilt_cos,
                      min_body_height):
    """Whether odometry still satisfies the kinematic proxy map contract."""
    gravity_body = projected_gravity_from_quaternion(x, y, z, w)
    return (tilt_cos(gravity_body) >= max_tilt_cos
            and float(body_height) >= float(min_body_height))


def leg_extension_height(position, gravity_body):
    """Median downward foot reach, NOT measured height above terrain.

    Go2 FK constants mirror go2_description/xacro/const.xacro. World Z does
    not enter the calculation: descending a slope cannot become a collapse
    just because the map datum is zero. No contact or ground is inferred.
    """
    q = np.asarray(position, dtype=np.float64)
    g = np.asarray(gravity_body, dtype=np.float64)
    if q.shape != (12,) or g.shape != (3,) or not np.isfinite(q).all() or not np.isfinite(g).all():
        raise ValueError('foot reach requires finite Go2 joints and gravity')
    hip, thigh, calf = q.reshape(4, 3).T
    side = np.array([1., -1., 1., -1.])
    front = np.array([1., 1., -1., -1.])
    down = -.213 * (np.cos(thigh) + np.cos(thigh + calf))
    feet = np.column_stack((front*.1934 - .213*(np.sin(thigh)+np.sin(thigh+calf)),
                            side*.0465 + side*.0955*np.cos(hip) - down*np.sin(hip),
                            side*.0955*np.sin(hip) + down*np.cos(hip)))
    return float(np.median(feet @ g))


def fault_support_mode(reason, fresh, finite, extension, tilt, linear_speed, angular_speed, joint_speed):
    """Navigation stays FAULT. A hold is admitted only from a settled posture.

    Bad feedback must disable effort; a falling/tipped robot receives bounded
    damping, never a blind get-up. A stationary upright body-pose fault may
    retain support without inferring task/motion success.
    """
    if not fresh or not finite:
        return 'DISABLED'
    if (reason == 'BODY_POSE_INVALID_MAP_RESTART_REQUIRED'
            and extension >= .18 and tilt >= math.cos(math.radians(30))
            and linear_speed <= .10 and angular_speed <= .5 and joint_speed <= 2.):
        return 'SUPPORT_HOLD'
    return 'DAMPING_ONLY'


def advance_limited_command(current, requested, limits, slew_rates, dt):
    """Clamp a velocity request and approach it with a per-axis slew limit."""
    current = np.asarray(current, dtype=np.float64)
    requested = np.asarray(requested, dtype=np.float64)
    limits = np.asarray(limits, dtype=np.float64)
    slew_rates = np.asarray(slew_rates, dtype=np.float64)
    if current.shape != (3,) or requested.shape != (3,):
        raise ValueError("current and requested commands must contain three axes")
    if limits.shape != (3,) or slew_rates.shape != (3,):
        raise ValueError("limits and slew_rates must contain three axes")
    if np.any(limits < 0.0) or np.any(slew_rates < 0.0) or dt < 0.0:
        raise ValueError("command limits, slew rates and dt must be non-negative")
    target = np.clip(requested, -limits, limits)
    max_delta = slew_rates * float(dt)
    return current + np.clip(target - current, -max_delta, max_delta)


def body_motion_stable(linear_velocity, angular_velocity,
                       max_linear_speed, max_angular_speed):
    """Whether the floating base is sufficiently settled to admit navigation."""
    linear_velocity = np.asarray(linear_velocity, dtype=np.float64)
    angular_velocity = np.asarray(angular_velocity, dtype=np.float64)
    if linear_velocity.shape != (3,) or angular_velocity.shape != (3,):
        raise ValueError("base velocities must contain three axes")
    return (np.linalg.norm(linear_velocity) <= float(max_linear_speed)
            and np.linalg.norm(angular_velocity) <= float(max_angular_speed))


def joint_motion_stable(velocity, max_joint_speed):
    """Whether every actuated joint is slow enough for a settling sample."""
    velocity = np.asarray(velocity, dtype=np.float64)
    if velocity.shape != (12,):
        raise ValueError("joint velocity must contain twelve joints")
    return np.max(np.abs(velocity)) <= float(max_joint_speed)


def upright_for_hold(body_height, gravity_body, linear_velocity,
                     angular_velocity, min_height, min_tilt_cos,
                     max_linear_speed, max_angular_speed):
    """Whether recovery is upright and slow enough to capture a hold."""
    return (float(body_height) >= float(min_height)
            and tilt_cos(gravity_body) >= float(min_tilt_cos)
            and body_motion_stable(
                linear_velocity, angular_velocity,
                max_linear_speed, max_angular_speed))


def message_stamp_ns(message):
    """Diagnostic source time; an absent/zero header is unknown, not fresh."""
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value != 0 else None


def diagnostic_age_ms(now_ns, stamp_ns):
    """Subtract times in the SAME clock domain; retain future/rewound stamps."""
    return None if stamp_ns is None else (now_ns - stamp_ns) * 1e-6


def summarize_policy_timing(records):
    """One low-rate summary of installed actions, never an actuator decision."""
    fields = (
        'observation_interval_ros_ms', 'queue_wall_ms', 'inference_wall_ms',
        'result_wait_wall_ms', 'observation_to_install_wall_ms',
        'observation_to_install_ros_ms', 'joint_sample_age_ms',
        'imu_sample_age_ms', 'odom_sample_age_ms',
        'estimator_observation_interval_ros_ms',
        'estimator_execution_interval_ros_ms', 'estimator_interval_wall_ms',
    )
    summary = {}
    for field in fields:
        values = [row[field] for row in records
                  if row.get(field) is not None and math.isfinite(row[field])]
        if values:
            summary[field] = {
                'count': len(values), 'mean': sum(values) / len(values),
                'min': min(values), 'max': max(values),
            }
    return summary
