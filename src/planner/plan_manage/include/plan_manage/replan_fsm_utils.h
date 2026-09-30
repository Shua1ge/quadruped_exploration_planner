#ifndef SCAN_PLANNER_REPLAN_FSM_UTILS_H_
#define SCAN_PLANNER_REPLAN_FSM_UTILS_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace scan_planner
{

struct ReferencePathLookahead
{
  bool valid{false};
  size_t progress_segment{0};
  double progress_ratio{0.0};
  Eigen::Vector3d projection{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target{Eigen::Vector3d::Zero()};
  Eigen::Vector3d tangent{Eigen::Vector3d::Zero()};
  double remaining_after_target{0.0};
};

inline ReferencePathLookahead projectReferencePathLookahead(
    const std::vector<Eigen::Vector3d> &path,
    const Eigen::Vector3d &position,
    size_t earliest_segment,
    double earliest_ratio,
    double lookahead,
    size_t search_segment_count = 20)
{
  ReferencePathLookahead result;
  if (path.size() < 2)
    return result;

  const size_t last_segment = path.size() - 2;
  earliest_segment = std::min(earliest_segment, last_segment);
  earliest_ratio = std::clamp(earliest_ratio, 0.0, 1.0);
  const size_t search_end = std::min(last_segment, earliest_segment + search_segment_count);

  double nearest_dist2 = std::numeric_limits<double>::infinity();
  size_t nearest_segment = earliest_segment;
  double nearest_ratio = earliest_ratio;
  Eigen::Vector3d nearest_projection = path[earliest_segment];

  for (size_t i = earliest_segment; i <= search_end; ++i)
  {
    const Eigen::Vector3d segment = path[i + 1] - path[i];
    const Eigen::Vector2d segment_xy = segment.head<2>();
    const double length2_xy = segment_xy.squaredNorm();
    if (length2_xy < 1e-12)
      continue;

    double ratio = (position.head<2>() - path[i].head<2>()).dot(segment_xy) / length2_xy;
    ratio = std::clamp(ratio, i == earliest_segment ? earliest_ratio : 0.0, 1.0);
    const Eigen::Vector3d projection = path[i] + ratio * segment;
    const double dist2 = (position.head<2>() - projection.head<2>()).squaredNorm();
    if (dist2 < nearest_dist2)
    {
      nearest_dist2 = dist2;
      nearest_segment = i;
      nearest_ratio = ratio;
      nearest_projection = projection;
    }
  }

  result.valid = true;
  result.progress_segment = nearest_segment;
  result.progress_ratio = nearest_ratio;
  result.projection = nearest_projection;
  result.target = nearest_projection;

  double distance_to_advance = std::max(0.0, lookahead);
  size_t target_segment = nearest_segment;
  double target_ratio = nearest_ratio;

  for (size_t i = nearest_segment; i <= last_segment; ++i)
  {
    const Eigen::Vector3d segment = path[i + 1] - path[i];
    const double length = segment.norm();
    if (length < 1e-9)
      continue;

    const double start_ratio = i == nearest_segment ? nearest_ratio : 0.0;
    const double available = (1.0 - start_ratio) * length;
    result.tangent = segment / length;
    if (distance_to_advance <= available)
    {
      target_segment = i;
      target_ratio = start_ratio + distance_to_advance / length;
      result.target = path[i] + target_ratio * segment;
      distance_to_advance = 0.0;
      break;
    }

    distance_to_advance -= available;
    target_segment = i;
    target_ratio = 1.0;
    result.target = path[i + 1];
  }

  double remaining = 0.0;
  const Eigen::Vector3d target_segment_vector = path[target_segment + 1] - path[target_segment];
  remaining += (1.0 - target_ratio) * target_segment_vector.norm();
  for (size_t i = target_segment + 1; i <= last_segment; ++i)
    remaining += (path[i + 1] - path[i]).norm();
  result.remaining_after_target = remaining;
  return result;
}

inline bool shouldResumePendingEmergencyPath(bool path_pending, bool have_target)
{
  return path_pending && have_target;
}

inline bool safetyLatchAllowsPlanning(
    bool replacement_planning_pending,
    bool emergency_path_pending,
    bool stopped_local_repair_active)
{
  return replacement_planning_pending || emergency_path_pending ||
         stopped_local_repair_active;
}

inline bool shouldStartStoppedLocalRepair(
    bool repair_pending, bool have_target, double planar_speed,
    bool pose_occupied_latched, bool temporal_safety_latched)
{
  return repair_pending && have_target && std::isfinite(planar_speed) &&
         planar_speed <= 0.05 && !pose_occupied_latched &&
         !temporal_safety_latched;
}

inline bool newPlanMayReleaseSafetyStop(
    bool pose_occupied_latched,
    bool predictive_hold_latched = false,
    bool temporal_safety_latched = false)
{
  return !pose_occupied_latched && !predictive_hold_latched &&
         !temporal_safety_latched;
}

inline bool brakingHoldMayReleaseAfterStop(
    double planar_speed, int consecutive_stopped_cycles,
    int required_stopped_cycles, bool pose_occupied_latched,
    bool planning_clearance_blocked)
{
  return std::isfinite(planar_speed) && planar_speed <= 0.05 &&
         required_stopped_cycles > 0 &&
         consecutive_stopped_cycles >= required_stopped_cycles &&
         !pose_occupied_latched && !planning_clearance_blocked;
}

inline bool clearanceViolationAllowedDuringEscape(
    bool started_inside_clearance, bool already_exited_clearance,
    bool physical_collision, double trajectory_time, double escape_deadline,
    size_t current_violations, size_t initial_violations)
{
  return started_inside_clearance && !already_exited_clearance &&
         !physical_collision && std::isfinite(trajectory_time) &&
         std::isfinite(escape_deadline) && trajectory_time >= 0.0 &&
         trajectory_time <= escape_deadline &&
         current_violations <= initial_violations;
}

inline bool clearanceEscapeConfirmedAtActualPose(
    bool escape_active, bool actual_clearance_blocked,
    int consecutive_free_cycles, int required_free_cycles)
{
  return escape_active && !actual_clearance_blocked &&
         required_free_cycles > 0 &&
         consecutive_free_cycles >= required_free_cycles;
}

inline std::vector<Eigen::Vector2d> structuredLocalRepairDirections(
    const Eigen::Vector2d &forward)
{
  Eigen::Vector2d unit_forward = forward;
  if (!unit_forward.allFinite() || unit_forward.norm() < 1e-6)
    unit_forward = Eigen::Vector2d::UnitX();
  else
    unit_forward.normalize();

  const Eigen::Vector2d left(-unit_forward.y(), unit_forward.x());
  const auto normalized = [](const Eigen::Vector2d &direction) {
    return direction.normalized();
  };
  return {
      normalized(unit_forward + left),
      normalized(unit_forward - left),
      left,
      -left,
      normalized(-0.35 * unit_forward + left),
      normalized(-0.35 * unit_forward - left),
      -unit_forward};
}

inline bool recoveryClearanceEvidenceIsNonWorsening(
    const std::vector<size_t> &violation_counts)
{
  if (violation_counts.empty())
    return false;
  for (size_t i = 1; i < violation_counts.size(); ++i)
  {
    if (violation_counts[i] > violation_counts[i - 1])
      return false;
  }
  return violation_counts.back() == 0;
}

inline double restToRestMinimumDuration(
    double distance, double speed_limit, double acceleration_limit,
    double time_margin = 1.0)
{
  if (!std::isfinite(distance) || !std::isfinite(speed_limit) ||
      !std::isfinite(acceleration_limit) || !std::isfinite(time_margin) ||
      distance <= 0.0 || speed_limit <= 0.0 ||
      acceleration_limit <= 0.0 || time_margin < 1.0)
    return 0.0;

  // A rest-to-rest triangular profile reaches 2*d/T peak speed and requires
  // 4*d/T^2 peak acceleration. Satisfy both bounds before adding a small
  // execution margin for the quadruped policy and spline fitting.
  const double speed_limited_time = 2.0 * distance / speed_limit;
  const double acceleration_limited_time =
      2.0 * std::sqrt(distance / acceleration_limit);
  return time_margin *
         std::max(speed_limited_time, acceleration_limited_time);
}

inline bool localRepairOwnsFailure(
    bool stopped_local_repair_active,
    bool structured_local_repair_active)
{
  return stopped_local_repair_active || structured_local_repair_active;
}

inline double brakingSweepHorizon(
    double speed, double maximum_deceleration, double reaction_time,
    double maximum_horizon)
{
  if (!std::isfinite(speed) || !std::isfinite(maximum_deceleration) ||
      !std::isfinite(reaction_time) || !std::isfinite(maximum_horizon) ||
      maximum_deceleration <= 0.0 || maximum_horizon < 0.0)
    return 0.0;
  return std::min(maximum_horizon,
                  std::max(0.0, reaction_time) +
                      std::max(0.0, speed) / maximum_deceleration);
}

inline double brakingSweepDistance(
    double speed, double maximum_deceleration, double reaction_time,
    double time)
{
  if (!std::isfinite(speed) || !std::isfinite(maximum_deceleration) ||
      !std::isfinite(reaction_time) || !std::isfinite(time) ||
      maximum_deceleration <= 0.0)
    return 0.0;
  const double v = std::max(0.0, speed);
  const double reaction = std::max(0.0, reaction_time);
  const double t = std::max(0.0, time);
  if (t <= reaction)
    return v * t;
  const double braking_time = std::min(t - reaction, v / maximum_deceleration);
  return v * reaction + v * braking_time -
         0.5 * maximum_deceleration * braking_time * braking_time;
}

inline double exponentialFilterAlpha(double dt, double time_constant)
{
  if (!std::isfinite(dt) || !std::isfinite(time_constant) || dt <= 0.0)
    return 0.0;
  if (time_constant <= 0.0)
    return 1.0;
  return 1.0 - std::exp(-dt / time_constant);
}

inline Eigen::Vector3d rotateBodyVelocityToWorld(
    const Eigen::Quaterniond &world_from_body,
    const Eigen::Vector3d &body_velocity)
{
  if (!world_from_body.coeffs().allFinite() ||
      !body_velocity.allFinite() || world_from_body.norm() < 1e-9)
    return Eigen::Vector3d::Zero();
  return world_from_body.normalized() * body_velocity;
}

inline double unwrapPlanarYaw(
    double previous_unwrapped_yaw, double previous_raw_yaw,
    double current_raw_yaw)
{
  if (!std::isfinite(previous_unwrapped_yaw) ||
      !std::isfinite(previous_raw_yaw) || !std::isfinite(current_raw_yaw))
    return 0.0;
  return previous_unwrapped_yaw +
         std::atan2(std::sin(current_raw_yaw - previous_raw_yaw),
                    std::cos(current_raw_yaw - previous_raw_yaw));
}

inline bool estimatePlanarMotionFromPoseHistory(
    const std::vector<double> &times,
    const std::vector<Eigen::Vector2d> &positions,
    const std::vector<double> &unwrapped_yaws,
    Eigen::Vector2d *world_velocity, double *yaw_rate)
{
  if (!world_velocity || !yaw_rate || times.size() < 3 ||
      positions.size() != times.size() ||
      unwrapped_yaws.size() != times.size())
    return false;

  const double time_origin = times.back();
  double mean_t = 0.0;
  Eigen::Vector2d mean_position = Eigen::Vector2d::Zero();
  double mean_yaw = 0.0;
  for (size_t i = 0; i < times.size(); ++i)
  {
    if (!std::isfinite(times[i]) || !positions[i].allFinite() ||
        !std::isfinite(unwrapped_yaws[i]))
      return false;
    mean_t += times[i] - time_origin;
    mean_position += positions[i];
    mean_yaw += unwrapped_yaws[i];
  }
  const double count = static_cast<double>(times.size());
  mean_t /= count;
  mean_position /= count;
  mean_yaw /= count;

  double denominator = 0.0;
  Eigen::Vector2d position_numerator = Eigen::Vector2d::Zero();
  double yaw_numerator = 0.0;
  for (size_t i = 0; i < times.size(); ++i)
  {
    const double centered_time = (times[i] - time_origin) - mean_t;
    denominator += centered_time * centered_time;
    position_numerator +=
        centered_time * (positions[i] - mean_position);
    yaw_numerator += centered_time * (unwrapped_yaws[i] - mean_yaw);
  }
  if (denominator < 1e-8 || times.back() - times.front() < 0.05)
    return false;

  *world_velocity = position_numerator / denominator;
  *yaw_rate = yaw_numerator / denominator;
  return world_velocity->allFinite() && std::isfinite(*yaw_rate);
}

inline double brakingSpeedAtTime(
    double initial_speed, double guaranteed_deceleration,
    double reaction_time, double time)
{
  if (!std::isfinite(initial_speed) ||
      !std::isfinite(guaranteed_deceleration) ||
      !std::isfinite(reaction_time) || !std::isfinite(time) ||
      guaranteed_deceleration <= 0.0)
    return 0.0;
  const double speed = std::max(0.0, initial_speed);
  const double braking_time = std::max(
      0.0, std::max(0.0, time) - std::max(0.0, reaction_time));
  return std::max(0.0, speed - guaranteed_deceleration * braking_time);
}

struct PlanarBrakingState
{
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  double yaw{0.0};
};

inline void advancePlanarBrakingState(
    PlanarBrakingState *state, double initial_speed,
    double initial_travel_heading, double initial_yaw,
    double initial_yaw_rate, double guaranteed_deceleration,
    double reaction_time, double elapsed_time, double dt)
{
  if (!state || !std::isfinite(dt) || dt <= 0.0 ||
      !std::isfinite(initial_travel_heading) ||
      !std::isfinite(initial_yaw) || !std::isfinite(initial_yaw_rate))
    return;

  const double start_speed = brakingSpeedAtTime(
      initial_speed, guaranteed_deceleration, reaction_time, elapsed_time);
  const double end_speed = brakingSpeedAtTime(
      initial_speed, guaranteed_deceleration, reaction_time,
      elapsed_time + dt);
  const double average_speed = 0.5 * (start_speed + end_speed);
  const double speed_ratio = initial_speed > 1e-6
      ? std::clamp(average_speed / initial_speed, 0.0, 1.0)
      : 0.0;
  const double average_yaw_rate = initial_yaw_rate * speed_ratio;
  const double midpoint_yaw = state->yaw + 0.5 * average_yaw_rate * dt;
  const double slip_angle = std::atan2(
      std::sin(initial_travel_heading - initial_yaw),
      std::cos(initial_travel_heading - initial_yaw));
  const double travel_heading = midpoint_yaw + slip_angle;
  state->position += average_speed * dt * Eigen::Vector2d(
      std::cos(travel_heading), std::sin(travel_heading));
  state->yaw = std::atan2(
      std::sin(state->yaw + average_yaw_rate * dt),
      std::cos(state->yaw + average_yaw_rate * dt));
}

inline bool shouldKeepExecutingAfterRollingReplanFailure(
    bool reference_path_active, bool reference_path_update_pending,
    bool safety_stop_active, bool execution_frozen,
    double remaining_time, double emergency_time)
{
  return reference_path_active && !reference_path_update_pending &&
         !safety_stop_active && !execution_frozen &&
         remaining_time > std::max(0.0, emergency_time);
}

inline bool isRollingTrajectoryStartFresh(
    double start_error, double maximum_start_error)
{
  return std::isfinite(start_error) && maximum_start_error >= 0.0 &&
         start_error <= maximum_start_error;
}

inline double speedAwareHandoffErrorLimit(
    double speed, double maximum_deceleration, double reaction_time,
    double minimum_error, double maximum_error)
{
  if (!std::isfinite(speed) || !std::isfinite(maximum_deceleration) ||
      !std::isfinite(reaction_time) || !std::isfinite(minimum_error) ||
      !std::isfinite(maximum_error) || maximum_deceleration <= 0.0 ||
      minimum_error < 0.0 || maximum_error < minimum_error)
    return 0.0;

  const double nonnegative_speed = std::max(0.0, speed);
  const double stopping_distance =
      nonnegative_speed * std::max(0.0, reaction_time) +
      nonnegative_speed * nonnegative_speed /
          (2.0 * maximum_deceleration);
  return std::clamp(
      maximum_error - stopping_distance, minimum_error, maximum_error);
}

inline size_t closestForwardTrajectorySample(
    const std::vector<Eigen::Vector3d> &samples,
    const Eigen::Vector3d &position)
{
  if (samples.empty())
    return 0;

  size_t best = 0;
  double best_distance2 = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < samples.size(); ++index)
  {
    const double distance2 =
        (samples[index].head<2>() - position.head<2>()).squaredNorm();
    // Prefer the later point for an exact tie so a handoff never asks the
    // controller to replay an already completed part of the new trajectory.
    if (distance2 <= best_distance2)
    {
      best_distance2 = distance2;
      best = index;
    }
  }
  return best;
}

inline double normalizePlanarAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

inline double interpolatePlanarYaw(
    double start_yaw, double end_yaw, double ratio)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  return normalizePlanarAngle(
      start_yaw + clamped_ratio * normalizePlanarAngle(end_yaw - start_yaw));
}

inline bool trajectorySamplesAreStationary(
    const std::vector<Eigen::Vector3d> &samples, double tolerance)
{
  if (samples.empty() || tolerance < 0.0)
    return false;
  const double tolerance2 = tolerance * tolerance;
  return std::all_of(
      samples.begin() + 1, samples.end(),
      [&](const Eigen::Vector3d &sample) {
        return (sample - samples.front()).squaredNorm() <= tolerance2;
      });
}

inline bool shouldReuseCurrentTrajectorySuffix(
    bool reference_path_update_pending, bool trajectory_valid,
    double remaining_time, double tracking_error,
    double maximum_tracking_error)
{
  return !reference_path_update_pending && trajectory_valid &&
         remaining_time > 0.0 && std::isfinite(tracking_error) &&
         maximum_tracking_error >= 0.0 &&
         tracking_error <= maximum_tracking_error;
}

inline bool shouldTriggerRollingReplan(
    double planned_progress, double odometry_progress,
    double progress_threshold)
{
  return std::isfinite(planned_progress) &&
         std::isfinite(odometry_progress) &&
         progress_threshold >= 0.0 &&
         planned_progress >= progress_threshold &&
         odometry_progress >= progress_threshold;
}

inline double predictiveHardStopTime(
    double speed, double maximum_deceleration,
    double reaction_time, double minimum_horizon)
{
  if (!std::isfinite(speed) || !std::isfinite(maximum_deceleration) ||
      !std::isfinite(reaction_time) || !std::isfinite(minimum_horizon) ||
      maximum_deceleration <= 0.0)
    return std::numeric_limits<double>::infinity();
  return std::max(std::max(0.0, minimum_horizon),
                  std::max(0.0, reaction_time) +
                      std::max(0.0, speed) / maximum_deceleration);
}

inline bool predictiveCollisionRequiresHardStop(
    double time_to_hit, double speed, double maximum_deceleration,
    double reaction_time, double minimum_horizon)
{
  return !std::isfinite(time_to_hit) || time_to_hit <= predictiveHardStopTime(
      speed, maximum_deceleration, reaction_time, minimum_horizon);
}

inline Eigen::Vector3d projectForwardVelocityToPath(
    const Eigen::Vector3d &velocity,
    const Eigen::Vector3d &path_tangent)
{
  Eigen::Vector3d projected = Eigen::Vector3d::Zero();
  const Eigen::Vector2d tangent_xy = path_tangent.head<2>();
  const double tangent_norm = tangent_xy.norm();
  if (tangent_norm < 1e-6)
    return projected;

  const Eigen::Vector2d unit_tangent = tangent_xy / tangent_norm;
  const double forward_speed =
      std::max(0.0, velocity.head<2>().dot(unit_tangent));
  projected.head<2>() = forward_speed * unit_tangent;
  return projected;
}

inline bool sampledSuffixIsReusable(size_t sample_count, double arc_length)
{
  return sample_count >= 2 && std::isfinite(arc_length) && arc_length > 1e-4;
}

inline bool shouldAcceptTrajectoryVersion(
    uint64_t incoming_request, int64_t incoming_trajectory,
    uint64_t active_request, int64_t active_trajectory)
{
  if (incoming_request < active_request)
    return false;
  if (incoming_request == active_request &&
      incoming_trajectory <= active_trajectory)
    return false;
  return true;
}

inline bool executionCommandTargetsCurrentOrNewer(
    uint64_t command_request, int64_t command_trajectory,
    uint64_t active_request, int64_t active_trajectory)
{
  if (command_request < active_request)
    return false;
  if (command_request == active_request &&
      command_trajectory < active_trajectory)
    return false;
  return true;
}

inline bool trajectorySupersedesExecutionCommand(
    uint64_t trajectory_request, int64_t trajectory_id,
    uint64_t command_request, int64_t command_trajectory)
{
  if (trajectory_request > command_request)
    return true;
  return trajectory_request == command_request &&
         trajectory_id > command_trajectory;
}

inline bool executionStateMatchesExecution(
    uint64_t state_request, int64_t state_trajectory,
    uint64_t executing_request, int64_t executing_trajectory)
{
  return state_request == executing_request &&
         state_trajectory == executing_trajectory;
}

inline bool safetyResultMatchesExecution(
    uint64_t checked_request, int64_t checked_trajectory,
    uint64_t executing_request, int64_t executing_trajectory)
{
  return checked_request == executing_request &&
         checked_trajectory == executing_trajectory;
}

} // namespace scan_planner

#endif // SCAN_PLANNER_REPLAN_FSM_UTILS_H_
