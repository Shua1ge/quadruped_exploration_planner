#ifndef SCAN_PLANNER_REPLAN_FSM_UTILS_H_
#define SCAN_PLANNER_REPLAN_FSM_UTILS_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace scan_planner
{

// Shared by calibrated-gait execution and candidate certification. The
// bounded horizon also handles stationary spline tails without a full scan.
template <typename SamplePosition>
inline double forwardGaitLookaheadTime(
    double progress, double duration, SamplePosition sample)
{
  double target = std::clamp(progress, 0.0, duration);
  const double end = std::min(duration, target + 2.0);
  Eigen::Vector3d previous = sample(target);
  double length = 0.0;
  while (target < end && length < 0.15)
  {
    target = std::min(end, target + 0.05);
    const Eigen::Vector3d point = sample(target);
    length += (point - previous).head<2>().norm();
    previous = point;
  }
  return target;
}

inline bool structuredRepairMayStartAnotherEscape(bool rejoin_pending, int committed)
{
  return !rejoin_pending || committed == 0;
}

inline bool normalizeFiniteQuaternion(
    Eigen::Quaterniond *quaternion,
    double minimum_squared_norm = 1e-12,
    double maximum_squared_norm = 1e12)
{
  if (quaternion == nullptr || !quaternion->coeffs().allFinite())
    return false;
  const double squared_norm = quaternion->squaredNorm();
  if (!std::isfinite(squared_norm) ||
      squared_norm < minimum_squared_norm ||
      squared_norm > maximum_squared_norm)
    return false;
  quaternion->normalize();
  return quaternion->coeffs().allFinite();
}

enum class CollisionRiskKind : uint8_t
{
  NONE = 0,
  PREFERRED_MARGIN = 1,
  HARD_MARGIN = 2,
};

struct PredictedCollisionInterval
{
  bool valid{false};
  uint64_t request_id{0};
  int64_t trajectory_id{0};
  uint64_t map_revision{0};
  CollisionRiskKind kind{CollisionRiskKind::NONE};
  double entry_time{0.0};
  double exit_time{0.0};
  double repair_start_time{0.0};
  double rejoin_time{0.0};
  double minimum_clearance{std::numeric_limits<double>::quiet_NaN()};
  Eigen::Vector3d entry_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d exit_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rejoin_position{Eigen::Vector3d::Zero()};
  bool exit_known{false};
  bool braking_window{false};
};

inline PredictedCollisionInterval makePredictedCollisionInterval(
    uint64_t request_id, int64_t trajectory_id, uint64_t map_revision,
    CollisionRiskKind kind, double entry_time, double exit_time,
    double speed, double guaranteed_deceleration, double reaction_time,
    double planning_time, double rejoin_margin, double current_time,
    double trajectory_duration, double confirmed_rejoin_time = -1.0)
{
  PredictedCollisionInterval interval;
  if (request_id == 0 || trajectory_id <= 0 || map_revision == 0 ||
      kind == CollisionRiskKind::NONE || !std::isfinite(entry_time) ||
      !std::isfinite(exit_time) || !std::isfinite(speed) ||
      !std::isfinite(guaranteed_deceleration) ||
      !std::isfinite(reaction_time) || !std::isfinite(planning_time) ||
      !std::isfinite(rejoin_margin) || !std::isfinite(current_time) ||
      !std::isfinite(trajectory_duration) ||
      std::isnan(confirmed_rejoin_time) || current_time < 0.0 ||
      entry_time < current_time || exit_time < entry_time || speed < 0.0 ||
      guaranteed_deceleration <= 0.0 || reaction_time < 0.0 ||
      planning_time < 0.0 || rejoin_margin < 0.0 ||
      trajectory_duration <= 0.0 || entry_time > trajectory_duration)
    return interval;

  const double lead_time = reaction_time + planning_time +
                           speed / guaranteed_deceleration;
  interval.request_id = request_id;
  interval.trajectory_id = trajectory_id;
  interval.map_revision = map_revision;
  interval.kind = kind;
  interval.entry_time = entry_time;
  interval.exit_time = std::min(exit_time, trajectory_duration);
  interval.repair_start_time = std::max(
      current_time, entry_time - lead_time);
  interval.rejoin_time = confirmed_rejoin_time >= 0.0
      ? confirmed_rejoin_time
      : std::min(trajectory_duration,
                 interval.exit_time + rejoin_margin);
  interval.valid = interval.exit_time > interval.entry_time &&
                   interval.repair_start_time < interval.entry_time &&
                   interval.rejoin_time > interval.exit_time &&
                   interval.rejoin_time <= trajectory_duration;
  return interval;
}

inline bool collisionIntervalMatchesExecution(
    const PredictedCollisionInterval &interval,
    uint64_t request_id, int64_t trajectory_id, uint64_t map_revision)
{
  return interval.valid && interval.request_id == request_id &&
         interval.trajectory_id == trajectory_id &&
         interval.map_revision == map_revision;
}

inline bool localPatchSeedIsUsable(
    const PredictedCollisionInterval &interval, double current_time,
    double trajectory_duration, double minimum_prefix,
    double minimum_suffix, double minimum_risk_span,
    uint64_t current_map_revision = 0)
{
  return interval.valid && interval.exit_known && !interval.braking_window &&
         (current_map_revision == 0 ||
          current_map_revision == interval.map_revision) &&
         std::isfinite(current_time) && std::isfinite(trajectory_duration) &&
         std::isfinite(minimum_prefix) && std::isfinite(minimum_suffix) &&
         std::isfinite(minimum_risk_span) && current_time >= 0.0 &&
         trajectory_duration > current_time && minimum_prefix >= 0.0 &&
         minimum_suffix >= 0.0 && minimum_risk_span > 0.0 &&
         interval.repair_start_time >= current_time + minimum_prefix &&
         interval.rejoin_time <= trajectory_duration - minimum_suffix &&
         interval.rejoin_time - interval.repair_start_time >= minimum_risk_span &&
         interval.entry_time >= interval.repair_start_time &&
         interval.exit_time <= interval.rejoin_time &&
         interval.entry_position.allFinite() &&
         interval.exit_position.allFinite() &&
         interval.rejoin_position.allFinite();
}

inline double uniformSeedSampleTime(
    double start_time, double end_time, size_t sample_index,
    size_t sample_count)
{
  if (!std::isfinite(start_time) || !std::isfinite(end_time) ||
      end_time <= start_time || sample_count < 2 ||
      sample_index >= sample_count)
    return std::numeric_limits<double>::quiet_NaN();
  return start_time + (end_time - start_time) *
      static_cast<double>(sample_index) / static_cast<double>(sample_count - 1);
}

inline bool trajectorySeedMatchesBoundaries(
    const std::vector<Eigen::Vector3d> &seed,
    const Eigen::Vector3d &start, const Eigen::Vector3d &end,
    double position_tolerance)
{
  return seed.size() >= 7 && start.allFinite() && end.allFinite() &&
         std::isfinite(position_tolerance) && position_tolerance >= 0.0 &&
         std::all_of(seed.begin(), seed.end(),
                     [](const Eigen::Vector3d &point) {
                       return point.allFinite();
                     }) &&
         (seed.front() - start).norm() <= position_tolerance &&
         (seed.back() - end).norm() <= position_tolerance;
}

struct LocalPatchBoundary
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  double position_tolerance{0.0};
  double velocity_tolerance{0.0};
  double acceleration_tolerance{0.0};
};

inline bool localPatchBoundaryIsContinuous(
    const Eigen::Vector3d &patch_position,
    const Eigen::Vector3d &patch_velocity,
    const Eigen::Vector3d &patch_acceleration,
    const Eigen::Vector3d &reference_position,
    const Eigen::Vector3d &reference_velocity,
    const Eigen::Vector3d &reference_acceleration,
    double position_tolerance, double velocity_tolerance,
    double acceleration_tolerance)
{
  return patch_position.allFinite() && patch_velocity.allFinite() &&
         patch_acceleration.allFinite() && reference_position.allFinite() &&
         reference_velocity.allFinite() && reference_acceleration.allFinite() &&
         std::isfinite(position_tolerance) && position_tolerance >= 0.0 &&
         std::isfinite(velocity_tolerance) && velocity_tolerance >= 0.0 &&
         std::isfinite(acceleration_tolerance) && acceleration_tolerance >= 0.0 &&
         (patch_position - reference_position).norm() <= position_tolerance &&
         (patch_velocity - reference_velocity).norm() <= velocity_tolerance &&
         (patch_acceleration - reference_acceleration).norm() <=
             acceleration_tolerance;
}

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

enum class ClearanceFailure : uint8_t
{
  NONE = 0,
  PHYSICAL_COLLISION = 1,
  PLANNING_MARGIN = 2,
  ESCAPE_WORSENING = 3,
  ESCAPE_TIMEOUT = 4,
};

struct ClearanceResult
{
  bool physical_collision{false};
  bool planning_margin_violation{false};
  bool escape_prefix_allowed{false};
  size_t violation_count{0};
  ClearanceFailure failure{ClearanceFailure::NONE};

  bool accepted() const
  {
    return failure == ClearanceFailure::NONE && !physical_collision &&
           (!planning_margin_violation || escape_prefix_allowed);
  }
};

inline ClearanceResult classifyClearanceResult(
    bool margin_violation, bool physical_collision,
    bool escape_prefix_allowed, bool escape_timed_out,
    bool escape_worsened, size_t violation_count)
{
  ClearanceResult result;
  result.physical_collision = physical_collision;
  result.planning_margin_violation = margin_violation;
  result.escape_prefix_allowed = escape_prefix_allowed;
  result.violation_count = violation_count;
  if (physical_collision)
    result.failure = ClearanceFailure::PHYSICAL_COLLISION;
  else if (escape_timed_out)
    result.failure = ClearanceFailure::ESCAPE_TIMEOUT;
  else if (escape_worsened)
    result.failure = ClearanceFailure::ESCAPE_WORSENING;
  else if (margin_violation && !escape_prefix_allowed)
    result.failure = ClearanceFailure::PLANNING_MARGIN;
  return result;
}

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

inline bool handoffClearanceViolationIsAdmissible(
    bool recovery_primitive, bool started_inside_clearance,
    bool already_exited_clearance, bool physical_collision,
    size_t current_violations, size_t previous_violations)
{
  // Handoff and runtime safety use the same rule: a recovery may start inside
  // the hard planning belt only when every swept pose remains physically free
  // and the number of violated clearance samples never increases.  A pose
  // that started clear receives no exception merely because it belongs to a
  // recovery primitive.
  return recovery_primitive && started_inside_clearance &&
         !already_exited_clearance && !physical_collision &&
         current_violations <= previous_violations;
}

inline bool structuredRepairSafetyInterruptionRequiresRetry(
    bool structured_repair_active, bool structured_repair_executing,
    uint8_t execution_mode, uint8_t recovery_execution_mode)
{
  return structured_repair_active && structured_repair_executing &&
         execution_mode == recovery_execution_mode;
}

inline bool clearanceEscapeConfirmedAtActualPose(
    bool escape_active, bool actual_clearance_blocked,
    bool clearance_improved, double actual_displacement,
    double required_displacement, double matched_progress_ratio,
    double required_progress_ratio, int consecutive_free_cycles,
    int required_free_cycles)
{
  return escape_active && !actual_clearance_blocked &&
         clearance_improved && std::isfinite(actual_displacement) &&
         std::isfinite(required_displacement) &&
         std::isfinite(matched_progress_ratio) &&
         std::isfinite(required_progress_ratio) &&
         actual_displacement >= required_displacement &&
         matched_progress_ratio >= required_progress_ratio &&
         required_free_cycles > 0 &&
         consecutive_free_cycles >= required_free_cycles;
}

inline bool referenceRequestMustBeDeferred(
    bool structured_repair_active, bool structured_repair_executing,
    bool structured_repair_rejoin_pending, bool stopped_repair_pending,
    bool stopped_repair_active, bool structured_handoff_pending)
{
  return structured_repair_active || structured_repair_executing ||
         structured_repair_rejoin_pending || stopped_repair_pending ||
         stopped_repair_active || structured_handoff_pending;
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

inline bool structuredRepairHandoffRetainsOwnership(
    bool repair_segment, bool repair_rejoin, bool ack_accepted)
{
  // An accepted escape segment is still only the first half of recovery.
  // A rejected escape or rejoin also remains owned so the finite retry budget
  // can produce one terminal failure.  Only an accepted rejoin completes it.
  return repair_segment || (repair_rejoin && !ack_accepted);
}

inline bool structuredRepairHandoffCompletesRecovery(
    bool repair_segment, bool repair_rejoin, bool ack_accepted)
{
  return !repair_segment && repair_rejoin && ack_accepted;
}

inline bool structuredRepairSegmentBudgetAvailable(
    int committed_segments, int max_segments)
{
  // This budget belongs to the complete recovery transaction, not to one
  // invocation of the anchor generator.  A successful short segment must not
  // silently buy a fresh family of attempts after its rejoin fails.
  return max_segments > 0 && committed_segments >= 0 &&
         committed_segments < max_segments;
}

inline bool structuredRepairTerminalFeedbackMatches(
    bool repair_executing,
    uint64_t repair_request, int64_t repair_trajectory,
    uint64_t feedback_request, int64_t feedback_trajectory,
    bool feedback_is_terminal)
{
  return repair_executing && feedback_is_terminal &&
         repair_request == feedback_request &&
         repair_trajectory == feedback_trajectory;
}

inline bool ordinaryRollingReplanAllowed(
    bool structured_repair_executing, bool replan_requested)
{
  return !structured_repair_executing && replan_requested;
}

enum class TrackingDeviationAction : uint8_t
{
  KEEP_EXECUTING = 0,
  ROLLING_REPLAN = 1,
  LOCAL_REPAIR = 2,
};

inline TrackingDeviationAction classifyTrackingDeviation(
    bool clearance_limited_execution, bool recovery_corridor_blocked,
    double tracking_error, double degraded_error,
    double recovery_error, double execution_age,
    double execution_grace_period, double no_progress_timeout,
    double odometry_progress, double minimum_progress)
{
  if (!std::isfinite(tracking_error) ||
      !std::isfinite(degraded_error) || degraded_error < 0.0 ||
      !std::isfinite(recovery_error) || recovery_error <= degraded_error ||
      !std::isfinite(execution_age) || execution_age < 0.0 ||
      !std::isfinite(execution_grace_period) || execution_grace_period < 0.0 ||
      !std::isfinite(no_progress_timeout) ||
      no_progress_timeout < execution_grace_period ||
      !std::isfinite(odometry_progress) || odometry_progress < 0.0 ||
      !std::isfinite(minimum_progress) || minimum_progress < 0.0)
    return TrackingDeviationAction::LOCAL_REPAIR;

  if (clearance_limited_execution && execution_age >= no_progress_timeout &&
      odometry_progress < minimum_progress)
    return TrackingDeviationAction::LOCAL_REPAIR;
  if (tracking_error <= degraded_error)
    return TrackingDeviationAction::KEEP_EXECUTING;
  if (recovery_corridor_blocked || tracking_error > recovery_error)
    return TrackingDeviationAction::LOCAL_REPAIR;
  if (!clearance_limited_execution)
    return TrackingDeviationAction::ROLLING_REPLAN;

  // A hard-safe clearance-limited trajectory is an explicitly accepted
  // fallback, not a provisional normal trajectory. Give the controller a
  // bounded interval in which to accelerate and correct moderate error. If
  // the physical robot still has not moved when that interval expires, hand
  // ownership to local recovery instead of publishing an equivalent fallback
  // and resetting the controller's trajectory clock.
  if (execution_age >= no_progress_timeout &&
      odometry_progress < minimum_progress)
    return TrackingDeviationAction::LOCAL_REPAIR;
  if (execution_age < execution_grace_period ||
      odometry_progress < minimum_progress)
    return TrackingDeviationAction::KEEP_EXECUTING;
  return TrackingDeviationAction::ROLLING_REPLAN;
}

inline double calibratedForwardGaitCommand(
    double desired_effective_speed, double minimum_command,
    double measured_effective_speed, double maximum_command)
{
  if (minimum_command <= 0.0 || desired_effective_speed <= 0.0)
    return desired_effective_speed;
  return std::clamp(desired_effective_speed * minimum_command /
                        measured_effective_speed,
                    minimum_command, maximum_command);
}

inline bool holdCommandAlreadyIssuedForVersion(
    uint64_t request, int64_t trajectory,
    uint64_t previous_request, int64_t previous_trajectory)
{
  return request == previous_request && trajectory == previous_trajectory;
}

enum class TimedOutHandoffExecution : uint8_t
{
  UNCONFIRMED,
  CANDIDATE_HELD,
  PREVIOUS_HELD,
};

inline TimedOutHandoffExecution reconcileTimedOutHandoff(
    bool timeout_hold_issued, bool controller_soft_hold,
    bool matching_hold_reason, uint64_t state_request,
    int64_t state_trajectory, uint64_t candidate_request,
    int64_t candidate_trajectory, uint64_t previous_request,
    int64_t previous_trajectory)
{
  if (!timeout_hold_issued || !controller_soft_hold ||
      !matching_hold_reason)
    return TimedOutHandoffExecution::UNCONFIRMED;
  if (state_request == candidate_request &&
      state_trajectory == candidate_trajectory)
    return TimedOutHandoffExecution::CANDIDATE_HELD;
  if (state_request == previous_request &&
      state_trajectory == previous_trajectory)
    return TimedOutHandoffExecution::PREVIOUS_HELD;
  return TimedOutHandoffExecution::UNCONFIRMED;
}

inline bool trajectoryHandoffTimedOut(
    bool pending, double elapsed_seconds, double timeout_seconds)
{
  return pending && std::isfinite(elapsed_seconds) &&
         std::isfinite(timeout_seconds) && timeout_seconds > 0.0 &&
         elapsed_seconds >= timeout_seconds;
}

inline bool trajectoryHandoffReconciliationTimedOut(
    bool pending, bool timeout_hold_issued,
    double elapsed_since_hold_seconds, double timeout_seconds)
{
  return pending && timeout_hold_issued &&
         std::isfinite(elapsed_since_hold_seconds) &&
         std::isfinite(timeout_seconds) && timeout_seconds > 0.0 &&
         elapsed_since_hold_seconds >= timeout_seconds;
}

inline int reserveTrajectoryIdAfterAmbiguousHandoff(
    int current_trajectory_id, int64_t ambiguous_trajectory_id)
{
  const int bounded_ambiguous_id = static_cast<int>(std::clamp<int64_t>(
      ambiguous_trajectory_id,
      static_cast<int64_t>(std::numeric_limits<int>::min()),
      static_cast<int64_t>(std::numeric_limits<int>::max())));
  return std::max(current_trajectory_id, bounded_ambiguous_id);
}

inline bool validBsplineStructure(
    size_t control_point_count, size_t knot_count, int order)
{
  if (order <= 0 || control_point_count <= static_cast<size_t>(order))
    return false;
  const size_t order_size = static_cast<size_t>(order);
  if (control_point_count >
      std::numeric_limits<size_t>::max() - order_size - 1)
    return false;
  return knot_count == control_point_count + order_size + 1;
}

inline bool validBsplineKnots(
    const std::vector<double> &knots, size_t control_point_count, int order)
{
  if (!validBsplineStructure(control_point_count, knots.size(), order))
    return false;
  for (size_t index = 0; index < knots.size(); ++index)
  {
    if (!std::isfinite(knots[index]) ||
        (index > 0 && knots[index] < knots[index - 1]))
      return false;
  }
  return knots[control_point_count] - knots[static_cast<size_t>(order)] >
         1e-9;
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
  if (!std::isfinite(angle))
    return std::numeric_limits<double>::quiet_NaN();
  return std::remainder(angle, 2.0 * M_PI);
}

inline int planarHandoffSweepSampleCount(
    double translation_distance, double yaw_delta,
    double translation_step, double yaw_step)
{
  if (!std::isfinite(translation_distance) || translation_distance < 0.0 ||
      !std::isfinite(yaw_delta) || !std::isfinite(translation_step) ||
      translation_step <= 0.0 || !std::isfinite(yaw_step) || yaw_step <= 0.0)
    return 0;
  const double normalized_delta = normalizePlanarAngle(yaw_delta);
  if (!std::isfinite(normalized_delta))
    return 0;
  const int translation_samples = static_cast<int>(
      std::ceil(translation_distance / translation_step));
  const int yaw_samples = static_cast<int>(
      std::ceil(std::abs(normalized_delta) / yaw_step));
  return std::max(translation_samples, yaw_samples);
}

inline double interpolatePlanarYaw(
    double start_yaw, double end_yaw, double ratio)
{
  if (!std::isfinite(start_yaw) || !std::isfinite(end_yaw) ||
      !std::isfinite(ratio))
    return std::numeric_limits<double>::quiet_NaN();
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
