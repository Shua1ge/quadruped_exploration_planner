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

inline bool sampledSuffixIsReusable(size_t sample_count, double arc_length)
{
  return sample_count >= 2 && std::isfinite(arc_length) && arc_length > 1e-4;
}

} // namespace scan_planner

#endif // SCAN_PLANNER_REPLAN_FSM_UTILS_H_
