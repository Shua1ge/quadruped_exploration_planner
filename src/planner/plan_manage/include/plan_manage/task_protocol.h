#pragma once
#include <cstdint>
#include <utility>

namespace scan_planner {
using TaskVersion = std::pair<std::uint64_t, std::uint64_t>;

// A planning request may change while the acknowledged predecessor still
// executes. Only its explicit task/version lineage authorizes continuation.
inline bool planningOwnsExecution(TaskVersion execution, std::uint64_t execution_request,
    TaskVersion planning, std::uint64_t planning_request,
    TaskVersion predecessor, bool transitioning) {
  if (!execution.first || !execution.second || !execution_request) return false;
  if (execution == planning) return execution_request == planning_request;
  return transitioning && predecessor.first && predecessor.second && execution == predecessor;
}

inline bool installedAlignmentMayFinish(bool owned, bool aligning, bool stalled,
    bool safety_stopped, bool feedback_fresh, bool map_fresh) {
  return owned && aligning && !stalled && !safety_stopped && feedback_fresh && map_fresh;
}

// task_id is a monotonic generation owned by Explorer. (0,0) is legacy mode.
inline bool taskCandidateIsLive(TaskVersion candidate, TaskVersion retired) {
  if (candidate.first == 0 || candidate.second == 0)
    return candidate.first == 0 && candidate.second == 0;
  return candidate > retired;
}

inline bool taskExecutionIsRetired(TaskVersion active, TaskVersion retired) {
  return active.first != 0 && active <= retired;
}
}  // namespace scan_planner
