#pragma once
#include <cstdint>
#include <utility>

namespace scan_planner {
using TaskVersion = std::pair<std::uint64_t, std::uint64_t>;

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
