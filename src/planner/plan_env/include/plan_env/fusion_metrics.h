#ifndef PLAN_ENV_FUSION_METRICS_H
#define PLAN_ENV_FUSION_METRICS_H

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <string>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

namespace plan_env {

// Measurement-only record for one GridMap fusion transaction.
// cls: 0 = empty callback (no pending update), 1 = empty raycast (no points),
//      2 = effective fusion (projected_points > 0).
struct FusionRecord
{
  int cls{0};
  int64_t start_steady_ns{0};
  int64_t tid{0};
  int64_t wall_ns{0};
  int64_t cpu_ns{0};
  static constexpr int64_t kUnavailable = std::numeric_limits<int64_t>::min();
  int64_t runnable_ns{kUnavailable};
  int64_t residual_ns{kUnavailable};
  int64_t prelock_ns{0};
  int64_t lock_wait_ns{0};
  int64_t raycast_ns{0};
  int64_t fusion_other_ns{0};
  int64_t snapshot_build_ns{0};
  int64_t snapshot_publish_ns{0};
  int64_t visualization_ns{0};
  int64_t callback_total_ns{0};
  int points{0};
  int64_t raycast_cpu_ns{0};
  int64_t fusion_other_cpu_ns{0};
  int64_t sensor_stamp_ns{0};
  uint64_t map_revision{0};
  uint64_t snapshot_revision{0};
};

inline int64_t steady_ns_now()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline int64_t thread_cpu_ns_now()
{
  timespec ts{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

inline int64_t current_tid()
{
  return static_cast<int64_t>(::syscall(SYS_gettid));
}

// Second field of /proc/self/task/<tid>/schedstat: cumulative run-queue wait.
// Returns -1 when the kernel does not expose it.
inline int64_t schedstat_runnable_ns()
{
  char path[64];
  std::snprintf(path, sizeof(path), "/proc/self/task/%ld/schedstat",
                static_cast<long>(current_tid()));
  FILE *f = std::fopen(path, "r");
  if (!f) return -1;
  unsigned long long cpu = 0, wait = 0, slices = 0;
  const int n = std::fscanf(f, "%llu %llu %llu", &cpu, &wait, &slices);
  std::fclose(f);
  return n >= 2 ? static_cast<int64_t>(wait) : -1;
}

class FusionMetrics
{
public:
  FusionMetrics(bool enabled, const std::string &path) : enabled_(enabled)
  {
    if (!enabled_) return;
    const std::string file = path.empty() ? "/tmp/grid_map_fusion_metrics.csv" : path;
    out_.open(file, std::ios::out | std::ios::trunc);
    if (out_)
    {
      out_ << "start_steady_ns,cls,tid,wall_ms,cpu_ms,runnable_ms,residual_ms,"
              "prelock_ms,lock_wait_ms,raycast_process_ms,fusion_other_ms,"
              "snapshot_build_ms,snapshot_publish_ms,visualization_ms,"
              "callback_total_ms,points,raycast_cpu_ms,fusion_other_cpu_ms,sensor_stamp_ns,map_revision,snapshot_revision\n";
      out_.flush();
    }
  }

  bool enabled() const { return enabled_ && out_.is_open(); }

  void record(const FusionRecord &r)
  {
    if (!enabled()) return;
    std::lock_guard<std::mutex> lock(mu_);
    out_ << r.start_steady_ns << ',' << r.cls << ',' << r.tid << ','
         << ns_to_ms(r.wall_ns) << ',' << ns_to_ms(r.cpu_ns) << ','
         << ns_to_ms(r.runnable_ns) << ',' << ns_to_ms(r.residual_ns) << ','
         << ns_to_ms(r.prelock_ns) << ',' << ns_to_ms(r.lock_wait_ns) << ','
         << ns_to_ms(r.raycast_ns) << ',' << ns_to_ms(r.fusion_other_ns) << ','
         << ns_to_ms(r.snapshot_build_ns) << ',' << ns_to_ms(r.snapshot_publish_ns) << ','
         << ns_to_ms(r.visualization_ns) << ',' << ns_to_ms(r.callback_total_ns) << ','
         << r.points << ',' << ns_to_ms(r.raycast_cpu_ns) << ',' << ns_to_ms(r.fusion_other_cpu_ns) << ','
         << r.sensor_stamp_ns << ',' << r.map_revision << ',' << r.snapshot_revision << '\n';
    if (++count_ % 100 == 0) out_.flush();
  }

  ~FusionMetrics()
  {
    if (out_.is_open()) out_.flush();
  }

private:
  static double ns_to_ms(int64_t v)
  {
    if (v == FusionRecord::kUnavailable) return std::numeric_limits<double>::quiet_NaN();
    return static_cast<double>(v) / 1e6;
  }
  bool enabled_{false};
  std::ofstream out_;
  std::mutex mu_;
  int count_{0};
};

}  // namespace plan_env

#endif
