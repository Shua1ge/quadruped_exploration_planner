#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace plan_env {
// Wire contract for the measured "height above the local ground" that travels
// in the PointCloud2 `return_type` byte. The estimator lives in plan_manage
// (observed_ground.hpp); this header is the shared single source of truth for
// the encoding so plan_env and plan_manage cannot drift apart.
//
//   0        invalid ray
//   1        measured return, height NOT estimated (unknown)
//   2        measured no-return (endpoint placed at the no-return range)
//   3..252   measured return, height = kHeightBase + (value-3)*kHeightStep
//            (clamped to [-0.50, +1.99] m, 1 cm resolution)
//   253..255 reserved; consumers treat them as unknown
constexpr uint8_t kGroundUnknown = 1;
constexpr uint8_t kGroundNoReturn = 2;
constexpr uint8_t kHeightFirst = 3;
constexpr uint8_t kHeightLast = 252;
constexpr double kHeightBase = -0.50;
constexpr double kHeightStep = 0.01;

inline uint8_t encodeGroundHeight(double height) {
  if(!std::isfinite(height)) return kGroundUnknown;
  const double clamped = std::clamp(height, kHeightBase,
      kHeightBase + (kHeightLast - kHeightFirst) * kHeightStep);
  return static_cast<uint8_t>(kHeightFirst +
      std::lround((clamped - kHeightBase) / kHeightStep));
}
inline double decodeGroundHeight(uint8_t value) {
  if(value < kHeightFirst || value > kHeightLast)
    return std::numeric_limits<double>::quiet_NaN();
  return kHeightBase + (value - kHeightFirst) * kHeightStep;
}
} // namespace plan_env
