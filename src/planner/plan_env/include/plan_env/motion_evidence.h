#pragma once
#include <Eigen/Core>
#include <array>
#include <map>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace plan_env {
// These are observations, not a claim that a robot can or cannot traverse them.
struct TerrainEvidence {
  Eigen::Vector3d measured{Eigen::Vector3d::Zero()};
  double ground_relative_height{0.};
  double capture_body_z{0.};
  uint32_t scan{0};
};
struct TerrainColumnEvidence {
  TerrainEvidence lowest, highest;
  uint32_t scan{0};
  void observe(const TerrainEvidence& fact) {
    if(scan!=fact.scan) {lowest=highest=fact;scan=fact.scan;return;}
    if(fact.measured.z()<lowest.measured.z()) lowest=fact;
    if(fact.measured.z()>highest.measured.z()) highest=fact;
  }
};
enum class TerrainVerdict { SUPPORT, LEG_TERRAIN, BODY_HEIGHT_RISE, DROP, OVERHEAD };
struct TerrainMotionPolicy {
  double floor_tolerance{.035}, maximum_rise{.22}, maximum_drop{.22}, body_top{.55};
  void validate() const {
    if (!(std::isfinite(floor_tolerance) && std::isfinite(maximum_rise) &&
          std::isfinite(maximum_drop) && std::isfinite(body_top) &&
          floor_tolerance>0 && maximum_rise>floor_tolerance &&
          maximum_drop>floor_tolerance && body_top>maximum_rise))
      throw std::invalid_argument("invalid Local terrain motion capability profile");
  }
  TerrainVerdict assess(const TerrainEvidence& fact) const {
    const double h=fact.ground_relative_height;
    if (std::abs(h)<=floor_tolerance) return TerrainVerdict::SUPPORT;
    if (h<=-maximum_drop) return TerrainVerdict::DROP;
    if (h>=maximum_rise && h<=body_top) return TerrainVerdict::BODY_HEIGHT_RISE;
    if (h>body_top) return TerrainVerdict::OVERHEAD;
    return TerrainVerdict::LEG_TERRAIN;
  }
};
using SourceKey=std::array<int,3>; // Global voxel identity; never a ring address or a hash.
struct OccupancyEvidence {
  enum : uint8_t { MEASURED=1, TERRAIN_CONSTRAINT=2 };
  uint8_t layers{0};
  uint32_t measured_scan{0};
  TerrainEvidence terrain;
};
struct MotionContact {
  SourceKey source;
  uint8_t layers{0};
  double penetration{0.};
  OccupancyEvidence evidence;
};
struct MotionAssessment {
  bool outside_map{false};
  bool unexplained_inflation{false};
  std::map<SourceKey,MotionContact> contacts;
  bool clear() const { return !outside_map && !unexplained_inflation && contacts.empty(); }
};
} // namespace plan_env
