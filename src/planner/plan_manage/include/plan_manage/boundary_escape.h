#pragma once
#include <plan_env/motion_evidence.h>
#include <algorithm>
#include <string>

namespace scan_planner {
// An escape is a specific fixed-yaw action, not permission to ignore a map.
struct BoundaryEscapeCertificate {
  Eigen::Vector3d start{Eigen::Vector3d::Zero()}, goal{Eigen::Vector3d::Zero()};
  double yaw{0.}, expires{0.};
  double cross_bound{.10}, z_bound{.025}, yaw_bound{.10}, extension{.10};
  uint64_t map_revision{0};
  std::map<plan_env::SourceKey,double> initial_contacts;
};
inline bool advanceEscapeContacts(std::map<plan_env::SourceKey,double>& remaining,
    const plan_env::MotionAssessment& assessment,double tolerance=.0) {
  if(assessment.outside_map || assessment.unexplained_inflation) return false;
  for(const auto& contact:assessment.contacts) {
    const auto old=remaining.find(contact.first);
    if(old==remaining.end() || !std::isfinite(contact.second.penetration) ||
        contact.second.penetration>old->second+tolerance) return false;
    // A newly derived terrain prohibition is not covered by a physical source
    // certificate, even when it happens to share the same global voxel.
    if(contact.second.layers & plan_env::OccupancyEvidence::TERRAIN_CONSTRAINT) return false;
  }
  for(auto it=remaining.begin();it!=remaining.end();) {
    const auto now=assessment.contacts.find(it->first);
    if(now==assessment.contacts.end()) it=remaining.erase(it);
    else {it->second=std::min(it->second,now->second.penetration);++it;}
  }
  return true;
}
inline bool boundaryEscapePoseAllowed(const BoundaryEscapeCertificate& c,
    const Eigen::Vector3d& p,double yaw,double now) {
  const Eigen::Vector2d delta=c.goal.head<2>()-c.start.head<2>();
  if(!p.allFinite() || !std::isfinite(yaw) || !std::isfinite(now) ||
      !std::isfinite(c.expires) || now>c.expires || delta.norm()<.1) return false;
  const auto direction=delta.normalized();
  const Eigen::Vector2d displacement=p.head<2>()-c.start.head<2>();
  const double progress=displacement.dot(direction);
  return progress>=-.02 && progress<=delta.norm()+c.extension &&
      std::abs(displacement.dot(Eigen::Vector2d(-direction.y(),direction.x())))<=c.cross_bound &&
      std::abs(p.z()-c.start.z())<=c.z_bound &&
      std::abs(std::remainder(yaw-c.yaw,2*std::acos(-1.)))<=c.yaw_bound;
}
inline double lateralCaptureStopLead(double along_speed,double deceleration,double latency) {
  if(!std::isfinite(along_speed) || !std::isfinite(deceleration) ||
      !std::isfinite(latency) || deceleration<=0 || latency<0) return 0.;
  const double v=std::max(0.,along_speed);
  return v*latency+v*v/(2*deceleration);
}
} // namespace scan_planner
