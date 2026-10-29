#pragma once

#include <optional>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <vector>

namespace scan_planner {

// Fit only already measured support heights. No IMU-derived floor, occupancy
// edits or permission to traverse unknown terrain is produced by this query.
inline std::optional<double> measuredSupportSlope(
    const std::vector<Eigen::Vector3d>& points) {
  if(points.size()<6) return std::nullopt;
  Eigen::Vector3d center=Eigen::Vector3d::Zero();
  for(const auto& point:points) {
    if(!point.allFinite()) return std::nullopt;
    center+=point;
  }
  center/=points.size();
  Eigen::Matrix3d system=Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs=Eigen::Vector3d::Zero();
  for(const auto& point:points) {
    const Eigen::Vector3d row(point.x()-center.x(),point.y()-center.y(),1.);
    system+=row*row.transpose();rhs+=row*(point.z()-center.z());
  }
  if(system.determinant()<1e-6) return std::nullopt;
  const Eigen::Vector3d plane=system.ldlt().solve(rhs);
  if(!plane.allFinite()) return std::nullopt;
  double squared_error=0.;
  for(const auto& point:points) {
    const double error=point.z()-center.z()-plane.x()*(point.x()-center.x())-
        plane.y()*(point.y()-center.y())-plane.z();
    squared_error+=error*error;
  }
  if(std::sqrt(squared_error/points.size())>.035) return std::nullopt;
  return std::atan(plane.head<2>().norm());
}

// Goal selection only: the FSM remains the sole trajectory/ACK owner.
struct LocalTaskDomain {
  std::vector<Eigen::Vector2d> route;
  std::vector<Eigen::Vector2d> terminals;
  Eigen::Vector2d observation_center = Eigen::Vector2d::Zero();
  double corridor_radius = 1.0;
  double observation_radius = 2.0;
  bool has_directional_gate = false;
  Eigen::Vector2d gate_origin = Eigen::Vector2d::Zero();
  Eigen::Vector2d gate_direction = Eigen::Vector2d::UnitX();
  double gate_min_progress = 0.0;
};

struct LocalTaskCandidate {
  Eigen::Vector2d goal;
  std::vector<Eigen::Vector2d> approach;
  double cost = 0.0;
  bool observation_station = false;
};

// Certify one specific rotate-then-drive action, not all possible departures.
// The caller compares it with a fixed-yaw staging action. Left/right recovery
// retains its separate physical-family budget and execution contract.
// The callback supplies the same footprint/margin semantics as final validation.
inline bool taskDepartureCertified(const Eigen::Vector2d& start, double measured_yaw,
    double departure_yaw, double stopping_distance,
    const std::function<bool(const Eigen::Vector2d&, double)>& footprint_safe) {
  if (!start.allFinite() || !std::isfinite(measured_yaw) ||
      !std::isfinite(departure_yaw) || !std::isfinite(stopping_distance) ||
      stopping_distance < 0 || stopping_distance > 4.0 || !footprint_safe) return false;
  const double period=2*std::acos(-1.0);
  measured_yaw=std::remainder(measured_yaw,period);
  departure_yaw=std::remainder(departure_yaw,period);
  const double delta=std::remainder(departure_yaw-measured_yaw,period);
  const int turns=std::max(1,static_cast<int>(std::ceil(std::abs(delta)/0.05)));
  for(int i=0;i<=turns;++i)
    if(!footprint_safe(start,measured_yaw+delta*i/turns)) return false;
  const int steps=std::max(1,static_cast<int>(std::ceil(stopping_distance/0.02)));
  const Eigen::Vector2d direction(std::cos(departure_yaw),std::sin(departure_yaw));
  for(int i=0;i<=steps;++i)
    if(!footprint_safe(start+direction*(stopping_distance*i/steps),departure_yaw)) return false;
  return true;
}

// A narrow departure may be able to drive before it can rotate. The finite
// staging positions preserve measured yaw, then certify the required turn
// and its forward braking at the new position. No free point alone qualifies.
inline std::optional<Eigen::Vector2d> taskForwardTurnStage(
    const Eigen::Vector2d& start,double yaw,const Eigen::Vector2d& goal,
    double stopping_distance,
    const std::function<bool(const Eigen::Vector2d&,double)>& safe,
    const std::function<bool(const Eigen::Vector2d&,double)>& turn_safe={}) {
  if(!start.allFinite() || !goal.allFinite() || !std::isfinite(yaw) || !safe)
    return std::nullopt;
  const Eigen::Vector2d forward(std::cos(yaw),std::sin(yaw));
  for(double distance : {.4,.8,1.2}) {
    const Eigen::Vector2d stage=start+distance*forward;
    const Eigen::Vector2d delta=goal-stage;
    if(delta.norm()<.20) continue;
    // Include ability to stop at the end of the incoming straight segment.
    if(!taskDepartureCertified(start,yaw,yaw,distance+stopping_distance,safe)) continue;
    bool can_turn=true;
    // Cover early capture and settling around the planned stopping point.
    // The actual stopped pose will still need a new handoff certificate.
    for(double offset : {-.15,0.,.15}) {
      const Eigen::Vector2d stop=stage+offset*forward;
      const Eigen::Vector2d outgoing=goal-stop;
      if(!taskDepartureCertified(stop,yaw,std::atan2(outgoing.y(),outgoing.x()),
                                 stopping_distance,turn_safe ? turn_safe : safe)) {can_turn=false;break;}
    }
    if(can_turn) return stage;
  }
  return std::nullopt;
}

enum class TaskDepartureKind { DIRECT, FORWARD_STAGE };

struct TaskDeparturePlan {
  TaskDepartureKind kind{TaskDepartureKind::DIRECT};
  Eigen::Vector2d target{Eigen::Vector2d::Zero()};
  Eigen::Vector2d objective{Eigen::Vector2d::Zero()};
  double body_yaw{0.};
};

// Geometry chooses an action before spline generation. A failed direct turn
// does not reject its objective when a certified straight conversion exists.
inline std::optional<TaskDeparturePlan> taskPlanDeparture(
    const Eigen::Vector2d& start,double yaw,const Eigen::Vector2d& objective,
    double stopping_distance,
    const std::function<bool(const Eigen::Vector2d&,double)>& safe,
    const std::function<bool(const Eigen::Vector2d&,double)>& turn_safe={}) {
  if(!start.allFinite() || !objective.allFinite() ||
      (objective-start).norm()<.20 || !std::isfinite(yaw)) return std::nullopt;
  const Eigen::Vector2d direction=objective-start;
  if(taskDepartureCertified(start,yaw,std::atan2(direction.y(),direction.x()),
      stopping_distance,turn_safe ? turn_safe : safe))
    return TaskDeparturePlan{TaskDepartureKind::DIRECT,objective,objective,yaw};
  const auto stage=taskForwardTurnStage(start,yaw,objective,stopping_distance,safe,turn_safe);
  if(stage) return TaskDeparturePlan{TaskDepartureKind::FORWARD_STAGE,*stage,objective,yaw};
  return std::nullopt;
}

struct TaskTerminalExit {
  TaskDepartureKind kind{TaskDepartureKind::DIRECT};
  Eigen::Vector2d waypoint{Eigen::Vector2d::Zero()};
  double departure_yaw{0.};
};

// A station needs one coherent exit over its capture band, not every possible
// future heading. Prefer the supplied route connection; if it needs a turn,
// compare a straight-then-turn conversion before other bounded exits. This is
// geometry evidence only: every actual successor still needs fresh validation.
inline std::optional<TaskTerminalExit> taskTerminalExitPlan(
    const Eigen::Vector2d& position,double yaw,double stopping_distance,
    const std::function<bool(const Eigen::Vector2d&,double)>& safe,
    double preferred_exit_yaw=std::numeric_limits<double>::quiet_NaN()) {
  if(!position.allFinite() || !std::isfinite(yaw) || !safe ||
      !std::isfinite(stopping_distance) || stopping_distance<=0. || stopping_distance>4.)
    return std::nullopt;
  const Eigen::Vector2d forward(std::cos(yaw),std::sin(yaw));
  const auto direct=[&](double heading) {
    for(double offset : {-.15,0.,.15})
      if(!taskDepartureCertified(position+offset*forward,yaw,heading,stopping_distance,safe))
        return false;
    return true;
  };
  if(std::isfinite(preferred_exit_yaw)) {
    if(direct(preferred_exit_yaw))
      return TaskTerminalExit{TaskDepartureKind::DIRECT,position+stopping_distance*
          Eigen::Vector2d(std::cos(preferred_exit_yaw),std::sin(preferred_exit_yaw)),preferred_exit_yaw};
    for(double distance : {.4,.8,1.2}) {
      bool valid=true;
      for(double offset : {-.15,0.,.15}) {
        const Eigen::Vector2d start=position+offset*forward,stage=start+distance*forward;
        if(!taskDepartureCertified(start,yaw,yaw,distance+stopping_distance,safe) ||
            !taskDepartureCertified(stage,yaw,preferred_exit_yaw,stopping_distance,safe)) {
          valid=false;break;
        }
      }
      if(valid) return TaskTerminalExit{TaskDepartureKind::FORWARD_STAGE,
          position+distance*forward,preferred_exit_yaw};
    }
  }
  for(double turn : {0.,M_PI/4,-M_PI/4,M_PI/2,-M_PI/2,3*M_PI/4,-3*M_PI/4,M_PI}) {
    const double heading=yaw+turn;
    if(direct(heading))
      return TaskTerminalExit{TaskDepartureKind::DIRECT,position+stopping_distance*
          Eigen::Vector2d(std::cos(heading),std::sin(heading)),heading};
  }
  return std::nullopt;
}

// Kept as the shared Boolean admission interface; selection, final validation
// and actual arrival now use the same exit semantics instead of forcing yaw+pi.
inline bool taskTerminalRestartCertified(const Eigen::Vector2d& position,double yaw,
    double stopping_distance,
    const std::function<bool(const Eigen::Vector2d&,double)>& safe,
    double preferred_exit_yaw=std::numeric_limits<double>::quiet_NaN()) {
  return taskTerminalExitPlan(position,yaw,stopping_distance,safe,preferred_exit_yaw).has_value();
}

// A dwell objective persists until capture or explicit invalidation. Transit
// endpoints instead extend ahead before capture; rebuilding the same near end
// is not a rolling successor and eventually forces a stop.
inline bool taskKeepLocalTarget(bool station,double distance,double measured_speed,
                                double prepare_time) {
  if(!std::isfinite(distance) || !std::isfinite(measured_speed) ||
      !std::isfinite(prepare_time)) return false;
  return distance>(station ? .20 : std::max(.40,std::max(0.,measured_speed)*
      std::max(0.,prepare_time)+.20));
}

// Preserve transit velocity only at a speed whose complete stopping prefix
// is certified. If only the gait floor fits, the FSM installs matching slow
// controller limits; a geometric short edge never decides this by itself.
inline double taskTransitTerminalSpeed(const Eigen::Vector2d& position,double yaw,
    double maximum_speed,double command_floor,double latency,double deceleration,
    const std::function<bool(const Eigen::Vector2d&,double)>& safe) {
  if(!std::isfinite(maximum_speed) || maximum_speed<=0. ||
      !std::isfinite(command_floor) || command_floor<=0. ||
      !std::isfinite(latency) || latency<0. || !std::isfinite(deceleration) || deceleration<=0.) return 0.;
  for(double speed : {maximum_speed,std::min(maximum_speed,command_floor)}) {
    const double distance=speed*latency+speed*speed/(2*deceleration);
    if(taskDepartureCertified(position,yaw,yaw,distance,safe)) return speed;
  }
  return 0.;
}

// At least one certified exit is required at a stopped task station. Ordinary
// exits rotate then drive; Go2 side exits translate at measured yaw and include
// lateral braking. This does not certify an arbitrary future task or trajectory.
inline uint16_t taskStationExitMask(const Eigen::Vector2d& position, double yaw,
    double forward_distance, double lateral_distance, bool lateral_enabled,
    const std::function<bool(const Eigen::Vector2d&,double)>& safe) {
  if (!position.allFinite() || !std::isfinite(yaw) || !safe ||
      !std::isfinite(forward_distance) || forward_distance<=0 || forward_distance>4 ||
      !std::isfinite(lateral_distance) || lateral_distance<=0 || lateral_distance>4) return 0;
  uint16_t mask=0;
  for(int i=0;i<8;++i)
    if(taskDepartureCertified(position,yaw,yaw+i*std::acos(-1.0)/4,forward_distance,safe))
      mask |= uint16_t(1)<<i;
  if(lateral_enabled) {
    const Eigen::Vector2d left(-std::sin(yaw),std::cos(yaw));
    const int steps=std::max(1,static_cast<int>(std::ceil(lateral_distance/.02)));
    for(int side=0;side<2;++side) {
      bool valid=true;
      for(int i=0;i<=steps && valid;++i)
        valid=safe(position+(side ? -1.0 : 1.0)*left*(lateral_distance*i/steps),yaw);
      if(valid) mask |= uint16_t(1)<<(8+side);
    }
  }
  return mask;
}

// Shared, explicitly certified capture envelope for fixed-yaw side recovery.
// Final progress may fall short by at most 3 cm; this extra sweep permits settling
// compensation during the same bounded primitive, never unplanned reversing.
constexpr double kLateralRecoveryCaptureExtension=.10;
constexpr double kLateralRecoveryLongitudinalBound=.10;

inline double lateralRecoveryTolerance(double length) {
  return std::isfinite(length) && length>=.05 ? std::min(.03,.20*length) : 0.0;
}

// Check the actual final pose, again after settling. A reference clock or a
// FINISHED message alone cannot establish useful physical recovery.
inline bool lateralRecoveryReached(const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal, const Eigen::Vector2d& actual,
    double body_yaw, double actual_yaw) {
  if(!start.allFinite() || !goal.allFinite() || !actual.allFinite() ||
      !std::isfinite(body_yaw) || !std::isfinite(actual_yaw)) return false;
  const double length=(goal-start).norm(), tolerance=lateralRecoveryTolerance(length);
  if(tolerance<=0 ||
      std::abs(std::remainder(actual_yaw-body_yaw,2*std::acos(-1.0)))>.10) return false;
  const Eigen::Vector2d direction=(goal-start)/length;
  const Eigen::Vector2d forward(std::cos(body_yaw),std::sin(body_yaw));
  // Side-step completion is measured transverse progress. The real policy
  // can drift longitudinally; bound that drift by the existing 0.10 m
  // tracking corridor, then Local recertifies its actual stopped pose/rejoin.
  // This is a departure recovery, not an observation station. More progress
  // inside the explicitly certified capture extension is useful; falling
  // short still has the strict 3 cm / 20 percent lower tolerance.
  const double progress=(actual-start).dot(direction);
  return progress<=length+kLateralRecoveryCaptureExtension &&
      std::abs((actual-start).dot(forward))<=kLateralRecoveryLongitudinalBound &&
      progress>=length-tolerance;
}

// One bounded Local search episode survives task/request changes. Search
// retries and activated motion have separate accounting; only related new
// geometry or a certified continuation after escape closes the context.
struct DepartureRecoveryMemory {
  bool used{false};
  Eigen::Vector2d position{Eigen::Vector2d::Zero()};
  double yaw{0};
  uint16_t exits{0};
  // One Local-owned bounded search episode, independent of task/request IDs.
  // `used` counts activated physical motion only; search attempts are separate.
  double started_at{-1.0}, retry_at{0.0}, action_started_at{-1.0}, evidence_checked_at{-1.0};
  unsigned searches{0}, reported_searches{0}, new_exit_confirmations{0};
  bool motion_failed{false};
  uint16_t pending_new_exits{0};
  bool observeExits(uint16_t current_exits) {
    const uint16_t added=current_exits & ~exits;
    if(!added) {pending_new_exits=0;new_exit_confirmations=0;return false;}
    const uint16_t common=pending_new_exits & added;
    pending_new_exits=common ? common : added;
    new_exit_confirmations=common ? new_exit_confirmations+1 : 1;
    return new_exit_confirmations>=3;
  }
  enum class Search { READY, WAIT, EXHAUSTED };
  Search searchState(double now) const {
    if (motion_failed) return Search::EXHAUSTED;
    if (started_at < 0.0) return Search::READY;
    if (!std::isfinite(now) || now < started_at || now-started_at >= 15.0)
      return Search::EXHAUSTED;
    return searches >= 2 ? Search::EXHAUSTED :
        now < retry_at ? Search::WAIT : Search::READY;
  }
  bool beginSearch(double now, const Eigen::Vector2d& actual,
                   double heading, uint16_t current_exits) {
    if (!std::isfinite(now) || now<0.0 || !actual.allFinite() || !std::isfinite(heading) ||
        searchState(now) != Search::READY) return false;
    if (started_at < 0.0) {
      started_at=now; position=actual; yaw=heading; exits=current_exits;
    }
    ++searches; retry_at=now+2.0;
    return true;
  }
  // A pending/active candidate retains its owner; exhausting the search count
  // does not cancel the last search's successful submission.
  bool timedOut(double now, bool executing) const {
    return started_at >= 0.0 && (!std::isfinite(now) || now < started_at ||
        now-started_at >= 15.0 || (executing && action_started_at >= 0.0 &&
        now-action_started_at >= 10.0));
  }
  void activated(double now) { used=true; action_started_at=now; }

};

inline bool taskDeparturePreviouslyRejected(double yaw, const std::vector<double>& rejected) {
  if(!std::isfinite(yaw)) return true;
  for(double previous:rejected)
    if(std::isfinite(previous) &&
       std::abs(std::remainder(yaw-previous,2*std::acos(-1.0))) < std::acos(-1.0)/8)
      return true;
  return false;
}

struct DirectedRoutePortal {
  Eigen::Vector2d center, direction;
  double radius{0};
};

inline bool crossesRoutePortal(const DirectedRoutePortal& portal,
                              const Eigen::Vector2d& before,
                              const Eigen::Vector2d& after) {
  if(!before.allFinite() || !after.allFinite() || !portal.center.allFinite() ||
     !portal.direction.allFinite() || portal.direction.norm()<1e-6 ||
     !std::isfinite(portal.radius) || portal.radius<=0) return false;
  const auto normal=portal.direction.normalized();
  const double a=(before-portal.center).dot(normal),b=(after-portal.center).dot(normal);
  if(a>=0 || b<0 || b-a<1e-6) return false;
  const Eigen::Vector2d crossing=before+(after-before)*(-a/(b-a));
  return (crossing-portal.center).norm()<=portal.radius;
}

// Only measured, continuous odometry can advance the NEXT checkpoint.
// This cursor is execution/topology state, never observed-information progress.
inline size_t advanceRoutePortal(const std::vector<DirectedRoutePortal>& portals,
                                 size_t cursor,const Eigen::Vector2d& before,
                                 const Eigen::Vector2d& after) {
  if(cursor>=portals.size() || (after-before).norm()>0.5) return cursor;
  double last_crossing=-1;
  while(cursor<portals.size() && crossesRoutePortal(portals[cursor],before,after)) {
    const auto normal=portals[cursor].direction.normalized();
    const double fraction=-(before-portals[cursor].center).dot(normal)/(after-before).dot(normal);
    if(fraction<=last_crossing+1e-6) break;
    last_crossing=fraction; ++cursor;
  }
  return cursor;
}

inline bool taskNeedsDwell(const LocalTaskDomain& domain,
                           const Eigen::Vector2d& point, bool visible) {
  // Seeing one boundary point while passing through the large observation
  // disk does not mean that this is the final observation station.
  return visible && (point-domain.observation_center).norm() <=
      std::min(0.8, domain.observation_radius);
}

inline bool taskStationCaptureReady(bool /*autonomous*/, bool heading_aligning,
                                    double measured_station_error) {
  // Task capture must not interrupt an active heading transition. The FSM
  // additionally requires exact terminal feedback, measured rest and an exit.
  return std::isfinite(measured_station_error) && measured_station_error>=0 &&
      measured_station_error<=0.20 && !heading_aligning;
}

inline bool taskRollingDirectionAllowed(const Eigen::Vector2d& displacement,
                                       double yaw, double speed) {
  if (!displacement.allFinite() || !std::isfinite(yaw) || !std::isfinite(speed)) return false;
  if (speed <= 0.05 || displacement.norm() <= 0.01) return true;
  return displacement.normalized().dot(Eigen::Vector2d(std::cos(yaw),std::sin(yaw))) >= std::cos(0.8);
}

inline double taskSegmentDistance(const Eigen::Vector2d& p,
                                 const Eigen::Vector2d& a,
                                 const Eigen::Vector2d& b) {
  const Eigen::Vector2d d = b - a;
  const double t = d.squaredNorm() > 1e-12
      ? std::max(0.0, std::min(1.0, (p-a).dot(d)/d.squaredNorm())) : 0.0;
  return (p-a-t*d).norm();
}

inline double taskRouteProgress(const std::vector<Eigen::Vector2d>& route,
                                const Eigen::Vector2d& p) {
  double best=std::numeric_limits<double>::infinity(), progress=0, offset=0;
  for(size_t i=1;i<route.size();++i) {
    const Eigen::Vector2d d=route[i]-route[i-1];
    const double length=d.norm();
    const double t=length>1e-9 ? std::clamp((p-route[i-1]).dot(d)/(length*length),0.0,1.0) : 0;
    const double distance=(p-route[i-1]-t*d).squaredNorm();
    if(distance<best) {best=distance; progress=offset+t*length;}
    offset+=length;
  }
  return progress;
}

inline std::vector<Eigen::Vector2d> simplifyTaskApproach(
    const std::vector<Eigen::Vector2d>& path,
    const std::function<bool(const Eigen::Vector2d&,const Eigen::Vector2d&)>& safe,
    const std::function<double(const Eigen::Vector2d&,const Eigen::Vector2d&)>& cost = {}) {
  if(path.size()<3) return path;
  std::vector<Eigen::Vector2d> result{path.front()};
  for(size_t i=0;i+1<path.size();) {
    size_t next=i+1;
    for(size_t j=path.size()-1;j>i+1;--j) {
      double original=0;
      if(cost) for(size_t k=i;k<j;++k) original+=cost(path[k],path[k+1]);
      // A hard-safe shortcut must not erase the clearance detour selected by
      // the search. Both costs are integrals over the complete segments.
      if(safe(path[i],path[j]) && (!cost || cost(path[i],path[j])<=original+1e-6))
        {next=j;break;}
    }
    result.push_back(path[next]); i=next;
  }
  return result;
}

inline bool taskSuffixHasUsefulHorizon(double remaining_time) {
  return std::isfinite(remaining_time) && remaining_time > 0.3;
}

inline double taskSeedArcDistance(double length, double start_displacement,
                                 double end_displacement, double u) {
  return start_displacement*u +
      (10*length-6*start_displacement-4*end_displacement)*std::pow(u,3) +
      (-15*length+8*start_displacement+7*end_displacement)*std::pow(u,4) +
      (6*length-3*start_displacement-3*end_displacement)*std::pow(u,5);
}

// Shape statistics describe the optimized curve; they are not safety limits.
// valid checks finite geometry and departure consistency only. Footprint, task
// domain, braking and dynamic feasibility are certified by the caller afterwards.
struct TaskShapeResult {
  bool valid{false};
  const char* reason{"invalid_input"};
  double length{0}, guide_length{0}, turn{0}, guide_turn{0};
  double deviation{0}, regression{0};
};

inline TaskShapeResult taskShapeResult(const std::vector<Eigen::Vector2d>& path,
                                      const std::vector<Eigen::Vector2d>& guide) {
  TaskShapeResult result;
  if(path.size()<2 || guide.size()<2) return result;
  const auto measure=[](const std::vector<Eigen::Vector2d>& points) {
    double length=0,turn=0; Eigen::Vector2d previous=Eigen::Vector2d::Zero();
    for(size_t i=1;i<points.size();++i) {
      const Eigen::Vector2d d=points[i]-points[i-1];
      if(!d.allFinite()) return std::make_pair(std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity());
      length+=d.norm();
      if(d.norm()<0.02) continue;
      if(previous.norm()>0) turn+=std::acos(std::clamp(previous.normalized().dot(d.normalized()),-1.0,1.0));
      previous=d;
    }
    return std::make_pair(length,turn);
  };
  const auto actual=measure(path), expected=measure(guide);
  result.length=actual.first; result.turn=actual.second;
  result.guide_length=expected.first; result.guide_turn=expected.second;
  if(!std::isfinite(actual.first) || !std::isfinite(expected.first)) return result;
  double maximum_progress=taskRouteProgress(guide,path.front());
  for(const auto& point:path) {
    double distance=std::numeric_limits<double>::infinity();
    for(size_t i=1;i<guide.size();++i)
      distance=std::min(distance,taskSegmentDistance(point,guide[i-1],guide[i]));
    const double progress=taskRouteProgress(guide,point);
    result.deviation=std::max(result.deviation,distance);
    result.regression=std::max(result.regression,maximum_progress-progress);
    maximum_progress=std::max(maximum_progress,progress);
  }
  // A short reversed prefix can be invisible to both the 2 cm turn sampling
  // and the clamped route-progress projection. It still makes the controller
  // turn around. Compare to the LOCAL approach, not the full route endpoints.
  Eigen::Vector2d departure=Eigen::Vector2d::Zero();
  for(size_t i=1;i<guide.size();++i) {
    departure=guide[i]-guide.front();
    if(departure.norm()>=0.02) break;
  }
  if(departure.norm()>=0.02) {
    for(size_t i=1;i<path.size();++i) {
      const Eigen::Vector2d delta=path[i]-path.front();
      if(delta.norm()<0.02) continue;
      if(delta.dot(departure)<0) {result.reason="departure_reversal";return result;}
      break;
    }
  }
  result.valid=true; result.reason="ok"; return result;
}

inline bool taskDomainContains(const LocalTaskDomain& domain,
                               const Eigen::Vector2d& p) {
  if (!p.allFinite() || !domain.observation_center.allFinite() ||
      !std::isfinite(domain.corridor_radius) || domain.corridor_radius < 0.0 ||
      !std::isfinite(domain.observation_radius) || domain.observation_radius < 0.0)
    return false;
  if (domain.has_directional_gate &&
      (!domain.gate_origin.allFinite() || !domain.gate_direction.allFinite() ||
       !std::isfinite(domain.gate_min_progress) || domain.gate_direction.norm()<1e-6 ||
       (p-domain.gate_origin).dot(domain.gate_direction.normalized()) < domain.gate_min_progress))
    return false;
  if ((p-domain.observation_center).norm() <= domain.observation_radius)
    return true;
  for (size_t i = 1; i < domain.route.size(); ++i)
    if (domain.route[i-1].allFinite() && domain.route[i].allFinite() &&
        taskSegmentDistance(p, domain.route[i-1], domain.route[i]) <=
            domain.corridor_radius) return true;
  return false;
}

// Bounded connected search, not arbitrary lateral displacement. edge_safe
// must validate the complete edge (footprint + clearance + turning), including
// unknown/map boundaries. A returned approach is NOT an executable trajectory:
// spline, dynamics, braking sweep and Controller ACK are still mandatory.
inline std::vector<LocalTaskCandidate> selectLocalTaskCandidates(
    const LocalTaskDomain& domain, const Eigen::Vector2d& start,
    const Eigen::Vector2d& preferred,
    const std::function<bool(const Eigen::Vector2d&, const Eigen::Vector2d&)>& edge_safe,
    const std::function<bool(const Eigen::Vector2d&)>& visible,
    double step = 0.2, double window = 4.0, size_t max_expansions = 2048,
    size_t max_candidates = 8,
    const std::function<bool(const Eigen::Vector2d&)>& goal_allowed = {},
    double initial_yaw = std::numeric_limits<double>::quiet_NaN(),
    const std::function<double(const Eigen::Vector2d&,const Eigen::Vector2d&)>& edge_cost = {},
    const Eigen::Vector2d* fixed_goal = nullptr,
    const std::function<bool(const Eigen::Vector2d&,double,double)>& turn_safe = {},
    const std::function<bool(const Eigen::Vector2d&,double,bool)>& terminal_safe = {},
    size_t* expansion_count = nullptr) {
  if(expansion_count) *expansion_count=0;
  std::vector<LocalTaskCandidate> result;
  if (!start.allFinite() || !preferred.allFinite() || !edge_safe || !visible ||
      !std::isfinite(step) || step <= 0.0 || !std::isfinite(window) ||
      window <= 0.0 || max_expansions == 0 || max_candidates == 0 ||
      !taskDomainContains(domain, start) || (fixed_goal &&
      (!fixed_goal->allFinite() || !taskDomainContains(domain,*fixed_goal)))) return result;
  struct Node { int x, y, heading; size_t parent; double distance, cost; };
  std::vector<Node> nodes{{0, 0, 8, 0, 0.0, 0.0}};
  using Entry=std::pair<double,size_t>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  open.push({(start-preferred).norm(),0});
  std::map<std::tuple<int,int,int>,double> best{{{0,0,8},0.0}};
  const int moves[8][2]={{1,0},{0,1},{-1,0},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
  // Anchor the departure lattice to measured heading. A world-axis lattice
  // forced even straight departures onto a 45-degree grid near narrow walls.
  const double basis_yaw=std::isfinite(initial_yaw) ? initial_yaw : 0.0;
  const Eigen::Rotation2Dd basis(basis_yaw);
  const auto point = [&](const Node& n) {
    return Eigen::Vector2d(start+basis*Eigen::Vector2d(step*n.x,step*n.y));
  };
  size_t expanded = 0;
  while (!open.empty() && expanded < max_expansions) {
    ++expanded;
    const size_t index = open.top().second; open.pop();
    const Node current = nodes[index];
    if(current.cost>best[{current.x,current.y,current.heading}]+1e-9) continue;
    const Eigen::Vector2d p = point(current);
    const Eigen::Vector2d goal = fixed_goal ? *fixed_goal : p;
    const double incoming=current.heading<8 ?
        basis_yaw+std::atan2(moves[current.heading][1],moves[current.heading][0]) : basis_yaw;
    const double arrival=(goal-p).norm()>1e-6 ? std::atan2(goal.y()-p.y(),goal.x()-p.x()) : incoming;
    const bool connects = !fixed_goal || ((p-goal).norm() <= 1.5*step && edge_safe(p,goal) &&
        (!turn_safe || turn_safe(p,incoming,arrival)));
    const bool station=taskNeedsDwell(domain,goal,visible(goal));
    const double extra = fixed_goal && connects && edge_cost ? edge_cost(p,goal) : 0.0;
    if (index > 0 && connects && std::isfinite(extra) && extra>=0 &&
        (!goal_allowed || goal_allowed(goal)) &&
        (!terminal_safe || terminal_safe(goal,arrival,station))) {
      LocalTaskCandidate candidate;
      candidate.goal = goal;
      candidate.observation_station = station;
      candidate.cost = (goal-preferred).norm() + 0.20*(current.cost+(goal-p).norm()+extra);
      for (size_t j=index;;j=nodes[j].parent) {
        candidate.approach.push_back(point(nodes[j]));
        if (j==0) break;
      }
      std::reverse(candidate.approach.begin(), candidate.approach.end());
      if (fixed_goal && (p-goal).norm()>1e-9) candidate.approach.push_back(goal);
      result.push_back(std::move(candidate));
      std::sort(result.begin(), result.end(), [](const LocalTaskCandidate& a,
                                                const LocalTaskCandidate& b) {
        return a.cost < b.cost;
      });
      if (result.size() > max_candidates) result.pop_back();
    }
    for (int heading=0;heading<8;++heading) {
      const auto& move=moves[heading];
      const int x=current.x+move[0], y=current.y+move[1];
      const Eigen::Vector2d next=start+basis*Eigen::Vector2d(step*x,step*y);
      if ((next-start).norm() > window || !taskDomainContains(domain,next) ||
          !edge_safe(p,next)) continue;
      const double yaw=basis_yaw+std::atan2(move[1],move[0]);
      const double previous_yaw=current.heading<8 ?
          basis_yaw+std::atan2(moves[current.heading][1],moves[current.heading][0]) : initial_yaw;
      const double turn=std::isfinite(previous_yaw) ?
          std::abs(std::remainder(yaw-previous_yaw,2*std::acos(-1.0))) : 0.0;
      if(turn_safe && !turn_safe(p,previous_yaw,yaw)) continue;
      const double extra=edge_cost ? edge_cost(p,next) : 0.0;
      if(!std::isfinite(extra) || extra<0) continue;
      const double cost=current.cost+(next-p).norm()+0.35*turn+extra;
      const auto key=std::make_tuple(x,y,heading);
      const auto old=best.find(key);
      if(old!=best.end() && old->second<=cost+1e-9) continue;
      best[key]=cost;
      nodes.push_back({x,y,heading,index,current.distance+(next-p).norm(),cost});
      open.push({cost+(next-preferred).norm(),nodes.size()-1});
    }
  }
  if(expansion_count) *expansion_count=expanded;
  return result;
}

}  // namespace scan_planner
