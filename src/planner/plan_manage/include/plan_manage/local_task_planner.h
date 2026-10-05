#pragma once

#include <Eigen/Core>
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

inline bool taskNeedsDwell(const LocalTaskDomain& domain,
                           const Eigen::Vector2d& point, bool visible) {
  // Seeing one boundary point while passing through the large observation
  // disk does not mean that this is the final observation station.
  return visible && (point-domain.observation_center).norm() <=
      std::min(0.8, domain.observation_radius);
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
    const std::function<bool(const Eigen::Vector2d&,const Eigen::Vector2d&)>& safe) {
  if(path.size()<3) return path;
  std::vector<Eigen::Vector2d> result{path.front()};
  for(size_t i=0;i+1<path.size();) {
    size_t next=i+1;
    for(size_t j=path.size()-1;j>i+1;--j)
      if(safe(path[i],path[j])) {next=j;break;}
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
  if(actual.first>1.5*expected.first+0.2) {result.reason="length"; return result;}
  if(actual.second>expected.second+0.8) {result.reason="turn"; return result;}
  double maximum_progress=taskRouteProgress(guide,path.front());
  for(const auto& point:path) {
    double distance=std::numeric_limits<double>::infinity();
    for(size_t i=1;i<guide.size();++i)
      distance=std::min(distance,taskSegmentDistance(point,guide[i-1],guide[i]));
    const double progress=taskRouteProgress(guide,point);
    result.deviation=std::max(result.deviation,distance);
    result.regression=std::max(result.regression,maximum_progress-progress);
    if(distance>0.4) {result.reason="guide_deviation"; return result;}
    if(progress<maximum_progress-0.15) {result.reason="progress_regression"; return result;}
    maximum_progress=std::max(maximum_progress,progress);
  }
  result.valid=true; result.reason="ok"; return result;
}

inline bool taskShapeFollowsGuide(const std::vector<Eigen::Vector2d>& path,
                                 const std::vector<Eigen::Vector2d>& guide) {
  return taskShapeResult(path,guide).valid;
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
    const std::function<double(const Eigen::Vector2d&,const Eigen::Vector2d&)>& edge_cost = {}) {
  std::vector<LocalTaskCandidate> result;
  if (!start.allFinite() || !preferred.allFinite() || !edge_safe || !visible ||
      !std::isfinite(step) || step <= 0.0 || !std::isfinite(window) ||
      window <= 0.0 || max_expansions == 0 || max_candidates == 0 ||
      !taskDomainContains(domain, start)) return result;
  struct Node { int x, y, heading; size_t parent; double distance, cost; };
  std::vector<Node> nodes{{0, 0, 8, 0, 0.0, 0.0}};
  using Entry=std::pair<double,size_t>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  open.push({(start-preferred).norm(),0});
  std::map<std::tuple<int,int,int>,double> best{{{0,0,8},0.0}};
  const int moves[8][2]={{1,0},{0,1},{-1,0},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
  const auto point = [&](const Node& n) {
    return Eigen::Vector2d(start.x()+step*n.x, start.y()+step*n.y);
  };
  size_t expanded = 0;
  while (!open.empty() && expanded++ < max_expansions) {
    const size_t index = open.top().second; open.pop();
    const Node current = nodes[index];
    if(current.cost>best[{current.x,current.y,current.heading}]+1e-9) continue;
    const Eigen::Vector2d p = point(current);
    if (index > 0 && (!goal_allowed || goal_allowed(p))) {
      const bool station = taskNeedsDwell(domain, p, visible(p));
      LocalTaskCandidate candidate;
      candidate.goal = p;
      candidate.observation_station = station;
      candidate.cost = (p-preferred).norm() + 0.20*current.cost;
      for (size_t j=index;;j=nodes[j].parent) {
        candidate.approach.push_back(point(nodes[j]));
        if (j==0) break;
      }
      std::reverse(candidate.approach.begin(), candidate.approach.end());
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
      const Eigen::Vector2d next(start.x()+step*x, start.y()+step*y);
      if ((next-start).norm() > window || !taskDomainContains(domain,next) ||
          !edge_safe(p,next)) continue;
      const double yaw=std::atan2(move[1],move[0]);
      const double previous_yaw=current.heading<8 ?
          std::atan2(moves[current.heading][1],moves[current.heading][0]) : initial_yaw;
      const double turn=std::isfinite(previous_yaw) ?
          std::abs(std::remainder(yaw-previous_yaw,2*std::acos(-1.0))) : 0.0;
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
  return result;
}

}  // namespace scan_planner
