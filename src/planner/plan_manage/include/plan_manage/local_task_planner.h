#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <queue>
#include <set>
#include <vector>

namespace scan_planner {

// Goal selection only: the FSM remains the sole trajectory/ACK owner.
struct LocalTaskDomain {
  std::vector<Eigen::Vector2d> route;
  std::vector<Eigen::Vector2d> terminals;
  Eigen::Vector2d observation_center = Eigen::Vector2d::Zero();
  double corridor_radius = 1.0;
  double observation_radius = 2.0;
};

struct LocalTaskCandidate {
  Eigen::Vector2d goal;
  std::vector<Eigen::Vector2d> approach;
  double cost = 0.0;
  bool observation_station = false;
};

inline double taskSegmentDistance(const Eigen::Vector2d& p,
                                 const Eigen::Vector2d& a,
                                 const Eigen::Vector2d& b) {
  const Eigen::Vector2d d = b - a;
  const double t = d.squaredNorm() > 1e-12
      ? std::max(0.0, std::min(1.0, (p-a).dot(d)/d.squaredNorm())) : 0.0;
  return (p-a-t*d).norm();
}

inline bool taskDomainContains(const LocalTaskDomain& domain,
                               const Eigen::Vector2d& p) {
  if (!p.allFinite() || !domain.observation_center.allFinite() ||
      !std::isfinite(domain.corridor_radius) || domain.corridor_radius < 0.0 ||
      !std::isfinite(domain.observation_radius) || domain.observation_radius < 0.0)
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
    size_t max_candidates = 8) {
  std::vector<LocalTaskCandidate> result;
  if (!start.allFinite() || !preferred.allFinite() || !edge_safe || !visible ||
      !std::isfinite(step) || step <= 0.0 || !std::isfinite(window) ||
      window <= 0.0 || max_expansions == 0 || max_candidates == 0 ||
      !taskDomainContains(domain, start)) return result;
  struct Node { int x, y; size_t parent; double distance; };
  std::vector<Node> nodes{{0, 0, 0, 0.0}};
  std::queue<size_t> open;
  open.push(0);
  std::set<std::pair<int,int>> visited{{0,0}};
  const int moves[8][2]={{1,0},{0,1},{-1,0},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
  const auto point = [&](const Node& n) {
    return Eigen::Vector2d(start.x()+step*n.x, start.y()+step*n.y);
  };
  size_t expanded = 0;
  while (!open.empty() && expanded++ < max_expansions) {
    const size_t index = open.front(); open.pop();
    const Node current = nodes[index];
    const Eigen::Vector2d p = point(current);
    if (index > 0) {
      const bool station = (p-domain.observation_center).norm() <=
          domain.observation_radius && visible(p);
      LocalTaskCandidate candidate;
      candidate.goal = p;
      candidate.observation_station = station;
      candidate.cost = (p-preferred).norm() + 0.05*current.distance;
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
    for (const auto& move : moves) {
      const int x=current.x+move[0], y=current.y+move[1];
      if (visited.count({x,y})) continue;
      const Eigen::Vector2d next(start.x()+step*x, start.y()+step*y);
      if ((next-start).norm() > window || !taskDomainContains(domain,next) ||
          !edge_safe(p,next)) continue;
      visited.insert({x,y});
      nodes.push_back({x,y,index,current.distance+(next-p).norm()});
      open.push(nodes.size()-1);
    }
  }
  return result;
}

}  // namespace scan_planner
