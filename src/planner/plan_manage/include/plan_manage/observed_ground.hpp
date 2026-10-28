#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <plan_env/ground_height.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <vector>

namespace scan_planner {
// Ground/height estimation. This header does NOT classify or decide anything:
// it reports, for each measured return that has a local ground fit, how high
// the point sits above that fitted ground, and reports "unknown" where no
// ground estimate exists. Whether a height may block the robot is the
// execution layer's decision, and how tall a step the robot can cross is a
// capability statement that does not belong in perception.
//
// The wire contract for the height (0/1/2, 3..252 = height at 1 cm steps,
// 253..255 reserved) lives in plan_env/ground_height.hpp so every consumer
// encodes and decodes the same one byte, and no extra PointCloud2 field or
// topic is needed.

struct ObservedGround {
  bool valid{false};
  Eigen::Vector3d plane{Eigen::Vector3d::Zero()}; // z = ax + by + c in body-centred XY
  size_t support_cells{0};
  // Height above the fitted local ground per input point; NaN = unknown.
  std::vector<float> heights;
};

inline ObservedGround estimateObservedGroundHeights(const std::vector<Eigen::Vector3d>& points,
    const std::vector<uint8_t>& types, const Eigen::Vector3d& body) {
  ObservedGround result;
  result.heights.assign(points.size(),
      std::numeric_limits<float>::quiet_NaN());
  if(points.size()!=types.size() || !body.allFinite()) return result;
  constexpr double cell=.25, residual=.025, max_slope=.35, body_height=.30;
  std::map<std::pair<int,int>,Eigen::Vector3d> lowest;
  for(size_t i=0;i<points.size();++i) {
    if(types[i]!=1 || !points[i].allFinite()) continue;
    const auto p=(points[i]-body).eval();const double radius=p.head<2>().norm();
    if(radius<.55 || radius>8. || p.z()<-.9 || p.z()>.55) continue;
    const auto key=std::make_pair(static_cast<int>(std::floor(p.x()/cell)),
                                  static_cast<int>(std::floor(p.y()/cell)));
    auto it=lowest.find(key);
    if(it==lowest.end() || p.z()<it->second.z()) lowest[key]=p;
  }
  if(lowest.size()<12) return result;
  std::vector<Eigen::Vector3d> samples;for(const auto& item:lowest) samples.push_back(item.second);
  // Deterministic bounded RANSAC; each XY cell has one vote, so a densely
  // sampled wall cannot dominate the sparse floor. Require near-body support.
  std::mt19937 random(7341);size_t best=0;Eigen::Vector3d model=Eigen::Vector3d::Zero();
  for(int trial=0;trial<96;++trial) {
    const auto& a=samples[random()%samples.size()];
    const auto& b=samples[random()%samples.size()];
    const auto& c=samples[random()%samples.size()];
    const Eigen::Vector3d normal=(b-a).cross(c-a);
    if(std::abs(normal.z())<.05) continue;
    Eigen::Vector3d plane(-normal.x()/normal.z(),-normal.y()/normal.z(),0.);
    plane.z()=a.z()-plane.head<2>().dot(a.head<2>());
    // The robot's known stance anchors which surface is its support, rather
    // than accepting a parallel raised step or a remote trench floor.
    if(plane.head<2>().norm()>max_slope || std::abs(plane.z()+body_height)>.06) continue;
    size_t count=0,near=0;
    for(const auto& p:samples) if(std::abs(p.z()-plane.head<2>().dot(p.head<2>())-plane.z())<=residual) {
      ++count;if(p.head<2>().norm()<1.5) ++near;
    }
    if(near>=3 && count>best) {best=count;model=plane;}
  }
  if(best<12) return result;
  std::vector<Eigen::Vector3d> support;
  for(const auto& p:samples) if(std::abs(p.z()-model.head<2>().dot(p.head<2>())-model.z())<=residual)
    support.push_back(p);
  result.valid=true;result.plane=model;
  // Extend the seed only through observed neighbours whose local slope and
  // residual agree. A single global plane flattens undulating mine floors;
  // disconnected raised steps and trench bottoms must not join the support.
  using Key=std::pair<int,int>;
  std::map<Key,std::vector<Eigen::Vector3d>> patches;
  auto insert_support=[&](const Eigen::Vector3d& p) {
    patches[{static_cast<int>(std::floor(p.x())),static_cast<int>(std::floor(p.y()))}].push_back(p);
  };
  for(const auto& p:support) insert_support(p);
  auto local_plane=[&](const Eigen::Vector3d& p,Eigen::Vector3d& plane) {
    const int x=std::floor(p.x()),y=std::floor(p.y());
    Eigen::Matrix3d system=Eigen::Matrix3d::Zero();Eigen::Vector3d rhs=Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> neighbours;
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
      auto found=patches.find({x+dx,y+dy});if(found==patches.end()) continue;
      for(const auto& q:found->second) if((q.head<2>()-p.head<2>()).squaredNorm()<=.8*.8) {
        Eigen::Vector3d row(q.x()-p.x(),q.y()-p.y(),1.);
        system+=row*row.transpose();rhs+=row*q.z();neighbours.push_back(q);
      }
    }
    if(neighbours.size()<3 || system.determinant()<.0003) return false;
    const Eigen::Vector3d fit=system.ldlt().solve(rhs);
    if(!fit.allFinite() || fit.head<2>().norm()>max_slope) return false;
    for(const auto& q:neighbours)
      if(std::abs(q.z()-fit.x()*(q.x()-p.x())-fit.y()*(q.y()-p.y())-fit.z())>.04) return false;
    plane=fit;return true;
  };
  std::vector<uint8_t> accepted(samples.size(),0);
  for(size_t i=0;i<samples.size();++i)
    accepted[i]=std::abs(samples[i].z()-model.head<2>().dot(samples[i].head<2>())-model.z())<=residual;
  for(int pass=0;pass<8;++pass) {
    std::vector<size_t> added;
    for(size_t i=0;i<samples.size();++i) if(!accepted[i]) {
      Eigen::Vector3d plane;
      if(local_plane(samples[i],plane) && std::abs(samples[i].z()-plane.z())<=.035) added.push_back(i);
    }
    if(added.empty()) break;
    for(size_t i:added) {accepted[i]=1;support.push_back(samples[i]);insert_support(samples[i]);}
  }
  result.support_cells=support.size();
  // Require neighbouring support over a two-dimensional patch. A few
  // collinear rays, wall bases and isolated rocks never become a ground plane.
  std::map<std::pair<int,int>,std::vector<Eigen::Vector2d>> bins;
  for(const auto& p:support) bins[{static_cast<int>(std::floor(p.x())),static_cast<int>(std::floor(p.y()))}].push_back(p.head<2>());
  for(size_t i=0;i<points.size();++i) {
    if(types[i]!=1 || !points[i].allFinite()) continue;
    const auto p=(points[i]-body).eval();if(p.head<2>().norm()<.55 || p.head<2>().norm()>8.) continue;
    const int x=std::floor(p.x()),y=std::floor(p.y());
    size_t count=0;Eigen::Vector2d sum=Eigen::Vector2d::Zero();Eigen::Matrix2d moment=Eigen::Matrix2d::Zero();
    for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
      auto found=bins.find({x+dx,y+dy});if(found==bins.end()) continue;
      for(const auto& q:found->second) if((q-p.head<2>()).squaredNorm()<=.8*.8) {
        ++count;sum+=q;moment+=q*q.transpose();
      }
    }
    if(count<3) continue;
    const Eigen::Matrix2d covariance=moment/static_cast<double>(count)-(sum/count)*(sum/count).transpose();
    if(covariance.determinant()<.00003) continue;
    Eigen::Vector3d plane;
    if(!local_plane(p,plane)) continue;
    // Height above the fitted ground at this point's own XY. No verdict:
    // what counts as floor, leg-height terrain, a body-clearance obstruction
    // or an overhead structure is the consumer's business.
    result.heights[i]=static_cast<float>(p.z()-plane.z());
  }
  return result;
}
} // namespace scan_planner
