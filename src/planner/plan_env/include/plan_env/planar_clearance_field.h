#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <vector>

namespace scan_planner {

// Immutable, planar cell-distance field. This is a path-quality cost, not a
// replacement for footprint, swept-motion or unknown-space certification.
class PlanarClearanceField {
 public:
  PlanarClearanceField(int width, int height, double resolution,
                       Eigen::Vector2d origin,
                       const std::function<bool(int,int)>& free)
      : width_(width), height_(height), resolution_(resolution), origin_(origin),
        distances_(static_cast<size_t>(std::max(0,width))*std::max(0,height), 0.0) {
    if(width<2 || height<2 || !std::isfinite(resolution) || resolution<=0 ||
       !origin.allFinite() || !free) throw std::invalid_argument("invalid planar clearance grid");
    std::vector<double> row(width), column(height), temporary(width*height);
    for (int y=0;y<height;++y) {
      for (int x=0;x<width;++x) row[x]=free(x,y) ? 1e12 : 0.0;
      const auto transformed=transform(row);
      for(int x=0;x<width;++x) temporary[y*width+x]=transformed[x];
    }
    for(int x=0;x<width;++x) {
      for(int y=0;y<height;++y) column[y]=temporary[y*width+x];
      const auto transformed=transform(column);
      for(int y=0;y<height;++y)
        distances_[y*width+x]=std::max(0.0,std::sqrt(transformed[y])*resolution-resolution/std::sqrt(2.0));
    }
  }

  double distance(const Eigen::Vector2d& point) const {
    if(!point.allFinite() || width_<2 || height_<2) return 0;
    const Eigen::Vector2d cell=(point-origin_)/resolution_-Eigen::Vector2d::Constant(.5);
    if(cell.x()<0 || cell.y()<0 || cell.x()>=width_-1 || cell.y()>=height_-1) return 0;
    const int x=static_cast<int>(std::floor(cell.x())),y=static_cast<int>(std::floor(cell.y()));
    const double u=cell.x()-x,v=cell.y()-y;
    return (1-v)*((1-u)*distances_[y*width_+x]+u*distances_[y*width_+x+1])+
           v*((1-u)*distances_[(y+1)*width_+x]+u*distances_[(y+1)*width_+x+1]);
  }

  double footprintDistance(const Eigen::Vector2d& point,double yaw,double offset) const {
    const Eigen::Vector2d heading(std::cos(yaw),std::sin(yaw));
    return std::min(distance(point+offset*heading),distance(point-offset*heading));
  }

  static double penalty(double clearance,double preferred) {
    const double deficit=std::max(0.0,preferred-clearance);
    return deficit*deficit;
  }

  static double preferenceCost(double clearance,double preferred) {
    if(preferred<=0) return 0;
    // Do not become flat just outside the preferred belt: maximizing nearest
    // obstacle distance also prefers the middle of a corridor. The bounded
    // tail stays finite at the wall and cannot override hard feasibility.
    const double ratio=preferred/(preferred+std::max(0.0,clearance));
    return penalty(clearance,preferred)+0.25*preferred*preferred*ratio*ratio;
  }

 private:
  // Lower envelope of squared parabolas: two separable O(n) passes.
  static std::vector<double> transform(const std::vector<double>& input) {
    const int n=static_cast<int>(input.size());
    std::vector<double> result(n),breaks(n+1);
    std::vector<int> sites(n);
    int last=0; sites[0]=0; breaks[0]=-1e30; breaks[1]=1e30;
    for(int q=1;q<n;++q) {
      double crossing;
      do {
        const int p=sites[last];
        crossing=((input[q]+double(q)*q)-(input[p]+double(p)*p))/(2.0*(q-p));
        if(crossing>breaks[last]) break;
        --last;
      } while(last>=0);
      ++last; sites[last]=q; breaks[last]=crossing; breaks[last+1]=1e30;
    }
    last=0;
    for(int q=0;q<n;++q) {
      while(breaks[last+1]<q) ++last;
      const double difference=q-sites[last];
      result[q]=difference*difference+input[sites[last]];
    }
    return result;
  }
  int width_,height_;
  double resolution_;
  Eigen::Vector2d origin_;
  std::vector<double> distances_;
};
} // namespace scan_planner
