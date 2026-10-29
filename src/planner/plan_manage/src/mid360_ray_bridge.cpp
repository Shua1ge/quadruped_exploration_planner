#include "plan_manage/mid360_rays.hpp"
#include "plan_manage/observed_ground.hpp"
#include <plan_env/grid_map.h>
#include <nav_msgs/msg/odometry.hpp>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <deque>
#include <limits>
#include <mutex>

namespace scan_planner {
class Mid360RayBridge : public rclcpp::Node {
public:
  Mid360RayBridge() : rclcpp::Node("mid360_ray_bridge") {
    no_return_range_=declare_parameter("no_return_range",3.0);
    topic_=declare_parameter<std::string>("gazebo_topic","/quad_0/lidar/points");
    ground_enabled_=declare_parameter("classify_observed_ground",false);
    const double slope_degrees=declare_parameter("observed_ground_max_slope_degrees",30.0);
    if(!std::isfinite(slope_degrees) || slope_degrees<=0. || slope_degrees>=60.)
      throw std::invalid_argument("invalid observed ground measurement angle");
    ground_options_.max_slope_radians=slope_degrees*std::acos(-1.0)/180.;
    mount_={declare_parameter("lidar_extrinsic_x",.187),declare_parameter("lidar_extrinsic_y",0.),declare_parameter("lidar_extrinsic_z",.0803)};
    const double roll=declare_parameter("lidar_extrinsic_roll",0.);
    const double pitch=declare_parameter("lidar_extrinsic_pitch",.22689280275926285);
    const double yaw=declare_parameter("lidar_extrinsic_yaw",0.);
    rotation_=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())).toRotationMatrix();
    if(!std::isfinite(no_return_range_) || no_return_range_<=0.) throw std::invalid_argument("invalid no-return range");
    publisher_=create_publisher<sensor_msgs::msg::PointCloud2>("/quad_0/lidar_rays",rclcpp::SensorDataQoS());
    if(ground_enabled_) {
      pose_sub_=create_subscription<nav_msgs::msg::Odometry>("body_pose",rclcpp::SensorDataQoS(),
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            const auto& p=msg->pose.pose.position;const auto& q=msg->pose.pose.orientation;
            plan_env::TimedPose pose;pose.stamp_ns=rclcpp::Time(msg->header.stamp).nanoseconds();
            pose.position={p.x,p.y,p.z};pose.orientation=Eigen::Quaterniond(q.w,q.x,q.y,q.z);
            if(pose.stamp_ns<=0 || !pose.position.allFinite() || !pose.orientation.coeffs().allFinite() || pose.orientation.norm()<1e-6) return;
            pose.orientation.normalize();std::lock_guard<std::mutex> lock(mutex_);
            if(!poses_.empty() && pose.stamp_ns<poses_.back().stamp_ns) {poses_.clear();pending_.clear();}
            if(poses_.empty() || pose.stamp_ns>poses_.back().stamp_ns) poses_.push_back(pose);
            while(poses_.size()>200) poses_.pop_front();
          });
      timer_=create_wall_timer(std::chrono::milliseconds(10),[this](){processPending();});
    }
    if(!transport_.Subscribe(topic_,&Mid360RayBridge::onScan,this)) throw std::runtime_error("cannot subscribe to 3-D scan");
    RCLCPP_INFO(get_logger(),"[MID360_RAYS] %s -> /quad_0/lidar_rays observed_ground=%d; physical XYZ retained",topic_.c_str(),ground_enabled_);
  }
  ~Mid360RayBridge() override {transport_.Unsubscribe(topic_);}
private:
  void onScan(const ignition::msgs::LaserScan& scan) {
    try {
      auto cloud=mid360RayCloud(scan,no_return_range_);
      if(!ground_enabled_) {publisher_->publish(cloud);return;}
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.push_back({std::move(cloud),std::chrono::steady_clock::now()});
      while(pending_.size()>3) pending_.pop_front();
    } catch(const std::invalid_argument& error) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"[MID360_RAYS] dropping scan: %s",error.what());
    }
  }
  void processPending() {
    sensor_msgs::msg::PointCloud2 cloud;plan_env::TimedPose body;
    {
      std::lock_guard<std::mutex> lock(mutex_);if(pending_.empty()) return;
      auto& next=pending_.front();
      const auto status=plan_env::lookupTimedPose(poses_,rclcpp::Time(next.first.header.stamp).nanoseconds(),50000000,20000000,body);
      if(status!=plan_env::PoseLookupResult::EXACT && status!=plan_env::PoseLookupResult::INTERPOLATED && status!=plan_env::PoseLookupResult::NEAREST) {
        if(std::chrono::steady_clock::now()-next.second>std::chrono::milliseconds(150)) {
          pending_.pop_front();RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"[GROUND_POSE_WAIT] dropping unmatched scan");
        }
        return;
      }
      cloud=std::move(next.first);pending_.pop_front();
    }
    const auto start=std::chrono::steady_clock::now();
    const Eigen::Vector3d origin=body.position+body.orientation*mount_;
    const Eigen::Matrix3d rotation=body.orientation.toRotationMatrix()*rotation_;
    std::vector<Eigen::Vector3d> points;std::vector<uint8_t> kinds;
    points.reserve(cloud.width*cloud.height);kinds.reserve(cloud.width*cloud.height);
    for(size_t i=0;i<cloud.width*cloud.height;++i) {
      float xyz[3];std::memcpy(xyz,cloud.data.data()+16*i,sizeof(xyz));
      points.push_back(rotation*Eigen::Vector3d(xyz[0],xyz[1],xyz[2])+origin);kinds.push_back(cloud.data[16*i+12]);
    }
    const int64_t capture_ns=rclcpp::Time(cloud.header.stamp).nanoseconds();
    const bool recent=last_ground_.valid && capture_ns>=last_ground_stamp_ &&
        capture_ns-last_ground_stamp_<=500000000LL;
    const auto ground=estimateObservedGroundHeights(points,kinds,body.position,
        ground_options_,recent ? &last_ground_ : nullptr);
    if(ground.valid) {
      // Retain only the tiny fitted reference, never the previous point cloud
      // or per-point heights. Invalid fresh input is still published unknown.
      last_ground_.valid=true;last_ground_.plane=ground.plane;
      last_ground_.body_origin=ground.body_origin;last_ground_stamp_=capture_ns;
    }
    size_t measured=0;
    std::vector<float> measured_heights;measured_heights.reserve(ground.heights.size());
    for(size_t i=0;i<kinds.size();++i) {
      if(kinds[i]!=1) continue;  // invalid / no-return keep their own byte
      ++measured;
      cloud.data[16*i+12]=plan_env::encodeGroundHeight(ground.heights[i]);
      if(std::isfinite(ground.heights[i])) measured_heights.push_back(ground.heights[i]);
    }
    publisher_->publish(cloud);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    double h50=std::numeric_limits<double>::quiet_NaN();
    if(!measured_heights.empty()) {
      std::nth_element(measured_heights.begin(),measured_heights.begin()+measured_heights.size()/2,measured_heights.end());
      h50=measured_heights[measured_heights.size()/2];
    }
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,
        "[OBSERVED_GROUND] valid=%d support_cells=%zu measured=%zu known=%zu unknown=%zu h50=%.2fm slope=%.3f processing=%.2fms reason=%s candidate_cells=%zu inliers=%zu slope_rejected=%zu height_rejected=%zu ground_world_z=%.3f",
        ground.valid,ground.support_cells,measured,measured_heights.size(),
        measured-measured_heights.size(),h50,ground.plane.head<2>().norm(),ms,
        ground.reason,ground.candidate_cells,ground.best_inliers,ground.rejected_slope,
        ground.rejected_height,ground.valid ? ground.worldPlane().head<2>().dot(body.position.head<2>())+
            ground.worldPlane().z() : std::numeric_limits<double>::quiet_NaN());
  }
  double no_return_range_;bool ground_enabled_;std::string topic_;
  ObservedGroundOptions ground_options_;
  ObservedGround last_ground_;
  int64_t last_ground_stamp_{0};
  Eigen::Vector3d mount_;Eigen::Matrix3d rotation_;
  std::mutex mutex_;std::deque<plan_env::TimedPose> poses_;
  std::deque<std::pair<sensor_msgs::msg::PointCloud2,std::chrono::steady_clock::time_point>> pending_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;ignition::transport::Node transport_;
};
}
int main(int argc,char** argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<scan_planner::Mid360RayBridge>());rclcpp::shutdown();return 0;}
