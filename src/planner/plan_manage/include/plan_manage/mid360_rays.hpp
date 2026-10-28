#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <ignition/msgs/laserscan.pb.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace scan_planner {
// Internal simultaneous-ray contract (not Livox wire format):
// return_type=0 invalid, 1 measured endpoint, 2 bounded measured no-return.
// Pose-aligned labels: 3 observed ground support, 4 negative step below
// nearby support, 5 protrusion above it; physical XYZ never changes.
// Invalid rays retain their organized index but have NaN XYZ. A miss endpoint
// is only a clearing limit and must never be inserted as an occupied voxel.
inline sensor_msgs::msg::PointCloud2 mid360RayCloud(
    const ignition::msgs::LaserScan& scan, double no_return_range) {
  const size_t width=scan.count(), height=scan.vertical_count();
  if (width<2 || height<1 || width*height>1000000 ||
      width*height!=static_cast<size_t>(scan.ranges_size()) ||
      scan.frame().empty() || !scan.has_header() || !scan.header().has_stamp() ||
      !std::isfinite(scan.angle_min()) || !std::isfinite(scan.angle_step()) ||
      !std::isfinite(scan.vertical_angle_min()) ||
      !std::isfinite(scan.vertical_angle_step()) || scan.angle_step()<=0. ||
      (height>1 && scan.vertical_angle_step()<=0.) ||
      !std::isfinite(scan.range_min()) || !std::isfinite(scan.range_max()) ||
      scan.range_min()<0. || scan.range_max()<=scan.range_min() ||
      !std::isfinite(no_return_range) || no_return_range<=0.)
    throw std::invalid_argument("invalid full 3-D Gazebo scan geometry");
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec=scan.header().stamp().sec();
  cloud.header.stamp.nanosec=scan.header().stamp().nsec();
  cloud.header.frame_id=scan.frame();
  // Gazebo LaserScan uses scoped names; match ros_gz_bridge's ROS TF names.
  for (size_t position=0; (position=cloud.header.frame_id.find("::",position))!=
       std::string::npos; ++position)
    cloud.header.frame_id.replace(position,2,"/");
  cloud.width=width; cloud.height=height; cloud.point_step=16;
  cloud.row_step=cloud.width*cloud.point_step; cloud.is_dense=false;
  // ROS messages declare host byte order explicitly.
  const uint16_t endian_probe=1;
  cloud.is_bigendian=(*reinterpret_cast<const uint8_t*>(&endian_probe)==0);
  for (size_t i=0; i<4; ++i) {
    sensor_msgs::msg::PointField field;
    field.name=i==0 ? "x" : i==1 ? "y" : i==2 ? "z" : "return_type";
    field.offset=i*4; field.count=1;
    field.datatype=i<3 ? sensor_msgs::msg::PointField::FLOAT32 :
                         sensor_msgs::msg::PointField::UINT8;
    cloud.fields.push_back(field);
  }
  cloud.data.resize(cloud.row_step*height,0);
  for (size_t row=0; row<height; ++row) {
    const double elevation=scan.vertical_angle_min()+row*scan.vertical_angle_step();
    for (size_t col=0; col<width; ++col) {
      const size_t index=row*width+col;
      const double measured=scan.ranges(index);
      const bool finite=std::isfinite(measured) && measured>=scan.range_min() &&
                        measured<=scan.range_max();
      const bool hit=finite && measured<scan.range_max();
      const bool miss=(std::isinf(measured) && measured>0.) ||
                      (finite && measured==scan.range_max());
      const uint8_t kind=hit ? 1 : miss ? 2 : 0;
      const double length=hit ? measured : std::min(scan.range_max(),no_return_range);
      const double azimuth=scan.angle_min()+col*scan.angle_step();
      const float invalid=std::numeric_limits<float>::quiet_NaN();
      const float xyz[3]={
        kind ? static_cast<float>(length*std::cos(elevation)*std::cos(azimuth)) : invalid,
        kind ? static_cast<float>(length*std::cos(elevation)*std::sin(azimuth)) : invalid,
        kind ? static_cast<float>(length*std::sin(elevation)) : invalid};
      auto* destination=cloud.data.data()+index*cloud.point_step;
      std::memcpy(destination,xyz,sizeof(xyz)); destination[12]=kind;
    }
  }
  return cloud;
}
} // namespace scan_planner
