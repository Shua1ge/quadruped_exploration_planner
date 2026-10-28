#include <gtest/gtest.h>
#include "plan_manage/mid360_rays.hpp"

namespace {
ignition::msgs::LaserScan scan() {
  ignition::msgs::LaserScan message;
  message.set_frame("go2::base::exploration_lidar");
  message.mutable_header()->mutable_stamp()->set_sec(12);
  message.mutable_header()->mutable_stamp()->set_nsec(345);
  message.set_count(3);message.set_vertical_count(2);
  message.set_angle_min(-.1);message.set_angle_step(.1);
  message.set_vertical_angle_min(-.12217304763960307);
  message.set_vertical_angle_step(1.0297442586766545);
  message.set_range_min(.1);message.set_range_max(40.);
  for(int i=0;i<6;++i)message.add_ranges(5.);
  return message;
}
float coordinate(const sensor_msgs::msg::PointCloud2& cloud,size_t index,size_t axis) {
  float value;std::memcpy(&value,cloud.data.data()+index*cloud.point_step+axis*4,4);
  return value;
}
}
TEST(Mid360Rays,PreservesVerticalGeometryAndCaptureFrame) {
  const auto cloud=scan_planner::mid360RayCloud(scan(),3.);
  EXPECT_EQ(cloud.width,3u);EXPECT_EQ(cloud.height,2u);
  EXPECT_EQ(cloud.header.frame_id,"go2/base/exploration_lidar");
  EXPECT_EQ(cloud.header.stamp.sec,12);EXPECT_EQ(cloud.header.stamp.nanosec,345u);
  EXPECT_NEAR(coordinate(cloud,1,2),5.*std::sin(-7.*M_PI/180.),1e-6);
  EXPECT_NEAR(coordinate(cloud,4,2),5.*std::sin(52.*M_PI/180.),1e-6);
  EXPECT_EQ(cloud.data[1*16+12],1u);
}
TEST(Mid360Rays,MissesClearOnlyToBoundAndInvalidRaysStayInvalid) {
  auto message=scan();
  message.set_ranges(0,std::numeric_limits<double>::infinity());
  message.set_ranges(1,-std::numeric_limits<double>::infinity());
  message.set_ranges(2,std::numeric_limits<double>::quiet_NaN());
  message.set_ranges(3,.05);message.set_ranges(4,41.);message.set_ranges(5,40.);
  const auto cloud=scan_planner::mid360RayCloud(message,3.);
  EXPECT_EQ(cloud.data[12],2u);EXPECT_EQ(cloud.data[5*16+12],2u);
  const double length=std::hypot(std::hypot(coordinate(cloud,0,0),
      coordinate(cloud,0,1)),coordinate(cloud,0,2));
  EXPECT_NEAR(length,3.,1e-6);
  for(size_t i=1;i<5;++i){EXPECT_EQ(cloud.data[i*16+12],0u);
    EXPECT_TRUE(std::isnan(coordinate(cloud,i,0)));}
}
TEST(Mid360Rays,RejectsIncompleteOrMalformedScan) {
  auto message=scan();message.set_vertical_count(3);
  EXPECT_THROW(scan_planner::mid360RayCloud(message,3.),std::invalid_argument);
  message=scan();message.set_vertical_angle_step(0.);
  EXPECT_THROW(scan_planner::mid360RayCloud(message,3.),std::invalid_argument);
}
