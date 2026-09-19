#include <gz/msgs/entity_wrench.pb.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace scan_planner
{
/// \brief ROS 2 node that converts cmd_vel to planar wrench for Gazebo ApplyLinkWrench.
///
/// Subscribes to /quad_0/cmd_vel (body-frame twist) and odometry, computes
/// XY force + yaw torque via PD control, and publishes to Gazebo Transport
/// /world/{world}/wrench/persistent for the built-in ApplyLinkWrench system.
class PlanarCmdVelAdapter : public rclcpp::Node
{
public:
  PlanarCmdVelAdapter() : Node("planar_cmd_vel_adapter")
  {
    world_name_ = declare_parameter<std::string>("world_name", "apply_link_wrench");
    model_name_ = declare_parameter<std::string>("model_name", "go2");
    link_name_ = declare_parameter<std::string>("link_name", "base");
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    kp_linear_ = declare_parameter<double>("kp_linear", 150.0);
    kd_linear_ = declare_parameter<double>("kd_linear", 30.0);
    kp_angular_ = declare_parameter<double>("kp_angular", 20.0);
    kd_angular_ = declare_parameter<double>("kd_angular", 5.0);
    max_force_ = declare_parameter<double>("max_force", 200.0);
    max_torque_ = declare_parameter<double>("max_torque", 30.0);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&PlanarCmdVelAdapter::cmdVelCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&PlanarCmdVelAdapter::odomCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PlanarCmdVelAdapter::timerCallback, this));

    const std::string wrench_topic = "/world/" + world_name_ + "/wrench/persistent";
    wrench_pub_ = gz_node_.Advertise<ignition::msgs::EntityWrench>(wrench_topic);
    if (!wrench_pub_)
    {
      RCLCPP_ERROR(get_logger(), "Failed to advertise %s", wrench_topic.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(),
        "PlanarCmdVelAdapter: world=%s model=%s link=%s, publishing to %s",
        world_name_.c_str(), model_name_.c_str(), link_name_.c_str(), wrench_topic.c_str());
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_linear_x_ = msg->linear.x;
    target_linear_y_ = msg->linear.y;
    target_angular_z_ = msg->angular.z;
    last_cmd_time_ = now();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_yaw_ = 2.0 * std::atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    current_vx_ = msg->twist.twist.linear.x;
    current_vy_ = msg->twist.twist.linear.y;
    current_vyaw_ = msg->twist.twist.angular.z;
    has_odom_ = true;
  }

  void timerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_odom_)
      return;

    const auto current_time = now();
    const double timeout_sec = (current_time - last_cmd_time_).seconds();
    double target_vx = target_linear_x_;
    double target_vy = target_linear_y_;
    double target_vyaw = target_angular_z_;
    if (timeout_sec > cmd_timeout_)
    {
      target_vx = 0.0;
      target_vy = 0.0;
      target_vyaw = 0.0;
    }

    // Transform body-frame target velocity to world frame
    const double cos_yaw = std::cos(odom_yaw_);
    const double sin_yaw = std::sin(odom_yaw_);
    const double target_vx_world = cos_yaw * target_vx - sin_yaw * target_vy;
    const double target_vy_world = sin_yaw * target_vx + cos_yaw * target_vy;

    // PD control for planar force
    const double error_vx = target_vx_world - current_vx_;
    const double error_vy = target_vy_world - current_vy_;
    double force_x = kp_linear_ * error_vx - kd_linear_ * current_vx_;
    double force_y = kp_linear_ * error_vy - kd_linear_ * current_vy_;

    const double force_mag = std::sqrt(force_x * force_x + force_y * force_y);
    if (force_mag > max_force_)
    {
      const double scale = max_force_ / force_mag;
      force_x *= scale;
      force_y *= scale;
    }

    // PD control for yaw torque
    const double error_vyaw = target_vyaw - current_vyaw_;
    double torque_z = kp_angular_ * error_vyaw - kd_angular_ * current_vyaw_;
    torque_z = std::clamp(torque_z, -max_torque_, max_torque_);

    // Publish wrench message
    ignition::msgs::EntityWrench wrench_msg;
    wrench_msg.mutable_entity()->set_name(model_name_ + "::" + link_name_);
    wrench_msg.mutable_entity()->set_type(ignition::msgs::Entity::LINK);
    wrench_msg.mutable_wrench()->mutable_force()->set_x(force_x);
    wrench_msg.mutable_wrench()->mutable_force()->set_y(force_y);
    wrench_msg.mutable_wrench()->mutable_force()->set_z(0.0);
    wrench_msg.mutable_wrench()->mutable_torque()->set_x(0.0);
    wrench_msg.mutable_wrench()->mutable_torque()->set_y(0.0);
    wrench_msg.mutable_wrench()->mutable_torque()->set_z(torque_z);

    if (!wrench_pub_.Publish(wrench_msg))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Failed to publish wrench");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  ignition::transport::Node gz_node_;
  ignition::transport::Node::Publisher wrench_pub_;

  std::mutex mutex_;
  std::string world_name_;
  std::string model_name_;
  std::string link_name_;
  double cmd_timeout_{0.5};
  double kp_linear_{150.0};
  double kd_linear_{30.0};
  double kp_angular_{20.0};
  double kd_angular_{5.0};
  double max_force_{200.0};
  double max_torque_{30.0};

  double target_linear_x_{0.0};
  double target_linear_y_{0.0};
  double target_angular_z_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  double odom_yaw_{0.0};
  double current_vx_{0.0};
  double current_vy_{0.0};
  double current_vyaw_{0.0};
  bool has_odom_{false};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::PlanarCmdVelAdapter>());
  rclcpp::shutdown();
  return 0;
}
