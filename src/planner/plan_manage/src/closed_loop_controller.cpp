#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"
#include "plan_manage/replan_fsm_utils.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    handoff_search_time_ = declare_parameter<double>("handoff_search_time", 1.0);
    handoff_sample_dt_ = declare_parameter<double>("handoff_sample_dt", 0.02);
    max_handoff_error_ = declare_parameter<double>("max_handoff_error", 0.30);
    if (handoff_search_time_ < 0.0 || handoff_sample_dt_ <= 0.0 ||
        max_handoff_error_ <= 0.0)
      throw std::runtime_error(
          "handoff parameters must have a non-negative search time and positive sample interval/error limit");

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "planning/emergency_stop", 10,
        std::bind(&ClosedLoopController::emergencyStopCallback, this, std::placeholders::_1));
    simulation_collision_sub_ = create_subscription<std_msgs::msg::Bool>(
        "simulation/collision", 10,
        std::bind(&ClosedLoopController::simulationCollisionCallback, this, std::placeholders::_1));
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);

    std::vector<Eigen::Vector3d> control_points;
    control_points.reserve(points.cols());
    for (Eigen::Index column = 0; column < points.cols(); ++column)
      control_points.push_back(points.col(column));
    if (trajectorySamplesAreStationary(control_points, 1e-4))
    {
      emergency_stop_ = true;
      RCLCPP_INFO(get_logger(),
                  "[EMERGENCY_HOLD_TRAJECTORY_IGNORED] trajectory=%lld; keeping cmd_vel stopped instead of tracking an old stop position",
                  static_cast<long long>(msg->traj_id));
      return;
    }

    std::vector<UniformBspline> candidate = {position, position.getDerivative()};
    candidate.push_back(candidate[1].getDerivative());
    const double candidate_duration = candidate[0].getTimeSum();
    double matched_time = 0.0;
    double start_error = 0.0;
    double matched_error = 0.0;
    if (have_odom_)
    {
      const double search_end = std::min(candidate_duration, handoff_search_time_);
      std::vector<Eigen::Vector3d> samples;
      std::vector<double> sample_times;
      for (double t = 0.0; t < search_end; t += handoff_sample_dt_)
      {
        samples.push_back(candidate[0].evaluateDeBoorT(t));
        sample_times.push_back(t);
      }
      samples.push_back(candidate[0].evaluateDeBoorT(search_end));
      sample_times.push_back(search_end);
      const size_t match = closestForwardTrajectorySample(samples, odom_pos_);
      matched_time = sample_times[match];
      start_error = (samples.front().head<2>() - odom_pos_.head<2>()).norm();
      matched_error = (samples[match].head<2>() - odom_pos_.head<2>()).norm();
      if (matched_error > max_handoff_error_)
      {
        RCLCPP_WARN(get_logger(),
                    "[TRAJECTORY_HANDOFF_REJECTED] trajectory=%lld start_error=%.3fm matched_error=%.3fm limit=%.3fm; preserving current execution state",
                    static_cast<long long>(msg->traj_id), start_error,
                    matched_error, max_handoff_error_);
        return;
      }
    }

    traj_ = std::move(candidate);
    traj_duration_ = candidate_duration;
    traj_id_ = msg->traj_id;
    exec_time_ = matched_time;
    last_update_time_ = now();
    receive_traj_ = true;
    // A planner-requested stop may be cleared by a replacement trajectory, but a
    // physical simulation collision stays latched for the lifetime of this run.
    emergency_stop_ = simulation_collision_latched_;
    RCLCPP_INFO(get_logger(),
                "[TRAJECTORY_HANDOFF] trajectory=%lld duration=%.3fs matched_time=%.3fs start_error=%.3fm matched_error=%.3fm",
                static_cast<long long>(traj_id_), traj_duration_, exec_time_,
                start_error, matched_error);
  }

  void emergencyStopCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (msg->data)
      emergency_stop_ = true;
  }

  void simulationCollisionCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (msg->data)
    {
      simulation_collision_latched_ = true;
      emergency_stop_ = true;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Simulation collision guard stopped the robot");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  void cmdCallback()
  {
    if (emergency_stop_)
    {
      publishExecutionFrozen(true);
      publishStop();
      return;
    }

    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr simulation_collision_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool emergency_stop_{false};
  bool simulation_collision_latched_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double handoff_search_time_, handoff_sample_dt_, max_handoff_error_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
