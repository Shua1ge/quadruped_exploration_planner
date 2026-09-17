#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/execution_command.hpp>
#include <scan_planner_msgs/msg/execution_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
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
    heading_stall_timeout_ = declare_parameter<double>("heading_stall_timeout", 3.0);
    heading_progress_epsilon_ = declare_parameter<double>("heading_progress_epsilon", 0.05);
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
        max_handoff_error_ <= 0.0 || heading_stall_timeout_ <= 0.0 ||
        heading_progress_epsilon_ <= 0.0)
      throw std::runtime_error(
          "handoff parameters must have a non-negative search time and positive sample interval/error limit");

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_state_pub_ = create_publisher<scan_planner_msgs::msg::ExecutionState>(
        "planning/execution_state", rclcpp::QoS(20).reliable());
    heading_error_pub_ = create_publisher<std_msgs::msg::Float64>("planning/go2_heading_error", 10);
    heading_stalled_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_heading_stalled", 10);
    execution_event_pub_ = create_publisher<std_msgs::msg::String>(
        "planning/local_execution_event", rclcpp::QoS(10).reliable());
    execution_command_sub_ = create_subscription<scan_planner_msgs::msg::ExecutionCommand>(
        "planning/execution_command", rclcpp::QoS(20).reliable(),
        std::bind(&ClosedLoopController::executionCommandCallback, this, std::placeholders::_1));
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

  void publishExecutionState(
      uint8_t state, const std::string &reason = "",
      double terminal_error = std::numeric_limits<double>::quiet_NaN(),
      bool force = false)
  {
    const auto current_time = now();
    if (!force && state == last_execution_state_ &&
        (current_time - last_execution_state_publish_time_).seconds() < 0.1)
      return;
    scan_planner_msgs::msg::ExecutionState msg;
    msg.request_id = active_request_id_;
    msg.trajectory_id = traj_id_;
    msg.state = state;
    msg.execution_time = exec_time_;
    msg.duration = traj_duration_;
    msg.terminal_error = terminal_error;
    msg.reason = reason;
    execution_state_pub_->publish(msg);
    last_execution_state_ = state;
    last_execution_state_publish_time_ = current_time;
  }

  void publishHeadingStalled(bool stalled)
  {
    std_msgs::msg::Bool msg;
    msg.data = stalled;
    heading_stalled_pub_->publish(msg);
  }

  void publishExecutionEvent(const char *event, double terminal_error)
  {
    std_msgs::msg::String msg;
    msg.data = std::string(event) +
        " request_id=" + std::to_string(active_request_id_) +
        " trajectory_id=" + std::to_string(traj_id_) +
        " terminal_error=" + std::to_string(terminal_error);
    execution_event_pub_->publish(msg);
  }

  void resetHeadingFreezeState(bool publish_clear = true)
  {
    const bool had_freeze_state = heading_frozen_ || heading_stall_reported_;
    if (heading_frozen_)
      RCLCPP_INFO(get_logger(), "[HEADING_FREEZE_EXIT] request_id=%llu trajectory=%lld",
                  static_cast<unsigned long long>(active_request_id_),
                  static_cast<long long>(traj_id_));
    heading_frozen_ = false;
    heading_stall_reported_ = false;
    heading_best_error_ = std::numeric_limits<double>::infinity();
    if (publish_clear && had_freeze_state)
      publishHeadingStalled(false);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    if (!shouldAcceptTrajectoryVersion(
            msg->request_id, msg->traj_id,
            active_request_id_,
            receive_traj_ ? traj_id_ : std::numeric_limits<std::int64_t>::min()))
    {
      RCLCPP_WARN(
          get_logger(),
          "[STALE_TRAJECTORY_IGNORED] request_id=%llu trajectory=%lld active_request_id=%llu active_trajectory=%lld",
          static_cast<unsigned long long>(msg->request_id),
          static_cast<long long>(msg->traj_id),
          static_cast<unsigned long long>(active_request_id_),
          static_cast<long long>(traj_id_));
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
      active_request_id_ = msg->request_id;
      traj_id_ = msg->traj_id;
      soft_hold_ = true;
      soft_hold_request_id_ = msg->request_id;
      soft_hold_trajectory_id_ = msg->traj_id;
      soft_hold_reason_ = "STATIONARY_HOLD_TRAJECTORY";
      // This is still an execution version.  Remember it so an older spline or
      // command from the same request cannot become current again.
      receive_traj_ = true;
      resetHeadingFreezeState();
      publishExecutionState(
          scan_planner_msgs::msg::ExecutionState::STATE_SOFT_HOLD,
          soft_hold_reason_, std::numeric_limits<double>::quiet_NaN(), true);
      RCLCPP_INFO(get_logger(),
                  "[VERSIONED_HOLD_TRAJECTORY] request_id=%llu trajectory=%lld; holding without tracking an old stop position",
                  static_cast<unsigned long long>(msg->request_id),
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
    active_request_id_ = msg->request_id;
    exec_time_ = matched_time;
    last_update_time_ = now();
    receive_traj_ = true;
    terminal_event_reported_ = false;
    terminal_best_error_ = std::numeric_limits<double>::infinity();
    terminal_last_progress_time_ = now();
    resetHeadingFreezeState();
    if (soft_hold_ && trajectorySupersedesExecutionCommand(
            msg->request_id, msg->traj_id,
            soft_hold_request_id_, soft_hold_trajectory_id_))
    {
      RCLCPP_INFO(
          get_logger(),
          "[VERSIONED_HOLD_RELEASED] hold_request=%llu hold_trajectory=%lld replacement_request=%llu replacement_trajectory=%lld",
          static_cast<unsigned long long>(soft_hold_request_id_),
          static_cast<long long>(soft_hold_trajectory_id_),
          static_cast<unsigned long long>(msg->request_id),
          static_cast<long long>(msg->traj_id));
      soft_hold_ = false;
      soft_hold_reason_.clear();
    }
    publishExecutionState(
        soft_hold_ ? scan_planner_msgs::msg::ExecutionState::STATE_SOFT_HOLD
                   : scan_planner_msgs::msg::ExecutionState::STATE_RUNNING,
        soft_hold_ ? soft_hold_reason_ : "", 0.0, true);
    RCLCPP_INFO(get_logger(),
                "[TRAJECTORY_HANDOFF] request_id=%llu trajectory=%lld duration=%.3fs matched_time=%.3fs start_error=%.3fm matched_error=%.3fm",
                static_cast<unsigned long long>(active_request_id_),
                static_cast<long long>(traj_id_), traj_duration_, exec_time_,
                start_error, matched_error);
  }

  void executionCommandCallback(
      const scan_planner_msgs::msg::ExecutionCommand::ConstSharedPtr msg)
  {
    if (!executionCommandTargetsCurrentOrNewer(
            msg->request_id, msg->trajectory_id,
            active_request_id_, receive_traj_ ? traj_id_ : 0))
    {
      RCLCPP_WARN(
          get_logger(),
          "[STALE_EXECUTION_COMMAND_IGNORED] command=%u request_id=%llu trajectory=%lld active_request_id=%llu active_trajectory=%lld reason=%s",
          static_cast<unsigned int>(msg->command),
          static_cast<unsigned long long>(msg->request_id),
          static_cast<long long>(msg->trajectory_id),
          static_cast<unsigned long long>(active_request_id_),
          static_cast<long long>(traj_id_), msg->reason.c_str());
      return;
    }
    if (msg->command == scan_planner_msgs::msg::ExecutionCommand::COMMAND_HOLD)
    {
      soft_hold_ = true;
      soft_hold_request_id_ = msg->request_id;
      soft_hold_trajectory_id_ = msg->trajectory_id;
      soft_hold_reason_ = msg->reason;
      resetHeadingFreezeState();
      publishExecutionState(
          scan_planner_msgs::msg::ExecutionState::STATE_SOFT_HOLD,
          soft_hold_reason_, std::numeric_limits<double>::quiet_NaN(), true);
      RCLCPP_INFO(
          get_logger(),
          "[EXECUTION_HOLD_ACCEPTED] request_id=%llu trajectory=%lld reason=%s",
          static_cast<unsigned long long>(msg->request_id),
          static_cast<long long>(msg->trajectory_id), msg->reason.c_str());
    }
    else if (msg->command == scan_planner_msgs::msg::ExecutionCommand::COMMAND_RESUME &&
             soft_hold_ && executionCommandTargetsCurrentOrNewer(
                 msg->request_id, msg->trajectory_id,
                 soft_hold_request_id_, soft_hold_trajectory_id_))
    {
      soft_hold_ = false;
      soft_hold_reason_.clear();
      last_update_time_ = now();
      RCLCPP_INFO(
          get_logger(),
          "[EXECUTION_RESUME_ACCEPTED] request_id=%llu trajectory=%lld",
          static_cast<unsigned long long>(msg->request_id),
          static_cast<long long>(msg->trajectory_id));
    }
  }

  void simulationCollisionCallback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (msg->data)
    {
      simulation_collision_latched_ = true;
      publishExecutionState(
          scan_planner_msgs::msg::ExecutionState::STATE_HARD_STOP,
          "SIMULATION_COLLISION", std::numeric_limits<double>::quiet_NaN(), true);
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
    if (simulation_collision_latched_)
    {
      resetHeadingFreezeState();
      publishExecutionState(
          scan_planner_msgs::msg::ExecutionState::STATE_HARD_STOP,
          "SIMULATION_COLLISION");
      publishStop();
      return;
    }

    if (soft_hold_)
    {
      resetHeadingFreezeState();
      publishExecutionState(
          scan_planner_msgs::msg::ExecutionState::STATE_SOFT_HOLD,
          soft_hold_reason_);
      publishStop();
      return;
    }

    if (!receive_traj_ || !have_odom_)
    {
      resetHeadingFreezeState();
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_IDLE);
      publishStop();
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    std_msgs::msg::Float64 heading_error_msg;
    heading_error_msg.data = yaw_error;
    heading_error_pub_->publish(heading_error_msg);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      const double abs_error = std::abs(yaw_error);
      if (!heading_frozen_)
      {
        heading_frozen_ = true;
        heading_stall_reported_ = false;
        heading_best_error_ = abs_error;
        heading_freeze_started_ = current_time;
        heading_last_progress_ = current_time;
        RCLCPP_WARN(get_logger(),
                    "[HEADING_FREEZE_ENTER] request_id=%llu trajectory=%lld error=%.3frad threshold=%.3frad",
                    static_cast<unsigned long long>(active_request_id_),
                    static_cast<long long>(traj_id_), abs_error,
                    heading_error_threshold_);
      }
      else if (abs_error + heading_progress_epsilon_ < heading_best_error_)
      {
        heading_best_error_ = abs_error;
        heading_last_progress_ = current_time;
      }

      const double frozen_for = (current_time - heading_freeze_started_).seconds();
      const double stalled_for = (current_time - heading_last_progress_).seconds();
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[HEADING_FREEZE_ACTIVE] request_id=%llu error=%.3frad best=%.3frad frozen=%.2fs no_progress=%.2fs",
          static_cast<unsigned long long>(active_request_id_), abs_error,
          heading_best_error_, frozen_for, stalled_for);
      if (!heading_stall_reported_ && stalled_for >= heading_stall_timeout_)
      {
        heading_stall_reported_ = true;
        publishHeadingStalled(true);
        RCLCPP_ERROR(get_logger(),
                     "[HEADING_FREEZE_STALLED] request_id=%llu trajectory=%lld error=%.3frad best=%.3frad no_progress=%.2fs",
                     static_cast<unsigned long long>(active_request_id_),
                     static_cast<long long>(traj_id_), abs_error,
                     heading_best_error_, stalled_for);
      }
      publishExecutionState(
          scan_planner_msgs::msg::ExecutionState::STATE_HEADING_FROZEN,
          heading_stall_reported_ ? "HEADING_STALLED" : "HEADING_ALIGNMENT");
      publishStop(yaw_command);
      last_update_time_ = current_time;
      return;
    }

    resetHeadingFreezeState();
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
    const double terminal_error = pos_error.norm();
    if (exec_time_ >= traj_duration_ && terminal_error < finish_dist_)
    {
      command = geometry_msgs::msg::Twist();
      publishExecutionState(
          scan_planner_msgs::msg::ExecutionState::STATE_FINISHED,
          "TERMINAL_REACHED", terminal_error);
      if (!terminal_event_reported_)
      {
        terminal_event_reported_ = true;
        publishExecutionEvent("LOCAL_TRAJECTORY_FINISHED", terminal_error);
        RCLCPP_INFO(get_logger(),
                    "[LOCAL_TRAJECTORY_FINISHED] request_id=%llu trajectory=%lld terminal_error=%.3fm",
                    static_cast<unsigned long long>(active_request_id_),
                    static_cast<long long>(traj_id_), terminal_error);
      }
    }
    else if (exec_time_ >= traj_duration_)
    {
      if (terminal_error + 0.02 < terminal_best_error_)
      {
        terminal_best_error_ = terminal_error;
        terminal_last_progress_time_ = current_time;
      }
      const double no_progress =
          (current_time - terminal_last_progress_time_).seconds();
      if (!terminal_event_reported_ &&
          no_progress >= std::max(1.0, time_forward_))
      {
        terminal_event_reported_ = true;
        publishExecutionState(
            scan_planner_msgs::msg::ExecutionState::STATE_STALLED,
            "TERMINAL_NO_PROGRESS", terminal_error, true);
        publishExecutionEvent("LOCAL_TRAJECTORY_STALLED", terminal_error);
        RCLCPP_ERROR(get_logger(),
                     "[LOCAL_TRAJECTORY_STALLED] request_id=%llu trajectory=%lld terminal_error=%.3fm no_progress=%.2fs",
                     static_cast<unsigned long long>(active_request_id_),
                     static_cast<long long>(traj_id_), terminal_error,
                     no_progress);
      }
      else if (terminal_event_reported_)
      {
        publishExecutionState(
            scan_planner_msgs::msg::ExecutionState::STATE_STALLED,
            "TERMINAL_NO_PROGRESS", terminal_error);
      }
      else
      {
        publishExecutionState(
            scan_planner_msgs::msg::ExecutionState::STATE_RUNNING,
            "TERMINAL_CONVERGING", terminal_error);
      }
    }
    else
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_RUNNING);
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<scan_planner_msgs::msg::ExecutionState>::SharedPtr execution_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heading_stalled_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_event_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::ExecutionCommand>::SharedPtr execution_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr simulation_collision_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  bool simulation_collision_latched_{false};
  bool soft_hold_{false};
  std::uint64_t soft_hold_request_id_{0};
  std::int64_t soft_hold_trajectory_id_{0};
  std::string soft_hold_reason_;
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  std::uint64_t active_request_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time heading_freeze_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time heading_last_progress_{0, 0, RCL_ROS_TIME};
  bool heading_frozen_{false};
  bool heading_stall_reported_{false};
  bool terminal_event_reported_{false};
  double terminal_best_error_{std::numeric_limits<double>::infinity()};
  rclcpp::Time terminal_last_progress_time_{0, 0, RCL_ROS_TIME};
  uint8_t last_execution_state_{std::numeric_limits<uint8_t>::max()};
  rclcpp::Time last_execution_state_publish_time_{0, 0, RCL_ROS_TIME};
  double heading_best_error_{std::numeric_limits<double>::infinity()};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double heading_stall_timeout_, heading_progress_epsilon_;
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
