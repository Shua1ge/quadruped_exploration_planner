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
#include <scan_planner_msgs/msg/locomotion_state.hpp>
#include <scan_planner_msgs/msg/trajectory_ack.hpp>
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
    min_moving_handoff_error_ = declare_parameter<double>(
        "min_moving_handoff_error", 0.10);
    handoff_reaction_time_ = declare_parameter<double>(
        "handoff_reaction_time", 0.25);
    handoff_max_deceleration_ = declare_parameter<double>(
        "handoff_max_deceleration", 0.50);
    max_handoff_yaw_error_ = declare_parameter<double>(
        "max_handoff_yaw_error", 0.80);
    if (handoff_search_time_ < 0.0 || handoff_sample_dt_ <= 0.0 ||
        max_handoff_error_ <= 0.0 || min_moving_handoff_error_ < 0.0 ||
        min_moving_handoff_error_ > max_handoff_error_ ||
        handoff_reaction_time_ < 0.0 || handoff_max_deceleration_ <= 0.0 ||
        max_handoff_yaw_error_ <= 0.0 ||
        heading_stall_timeout_ <= 0.0 ||
        heading_progress_epsilon_ <= 0.0)
      throw std::runtime_error(
          "handoff parameters must have a non-negative search time and positive sample interval/error limit");

    active_max_vx_ = max_vx_;
    active_max_vy_ = max_vy_;
    active_max_vyaw_ = max_vyaw_;
    active_max_tracking_correction_ = std::max(max_vx_, max_vy_);

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_state_pub_ = create_publisher<scan_planner_msgs::msg::ExecutionState>(
        "planning/execution_state", rclcpp::QoS(20).reliable());
    trajectory_ack_pub_ = create_publisher<scan_planner_msgs::msg::TrajectoryAck>(
        "planning/trajectory_ack", rclcpp::QoS(20).reliable());
    heading_error_pub_ = create_publisher<std_msgs::msg::Float64>("planning/go2_heading_error", 10);
    heading_stalled_pub_ = create_publisher<std_msgs::msg::Bool>("planning/go2_heading_stalled", 10);
    execution_event_pub_ = create_publisher<std_msgs::msg::String>(
        "planning/local_execution_event", rclcpp::QoS(10).reliable());
    execution_command_sub_ = create_subscription<scan_planner_msgs::msg::ExecutionCommand>(
        "planning/execution_command", rclcpp::QoS(20).reliable(),
        std::bind(&ClosedLoopController::executionCommandCallback, this, std::placeholders::_1));
    locomotion_state_sub_ = create_subscription<scan_planner_msgs::msg::LocomotionState>(
        "/robot/locomotion_state", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&ClosedLoopController::locomotionStateCallback, this, std::placeholders::_1));
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
    return normalizePlanarAngle(angle);
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  void publishTrajectoryAck(
      const scan_planner_msgs::msg::Bspline &candidate, uint8_t status,
      const std::string &reason, double matched_time = 0.0,
      double position_error = std::numeric_limits<double>::quiet_NaN(),
      double yaw_error = std::numeric_limits<double>::quiet_NaN())
  {
    scan_planner_msgs::msg::TrajectoryAck ack;
    ack.stamp = now();
    ack.request_id = candidate.request_id;
    ack.trajectory_id = candidate.traj_id;
    ack.status = status;
    ack.reason = reason;
    ack.matched_time = matched_time;
    ack.position_error = position_error;
    ack.yaw_error = yaw_error;
    trajectory_ack_pub_->publish(ack);
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    if (fixed_trajectory_yaw_)
      return trajectory_yaw_;
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
    cmd.angular.z = std::clamp(
        yaw_rate, -active_max_vyaw_, active_max_vyaw_);
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
    if (!validBsplineStructure(
            msg->pos_pts.size(), msg->knots.size(), msg->order) ||
        !validBsplineKnots(msg->knots, msg->pos_pts.size(), msg->order))
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      publishTrajectoryAck(
          *msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
          "INVALID_SPLINE_SHAPE");
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
      publishTrajectoryAck(
          *msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
          "STALE_TRAJECTORY_VERSION");
      return;
    }
    if (msg->execution_mode >
            scan_planner_msgs::msg::Bspline::MODE_RECOVERY_PRIMITIVE ||
        !std::isfinite(msg->max_forward_speed) ||
        !std::isfinite(msg->max_lateral_speed) ||
        !std::isfinite(msg->max_yaw_rate) ||
        !std::isfinite(msg->max_tracking_correction) ||
        !std::isfinite(msg->min_forward_command) ||
        !std::isfinite(msg->min_forward_effective_speed) ||
        msg->min_forward_command < 0.0 ||
        msg->min_forward_effective_speed < 0.0 ||
        (msg->min_forward_command > 0.0 &&
         (msg->min_forward_effective_speed <= 0.0 ||
          msg->min_forward_command > std::min(max_vx_, msg->max_forward_speed))) ||
        msg->max_forward_speed < 0.0 || msg->max_lateral_speed < 0.0 ||
        msg->max_yaw_rate < 0.0 || msg->max_tracking_correction < 0.0)
    {
      RCLCPP_WARN(
          get_logger(), "Ignoring B-spline with invalid execution profile");
      publishTrajectoryAck(
          *msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
          "INVALID_EXECUTION_PROFILE");
      return;
    }
    const double candidate_max_vx = msg->max_forward_speed > 0.0
        ? std::min(msg->max_forward_speed, max_vx_) : max_vx_;
    const double candidate_max_vy = msg->max_lateral_speed > 0.0
        ? std::min(msg->max_lateral_speed, max_vy_) : max_vy_;
    const double candidate_max_vyaw = msg->max_yaw_rate > 0.0
        ? std::min(msg->max_yaw_rate, max_vyaw_) : max_vyaw_;
    const double candidate_max_tracking_correction =
        msg->max_tracking_correction > 0.0
            ? msg->max_tracking_correction
            : std::max(candidate_max_vx, candidate_max_vy);
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    if (!points.allFinite() || !knots.allFinite())
    {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite B-spline");
      publishTrajectoryAck(
          *msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
          "NONFINITE_SPLINE");
      return;
    }
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);

    const bool candidate_fixed_yaw = !msg->yaw_pts.empty();
    const double candidate_body_yaw =
        candidate_fixed_yaw ? msg->yaw_pts.front() : 0.0;
    if (candidate_fixed_yaw && !std::isfinite(candidate_body_yaw))
    {
      RCLCPP_WARN(get_logger(), "Ignoring B-spline with invalid body yaw");
      publishTrajectoryAck(
          *msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
          "INVALID_BODY_YAW");
      return;
    }

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
      publishTrajectoryAck(
          *msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_ACCEPTED,
          "STATIONARY_HOLD_ACCEPTED");
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
    double matched_yaw_error = 0.0;
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
      const double handoff_speed = odom_vel_.head<2>().norm();
      const double dynamic_handoff_limit = speedAwareHandoffErrorLimit(
          handoff_speed, handoff_max_deceleration_, handoff_reaction_time_,
          min_moving_handoff_error_, max_handoff_error_);
      if (matched_error > dynamic_handoff_limit)
      {
        RCLCPP_WARN(get_logger(),
                    "[TRAJECTORY_HANDOFF_REJECTED] trajectory=%lld start_error=%.3fm matched_error=%.3fm dynamic_limit=%.3fm speed=%.3fm/s; preserving current execution state",
                    static_cast<long long>(msg->traj_id), start_error,
                    matched_error, dynamic_handoff_limit, handoff_speed);
        publishTrajectoryAck(
            *msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
            "HANDOFF_POSITION_ERROR", matched_time, matched_error,
            matched_yaw_error);
        return;
      }

      // A rolling candidate may legitimately leave the current pose in a new
      // direction after local repair.  Position continuity is mandatory, but
      // yaw continuity is not: cmdCallback already freezes translation and
      // performs HEADING_ALIGNMENT whenever the tangent error exceeds
      // heading_error_threshold_.  Rejecting such a candidate here leaves an
      // expired active spline installed forever and creates a replan/reject
      // loop.  Measure the discontinuity for diagnostics and accept the
      // candidate so the existing in-place alignment state can resolve it.
      if (candidate_fixed_yaw)
      {
        matched_yaw_error = std::abs(
            normalizeAngle(candidate_body_yaw - odom_yaw_));
      }
      else if (receive_traj_ && !soft_hold_)
      {
        const double before_time = std::max(
            0.0, matched_time - handoff_sample_dt_);
        const double after_time = std::min(
            candidate_duration, matched_time + handoff_sample_dt_);
        const Eigen::Vector3d before =
            candidate[0].evaluateDeBoorT(before_time);
        const Eigen::Vector3d after =
            candidate[0].evaluateDeBoorT(after_time);
        const Eigen::Vector2d tangent = after.head<2>() - before.head<2>();
        if (tangent.squaredNorm() > 1e-8)
        {
          const double candidate_yaw = std::atan2(tangent.y(), tangent.x());
          matched_yaw_error = std::abs(
              normalizeAngle(candidate_yaw - odom_yaw_));
          if (matched_yaw_error > max_handoff_yaw_error_)
          {
            RCLCPP_WARN(
                get_logger(),
                "[TRAJECTORY_HANDOFF_HEADING_ALIGNMENT] trajectory=%lld matched_yaw_error=%.3frad threshold=%.3frad; accepting position-continuous candidate and freezing translation until aligned",
                static_cast<long long>(msg->traj_id), matched_yaw_error,
                max_handoff_yaw_error_);
          }
        }
      }
    }

    traj_ = std::move(candidate);
    traj_duration_ = candidate_duration;
    traj_id_ = msg->traj_id;
    active_request_id_ = msg->request_id;
    fixed_trajectory_yaw_ = candidate_fixed_yaw;
    trajectory_yaw_ = candidate_body_yaw;
    active_execution_mode_ = msg->execution_mode;
    active_max_vx_ = candidate_max_vx;
    active_min_forward_command_ = msg->min_forward_command;
    active_min_forward_effective_speed_ = msg->min_forward_effective_speed;
    active_max_vy_ = candidate_max_vy;
    active_max_vyaw_ = candidate_max_vyaw;
    active_max_tracking_correction_ =
        candidate_max_tracking_correction;
    exec_time_ = matched_time;
    last_update_time_ = now();
    receive_traj_ = true;
    terminal_event_reported_ = false;
    spatial_terminal_reached_ = false;
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
    publishTrajectoryAck(
        *msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_ACCEPTED,
        "HANDOFF_ACCEPTED", matched_time, matched_error, matched_yaw_error);
    publishExecutionState(
        soft_hold_ ? scan_planner_msgs::msg::ExecutionState::STATE_SOFT_HOLD
                   : scan_planner_msgs::msg::ExecutionState::STATE_RUNNING,
        soft_hold_ ? soft_hold_reason_ : "", 0.0, true);
    RCLCPP_INFO(get_logger(),
                "[TRAJECTORY_HANDOFF] request_id=%llu trajectory=%lld duration=%.3fs matched_time=%.3fs start_error=%.3fm matched_error=%.3fm matched_yaw_error=%.3frad fixed_body_yaw=%d execution_mode=%u limits=(%.2f,%.2f,%.2f) correction=%.2f",
                static_cast<unsigned long long>(active_request_id_),
                static_cast<long long>(traj_id_), traj_duration_, exec_time_,
                start_error, matched_error, matched_yaw_error,
                fixed_trajectory_yaw_ ? 1 : 0,
                static_cast<unsigned int>(active_execution_mode_),
                active_max_vx_, active_max_vy_, active_max_vyaw_,
                active_max_tracking_correction_);
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

  void locomotionStateCallback(
      const scan_planner_msgs::msg::LocomotionState::ConstSharedPtr msg)
  {
    locomotion_actuation_ready_ = msg->actuation_ready;
    last_locomotion_state_time_ = now();
  }

  bool locomotionReady(const rclcpp::Time &current_time) const
  {
    return locomotion_actuation_ready_ &&
           last_locomotion_state_time_.nanoseconds() > 0 &&
           (current_time - last_locomotion_state_time_).seconds() <= 0.5;
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
    const Eigen::Vector3d position(
        msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    const Eigen::Vector3d velocity(
        msg->twist.twist.linear.x, msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
    Eigen::Quaterniond orientation(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    if (!position.allFinite() || !velocity.allFinite() ||
        !normalizeFiniteQuaternion(&orientation))
    {
      have_odom_ = false;
      publishStop();
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[INVALID_ODOMETRY_REJECTED] controller stopped; pose, velocity, or quaternion is non-finite/degenerate");
      return;
    }
    const double yaw = std::atan2(
        2.0 * (orientation.w() * orientation.z() +
               orientation.x() * orientation.y()),
        1.0 - 2.0 * (orientation.y() * orientation.y() +
                     orientation.z() * orientation.z()));
    if (!std::isfinite(yaw))
    {
      have_odom_ = false;
      publishStop();
      return;
    }
    odom_pos_ = position;
    odom_vel_ = velocity;
    odom_yaw_ = yaw;
    have_odom_ = true;
  }

  void cmdCallback()
  {
    const auto current_time = now();
    if (!locomotionReady(current_time))
    {
      last_update_time_ = current_time;
      publishStop();
      return;
    }
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
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const bool spatial_gait = active_min_forward_command_ > 0.0;
    if (spatial_gait && spatial_terminal_reached_)
    {
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_FINISHED,
                            "TERMINAL_REACHED");
      publishStop();
      last_update_time_ = current_time;
      return;
    }
    double spatial_target_time = exec_time_;
    if (spatial_gait)
    {
      // Match actual progress, not a reference clock that advances while the
      // calibrated gait is slower than its command. Never chase a past point.
      double best_distance = (traj_[0].evaluateDeBoorT(exec_time_) - odom_pos_).head<2>().squaredNorm();
      const double search_end = std::min(traj_duration_, exec_time_ + 2.0);
      for (double t = exec_time_; t <= search_end + 1e-9; t += 0.05)
      {
        const double distance = (traj_[0].evaluateDeBoorT(t) - odom_pos_).head<2>().squaredNorm();
        if (distance < best_distance)
        {
          best_distance = distance;
          spatial_target_time = t;
        }
      }
      exec_time_ = spatial_target_time;
      spatial_target_time = forwardGaitLookaheadTime(exec_time_, traj_duration_,
          [&](double t) { return traj_[0].evaluateDeBoorT(t); });
      const double endpoint_error = (traj_[0].evaluateDeBoorT(traj_duration_) - odom_pos_).head<2>().norm();
      if (spatial_target_time >= traj_duration_ && endpoint_error < finish_dist_)
      {
        exec_time_ = traj_duration_;
        terminal_event_reported_ = true;
        spatial_terminal_reached_ = true;
        publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_FINISHED,
                              "TERMINAL_REACHED", endpoint_error, true);
        publishExecutionEvent("LOCAL_TRAJECTORY_FINISHED", endpoint_error);
        publishStop();
        last_update_time_ = current_time;
        return;
      }
    }
    const double t_eval = std::min(spatial_gait ? spatial_target_time : exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double desired_yaw = spatial_gait
        ? std::atan2(pos_des.y() - odom_pos_.y(), pos_des.x() - odom_pos_.x())
        : estimateDesiredYaw(t_eval, pos_des);
    const double yaw_error = normalizeAngle(desired_yaw - odom_yaw_);
    std_msgs::msg::Float64 heading_error_msg;
    heading_error_msg.data = yaw_error;
    heading_error_pub_->publish(heading_error_msg);
    const double yaw_command = std::clamp(
        kp_yaw_ * yaw_error, -active_max_vyaw_, active_max_vyaw_);
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
    if (!spatial_gait)
      exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d tracking_correction = clampNorm(
        kp_pos_ * pos_error, active_max_tracking_correction_);
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + tracking_correction,
        std::max(active_max_vx_, active_max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(
        c * vel_world.x() + s * vel_world.y(),
        -active_max_vx_, active_max_vx_);
    command.linear.x = calibratedForwardGaitCommand(
        command.linear.x, active_min_forward_command_,
        active_min_forward_effective_speed_, active_max_vx_);
    command.linear.y = std::clamp(
        -s * vel_world.x() + c * vel_world.y(),
        -active_max_vy_, active_max_vy_);
    command.angular.z = yaw_command;
    if (spatial_gait)
    {
      // This policy has no calibrated reverse/strafe primitive. Steer toward
      // the forward target; do not alternate tiny reverse and minimum forward.
      command.linear.x = active_min_forward_command_;
      command.linear.y = 0.0;
    }
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

      // Once terminal convergence has made no physical progress, continuing
      // to publish a shrinking proportional command leaves an RL quadruped in
      // its low-speed gait deadband (typically a small reverse/lateral
      // command).  Hold this validated active trajectory stationary while
      // SCAN creates and validates a replacement from current odometry.
      if (terminal_event_reported_)
        command = geometry_msgs::msg::Twist();
    }
    else
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_RUNNING);
    cmd_vel_pub_->publish(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<scan_planner_msgs::msg::ExecutionState>::SharedPtr execution_state_pub_;
  rclcpp::Publisher<scan_planner_msgs::msg::TrajectoryAck>::SharedPtr trajectory_ack_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heading_stalled_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_event_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::ExecutionCommand>::SharedPtr execution_command_sub_;
  rclcpp::Subscription<scan_planner_msgs::msg::LocomotionState>::SharedPtr locomotion_state_sub_;
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
  bool fixed_trajectory_yaw_{false};
  double trajectory_yaw_{0.0};
  uint8_t active_execution_mode_{
      scan_planner_msgs::msg::Bspline::MODE_NORMAL};
  double active_max_vx_{0.0};
  double active_min_forward_command_{0.0};
  double active_min_forward_effective_speed_{0.0};
  double active_max_vy_{0.0};
  double active_max_vyaw_{0.0};
  double active_max_tracking_correction_{0.0};
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  std::uint64_t active_request_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_vel_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_locomotion_state_time_{0, 0, RCL_ROS_TIME};
  bool locomotion_actuation_ready_{false};
  rclcpp::Time heading_freeze_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time heading_last_progress_{0, 0, RCL_ROS_TIME};
  bool heading_frozen_{false};
  bool heading_stall_reported_{false};
  bool terminal_event_reported_{false};
  bool spatial_terminal_reached_{false};
  double terminal_best_error_{std::numeric_limits<double>::infinity()};
  rclcpp::Time terminal_last_progress_time_{0, 0, RCL_ROS_TIME};
  uint8_t last_execution_state_{std::numeric_limits<uint8_t>::max()};
  rclcpp::Time last_execution_state_publish_time_{0, 0, RCL_ROS_TIME};
  double heading_best_error_{std::numeric_limits<double>::infinity()};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double heading_stall_timeout_, heading_progress_epsilon_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double handoff_search_time_, handoff_sample_dt_, max_handoff_error_;
  double min_moving_handoff_error_, handoff_reaction_time_;
  double handoff_max_deceleration_;
  double max_handoff_yaw_error_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
