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
#include "plan_manage/local_task_planner.h"
#include "plan_manage/boundary_escape.h"
#include "plan_manage/task_protocol.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    heading_release_threshold_ = declare_parameter<double>("heading_release_threshold", 0.6);
    heading_stall_timeout_ = declare_parameter<double>("heading_stall_timeout", 3.0);
    heading_progress_epsilon_ = declare_parameter<double>("heading_progress_epsilon", 0.05);
    motion_stall_timeout_ = declare_parameter<double>("motion_stall_timeout", 3.0);
    motion_progress_epsilon_ = declare_parameter<double>("motion_progress_epsilon", 0.05);
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
    // A small residual yaw after a certified handoff may be absorbed by the
    // tracker while translation continues, instead of forcing an in-place turn.
    // Zero disables the exception and restores the original align-then-connect
    // path.
    handoff_moving_alignment_tolerance_ = declare_parameter<double>(
        "handoff_moving_alignment_tolerance", 0.20);
    handoff_moving_speed_limit_ = declare_parameter<double>(
        "handoff_moving_speed_limit", 0.25);
    if (handoff_search_time_ < 0.0 || handoff_sample_dt_ <= 0.0 ||
        max_handoff_error_ <= 0.0 || min_moving_handoff_error_ < 0.0 ||
        min_moving_handoff_error_ > max_handoff_error_ ||
        handoff_reaction_time_ < 0.0 || handoff_max_deceleration_ <= 0.0 ||
        max_handoff_yaw_error_ <= 0.0 ||
        !std::isfinite(heading_error_threshold_) || heading_error_threshold_ <= 0.0 ||
        !std::isfinite(heading_release_threshold_) || heading_release_threshold_ < 0.0 ||
        heading_release_threshold_ >= heading_error_threshold_ ||
        heading_stall_timeout_ <= 0.0 ||
        heading_progress_epsilon_ <= 0.0 ||
        !std::isfinite(motion_stall_timeout_) || motion_stall_timeout_ <= 0.0 ||
        !std::isfinite(motion_progress_epsilon_) || motion_progress_epsilon_ <= 0.0)
      throw std::runtime_error(
          "handoff parameters must have a non-negative search time and positive sample interval/error limit");
    if (!std::isfinite(handoff_moving_alignment_tolerance_) ||
        handoff_moving_alignment_tolerance_ < 0.0 ||
        handoff_moving_alignment_tolerance_ > kMaxMovingAlignmentTolerance ||
        !std::isfinite(handoff_moving_speed_limit_) ||
        handoff_moving_speed_limit_ < 0.0 ||
        handoff_moving_speed_limit_ > kMaxMovingAlignmentSpeedLimit)
      throw std::runtime_error(
          "handoff moving-alignment tolerance must be finite in [0, 0.35] rad and the speed limit finite in [0, 1.0] m/s");

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
  // Bounds for the optional moving-alignment exception. A residual heading
  // beyond ~20 deg is no longer a small tracking correction and must keep the
  // certified in-place turn; 1.0 m/s is the hard sanity ceiling for measured
  // planar speed, independent of a possibly misconfigured max_vx.
  static constexpr double kMaxMovingAlignmentTolerance = 0.35;
  static constexpr double kMaxMovingAlignmentSpeedLimit = 1.0;

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
    ack.task_id = candidate.task_id;
    ack.task_version = candidate.task_version;
    ack.candidate_id = candidate.candidate_id;
    ack.trajectory_id = candidate.traj_id;
    ack.status = status;
    ack.reason = reason;
    ack.matched_time = matched_time;
    ack.position_error = position_error;
    ack.yaw_error = yaw_error;
    trajectory_ack_pub_->publish(ack);
  }

  double candidateDesiredYaw(const std::vector<UniformBspline>& curve,
      double duration, double t_cur, bool fixed_yaw, double body_yaw,
      bool measured_progress) const
  {
    if (fixed_yaw) return body_yaw;
    Eigen::Vector3d anchor=curve[0].evaluateDeBoorT(t_cur);
    if(measured_progress) anchor=odom_pos_;
    // At a zero-velocity endpoint, a numerical derivative is not a new
    // steering objective. Retain heading instead of turning toward noise.
    return trajectorySteeringYaw(t_cur,duration,anchor,odom_yaw_,
        [&](double t) { return curve[0].evaluateDeBoorT(t); });
  }

  void publishStop(double yaw_rate = 0.0)
  {
    physical_progress_.reset();
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
    msg.task_id = active_task_id_;
    msg.task_version = active_task_version_;
    msg.trajectory_id = traj_id_;
    msg.state = state;
    msg.execution_time = exec_time_;
    msg.duration = traj_duration_;
    msg.terminal_error = terminal_error;
    msg.reason = reason;
    execution_state_pub_->publish(msg);
    if(last_execution_state_!=state)
      RCLCPP_INFO(get_logger(),"[CONTROLLER_EXECUTION_STATE] task=%llu version=%llu request=%llu trajectory=%lld state=%u reason=%s",
          static_cast<unsigned long long>(active_task_id_),static_cast<unsigned long long>(active_task_version_),
          static_cast<unsigned long long>(active_request_id_),static_cast<long long>(traj_id_),static_cast<unsigned>(state),reason.c_str());
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
    alignment_yaw_bias_ = 0.0;
    heading_stall_reported_ = false;
    heading_best_error_ = std::numeric_limits<double>::infinity();
    if (publish_clear && had_freeze_state)
      publishHeadingStalled(false);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (!taskCandidateIsLive({msg->task_id, msg->task_version}, retired_task_))
    {
      publishTrajectoryAck(*msg,
          scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
          "TASK_RETIRED_OR_INVALID");
      return;
    }
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
            scan_planner_msgs::msg::Bspline::MODE_FORWARD_TURN_STAGE ||
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
    if (!std::isfinite(msg->terminal_speed_limit) || msg->terminal_speed_limit<0. ||
        (msg->terminal_speed_limit>0. &&
         (!std::isfinite(msg->terminal_braking_deceleration) || msg->terminal_braking_deceleration<=0. ||
          !std::isfinite(msg->terminal_command_latency) || msg->terminal_command_latency<0. ||
          msg->terminal_speed_limit>max_vx_))) {
      publishTrajectoryAck(*msg,scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
                           "INVALID_TERMINAL_SPEED_PROFILE");return;
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

    if(msg->execution_mode==scan_planner_msgs::msg::Bspline::MODE_FORWARD_TURN_STAGE &&
        (!candidate_fixed_yaw || !msg->handoff_certified || msg->min_forward_command<=0. ||
         !std::isfinite(msg->forward_stage_stop_latency) || msg->forward_stage_stop_latency<=0. ||
         !std::isfinite(msg->forward_stage_deceleration) || msg->forward_stage_deceleration<=0.)) {
      publishTrajectoryAck(*msg,scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
                           "FORWARD_STAGE_CERTIFICATE_INVALID");return;
    }
    std::vector<Eigen::Vector3d> control_points;
    control_points.reserve(points.cols());
    for (Eigen::Index column = 0; column < points.cols(); ++column)
      control_points.push_back(points.col(column));
    if (trajectorySamplesAreStationary(control_points, 1e-4))
    {
      active_request_id_ = msg->request_id;
      active_task_id_ = msg->task_id;
      active_task_version_ = msg->task_version;
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
          *msg, msg->task_id ? scan_planner_msgs::msg::TrajectoryAck::STATUS_HOLD_INSTALLED
                            : scan_planner_msgs::msg::TrajectoryAck::STATUS_ACCEPTED,
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
    const double turn_position_allowance=msg->handoff_turn_position_allowance==0.
        ? kHandoffPositionAllowance : msg->handoff_turn_position_allowance;
    const double alignment_tolerance=msg->handoff_alignment_tolerance==0.
        ? kHandoffAlignmentTolerance : msg->handoff_alignment_tolerance;
    if(!validCertifiedAlignmentTolerance(alignment_tolerance) ||
        (candidate_fixed_yaw && alignment_tolerance>kHandoffAlignmentTolerance) ||
        (!msg->handoff_certified && alignment_tolerance>kHandoffAlignmentTolerance)) {
      publishTrajectoryAck(*msg,scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
                           "HANDOFF_CERTIFICATE_YAW_BAND_INVALID");
      return;
    }
    if(!std::isfinite(turn_position_allowance) || turn_position_allowance<kHandoffPositionAllowance ||
        turn_position_allowance>kHandoffTurnPositionAllowance ||
        (candidate_fixed_yaw && turn_position_allowance>kHandoffPositionAllowance)) {
      publishTrajectoryAck(*msg,scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
                           "HANDOFF_CERTIFICATE_TURN_ENVELOPE_INVALID");
      return;
    }
    if (msg->task_id && !have_odom_) {
      publishTrajectoryAck(*msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
                           "HANDOFF_STATE_UNAVAILABLE");
      return;
    }
    if (have_odom_)
    {
      if (msg->task_id || msg->handoff_certified)
      {
        const Eigen::Vector3d certified_position(msg->handoff_position.x,
            msg->handoff_position.y, msg->handoff_position.z);
        // Validate time before evaluating the curve, including malformed input.
        const double safe_time = std::isfinite(msg->handoff_matched_time)
            ? std::clamp(msg->handoff_matched_time, 0.0, candidate_duration) : 0.0;
        const double target_yaw = candidateDesiredYaw(candidate, candidate_duration,
            safe_time, candidate_fixed_yaw, candidate_body_yaw,
            msg->use_measured_progress);
        const char* failure = handoffCertificateFailure(msg->handoff_certified,
            msg->handoff_matched_time, candidate_duration, certified_position,
            odom_pos_, msg->handoff_body_yaw, odom_yaw_,
            msg->handoff_target_yaw, target_yaw);
        if (failure) {
          publishTrajectoryAck(*msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED, failure);
          return;  // Preserve the predecessor, its command and execution clock.
        }
        matched_time = msg->handoff_matched_time;
        start_error = (candidate[0].evaluateDeBoorT(0).head<2>()-odom_pos_.head<2>()).norm();
        matched_error = (candidate[0].evaluateDeBoorT(matched_time).head<2>()-odom_pos_.head<2>()).norm();
      }
      else
      {
      // Legacy non-task messages retain independent matching.
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
      }
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

      // Certification, ACK and command execution use the same forward heading.
      // Near rest we may align in place; a moving discontinuity is rejected
      // before installing the candidate, preserving the old execution state.
      matched_yaw_error = std::abs(normalizeAngle(candidateDesiredYaw(
          candidate, candidate_duration, matched_time, candidate_fixed_yaw,
          candidate_body_yaw, msg->use_measured_progress) - odom_yaw_));
      if (msg->rolling_task_handoff) {
        if(handoff_alignment_pending_) {
          publishTrajectoryAck(*msg,scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
              "HANDOFF_ROLLING_ALIGNMENT_PENDING",matched_time,matched_error,matched_yaw_error);
          return; // Preserve predecessor ownership; Local may request confirmed rest.
        }
        const Eigen::Vector3d candidate_velocity = candidate[1].evaluateDeBoorT(matched_time);
        // The spline derivative is world-frame; the odometry twist is
        // body-frame. Comparing them directly makes the implied velocity jump
        // grow with yaw (~0.7 m/s at 90 deg for a 0.5 m/s handoff), so a
        // continuous successor is rejected purely because of its heading.
        const Eigen::Vector2d odom_world_velocity = odom_world_velocity_.head<2>();
        // Same reaction/deceleration contract as the position gate. Bound
        // velocity change before installation, including nearly-rest cases.
        const double velocity_limit = handoff_max_deceleration_ * handoff_reaction_time_;
        const bool heading_jump = matched_yaw_error >
            std::min(max_handoff_yaw_error_,heading_release_threshold_);
        if (heading_jump ||
            (candidate_velocity.head<2>()-odom_world_velocity).norm() > velocity_limit) {
          publishTrajectoryAck(*msg, scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
              heading_jump ? "HANDOFF_ROLLING_HEADING_ERROR" : "HANDOFF_ROLLING_VELOCITY_ERROR",
              matched_time,matched_error,matched_yaw_error);
          return;
        }
      }
      if (matched_yaw_error > max_handoff_yaw_error_ &&
          handoff_speed > 0.05)
      {
        publishTrajectoryAck(*msg,
            scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
            "HANDOFF_MOVING_HEADING_ERROR", matched_time, matched_error,
            matched_yaw_error);
        return;
      }
      if (matched_yaw_error > max_handoff_yaw_error_)
      {
        RCLCPP_WARN(
                get_logger(),
                "[TRAJECTORY_HANDOFF_HEADING_ALIGNMENT] trajectory=%lld matched_yaw_error=%.3frad threshold=%.3frad; accepting position-continuous candidate and freezing translation until aligned",
                static_cast<long long>(msg->traj_id), matched_yaw_error,
                max_handoff_yaw_error_);
      }
    }

    const bool same_execution_context = receive_traj_ &&
        ((msg->task_id != 0 && msg->task_version != 0 &&
          active_task_id_ == msg->task_id &&
          active_task_version_ == msg->task_version) ||
         (msg->task_id == 0 && active_task_id_ == 0 &&
          active_request_id_ == msg->request_id));
    const bool resuming_hold = soft_hold_;
    const bool unfinished_alignment=handoff_alignment_pending_ ||
        handoff_alignment_deviated_ || uncertified_turn_latched_;
    if(msg->handoff_certified && candidate_fixed_yaw && unfinished_alignment &&
        (odom_vel_.head<2>().norm()>.05 || !std::isfinite(odom_yaw_rate_) ||
         std::abs(odom_yaw_rate_)>.10)) {
      publishTrajectoryAck(*msg,scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED,
          "HANDOFF_FIXED_YAW_REQUIRES_STOP",matched_time,matched_error,matched_yaw_error);
      return;
    }
    if (msg->task_id && active_task_id_ &&
        std::make_pair(msg->task_id,msg->task_version) >
        std::make_pair(active_task_id_,active_task_version_))
      // Fence the predecessor atomically with installation, not on a later
      // ROS command: a delayed old spline must never undo a successful swap.
      retired_task_ = std::max(retired_task_,
          std::make_pair(active_task_id_,active_task_version_));
    traj_ = std::move(candidate);
    traj_duration_ = candidate_duration;
    traj_id_ = msg->traj_id;
    active_request_id_ = msg->request_id;
    active_task_id_ = msg->task_id;
    active_task_version_ = msg->task_version;
    fixed_trajectory_yaw_ = candidate_fixed_yaw;
    trajectory_yaw_ = candidate_body_yaw;
    active_execution_mode_ = msg->execution_mode;
    terminal_speed_limit_=msg->terminal_speed_limit;
    terminal_deceleration_=msg->terminal_braking_deceleration;
    terminal_latency_=msg->terminal_command_latency;
    last_turn_certificate_stamp_=0;
    forward_stage_braking_=false;forward_stage_failure_.clear();forward_stage_rest_gate_={};
    forward_stage_installed_at_=now();forward_stage_progress_.reset();
    forward_stage_stop_latency_=msg->forward_stage_stop_latency;
    forward_stage_deceleration_=msg->forward_stage_deceleration;
    active_max_vx_ = candidate_max_vx;
    active_min_forward_command_ = msg->min_forward_command;
    active_measured_progress_ = msg->use_measured_progress;
    active_min_forward_effective_speed_ = msg->min_forward_effective_speed;
    active_max_vy_ = candidate_max_vy;
    active_max_vyaw_ = candidate_max_vyaw;
    active_max_tracking_correction_ =
        candidate_max_tracking_correction;
    exec_time_ = matched_time;
    last_update_time_ = now();
    receive_traj_ = true;
    terminal_event_reported_ = false;
    if (!same_execution_context || resuming_hold)
    {
      motion_stalled_ = false;
      physical_progress_.reset();
    }
    terminal_capture_latched_ = false;
    terminal_rest_gate_={};terminal_capture_started_=now();
    terminal_best_error_ = std::numeric_limits<double>::infinity();
    terminal_last_progress_time_ = now();
    resetHeadingFreezeState();
    handoff_alignment_pending_=handoffNeedsHeadingAlignment(msg->handoff_certified,
        candidate_fixed_yaw,unfinished_alignment,odom_vel_.head<2>().norm(),
        matched_yaw_error,heading_release_threshold_);
    const auto install_time=now();
    const bool measured_rest=measured_rest_gate_.settled_samples>=3 &&
        measured_rest_gate_.last_pose_stamp==odom_stamp_ns_ &&
        odom_stamp_ns_>0 && install_time.nanoseconds()>=odom_stamp_ns_ &&
        install_time.nanoseconds()-odom_stamp_ns_<=250000000LL &&
        odom_vel_.head<2>().norm()<=.05 &&
        std::isfinite(odom_yaw_rate_) && std::abs(odom_yaw_rate_)<=.10;
    handoff_alignment_started_=handoff_alignment_pending_ && measured_rest;
    // Reuse fresh, measured rest across candidate installation. Only a
    // fixed-yaw departure with no such proof waits; normal moving splines
    // keep rolling. Residual angular motion cannot count as rest.
    handoff_departure_wait_rest_=msg->handoff_certified && !measured_rest &&
        (candidate_fixed_yaw ||
         (odom_vel_.head<2>().norm()<=.05 &&
          (!std::isfinite(odom_yaw_rate_) || std::abs(odom_yaw_rate_)>.10)));
    handoff_certified_body_yaw_=msg->handoff_body_yaw;
    handoff_alignment_rest_gate_={};
    handoff_alignment_wait_started_=now();
    handoff_alignment_deviated_ = false;
    uncertified_turn_latched_ = false;
    lateral_recovery_deviated_ = false;
    lateral_stop_reason_.clear();
    lateral_capture_waiting_=false;
    lateral_capture_attempts_=0;
    lateral_capture_compensation_=0.;
    lateral_capture_rest_gate_={};
    handoff_turn_position_allowance_=turn_position_allowance;
    handoff_alignment_tolerance_=alignment_tolerance;
    handoff_alignment_position_ = Eigen::Vector3d(msg->handoff_position.x,
        msg->handoff_position.y, msg->handoff_position.z);
    handoff_alignment_yaw_ = msg->handoff_target_yaw;
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
        *msg, msg->task_id
            ? (soft_hold_ ? scan_planner_msgs::msg::TrajectoryAck::STATUS_HOLD_INSTALLED
                          : scan_planner_msgs::msg::TrajectoryAck::STATUS_ACTIVATED)
            : scan_planner_msgs::msg::TrajectoryAck::STATUS_ACCEPTED,
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
    // Retirement is a task fence, not a trajectory command. It also rejects
    // delayed candidates that arrive after cancellation, before any ACK.
    if (msg->command == scan_planner_msgs::msg::ExecutionCommand::COMMAND_RETIRE_TASK)
    {
      if (msg->task_id == 0 || msg->task_version == 0) return;
      retired_task_ = std::max(retired_task_,
          std::make_pair(msg->task_id, msg->task_version));
      if (taskExecutionIsRetired({active_task_id_, active_task_version_}, retired_task_))
      {
        soft_hold_ = true;
        soft_hold_reason_ = "TASK_RETIRED";
        soft_hold_request_id_ = active_request_id_;
        soft_hold_trajectory_id_ = traj_id_;
        publishStop();
        publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_SOFT_HOLD,
                              soft_hold_reason_, 0.0, true);
      }
      scan_planner_msgs::msg::TrajectoryAck ack;
      ack.stamp = now();
      ack.task_id = msg->task_id;
      ack.task_version = msg->task_version;
      ack.status = scan_planner_msgs::msg::TrajectoryAck::STATUS_TASK_RETIRED;
      ack.reason = "TASK_RETIRED";
      trajectory_ack_pub_->publish(ack);
      return;
    }
    if (taskExecutionIsRetired({active_task_id_, active_task_version_}, retired_task_))
      return;
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
    if (msg->command == scan_planner_msgs::msg::ExecutionCommand::COMMAND_REFRESH_TURN) {
      const Eigen::Vector3d position(msg->certificate_position.x,
          msg->certificate_position.y,msg->certificate_position.z);
      const int64_t stamp=rclcpp::Time(msg->certificate_stamp).nanoseconds();
      if(msg->task_id!=active_task_id_ || msg->task_version!=active_task_version_ ||
          msg->request_id!=active_request_id_ || msg->trajectory_id!=traj_id_ ||
          msg->certificate_map_revision==0 || !position.allFinite() ||
          handoff_departure_wait_rest_ ||
          !turnRefreshAcceptable(handoff_alignment_pending_,soft_hold_,handoff_alignment_deviated_,
              now().nanoseconds(),stamp,last_turn_certificate_stamp_,
              (position-odom_pos_).head<2>().norm(),
              normalizeAngle(msg->certificate_target_yaw-handoff_alignment_yaw_),
              msg->certificate_position_allowance,handoff_turn_position_allowance_)) return;
      handoff_alignment_position_=position;
      handoff_turn_position_allowance_=msg->certificate_position_allowance;
      last_turn_certificate_stamp_=stamp;
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,
          "[TURN_CERTIFICATE_REFRESHED] request=%llu trajectory=%lld map=%llu radius=%.3fm",
          static_cast<unsigned long long>(active_request_id_),static_cast<long long>(traj_id_),
          static_cast<unsigned long long>(msg->certificate_map_revision),handoff_turn_position_allowance_);
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
      measured_rest_gate_={};
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
      measured_rest_gate_={};
      publishStop();
      return;
    }
    odom_pos_ = position;
    odom_vel_ = velocity;
    odom_world_velocity_ = rotateBodyVelocityToWorld(orientation, velocity);
    odom_yaw_ = yaw;
    odom_yaw_rate_=msg->twist.twist.angular.z;
    const int64_t next_stamp_ns=rclcpp::Time(msg->header.stamp).nanoseconds();
    if(next_stamp_ns<=odom_stamp_ns_ ||
        (odom_stamp_ns_>0 && next_stamp_ns-odom_stamp_ns_>250000000LL))
      measured_rest_gate_={};
    odom_stamp_ns_=next_stamp_ns;
    const int64_t now_ns=now().nanoseconds();
    const bool fresh=odom_stamp_ns_>0 && now_ns>=odom_stamp_ns_ &&
        now_ns-odom_stamp_ns_<=250000000LL;
    measured_rest_gate_.observe(true,fresh,odom_vel_.head<2>().norm(),
                                odom_yaw_rate_,odom_stamp_ns_);
    have_odom_ = true;
  }

  void latchLateralRecoveryStop(const char* reason,
      const Eigen::Vector2d& start,const Eigen::Vector2d& goal,double yaw_error)
  {
    const Eigen::Vector2d direction=(goal-start).normalized();
    const double progress=(odom_pos_.head<2>()-start).dot(direction);
    const double longitudinal=(odom_pos_.head<2>()-start).dot(
        Eigen::Vector2d(std::cos(trajectory_yaw_),std::sin(trajectory_yaw_)));
    lateral_recovery_deviated_=true;
    lateral_stop_reason_=reason;
    RCLCPP_WARN(get_logger(),
        "[LATERAL_RECOVERY_STOP] reason=%s request=%llu trajectory=%lld phase=%s stop_checks=%d planned=%.3fm progress=%.3fm overshoot=%.3fm longitudinal=%.3fm yaw_error=%.4frad speed=%.3fm/s pose=(%.3f,%.3f) start=(%.3f,%.3f) goal=(%.3f,%.3f); keeping zero command until new certified candidate",
        reason,static_cast<unsigned long long>(active_request_id_),static_cast<long long>(traj_id_),
        lateral_capture_waiting_ ? "SETTLING" : "TRANSLATING",lateral_capture_attempts_,
        (goal-start).norm(),progress,progress-(goal-start).norm(),longitudinal,yaw_error,
        odom_vel_.head<2>().norm(),odom_pos_.x(),odom_pos_.y(),start.x(),start.y(),goal.x(),goal.y());
    publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_STALLED,
                          lateral_stop_reason_,(odom_pos_.head<2>()-goal).norm(),true);
    publishStop();
    last_update_time_=now();
  }

  // Completion relinquishes the old motion command. A certified successor
  // can still install immediately; an exhausted spline cannot steer itself
  // back into execution while Local computes that successor.
  void holdTerminalCapture(const rclcpp::Time& current_time)
  {
    publishStop();last_update_time_=current_time;
    const bool fresh=odom_stamp_ns_>0 && current_time.nanoseconds()>=odom_stamp_ns_ &&
        current_time.nanoseconds()-odom_stamp_ns_<=250000000LL;
    const double elapsed=(current_time-terminal_capture_started_).seconds();
    const bool settled=terminal_rest_gate_.observe(elapsed>=.3,fresh,
        odom_vel_.head<2>().norm(),odom_yaw_rate_,odom_stamp_ns_);
    const double error=(traj_[0].evaluateDeBoorT(traj_duration_)-odom_pos_).head<2>().norm();
    if(error>finish_dist_) {
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_STALLED,
                            "TERMINAL_CAPTURE_DRIFT",error);
    } else if(settled) {
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_FINISHED,
                            "TERMINAL_REACHED",error);
      if(!terminal_event_reported_) {
        terminal_event_reported_=true;
        publishExecutionEvent("LOCAL_TRAJECTORY_FINISHED",error);
        RCLCPP_INFO(get_logger(),"[LOCAL_TRAJECTORY_FINISHED] request_id=%llu trajectory=%lld terminal_error=%.3fm; measured rest confirmed",
            static_cast<unsigned long long>(active_request_id_),static_cast<long long>(traj_id_),error);
      }
    } else if(elapsed>5.) {
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_STALLED,
                            "TERMINAL_STOP_UNCONFIRMED",error);
    } else {
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_RUNNING,
                            "TERMINAL_SETTLING",error);
    }
  }

  void failForwardStage(const char* reason,double error,const rclcpp::Time& current_time)
  {
    if(forward_stage_failure_.empty()) {
      forward_stage_failure_=reason;
      RCLCPP_WARN(get_logger(),"[FORWARD_STAGE_STOP] request=%llu trajectory=%lld reason=%s error=%.3fm pose=(%.3f,%.3f)",
          static_cast<unsigned long long>(active_request_id_),static_cast<long long>(traj_id_),reason,error,odom_pos_.x(),odom_pos_.y());
    }
    publishStop();last_update_time_=current_time;
    publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_STALLED,
                          forward_stage_failure_,error);
  }

  bool controlForwardStage(const rclcpp::Time& current_time)
  {
    const auto start=traj_[0].evaluateDeBoorT(0.).head<2>().eval();
    const auto goal=traj_[0].evaluateDeBoorT(traj_duration_).head<2>().eval();
    const auto state=forwardStageState(start,goal,odom_pos_.head<2>(),trajectory_yaw_,
        odom_yaw_,odom_vel_.head<2>().norm(),forward_stage_stop_latency_,forward_stage_deceleration_);
    const bool fresh=odom_stamp_ns_>0 && current_time.nanoseconds()>=odom_stamp_ns_ &&
        current_time.nanoseconds()-odom_stamp_ns_<=250000000LL;
    if(!forward_stage_failure_.empty() || state.failure || !fresh) {
      failForwardStage(state.failure ? state.failure : "FORWARD_STAGE_ODOMETRY_STALE",state.error,current_time);
      return true;
    }
    if(forward_stage_braking_) {
      publishStop();last_update_time_=current_time;
      const double elapsed=(current_time-forward_stage_brake_started_).seconds();
      if(forward_stage_rest_gate_.observe(elapsed>=.3,fresh,odom_vel_.head<2>().norm(),
                                          odom_yaw_rate_,odom_stamp_ns_)) {
        if(state.error>kForwardStageCaptureRadius) {
          failForwardStage("FORWARD_STAGE_STOP_SHORT",state.error,current_time);return true;
        }
        exec_time_=traj_duration_;
        publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_FINISHED,
                              "FORWARD_STAGE_CAPTURED",state.error);
        if(!terminal_event_reported_) {
          terminal_event_reported_=true;publishExecutionEvent("LOCAL_TRAJECTORY_FINISHED",state.error);
        }
      } else if(elapsed>5.) failForwardStage("FORWARD_STAGE_STOP_UNCONFIRMED",state.error,current_time);
      else publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_RUNNING,
                                 "FORWARD_STAGE_BRAKING",state.error);
      return true;
    }
    if((current_time-forward_stage_installed_at_).seconds()>
          std::max(5.,4.*(goal-start).norm()/active_min_forward_effective_speed_)+5. ||
        forward_stage_progress_.stalled(current_time.seconds(),Eigen::Vector2d(state.progress,0.),
            true,motion_stall_timeout_,motion_progress_epsilon_)) {
      failForwardStage("FORWARD_STAGE_NO_PROGRESS",state.error,current_time);return true;
    }
    if(state.brake) {
      forward_stage_braking_=true;forward_stage_brake_started_=current_time;forward_stage_rest_gate_={};
      RCLCPP_INFO(get_logger(),"[FORWARD_STAGE_BRAKING] request=%llu trajectory=%lld remaining=%.3fm speed=%.3fm/s latency=%.3fs deceleration=%.3fm/s2",
          static_cast<unsigned long long>(active_request_id_),static_cast<long long>(traj_id_),state.remaining,odom_vel_.head<2>().norm(),
          forward_stage_stop_latency_,forward_stage_deceleration_);
      publishStop();last_update_time_=current_time;
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_RUNNING,"FORWARD_STAGE_BRAKING",state.error);
      return true;
    }
    return false;
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
      physical_progress_.reset();
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
    if (handoff_alignment_pending_ &&
        (odom_pos_-handoff_alignment_position_).head<2>().norm() > handoff_turn_position_allowance_)
      handoff_alignment_deviated_ = true;
    if(handoff_departure_wait_rest_ &&
        ((odom_pos_-handoff_alignment_position_).head<2>().norm()>handoff_turn_position_allowance_ ||
         std::abs(normalizeAngle(odom_yaw_-handoff_certified_body_yaw_))>kHandoffYawAllowance))
      handoff_alignment_deviated_=true;
    if (handoff_alignment_deviated_ || uncertified_turn_latched_ || lateral_recovery_deviated_) {
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_STALLED,
                            handoff_alignment_deviated_ ? (handoff_departure_wait_rest_ ? "HANDOFF_REST_STATE_CHANGED" : "HANDOFF_ALIGNMENT_DEVIATED") :
                                (lateral_recovery_deviated_ ? lateral_stop_reason_ :
                                    "UNCERTIFIED_TURN_REQUIRED"),
                            lateral_recovery_deviated_
                              ? (odom_pos_-traj_[0].evaluateDeBoorT(traj_duration_)).head<2>().norm()
                              : (odom_pos_-handoff_alignment_position_).head<2>().norm());
      // Lateral terminal_error is endpoint distance, never start displacement.
      publishStop();
      last_update_time_ = current_time;
      return;  // Only a newly certified candidate may release this latch.
    }
    if(handoff_departure_wait_rest_ || (handoff_alignment_pending_ && !handoff_alignment_started_)) {
      const bool fresh=odom_stamp_ns_>0 && current_time.nanoseconds()>=odom_stamp_ns_ &&
          current_time.nanoseconds()-odom_stamp_ns_<=250000000LL;
      if(!handoff_alignment_rest_gate_.observe(true,fresh,odom_vel_.head<2>().norm(),
                                              odom_yaw_rate_,odom_stamp_ns_)) {
        const bool timeout=(current_time-handoff_alignment_wait_started_).seconds()>5.;
        publishExecutionState(timeout ? scan_planner_msgs::msg::ExecutionState::STATE_STALLED :
            scan_planner_msgs::msg::ExecutionState::STATE_HEADING_FROZEN,
            timeout ? "HANDOFF_REST_UNCONFIRMED" : "HEADING_ALIGNMENT_WAIT_REST");
        publishStop();last_update_time_=current_time;return;
      }
      handoff_departure_wait_rest_=false;
      handoff_alignment_started_=true;handoff_alignment_rest_gate_={};
    }
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const bool calibrated_forward = active_min_forward_command_ > 0.0;
    const bool measured_progress = active_measured_progress_;
    if(active_execution_mode_==scan_planner_msgs::msg::Bspline::MODE_FORWARD_TURN_STAGE &&
        controlForwardStage(current_time)) return;
    if (terminal_capture_latched_)
    {
      holdTerminalCapture(current_time);
      return;
    }
    double spatial_target_time = exec_time_;
    if (measured_progress && !handoff_alignment_pending_)
    {
      // Match actual progress, not a reference clock that advances while the
      // calibrated gait is slower than its command. Never chase a past point.
      exec_time_=measuredTrajectoryProgress(exec_time_,traj_duration_,odom_pos_,
          [&](double t) {return Eigen::Vector3d(traj_[0].evaluateDeBoorT(t));});
      spatial_target_time = forwardGaitLookaheadTime(exec_time_, traj_duration_,
          [&](double t) { return traj_[0].evaluateDeBoorT(t); });
      const double endpoint_error = (traj_[0].evaluateDeBoorT(traj_duration_) - odom_pos_).head<2>().norm();
      // Fixed-yaw forward staging must reach its safe turning position;
      // the ordinary 15 cm arrival band may still lie in the narrow approach.
      const double capture_distance=fixed_trajectory_yaw_ ? .03 : finish_dist_;
      if (active_execution_mode_!=scan_planner_msgs::msg::Bspline::MODE_FORWARD_TURN_STAGE &&
          spatial_target_time >= traj_duration_ && endpoint_error < capture_distance)
      {
        exec_time_ = traj_duration_;
        terminal_capture_latched_ = true;
        terminal_capture_started_=current_time;terminal_rest_gate_={};
        holdTerminalCapture(current_time);
        return;
      }
    }
    const double t_eval = std::min(measured_progress ? spatial_target_time : exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    // During initial alignment use the certified target, not a fresh heading
    // from pose drift or a later spline prefix. Progress remains frozen.
    const double desired_yaw = handoff_alignment_pending_ ? handoff_alignment_yaw_ :
        candidateDesiredYaw(traj_, traj_duration_,
            measured_progress ? exec_time_ : t_eval, fixed_trajectory_yaw_,
            trajectory_yaw_, measured_progress);
    const double yaw_error = normalizeAngle(desired_yaw - odom_yaw_);
    std_msgs::msg::Float64 heading_error_msg;
    heading_error_msg.data = yaw_error;
    heading_error_pub_->publish(heading_error_msg);
    const double yaw_command = std::clamp(
        kp_yaw_ * yaw_error, -active_max_vyaw_, active_max_vyaw_);
    // A stopped handoff must complete its certified rotation before driving.
    // Normal tracking hysteresis is too loose for this align-then-connect contract.
    // Exception: with a small residual error and bounded measured speed the
    // tracker may translate while correcting heading. The flag is only
    // evaluated while a certified handoff is still pending; fixed-yaw turns
    // never reach here (handoffNeedsHeadingAlignment rejects them).
    // odom_vel_ is the body-frame twist; only its planar norm is used, which
    // equals the norm of the world-frame velocity, so no frame conversion is
    // needed for this gate.
    const double handoff_alignment_speed = odom_vel_.head<2>().norm();
    const bool handoff_moving_alignment = handoff_alignment_pending_ &&
        handoff_moving_alignment_tolerance_ > 0.0 &&
        handoff_moving_speed_limit_ > 0.0 &&
        std::isfinite(yaw_error) && std::isfinite(handoff_alignment_speed) &&
        std::abs(yaw_error) <= handoff_moving_alignment_tolerance_ &&
        handoff_alignment_speed <= handoff_moving_speed_limit_;
    if (handoff_moving_alignment)
    {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[HANDOFF_MOVING_ALIGNMENT] request_id=%llu trajectory=%lld error=%.3frad tolerance=%.3frad speed=%.3fm/s limit=%.3fm/s; continuing translation while tracking corrects heading",
          static_cast<unsigned long long>(active_request_id_),
          static_cast<long long>(traj_id_), std::abs(yaw_error),
          handoff_moving_alignment_tolerance_, handoff_alignment_speed,
          handoff_moving_speed_limit_);
    }
    const bool freeze=handoff_alignment_pending_
        ? (!handoff_moving_alignment &&
           std::abs(yaw_error)>handoff_alignment_tolerance_)
        : headingFreezeRequired(heading_frozen_,yaw_error,
                                heading_error_threshold_,heading_release_threshold_);
    if (freeze)
    {
      if(handoff_alignment_pending_) handoff_alignment_rest_gate_={};
      // The candidate certificate authorizes its initial align-then-connect
      // only. A new in-place turn during execution needs a fresh Local grant,
      // even when the old spline itself remains position-continuous.
      if (active_task_id_ && !handoff_alignment_pending_) {
        uncertified_turn_latched_ = true;
        publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_STALLED,
                              "UNCERTIFIED_TURN_REQUIRED");
        publishStop();
        last_update_time_ = current_time;
        return;
      }
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
                    handoff_alignment_pending_ ? handoff_alignment_tolerance_ : heading_error_threshold_);
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
      physical_progress_.reset();
      // A proportional command can stall below the real gait's effective yaw
      // response. Ramp a bounded bias only in a certified initial turn with
      // measured low angular speed. Keep the certificate, timeout and yaw cap.
      // Do not carry a bias through target crossing or into path tracking.
      double alignment_command = yaw_command;
      if (handoff_alignment_pending_ && active_task_id_) {
        if (alignment_yaw_bias_ * yaw_error < 0.0) alignment_yaw_bias_ = 0.0;
        if (stalled_for >= .30 && std::abs(odom_yaw_rate_) < .08)
          alignment_yaw_bias_ = std::copysign(
              std::min(.35, std::abs(alignment_yaw_bias_) + .25 * std::clamp(dt, 0.0, .05)),
              yaw_error);
        alignment_command = std::clamp(yaw_command + alignment_yaw_bias_,
                                        -active_max_vyaw_, active_max_vyaw_);
      }
      publishStop(alignment_command);
      last_update_time_ = current_time;
      return;
    }

    alignment_yaw_bias_ = 0.0; // Captured: zero command and measured settling.
    // The moving-alignment exception intentionally skips the measured-rest
    // settle: translation must continue while the tracker removes the small
    // residual yaw. Every other pending handoff keeps the rest contract.
    if(handoff_alignment_pending_ && !handoff_moving_alignment) {
      const bool fresh=odom_stamp_ns_>0 && current_time.nanoseconds()>=odom_stamp_ns_ &&
          current_time.nanoseconds()-odom_stamp_ns_<=250000000LL;
      if(!handoff_alignment_rest_gate_.observe(true,fresh,odom_vel_.head<2>().norm(),
                                               odom_yaw_rate_,odom_stamp_ns_)) {
        publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_HEADING_FROZEN,
                              "HEADING_ALIGNMENT_SETTLING");
        publishStop();last_update_time_=current_time;return;
      }
    }
    handoff_alignment_pending_ = false;
    handoff_alignment_started_=false;handoff_alignment_rest_gate_={};
    resetHeadingFreezeState();
    if (!measured_progress)
      exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    // Ordinary measured-progress tracking needs an actual forward target too;
    // sampling zero-speed departure at the nearest point would never start.
    const double tracking_time=measured_progress ? t_eval : exec_time_;
    pos_des = traj_[0].evaluateDeBoorT(tracking_time);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(tracking_time);
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
    if (active_execution_mode_ == scan_planner_msgs::msg::Bspline::MODE_RECOVERY_PRIMITIVE &&
        fixed_trajectory_yaw_ && !calibrated_forward)
    {
      // This Local-certified primitive translates left/right at fixed body
      // yaw. Do not turn its lateral feedforward into a forward command floor.
      command.linear.x = 0.0;
      // An upper bound alone left the real command near 0.06 m/s, inside
      // the legacy lateral deadband. The certified primitive runs a bounded
      // constant transverse command until measured capture, not spline time.
      const Eigen::Vector2d left(-std::sin(trajectory_yaw_),std::cos(trajectory_yaw_));
      const Eigen::Vector2d start=traj_[0].evaluateDeBoorT(0).head<2>();
      const Eigen::Vector2d goal=traj_[0].evaluateDeBoorT(traj_duration_).head<2>();
      command.linear.y = std::copysign(active_max_vy_,(goal-start).dot(left));
      const Eigen::Vector2d direction=(goal-start).normalized();
      const double overshoot=(odom_pos_.head<2>()-goal).dot(direction);
      const double cross=(odom_pos_.head<2>()-start).dot(Eigen::Vector2d(
          std::cos(trajectory_yaw_),std::sin(trajectory_yaw_)));
      if (overshoot>kLateralRecoveryCaptureExtension ||
          std::abs(cross)>kLateralRecoveryLongitudinalBound || std::abs(yaw_error)>.12) {
        const char* reason=overshoot>kLateralRecoveryCaptureExtension ?
            "LATERAL_RECOVERY_OVERSHOOT" :
            std::abs(cross)>kLateralRecoveryLongitudinalBound ?
            "LATERAL_RECOVERY_LONGITUDINAL_DRIFT" : "LATERAL_RECOVERY_HEADING_DRIFT";
        latchLateralRecoveryStop(reason,start,goal,yaw_error);
        return;  // Never reverse toward an uncertified recovery side.
      }
      const double progress=(odom_pos_.head<2>()-start).dot(direction);
      // Boundary exits carry an explicit braking model. Capture begins before
      // the endpoint using measured speed; waiting until geometric arrival
      // previously added the whole physical stopping distance as overshoot.
      const double stop_lead=terminal_deceleration_>0. && terminal_latency_>0. ?
          lateralCaptureStopLead(odom_vel_.head<2>().dot(direction),terminal_deceleration_,terminal_latency_) : 0.;
      if (!lateral_capture_waiting_ && progress+stop_lead>=(goal-start).norm()+lateral_capture_compensation_) {
        lateral_capture_waiting_=true;
        lateral_capture_stop_time_=current_time;
        lateral_capture_peak_=progress;
        ++lateral_capture_attempts_;
        lateral_capture_rest_gate_={};
      }
      if (lateral_capture_waiting_) {
        const double stopped_for=(current_time-lateral_capture_stop_time_).seconds();
        const bool fresh=odom_stamp_ns_>0 && current_time.nanoseconds()>=odom_stamp_ns_ &&
            current_time.nanoseconds()-odom_stamp_ns_<=250000000LL;
        const bool settled=lateral_capture_rest_gate_.observe(stopped_for>=.8,
            fresh && odom_vel_.head<2>().norm()<=.01,odom_vel_.head<2>().norm(),
            odom_yaw_rate_,odom_stamp_ns_);
        if (settled && lateralRecoveryReached(start,goal,odom_pos_.head<2>(),trajectory_yaw_,odom_yaw_)) {
          exec_time_=traj_duration_;
          terminal_capture_latched_=true;
          publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_FINISHED,
                                "TERMINAL_REACHED");
          publishStop();
          return;
        }
        if (stopped_for>5. || (settled && (lateral_capture_attempts_>=3 || overshoot>0.))) {
          latchLateralRecoveryStop(stopped_for>5. ?
              "LATERAL_RECOVERY_STOP_UNCONFIRMED" : "LATERAL_RECOVERY_CAPTURE_EXHAUSTED",
              start,goal,yaw_error);
          return;
        }
        if (settled) {
          // Measure this policy's settling retreat, then compensate inside the
          // Local-certified extension. Three stop checks bound the primitive.
          lateral_capture_compensation_=std::clamp(lateral_capture_peak_-progress+.01,
              0.,kLateralRecoveryCaptureExtension-.02);
          lateral_capture_waiting_=false;
          physical_progress_.reset();
        } else {
          physical_progress_.reset();
          publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_RUNNING,
                                "LATERAL_CAPTURE_SETTLING");
          publishStop();
          return;
        }
      }
    }
    else if (calibrated_forward)
    {
      // Ordinary forward-gait trajectories steer toward
      // the forward target; do not alternate tiny reverse and minimum forward.
      command.linear.x = active_min_forward_command_;
      command.linear.y = 0.0;
    }
    if(terminal_speed_limit_>0.) {
      const double remaining=(traj_[0].evaluateDeBoorT(traj_duration_)-odom_pos_).head<2>().norm();
      const double bound=terminalApproachSpeed(active_max_vx_,terminal_speed_limit_,
          remaining,terminal_deceleration_,terminal_latency_);
      const double speed=std::hypot(command.linear.x,command.linear.y);
      if(speed>bound && speed>0.) {command.linear.x*=bound/speed;command.linear.y*=bound/speed;}
    }
    const double terminal_error = pos_error.norm();
    const bool lateral_recovery=active_execution_mode_==scan_planner_msgs::msg::Bspline::MODE_RECOVERY_PRIMITIVE &&
        fixed_trajectory_yaw_ && !calibrated_forward;
    const Eigen::Vector2d recovery_start=traj_[0].evaluateDeBoorT(0).head<2>();
    const Eigen::Vector2d recovery_goal=traj_[0].evaluateDeBoorT(traj_duration_).head<2>();
    const double terminal_tolerance=lateral_recovery ?
        lateralRecoveryTolerance((recovery_goal-recovery_start).norm()) : finish_dist_;
    const bool terminal_reached=lateral_recovery ?
        lateralRecoveryReached(recovery_start,recovery_goal,odom_pos_.head<2>(),trajectory_yaw_,odom_yaw_) :
        terminal_error<terminal_tolerance;
    const double endpoint_error =
        (traj_[0].evaluateDeBoorT(traj_duration_) - odom_pos_).head<2>().norm();
    const bool execution_no_progress = physical_progress_.stalled(
            current_time.seconds(), odom_pos_.head<2>(),
            std::hypot(command.linear.x,command.linear.y) > 0.05 &&
                endpoint_error > terminal_tolerance,
            motion_stall_timeout_,motion_progress_epsilon_);
    if (execution_no_progress)
    {
      if (!motion_stalled_)
      {
        motion_stalled_ = true;
        publishExecutionEvent("LOCAL_TRAJECTORY_STALLED",endpoint_error);
        RCLCPP_ERROR(get_logger(),
            "[EXECUTION_NO_PROGRESS] request_id=%llu trajectory=%lld window=%.2fs displacement<%.3fm execution_time=%.2f/%.2fs; retaining the acknowledged spline while SCAN validates replacements",
            static_cast<unsigned long long>(active_request_id_),static_cast<long long>(traj_id_),
            motion_stall_timeout_,motion_progress_epsilon_,exec_time_,traj_duration_);
      }
    }
    else
      motion_stalled_ = false;
    if (!lateral_recovery && terminal_reached && exec_time_ >= traj_duration_)
    {
      terminal_capture_latched_=true;
      terminal_capture_started_=current_time;terminal_rest_gate_={};
      holdTerminalCapture(current_time);
      return;
    }
    else if (!lateral_recovery && exec_time_ >= traj_duration_)
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
    else if (motion_stalled_)
      publishExecutionState(scan_planner_msgs::msg::ExecutionState::STATE_STALLED,
                            "EXECUTION_NO_PROGRESS",endpoint_error);
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
  bool handoff_alignment_pending_{false};
  bool handoff_alignment_started_{false};
  bool handoff_departure_wait_rest_{false};
  double handoff_certified_body_yaw_{0.};
  double handoff_turn_position_allowance_{kHandoffPositionAllowance};
  double handoff_alignment_tolerance_{kHandoffAlignmentTolerance};
  TaskTransitionRestGate handoff_alignment_rest_gate_;
  TaskTransitionRestGate measured_rest_gate_;
  rclcpp::Time handoff_alignment_wait_started_{0,0,RCL_ROS_TIME};
  bool handoff_alignment_deviated_{false};
  bool uncertified_turn_latched_{false};
  bool lateral_recovery_deviated_{false};
  std::string lateral_stop_reason_;
  bool lateral_capture_waiting_{false};
  int lateral_capture_attempts_{0};
  double lateral_capture_compensation_{0.};
  double lateral_capture_peak_{0.};
  rclcpp::Time lateral_capture_stop_time_{0,0,RCL_ROS_TIME};
  TaskTransitionRestGate lateral_capture_rest_gate_;
  int64_t odom_stamp_ns_{0};
  double odom_yaw_rate_{0.};
  Eigen::Vector3d handoff_alignment_position_{Eigen::Vector3d::Zero()};
  double handoff_alignment_yaw_{0.0};
  int64_t last_turn_certificate_stamp_{0};
  double terminal_speed_limit_{0.},terminal_deceleration_{0.},terminal_latency_{0.};
  bool fixed_trajectory_yaw_{false};
  double trajectory_yaw_{0.0};
  uint8_t active_execution_mode_{
      scan_planner_msgs::msg::Bspline::MODE_NORMAL};
  double active_max_vx_{0.0};
  double active_min_forward_command_{0.0};
  bool active_measured_progress_{false};
  double active_min_forward_effective_speed_{0.0};
  double active_max_vy_{0.0};
  double active_max_vyaw_{0.0};
  double active_max_tracking_correction_{0.0};
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  std::uint64_t active_request_id_{0};
  std::uint64_t active_task_id_{0};
  std::uint64_t active_task_version_{0};
  std::pair<std::uint64_t, std::uint64_t> retired_task_{0, 0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_world_velocity_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_locomotion_state_time_{0, 0, RCL_ROS_TIME};
  bool locomotion_actuation_ready_{false};
  rclcpp::Time heading_freeze_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time heading_last_progress_{0, 0, RCL_ROS_TIME};
  double alignment_yaw_bias_{0.0};
  bool heading_frozen_{false};
  bool heading_stall_reported_{false};
  bool terminal_event_reported_{false};
  bool motion_stalled_{false};
  PhysicalProgressWindow physical_progress_;
  double motion_stall_timeout_, motion_progress_epsilon_;
  bool terminal_capture_latched_{false};
  bool forward_stage_braking_{false};
  std::string forward_stage_failure_;
  double forward_stage_stop_latency_{0.},forward_stage_deceleration_{0.};
  rclcpp::Time forward_stage_installed_at_{0,0,RCL_ROS_TIME};
  rclcpp::Time forward_stage_brake_started_{0,0,RCL_ROS_TIME};
  TaskTransitionRestGate forward_stage_rest_gate_;
  PhysicalProgressWindow forward_stage_progress_;
  TaskTransitionRestGate terminal_rest_gate_;
  rclcpp::Time terminal_capture_started_{0,0,RCL_ROS_TIME};
  double terminal_best_error_{std::numeric_limits<double>::infinity()};
  rclcpp::Time terminal_last_progress_time_{0, 0, RCL_ROS_TIME};
  uint8_t last_execution_state_{std::numeric_limits<uint8_t>::max()};
  rclcpp::Time last_execution_state_publish_time_{0, 0, RCL_ROS_TIME};
  double heading_best_error_{std::numeric_limits<double>::infinity()};
  double time_forward_, heading_error_threshold_, heading_release_threshold_, kp_pos_, kp_yaw_;
  double heading_stall_timeout_, heading_progress_epsilon_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double handoff_search_time_, handoff_sample_dt_, max_handoff_error_;
  double min_moving_handoff_error_, handoff_reaction_time_;
  double handoff_max_deceleration_;
  double max_handoff_yaw_error_;
  double handoff_moving_alignment_tolerance_{0.0};
  double handoff_moving_speed_limit_{0.0};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
