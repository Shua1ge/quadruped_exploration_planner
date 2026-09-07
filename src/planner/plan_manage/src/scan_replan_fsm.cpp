
#include <plan_manage/scan_replan_fsm.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    emergency_path_pending_ = false;
    tracking_recovery_active_ = false;
    collision_segment_pending_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    reference_path_lookahead_ = load_parameter<double>(node_, "fsm.reference_path_lookahead", 3.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    rolling_replan_retry_period_ = load_parameter<double>(
        node_, "fsm.rolling_replan_retry_period", 0.25);
    if (rolling_replan_retry_period_ <= 0.0)
      throw std::runtime_error("fsm.rolling_replan_retry_period must be positive");
    rolling_replan_max_start_error_ = load_parameter<double>(
        node_, "fsm.rolling_replan_max_start_error", 0.30);
    if (rolling_replan_max_start_error_ <= 0.0)
      throw std::runtime_error("fsm.rolling_replan_max_start_error must be positive");
    goal_tolerance_ = load_parameter<double>(node_, "fsm.goal_tolerance", 0.25);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 5);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    planning_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    map_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    map_visualization_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    safety_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(
        node_, visualization_, map_callback_group_,
        map_visualization_callback_group_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this),
                                           planning_callback_group_);
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this),
                                             safety_callback_group_);
    rclcpp::SubscriptionOptions planning_options;
    planning_options.callback_group = planning_callback_group_;
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1),
        planning_options);
    rclcpp::SubscriptionOptions safety_options;
    safety_options.callback_group = safety_callback_group_;
    safety_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::safetyOdometryCallback, this, std::placeholders::_1),
        safety_options);
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1),
        safety_options);

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());
    blocked_segment_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "planning/blocked_segment", 10);
    emergency_stop_pub_ = node_->create_publisher<std_msgs::msg::Bool>("planning/emergency_stop", 10);
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("planning/status", 10);
    publishStatus("IDLE");

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1),
          planning_options);
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1,
          std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1),
          planning_options);
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      next_rolling_replan_attempt_ns_ = 0;
      if (safety_stop_active_.exchange(false))
        RCLCPP_INFO(node_->get_logger(),
                    "[SAFETY_RELEASE] source=new_manual_goal; planning may resume");

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    if (reference_path_active_ && reference_path_.size() >= 2)
    {
      const Eigen::Vector3d final_pt = reference_path_.back();
      const Eigen::Vector3d final_prev = reference_path_[reference_path_.size() - 2];
      if (map->getInflateOccupancy(
              final_pt, estimateYawFromSegment(final_prev, final_pt)) <= 0)
        return true;

      constexpr double sample_distance = 0.05;
      const Eigen::Vector3d raw_end = end_pt_;
      for (size_t reverse_index = reference_path_.size() - 1;
           reverse_index > 0; --reverse_index)
      {
        const size_t segment_index = reverse_index - 1;
        const Eigen::Vector3d &from = reference_path_[segment_index];
        const Eigen::Vector3d &to = reference_path_[segment_index + 1];
        const Eigen::Vector3d segment = to - from;
        const double length = segment.norm();
        const int samples = std::max(1, static_cast<int>(std::ceil(length / sample_distance)));
        const double yaw = estimateYawFromSegment(from, to);

        for (int sample = samples - 1; sample >= 0; --sample)
        {
          const double ratio = static_cast<double>(sample) / samples;
          const Eigen::Vector3d candidate = from + ratio * segment;
          if (map->getInflateOccupancy(candidate, yaw) != 0)
            continue;

          end_pt_ = candidate;
          reference_path_.resize(segment_index + 1);
          if (reference_path_.empty() ||
              (reference_path_.back() - candidate).norm() > 1e-3)
            reference_path_.push_back(candidate);
          else
            reference_path_.back() = candidate;

          RCLCPP_WARN(node_->get_logger(),
                      "Reference target [%.2f, %.2f, %.2f] is occupied; "
                      "retreating on the supplied path to [%.2f, %.2f, %.2f]",
                      raw_end(0), raw_end(1), raw_end(2),
                      end_pt_(0), end_pt_(1), end_pt_(2));
          return true;
        }
      }

      RCLCPP_ERROR(node_->get_logger(),
                   "Reference target is occupied and no free point exists on the supplied path");
      return false;
    }

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    const uint64_t request_id =
        static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL
        + static_cast<uint64_t>(msg->header.stamp.nanosec);
    const uint64_t active_request_id = active_reference_request_id_.load();
    if (request_id != 0 && active_request_id != 0 && request_id <= active_request_id)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Ignoring stale initial_path request_id=%llu active_request_id=%llu",
                  static_cast<unsigned long long>(request_id),
                  static_cast<unsigned long long>(active_request_id));
      return;
    }
    active_reference_request_id_.store(request_id);
    next_rolling_replan_attempt_ns_ = 0;

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      wp(2) = pose_stamped.pose.position.z + body_height_; // Adjust for body height
      if (waypoints.empty() || (wp - waypoints.back()).norm() > 1e-3)
        waypoints.push_back(wp);
    }

    if (waypoints.size() < 2)
    {
      RCLCPP_ERROR(node_->get_logger(), "Reference path must contain at least two distinct poses");
      requestExecutionStop("INVALID_REFERENCE_PATH");
      return;
    }

    // Keep the collision-checked polyline as the global reference.  Fitting a
    // single high-order polynomial through a maze path can overshoot corners
    // and leave the A-star safety corridor before local planning even starts.
    trigger_ = true;
    init_pt_ = odom_pos_;
    reference_path_ = waypoints;
    reference_progress_idx_ = 0;
    reference_progress_ratio_ = 0.0;
    reference_path_active_ = true;
    end_pt_ = reference_path_.back();

    bool success = planner_manager_->planGlobalTraj(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success && !adjustGlobalTargetIfOccupied())
      success = false;

    if (success)
    {
      visualization_->displayGlobalPathList(reference_path_, 0.1, 0);
      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);
      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;
      reference_path_update_pending_ = true;
      pending_reference_request_id_ = request_id;
      if (exec_state_ == EMERGENCY_STOP)
        emergency_path_pending_ = true;
    }

    if (success)
    {
      if (safety_stop_active_.exchange(false))
        RCLCPP_INFO(node_->get_logger(),
                    "[SAFETY_RELEASE] source=new_reference_path; planning may resume");
      collision_segment_pending_ = false;
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
      publishReferenceStatus("PATH_ACCEPTED", request_id);
    }
    else
    {
      reference_path_active_ = false;
      requestExecutionStop("REFERENCE_PATH_REJECTED");
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  void SCANReplanFSM::safetyOdometryCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    std::lock_guard<std::mutex> lock(safety_odom_mutex_);
    safety_odom_pos_ << msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z;
    safety_odom_orient_ = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    if (safety_odom_orient_.norm() > 1e-6)
      safety_odom_orient_.normalize();
    else
      safety_odom_orient_ = Eigen::Quaterniond::Identity();
    safety_have_odom_ = true;
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::publishStatus(const std::string &status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
  }

  void SCANReplanFSM::publishReferenceStatus(
      const std::string &status, uint64_t request_id)
  {
    if (request_id == 0)
      request_id = active_reference_request_id_.load();
    if (request_id == 0)
    {
      publishStatus(status);
      return;
    }
    publishStatus(status + " request_id=" + std::to_string(request_id));
  }

  void SCANReplanFSM::publishBlockedSegment()
  {
    if (!collision_segment_pending_)
      return;

    nav_msgs::msg::Path msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = self_inflation_frame_id_;
    msg.poses.resize(2);
    for (auto &pose : msg.poses)
    {
      pose.header = msg.header;
      pose.pose.orientation.w = 1.0;
    }
    msg.poses[0].pose.position.x = last_collision_free_pos_(0);
    msg.poses[0].pose.position.y = last_collision_free_pos_(1);
    msg.poses[0].pose.position.z = last_collision_free_pos_(2);
    msg.poses[1].pose.position.x = first_collision_pos_(0);
    msg.poses[1].pose.position.y = first_collision_pos_(1);
    msg.poses[1].pose.position.z = first_collision_pos_(2);
    blocked_segment_pub_->publish(msg);
    RCLCPP_WARN(node_->get_logger(),
                "Published locally blocked segment [%.2f, %.2f] -> [%.2f, %.2f]",
                last_collision_free_pos_(0), last_collision_free_pos_(1),
                first_collision_pos_(0), first_collision_pos_(1));
  }

  void SCANReplanFSM::requestExecutionStop(const std::string &reason)
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    emergency_stop_pub_->publish(msg);
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      publishReferenceStatus(reason);
    else
      publishStatus(reason);
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;

    if (new_state == EXEC_TRAJ)
      publishStatus("RUNNING");
    else if (new_state == GEN_NEW_TRAJ || new_state == REPLAN_TRAJ)
      publishStatus("PLANNING");
    else if (new_state == EMERGENCY_STOP)
      publishStatus("EMERGENCY_STOP");
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();

    // A real-time veto is a latch, not a momentary warning.  The planning
    // group must not reuse the rejected reference path while the Explorer's
    // replacement path is still queued behind this callback.
    if (safety_stop_active_.load())
    {
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[SAFETY_LATCHED] waiting for a newly accepted goal/reference path");
      return;
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {
        next_rolling_replan_attempt_ns_ = 0;
        replan_fail_count_ = 0;
        tracking_recovery_active_ = false;
        collision_segment_pending_ = false;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (!isWaypointSequenceMode() &&
          (end_pt_.head<2>() - odom_pos_.head<2>()).norm() <= goal_tolerance_)
      {
        have_target_ = false;
        reference_path_active_ = false;
        requestExecutionStop("REACHED");
        changeFSMExecState(WAIT_TARGET, "GOAL_REACHED");
        return;
      }

      if (planFromCurrentTraj())
      {
        next_rolling_replan_attempt_ns_ = 0;
        replan_fail_count_ = 0;
        tracking_recovery_active_ = false;
        collision_segment_pending_ = false;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        const auto *info = &planner_manager_->local_data_;
        const double t_cur = std::clamp(
            (node_->now() - info->start_time_).seconds(), 0.0, info->duration_);
        const double remaining_time = std::max(0.0, info->duration_ - t_cur);
        if (shouldKeepExecutingAfterRollingReplanFailure(
                reference_path_active_, reference_path_update_pending_,
                safety_stop_active_.load(), go2_execution_frozen_.load(),
                remaining_time, emergency_time_))
        {
          next_rolling_replan_attempt_ns_ =
              node_->now().nanoseconds() + static_cast<int64_t>(
                  rolling_replan_retry_period_ * 1e9);
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "[ROLLING_REPLAN_RETRY] planning failed with %.2fs left; continuing the validated current trajectory and retrying in %.2fs",
              remaining_time, rolling_replan_retry_period_);
          changeFSMExecState(EXEC_TRAJ, "ROLLING_RETRY");
        }
        else
        {
          next_rolling_replan_attempt_ns_ = 0;
          requestExecutionStop("REPLANNING");
          replan_fail_count_++;
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (!isWaypointSequenceMode() &&
          (end_pt_.head<2>() - odom_pos_.head<2>()).norm() <= goal_tolerance_)
      {
        have_target_ = false;
        reference_path_active_ = false;
        requestExecutionStop("REACHED");
        changeFSMExecState(WAIT_TARGET, "GOAL_REACHED");
        return;
      }

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        // A local B-spline ending is not the end of a long reference path.
        // Continue from odometry immediately instead of reporting a false
        // global REACHED and waiting for the explorer to issue another task.
        if (reference_path_active_ &&
            (end_pt_.head<2>() - odom_pos_.head<2>()).norm() > goal_tolerance_)
        {
          if (planFromCurrentTraj())
          {
            next_rolling_replan_attempt_ns_ = 0;
            replan_fail_count_ = 0;
            tracking_recovery_active_ = false;
            collision_segment_pending_ = false;
            changeFSMExecState(EXEC_TRAJ, "LOCAL_SEGMENT_DONE");
          }
          else
          {
            next_rolling_replan_attempt_ns_ = 0;
            requestExecutionStop("REPLANNING");
            replan_fail_count_++;
            changeFSMExecState(REPLAN_TRAJ, "LOCAL_SEGMENT_DONE");
          }
          return;
        }

        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;
        reference_path_active_ = false;
        requestExecutionStop("REACHED");

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if (next_rolling_replan_attempt_ns_ > 0 &&
               time_now.nanoseconds() < next_rolling_replan_attempt_ns_)
      {
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          need_hover_stop_ = false;
          if (shouldResumePendingEmergencyPath(emergency_path_pending_, have_target_))
          {
            emergency_path_pending_ = false;
            trigger_ = true;
            replan_fail_count_ = 0;
            RCLCPP_INFO(node_->get_logger(),
                        "Exiting EMERGENCY_STOP; planning the path received while stopping");
            changeFSMExecState(GEN_NEW_TRAJ, "EMERGENCY_PATH");
          }
          else
          {
            RCLCPP_INFO(node_->get_logger(),
                        "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
            have_target_ = false;
            trigger_ = false;
            changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
          }
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      const bool tracking_recovery_failed = tracking_recovery_active_;
      RCLCPP_WARN(node_->get_logger(),
                  tracking_recovery_failed
                      ? "Tracking recovery failed %d times; emergency stop and retry the committed reference path"
                      : "Replan failed %d times; emergency stop and wait for a new target",
                  replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      emergency_path_pending_ = tracking_recovery_failed && have_target_;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
      requestExecutionStop(tracking_recovery_failed
                               ? "TRACKING_RECOVERY_FAILED"
                               : "BLOCKED");
      tracking_recovery_active_ = false;
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    const bool trajectory_valid =
        info->start_time_.seconds() > 1e-5 && info->duration_ > 1e-5;
    const double remaining_time =
        trajectory_valid ? std::max(0.0, info->duration_ - t_cur) : 0.0;
    const double tracking_error = trajectory_valid
        ? (info->position_traj_.evaluateDeBoorT(t_cur).head<2>() -
           odom_pos_.head<2>()).norm()
        : std::numeric_limits<double>::infinity();
    const bool reuse_suffix = shouldReuseCurrentTrajectorySuffix(
        reference_path_update_pending_, trajectory_valid, remaining_time,
        tracking_error, rolling_replan_max_start_error_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    if (reference_path_active_)
    {
      // A replacement global path may turn away from the previous local
      // trajectory.  Inherit the measured motion only in the new path
      // direction; carrying the old desired velocity creates a large entry
      // arc before collision optimization begins.
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      alignStartStateToReferencePath();
    }
    else
    {
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    }

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    bool success = false;
    if (reuse_suffix)
    {
      RCLCPP_INFO(node_->get_logger(),
                  "[ROLLING_REPLAN_MODE] reuse_validated_suffix remaining=%.2fs tracking_error=%.2fm",
                  remaining_time, tracking_error);
      success = callReboundReplan(false, false);
      if (success)
        return true;
      RCLCPP_WARN(node_->get_logger(),
                  "[ROLLING_REPLAN_FALLBACK] validated suffix could not be repaired; regenerating from current odometry");
    }

    success = callReboundReplan(true, false);
    if (success)
      return true;

    RCLCPP_WARN(node_->get_logger(),
                "[ROLLING_REPLAN_FALLBACK] deterministic regeneration failed; trying randomized initialization");
    return callReboundReplan(true, true);
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    if (reference_path_active_)
    {
      alignStartStateToReferencePath();
      return;
    }

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::alignStartStateToReferencePath()
  {
    if (!reference_path_active_ || reference_path_.size() < 2)
      return;

    // Use the segment at the robot, not a segment beyond the next corner.
    const ReferencePathLookahead entry = projectReferencePathLookahead(
        reference_path_, start_pt_, reference_progress_idx_,
        reference_progress_ratio_, 0.20);
    Eigen::Vector2d tangent = entry.tangent.head<2>();
    const double tangent_norm = tangent.norm();
    if (!entry.valid || tangent_norm < 1e-6)
    {
      start_vel_.setZero();
      start_acc_.setZero();
      return;
    }

    tangent /= tangent_norm;
    const double forward_speed = std::max(0.0, start_vel_.head<2>().dot(tangent));
    start_vel_.head<2>() = forward_speed * tangent;
    start_vel_(2) = 0.0;
    start_acc_.setZero();
  }

  void SCANReplanFSM::updateExecutionTrajectorySnapshot(const LocalTrajData &info)
  {
    std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
    execution_snapshot_.position = info.position_traj_;
    execution_snapshot_.start_time = info.start_time_;
    execution_snapshot_.duration = info.duration_;
    execution_snapshot_.request_id = active_reference_request_id_.load();
    execution_snapshot_.valid = info.start_time_.seconds() > 1e-5 && info.duration_ > 0.0;
  }

  void SCANReplanFSM::tripRealtimeSafety(
      const std::string &reason, const Eigen::Vector3d *last_free,
      const Eigen::Vector3d *first_blocked, uint64_t request_id)
  {
    if (request_id != 0 && request_id != active_reference_request_id_.load())
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Ignoring safety result for stale request_id=%llu",
                  static_cast<unsigned long long>(request_id));
      return;
    }
    bool expected = false;
    if (!safety_stop_active_.compare_exchange_strong(expected, true))
      return;

    safety_generation_.fetch_add(1);
    requestExecutionStop(reason);
    if (!last_free || !first_blocked)
      return;

    nav_msgs::msg::Path msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = self_inflation_frame_id_;
    msg.poses.resize(2);
    for (auto &pose : msg.poses)
    {
      pose.header = msg.header;
      pose.pose.orientation.w = 1.0;
    }
    msg.poses[0].pose.position.x = last_free->x();
    msg.poses[0].pose.position.y = last_free->y();
    msg.poses[0].pose.position.z = last_free->z();
    msg.poses[1].pose.position.x = first_blocked->x();
    msg.poses[1].pose.position.y = first_blocked->y();
    msg.poses[1].pose.position.z = first_blocked->z();
    blocked_segment_pub_->publish(msg);
  }

  void SCANReplanFSM::checkCollisionRealtimeCallback()
  {
    const auto wall_now = std::chrono::steady_clock::now();
    double callback_gap = 0.0;
    if (last_safety_callback_wall_.time_since_epoch().count() != 0)
      callback_gap = std::chrono::duration<double>(wall_now - last_safety_callback_wall_).count();
    last_safety_callback_wall_ = wall_now;

    auto map = planner_manager_->grid_map_;
    const auto map_snapshot = map->captureInflatedOccupancySnapshot();
    map->useInflatedOccupancySnapshotForCurrentThread(map_snapshot);
    struct SnapshotScope
    {
      GridMap::Ptr map;
      ~SnapshotScope() { map->clearInflatedOccupancySnapshotForCurrentThread(); }
    } snapshot_scope{map};
    if (callback_gap > 0.15)
      RCLCPP_WARN(node_->get_logger(),
                  "[SAFETY_SCHEDULER_LAG] callback_gap=%.1fms map_age=%.1fms",
                  callback_gap * 1000.0, map->getMapAgeSeconds() * 1000.0);

    Eigen::Vector3d odom_pos;
    Eigen::Quaterniond odom_orient;
    {
      std::lock_guard<std::mutex> lock(safety_odom_mutex_);
      if (!safety_have_odom_)
        return;
      odom_pos = safety_odom_pos_;
      odom_orient = safety_odom_orient_;
    }

    ExecutionTrajectorySnapshot trajectory;
    {
      std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
      if (!execution_snapshot_.valid)
        return;
      if (go2_execution_frozen_.load() && callback_gap > 0.0 && callback_gap < 0.5)
        execution_snapshot_.start_time += rclcpp::Duration::from_seconds(callback_gap);
      trajectory = execution_snapshot_;
    }

    const double map_age = map->getMapAgeSeconds();
    if (map_age > 0.25)
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[SAFETY_MAP_STALE] map_age=%.1fms revision=%lu",
                           map_age * 1000.0,
                           static_cast<unsigned long>(map->getMapRevision()));

    const double actual_yaw = std::atan2(
        2.0 * (odom_orient.w() * odom_orient.z() + odom_orient.x() * odom_orient.y()),
        1.0 - 2.0 * (odom_orient.y() * odom_orient.y() + odom_orient.z() * odom_orient.z()));
    const auto segment_yaw = [actual_yaw](const Eigen::Vector3d &from,
                                          const Eigen::Vector3d &to) {
      const Eigen::Vector2d diff = to.head<2>() - from.head<2>();
      return diff.squaredNorm() < 1e-8 ? actual_yaw : std::atan2(diff.y(), diff.x());
    };

    if (map->getInflateOccupancy(odom_pos, actual_yaw) != 0)
    {
      tripRealtimeSafety("BLOCKED");
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[REALTIME_SAFETY_STOP] reason=pose_occupied pos=(%.2f,%.2f) map_age=%.1fms",
          odom_pos.x(), odom_pos.y(), map_age * 1000.0);
      return;
    }

    double t_cur = (node_->now() - trajectory.start_time).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), trajectory.duration);
    const Eigen::Vector3d tracked_pos = trajectory.position.evaluateDeBoorT(t_cur);
    const Eigen::Vector3d connector = tracked_pos - odom_pos;
    const double connector_length = connector.head<2>().norm();
    constexpr double spatial_step = 0.05;
    if (connector_length > spatial_step)
    {
      const int samples = std::max(1, static_cast<int>(std::ceil(connector_length / spatial_step)));
      const double yaw = segment_yaw(odom_pos, tracked_pos);
      for (int sample = 1; sample <= samples; ++sample)
      {
        const Eigen::Vector3d point = odom_pos +
            (static_cast<double>(sample) / samples) * connector;
        if (map->getInflateOccupancy(point, yaw) != 0)
        {
          tripRealtimeSafety("BLOCKED", &odom_pos, &point, trajectory.request_id);
          RCLCPP_WARN(node_->get_logger(),
                      "[REALTIME_SAFETY_STOP] reason=connector_blocked error=%.2fm hit=(%.2f,%.2f) map_age=%.1fms",
                      connector_length, point.x(), point.y(), map_age * 1000.0);
          return;
        }
      }
    }

    constexpr double time_step = 0.02;
    Eigen::Vector3d last_free = odom_pos;
    for (double t = t_cur; t < trajectory.duration; t += time_step)
    {
      const Eigen::Vector3d pos = trajectory.position.evaluateDeBoorT(t);
      const Eigen::Vector3d next = trajectory.position.evaluateDeBoorT(
          std::min(t + time_step, trajectory.duration));
      if (map->getInflateOccupancy(pos, segment_yaw(pos, next)) != 0)
      {
        tripRealtimeSafety("BLOCKED", &last_free, &pos, trajectory.request_id);
        RCLCPP_WARN(node_->get_logger(),
                    "[REALTIME_SAFETY_STOP] reason=trajectory_blocked time_to_hit=%.2fs hit=(%.2f,%.2f) map_age=%.1fms",
                    std::max(0.0, t - t_cur), pos.x(), pos.y(), map_age * 1000.0);
        return;
      }
      last_free = pos;
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    checkCollisionRealtimeCallback();
  }
  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    auto map = planner_manager_->grid_map_;
    // reboundReplan commits its successful result to local_data_ before the
    // real-time publication checks below.  Preserve the trajectory that the
    // controller is actually executing so every rejected result is atomic.
    const LocalTrajData executing_trajectory = planner_manager_->local_data_;
    const auto reject_planned_trajectory = [&]() {
      planner_manager_->local_data_ = executing_trajectory;
      return false;
    };
    const auto map_snapshot = map->captureInflatedOccupancySnapshot();
    const uint64_t safety_generation_before = safety_generation_.load();
    const auto planning_started = std::chrono::steady_clock::now();
    map->useInflatedOccupancySnapshotForCurrentThread(map_snapshot);
    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    map->clearInflatedOccupancySnapshotForCurrentThread();
    const double optimization_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - planning_started).count();
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      // The optimizer and final validation use separate immutable revisions.
      // Keep the newest completed snapshot bound through every occupancy query
      // below so concurrent ray fusion cannot stall trajectory publication.
      const auto validation_started = std::chrono::steady_clock::now();
      const auto validation_snapshot = map->captureInflatedOccupancySnapshot();
      map->useInflatedOccupancySnapshotForCurrentThread(validation_snapshot);
      struct ValidationSnapshotScope
      {
        GridMap::Ptr map;
        ~ValidationSnapshotScope()
        {
          map->clearInflatedOccupancySnapshotForCurrentThread();
        }
      } validation_snapshot_scope{map};

      if (safety_generation_.load() != safety_generation_before)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "[PLAN_RESULT_DISCARDED] realtime safety stopped execution while this plan was computing");
        return reject_planned_trajectory();
      }

      auto info = &planner_manager_->local_data_;

      // The optimizer is not the final safety authority.  Sample the complete
      // B-spline again immediately before publishing it to the controller so a
      // successful solver result can never authorize an occupied trajectory.
      constexpr double validation_dt = 0.02;
      for (double t = 0.0; t <= info->duration_; t += validation_dt)
      {
        const Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
        const Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(
            std::min(t + validation_dt, info->duration_));
        if (planner_manager_->grid_map_->getInflateOccupancy(
                pos, estimateYawFromSegment(pos, pos_next)))
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "Rejecting optimized trajectory: collision at t=%.3fs, position=(%.3f, %.3f, %.3f)",
                       t, pos.x(), pos.y(), pos.z());
          return reject_planned_trajectory();
        }
      }

      Eigen::Vector3d live_odom;
      bool have_live_odom = false;
      {
        std::lock_guard<std::mutex> lock(safety_odom_mutex_);
        have_live_odom = safety_have_odom_;
        live_odom = safety_odom_pos_;
      }
      if (have_live_odom)
      {
        const Eigen::Vector3d trajectory_start = info->position_traj_.evaluateDeBoorT(0.0);
        const Eigen::Vector3d connector = trajectory_start - live_odom;
        const double connector_length = connector.head<2>().norm();
        if (!isRollingTrajectoryStartFresh(
                connector_length, rolling_replan_max_start_error_))
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "[PLAN_RESULT_DISCARDED] stale trajectory start error=%.2fm limit=%.2fm; preserving the currently executing trajectory",
                       connector_length, rolling_replan_max_start_error_);
          return reject_planned_trajectory();
        }
        if (connector_length > 0.05)
        {
          const int samples = std::max(1, static_cast<int>(std::ceil(connector_length / 0.05)));
          const double connector_yaw = std::atan2(connector.y(), connector.x());
          for (int sample = 1; sample <= samples; ++sample)
          {
            const Eigen::Vector3d point = live_odom +
                (static_cast<double>(sample) / samples) * connector;
            if (map->getInflateOccupancy(point, connector_yaw) != 0)
            {
              RCLCPP_ERROR(node_->get_logger(),
                           "[PLAN_RESULT_DISCARDED] latest odom connector is blocked error=%.2fm hit=(%.2f,%.2f)",
                           connector_length, point.x(), point.y());
              return reject_planned_trajectory();
            }
          }
        }
      }

      if (safety_generation_.load() != safety_generation_before)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "[PLAN_RESULT_DISCARDED] realtime safety stopped execution during final validation");
        return reject_planned_trajectory();
      }

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      updateExecutionTrajectorySnapshot(*info);
      bspline_pub_->publish(bspline);
      if (reference_path_update_pending_)
      {
        publishReferenceStatus("PATH_TRAJECTORY_READY", pending_reference_request_id_);
        reference_path_update_pending_ = false;
        pending_reference_request_id_ = 0;
      }

      visualization_->displayOptimalTraj(info->position_traj_, 0);

      const double validation_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - validation_started).count();
      RCLCPP_INFO(node_->get_logger(),
                  "[PLAN_TIMING] optimization=%.1fms validation=%.1fms total=%.1fms planning_revision=%lu validation_revision=%lu live_revision=%lu map_age=%.1fms result=success",
                  optimization_ms, validation_ms, optimization_ms + validation_ms,
                  static_cast<unsigned long>(map_snapshot->revision),
                  static_cast<unsigned long>(validation_snapshot->revision),
                  static_cast<unsigned long>(map->getMapRevision()),
                  map->getMapAgeSeconds() * 1000.0);
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(),
                  "[PLAN_TIMING] optimization=%.1fms validation=0.0ms total=%.1fms planning_revision=%lu validation_revision=0 live_revision=%lu map_age=%.1fms result=failure",
                  optimization_ms, optimization_ms,
                  static_cast<unsigned long>(map_snapshot->revision),
                  static_cast<unsigned long>(map->getMapRevision()),
                  map->getMapAgeSeconds() * 1000.0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    updateExecutionTrajectorySnapshot(*info);
    bspline_pub_->publish(bspline);

    return true;
  }

  void SCANReplanFSM::getLocalTarget()
  {
    if (reference_path_active_)
    {
      getReferencePathLocalTarget();
      return;
    }

    double t;

    double t_step = planning_horizon_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    double target_t = planner_manager_->global_data_.global_duration_;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                     dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = t;
        break;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      target_t = planner_manager_->global_data_.global_duration_;
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= planner_manager_->global_data_.global_duration_)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, dist_min_t))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      // cout << "AA" << endl;
    }
  }

  void SCANReplanFSM::getReferencePathLocalTarget()
  {
    if (reference_path_.size() < 2)
    {
      local_target_pt_ = end_pt_;
      local_target_vel_.setZero();
      return;
    }

    const double lookahead = std::max(0.5, reference_path_lookahead_);
    ReferencePathLookahead path_sample = projectReferencePathLookahead(
        reference_path_, start_pt_, reference_progress_idx_, reference_progress_ratio_, lookahead);
    if (!path_sample.valid)
    {
      local_target_pt_ = end_pt_;
      local_target_vel_.setZero();
      return;
    }

    reference_progress_idx_ = path_sample.progress_segment;
    reference_progress_ratio_ = path_sample.progress_ratio;
    Eigen::Vector3d target = path_sample.target;
    Eigen::Vector3d tangent = path_sample.tangent;

    auto isOccupied = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(
                 pt, estimateYawFromSegment(start_pt_, pt)) != 0;
    };

    // Sensor-map inflation can be slightly more conservative than the global
    // grid.  Stay on the supplied polyline and shorten lookahead; never move a
    // target sideways off the globally validated corridor.
    if (isOccupied(target))
    {
      bool found = false;
      for (double shorter_lookahead = lookahead - 0.2;
           shorter_lookahead >= 0.4;
           shorter_lookahead -= 0.2)
      {
        const ReferencePathLookahead shorter_sample = projectReferencePathLookahead(
            reference_path_, start_pt_, reference_progress_idx_, reference_progress_ratio_,
            shorter_lookahead);
        if (shorter_sample.valid &&
            (shorter_sample.target - start_pt_).norm() > 0.4 &&
            !isOccupied(shorter_sample.target))
        {
          target = shorter_sample.target;
          tangent = shorter_sample.tangent;
          path_sample.remaining_after_target = shorter_sample.remaining_after_target;
          found = true;
          break;
        }
      }
      if (!found)
      {
        target = start_pt_;
        tangent.setZero();
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "No free reference-path lookahead is currently visible");
      }
    }

    local_target_pt_ = target;

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[ROLLING_LOCAL_TARGET] lookahead=%.2fm target_distance=%.2fm remaining_after_target=%.2fm",
        lookahead, (local_target_pt_ - start_pt_).norm(),
        path_sample.remaining_after_target);

    const double stop_distance =
        planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_ /
        (2.0 * planner_manager_->pp_.max_acc_);
    if (path_sample.remaining_after_target <= stop_distance || tangent.norm() < 1e-6)
      local_target_vel_.setZero();
    else
      local_target_vel_ = tangent * planner_manager_->pp_.max_vel_;
  }

} // namespace scan_planner
