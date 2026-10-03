
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
    heading_stall_handled_ = false;
    heading_freeze_recoveries_ = 0;
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
    predictive_replan_reaction_time_ = load_parameter<double>(
        node_, "fsm.predictive_replan_reaction_time", 0.25);
    predictive_hard_stop_min_time_ = load_parameter<double>(
        node_, "fsm.predictive_hard_stop_min_time", 0.35);
    tracking_degraded_error_ = load_parameter<double>(
        node_, "fsm.tracking_degraded_error", 0.10);
    tracking_recovery_max_error_ = load_parameter<double>(
        node_, "fsm.tracking_recovery_max_error", 0.30);
    tracking_match_back_time_ = load_parameter<double>(
        node_, "fsm.tracking_match_back_time", 0.40);
    tracking_match_forward_time_ = load_parameter<double>(
        node_, "fsm.tracking_match_forward_time", 0.80);
    planning_clearance_margin_ = load_parameter<double>(
        node_, "fsm.planning_clearance_margin", 0.10);
    preferred_clearance_margin_ = load_parameter<double>(
        node_, "fsm.preferred_clearance_margin", 0.20);
    clearance_limited_max_forward_speed_ = load_parameter<double>(
        node_, "fsm.clearance_limited_max_forward_speed", 0.30);
    min_forward_command_ = load_parameter<double>(
        node_, "fsm.min_forward_command", 0.30);
    min_forward_effective_speed_ = load_parameter<double>(
        node_, "fsm.min_forward_effective_speed", 0.165);
    if (!std::isfinite(min_forward_command_) || min_forward_command_ <= 0.0 ||
        !std::isfinite(min_forward_effective_speed_) || min_forward_effective_speed_ <= 0.0 ||
        clearance_limited_max_forward_speed_ < min_forward_command_)
      throw std::invalid_argument("invalid measured forward gait profile");
    clearance_limited_max_lateral_speed_ = load_parameter<double>(
        node_, "fsm.clearance_limited_max_lateral_speed", 0.08);
    clearance_limited_max_yaw_rate_ = load_parameter<double>(
        node_, "fsm.clearance_limited_max_yaw_rate", 0.50);
    clearance_limited_max_tracking_correction_ = load_parameter<double>(
        node_, "fsm.clearance_limited_max_tracking_correction", 0.08);
    clearance_limited_execution_grace_ = load_parameter<double>(
        node_, "fsm.clearance_limited_execution_grace", 1.00);
    clearance_limited_no_progress_timeout_ = load_parameter<double>(
        node_, "fsm.clearance_limited_no_progress_timeout", 2.00);
    clearance_limited_min_progress_ = load_parameter<double>(
        node_, "fsm.clearance_limited_min_progress", 0.05);
    local_repair_anchor_min_distance_ = load_parameter<double>(
        node_, "fsm.local_repair_anchor_min_distance", 0.45);
    local_repair_anchor_max_distance_ = load_parameter<double>(
        node_, "fsm.local_repair_anchor_max_distance", 0.90);
    local_repair_anchor_step_ = load_parameter<double>(
        node_, "fsm.local_repair_anchor_step", 0.15);
    local_repair_max_backtrack_ = load_parameter<double>(
        node_, "fsm.local_repair_max_backtrack", 0.60);
    local_repair_max_candidates_ = load_parameter<int>(
        node_, "fsm.local_repair_max_candidates", 6);
    local_repair_max_segments_per_transaction_ = load_parameter<int>(
        node_, "fsm.local_repair_max_segments_per_transaction", 3);
    local_repair_max_speed_ = load_parameter<double>(
        node_, "fsm.local_repair_max_speed", 0.30);
    local_repair_time_margin_ = load_parameter<double>(
        node_, "fsm.local_repair_time_margin", 1.20);
    enable_holonomic_lateral_repair_ = load_parameter<bool>(
        node_, "fsm.enable_holonomic_lateral_repair", false);
    braking_sweep_spatial_step_ = load_parameter<double>(
        node_, "fsm.braking_sweep_spatial_step", 0.05);
    braking_sweep_max_horizon_ = load_parameter<double>(
        node_, "fsm.braking_sweep_max_horizon", 4.0);
    safety_map_stale_warn_age_ = load_parameter<double>(
        node_, "fsm.safety_map_stale_warn_age", 0.25);
    safety_map_stale_hold_age_ = load_parameter<double>(
        node_, "fsm.safety_map_stale_hold_age", 0.50);
    safety_callback_hold_gap_ = load_parameter<double>(
        node_, "fsm.safety_callback_hold_gap", 0.30);
    odom_velocity_filter_tau_ = load_parameter<double>(
        node_, "fsm.odom_velocity_filter_tau", 0.20);
    motion_estimation_window_ = load_parameter<double>(
        node_, "fsm.motion_estimation_window", 0.35);
    yaw_rate_filter_tau_ = load_parameter<double>(
        node_, "fsm.yaw_rate_filter_tau", 0.15);
    guaranteed_braking_deceleration_ = load_parameter<double>(
        node_, "fsm.guaranteed_braking_deceleration", 0.32);
    command_stop_latency_ = load_parameter<double>(
        node_, "fsm.command_stop_latency", 0.30);
    trajectory_ack_timeout_ = load_parameter<double>(
        node_, "fsm.trajectory_ack_timeout", 0.75);
    trajectory_ack_reconciliation_timeout_ = load_parameter<double>(
        node_, "fsm.trajectory_ack_reconciliation_timeout", 0.75);
    pose_free_release_cycles_ = load_parameter<int>(
        node_, "fsm.pose_free_release_cycles", 3);
    local_hold_release_cycles_ = load_parameter<int>(
        node_, "fsm.local_hold_release_cycles", 3);
    if (predictive_replan_reaction_time_ < 0.0 ||
        predictive_hard_stop_min_time_ < 0.0 ||
        tracking_degraded_error_ < 0.0 ||
        tracking_recovery_max_error_ <= tracking_degraded_error_ ||
        tracking_match_back_time_ < 0.0 ||
        tracking_match_forward_time_ <= 0.0 ||
        planning_clearance_margin_ < 0.0 ||
        preferred_clearance_margin_ <= planning_clearance_margin_ ||
        clearance_limited_max_forward_speed_ <= 0.0 ||
        clearance_limited_max_lateral_speed_ < 0.0 ||
        clearance_limited_max_yaw_rate_ <= 0.0 ||
        clearance_limited_max_tracking_correction_ <= 0.0 ||
        clearance_limited_execution_grace_ < 0.0 ||
        clearance_limited_no_progress_timeout_ <
            clearance_limited_execution_grace_ ||
        clearance_limited_min_progress_ < 0.0 ||
        local_repair_anchor_min_distance_ <= 0.2 ||
        local_repair_anchor_max_distance_ <
            local_repair_anchor_min_distance_ ||
        local_repair_anchor_step_ <= 0.0 ||
        local_repair_max_backtrack_ < 0.0 ||
        local_repair_max_candidates_ <= 0 ||
        local_repair_max_segments_per_transaction_ <= 0 ||
        local_repair_max_speed_ <= 0.0 ||
        local_repair_time_margin_ < 1.0 ||
        braking_sweep_spatial_step_ <= 0.0 ||
        braking_sweep_max_horizon_ <= 0.0 ||
        safety_map_stale_warn_age_ < 0.0 ||
        safety_map_stale_hold_age_ <= safety_map_stale_warn_age_ ||
        safety_callback_hold_gap_ <= 0.0 ||
        odom_velocity_filter_tau_ < 0.0 ||
        motion_estimation_window_ < 0.05 ||
        yaw_rate_filter_tau_ < 0.0 ||
        guaranteed_braking_deceleration_ <= 0.0 ||
        command_stop_latency_ < 0.0 ||
        trajectory_ack_timeout_ <= 0.0 ||
        trajectory_ack_reconciliation_timeout_ <= 0.0 ||
        pose_free_release_cycles_ <= 0 ||
        local_hold_release_cycles_ <= 0)
      throw std::runtime_error(
          "predictive collision and tracking recovery parameters are invalid");
    goal_tolerance_ = load_parameter<double>(node_, "fsm.goal_tolerance", 0.25);
    heading_freeze_max_recoveries_ = load_parameter<int>(
        node_, "fsm.heading_freeze_max_recoveries", 2);
    if (heading_freeze_max_recoveries_ < 1)
      throw std::runtime_error("fsm.heading_freeze_max_recoveries must be at least 1");
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 5);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    live_body_height_ = load_parameter<bool>(node_, "grid_map.live_body_height", true);
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
    odom_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    control_feedback_callback_group_ = node_->create_callback_group(
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

    // Precompute the small 2-D stencil once. Every later safety query runs
    // against the callback's immutable occupancy snapshot without allocating
    // or holding the mapper's write lock.
    planning_clearance_offsets_.clear();
    const double resolution = planner_manager_->grid_map_->getResolution();
    const double maximum_clearance_margin = std::max(
        planning_clearance_margin_, preferred_clearance_margin_);
    const int clearance_cells = static_cast<int>(
        std::ceil(maximum_clearance_margin / resolution));
    for (int x = -clearance_cells; x <= clearance_cells; ++x)
      for (int y = -clearance_cells; y <= clearance_cells; ++y)
      {
        const Eigen::Vector2d offset(x * resolution, y * resolution);
        if (offset.norm() <= maximum_clearance_margin + 1e-9)
          planning_clearance_offsets_.push_back(offset);
      }
    if (planning_clearance_offsets_.empty())
      planning_clearance_offsets_.push_back(Eigen::Vector2d::Zero());
    RCLCPP_INFO(
        node_->get_logger(),
        "[LOCAL_SAFETY_STENCIL] margin=%.2fm resolution=%.2fm offsets=%zu",
        planning_clearance_margin_, resolution,
        planning_clearance_offsets_.size());

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this),
                                           planning_callback_group_);
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this),
                                             safety_callback_group_);
    last_fsm_callback_wall_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());

    rclcpp::SubscriptionOptions planning_options;
    planning_options.callback_group = planning_callback_group_;
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_callback_group_;
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::safetyOdometryCallback, this, std::placeholders::_1),
        odom_options);
    rclcpp::SubscriptionOptions safety_options;
    safety_options.callback_group = safety_callback_group_;
    rclcpp::SubscriptionOptions control_feedback_options;
    control_feedback_options.callback_group = control_feedback_callback_group_;
    execution_state_sub_ = node_->create_subscription<scan_planner_msgs::msg::ExecutionState>(
        "planning/execution_state", rclcpp::QoS(20).reliable(),
        std::bind(&SCANReplanFSM::executionStateCallback, this, std::placeholders::_1),
        control_feedback_options);
    trajectory_ack_sub_ = node_->create_subscription<scan_planner_msgs::msg::TrajectoryAck>(
        "planning/trajectory_ack", rclcpp::QoS(20).reliable(),
        std::bind(&SCANReplanFSM::trajectoryAckCallback, this, std::placeholders::_1),
        control_feedback_options);
    go2_heading_stalled_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_heading_stalled", 10,
        std::bind(&SCANReplanFSM::go2HeadingStalledCallback, this, std::placeholders::_1),
        control_feedback_options);
    go2_heading_error_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "planning/go2_heading_error", 10,
        std::bind(&SCANReplanFSM::go2HeadingErrorCallback, this, std::placeholders::_1),
        control_feedback_options);

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());
    blocked_segment_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "planning/blocked_segment", 10);
    execution_command_pub_ = node_->create_publisher<scan_planner_msgs::msg::ExecutionCommand>(
        "planning/execution_command", rclcpp::QoS(20).reliable());
    failure_evidence_pub_ = node_->create_publisher<scan_planner_msgs::msg::FailureEvidence>(
        "planning/failure_evidence", rclcpp::QoS(20).reliable());
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
      if (newPlanMayReleaseSafetyStop(
              pose_occupied_latched_.load(), predictive_hold_latched_.load(),
              temporal_safety_latched_.load()) &&
          !invalid_odom_latched_.load() &&
          safety_stop_active_.exchange(false))
        RCLCPP_INFO(node_->get_logger(),
                    "[SAFETY_RELEASE] source=new_manual_goal; planning may resume");
      else if (pose_occupied_latched_.load())
        RCLCPP_WARN(node_->get_logger(),
                    "[SAFETY_RELEASE_SUPPRESSED] source=new_manual_goal reason=pose_occupied; waiting for the physical footprint to become free");

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

    const auto map_snapshot = map->captureInflatedOccupancySnapshot();
    map->useInflatedOccupancySnapshotForCurrentThread(map_snapshot);
    struct TargetCheckSnapshotScope
    {
      GridMap::Ptr map;
      ~TargetCheckSnapshotScope()
      {
        map->clearInflatedOccupancySnapshotForCurrentThread();
      }
    } target_check_snapshot_scope{map};

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
    const auto callback_started = std::chrono::steady_clock::now();
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }
    refreshPlanningOdomFromSafety();

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
    if (recoveryTransactionOwnsReferenceRequest())
    {
      bool retained = false;
      {
        std::lock_guard<std::mutex> lock(deferred_reference_path_mutex_);
        uint64_t deferred_request_id = 0;
        if (deferred_reference_path_)
          deferred_request_id =
              static_cast<uint64_t>(deferred_reference_path_->header.stamp.sec) *
                  1000000000ULL +
              static_cast<uint64_t>(deferred_reference_path_->header.stamp.nanosec);
        if (!deferred_reference_path_ || request_id > deferred_request_id)
        {
          deferred_reference_path_ = msg;
          retained = true;
        }
      }
      if (retained)
      {
        RCLCPP_INFO(
            node_->get_logger(),
            "[REFERENCE_PATH_DEFERRED] request_id=%llu active_request_id=%llu; structured local recovery retains ownership until rejoin ACK or terminal rejection",
            static_cast<unsigned long long>(request_id),
            static_cast<unsigned long long>(active_request_id));
        publishReferenceStatus("PATH_DEFERRED", request_id);
      }
      return;
    }
    active_reference_request_id_.store(request_id);
    completed_reference_request_id_.store(0);
    // A newer request supersedes the previous request even if its reference
    // later proves invalid.  Do not let recovery ownership or retry state from
    // the old request leak into the new request's terminal status.
    structured_local_repair_active_.store(false);
    structured_local_repair_executing_.store(false);
    structured_local_repair_finished_.store(false);
    structured_local_repair_rejoin_pending_.store(false);
    structured_local_repair_interrupted_.store(false);
    structured_local_repair_request_id_.store(0);
    structured_local_repair_trajectory_id_.store(0);
    structured_local_repair_segments_committed_.store(0);
    stopped_local_repair_pending_.store(false);
    stopped_local_repair_active_.store(false);
    last_wait_target_status_ns_.store(0);
    next_rolling_replan_attempt_ns_ = 0;
    go2_heading_stalled_.store(false);
    heading_stall_handled_ = false;
    heading_freeze_recoveries_ = 0;

    // A reference received while the real footprint is latched occupied
    // cannot become executable.  Previously it was reported PATH_ACCEPTED but
    // could never reach PATH_TRAJECTORY_READY, leaving Explorer's request
    // permanently in flight.  End this request explicitly without weakening
    // the physical-footprint latch.
    if (!newPlanMayReleaseSafetyStop(pose_occupied_latched_.load()))
    {
      have_target_ = false;
      have_new_target_ = false;
      reference_path_active_ = false;
      reference_path_update_pending_ = false;
      pending_reference_request_id_ = 0;
      RCLCPP_WARN(
          node_->get_logger(),
          "[REFERENCE_PATH_REJECTED] request_id=%llu reason=POSE_OCCUPIED; physical footprint must become free before another path can activate",
          static_cast<unsigned long long>(request_id));
      requestExecutionStop("REFERENCE_PATH_REJECTED");
      return;
    }

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses)
    {
      Eigen::Vector3d wp;
      wp(0) = pose_stamped.pose.position.x;
      wp(1) = pose_stamped.pose.position.y;
      // Explorer publishes a 2D path with z = 0.0, so its z carries no terrain
      // height.  A fixed body_height offset only holds on flat ground; on a
      // ledge the trajectory lands inside the terrain and every replan
      // collides at t = 0.  Track the live body height instead.
      wp(2) = live_body_height_
                  ? odom_pos_.z()
                  : pose_stamped.pose.position.z + body_height_;
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
      predictive_replan_requested_.store(false);
      // PATH_ACCEPTED only authorizes candidate generation.  Keep an existing
      // execution HOLD latched until the replacement B-spline has passed final
      // validation, has been published, and owns the execution snapshot.
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

      const double processing_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - callback_started).count();
      const double queue_age_ms = std::max(
          0.0, (node_->now() - msg->header.stamp).seconds() * 1000.0);
      RCLCPP_INFO(node_->get_logger(),
                  "[REFERENCE_PATH_ACCEPTED] request_id=%llu queue_age=%.1fms processing=%.1fms waypoints=%zu",
                  static_cast<unsigned long long>(request_id), queue_age_ms,
                  processing_ms, reference_path_.size());
      publishReferenceStatus("PATH_ACCEPTED", request_id);
    }
    else
    {
      reference_path_active_ = false;
      requestExecutionStop("REFERENCE_PATH_REJECTED");
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  bool SCANReplanFSM::recoveryTransactionOwnsReferenceRequest()
  {
    bool structured_handoff_pending = false;
    {
      std::lock_guard<std::mutex> lock(pending_handoff_mutex_);
      structured_handoff_pending =
          pending_handoff_.valid &&
          (pending_handoff_.structured_repair_segment ||
           pending_handoff_.structured_repair_rejoin);
    }
    return referenceRequestMustBeDeferred(
        structured_local_repair_active_.load(),
        structured_local_repair_executing_.load(),
        structured_local_repair_rejoin_pending_.load(),
        stopped_local_repair_pending_.load(),
        stopped_local_repair_active_.load(), structured_handoff_pending);
  }

  bool SCANReplanFSM::activateDeferredReferencePathIfReady()
  {
    if (recoveryTransactionOwnsReferenceRequest())
      return false;

    nav_msgs::msg::Path::ConstSharedPtr deferred;
    {
      std::lock_guard<std::mutex> lock(deferred_reference_path_mutex_);
      deferred = deferred_reference_path_;
      deferred_reference_path_.reset();
    }
    if (!deferred)
      return false;

    const uint64_t request_id =
        static_cast<uint64_t>(deferred->header.stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(deferred->header.stamp.nanosec);
    RCLCPP_INFO(
        node_->get_logger(),
        "[REFERENCE_PATH_DEFERRED_ACTIVATED] request_id=%llu; previous recovery transaction reached a terminal boundary",
        static_cast<unsigned long long>(request_id));
    pathCallback(deferred);
    return true;
  }

  void SCANReplanFSM::finishGoalReached(const char *source)
  {
    const uint64_t completed_request = active_reference_request_id_.load();
    const bool recovery_was_active =
        structured_local_repair_active_.exchange(false) ||
        structured_local_repair_executing_.load() ||
        structured_local_repair_rejoin_pending_.load() ||
        stopped_local_repair_pending_.load() ||
        stopped_local_repair_active_.load();

    structured_local_repair_executing_.store(false);
    structured_local_repair_finished_.store(false);
    structured_local_repair_rejoin_pending_.store(false);
    structured_local_repair_interrupted_.store(false);
    structured_local_repair_request_id_.store(0);
    structured_local_repair_trajectory_id_.store(0);
    structured_local_repair_segments_committed_.store(0);
    stopped_local_repair_pending_.store(false);
    stopped_local_repair_active_.store(false);
    terminal_repair_requested_.store(false);
    terminal_repair_error_.store(0.0);
    predictive_replan_requested_.store(false);
    tracking_recovery_active_ = false;
    replan_fail_count_ = 0;
    next_rolling_replan_attempt_ns_ = 0;
    have_target_ = false;
    have_new_target_ = false;
    reference_path_active_ = false;
    reference_path_update_pending_ = false;
    pending_reference_request_id_ = 0;

    requestExecutionStop("REACHED");
    changeFSMExecState(WAIT_TARGET, source);
    if (recovery_was_active)
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "[STRUCTURED_LOCAL_REPAIR_GOAL_REACHED] request_id=%llu; closing recovery ownership so a deferred reference path may activate",
          static_cast<unsigned long long>(completed_request));
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    const Eigen::Vector3d position(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    const Eigen::Vector3d velocity(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
    Eigen::Quaterniond orientation(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    if (!position.allFinite() || !velocity.allFinite() ||
        !normalizeFiniteQuaternion(&orientation))
    {
      have_odom_ = false;
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[ODOMETRY_REJECTED] planning odometry contains a non-finite state or invalid quaternion");
      return;
    }

    odom_pos_ = position;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_ = velocity;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_ = orientation;

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
    const Eigen::Vector3d position_world(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    Eigen::Quaterniond world_from_body(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    const Eigen::Vector3d body_velocity(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
    const double angular_z = msg->twist.twist.angular.z;
    if (!position_world.allFinite() || !body_velocity.allFinite() ||
        !std::isfinite(angular_z) ||
        !normalizeFiniteQuaternion(&world_from_body))
    {
      {
        std::lock_guard<std::mutex> lock(safety_odom_mutex_);
        safety_have_odom_ = false;
        safety_pose_history_.clear();
        safety_odom_velocity_stamp_ns_ = 0;
        safety_odom_vel_.setZero();
        safety_yaw_rate_ = 0.0;
      }
      invalid_odom_release_cycles_.store(0);
      latchLocalSafetyHold("INVALID_ODOMETRY", invalid_odom_latched_);
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[ODOMETRY_REJECTED] safety odometry contains a non-finite state or invalid quaternion; retaining HOLD and last valid planning state");
      return;
    }

    std::lock_guard<std::mutex> lock(safety_odom_mutex_);
    const Eigen::Vector3d twist_velocity_world =
        rotateBodyVelocityToWorld(world_from_body, body_velocity);
    int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    if (stamp_ns <= 0)
      stamp_ns = node_->now().nanoseconds();

    // Gazebo time returns to zero on a fresh run. Never regress a new motion
    // estimate against pose samples left from the previous clock epoch.
    if (!safety_pose_history_.empty() &&
        stamp_ns <= safety_pose_history_.back().stamp_ns)
    {
      safety_pose_history_.clear();
      safety_odom_velocity_stamp_ns_ = 0;
      safety_odom_vel_.setZero();
      safety_yaw_rate_ = 0.0;
    }

    const double raw_yaw = std::atan2(
        2.0 * (world_from_body.w() * world_from_body.z() +
               world_from_body.x() * world_from_body.y()),
        1.0 - 2.0 * (world_from_body.y() * world_from_body.y() +
                     world_from_body.z() * world_from_body.z()));
    double unwrapped_yaw = raw_yaw;
    if (!safety_pose_history_.empty())
    {
      const SafetyPoseSample &last = safety_pose_history_.back();
      unwrapped_yaw = unwrapPlanarYaw(
          last.unwrapped_yaw, last.raw_yaw, raw_yaw);
    }
    safety_pose_history_.push_back(
        SafetyPoseSample{stamp_ns, position_world, raw_yaw, unwrapped_yaw});
    const int64_t history_cutoff_ns = stamp_ns - static_cast<int64_t>(
        motion_estimation_window_ * 1e9);
    while (safety_pose_history_.size() > 3 &&
           safety_pose_history_[1].stamp_ns < history_cutoff_ns)
      safety_pose_history_.pop_front();

    std::vector<double> history_times;
    std::vector<Eigen::Vector2d> history_positions;
    std::vector<double> history_yaws;
    history_times.reserve(safety_pose_history_.size());
    history_positions.reserve(safety_pose_history_.size());
    history_yaws.reserve(safety_pose_history_.size());
    for (const SafetyPoseSample &sample : safety_pose_history_)
    {
      history_times.push_back(static_cast<double>(sample.stamp_ns) * 1e-9);
      history_positions.push_back(sample.position_world.head<2>());
      history_yaws.push_back(sample.unwrapped_yaw);
    }
    Eigen::Vector2d pose_velocity_world = Eigen::Vector2d::Zero();
    double pose_yaw_rate = 0.0;
    const bool have_pose_motion = estimatePlanarMotionFromPoseHistory(
        history_times, history_positions, history_yaws,
        &pose_velocity_world, &pose_yaw_rate);
    Eigen::Vector3d measured_velocity_world = twist_velocity_world;
    if (have_pose_motion)
    {
      measured_velocity_world.head<2>() = pose_velocity_world;
      // z velocity does not participate in planar braking prediction. Keep
      // the transformed twist value rather than differentiating gait bounce.
    }

    if (!safety_have_odom_ || safety_odom_velocity_stamp_ns_ == 0 ||
        stamp_ns <= safety_odom_velocity_stamp_ns_)
    {
      safety_odom_vel_ = measured_velocity_world;
      safety_yaw_rate_ = have_pose_motion ? pose_yaw_rate
                                          : angular_z;
    }
    else
    {
      const double dt = std::min(
          0.25, static_cast<double>(stamp_ns - safety_odom_velocity_stamp_ns_) *
                    1e-9);
      const double alpha = exponentialFilterAlpha(
          dt, odom_velocity_filter_tau_);
      safety_odom_vel_ +=
          alpha * (measured_velocity_world - safety_odom_vel_);
      const double yaw_alpha = exponentialFilterAlpha(
          dt, yaw_rate_filter_tau_);
      const double measured_yaw_rate = have_pose_motion
          ? pose_yaw_rate : angular_z;
      safety_yaw_rate_ +=
          yaw_alpha * (measured_yaw_rate - safety_yaw_rate_);
    }
    safety_odom_velocity_stamp_ns_ = stamp_ns;
    safety_odom_pos_ = position_world;
    safety_odom_orient_ = world_from_body;
    safety_have_odom_ = true;

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[SAFETY_MOTION_ESTIMATE] source=%s world_v=(%.2f,%.2f)m/s yaw_rate=%.2frad/s twist_world=(%.2f,%.2f)m/s samples=%zu",
        have_pose_motion ? "pose_history" : "rotated_twist",
        safety_odom_vel_.x(), safety_odom_vel_.y(), safety_yaw_rate_,
        twist_velocity_world.x(), twist_velocity_world.y(),
        safety_pose_history_.size());
  }

  void SCANReplanFSM::refreshPlanningOdomFromSafety()
  {
    std::lock_guard<std::mutex> lock(safety_odom_mutex_);
    if (!safety_have_odom_)
      return;
    odom_pos_ = safety_odom_pos_;
    odom_vel_ = safety_odom_vel_;
    odom_orient_ = safety_odom_orient_;
    have_odom_ = true;
  }

  void SCANReplanFSM::executionStateCallback(
      const scan_planner_msgs::msg::ExecutionState::ConstSharedPtr &msg)
  {
    {
      std::unique_lock<std::mutex> pending_lock(pending_handoff_mutex_);
      if (pending_handoff_.valid && pending_handoff_.timeout_hold_issued)
      {
        uint64_t previous_request = 0;
        int64_t previous_trajectory = 0;
        {
          std::lock_guard<std::mutex> execution_lock(execution_snapshot_mutex_);
          if (execution_snapshot_.valid)
          {
            previous_request = execution_snapshot_.request_id;
            previous_trajectory = execution_snapshot_.trajectory_id;
          }
        }
        const auto outcome = reconcileTimedOutHandoff(
            true,
            msg->state == scan_planner_msgs::msg::ExecutionState::STATE_SOFT_HOLD,
            msg->reason == "CONTROLLER_ACK_UNKNOWN",
            msg->request_id, msg->trajectory_id,
            pending_handoff_.request_id,
            pending_handoff_.trajectory_id,
            previous_request, previous_trajectory);
        if (outcome == TimedOutHandoffExecution::CANDIDATE_HELD ||
            (outcome == TimedOutHandoffExecution::PREVIOUS_HELD &&
             pending_handoff_.timeout_candidate_rejected))
        {
          const PendingTrajectoryHandoff handoff = pending_handoff_;
          if (outcome == TimedOutHandoffExecution::CANDIDATE_HELD)
          {
            planner_manager_->local_data_ = handoff.candidate;
            updateExecutionTrajectorySnapshot(
                handoff.candidate, handoff.request_id,
                handoff.clearance_escape_active,
                handoff.clearance_escape_deadline,
                handoff.initial_clearance_violations,
                handoff.fixed_body_yaw, handoff.body_yaw,
                handoff.execution_mode);
          }
          else
          {
            planner_manager_->local_data_ = handoff.previous;
          }
          pending_handoff_.valid = false;
          pending_lock.unlock();
          safety_stop_active_.store(true);
          stopped_local_repair_pending_.store(true);
          predictive_replan_requested_.store(false);
          collision_interval_replan_requested_.store(false);
          {
            std::lock_guard<std::mutex> interval_lock(execution_snapshot_mutex_);
            predicted_collision_interval_.valid = false;
          }
          RCLCPP_ERROR(
              node_->get_logger(),
              "[TRAJECTORY_ACK_TIMEOUT_RECONCILED_HOLD] request=%llu trajectory=%lld candidate_installed=%d; stopped recovery required",
              static_cast<unsigned long long>(msg->request_id),
              static_cast<long long>(msg->trajectory_id),
              outcome == TimedOutHandoffExecution::CANDIDATE_HELD);
        }
      }
    }
    uint64_t executing_request = 0;
    int64_t executing_trajectory = 0;
    bool valid = false;
    {
      std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
      valid = execution_snapshot_.valid;
      executing_request = execution_snapshot_.request_id;
      executing_trajectory = execution_snapshot_.trajectory_id;
    }
    if (!valid || !executionStateMatchesExecution(
            msg->request_id, msg->trajectory_id,
            executing_request, executing_trajectory))
    {
      // The controller publishes its idle snapshot before SCAN has accepted a
      // reference.  It carries no command and is not a stale execution fault.
      if (!valid && msg->request_id == 0 && msg->trajectory_id == 0 &&
          msg->state == scan_planner_msgs::msg::ExecutionState::STATE_IDLE)
        return;
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[STALE_EXECUTION_STATE_IGNORED] state_request=%llu state_trajectory=%lld executing_request=%llu executing_trajectory=%lld state=%u",
          static_cast<unsigned long long>(msg->request_id),
          static_cast<long long>(msg->trajectory_id),
          static_cast<unsigned long long>(executing_request),
          static_cast<long long>(executing_trajectory),
          static_cast<unsigned int>(msg->state));
      return;
    }
    go2_execution_frozen_.store(
        msg->state == scan_planner_msgs::msg::ExecutionState::STATE_HEADING_FROZEN ||
        msg->state == scan_planner_msgs::msg::ExecutionState::STATE_SOFT_HOLD ||
        msg->state == scan_planner_msgs::msg::ExecutionState::STATE_HARD_STOP);
    if (std::isfinite(msg->execution_time))
    {
      std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
      if (execution_snapshot_.valid && execution_snapshot_.request_id == msg->request_id &&
          execution_snapshot_.trajectory_id == msg->trajectory_id)
        execution_snapshot_.controller_execution_time = std::clamp(
            msg->execution_time, 0.0, execution_snapshot_.duration);
    }

    if (msg->state == scan_planner_msgs::msg::ExecutionState::STATE_STALLED &&
        msg->reason == "TERMINAL_NO_PROGRESS")
    {
      terminal_repair_error_.store(msg->terminal_error);
      terminal_repair_requested_.store(true);
    }

    const bool structured_feedback_terminal =
        msg->state == scan_planner_msgs::msg::ExecutionState::STATE_FINISHED ||
        msg->state == scan_planner_msgs::msg::ExecutionState::STATE_STALLED;
    if (structuredRepairTerminalFeedbackMatches(
            structured_local_repair_executing_.load(),
            structured_local_repair_request_id_.load(),
            structured_local_repair_trajectory_id_.load(),
            msg->request_id, msg->trajectory_id,
            structured_feedback_terminal))
    {
      structured_local_repair_finished_.store(true);
      RCLCPP_INFO(
          node_->get_logger(),
          "[STRUCTURED_LOCAL_REPAIR_EXECUTION_TERMINAL] request_id=%llu trajectory=%lld state=%u terminal_error=%.3fm; retaining recovery ownership until reference rejoin",
          static_cast<unsigned long long>(msg->request_id),
          static_cast<long long>(msg->trajectory_id),
          static_cast<unsigned int>(msg->state), msg->terminal_error);
    }
  }

  void SCANReplanFSM::trajectoryAckCallback(
      const scan_planner_msgs::msg::TrajectoryAck::ConstSharedPtr &msg)
  {
    PendingTrajectoryHandoff handoff;
    std::unique_lock<std::mutex> handoff_lock(pending_handoff_mutex_);
    if (!pending_handoff_.valid ||
        pending_handoff_.request_id != msg->request_id ||
        pending_handoff_.trajectory_id != msg->trajectory_id)
    {
      // Emergency-stop splines are committed synchronously because safety
      // must not wait for a round trip.  Their controller ACK therefore has
      // no pending transaction, but it is not stale if it names the currently
      // executing snapshot exactly.
      bool acknowledges_current_execution = false;
      {
        std::lock_guard<std::mutex> execution_lock(execution_snapshot_mutex_);
        acknowledges_current_execution =
            execution_snapshot_.valid &&
            execution_snapshot_.request_id == msg->request_id &&
            execution_snapshot_.trajectory_id == msg->trajectory_id;
      }
      if (!acknowledges_current_execution)
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 1000,
            "[STALE_TRAJECTORY_ACK_IGNORED] request_id=%llu trajectory=%lld status=%u",
            static_cast<unsigned long long>(msg->request_id),
            static_cast<long long>(msg->trajectory_id),
            static_cast<unsigned int>(msg->status));
      return;
    }
    handoff = pending_handoff_;
    if (handoff.timeout_hold_issued)
    {
      if (msg->status == scan_planner_msgs::msg::TrajectoryAck::STATUS_REJECTED)
        pending_handoff_.timeout_candidate_rejected = true;
      RCLCPP_ERROR(
          node_->get_logger(),
          "[LATE_TRAJECTORY_ACK_UNCONFIRMED] request=%llu trajectory=%lld status=%u; preserving timeout HOLD until execution-state reconciliation",
          static_cast<unsigned long long>(msg->request_id),
          static_cast<long long>(msg->trajectory_id),
          static_cast<unsigned int>(msg->status));
      return;
    }

    if (msg->status != scan_planner_msgs::msg::TrajectoryAck::STATUS_ACCEPTED)
    {
      planner_manager_->local_data_ = handoff.previous;
      if (handoff.collision_interval_repair)
      {
        collision_interval_replan_requested_.store(true);
        predictive_replan_requested_.store(true);
      }
      else
      {
        predictive_replan_requested_.store(true);
      }
      const bool recovery_handoff =
          handoff.structured_repair_segment ||
          handoff.structured_repair_rejoin;
      if (structuredRepairHandoffRetainsOwnership(
              handoff.structured_repair_segment,
              handoff.structured_repair_rejoin, false))
      {
        structured_local_repair_active_.store(true);
        if (handoff.structured_repair_segment)
        {
          stopped_local_repair_pending_.store(true);
          stopped_local_repair_active_.store(true);
        }
        else
        {
          // The escape already finished, but Controller did not accept the
          // rejoin candidate.  Keep the transaction in its rejoin phase.
          structured_local_repair_executing_.store(false);
          structured_local_repair_rejoin_pending_.store(true);
        }
        ++replan_fail_count_;
      }
      // Clear pending only after the previous planning state is restored.
      // Until this assignment, execFSMCallback remains unable to start a new
      // plan and cannot observe a half-committed handoff.
      pending_handoff_.valid = false;
      handoff_lock.unlock();
      RCLCPP_WARN(
          node_->get_logger(),
          "[CANDIDATE_TRAJECTORY_REJECTED] request_id=%llu trajectory=%lld reason=%s position_error=%.3fm yaw_error=%.3frad; active execution snapshot preserved",
          static_cast<unsigned long long>(msg->request_id),
          static_cast<long long>(msg->trajectory_id), msg->reason.c_str(),
          msg->position_error, msg->yaw_error);
      if (recovery_handoff &&
          replan_fail_count_ >= max_replan_fail_count_)
        finishProcess();
      return;
    }

    planner_manager_->local_data_ = handoff.candidate;
    if (handoff.collision_interval_repair)
    {
      std::lock_guard<std::mutex> interval_lock(execution_snapshot_mutex_);
      if (collisionIntervalMatchesExecution(
              predicted_collision_interval_,
              handoff.collision_interval_request_id,
              handoff.collision_interval_trajectory_id,
              handoff.collision_interval_map_revision))
      {
        predicted_collision_interval_.valid = false;
        collision_interval_replan_requested_.store(false);
      }
    }
    updateExecutionTrajectorySnapshot(
        handoff.candidate, handoff.request_id,
        handoff.clearance_escape_active,
        handoff.clearance_escape_deadline,
        handoff.initial_clearance_violations,
        handoff.fixed_body_yaw, handoff.body_yaw,
        handoff.execution_mode);
    if (handoff.structured_repair_segment)
    {
      structured_local_repair_active_.store(true);
      structured_local_repair_executing_.store(true);
      structured_local_repair_finished_.store(false);
      structured_local_repair_rejoin_pending_.store(false);
      structured_local_repair_interrupted_.store(false);
      structured_local_repair_request_id_.store(handoff.request_id);
      structured_local_repair_trajectory_id_.store(handoff.trajectory_id);
      stopped_local_repair_pending_.store(false);
      stopped_local_repair_active_.store(false);
      predictive_hold_latched_.store(false);
      predictive_hold_release_cycles_.store(0);
      predictive_replan_requested_.store(false);
      tracking_recovery_active_ = false;
      const int committed_segments =
          structured_local_repair_segments_committed_.fetch_add(1) + 1;
      // Do not reset a transaction-wide budget here.  The previous behavior
      // let every accepted 0.45 m segment purchase a fresh retry family,
      // producing an unbounded left/right crawl when every rejoin failed.
      RCLCPP_INFO(
          node_->get_logger(),
          "[STRUCTURED_LOCAL_REPAIR_BUDGET] request_id=%llu committed=%d/%d",
          static_cast<unsigned long long>(handoff.request_id),
          committed_segments, local_repair_max_segments_per_transaction_);
      setLocalRecoveryState(
          LocalRecoveryState::LOCAL_REPAIR, 0.0,
          "structured_repair_segment_acknowledged");
    }
    else if (structuredRepairHandoffCompletesRecovery(
                 handoff.structured_repair_segment,
                 handoff.structured_repair_rejoin, true))
    {
      // This exact candidate is now the controller's execution version.  The
      // recovery transaction ends here, not when SCAN merely generated or
      // published the candidate.
      structured_local_repair_active_.store(false);
      structured_local_repair_executing_.store(false);
      structured_local_repair_finished_.store(false);
      structured_local_repair_rejoin_pending_.store(false);
      structured_local_repair_interrupted_.store(false);
      structured_local_repair_request_id_.store(0);
      structured_local_repair_trajectory_id_.store(0);
      structured_local_repair_segments_committed_.store(0);
      stopped_local_repair_pending_.store(false);
      stopped_local_repair_active_.store(false);
      predictive_replan_requested_.store(false);
      tracking_recovery_active_ = false;
      replan_fail_count_ = 0;
      setLocalRecoveryState(
          LocalRecoveryState::TRACKING, 0.0,
          "structured_repair_rejoin_acknowledged");
      RCLCPP_INFO(
          node_->get_logger(),
          "[STRUCTURED_LOCAL_REPAIR_COMPLETE] request_id=%llu trajectory=%lld; controller accepted the reference-path rejoin",
          static_cast<unsigned long long>(handoff.request_id),
          static_cast<long long>(handoff.trajectory_id));
    }
    else if (!structured_local_repair_active_.load())
    {
      predictive_replan_requested_.store(false);
      tracking_recovery_active_ = false;
      replan_fail_count_ = 0;
      setLocalRecoveryState(
          LocalRecoveryState::TRACKING, 0.0,
          "candidate_trajectory_acknowledged");
    }

    const bool released_terminal_repair_hold =
        terminal_local_repair_hold_.exchange(false);
    if (newPlanMayReleaseSafetyStop(
            pose_occupied_latched_.load(), predictive_hold_latched_.load(),
            temporal_safety_latched_.load()) &&
        !invalid_odom_latched_.load() &&
        safety_stop_active_.exchange(false))
    {
      RCLCPP_INFO(
          node_->get_logger(),
          "[SAFETY_RELEASE] source=controller_handoff_ack request_id=%llu trajectory=%lld clearance_escape=%d",
          static_cast<unsigned long long>(handoff.request_id),
          static_cast<long long>(handoff.trajectory_id),
          handoff.clearance_escape_active ? 1 : 0);
    }
    if (released_terminal_repair_hold)
      RCLCPP_INFO(
          node_->get_logger(),
          "[LOCAL_REPAIR_TERMINAL_HOLD_RELEASED] request_id=%llu trajectory=%lld source=controller_handoff_ack",
          static_cast<unsigned long long>(handoff.request_id),
          static_cast<long long>(handoff.trajectory_id));

    if (handoff.reference_path_update && reference_path_update_pending_ &&
        pending_reference_request_id_ == handoff.reference_request_id)
    {
      publishReferenceStatus(
          "PATH_TRAJECTORY_READY", handoff.reference_request_id);
      reference_path_update_pending_ = false;
      pending_reference_request_id_ = 0;
    }
    // The candidate, execution snapshot, latch ownership, and reference-path
    // terminal status now describe one version.  Only now may planning resume.
    pending_handoff_.valid = false;
    handoff_lock.unlock();
    // RUNNING is an execution fact, not a publication fact.  Keep the FSM in
    // GEN_NEW_TRAJ/REPLAN_TRAJ while the candidate is pending and expose the
    // transition only after Controller accepted this exact version.
    changeFSMExecState(EXEC_TRAJ, "TRAJECTORY_ACK");
    RCLCPP_INFO(
        node_->get_logger(),
        "[CANDIDATE_TRAJECTORY_COMMITTED] request_id=%llu trajectory=%lld matched_time=%.3fs position_error=%.3fm",
        static_cast<unsigned long long>(handoff.request_id),
        static_cast<long long>(handoff.trajectory_id), msg->matched_time,
        msg->position_error);
  }

  void SCANReplanFSM::go2HeadingStalledCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_heading_stalled_.store(msg->data);
  }

  void SCANReplanFSM::go2HeadingErrorCallback(const std_msgs::msg::Float64::ConstSharedPtr &msg)
  {
    go2_heading_error_.store(msg->data);
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

  void SCANReplanFSM::publishFailureEvidence(
      uint64_t request_id, int64_t trajectory_id,
      const std::string &stage, const std::string &reason,
      const Eigen::Vector3d &position, uint32_t attempted_candidates,
      double required_clearance, ClearanceFailure clearance_failure,
      uint8_t failure_scope)
  {
    scan_planner_msgs::msg::FailureEvidence evidence;
    evidence.stamp = node_->now();
    evidence.request_id = request_id;
    evidence.trajectory_id = trajectory_id;
    evidence.stage = stage;
    evidence.reason = reason;
    evidence.position.x = position.x();
    evidence.position.y = position.y();
    evidence.position.z = position.z();
    evidence.attempted_candidates = attempted_candidates;
    evidence.required_clearance = required_clearance;
    evidence.clearance_failure = static_cast<uint8_t>(clearance_failure);
    evidence.failure_scope = failure_scope;
    failure_evidence_pub_->publish(evidence);
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

  void SCANReplanFSM::requestExecutionStop(
      const std::string &reason, bool publish_reference_status)
  {
    scan_planner_msgs::msg::ExecutionCommand msg;
    msg.command = scan_planner_msgs::msg::ExecutionCommand::COMMAND_HOLD;
    msg.reason = reason;
    {
      std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
      if (execution_snapshot_.valid)
      {
        msg.request_id = execution_snapshot_.request_id;
        msg.trajectory_id = execution_snapshot_.trajectory_id;
      }
      else
      {
        msg.request_id = active_reference_request_id_.load();
        msg.trajectory_id = 0;
      }
    }
    const int64_t now_ns = node_->now().nanoseconds();
    bool publish_command = true;
    {
      std::lock_guard<std::mutex> lock(execution_command_mutex_);
      // HOLD is level-triggered at the controller.  Once a version is held,
      // changing the diagnostic reason does not require another command.  A
      // newer trajectory id naturally creates a new command owner.
      const bool duplicate = holdCommandAlreadyIssuedForVersion(
          msg.request_id, msg.trajectory_id,
          last_execution_command_request_id_,
          last_execution_command_trajectory_id_);
      if (duplicate)
        publish_command = false;
      else
      {
        last_execution_command_request_id_ = msg.request_id;
        last_execution_command_trajectory_id_ = msg.trajectory_id;
        last_execution_command_reason_ = reason;
        last_execution_command_time_ns_ = now_ns;
      }
    }
    if (publish_command)
    {
      execution_command_pub_->publish(msg);
      RCLCPP_INFO(
          node_->get_logger(),
          "[EXECUTION_COMMAND] command=HOLD request_id=%llu trajectory=%lld reason=%s",
          static_cast<unsigned long long>(msg.request_id),
          static_cast<long long>(msg.trajectory_id), reason.c_str());
    }
    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH && publish_reference_status)
    {
      if (reason == "REACHED")
      {
        completed_reference_request_id_.store(
            active_reference_request_id_.load());
        last_wait_target_status_ns_.store(0);
      }
      publishReferenceStatus(reason);
    }
    else if (publish_reference_status)
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
    last_fsm_callback_wall_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    fsm_callback_stall_reported_.store(false);
    uint64_t timed_out_request = 0;
    int64_t timed_out_trajectory = 0;
    bool reconciliation_timed_out = false;
    PendingTrajectoryHandoff unresolved_handoff;
    {
      std::lock_guard<std::mutex> lock(pending_handoff_mutex_);
      if (pending_handoff_.valid && !pending_handoff_.timeout_hold_issued)
      {
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() -
            pending_handoff_.submitted_at).count();
        if (trajectoryHandoffTimedOut(
                true, elapsed, trajectory_ack_timeout_))
        {
          pending_handoff_.timeout_hold_issued = true;
          pending_handoff_.timeout_hold_issued_at =
              std::chrono::steady_clock::now();
          timed_out_request = pending_handoff_.request_id;
          timed_out_trajectory = pending_handoff_.trajectory_id;
        }
      }
      else if (pending_handoff_.valid &&
               pending_handoff_.timeout_hold_issued)
      {
        const double elapsed_since_hold = std::chrono::duration<double>(
            std::chrono::steady_clock::now() -
            pending_handoff_.timeout_hold_issued_at).count();
        if (trajectoryHandoffReconciliationTimedOut(
                true, true, elapsed_since_hold,
                trajectory_ack_reconciliation_timeout_))
        {
          unresolved_handoff = pending_handoff_;
          pending_handoff_.valid = false;
          reconciliation_timed_out = true;
        }
      }
    }
    if (timed_out_request != 0)
    {
      scan_planner_msgs::msg::ExecutionCommand command;
      command.command = scan_planner_msgs::msg::ExecutionCommand::COMMAND_HOLD;
      command.request_id = timed_out_request;
      command.trajectory_id = timed_out_trajectory;
      command.reason = "CONTROLLER_ACK_UNKNOWN";
      execution_command_pub_->publish(command);
      RCLCPP_ERROR(node_->get_logger(),
                   "[TRAJECTORY_ACK_UNKNOWN_HOLD] request=%llu trajectory=%lld; awaiting controller state",
                   static_cast<unsigned long long>(timed_out_request),
                   static_cast<long long>(timed_out_trajectory));
    }
    if (reconciliation_timed_out)
    {
      // The timeout HOLD targets the candidate version and therefore stops
      // either possible execution owner.  Once Controller fails to report an
      // exact version within the second bounded window, discard the candidate
      // transaction and recover from live odometry instead of occupying the
      // planning FSM forever.
      LocalTrajData recovery_base = unresolved_handoff.previous;
      // The Controller may have accepted the candidate even though both ACK
      // and execution-state confirmation were lost.  Reserve that trajectory
      // id so the next recovery candidate is strictly newer in either case.
      recovery_base.traj_id_ = reserveTrajectoryIdAfterAmbiguousHandoff(
          recovery_base.traj_id_, unresolved_handoff.trajectory_id);
      planner_manager_->local_data_ = recovery_base;
      scan_planner_msgs::msg::ExecutionCommand command;
      command.command =
          scan_planner_msgs::msg::ExecutionCommand::COMMAND_HOLD;
      command.request_id = unresolved_handoff.request_id;
      command.trajectory_id = unresolved_handoff.trajectory_id;
      command.reason = "CONTROLLER_ACK_UNRESOLVED";
      execution_command_pub_->publish(command);
      safety_stop_active_.store(true);
      stopped_local_repair_pending_.store(true);
      predictive_replan_requested_.store(true);
      collision_interval_replan_requested_.store(false);
      {
        std::lock_guard<std::mutex> interval_lock(execution_snapshot_mutex_);
        predicted_collision_interval_.valid = false;
      }
      RCLCPP_ERROR(
          node_->get_logger(),
          "[TRAJECTORY_ACK_RECONCILIATION_TIMEOUT] request=%llu trajectory=%lld; candidate transaction aborted, HOLD retained, recovery will restart from live odometry",
          static_cast<unsigned long long>(unresolved_handoff.request_id),
          static_cast<long long>(unresolved_handoff.trajectory_id));
    }
    {
      std::lock_guard<std::mutex> lock(pending_handoff_mutex_);
      if (pending_handoff_.valid)
      {
        // The planning timer must not advance trajectory time, trigger another
        // replan, or complete a request while Controller still owns the old
        // execution version.  Odometry and real-time safety use independent
        // callback groups and continue running during this bounded wait.
        return;
      }
    }
    if (activateDeferredReferencePathIfReady())
      return;
    refreshPlanningOdomFromSafety();
    if (have_odom_)
    {
      if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
      {
        rviz_goal_height_ = odom_pos_(2);
        rviz_height_ready_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "Set RViz goal height from initial body_pose z: %.3f",
                    rviz_goal_height_);
      }
      publishSelfInflationMarker();
      if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
      {
        preset_started_ = true;
        planGlobalTrajbyGivenWps();
      }
    }
    updateLocalTrajTimeFreeze();
    if (!go2_heading_stalled_.load())
      heading_stall_handled_ = false;

    // A real-time veto is a latch, not a momentary warning.  The planning
    // group must not reuse the rejected reference path while the Explorer's
    // replacement path is still queued behind this callback.
    const bool replacement_planning_pending =
        navi_mode_ == NAVI_MODE::REFERENCE_PATH &&
        reference_path_update_pending_ && have_target_ &&
        (exec_state_ == GEN_NEW_TRAJ || exec_state_ == REPLAN_TRAJ);
    double safety_planar_speed = 0.0;
    {
      std::lock_guard<std::mutex> lock(safety_odom_mutex_);
      safety_planar_speed = safety_odom_vel_.head<2>().norm();
    }
    if (safety_stop_active_.load() && shouldStartStoppedLocalRepair(
            stopped_local_repair_pending_.load(), have_target_,
            safety_planar_speed, pose_occupied_latched_.load(),
            temporal_safety_latched_.load() || invalid_odom_latched_.load()))
    {
      stopped_local_repair_pending_.store(false);
      stopped_local_repair_active_.store(true);
      tracking_recovery_active_ = false;
      predictive_replan_requested_.store(false);
      replan_fail_count_ = 0;
      next_rolling_replan_attempt_ns_ = 0;
      have_new_target_ = true;
      refreshPlanningOdomFromSafety();
      setLocalRecoveryState(
          LocalRecoveryState::LOCAL_REPAIR, 0.0,
          "hard_stop_stationary_replan");
      changeFSMExecState(GEN_NEW_TRAJ, "LOCAL_REPAIR_FROM_REST");
      RCLCPP_WARN(
          node_->get_logger(),
          "[LOCAL_REPAIR_FROM_REST] request_id=%llu speed=%.3fm/s; retaining the Explorer target and generating validated local alternatives",
          static_cast<unsigned long long>(active_reference_request_id_.load()),
          safety_planar_speed);
      return;
    }
    const bool emergency_recovery_pending =
        exec_state_ == EMERGENCY_STOP && emergency_path_pending_ && have_target_;
    const bool stopped_repair_planning =
        stopped_local_repair_active_.load() &&
        (exec_state_ == GEN_NEW_TRAJ || exec_state_ == REPLAN_TRAJ);
    if (safety_stop_active_.load() && !safetyLatchAllowsPlanning(
            replacement_planning_pending, emergency_recovery_pending,
            stopped_repair_planning))
    {
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[SAFETY_LATCHED] waiting for a validated replacement trajectory");
      return;
    }

    // A published structured repair owns execution until feedback for that
    // exact request/trajectory version says it is terminal.  Only then may the
    // FSM try to rejoin the unchanged reference path.  Ordinary predictive
    // rolling replans must not interrupt the short repair halfway through.
    if (structured_local_repair_active_.load() &&
        structured_local_repair_executing_.load() &&
        structured_local_repair_finished_.exchange(false) &&
        exec_state_ == EXEC_TRAJ && have_target_)
    {
      structured_local_repair_executing_.store(false);
      structured_local_repair_rejoin_pending_.store(true);
      predictive_replan_requested_.store(false);
      terminal_repair_requested_.store(false);
      next_rolling_replan_attempt_ns_ = 0;
      refreshPlanningOdomFromSafety();
      changeFSMExecState(REPLAN_TRAJ, "STRUCTURED_REPAIR_FINISHED");
      RCLCPP_INFO(
          node_->get_logger(),
          "[STRUCTURED_LOCAL_REPAIR_REJOIN] request_id=%llu trajectory=%lld; repair finished, rejoining the unchanged reference path",
          static_cast<unsigned long long>(
              structured_local_repair_request_id_.load()),
          static_cast<long long>(
              structured_local_repair_trajectory_id_.load()));
      return;
    }

    // The controller has exhausted the active spline but the robot is still
    // outside its terminal tolerance.  This is a local execution failure, not
    // evidence that the Explorer's committed region or reference path is
    // invalid.  Regenerate a candidate from live odometry and keep the global
    // commitment intact.  The controller holds the last active trajectory at
    // zero command until that candidate passes the normal handoff checks.
    if (!structured_local_repair_executing_.load() &&
        terminal_repair_requested_.load() &&
        exec_state_ == EXEC_TRAJ && have_target_)
    {
      terminal_repair_requested_.store(false);
      const double terminal_error = terminal_repair_error_.load();
      refreshPlanningOdomFromSafety();
      predictive_replan_requested_.store(true);
      tracking_recovery_active_ = true;
      next_rolling_replan_attempt_ns_ = 0;
      setLocalRecoveryState(
          LocalRecoveryState::LOCAL_REPAIR, terminal_error,
          "terminal_no_progress");
      changeFSMExecState(REPLAN_TRAJ, "TERMINAL_NO_PROGRESS");
      RCLCPP_WARN(
          node_->get_logger(),
          "[TERMINAL_LOCAL_REPAIR] terminal_error=%.3fm; regenerating from current odometry while preserving the committed reference path",
          terminal_error);
      return;
    }

    // A collision still outside the dynamic stopping horizon is a planning
    // request, not a terminal failure.  Replan from current odometry while the
    // validated prefix remains executable; the safety timer will escalate to
    // a hard HOLD if the obstacle enters the stopping horizon first.
    if (ordinaryRollingReplanAllowed(
            structured_local_repair_executing_.load(),
            predictive_replan_requested_.load()) &&
        exec_state_ == EXEC_TRAJ &&
        have_target_ &&
        (next_rolling_replan_attempt_ns_ == 0 ||
         node_->now().nanoseconds() >= next_rolling_replan_attempt_ns_))
    {
      refreshPlanningOdomFromSafety();
      tracking_recovery_active_ =
          local_recovery_state_.load() !=
          static_cast<uint8_t>(LocalRecoveryState::TRACKING);
      next_rolling_replan_attempt_ns_ = 0;
      changeFSMExecState(REPLAN_TRAJ, "PREDICTIVE_COLLISION");
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[PREDICTIVE_COLLISION_REPLAN] replanning from current odometry before the stopping horizon");
      return;
    }

    if (go2_heading_stalled_.load() && !heading_stall_handled_ &&
        exec_state_ == EXEC_TRAJ && have_target_)
    {
      heading_stall_handled_ = true;
      tracking_recovery_active_ = true;
      heading_freeze_recoveries_++;
      refreshPlanningOdomFromSafety();
      RCLCPP_WARN(
          node_->get_logger(),
          "[HEADING_FREEZE_RECOVERY] request_id=%llu attempt=%d/%d yaw_error=%.3frad; replanning from current odometry",
          static_cast<unsigned long long>(active_reference_request_id_.load()),
          heading_freeze_recoveries_, heading_freeze_max_recoveries_,
          go2_heading_error_.load());

      if (heading_freeze_recoveries_ <= heading_freeze_max_recoveries_)
      {
        next_rolling_replan_attempt_ns_ = 0;
        changeFSMExecState(REPLAN_TRAJ, "HEADING_FREEZE_STALLED");
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "[HEADING_FREEZE_ABORT] request_id=%llu recovery budget exhausted; returning BLOCKED to Explorer",
                     static_cast<unsigned long long>(active_reference_request_id_.load()));
        have_target_ = false;
        reference_path_active_ = false;
        reference_path_update_pending_ = false;
        requestExecutionStop("BLOCKED");
        tracking_recovery_active_ = false;
        changeFSMExecState(WAIT_TARGET, "HEADING_FREEZE_ABORT");
      }
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
      // REACHED is a one-shot event.  WAIT_TARGET is its durable state and is
      // repeated at low rate so Explorer can recover a missed completion.
      const uint64_t completed_id = completed_reference_request_id_.load();
      if (!have_target_ && completed_id != 0)
      {
        const int64_t now_ns = node_->now().nanoseconds();
        const int64_t previous_ns = last_wait_target_status_ns_.load();
        if (previous_ns == 0 || now_ns - previous_ns >= 500000000LL)
        {
          last_wait_target_status_ns_.store(now_ns);
          publishReferenceStatus("WAIT_TARGET", completed_id);
        }
      }
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

      // A stopped local repair is a different planning problem from normal
      // rolling replanning.  Go directly to the bounded, low-speed escape
      // candidates instead of repeatedly accepting an ordinary trajectory
      // whose future prefix merely promises to leave the clearance belt.
      const bool stopped_structured_repair =
          stopped_local_repair_active_.load();
      bool success = stopped_structured_repair
                         ? tryStructuredLocalRepair()
                         : callReboundReplan(true, flag_random_poly_init);
      if (!success && !stopped_structured_repair && flag_random_poly_init &&
          reference_path_active_ &&
          (emergency_path_pending_ || safety_stop_active_.load()))
        success = tryStructuredLocalRepair();
      if (success)
      {
        // callReboundReplan() has only submitted a candidate.  The matching
        // TrajectoryAck callback owns every recovery-state transition; doing
        // it here races the ACK and used to erase recovery ownership before
        // the escape/rejoin transaction was complete.
        next_rolling_replan_attempt_ns_ = 0;
        collision_segment_pending_ = false;
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
        finishGoalReached("GOAL_REACHED");
        return;
      }

      if (planFromCurrentTraj())
      {
        // Submission is phase one only.  In particular, a recovery rejoin
        // remains owned by LOCAL_REPAIR until Controller ACKs this exact
        // request/trajectory version.
        next_rolling_replan_attempt_ns_ = 0;
        collision_segment_pending_ = false;
      }
      else
      {
        const auto *info = &planner_manager_->local_data_;
        const double t_cur = executionProgressTime(*info);
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
      if (structured_local_repair_executing_.load())
      {
        // The controller's versioned FINISHED/STALLED feedback above is the
        // sole normal completion signal for this recovery segment.  Safety
        // callbacks remain active and can still issue a hard stop.
        predictive_replan_requested_.store(false);
      }
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = executionProgressTime(*info);
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (!isWaypointSequenceMode() &&
          (end_pt_.head<2>() - odom_pos_.head<2>()).norm() <= goal_tolerance_)
      {
        finishGoalReached("GOAL_REACHED");
        return;
      }

      if (structured_local_repair_executing_.load())
      {
        // The repair endpoint is a local anchor, not the global goal.  Keep
        // executing this exact version until versioned controller feedback
        // marks it terminal; the safety timer remains independently active.
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

        finishGoalReached("FSM");
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
      else
      {
        const double planned_progress =
            (info->start_pos_.head<2>() - pos.head<2>()).norm();
        const double odometry_progress =
            (info->start_pos_.head<2>() - odom_pos_.head<2>()).norm();
        if (!shouldTriggerRollingReplan(
                planned_progress, odometry_progress, replan_thresh_))
          return;

        RCLCPP_INFO(
            node_->get_logger(),
            "[ROLLING_REPLAN_TRIGGER] planned_progress=%.2fm odometry_progress=%.2fm threshold=%.2fm",
            planned_progress, odometry_progress, replan_thresh_);
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
      const bool stopped_repair_owned =
          stopped_local_repair_active_.exchange(false);
      const bool structured_repair_owned =
          structured_local_repair_active_.exchange(false);
      if (localRepairOwnsFailure(
              stopped_repair_owned, structured_repair_owned))
      {
        const int committed_repair_segments =
            structured_local_repair_segments_committed_.exchange(0);
        const uint32_t attempted_recovery_candidates =
            static_cast<uint32_t>(std::max(
                replan_fail_count_, committed_repair_segments));
        const uint64_t failed_request = active_reference_request_id_.load();
        const int64_t failed_trajectory =
            structured_local_repair_trajectory_id_.load();
        stopped_local_repair_pending_.store(false);
        structured_local_repair_executing_.store(false);
        structured_local_repair_finished_.store(false);
        structured_local_repair_rejoin_pending_.store(false);
        structured_local_repair_interrupted_.store(false);
        structured_local_repair_request_id_.store(0);
        structured_local_repair_trajectory_id_.store(0);
        replan_fail_count_ = 0;
        next_rolling_replan_attempt_ns_ = 0;
        have_target_ = false;
        have_new_target_ = false;
        reference_path_active_ = false;
        reference_path_update_pending_ = false;
        pending_reference_request_id_ = 0;
        emergency_path_pending_ = false;
        tracking_recovery_active_ = false;
        trigger_ = false;
        terminal_local_repair_hold_.store(true);
        predictive_hold_latched_.store(false);
        predictive_hold_release_cycles_.store(0);
        predictive_replan_requested_.store(false);
        {
          std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
          execution_snapshot_.valid = false;
          execution_snapshot_.clearance_escape_active = false;
          execution_snapshot_.clearance_escape_free_cycles = 0;
        }
        publishFailureEvidence(
            failed_request, failed_trajectory,
            "STRUCTURED_LOCAL_REPAIR", "FINITE_CANDIDATE_FAMILY_EXHAUSTED",
            end_pt_, attempted_recovery_candidates,
            planning_clearance_margin_, ClearanceFailure::NONE,
            scan_planner_msgs::msg::FailureEvidence::SCOPE_VIEWPOINT);
        // Publish graph evidence before the terminal state transition.  The
        // graph may update route constraints immediately, while the following
        // status remains the sole owner of request/goal lifecycle changes.
        changeFSMExecState(WAIT_TARGET, "VIEWPOINT_LOCAL_REJECTED");
        requestExecutionStop("VIEWPOINT_LOCAL_REJECTED");
        RCLCPP_ERROR(
            node_->get_logger(),
            "[VIEWPOINT_LOCAL_REJECTED] request_id=%llu attempts=%u; retaining HOLD and returning only the failed viewpoint to Explorer",
            static_cast<unsigned long long>(failed_request),
            attempted_recovery_candidates);
        return;
      }
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
    refreshPlanningOdomFromSafety();
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = executionProgressTime(*info);
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    const bool trajectory_valid =
        info->start_time_.seconds() > 1e-5 && info->duration_ > 1e-5;
    const double remaining_time =
        trajectory_valid ? std::max(0.0, info->duration_ - t_cur) : 0.0;
    const double tracking_error = trajectory_valid
        ? (info->position_traj_.evaluateDeBoorT(t_cur).head<2>() -
           odom_pos_.head<2>()).norm()
        : std::numeric_limits<double>::infinity();
    if (collision_interval_replan_requested_.load() && trajectory_valid)
    {
      PredictedCollisionInterval interval;
      {
        std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
        interval = predicted_collision_interval_;
      }
      const uint64_t current_request = active_reference_request_id_.load();
      const uint64_t current_map_revision =
          planner_manager_->grid_map_->getMapRevision();
      if (interval.kind == CollisionRiskKind::HARD_MARGIN &&
          interval.valid && interval.request_id == current_request &&
          interval.trajectory_id == info->traj_id_ &&
          localPatchSeedIsUsable(
              interval, t_cur, info->duration_, 0.12, 0.30, 0.20,
              current_map_revision) &&
          tracking_error <= rolling_replan_max_start_error_ &&
          (interval.rejoin_position -
           info->position_traj_.evaluateDeBoorT(interval.rejoin_time).head<3>())
                  .head<2>().norm() <= 0.08)
      {
        const double splice_start = interval.repair_start_time;
        const double splice_end = interval.rejoin_time;
        const double remaining_duration = info->duration_ - t_cur;
        const int sample_count = std::max(
            7, static_cast<int>(std::ceil(
                remaining_duration / std::max(
                    0.04, planner_manager_->pp_.ctrl_pt_dist /
                              std::max(0.10, planner_manager_->pp_.max_vel_)))) + 1);
        const double sample_step = remaining_duration / (sample_count - 1);
        const Eigen::Vector3d matched_start =
            info->position_traj_.evaluateDeBoorT(t_cur).head<3>();
        if ((odom_pos_ - matched_start).head<2>().norm() >
            rolling_replan_max_start_error_)
        {
          collision_interval_replan_requested_.store(false);
          RCLCPP_WARN(
              node_->get_logger(),
              "[LOCAL_INTERVAL_REPAIR_STALE_START] odom_to_trajectory=%.3fm limit=%.3fm",
              (odom_pos_ - matched_start).head<2>().norm(),
              rolling_replan_max_start_error_);
          return false;
        }
        const Eigen::Vector3d patch_start =
            info->position_traj_.evaluateDeBoorT(splice_start).head<3>();
        const Eigen::Vector3d rejoin = interval.rejoin_position;
        const double patch_length = (rejoin - patch_start).norm();
        const double minimum_patch_duration = std::max(
            patch_length / std::max(0.10, planner_manager_->pp_.max_vel_),
            std::sqrt(patch_length /
                      std::max(0.10, planner_manager_->pp_.max_acc_)));
        if (splice_end - splice_start < minimum_patch_duration)
        {
          RCLCPP_WARN(
              node_->get_logger(),
              "[LOCAL_INTERVAL_REPAIR_REJECTED] duration=%.2f minimum=%.2f",
              splice_end - splice_start, minimum_patch_duration);
          collision_interval_replan_requested_.store(false);
          return false;
        }
        std::vector<Eigen::Vector3d> seed;
        seed.reserve(sample_count);
        for (int index = 0; index < sample_count; ++index)
        {
          const double t = uniformSeedSampleTime(
              t_cur, info->duration_, static_cast<size_t>(index),
              static_cast<size_t>(sample_count));
          if (index == 0)
          {
            seed.push_back(odom_pos_);
            continue;
          }
          if (t <= splice_start || t >= splice_end)
          {
            seed.push_back(info->position_traj_.evaluateDeBoorT(t).head<3>());
            continue;
          }
          const double ratio = (t - splice_start) /
                               (splice_end - splice_start);
          seed.push_back(patch_start + ratio * (rejoin - patch_start));
        }
        const Eigen::Vector3d final_position =
            info->position_traj_.evaluateDeBoorT(info->duration_).head<3>();

        std::vector<Eigen::Vector3d> derivatives = {
            info->velocity_traj_.evaluateDeBoorT(t_cur).head<3>(),
            info->velocity_traj_.evaluateDeBoorT(info->duration_).head<3>(),
            info->acceleration_traj_.evaluateDeBoorT(t_cur).head<3>(),
            info->acceleration_traj_.evaluateDeBoorT(info->duration_).head<3>()};
        const Eigen::Vector3d saved_start = start_pt_;
        const Eigen::Vector3d saved_velocity = start_vel_;
        const Eigen::Vector3d saved_acceleration = start_acc_;
        const Eigen::Vector3d saved_goal = end_pt_;
        const Eigen::Vector3d saved_target = local_target_pt_;
        const Eigen::Vector3d saved_target_velocity = local_target_vel_;
        start_pt_ = odom_pos_;
        // The seed begins at the measured pose, so its derivatives must come
        // from the same measured state.  safety_odom_vel_ is already filtered
        // in the world frame; using the old reference derivatives here would
        // create a position-continuous but dynamically discontinuous handoff.
        const Eigen::Vector3d measured_start_velocity = odom_vel_;
        const Eigen::Vector3d measured_start_acceleration =
            Eigen::Vector3d::Zero();
        start_vel_ = measured_start_velocity;
        start_acc_ = measured_start_acceleration;
        end_pt_ = info->position_traj_.evaluateDeBoorT(info->duration_).head<3>();
        local_target_pt_ = end_pt_;
        local_target_vel_ = derivatives[1];
        const bool patch_success = trajectorySeedMatchesBoundaries(
            seed, odom_pos_, final_position, 0.02) &&
            callReboundReplan(
                true, false, &local_target_pt_, false, &seed,
                &local_target_vel_, &derivatives[3],
                &measured_start_velocity, &measured_start_acceleration,
                sample_step,
                &interval, t_cur);
        start_pt_ = saved_start;
        start_vel_ = saved_velocity;
        start_acc_ = saved_acceleration;
        end_pt_ = saved_goal;
        local_target_pt_ = saved_target;
        local_target_vel_ = saved_target_velocity;
        if (patch_success)
        {
          bool handoff_pending = false;
          {
            std::lock_guard<std::mutex> lock(pending_handoff_mutex_);
            handoff_pending = pending_handoff_.valid &&
                pending_handoff_.collision_interval_repair &&
                pending_handoff_.collision_interval_request_id ==
                    interval.request_id &&
                pending_handoff_.collision_interval_trajectory_id ==
                    interval.trajectory_id &&
                pending_handoff_.collision_interval_map_revision ==
                    interval.map_revision;
          }
          RCLCPP_INFO(
              node_->get_logger(),
              "[LOCAL_INTERVAL_REPAIR_SUBMITTED] request=%llu base_trajectory=%lld map_revision=%llu interval=[%.2f,%.2f] rejoin=%.2f seed_points=%zu ack_pending=%d",
              static_cast<unsigned long long>(interval.request_id),
              static_cast<long long>(interval.trajectory_id),
              static_cast<unsigned long long>(interval.map_revision),
              interval.entry_time, interval.exit_time, interval.rejoin_time,
              seed.size(), handoff_pending);
          return handoff_pending;
        }
        RCLCPP_WARN(
            node_->get_logger(),
            "[LOCAL_INTERVAL_REPAIR_FAILED] request=%llu trajectory=%lld; trying ordinary rolling replanning",
            static_cast<unsigned long long>(interval.request_id),
            static_cast<long long>(interval.trajectory_id));
      }
      else
      {
        RCLCPP_INFO(
            node_->get_logger(),
            "[LOCAL_INTERVAL_REPAIR_SKIPPED] valid=%d exit_known=%d braking=%d map_revision=%llu current_map_revision=%llu current_t=%.2f",
            interval.valid, interval.exit_known, interval.braking_window,
            static_cast<unsigned long long>(interval.map_revision),
            static_cast<unsigned long long>(current_map_revision), t_cur);
      }
      bool interval_handoff_pending = false;
      {
        std::lock_guard<std::mutex> lock(pending_handoff_mutex_);
        interval_handoff_pending = pending_handoff_.valid &&
                                   pending_handoff_.collision_interval_repair;
      }
      if (!interval_handoff_pending)
      {
        collision_interval_replan_requested_.store(false);
        std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
        predicted_collision_interval_.valid = false;
      }
    }
    const bool reuse_suffix = shouldReuseCurrentTrajectorySuffix(
        reference_path_update_pending_, trajectory_valid, remaining_time,
        tracking_error, rolling_replan_max_start_error_) &&
        !predictive_replan_requested_.load();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;
    if (trajectory_valid)
    {
      // A rolling replacement must preserve the command state of the
      // trajectory currently being executed.  Quadruped odometry velocity
      // oscillates over every gait cycle and can be momentarily zero or
      // backwards even during steady forward motion; sampling that single
      // frame made every replacement spline restart from rest.
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    }
    else
    {
      start_vel_ = odom_vel_;
      start_acc_.setZero();
    }
    if (reference_path_active_)
    {
      // A replacement global path may turn away from the previous local
      // trajectory.  Preserve the previous desired speed only in the new path
      // direction; carrying its lateral component creates a large entry arc
      // before collision optimization begins.
      alignStartStateToReferencePath();
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
    if (callReboundReplan(true, true))
      return true;

    if (reference_path_active_ &&
        (predictive_replan_requested_.load() || tracking_recovery_active_ ||
         stopped_local_repair_active_.load() ||
         structured_local_repair_active_.load() ||
         safety_stop_active_.load()))
      return tryStructuredLocalRepair();
    return false;
  }

  bool SCANReplanFSM::tryStructuredLocalRepair()
  {
    if (!reference_path_active_ || reference_path_.size() < 2 ||
        !have_target_)
      return false;

    // From this point until a validated candidate succeeds (or the bounded
    // retry budget is exhausted), SCAN owns the request as LOCAL_REPAIR.  A
    // failed optimizer call must not silently downgrade it to generic
    // TRACKING_RECOVERY_FAILED/BLOCKED.
    const bool new_recovery_transaction =
        !structured_local_repair_active_.exchange(true);
    if (new_recovery_transaction)
    {
      structured_local_repair_segments_committed_.store(0);
      structured_local_repair_interrupted_.store(false);
    }

    const int committed_segments =
        structured_local_repair_segments_committed_.load();
    if (!structuredRepairMayStartAnotherEscape(
            structured_local_repair_rejoin_pending_.load(), committed_segments))
    {
      replan_fail_count_ = max_replan_fail_count_;
      RCLCPP_ERROR(node_->get_logger(),
          "[STRUCTURED_REJOIN_EXHAUSTED] request=%llu; no repeated escape after a failed rejoin",
          static_cast<unsigned long long>(active_reference_request_id_.load()));
      return false;
    }
    if (!structuredRepairSegmentBudgetAvailable(
            committed_segments,
            local_repair_max_segments_per_transaction_))
    {
      // Escalate directly to the transaction terminal handled by
      // finishProcess().  Re-entering the anchor generator here would merely
      // create another geometrically similar short segment from a new pose.
      replan_fail_count_ = max_replan_fail_count_;
      RCLCPP_ERROR(
          node_->get_logger(),
          "[STRUCTURED_LOCAL_REPAIR_TRANSACTION_EXHAUSTED] request_id=%llu committed=%d/%d; rejecting this viewpoint instead of generating another crawl segment",
          static_cast<unsigned long long>(
              active_reference_request_id_.load()),
          committed_segments, local_repair_max_segments_per_transaction_);
      return false;
    }

    double safety_planar_speed = 0.0;
    {
      std::lock_guard<std::mutex> lock(safety_odom_mutex_);
      safety_planar_speed = safety_odom_vel_.head<2>().norm();
    }
    if (!std::isfinite(safety_planar_speed) || safety_planar_speed > 0.05)
    {
      // A rest-to-rest escape trajectory is only a valid handoff after the
      // physical robot has settled.  Retain this request, latch HOLD, and let
      // the normal stopped-local-repair transition resume planning once odom
      // confirms the speed threshold.
      stopped_local_repair_pending_.store(true);
      stopped_local_repair_active_.store(false);
      if (!safety_stop_active_.exchange(true))
        safety_generation_.fetch_add(1);
      predictive_replan_requested_.store(false);
      replan_fail_count_ = 0;
      setLocalRecoveryState(
          LocalRecoveryState::LOCAL_REPAIR, 0.0,
          "structured_repair_waiting_for_stop");
      requestExecutionStop("LOCAL_REPAIR_HOLD", false);
      RCLCPP_WARN(
          node_->get_logger(),
          "[STRUCTURED_LOCAL_REPAIR_WAITING_FOR_STOP] speed=%.3fm/s; retaining request_id=%llu and deferring rest-to-rest candidate generation",
          safety_planar_speed,
          static_cast<unsigned long long>(
              active_reference_request_id_.load()));
      return false;
    }

    ReferencePathLookahead reference_entry = projectReferencePathLookahead(
        reference_path_, start_pt_, reference_progress_idx_,
        reference_progress_ratio_, 0.50);
    Eigen::Vector2d forward = reference_entry.tangent.head<2>();
    if (!reference_entry.valid || forward.norm() < 1e-6)
      forward = end_pt_.head<2>() - start_pt_.head<2>();
    if (forward.norm() < 1e-6)
      return false;
    forward.normalize();

    struct RecoveryAnchor
    {
      Eigen::Vector3d point;
      size_t direction_index{0};
      double distance{0.0};
    };
    std::vector<RecoveryAnchor> anchors;
    const auto directions = structuredLocalRepairDirections(forward);
    auto map = planner_manager_->grid_map_;
    const auto snapshot = map->captureInflatedOccupancySnapshot();
    map->useInflatedOccupancySnapshotForCurrentThread(snapshot);
    const double actual_yaw = getOdomYaw();
    const bool fixed_body_yaw_repair = enable_holonomic_lateral_repair_;
    if (!fixed_body_yaw_repair)
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[DISTURBANCE_RECAPTURE] using turn-then-drive primitives; fixed-yaw lateral translation remains disabled");

    for (size_t direction_index = 0;
         direction_index < directions.size(); ++direction_index)
    {
      const Eigen::Vector2d &direction = directions[direction_index];
      bool accepted_direction = false;
      for (double distance = local_repair_anchor_min_distance_;
           distance <= local_repair_anchor_max_distance_ + 1e-9;
           distance += local_repair_anchor_step_)
      {
        const double backward_distance =
            std::max(0.0, -distance * direction.dot(forward));
        if (backward_distance > local_repair_max_backtrack_ + 1e-9)
          continue;

        Eigen::Vector3d anchor = start_pt_;
        anchor.head<2>() += distance * direction;
        const int samples = std::max(
            1, static_cast<int>(std::ceil(distance / 0.05)));
        std::vector<size_t> violation_counts;
        violation_counts.reserve(samples + 1);
        bool physical_collision = false;
        const double target_yaw = std::atan2(direction.y(), direction.x());

        // A non-holonomic recovery is two motions, not a diagonal strafe:
        // first rotate in place while the physical footprint remains free,
        // then drive along the candidate direction.  The planning belt may
        // already be violated at the disturbed start pose, but the physical
        // footprint is never relaxed during either phase.
        if (!fixed_body_yaw_repair)
        {
          constexpr double yaw_step = 5.0 * M_PI / 180.0;
          const double yaw_delta = normalizePlanarAngle(
              target_yaw - actual_yaw);
          const int yaw_samples = std::max(
              1, static_cast<int>(std::ceil(
                     std::abs(yaw_delta) / yaw_step)));
          for (int yaw_sample = 0; yaw_sample <= yaw_samples; ++yaw_sample)
          {
            const double ratio =
                static_cast<double>(yaw_sample) / yaw_samples;
            const double rotation_yaw = interpolatePlanarYaw(
                actual_yaw, target_yaw, ratio);
            if (!std::isfinite(rotation_yaw) ||
                map->getInflateOccupancy(start_pt_, rotation_yaw) != 0)
            {
              physical_collision = true;
              break;
            }
            size_t rotation_violations = 0;
            footprintOccupiedWithMargin(
                map, start_pt_, rotation_yaw,
                planning_clearance_margin_, nullptr,
                &rotation_violations);
            violation_counts.push_back(rotation_violations);
          }
        }

        for (int sample = 0; sample <= samples; ++sample)
        {
          if (physical_collision)
            break;
          const double ratio = static_cast<double>(sample) / samples;
          const Eigen::Vector3d point = start_pt_ + ratio * (anchor - start_pt_);
          const double yaw = fixed_body_yaw_repair
                                 ? actual_yaw
                                 : target_yaw;
          if (!std::isfinite(yaw))
          {
            physical_collision = true;
            break;
          }
          size_t violations = 0;
          footprintOccupiedWithMargin(
              map, point, yaw, planning_clearance_margin_, nullptr,
              &violations);
          // The target-yaw start pose was already appended by the rotation
          // sweep. Avoid double-counting it while preserving the complete
          // monotonic sequence for fixed-yaw legacy repair.
          if (fixed_body_yaw_repair || sample > 0)
            violation_counts.push_back(violations);
          if (map->getInflateOccupancy(point, yaw) != 0)
          {
            physical_collision = true;
            break;
          }
        }
        if (physical_collision ||
            !recoveryClearanceEvidenceIsNonWorsening(violation_counts))
          continue;

        // Reject an escape that has no certified next step toward the same
        // reference. This is a bounded connector check, not a guarantee that
        // subsequent optimization will succeed.
        const auto rejoin = projectReferencePathLookahead(
            reference_path_, anchor, reference_progress_idx_,
            reference_progress_ratio_, 0.50);
        if (!rejoin.valid)
          continue;
        const Eigen::Vector3d connector = rejoin.target - anchor;
        const double rejoin_yaw = std::atan2(connector.y(), connector.x());
        bool reconnect_free = true;
        const int turn_samples = std::max(1, static_cast<int>(std::ceil(
            std::abs(normalizePlanarAngle(rejoin_yaw - target_yaw)) / (5.0 * M_PI / 180.0))));
        for (int i = 0; i <= turn_samples; ++i)
          if (footprintOccupiedWithMargin(map, anchor,
                  interpolatePlanarYaw(target_yaw, rejoin_yaw,
                      static_cast<double>(i) / turn_samples),
                  planning_clearance_margin_))
            reconnect_free = false;
        const int connector_samples = std::max(1, static_cast<int>(std::ceil(connector.head<2>().norm() / 0.05)));
        for (int i = 0; reconnect_free && i <= connector_samples; ++i)
          if (footprintOccupiedWithMargin(map,
                  anchor + connector * (static_cast<double>(i) / connector_samples),
                  rejoin_yaw, planning_clearance_margin_))
            reconnect_free = false;
        if (!reconnect_free)
          continue;
        anchors.push_back({anchor, direction_index, distance});
        accepted_direction = true;
        break;
      }
      if (accepted_direction &&
          static_cast<int>(anchors.size()) >= local_repair_max_candidates_)
        break;
    }
    map->clearInflatedOccupancySnapshotForCurrentThread();

    if (anchors.empty())
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[STRUCTURED_LOCAL_REPAIR_UNAVAILABLE] no bounded corridor monotonically exits the planning belt; preserving the Explorer target");
      return false;
    }

    const Eigen::Vector3d saved_local_target = local_target_pt_;
    const Eigen::Vector3d saved_local_velocity = local_target_vel_;
    const Eigen::Vector3d saved_start_velocity = start_vel_;
    const Eigen::Vector3d saved_start_acceleration = start_acc_;
    start_vel_.setZero();
    start_acc_.setZero();
    for (size_t attempt = 0; attempt < anchors.size(); ++attempt)
    {
      const RecoveryAnchor &anchor = anchors[attempt];
      const double minimum_duration = restToRestMinimumDuration(
          anchor.distance, local_repair_max_speed_,
          planner_manager_->pp_.max_acc_, local_repair_time_margin_);
      RCLCPP_WARN(
          node_->get_logger(),
          "[STRUCTURED_LOCAL_REPAIR_ATTEMPT] request_id=%llu attempt=%zu/%zu direction=%zu anchor=(%.2f,%.2f,%.2f) distance=%.2fm max_speed=%.2fm/s min_duration=%.2fs; global_target=(%.2f,%.2f,%.2f) unchanged",
          static_cast<unsigned long long>(
              active_reference_request_id_.load()),
          attempt + 1, anchors.size(), anchor.direction_index,
          anchor.point.x(), anchor.point.y(), anchor.point.z(),
          anchor.distance, local_repair_max_speed_, minimum_duration,
          end_pt_.x(), end_pt_.y(), end_pt_.z());
      if (callReboundReplan(
              true, false, &anchor.point, true, nullptr, nullptr, nullptr,
              nullptr, nullptr, 0.0, nullptr, 0.0,
              fixed_body_yaw_repair))
      {
        start_vel_ = saved_start_velocity;
        start_acc_ = saved_start_acceleration;
        RCLCPP_INFO(
            node_->get_logger(),
            "[STRUCTURED_LOCAL_REPAIR_SELECTED] request_id=%llu anchor=(%.2f,%.2f,%.2f); executing the short escape segment before rejoining the same reference path",
            static_cast<unsigned long long>(
                active_reference_request_id_.load()),
            anchor.point.x(), anchor.point.y(), anchor.point.z());
        return true;
      }
    }
    start_vel_ = saved_start_velocity;
    start_acc_ = saved_start_acceleration;
    local_target_pt_ = saved_local_target;
    local_target_vel_ = saved_local_velocity;
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[STRUCTURED_LOCAL_REPAIR_EXHAUSTED] %zu geometrically distinct anchors failed validation; preserving the Explorer target",
        anchors.size());
    return false;
  }

  double SCANReplanFSM::executionProgressTime(const LocalTrajData &info)
  {
    std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
    if (execution_snapshot_.valid && execution_snapshot_.trajectory_id == info.traj_id_ &&
        min_forward_command_ > 0.0 && execution_snapshot_.execution_mode !=
            scan_planner_msgs::msg::Bspline::MODE_NORMAL)
      return execution_snapshot_.controller_execution_time;
    return std::clamp((node_->now() - info.start_time_).seconds(),
                      0.0, std::max(0.0, info.duration_));
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    refreshPlanningOdomFromSafety();
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    {
      std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
      if (execution_snapshot_.valid && min_forward_command_ > 0.0 &&
          execution_snapshot_.execution_mode != scan_planner_msgs::msg::Bspline::MODE_NORMAL)
      {
        // Spatial gait does not follow wall-clock spline derivatives. Seed
        // replacements with measured velocity rather than fictitious progress.
        if (reference_path_active_)
          alignStartStateToReferencePath();
        return;
      }
    }
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
    {
      if (reference_path_active_)
        alignStartStateToReferencePath();
      return;
    }

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
    {
      if (reference_path_active_)
        alignStartStateToReferencePath();
      return;
    }

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    if (reference_path_active_)
    {
      alignStartStateToReferencePath();
      return;
    }

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

    start_vel_ = projectForwardVelocityToPath(start_vel_, entry.tangent);
    start_acc_.setZero();
  }

  void SCANReplanFSM::updateExecutionTrajectorySnapshot(
      const LocalTrajData &info, uint64_t request_id,
      bool clearance_escape_active, double clearance_escape_deadline,
      size_t initial_clearance_violations, bool fixed_body_yaw,
      double body_yaw, uint8_t execution_mode)
  {
    Eigen::Vector3d accepted_position =
        info.position_traj_.evaluateDeBoorT(0.0);
    {
      std::lock_guard<std::mutex> safety_lock(safety_odom_mutex_);
      if (safety_have_odom_ && safety_odom_pos_.allFinite())
        accepted_position = safety_odom_pos_;
    }
    std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
    const bool preserve_progress = execution_snapshot_.valid &&
        execution_snapshot_.request_id == request_id &&
        execution_snapshot_.execution_mode ==
            scan_planner_msgs::msg::Bspline::MODE_CLEARANCE_LIMITED &&
        execution_mode == scan_planner_msgs::msg::Bspline::MODE_CLEARANCE_LIMITED;
    const auto progress_started = execution_snapshot_.accepted_at;
    const auto progress_position = execution_snapshot_.accepted_position;
    execution_snapshot_.position = info.position_traj_;
    execution_snapshot_.start_time = info.start_time_;
    execution_snapshot_.duration = info.duration_;
    execution_snapshot_.controller_execution_time = 0.0;
    execution_snapshot_.request_id = request_id;
    execution_snapshot_.trajectory_id = info.traj_id_;
    execution_snapshot_.clearance_escape_active = clearance_escape_active;
    execution_snapshot_.clearance_escape_deadline = clearance_escape_deadline;
    execution_snapshot_.initial_clearance_violations =
        initial_clearance_violations;
    execution_snapshot_.clearance_escape_free_cycles = 0;
    const Eigen::Vector3d escape_start =
        info.position_traj_.evaluateDeBoorT(0.0);
    const Eigen::Vector3d escape_end =
        info.position_traj_.evaluateDeBoorT(info.duration_);
    const double escape_distance =
        (escape_end.head<2>() - escape_start.head<2>()).norm();
    execution_snapshot_.clearance_escape_start_position = escape_start;
    execution_snapshot_.clearance_escape_min_displacement =
        clearance_escape_active
            ? std::min(0.10, std::max(0.03, 0.25 * escape_distance))
            : 0.0;
    execution_snapshot_.clearance_escape_min_progress_ratio =
        clearance_escape_active ? 0.15 : 0.0;
    execution_snapshot_.fixed_body_yaw = fixed_body_yaw;
    execution_snapshot_.body_yaw = body_yaw;
    execution_snapshot_.execution_mode = execution_mode;
    execution_snapshot_.accepted_at = preserve_progress
        ? progress_started : std::chrono::steady_clock::now();
    execution_snapshot_.accepted_position = preserve_progress
        ? progress_position : accepted_position;
    execution_snapshot_.valid = info.start_time_.seconds() > 1e-5 && info.duration_ > 0.0;
    terminal_repair_requested_.store(false);
    terminal_repair_error_.store(0.0);
    // Frozen is versioned feedback.  Never carry the previous trajectory's
    // frozen bit across a handoff while waiting for the new state heartbeat.
    go2_execution_frozen_.store(false);
  }

  void SCANReplanFSM::setLocalRecoveryState(
      LocalRecoveryState state, double tracking_error, const char *reason)
  {
    const uint8_t next = static_cast<uint8_t>(state);
    const uint8_t previous = local_recovery_state_.exchange(next);
    if (previous == next)
      return;

    const char *state_name = "TRACKING";
    if (state == LocalRecoveryState::TRACKING_DEGRADED)
      state_name = "TRACKING_DEGRADED";
    else if (state == LocalRecoveryState::LOCAL_REPAIR)
      state_name = "LOCAL_REPAIR";
    RCLCPP_WARN(
        node_->get_logger(),
        "[LOCAL_RECOVERY_STATE] state=%s tracking_error=%.3fm reason=%s",
        state_name, tracking_error, reason ? reason : "");
  }

  void SCANReplanFSM::tripRealtimeSafety(
      const std::string &reason, const Eigen::Vector3d *last_free,
      const Eigen::Vector3d *first_blocked, uint64_t request_id,
      int64_t trajectory_id)
  {
    // A pose collision describes the robot's physical state and must never be
    // discarded because a trajectory handoff is in progress.  Look-ahead
    // results, however, are valid only for the exact trajectory they checked.
    const bool versioned_lookahead = request_id != 0 || trajectory_id != 0;
    if (versioned_lookahead)
    {
      std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
      if (!execution_snapshot_.valid || !safetyResultMatchesExecution(
              request_id, trajectory_id,
              execution_snapshot_.request_id,
              execution_snapshot_.trajectory_id))
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "[STALE_SAFETY_RESULT_IGNORED] checked_request=%llu checked_trajectory=%lld executing_request=%llu executing_trajectory=%lld",
            static_cast<unsigned long long>(request_id),
            static_cast<long long>(trajectory_id),
            static_cast<unsigned long long>(execution_snapshot_.request_id),
            static_cast<long long>(execution_snapshot_.trajectory_id));
        return;
      }
    }
    bool expected = false;
    if (!safety_stop_active_.compare_exchange_strong(expected, true))
      return;

    predictive_replan_requested_.store(false);
    safety_generation_.fetch_add(1);
    const bool recover_locally_after_stop =
        reason == "BLOCKED" && have_target_ && !pose_occupied_latched_.load();
    if (recover_locally_after_stop)
    {
      stopped_local_repair_pending_.store(true);
      stopped_local_repair_active_.store(false);
      setLocalRecoveryState(
          LocalRecoveryState::LOCAL_REPAIR, 0.0,
          "trajectory_blocked_hard_stop");
      // BLOCKED is not terminal yet.  Hold the controller, but keep the same
      // request owned by SCAN until the stopped local-repair budget is spent.
      requestExecutionStop("LOCAL_REPAIR_HOLD", false);
    }
    else
    {
      stopped_local_repair_pending_.store(false);
      stopped_local_repair_active_.store(false);
      requestExecutionStop(reason);
    }
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

  bool SCANReplanFSM::footprintOccupiedWithMargin(
      const GridMap::Ptr &map, const Eigen::Vector3d &position,
      double yaw, double margin, Eigen::Vector3d *first_blocked,
      size_t *blocked_count) const
  {
    const double usable_margin = std::max(0.0, margin);
    size_t count = 0;
    for (const Eigen::Vector2d &offset : planning_clearance_offsets_)
    {
      if (offset.norm() > usable_margin + 1e-9)
        continue;
      Eigen::Vector3d query = position;
      query.head<2>() += offset;
      if (map->getInflateOccupancy(query, yaw) != 0)
      {
        if (count == 0 && first_blocked)
          *first_blocked = query;
        ++count;
        if (!blocked_count)
          return true;
      }
    }
    if (blocked_count)
      *blocked_count = count;
    return count != 0;
  }

  void SCANReplanFSM::latchLocalSafetyHold(
      const std::string &reason, std::atomic<bool> &source_latch)
  {
    bool expected = false;
    if (!source_latch.compare_exchange_strong(expected, true))
      return;
    safety_stop_active_.store(true);
    safety_generation_.fetch_add(1);
    predictive_replan_requested_.store(false);
    requestExecutionStop(reason, false);
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
    Eigen::Vector3d odom_vel;
    Eigen::Quaterniond odom_orient;
    double odom_yaw_rate = 0.0;
    {
      std::lock_guard<std::mutex> lock(safety_odom_mutex_);
      if (!safety_have_odom_)
        return;
      odom_pos = safety_odom_pos_;
      odom_vel = safety_odom_vel_;
      odom_orient = safety_odom_orient_;
      odom_yaw_rate = safety_yaw_rate_;
    }

    if (invalid_odom_latched_.load())
    {
      const int valid_cycles = invalid_odom_release_cycles_.fetch_add(1) + 1;
      if (valid_cycles < local_hold_release_cycles_)
        return;
      invalid_odom_release_cycles_.store(0);
      invalid_odom_latched_.store(false);
      predictive_replan_requested_.store(true);
      setLocalRecoveryState(
          LocalRecoveryState::LOCAL_REPAIR, 0.0,
          "valid_odometry_restored");
      if (!pose_occupied_latched_.load() &&
          !predictive_hold_latched_.load() &&
          !temporal_safety_latched_.load() &&
          !stopped_local_repair_pending_.load() &&
          safety_stop_active_.exchange(false))
      {
        RCLCPP_INFO(
            node_->get_logger(),
            "[LOCAL_SAFETY_RELEASE] source=valid_odometry valid_cycles=%d; requesting a fresh rolling trajectory",
            valid_cycles);
      }
      return;
    }

    const double map_age = map->getMapAgeSeconds();
    if (map_age > safety_map_stale_warn_age_)
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "[SAFETY_MAP_STALE] map_age=%.1fms revision=%lu",
                           map_age * 1000.0,
                           static_cast<unsigned long>(map->getMapRevision()));

    const bool temporal_fault =
        map_age > safety_map_stale_hold_age_ ||
        callback_gap > safety_callback_hold_gap_;
    if (temporal_fault)
    {
      temporal_safety_release_cycles_.store(0);
      latchLocalSafetyHold("LOCAL_PERCEPTION_STALE",
                           temporal_safety_latched_);
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[LOCAL_SAFETY_HOLD] reason=temporal_fault map_age=%.1fms callback_gap=%.1fms; preserving Explorer commitment",
          map_age * 1000.0, callback_gap * 1000.0);
      return;
    }

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
      pose_occupied_latched_.store(true);
      pose_free_confirmation_cycles_.store(0);
      tripRealtimeSafety("POSE_OCCUPIED");
      RCLCPP_ERROR_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "[REALTIME_SAFETY_STOP] reason=pose_occupied pos=(%.2f,%.2f) map_age=%.1fms",
          odom_pos.x(), odom_pos.y(), map_age * 1000.0);
      return;
    }

    if (pose_occupied_latched_.load())
    {
      const int free_cycles = pose_free_confirmation_cycles_.fetch_add(1) + 1;
      if (free_cycles < pose_free_release_cycles_)
        return;

      pose_free_confirmation_cycles_.store(0);
      pose_occupied_latched_.store(false);
      if (!predictive_hold_latched_.load() &&
          !temporal_safety_latched_.load() &&
          !invalid_odom_latched_.load() &&
          !stopped_local_repair_pending_.load() &&
          safety_stop_active_.exchange(false))
      {
        RCLCPP_INFO(
            node_->get_logger(),
            "[SAFETY_RELEASE] source=physical_footprint_free free_cycles=%d; planning may resume",
            free_cycles);
      }
    }

    // LOCAL_REPAIR_EXHAUSTED is a terminal ownership state for the failed
    // request.  It may only be released by publishing a fully validated
    // trajectory for a newer request; ordinary predictive/temporal release
    // paths must not revive the invalidated execution snapshot.
    if (terminal_local_repair_hold_.load())
      return;

    if (temporal_safety_latched_.load())
    {
      const int fresh_cycles = temporal_safety_release_cycles_.fetch_add(1) + 1;
      if (fresh_cycles < local_hold_release_cycles_)
        return;
      temporal_safety_release_cycles_.store(0);
      temporal_safety_latched_.store(false);
      if (!pose_occupied_latched_.load() &&
          !predictive_hold_latched_.load() &&
          !invalid_odom_latched_.load() &&
          !stopped_local_repair_pending_.load() &&
          safety_stop_active_.exchange(false))
      {
        predictive_replan_requested_.store(true);
        setLocalRecoveryState(
            LocalRecoveryState::LOCAL_REPAIR, 0.0,
            "local_perception_fresh_again");
        RCLCPP_INFO(
            node_->get_logger(),
            "[LOCAL_SAFETY_RELEASE] source=temporal_fault fresh_cycles=%d; requesting a fresh rolling trajectory",
            fresh_cycles);
      }
      return;
    }

    const double actual_speed = odom_vel.head<2>().norm();
    if (predictive_hold_latched_.load())
    {
      if (actual_speed > 0.05)
      {
        predictive_hold_release_cycles_.store(0);
        return;
      }
      const int stopped_cycles = predictive_hold_release_cycles_.fetch_add(1) + 1;
      if (stopped_cycles < local_hold_release_cycles_)
        return;

      size_t current_clearance_violations = 0;
      const bool current_clearance_blocked = footprintOccupiedWithMargin(
          map, odom_pos, actual_yaw, planning_clearance_margin_, nullptr,
          &current_clearance_violations);
      if (!brakingHoldMayReleaseAfterStop(
              actual_speed, stopped_cycles, local_hold_release_cycles_,
              pose_occupied_latched_.load(), current_clearance_blocked))
      {
        // Zero speed removes the kinetic risk, but it does not make this a
        // valid restart pose. Retain HOLD and transfer planning ownership to
        // the bounded rest-to-rest clearance escape. A validated candidate is
        // the only event allowed to release this predictive latch.
        predictive_hold_release_cycles_.store(0);
        stopped_local_repair_pending_.store(true);
        predictive_replan_requested_.store(true);
        setLocalRecoveryState(
            LocalRecoveryState::LOCAL_REPAIR, 0.0,
            "braking_hold_needs_clearance_escape");
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[LOCAL_SAFETY_HOLD] reason=stopped_inside_planning_belt violations=%zu; retaining HOLD until a validated clearance escape is published",
            current_clearance_violations);
        return;
      }
      predictive_hold_release_cycles_.store(0);
      predictive_hold_latched_.store(false);
      if (!pose_occupied_latched_.load() &&
          !temporal_safety_latched_.load() &&
          !invalid_odom_latched_.load() &&
          !stopped_local_repair_pending_.load() &&
          safety_stop_active_.exchange(false))
      {
        predictive_replan_requested_.store(true);
        setLocalRecoveryState(
            LocalRecoveryState::LOCAL_REPAIR, 0.0,
            "braking_hold_stopped");
        RCLCPP_INFO(
            node_->get_logger(),
            "[LOCAL_SAFETY_RELEASE] source=braking_sweep stopped_cycles=%d; requesting a fresh rolling trajectory",
            stopped_cycles);
      }
      return;
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

    // Predict where the physical robot can travel before a HOLD takes effect.
    // Velocity is expressed in odom/world coordinates and the footprint
    // follows the measured yaw rate. A fixed-direction line is unsafe while a
    // quadruped is turning because it puts the braking distance on the wrong
    // side of a corridor.
    if (actual_speed > 0.03)
    {
      const double horizon = brakingSweepHorizon(
          actual_speed, guaranteed_braking_deceleration_,
          command_stop_latency_, braking_sweep_max_horizon_);
      const double sample_dt = std::clamp(
          braking_sweep_spatial_step_ / actual_speed, 0.02, 0.10);
      const int samples = std::max(
          1, static_cast<int>(std::ceil(horizon / sample_dt)));
      const double travel_heading = std::atan2(odom_vel.y(), odom_vel.x());
      PlanarBrakingState predicted_state;
      predicted_state.position = odom_pos.head<2>();
      predicted_state.yaw = actual_yaw;
      Eigen::Vector3d blocked_point;
      double elapsed = 0.0;
      bool braking_escape_exited_clearance = false;
      size_t previous_braking_violations =
          trajectory.initial_clearance_violations;
      for (int sample = 1; sample <= samples; ++sample)
      {
        const double time = horizon * static_cast<double>(sample) / samples;
        advancePlanarBrakingState(
            &predicted_state, actual_speed, travel_heading, actual_yaw,
            odom_yaw_rate, guaranteed_braking_deceleration_,
            command_stop_latency_, elapsed, time - elapsed);
        elapsed = time;
        Eigen::Vector3d predicted = odom_pos;
        predicted.head<2>() = predicted_state.position;
        size_t clearance_violations = 0;
        const bool clearance_blocked = footprintOccupiedWithMargin(
            map, predicted, predicted_state.yaw,
            planning_clearance_margin_, &blocked_point,
            &clearance_violations);
        if (!clearance_blocked)
        {
          braking_escape_exited_clearance = true;
          previous_braking_violations = 0;
          continue;
        }
        const bool physical_collision =
            map->getInflateOccupancy(predicted, predicted_state.yaw) != 0;
        if (clearanceViolationAllowedDuringEscape(
                trajectory.clearance_escape_active,
                braking_escape_exited_clearance, physical_collision, time,
                trajectory.clearance_escape_deadline,
                clearance_violations, previous_braking_violations))
        {
          previous_braking_violations = clearance_violations;
          RCLCPP_INFO_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 250,
              "[CLEARANCE_ESCAPE_BRAKING_SWEEP_ALLOWED] request_id=%llu trajectory=%lld time=%.2fs violations=%zu; predicted footprint is not worsening and remains physically free",
              static_cast<unsigned long long>(trajectory.request_id),
              static_cast<long long>(trajectory.trajectory_id), time,
              clearance_violations);
          continue;
        }
        if (clearance_blocked)
        {
          if (structuredRepairSafetyInterruptionRequiresRetry(
                  structured_local_repair_active_.load(),
                  structured_local_repair_executing_.load(),
                  trajectory.execution_mode,
                  scan_planner_msgs::msg::Bspline::MODE_RECOVERY_PRIMITIVE))
          {
            // The recovery transaction still owns the global request, but the
            // current segment is terminally unusable. Transfer ownership back
            // to the stopped candidate search instead of leaving Explorer and
            // SCAN waiting forever on an executing segment that Controller has
            // already held.
            structured_local_repair_executing_.store(false);
            structured_local_repair_finished_.store(false);
            structured_local_repair_rejoin_pending_.store(false);
            structured_local_repair_interrupted_.store(true);
            stopped_local_repair_pending_.store(true);
            stopped_local_repair_active_.store(false);
            predictive_replan_requested_.store(true);
            setLocalRecoveryState(
                LocalRecoveryState::LOCAL_REPAIR, 0.0,
                "structured_repair_interrupted_by_safety");
            RCLCPP_WARN(
                node_->get_logger(),
                "[STRUCTURED_LOCAL_REPAIR_INTERRUPTED] request_id=%llu trajectory=%lld; retrying the remaining bounded candidates after stop",
                static_cast<unsigned long long>(trajectory.request_id),
                static_cast<long long>(trajectory.trajectory_id));
          }
          predictive_hold_release_cycles_.store(0);
          latchLocalSafetyHold(
              "PREDICTED_BRAKING_HOLD", predictive_hold_latched_);
          RCLCPP_ERROR_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 500,
              "[LOCAL_SAFETY_HOLD] reason=actual_braking_sweep speed=%.2fm/s yaw_rate=%.2frad/s horizon=%.2fs predicted_stop=(%.2f,%.2f) hit=(%.2f,%.2f) margin=%.2fm; preserving Explorer commitment",
              actual_speed, odom_yaw_rate, horizon,
              predicted_state.position.x(), predicted_state.position.y(),
              blocked_point.x(), blocked_point.y(), planning_clearance_margin_);
          return;
        }
      }
    }

    double t_cur = (node_->now() - trajectory.start_time).seconds();
    if (min_forward_command_ > 0.0 && trajectory.execution_mode !=
        scan_planner_msgs::msg::Bspline::MODE_NORMAL)
      t_cur = trajectory.controller_execution_time;
    t_cur = std::min(std::max(t_cur, 0.0), trajectory.duration);
    // Match the physical robot to a nearby point on the active trajectory.
    // The raw time-indexed error is not an executable recovery trajectory.
    const double match_begin = std::max(0.0, t_cur - tracking_match_back_time_);
    const double match_end = std::min(
        trajectory.duration, t_cur + tracking_match_forward_time_);
    double matched_time = t_cur;
    double matched_error2 = std::numeric_limits<double>::infinity();
    constexpr double match_step = 0.02;
    for (double t = match_begin; t <= match_end + 1e-6; t += match_step)
    {
      const double sample_time = std::min(t, match_end);
      const Eigen::Vector3d sample =
          trajectory.position.evaluateDeBoorT(sample_time);
      const double error2 =
          (sample.head<2>() - odom_pos.head<2>()).squaredNorm();
      if (error2 < matched_error2)
      {
        matched_error2 = error2;
        matched_time = sample_time;
      }
    }

    const Eigen::Vector3d tracked_pos =
        trajectory.position.evaluateDeBoorT(matched_time);
    const Eigen::Vector3d connector = tracked_pos - odom_pos;
    const double connector_length = connector.head<2>().norm();
    constexpr double spatial_step = 0.05;
    if (connector_length <= tracking_degraded_error_)
    {
      setLocalRecoveryState(
          LocalRecoveryState::TRACKING, connector_length,
          "nearest_trajectory_match");
    }
    else
    {
      const double tangent_before_time = std::max(0.0, matched_time - match_step);
      const double tangent_after_time = std::min(
          trajectory.duration, matched_time + match_step);
      const Eigen::Vector3d tangent_before =
          trajectory.position.evaluateDeBoorT(tangent_before_time);
      const Eigen::Vector3d tangent_after =
          trajectory.position.evaluateDeBoorT(tangent_after_time);
      const double trajectory_yaw = trajectory.fixed_body_yaw
                                        ? trajectory.body_yaw
                                        : segment_yaw(tangent_before,
                                                      tangent_after);
      const int samples = std::max(
          1, static_cast<int>(std::ceil(connector_length / spatial_step)));
      bool recovery_corridor_blocked = false;
      Eigen::Vector3d first_recovery_blocked = tracked_pos;
      for (int sample = 1; sample <= samples; ++sample)
      {
        const double ratio = static_cast<double>(sample) / samples;
        const Eigen::Vector3d point = odom_pos + ratio * connector;
        const double yaw = interpolatePlanarYaw(
            actual_yaw, trajectory_yaw, ratio);
        if (!std::isfinite(yaw))
        {
          recovery_corridor_blocked = true;
          first_recovery_blocked = point;
          break;
        }
        if (footprintOccupiedWithMargin(
                map, point, yaw, planning_clearance_margin_))
        {
          recovery_corridor_blocked = true;
          first_recovery_blocked = point;
          break;
        }
      }

      const double execution_age = std::max(
          0.0, std::chrono::duration<double>(
                   std::chrono::steady_clock::now() -
                   trajectory.accepted_at).count());
      const double odometry_progress =
          (odom_pos.head<2>() -
           trajectory.accepted_position.head<2>()).norm();
      const bool clearance_limited_execution =
          trajectory.execution_mode ==
          scan_planner_msgs::msg::Bspline::MODE_CLEARANCE_LIMITED;
      if (clearance_limited_execution &&
          odometry_progress >= clearance_limited_min_progress_)
      {
        std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
        if (execution_snapshot_.valid &&
            execution_snapshot_.request_id == trajectory.request_id &&
            execution_snapshot_.trajectory_id == trajectory.trajectory_id)
        {
          execution_snapshot_.accepted_at = std::chrono::steady_clock::now();
          execution_snapshot_.accepted_position = odom_pos;
        }
      }
      const TrackingDeviationAction tracking_action =
          classifyTrackingDeviation(
              clearance_limited_execution, recovery_corridor_blocked,
              connector_length, tracking_degraded_error_,
              tracking_recovery_max_error_, execution_age,
              clearance_limited_execution_grace_,
              clearance_limited_no_progress_timeout_, odometry_progress,
              clearance_limited_min_progress_);

      if (tracking_action == TrackingDeviationAction::ROLLING_REPLAN)
      {
        predictive_replan_requested_.store(true);
        setLocalRecoveryState(
            LocalRecoveryState::TRACKING_DEGRADED, connector_length,
            "tracking_error");
      }
      else if (tracking_action == TrackingDeviationAction::LOCAL_REPAIR)
      {
        predictive_replan_requested_.store(false);
        const bool clearance_limited_no_progress =
            clearance_limited_execution && !recovery_corridor_blocked &&
            execution_age >= clearance_limited_no_progress_timeout_ &&
            odometry_progress < clearance_limited_min_progress_;
        if (clearance_limited_no_progress)
        {
          // Replanning another copy of the same hard-safe fallback would only
          // restart the controller clock. Enter the existing bounded stopped
          // recovery transaction; if its finite candidate family is exhausted,
          // it returns VIEWPOINT_LOCAL_REJECTED to Explorer.
          terminal_repair_requested_.store(false);
          stopped_local_repair_pending_.store(true);
          if (!safety_stop_active_.exchange(true))
          {
            safety_generation_.fetch_add(1);
            requestExecutionStop("CLEARANCE_LIMITED_NO_PROGRESS", false);
          }
          RCLCPP_ERROR_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 500,
              "[CLEARANCE_LIMITED_NO_PROGRESS] request=%llu trajectory=%lld age=%.2fs tracking_error=%.3fm odom_progress=%.3fm/%.3fm; entering bounded stopped recovery without publishing an equivalent fallback",
              static_cast<unsigned long long>(trajectory.request_id),
              static_cast<long long>(trajectory.trajectory_id), execution_age,
              connector_length, odometry_progress,
              clearance_limited_min_progress_);
        }
        else
        {
          terminal_repair_error_.store(connector_length);
          terminal_repair_requested_.store(true);
        }
        setLocalRecoveryState(
            LocalRecoveryState::LOCAL_REPAIR, connector_length,
            recovery_corridor_blocked ? "recovery_corridor_blocked"
                : (clearance_limited_no_progress
                       ? "clearance_limited_no_progress"
                       : "tracking_recovery_error_exceeded"));
      }
      else
      {
        // MODE_CLEARANCE_LIMITED was selected precisely because no preferred
        // clearance route exists. While the hard corridor remains free, keep
        // this ACKed execution version instead of replacing it with an
        // equivalent fallback and starving physical execution.
        setLocalRecoveryState(
            LocalRecoveryState::TRACKING_DEGRADED, connector_length,
            "clearance_limited_execution_owned");
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[CLEARANCE_LIMITED_EXECUTION_OWNED] request=%llu trajectory=%lld age=%.2fs tracking_error=%.3fm odom_progress=%.3fm; hard corridor remains free",
            static_cast<unsigned long long>(trajectory.request_id),
            static_cast<long long>(trajectory.trajectory_id), execution_age,
            connector_length, odometry_progress);
      }
      if (recovery_corridor_blocked)
      {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "[LOCAL_REPAIR_REQUESTED] tracking_error=%.2fm hit=(%.2f,%.2f) matched_t=%.2fs map_age=%.1fms; preserving the committed reference path",
            connector_length, first_recovery_blocked.x(),
            first_recovery_blocked.y(), matched_time, map_age * 1000.0);
      }
    }

    if (trajectory.clearance_escape_active)
    {
      size_t actual_clearance_violations = 0;
      const bool actual_clearance_blocked = footprintOccupiedWithMargin(
          map, odom_pos, actual_yaw, planning_clearance_margin_, nullptr,
          &actual_clearance_violations);
      int actual_free_cycles = 0;
      bool completed = false;
      const double actual_displacement =
          (odom_pos.head<2>() -
           trajectory.clearance_escape_start_position.head<2>()).norm();
      const double matched_progress_ratio =
          trajectory.duration > 1e-6
              ? std::clamp(matched_time / trajectory.duration, 0.0, 1.0)
              : 0.0;
      const bool clearance_improved =
          trajectory.initial_clearance_violations >
          actual_clearance_violations;
      {
        std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
        if (execution_snapshot_.valid &&
            safetyResultMatchesExecution(
                trajectory.request_id, trajectory.trajectory_id,
                execution_snapshot_.request_id,
                execution_snapshot_.trajectory_id))
        {
          if (actual_clearance_blocked)
            execution_snapshot_.clearance_escape_free_cycles = 0;
          else
            ++execution_snapshot_.clearance_escape_free_cycles;
          actual_free_cycles =
              execution_snapshot_.clearance_escape_free_cycles;
          completed = clearanceEscapeConfirmedAtActualPose(
              execution_snapshot_.clearance_escape_active,
              actual_clearance_blocked, clearance_improved,
              actual_displacement,
              execution_snapshot_.clearance_escape_min_displacement,
              matched_progress_ratio,
              execution_snapshot_.clearance_escape_min_progress_ratio,
              actual_free_cycles,
              local_hold_release_cycles_);
          if (completed)
          {
            execution_snapshot_.clearance_escape_active = false;
            execution_snapshot_.clearance_escape_free_cycles = 0;
          }
        }
      }
      if (completed)
      {
        trajectory.clearance_escape_active = false;
        RCLCPP_INFO(
            node_->get_logger(),
            "[CLEARANCE_ESCAPE_COMPLETE] request_id=%llu trajectory=%lld matched_t=%.2fs progress=%.2f actual_pose=(%.2f,%.2f) displacement=%.2fm/%.2fm violations=%zu/%zu free_cycles=%d",
            static_cast<unsigned long long>(trajectory.request_id),
            static_cast<long long>(trajectory.trajectory_id), matched_time,
            matched_progress_ratio, odom_pos.x(), odom_pos.y(),
            actual_displacement,
            trajectory.clearance_escape_min_displacement,
            actual_clearance_violations,
            trajectory.initial_clearance_violations, actual_free_cycles);
      }
      else if (actual_clearance_blocked)
      {
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 250,
            "[CLEARANCE_ESCAPE_WAITING] request_id=%llu trajectory=%lld actual_violations=%zu matched_t=%.2fs",
            static_cast<unsigned long long>(trajectory.request_id),
            static_cast<long long>(trajectory.trajectory_id),
            actual_clearance_violations, matched_time);
      }
    }

    constexpr double time_step = 0.02;
    Eigen::Vector3d last_free = odom_pos;
    bool runtime_escape_exited_clearance = false;
    size_t previous_runtime_violations =
        trajectory.initial_clearance_violations;
    double speed = odom_vel.head<2>().norm();
    double first_preferred_conflict_time =
        std::numeric_limits<double>::infinity();
    double first_hard_time = std::numeric_limits<double>::infinity();
    double last_hard_time = -1.0;
    Eigen::Vector3d first_hard_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_hard_position = Eigen::Vector3d::Zero();
    int consecutive_hard_free_after_risk = 0;
    bool hard_exit_known = false;
    double hard_rejoin_time = std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d hard_rejoin_position = Eigen::Vector3d::Zero();
    for (double t = matched_time; t < trajectory.duration; t += time_step)
    {
      const Eigen::Vector3d pos = trajectory.position.evaluateDeBoorT(t);
      const Eigen::Vector3d next = trajectory.position.evaluateDeBoorT(
          std::min(t + time_step, trajectory.duration));
      const double trajectory_yaw = trajectory.fixed_body_yaw
                                        ? trajectory.body_yaw
                                        : segment_yaw(pos, next);
      size_t clearance_violations = 0;
      const bool clearance_blocked = footprintOccupiedWithMargin(
          map, pos, trajectory_yaw, planning_clearance_margin_, nullptr,
          &clearance_violations);
      if (!clearance_blocked)
      {
        if (std::isfinite(first_hard_time) && !hard_exit_known)
        {
          ++consecutive_hard_free_after_risk;
          if (consecutive_hard_free_after_risk >= 5)
          {
            hard_exit_known = true;
            hard_rejoin_time = t;
            hard_rejoin_position = pos;
          }
        }
        const bool preferred_clearance_blocked =
            trajectory.execution_mode ==
                scan_planner_msgs::msg::Bspline::MODE_NORMAL &&
            footprintOccupiedWithMargin(
                map, pos, trajectory_yaw,
                preferred_clearance_margin_);
        if (preferred_clearance_blocked)
        {
          first_preferred_conflict_time = std::min(
              first_preferred_conflict_time,
              std::max(0.0, t - matched_time));
        }
        runtime_escape_exited_clearance = true;
        previous_runtime_violations = 0;
        last_free = pos;
        continue;
      }

      const bool physical_collision =
          map->getInflateOccupancy(pos, trajectory_yaw) != 0;
      if (clearanceViolationAllowedDuringEscape(
              trajectory.clearance_escape_active,
              runtime_escape_exited_clearance,
              physical_collision, t,
              trajectory.clearance_escape_deadline,
              clearance_violations,
              previous_runtime_violations))
      {
        previous_runtime_violations = clearance_violations;
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 250,
            "[CLEARANCE_ESCAPE_EXECUTING] request_id=%llu trajectory=%lld t=%.2fs violations=%zu/%zu",
            static_cast<unsigned long long>(trajectory.request_id),
            static_cast<long long>(trajectory.trajectory_id), t,
            clearance_violations,
            trajectory.initial_clearance_violations);
        continue;
      }

      if (clearance_blocked)
      {
        const double time_to_hit = std::max(0.0, t - matched_time);
        if (!std::isfinite(first_hard_time))
        {
          first_hard_time = time_to_hit;
          first_hard_position = pos;
        }
        last_hard_time = time_to_hit;
        consecutive_hard_free_after_risk = 0;
        hard_exit_known = false;
        hard_rejoin_time = std::numeric_limits<double>::quiet_NaN();
        double speed = 0.0;
        {
          std::lock_guard<std::mutex> lock(safety_odom_mutex_);
          speed = safety_odom_vel_.head<2>().norm();
        }
        const double max_deceleration = guaranteed_braking_deceleration_;
        const double hard_stop_time = predictiveHardStopTime(
            speed, max_deceleration, command_stop_latency_,
            predictive_hard_stop_min_time_);
        if (predictiveCollisionRequiresHardStop(
                time_to_hit, speed, max_deceleration,
                command_stop_latency_,
                predictive_hard_stop_min_time_))
        {
          tripRealtimeSafety("BLOCKED", &last_free, &pos,
                             trajectory.request_id, trajectory.trajectory_id);
          RCLCPP_WARN(node_->get_logger(),
                      "[REALTIME_SAFETY_STOP] reason=trajectory_blocked time_to_hit=%.2fs hard_stop_time=%.2fs speed=%.2fm/s hit=(%.2f,%.2f) map_age=%.1fms",
                      time_to_hit, hard_stop_time, speed, pos.x(), pos.y(),
                      map_age * 1000.0);
          return;
        }
        else
        {
          predictive_replan_requested_.store(true);
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 500,
              "[PREDICTIVE_COLLISION_WARNING] time_to_hit=%.2fs hard_stop_time=%.2fs speed=%.2fm/s hit=(%.2f,%.2f); scanning interval for local repair",
              time_to_hit, hard_stop_time, speed, pos.x(), pos.y());
        }
        continue;
      }
    }

    const bool has_hard_interval = std::isfinite(first_hard_time) &&
                                   hard_exit_known &&
                                   last_hard_time > first_hard_time;
    const bool has_preferred_interval =
        std::isfinite(first_preferred_conflict_time);
    if (has_hard_interval || has_preferred_interval)
    {
      const CollisionRiskKind kind = has_hard_interval
          ? CollisionRiskKind::HARD_MARGIN
          : CollisionRiskKind::PREFERRED_MARGIN;
      const double entry_time = matched_time +
          (has_hard_interval ? first_hard_time : first_preferred_conflict_time);
      const double exit_time = matched_time +
          (has_hard_interval ? last_hard_time
                             : first_preferred_conflict_time + time_step);
      PredictedCollisionInterval interval = makePredictedCollisionInterval(
          trajectory.request_id, trajectory.trajectory_id, map_snapshot->revision,
          kind, std::max(matched_time, entry_time - time_step), exit_time,
          speed, guaranteed_braking_deceleration_, command_stop_latency_,
          predictive_replan_reaction_time_, 0.0, matched_time,
          trajectory.duration,
          has_hard_interval ? hard_rejoin_time : -1.0);
      if (interval.valid)
      {
        if (has_hard_interval)
          interval.entry_position = first_hard_position;
        else
          interval.entry_position = trajectory.position.evaluateDeBoorT(
              entry_time).head<3>();
        interval.exit_position = has_hard_interval
            ? last_hard_position : interval.entry_position;
        interval.rejoin_position = has_hard_interval
            ? hard_rejoin_position : Eigen::Vector3d::Zero();
        interval.rejoin_time = has_hard_interval
            ? hard_rejoin_time : interval.rejoin_time;
        interval.exit_known = has_hard_interval;
        interval.braking_window = false;
        std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
        if (execution_snapshot_.valid &&
            safetyResultMatchesExecution(
                trajectory.request_id, trajectory.trajectory_id,
                execution_snapshot_.request_id,
                execution_snapshot_.trajectory_id) &&
            (!predicted_collision_interval_.valid ||
             !collision_interval_replan_requested_.load() ||
             predicted_collision_interval_.request_id != trajectory.request_id ||
             predicted_collision_interval_.trajectory_id != trajectory.trajectory_id))
        {
          predicted_collision_interval_ = interval;
          collision_interval_replan_requested_.store(true);
          predictive_replan_requested_.store(true);
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 500,
              "[PREDICTED_COLLISION_INTERVAL] request=%llu trajectory=%lld map_revision=%llu risk=[%.2f,%.2f] repair=[%.2f,%.2f] exit_known=%d",
              static_cast<unsigned long long>(interval.request_id),
              static_cast<long long>(interval.trajectory_id),
              static_cast<unsigned long long>(interval.map_revision),
              interval.entry_time, interval.exit_time,
              interval.repair_start_time, interval.rejoin_time,
              interval.exit_known);
        }
      }
    }
    // Preferred clearance is a replanning trigger, never a reason to skip the
    // rest of the hard-safety scan.  Only request the smooth replacement after
    // the complete suffix has been proven free of hard-margin conflicts.
    if (std::isfinite(first_preferred_conflict_time))
    {
      predictive_replan_requested_.store(true);
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[PREFERRED_CLEARANCE_REPLAN] time_to_belt=%.2fs hard_margin=%.2fm preferred_margin=%.2fm; complete suffix remains hard-safe, requesting a wider or speed-limited replacement",
          first_preferred_conflict_time, planning_clearance_margin_,
          preferred_clearance_margin_);
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    const int64_t now_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const int64_t last_fsm_wall_ns = last_fsm_callback_wall_ns_.load();
    const double fsm_gap = static_cast<double>(now_wall_ns - last_fsm_wall_ns) * 1e-9;
    if (last_fsm_wall_ns > 0 && fsm_gap > 1.0 &&
        !fsm_callback_stall_reported_.exchange(true))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[FSM_CALLBACK_STALLED] no FSM callback for %.2fs while safety callbacks remain alive",
                   fsm_gap);
    }
    checkCollisionRealtimeCallback();
  }
  bool SCANReplanFSM::callReboundReplan(
      bool flag_use_poly_init, bool flag_randomPolyTraj,
      const Eigen::Vector3d *local_target_override,
      bool low_speed_local_repair,
      const std::vector<Eigen::Vector3d> *seed_path,
      const Eigen::Vector3d *local_target_velocity_override,
      const Eigen::Vector3d *local_target_acceleration_override,
      const Eigen::Vector3d *seed_start_velocity_override,
      const Eigen::Vector3d *seed_start_acceleration_override,
      double seed_sample_interval,
      const PredictedCollisionInterval *seed_interval,
      double seed_base_time,
      bool fixed_body_yaw_repair)
  {
    auto map = planner_manager_->grid_map_;
    const uint64_t planning_request_id = active_reference_request_id_.load();
    const double local_repair_body_yaw =
        low_speed_local_repair ? getOdomYaw() : 0.0;
    struct TargetStateRestore
    {
      SCANReplanFSM *fsm;
      bool enabled;
      Eigen::Vector3d end_pt;
      Eigen::Vector3d local_target_pt;
      Eigen::Vector3d local_target_vel;
      ~TargetStateRestore()
      {
        if (!enabled)
          return;
        fsm->end_pt_ = end_pt;
        fsm->local_target_pt_ = local_target_pt;
        fsm->local_target_vel_ = local_target_vel;
      }
    } target_state_restore{this, seed_path != nullptr, end_pt_,
                           local_target_pt_, local_target_vel_};
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
    if (local_target_override)
    {
      local_target_pt_ = *local_target_override;
      local_target_vel_ = local_target_velocity_override
                              ? *local_target_velocity_override
                              : Eigen::Vector3d::Zero();
    }
    else
    {
      getLocalTarget();
    }
    const Eigen::Vector3d target_acceleration =
        local_target_acceleration_override
            ? *local_target_acceleration_override
            : Eigen::Vector3d::Zero();

    double initialization_speed_limit = -1.0;
    double minimum_initial_duration = 0.0;
    if (low_speed_local_repair)
    {
      initialization_speed_limit = local_repair_max_speed_;
      minimum_initial_duration = restToRestMinimumDuration(
          (local_target_pt_ - start_pt_).norm(), local_repair_max_speed_,
          planner_manager_->pp_.max_acc_, local_repair_time_margin_);
    }
    else if (seed_path)
    {
      double seed_length = 0.0;
      for (size_t i = 1; i < seed_path->size(); ++i)
        seed_length += ((*seed_path)[i] - (*seed_path)[i - 1]).norm();
      minimum_initial_duration = std::max(
          seed_length / std::max(0.10, planner_manager_->pp_.max_vel_),
          std::sqrt(seed_length / std::max(0.10, planner_manager_->pp_.max_acc_)));
    }
    const bool initialize_from_polynomial =
        have_new_target_ || flag_use_poly_init;
    const auto plan_with_margin = [&](double clearance_margin) {
      const Eigen::Vector3d seed_start_velocity =
          seed_start_velocity_override ? *seed_start_velocity_override : start_vel_;
      const Eigen::Vector3d seed_start_acceleration =
          seed_start_acceleration_override ? *seed_start_acceleration_override : start_acc_;
      return planner_manager_->reboundReplan(
          start_pt_, seed_start_velocity, seed_start_acceleration,
          local_target_pt_, local_target_vel_, initialize_from_polynomial,
          flag_randomPolyTraj, initialization_speed_limit,
          minimum_initial_duration,
          low_speed_local_repair && fixed_body_yaw_repair
              ? local_repair_body_yaw
              : std::numeric_limits<double>::quiet_NaN(),
          clearance_margin, seed_path,
          target_acceleration.x(), target_acceleration.y(),
          target_acceleration.z(), seed_sample_interval);
    };

    // Generate normal candidates against the preferred belt first, so A* and
    // optimization actively search for a wider route.  A narrow but physically
    // valid corridor is not rejected outright: retry once with the hard belt,
    // then mark the resulting trajectory clearance-limited during validation.
    // Structured escape remains the sole exception because it may start
    // inside even the hard belt and is validated by its non-worsening exit.
    bool plan_success = plan_with_margin(
        low_speed_local_repair ? 0.0 : preferred_clearance_margin_);
    if (!plan_success && !low_speed_local_repair)
    {
      planner_manager_->local_data_ = executing_trajectory;
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "[PREFERRED_CLEARANCE_FALLBACK] no %.2fm route; retrying the same target with hard margin %.2fm and a controller-limited execution profile",
          preferred_clearance_margin_, planning_clearance_margin_);
      plan_success = plan_with_margin(planning_clearance_margin_);
    }
    map->clearInflatedOccupancySnapshotForCurrentThread();
    const double optimization_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - planning_started).count();
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      if (planning_request_id != active_reference_request_id_.load())
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "[PLAN_RESULT_DISCARDED] reference changed while planning old_request_id=%llu active_request_id=%llu",
            static_cast<unsigned long long>(planning_request_id),
            static_cast<unsigned long long>(
                active_reference_request_id_.load()));
        return reject_planned_trajectory();
      }

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

      // Validation can take long enough for a newer global route to arrive.
      // Never publish a spline derived from the superseded request.
      if (planning_request_id != active_reference_request_id_.load())
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "[PLAN_RESULT_DISCARDED] reference changed during final validation old_request_id=%llu active_request_id=%llu",
            static_cast<unsigned long long>(planning_request_id),
            static_cast<unsigned long long>(active_reference_request_id_.load()));
        return reject_planned_trajectory();
      }

      auto info = &planner_manager_->local_data_;
      if (seed_path)
      {
        if (!seed_interval || !seed_interval->valid)
          return reject_planned_trajectory();
        {
          std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
          if (!execution_snapshot_.valid ||
              !safetyResultMatchesExecution(
                  seed_interval->request_id, seed_interval->trajectory_id,
                  execution_snapshot_.request_id,
                  execution_snapshot_.trajectory_id))
            return reject_planned_trajectory();
        }
        const PredictedCollisionInterval &interval = *seed_interval;
        const double candidate_entry_time =
            std::max(0.0, interval.repair_start_time - seed_base_time);
        const double candidate_rejoin_time =
            std::max(0.0, interval.rejoin_time - seed_base_time);
        if (candidate_entry_time >= candidate_rejoin_time ||
            candidate_rejoin_time > info->duration_)
          return reject_planned_trajectory();
        const bool entry_continuous = localPatchBoundaryIsContinuous(
            info->position_traj_.evaluateDeBoorT(candidate_entry_time).head<3>(),
            info->velocity_traj_.evaluateDeBoorT(candidate_entry_time).head<3>(),
            info->acceleration_traj_.evaluateDeBoorT(candidate_entry_time).head<3>(),
            executing_trajectory.position_traj_.evaluateDeBoorT(
                interval.repair_start_time).head<3>(),
            executing_trajectory.velocity_traj_.evaluateDeBoorT(
                interval.repair_start_time).head<3>(),
            executing_trajectory.acceleration_traj_.evaluateDeBoorT(
                interval.repair_start_time).head<3>(),
            0.08, 0.20, 0.50);
        const Eigen::Vector3d patch_position =
            info->position_traj_.evaluateDeBoorT(candidate_rejoin_time).head<3>();
        const Eigen::Vector3d patch_velocity =
            info->velocity_traj_.evaluateDeBoorT(candidate_rejoin_time).head<3>();
        const Eigen::Vector3d patch_acceleration =
            info->acceleration_traj_.evaluateDeBoorT(candidate_rejoin_time).head<3>();
        const Eigen::Vector3d reference_velocity =
            executing_trajectory.velocity_traj_.evaluateDeBoorT(
                interval.rejoin_time).head<3>();
        const Eigen::Vector3d reference_acceleration =
            executing_trajectory.acceleration_traj_.evaluateDeBoorT(
                interval.rejoin_time).head<3>();
        const double candidate_time_scale =
            info->duration_ / std::max(
                1e-6, executing_trajectory.duration_ - seed_base_time);
        const bool rejoin_continuous = localPatchBoundaryIsContinuous(
            patch_position, patch_velocity, patch_acceleration,
            executing_trajectory.position_traj_.evaluateDeBoorT(
                interval.rejoin_time).head<3>(),
            reference_velocity, reference_acceleration,
            0.08, 0.20, 0.50);
        if (std::abs(candidate_time_scale - 1.0) > 0.05 ||
            !entry_continuous || !rejoin_continuous)
        {
          RCLCPP_WARN(
              node_->get_logger(),
              "[LOCAL_INTERVAL_SPLICE_REJECTED] time_scale=%.3f p_error=%.3fm v_error=%.3fm/s a_error=%.3fm/s2",
              candidate_time_scale,
              (patch_position - interval.rejoin_position).norm(),
              (patch_velocity - reference_velocity).norm(),
              (patch_acceleration - reference_acceleration).norm());
          return reject_planned_trajectory();
        }
      }

      // The optimizer is not the final safety authority.  Sample the complete
      // B-spline again immediately before publishing it to the controller so a
      // successful solver result can never authorize an occupied trajectory.
      constexpr double validation_dt = 0.02;
      double clearance_escape_deadline =
          low_speed_local_repair && minimum_initial_duration > 0.0
              ? minimum_initial_duration
              : 0.75;
      bool candidate_started_inside_clearance = false;
      bool candidate_exited_clearance = false;
      bool candidate_requires_clearance_limited_profile = false;
      size_t initial_clearance_violations = 0;
      for (double t = 0.0; t <= info->duration_; t += validation_dt)
      {
        const Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
        const Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(
            std::min(t + validation_dt, info->duration_));
        Eigen::Vector3d blocked_point;
        size_t clearance_violations = 0;
        // Legacy opt-in lateral repair keeps the measured body yaw. The
        // default disturbance recapture follows the candidate tangent, so
        // the controller aligns in place before it advances.
        const double candidate_yaw =
            low_speed_local_repair && fixed_body_yaw_repair
                ? local_repair_body_yaw
                : estimateYawFromSegment(pos, pos_next);
        const bool clearance_blocked = footprintOccupiedWithMargin(
                map, pos, candidate_yaw,
                planning_clearance_margin_, &blocked_point,
                &clearance_violations);
        if (t <= 1e-9)
        {
          candidate_started_inside_clearance = clearance_blocked;
          initial_clearance_violations = clearance_violations;
        }

        if (!clearance_blocked)
        {
          if (!low_speed_local_repair && footprintOccupiedWithMargin(
                  map, pos, candidate_yaw,
                  preferred_clearance_margin_))
            candidate_requires_clearance_limited_profile = true;
          candidate_exited_clearance = true;
          continue;
        }

        const bool physical_collision =
            map->getInflateOccupancy(pos, candidate_yaw) != 0;
        const bool valid_escape_prefix =
            clearanceViolationAllowedDuringEscape(
                candidate_started_inside_clearance,
                candidate_exited_clearance, physical_collision, t,
                clearance_escape_deadline, clearance_violations,
                initial_clearance_violations);
        const ClearanceResult clearance_result = classifyClearanceResult(
            clearance_blocked, physical_collision, valid_escape_prefix,
            t > clearance_escape_deadline,
            clearance_violations > initial_clearance_violations,
            clearance_violations);
        if (!clearance_result.accepted())
        {
          publishFailureEvidence(
              planning_request_id, info->traj_id_, "FINAL_CLEARANCE",
              "CANDIDATE_CLEARANCE_REJECTED", blocked_point,
              static_cast<uint32_t>(replan_fail_count_ + 1),
              planning_clearance_margin_, clearance_result.failure);
          RCLCPP_ERROR(node_->get_logger(),
                       "[CANDIDATE_CLEARANCE_REJECTED] t=%.3fs center=(%.3f,%.3f,%.3f) hit=(%.3f,%.3f) hard_margin=%.2fm violations=%zu initial=%zu physical_collision=%d failure=%u",
                       t, pos.x(), pos.y(), pos.z(), blocked_point.x(),
                       blocked_point.y(), planning_clearance_margin_,
                       clearance_violations, initial_clearance_violations,
                       physical_collision,
                       static_cast<unsigned int>(clearance_result.failure));
          return reject_planned_trajectory();
        }
      }
      if (candidate_started_inside_clearance && !candidate_exited_clearance)
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[CANDIDATE_CLEARANCE_REJECTED] candidate starts inside the hard planning belt but does not exit within %.2fs",
            clearance_escape_deadline);
        return reject_planned_trajectory();
      }

      if (candidate_requires_clearance_limited_profile || low_speed_local_repair)
      {
        // Retiming a splice would invalidate its already checked join times.
        // Fall back to a full candidate instead of silently changing a patch.
        if (seed_path || (low_speed_local_repair && fixed_body_yaw_repair) ||
            (low_speed_local_repair && local_repair_max_speed_ < min_forward_command_))
          return reject_planned_trajectory();
        // The measured gait is not capable of following an arbitrarily slow
        // reference. Retiming limits reference speed to its observed response;
        // the separately authorized command floor is checked conservatively.
        double speed_bound = 0.0;
        const auto velocity_points = info->velocity_traj_.getControlPoint();
        for (int i = 0; i < velocity_points.cols(); ++i)
          speed_bound = std::max(speed_bound, velocity_points.col(i).head<2>().norm());
        const double scale = std::max(1.0, speed_bound / min_forward_effective_speed_);
        if (scale > 1.0)
        {
          Eigen::VectorXd retimed_knots = info->position_traj_.getKnot();
          const double time_origin = retimed_knots(3);
          retimed_knots = (retimed_knots.array() - time_origin) * scale + time_origin;
          info->position_traj_.setKnot(retimed_knots);
          info->velocity_traj_ = info->position_traj_.getDerivative();
          info->acceleration_traj_ = info->velocity_traj_.getDerivative();
          info->duration_ = info->position_traj_.getTimeSum();
          clearance_escape_deadline *= scale;
        }
        for (double t = 0.0; t <= info->duration_; t += 0.10)
        {
          const Eigen::Vector3d origin = info->position_traj_.evaluateDeBoorT(t);
          const Eigen::Vector3d before = info->position_traj_.evaluateDeBoorT(
              std::max(0.0, t - 0.05));
          const Eigen::Vector3d next = info->position_traj_.evaluateDeBoorT(
              std::min(t + 0.05, info->duration_));
          const double yaw = estimateYawFromSegment(before, next);
          const double target_time = forwardGaitLookaheadTime(t, info->duration_,
              [&](double sample_time) { return info->position_traj_.evaluateDeBoorT(sample_time); });
          const Eigen::Vector3d target = info->position_traj_.evaluateDeBoorT(target_time);
          const double steering_yaw = (target - origin).head<2>().norm() > 1e-6
              ? estimateYawFromSegment(origin, target) : yaw;
          const double turn = normalizePlanarAngle(steering_yaw - yaw);
          const int turn_samples = std::max(1, static_cast<int>(std::ceil(
              std::abs(turn) / (5.0 * M_PI / 180.0))));
          for (int i = 0; i <= turn_samples; ++i)
          {
            const double body_yaw = interpolatePlanarYaw(yaw, steering_yaw,
                static_cast<double>(i) / turn_samples);
            size_t violations = 0;
            const bool blocked = footprintOccupiedWithMargin(map, origin,
                body_yaw, planning_clearance_margin_, nullptr, &violations);
            if (blocked && !handoffClearanceViolationIsAdmissible(
                    low_speed_local_repair, candidate_started_inside_clearance,
                    t > clearance_escape_deadline,
                    map->getInflateOccupancy(origin, body_yaw) != 0,
                    violations, initial_clearance_violations))
              return reject_planned_trajectory();
          }
          // Certify the controller's steering direction and bounded turning
          // braking paths, not only the theoretical spline tangent.
          const double reaction = command_stop_latency_ + predictive_replan_reaction_time_;
          const double horizon = reaction + min_forward_command_ / guaranteed_braking_deceleration_;
          for (double rate : {-clearance_limited_max_yaw_rate_, 0.0, clearance_limited_max_yaw_rate_})
          {
            PlanarBrakingState sweep;
            sweep.position = origin.head<2>();
            sweep.yaw = steering_yaw;
            size_t previous_count = 0;
            const bool initially_blocked = footprintOccupiedWithMargin(map, origin,
                sweep.yaw, planning_clearance_margin_, nullptr, &previous_count);
            bool sweep_exited = !initially_blocked;
            double elapsed = 0.0;
            while (elapsed < horizon)
            {
              const double dt = std::min(0.05, horizon - elapsed);
              advancePlanarBrakingState(&sweep, min_forward_command_, steering_yaw,
                  steering_yaw, rate, guaranteed_braking_deceleration_, reaction, elapsed, dt);
              elapsed += dt;
              const Eigen::Vector3d point(sweep.position.x(), sweep.position.y(), origin.z());
              size_t count = 0;
              const bool blocked = footprintOccupiedWithMargin(map, point, sweep.yaw,
                  planning_clearance_margin_, nullptr, &count);
              if (blocked && !handoffClearanceViolationIsAdmissible(
                      low_speed_local_repair, initially_blocked, sweep_exited,
                      map->getInflateOccupancy(point, sweep.yaw) != 0, count, previous_count))
              {
                RCLCPP_WARN(node_->get_logger(),
                    "[FORWARD_GAIT_STEERING_REJECTED] t=%.2f yaw_rate=%.2f", t, rate);
                return reject_planned_trajectory();
              }
              sweep_exited = sweep_exited || !blocked;
              previous_count = count;
            }
          }
          const double stopping_distance = min_forward_command_ *
              (command_stop_latency_ + predictive_replan_reaction_time_) +
              min_forward_command_ * min_forward_command_ / (2.0 * guaranteed_braking_deceleration_);
          size_t previous = 0;
          const bool starts_inside = footprintOccupiedWithMargin(
              map, origin, yaw, planning_clearance_margin_, nullptr, &previous);
          bool exited = !starts_inside;
          for (double distance = 0.02; distance <= stopping_distance + 0.02; distance += 0.02)
          {
            const Eigen::Vector3d point = origin + std::min(distance, stopping_distance) *
                Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
            size_t violations = 0;
            const bool blocked = footprintOccupiedWithMargin(
                map, point, yaw, planning_clearance_margin_, nullptr, &violations);
            if (blocked && !handoffClearanceViolationIsAdmissible(
                    low_speed_local_repair, starts_inside, exited,
                    map->getInflateOccupancy(point, yaw) != 0, violations, previous))
            {
              RCLCPP_WARN(node_->get_logger(),
                          "[FORWARD_GAIT_SWEEP_REJECTED] command=%.3f effective=%.3f t=%.2f; trying another local candidate",
                          min_forward_command_, min_forward_effective_speed_, t);
              return reject_planned_trajectory();
            }
            exited = exited || !blocked;
            previous = violations;
          }
        }
        RCLCPP_INFO(node_->get_logger(),
                    "[FORWARD_GAIT_PROFILE] command_min=%.3f reference_max=%.3f time_scale=%.2f duration=%.2fs",
                    min_forward_command_, min_forward_effective_speed_, scale, info->duration_);
      }

      Eigen::Vector3d live_odom;
      Eigen::Vector3d live_velocity = Eigen::Vector3d::Zero();
      Eigen::Quaterniond live_orient = Eigen::Quaterniond::Identity();
      bool have_live_odom = false;
      bool handoff_started_inside_clearance =
          candidate_started_inside_clearance;
      size_t handoff_initial_clearance_violations =
          initial_clearance_violations;
      {
        std::lock_guard<std::mutex> lock(safety_odom_mutex_);
        have_live_odom = safety_have_odom_;
        live_odom = safety_odom_pos_;
        live_velocity = safety_odom_vel_;
        live_orient = safety_odom_orient_;
      }
      if (have_live_odom)
      {
        const double search_end = std::min(
            info->duration_, tracking_match_forward_time_);
        double matched_time = 0.0;
        double matched_error2 = std::numeric_limits<double>::infinity();
        constexpr double handoff_sample_dt = 0.02;
        for (double t = 0.0; t <= search_end + 1e-6;
             t += handoff_sample_dt)
        {
          const double sample_time = std::min(t, search_end);
          const Eigen::Vector3d sample =
              info->position_traj_.evaluateDeBoorT(sample_time);
          const double error2 =
              (sample.head<2>() - live_odom.head<2>()).squaredNorm();
          if (error2 < matched_error2)
          {
            matched_error2 = error2;
            matched_time = sample_time;
          }
        }

        const Eigen::Vector3d matched_position =
            info->position_traj_.evaluateDeBoorT(matched_time);
        const Eigen::Vector3d connector = matched_position - live_odom;
        const double connector_length = connector.head<2>().norm();
        const double handoff_speed = live_velocity.head<2>().norm();
        const double dynamic_handoff_limit = speedAwareHandoffErrorLimit(
            handoff_speed, guaranteed_braking_deceleration_,
            command_stop_latency_, tracking_degraded_error_,
            rolling_replan_max_start_error_);
        if (!isRollingTrajectoryStartFresh(
                connector_length, dynamic_handoff_limit))
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "[SPEED_AWARE_HANDOFF_HOLD] error=%.2fm dynamic_limit=%.2fm speed=%.2fm/s matched_t=%.2fs; stopping before local repair",
                       connector_length, dynamic_handoff_limit, handoff_speed,
                       matched_time);
          predictive_replan_requested_.store(true);
          tracking_recovery_active_ = true;
          setLocalRecoveryState(
              LocalRecoveryState::LOCAL_REPAIR, connector_length,
              "speed_aware_handoff_hold");
          requestExecutionStop("HANDOFF_PREDICTED_UNSAFE");
          return reject_planned_trajectory();
        }
        const double actual_yaw = std::atan2(
            2.0 * (live_orient.w() * live_orient.z() +
                   live_orient.x() * live_orient.y()),
            1.0 - 2.0 * (live_orient.y() * live_orient.y() +
                         live_orient.z() * live_orient.z()));
        const double tangent_before_time = std::max(
            0.0, matched_time - handoff_sample_dt);
        const double tangent_after_time = std::min(
            info->duration_, matched_time + handoff_sample_dt);
        const Eigen::Vector3d tangent_before =
            info->position_traj_.evaluateDeBoorT(tangent_before_time);
        const Eigen::Vector3d tangent_after =
            info->position_traj_.evaluateDeBoorT(tangent_after_time);
        const double trajectory_yaw =
            low_speed_local_repair && fixed_body_yaw_repair
                ? local_repair_body_yaw
                : estimateYawFromSegment(tangent_before, tangent_after);
        size_t previous_handoff_violations = 0;
        const bool actual_clearance_blocked = footprintOccupiedWithMargin(
            map, live_odom, actual_yaw, planning_clearance_margin_, nullptr,
            &previous_handoff_violations);
        handoff_started_inside_clearance = actual_clearance_blocked;
        handoff_initial_clearance_violations = previous_handoff_violations;
        bool handoff_exited_clearance = !actual_clearance_blocked;

        const auto validate_handoff_pose =
            [&](const Eigen::Vector3d &point, double yaw,
                const char *phase) -> bool
        {
          if (!std::isfinite(yaw))
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "[PLAN_RESULT_DISCARDED] handoff %s has non-finite yaw",
                         phase);
            return false;
          }
          const bool physical_collision =
              map->getInflateOccupancy(point, yaw) != 0;
          size_t clearance_violations = 0;
          const bool clearance_blocked = footprintOccupiedWithMargin(
              map, point, yaw, planning_clearance_margin_, nullptr,
              &clearance_violations);
          if (!clearance_blocked)
          {
            handoff_exited_clearance = true;
            previous_handoff_violations = 0;
            return true;
          }
          const bool admissible = handoffClearanceViolationIsAdmissible(
              low_speed_local_repair, handoff_started_inside_clearance,
              handoff_exited_clearance, physical_collision,
              clearance_violations, previous_handoff_violations);
          if (!admissible)
          {
            RCLCPP_ERROR(
                node_->get_logger(),
                "[PLAN_RESULT_DISCARDED] handoff %s is blocked point=(%.2f,%.2f) physical=%d violations=%zu previous=%zu started_inside=%d",
                phase, point.x(), point.y(), physical_collision ? 1 : 0,
                clearance_violations, previous_handoff_violations,
                handoff_started_inside_clearance ? 1 : 0);
            return false;
          }
          previous_handoff_violations = clearance_violations;
          return true;
        };

        // Match Controller execution: first align the body at the live pose,
        // then traverse the connector.  Coupling rotation and translation in
        // one interpolation missed the pure-rotation footprint that caused
        // the runtime braking sweep to veto an accepted candidate.
        constexpr double handoff_yaw_step = 5.0 * M_PI / 180.0;
        const double yaw_delta = normalizePlanarAngle(
            trajectory_yaw - actual_yaw);
        const int yaw_samples = std::max(
            1, static_cast<int>(std::ceil(
                   std::abs(yaw_delta) / handoff_yaw_step)));
        for (int sample = 1; sample <= yaw_samples; ++sample)
        {
          const double ratio = static_cast<double>(sample) / yaw_samples;
          if (!validate_handoff_pose(
                  live_odom,
                  interpolatePlanarYaw(actual_yaw, trajectory_yaw, ratio),
                  "rotation"))
            return reject_planned_trajectory();
        }
        const int translation_samples = std::max(
            1, static_cast<int>(std::ceil(connector_length / 0.05)));
        for (int sample = 1; sample <= translation_samples; ++sample)
        {
          const double ratio =
              static_cast<double>(sample) / translation_samples;
          if (!validate_handoff_pose(
                  live_odom + ratio * connector, trajectory_yaw,
                  "translation"))
            return reject_planned_trajectory();
        }
      }

      if (safety_generation_.load() != safety_generation_before)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "[PLAN_RESULT_DISCARDED] realtime safety stopped execution during final validation");
        return reject_planned_trajectory();
      }

      if (planning_request_id != active_reference_request_id_.load())
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "[PLAN_RESULT_DISCARDED] reference changed before publication old_request_id=%llu active_request_id=%llu",
            static_cast<unsigned long long>(planning_request_id),
            static_cast<unsigned long long>(active_reference_request_id_.load()));
        return reject_planned_trajectory();
      }
      if (seed_path)
      {
        bool same_interval = false;
        {
          std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
          same_interval = seed_interval && collisionIntervalMatchesExecution(
              predicted_collision_interval_, seed_interval->request_id,
              seed_interval->trajectory_id, seed_interval->map_revision) &&
              predicted_collision_interval_.entry_time == seed_interval->entry_time &&
              predicted_collision_interval_.exit_time == seed_interval->exit_time &&
              predicted_collision_interval_.rejoin_time == seed_interval->rejoin_time;
        }
        if (!same_interval)
          return reject_planned_trajectory();
        const auto latest_snapshot = map->captureInflatedOccupancySnapshot();
        if (latest_snapshot->revision != validation_snapshot->revision)
        {
          map->useInflatedOccupancySnapshotForCurrentThread(latest_snapshot);
          for (double t = 0.0; t <= info->duration_ + 1e-6;
               t += validation_dt)
          {
            const double sample_time = std::min(t, info->duration_);
            const Eigen::Vector3d position =
                info->position_traj_.evaluateDeBoorT(sample_time);
            const Eigen::Vector3d next =
                info->position_traj_.evaluateDeBoorT(
                    std::min(sample_time + validation_dt, info->duration_));
            const double yaw = estimateYawFromSegment(position, next);
            if (footprintOccupiedWithMargin(
                    map, position, yaw, planning_clearance_margin_))
            {
              RCLCPP_WARN(
                  node_->get_logger(),
                  "[LOCAL_INTERVAL_REPAIR_STALE_MAP_BLOCKED] revision=%llu t=%.2f",
                  static_cast<unsigned long long>(latest_snapshot->revision),
                  sample_time);
              return reject_planned_trajectory();
            }
          }
          RCLCPP_INFO(
              node_->get_logger(),
              "[LOCAL_INTERVAL_REPAIR_REVALIDATED] original_revision=%llu latest_revision=%llu",
              static_cast<unsigned long long>(validation_snapshot->revision),
              static_cast<unsigned long long>(latest_snapshot->revision));
        }
      }

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;
      bspline.request_id = planning_request_id;
      bspline.execution_mode = low_speed_local_repair
          ? scan_planner_msgs::msg::Bspline::MODE_RECOVERY_PRIMITIVE
          : (candidate_requires_clearance_limited_profile
                 ? scan_planner_msgs::msg::Bspline::MODE_CLEARANCE_LIMITED
                 : scan_planner_msgs::msg::Bspline::MODE_NORMAL);
      if (bspline.execution_mode !=
          scan_planner_msgs::msg::Bspline::MODE_NORMAL)
      {
        bspline.min_forward_command = min_forward_command_;
        bspline.min_forward_effective_speed = min_forward_effective_speed_;
        bspline.max_forward_speed = low_speed_local_repair
            ? local_repair_max_speed_
            : clearance_limited_max_forward_speed_;
        bspline.max_lateral_speed = low_speed_local_repair
            ? std::min(local_repair_max_speed_,
                       clearance_limited_max_lateral_speed_)
            : clearance_limited_max_lateral_speed_;
        bspline.max_yaw_rate = clearance_limited_max_yaw_rate_;
        bspline.max_tracking_correction =
            clearance_limited_max_tracking_correction_;
      }

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

      // Only the legacy lateral-repair mode has a fixed-yaw contract. Default
      // disturbance recapture leaves yaw_pts empty so the controller follows
      // the validated tangent-yaw profile.
      if (low_speed_local_repair && fixed_body_yaw_repair)
      {
        bspline.yaw_pts.push_back(local_repair_body_yaw);
        bspline.yaw_dt = info->duration_;
      }

      // Publishing is phase one of a trajectory transaction.  The controller
      // may still reject this candidate after matching it against the current
      // physical pose and speed.  Preserve the active execution snapshot and
      // every safety latch until the exact request/trajectory version is ACKed.
      {
        std::lock_guard<std::mutex> lock(pending_handoff_mutex_);
        pending_handoff_.candidate = *info;
        pending_handoff_.previous = executing_trajectory;
        pending_handoff_.request_id = planning_request_id;
        pending_handoff_.trajectory_id = info->traj_id_;
        pending_handoff_.clearance_escape_active =
            handoff_started_inside_clearance;
        pending_handoff_.clearance_escape_deadline = clearance_escape_deadline;
        pending_handoff_.initial_clearance_violations =
            handoff_initial_clearance_violations;
        pending_handoff_.fixed_body_yaw =
            low_speed_local_repair && fixed_body_yaw_repair;
        pending_handoff_.body_yaw = local_repair_body_yaw;
        pending_handoff_.execution_mode = bspline.execution_mode;
        pending_handoff_.structured_repair_segment =
            low_speed_local_repair;
        pending_handoff_.structured_repair_rejoin =
            !low_speed_local_repair &&
            structured_local_repair_active_.load() &&
            structured_local_repair_rejoin_pending_.load();
        pending_handoff_.reference_path_update =
            reference_path_update_pending_;
        pending_handoff_.collision_interval_repair = seed_path != nullptr;
        if (pending_handoff_.collision_interval_repair)
        {
          pending_handoff_.collision_interval_request_id =
              seed_interval->request_id;
          pending_handoff_.collision_interval_trajectory_id =
              seed_interval->trajectory_id;
          pending_handoff_.collision_interval_map_revision =
              seed_interval->map_revision;
        }
        pending_handoff_.reference_request_id =
            pending_reference_request_id_;
        pending_handoff_.submitted_at = std::chrono::steady_clock::now();
        pending_handoff_.timeout_hold_issued_at = {};
        pending_handoff_.timeout_hold_issued = false;
        pending_handoff_.timeout_candidate_rejected = false;
        pending_handoff_.valid = true;
      }
      bspline_pub_->publish(bspline);
      RCLCPP_INFO(
          node_->get_logger(),
          "[CANDIDATE_TRAJECTORY_SUBMITTED] request_id=%llu trajectory=%lld clearance_escape=%d execution_mode=%u; waiting for controller ACK before replacing execution",
          static_cast<unsigned long long>(planning_request_id),
          static_cast<long long>(info->traj_id_),
          candidate_started_inside_clearance ? 1 : 0,
          static_cast<unsigned int>(bspline.execution_mode));

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
    const uint64_t stop_request_id = active_reference_request_id_.load();
    bspline.request_id = stop_request_id;

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

    bspline_pub_->publish(bspline);
    updateExecutionTrajectorySnapshot(*info, stop_request_id);

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
