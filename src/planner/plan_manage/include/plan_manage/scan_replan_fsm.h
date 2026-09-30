#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <scan_planner_msgs/msg/execution_command.hpp>
#include <scan_planner_msgs/msg/execution_state.hpp>
#include <plan_manage/planner_manager.h>
#include <plan_manage/replan_fsm_utils.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner_msgs::msg::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double reference_path_lookahead_;
    double emergency_time_;
    double rolling_replan_retry_period_;
    double rolling_replan_max_start_error_;
    double predictive_replan_reaction_time_{0.25};
    double predictive_hard_stop_min_time_{0.35};
    double tracking_degraded_error_{0.10};
    double tracking_recovery_max_error_{0.30};
    double tracking_match_back_time_{0.40};
    double tracking_match_forward_time_{0.80};
    double planning_clearance_margin_{0.10};
    double local_repair_anchor_min_distance_{0.45};
    double local_repair_anchor_max_distance_{0.90};
    double local_repair_anchor_step_{0.15};
    double local_repair_max_backtrack_{0.60};
    double local_repair_max_speed_{0.25};
    double local_repair_time_margin_{1.20};
    int local_repair_max_candidates_{6};
    double braking_sweep_spatial_step_{0.05};
    double braking_sweep_max_horizon_{4.0};
    double safety_map_stale_warn_age_{0.25};
    double safety_map_stale_hold_age_{0.50};
    double safety_callback_hold_gap_{0.30};
    double odom_velocity_filter_tau_{0.20};
    double motion_estimation_window_{0.35};
    double yaw_rate_filter_tau_{0.15};
    double guaranteed_braking_deceleration_{0.32};
    double command_stop_latency_{0.30};
    int pose_free_release_cycles_{3};
    int local_hold_release_cycles_{3};
    double goal_tolerance_;
    int heading_freeze_max_recoveries_{2};
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    bool live_body_height_{true};
    std::string self_inflation_frame_id_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool preset_started_{false};
    bool rviz_height_ready_;
    std::atomic<bool> go2_execution_frozen_{false};
    std::atomic<bool> go2_heading_stalled_{false};
    std::atomic<double> go2_heading_error_{0.0};
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{5};
    int64_t next_rolling_replan_attempt_ns_{0};
    rclcpp::Time last_freeze_update_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    std::vector<Eigen::Vector3d> reference_path_;
    size_t reference_progress_idx_{0};
    double reference_progress_ratio_{0.0};
    bool reference_path_active_{false};
    bool reference_path_update_pending_{false};
    std::atomic<uint64_t> active_reference_request_id_{0};
    std::atomic<uint64_t> completed_reference_request_id_{0};
    std::atomic<int64_t> last_wait_target_status_ns_{0};
    uint64_t pending_reference_request_id_{0};
    int current_wp_;

    bool flag_escape_emergency_;
    bool emergency_path_pending_{false};
    bool tracking_recovery_active_{false};
    std::atomic<bool> stopped_local_repair_pending_{false};
    std::atomic<bool> stopped_local_repair_active_{false};
    std::atomic<bool> structured_local_repair_active_{false};
    std::atomic<bool> terminal_local_repair_hold_{false};
    bool heading_stall_handled_{false};
    int heading_freeze_recoveries_{0};
    bool collision_segment_pending_{false};
    Eigen::Vector3d last_collision_free_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d first_collision_pos_{Eigen::Vector3d::Zero()};

    struct ExecutionTrajectorySnapshot
    {
      UniformBspline position;
      rclcpp::Time start_time;
      double duration{0.0};
      uint64_t request_id{0};
      int64_t trajectory_id{0};
      bool clearance_escape_active{false};
      double clearance_escape_deadline{0.0};
      size_t initial_clearance_violations{0};
      int clearance_escape_free_cycles{0};
      bool fixed_body_yaw{false};
      double body_yaw{0.0};
      bool valid{false};
    };
    std::mutex execution_snapshot_mutex_;
    ExecutionTrajectorySnapshot execution_snapshot_;
    std::mutex execution_command_mutex_;
    uint64_t last_execution_command_request_id_{0};
    int64_t last_execution_command_trajectory_id_{0};
    int64_t last_execution_command_time_ns_{0};
    std::string last_execution_command_reason_;
    std::mutex safety_odom_mutex_;
    Eigen::Vector3d safety_odom_pos_{Eigen::Vector3d::Zero()};
    // All safety velocities are stored in the odom/world frame. Odometry
    // twist arrives in child_frame_id and must never be used here directly.
    Eigen::Vector3d safety_odom_vel_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond safety_odom_orient_{Eigen::Quaterniond::Identity()};
    double safety_yaw_rate_{0.0};
    int64_t safety_odom_velocity_stamp_ns_{0};
    bool safety_have_odom_{false};
    struct SafetyPoseSample
    {
      int64_t stamp_ns{0};
      Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
      double raw_yaw{0.0};
      double unwrapped_yaw{0.0};
    };
    std::deque<SafetyPoseSample> safety_pose_history_;
    std::vector<Eigen::Vector2d> planning_clearance_offsets_;
    std::atomic<bool> safety_stop_active_{false};
    std::atomic<bool> pose_occupied_latched_{false};
    std::atomic<int> pose_free_confirmation_cycles_{0};
    std::atomic<bool> predictive_hold_latched_{false};
    std::atomic<int> predictive_hold_release_cycles_{0};
    std::atomic<bool> temporal_safety_latched_{false};
    std::atomic<int> temporal_safety_release_cycles_{0};
    std::atomic<uint64_t> safety_generation_{0};
    std::atomic<bool> predictive_replan_requested_{false};
    std::atomic<bool> terminal_repair_requested_{false};
    std::atomic<double> terminal_repair_error_{0.0};
    enum class LocalRecoveryState : uint8_t
    {
      TRACKING = 0,
      TRACKING_DEGRADED = 1,
      LOCAL_REPAIR = 2,
    };
    std::atomic<uint8_t> local_recovery_state_{
        static_cast<uint8_t>(LocalRecoveryState::TRACKING)};
    std::atomic<int64_t> last_fsm_callback_wall_ns_{0};
    std::atomic<bool> fsm_callback_stall_reported_{false};
    std::chrono::steady_clock::time_point last_safety_callback_wall_{};

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
    rclcpp::CallbackGroup::SharedPtr control_feedback_callback_group_;
    rclcpp::CallbackGroup::SharedPtr map_callback_group_;
    rclcpp::CallbackGroup::SharedPtr map_visualization_callback_group_;
    rclcpp::CallbackGroup::SharedPtr safety_callback_group_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<scan_planner_msgs::msg::ExecutionState>::SharedPtr execution_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_heading_stalled_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr go2_heading_error_sub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blocked_segment_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::ExecutionCommand>::SharedPtr execution_command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    /* helper functions */
    bool callReboundReplan(
        bool flag_use_poly_init, bool flag_randomPolyTraj,
        const Eigen::Vector3d *local_target_override = nullptr,
        bool low_speed_local_repair = false); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    bool tryStructuredLocalRepair();
    void setStartStateFromOdomOrCurrentTraj();
    void refreshPlanningOdomFromSafety();
    void alignStartStateToReferencePath();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool adjustGlobalTargetIfOccupied();
    void getLocalTarget();
    void getReferencePathLocalTarget();
    void finishProcess();
    void publishSelfInflationMarker();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();
    void requestExecutionStop(const std::string &reason,
                              bool publish_reference_status = true);
    void publishStatus(const std::string &status);
    void publishReferenceStatus(const std::string &status, uint64_t request_id = 0);
    void publishBlockedSegment();
    void updateExecutionTrajectorySnapshot(const LocalTrajData &info,
                                           uint64_t request_id,
                                           bool clearance_escape_active = false,
                                           double clearance_escape_deadline = 0.0,
                                           size_t initial_clearance_violations = 0,
                                           bool fixed_body_yaw = false,
                                           double body_yaw = 0.0);
    void setLocalRecoveryState(LocalRecoveryState state,
                               double tracking_error,
                               const char *reason);
    void tripRealtimeSafety(const std::string &reason,
                            const Eigen::Vector3d *last_free = nullptr,
                            const Eigen::Vector3d *first_blocked = nullptr,
                            uint64_t request_id = 0,
                            int64_t trajectory_id = 0);
    bool footprintOccupiedWithMargin(
        const GridMap::Ptr &map, const Eigen::Vector3d &position,
        double yaw, double margin,
        Eigen::Vector3d *first_blocked = nullptr,
        size_t *blocked_count = nullptr) const;
    void latchLocalSafetyHold(const std::string &reason,
                              std::atomic<bool> &source_latch);

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void checkCollisionRealtimeCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void safetyOdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void executionStateCallback(
        const scan_planner_msgs::msg::ExecutionState::ConstSharedPtr &msg);
    void go2HeadingStalledCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
    void go2HeadingErrorCallback(const std_msgs::msg::Float64::ConstSharedPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
