#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
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
    double goal_tolerance_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool preset_started_{false};
    bool rviz_height_ready_;
    std::atomic<bool> go2_execution_frozen_{false};
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
    uint64_t pending_reference_request_id_{0};
    int current_wp_;

    bool flag_escape_emergency_;
    bool emergency_path_pending_{false};
    bool tracking_recovery_active_{false};
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
      bool valid{false};
    };
    std::mutex execution_snapshot_mutex_;
    ExecutionTrajectorySnapshot execution_snapshot_;
    std::mutex safety_odom_mutex_;
    Eigen::Vector3d safety_odom_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d safety_odom_vel_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond safety_odom_orient_{Eigen::Quaterniond::Identity()};
    bool safety_have_odom_{false};
    std::atomic<bool> safety_stop_active_{false};
    std::atomic<uint64_t> safety_generation_{0};
    std::chrono::steady_clock::time_point last_safety_callback_wall_{};

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
    rclcpp::CallbackGroup::SharedPtr map_callback_group_;
    rclcpp::CallbackGroup::SharedPtr map_visualization_callback_group_;
    rclcpp::CallbackGroup::SharedPtr safety_callback_group_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr safety_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blocked_segment_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
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
    void requestExecutionStop(const std::string &reason);
    void publishStatus(const std::string &status);
    void publishReferenceStatus(const std::string &status, uint64_t request_id = 0);
    void publishBlockedSegment();
    void updateExecutionTrajectorySnapshot(const LocalTrajData &info,
                                           uint64_t request_id);
    void tripRealtimeSafety(const std::string &reason,
                            const Eigen::Vector3d *last_free = nullptr,
                            const Eigen::Vector3d *first_blocked = nullptr,
                            uint64_t request_id = 0,
                            int64_t trajectory_id = 0);

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void checkCollisionRealtimeCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void safetyOdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);

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
