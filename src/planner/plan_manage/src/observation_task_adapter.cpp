#include <plan_manage/scan_replan_fsm.h>

namespace scan_planner {

bool SCANReplanFSM::requestTaskRestartFromRest(const std::string& reason, bool current_turn) {
  if (!task_mode_ || !observation_task_) return false;
  if(task_restart_from_rest_requested_.load()) return true;
  if(!current_turn && (!rolling_task_handoff_.load() || !rolling_predecessor_id_.load() ||
      task_restart_from_rest_used_.load())) return false;
  task_restart_current_turn_.store(current_turn);
  task_restart_wait_started_=0;
  task_transition_hold_stamp_.store(0);
  task_restart_from_rest_requested_.store(true);
  requestExecutionStop(current_turn ? "TURN_RECERTIFICATION_REQUIRES_STOP" :
                                      "TASK_TRANSITION_REQUIRES_STOP", false);
  RCLCPP_WARN(node_->get_logger(),
      "[TASK_TRANSITION_WAIT_STOP] task=%llu request=%llu reason=%s; same objective, no station exclusion",
      static_cast<unsigned long long>(task_id_.load()),
      static_cast<unsigned long long>(active_reference_request_id_.load()),reason.c_str());
  return true;
}

void SCANReplanFSM::publishObservationFeedback(uint8_t state, const std::string& reason) {
  if (!task_mode_ || !observation_task_) return;
  scan_planner_msgs::msg::ObservationFeedback feedback;
  feedback.header.stamp = node_->now();
  feedback.header.frame_id = self_inflation_frame_id_;
  feedback.task_id = task_id_.load();
  feedback.task_version = task_version_.load();
  feedback.request_id = active_reference_request_id_.load();
  feedback.candidate_id = task_candidate_id_.load();
  feedback.state = state;
  feedback.station_adjustments = task_station_adjustments_;
  feedback.route_graph_revision=observation_task_->route_graph_revision;
  feedback.route_connection_cursor=static_cast<uint32_t>(task_route_cursor_);
  feedback.reason = reason;
  if(state==scan_planner_msgs::msg::ObservationFeedback::LOCAL_MOTION_EXHAUSTED ||
     state==scan_planner_msgs::msg::ObservationFeedback::EXECUTION_BLOCKED ||
     state==scan_planner_msgs::msg::ObservationFeedback::TASK_EXHAUSTED ||
     state==scan_planner_msgs::msg::ObservationFeedback::PATH_REJECTED) {
    std::lock_guard<std::mutex> lock(task_failure_trace_mutex_);
    feedback.trigger_reason=task_failure_trigger_;
    feedback.recovery_reason=task_failure_recovery_;
    feedback.planning_reason=task_failure_planning_;
    RCLCPP_WARN(node_->get_logger(),
        "[TASK_MOTION_FAILURE_TRACE] task=%llu reason=%s trigger=%s recovery=%s planning=%s",
        static_cast<unsigned long long>(feedback.task_id),reason.c_str(),
        feedback.trigger_reason.c_str(),feedback.recovery_reason.c_str(),feedback.planning_reason.c_str());
    // An exhausted motion family is not proof that every departure is blocked.
    // Only current physical footprint occupancy gives a pose-wide scope.
    if(reason=="RECOVERY_START_FOOTPRINT_OCCUPIED" ||
       footprintOccupiedWithMargin(planner_manager_->grid_map_,odom_pos_,getOdomYaw(),0.0))
      feedback.failure_scope=scan_planner_msgs::msg::ObservationFeedback::FAILURE_CURRENT_POSE;
  }
  // A reached station is measured, not the requested nominal pose.
  const auto& point = (state == scan_planner_msgs::msg::ObservationFeedback::STATION_REACHED ||
      state == scan_planner_msgs::msg::ObservationFeedback::EXECUTION_BLOCKED ||
      state == scan_planner_msgs::msg::ObservationFeedback::LOCAL_MOTION_EXHAUSTED)
      ? odom_pos_ : task_station_;
  feedback.station.x = point.x(); feedback.station.y = point.y(); feedback.station.z = point.z();
  {
    std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
    if (execution_snapshot_.valid) feedback.trajectory_id = execution_snapshot_.trajectory_id;
  }
  task_feedback_pub_->publish(feedback);
}

void SCANReplanFSM::publishTaskCancellationFeedback(
    TaskVersion version, uint8_t state, const std::string& reason) {
  scan_planner_msgs::msg::ObservationFeedback feedback;
  feedback.header.stamp = node_->now();
  feedback.header.frame_id = self_inflation_frame_id_;
  feedback.task_id = version.first; feedback.task_version = version.second;
  feedback.state = state;
  feedback.reason = reason;
  task_feedback_pub_->publish(feedback);
}

void SCANReplanFSM::retireObservationTask(TaskVersion version) {
  if (version.first == 0 || version.second == 0) return;
  if (task_retirement_pending_ && version <= retiring_task_) return;
  // Cancelling a pending successor must also stop/fence its predecessor.
  if (rolling_predecessor_id_.load()) {
    scan_planner_msgs::msg::ExecutionCommand command;
    command.command = scan_planner_msgs::msg::ExecutionCommand::COMMAND_RETIRE_TASK;
    command.task_id = rolling_predecessor_id_.exchange(0);
    command.task_version = rolling_predecessor_version_.exchange(0);
    command.reason = "ROLLING_SUCCESSOR_CANCELLED";
    execution_command_pub_->publish(command);
    retired_task_ = std::max(retired_task_,TaskVersion{command.task_id,command.task_version});
  }
  rolling_task_handoff_.store(false);
  task_turn_stage_active_=false;task_turn_stage_trajectory_id_=0;task_turn_stage_rest_gate_={};
  task_restart_from_rest_requested_.store(false);
  task_restart_current_turn_.store(false);
  retiring_task_ = std::max(retiring_task_, version);
  retired_task_ = std::max(retired_task_, version);
  task_retirement_pending_ = true;
  last_task_retirement_ns_ = 0;
  task_retirement_started_ns_ = node_->now().nanoseconds();
  task_retirement_unconfirmed_ = false;
  have_target_ = false;
  reference_path_active_ = false;
  reference_path_update_pending_ = false;
  predictive_replan_requested_.store(false);
  stopped_local_repair_pending_.store(false);
  terminal_local_repair_hold_.store(true);
  changeFSMExecState(WAIT_TARGET, "TASK_RETIRING");
}

void SCANReplanFSM::confirmObservationTaskRetirement(TaskVersion version) {
  if (!version.first || !version.second || version > retired_task_) return;
  // Controller's existing retirement contract fences every version up to
  // this watermark. Only an actual ACK (or an acknowledged successor swap)
  // may advance the confirmed watermark, never merely sending a command.
  confirmed_retired_task_ = std::max(confirmed_retired_task_, version);
  if (!task_retirement_pending_ || version != retiring_task_) {
    // A predecessor confirmation must not clear the successor's pending
    // cancellation, but its exact terminal fact still needs to be delivered.
    publishTaskCancellationFeedback(version,
        scan_planner_msgs::msg::ObservationFeedback::CANCEL_ACK,
        "CONTROLLER_TASK_RETIRED");
    return;
  }
  task_retirement_pending_ = false;
  task_retirement_unconfirmed_ = false;
  {
    std::lock_guard<std::mutex> lock(pending_handoff_mutex_);
    if (pending_handoff_.valid &&
        taskExecutionIsRetired({pending_handoff_.task_id, pending_handoff_.task_version}, retired_task_)) {
      planner_manager_->local_data_.traj_id_ = reserveTrajectoryIdAfterAmbiguousHandoff(
          planner_manager_->local_data_.traj_id_, pending_handoff_.trajectory_id);
      pending_handoff_.valid = false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
    execution_snapshot_.valid = false;
    predicted_collision_interval_.valid = false;
  }
  // An actual retirement ACK fences every old candidate. Only now release
  // recovery/rejoin ownership, so a new task is not deferred behind a retired
  // transaction. Preserve the physical departure memory across task changes.
  structured_local_repair_active_.store(false);
  structured_local_repair_executing_.store(false);
  structured_local_repair_finished_.store(false);
  structured_local_repair_rejoin_pending_.store(false);
  structured_failed_rejoin_pending_.store(false);
  stopped_local_repair_pending_.store(false);
  stopped_local_repair_active_.store(false);
  task_lateral_repair_authorized_.store(false);
  // Cancellation may name a queued successor, not observation_task_.
  // Confirm the cancelled identity so Explorer can finish its transaction.
  publishTaskCancellationFeedback(version,
      scan_planner_msgs::msg::ObservationFeedback::CANCEL_ACK, "CONTROLLER_TASK_RETIRED");
  observation_task_.reset();
  task_id_.store(0); task_version_.store(0);
}

void SCANReplanFSM::observationTaskCallback(
    const scan_planner_msgs::msg::ObservationTask::ConstSharedPtr& msg) {
  if (!task_mode_ || !msg || !msg->task_id || !msg->task_version) return;
  const TaskVersion incoming{msg->task_id, msg->task_version};
  if (incoming <= retired_task_) {
    // Reject old motion immediately, but repeat a cancellation terminal only
    // when Controller has confirmed it. A local fence alone is not an ACK.
    if (msg->lifecycle == scan_planner_msgs::msg::ObservationTask::CANCELLED ||
        msg->lifecycle == scan_planner_msgs::msg::ObservationTask::SUPERSEDED) {
      if (incoming <= confirmed_retired_task_) {
        publishTaskCancellationFeedback(incoming,
            scan_planner_msgs::msg::ObservationFeedback::CANCEL_ACK,
            "CONTROLLER_TASK_ALREADY_RETIRED");
      } else {
        scan_planner_msgs::msg::ExecutionCommand command;
        command.command = scan_planner_msgs::msg::ExecutionCommand::COMMAND_RETIRE_TASK;
        command.task_id = incoming.first; command.task_version = incoming.second;
        command.reason = "TASK_RETIREMENT_CONFIRMATION_PENDING";
        execution_command_pub_->publish(command);
      }
    }
    return;
  }
  const TaskVersion current{task_id_.load(), task_version_.load()};
  if (msg->lifecycle == scan_planner_msgs::msg::ObservationTask::CANCELLED ||
      msg->lifecycle == scan_planner_msgs::msg::ObservationTask::SUPERSEDED) {
    const bool cancels_queued_task = deferred_task_ &&
        incoming == TaskVersion{deferred_task_->task_id, deferred_task_->task_version};
    if (cancels_queued_task)
      deferred_task_.reset();
    // A rejected/unprocessed successor still needs a Controller fence and
    // CANCEL_ACK. Otherwise Explorer waits forever for its cancellation.
    if (incoming == current || cancels_queued_task || incoming > current)
      retireObservationTask(incoming);
    else {
      if (incoming == TaskVersion{rolling_predecessor_id_.load(),rolling_predecessor_version_.load()}) {
        // An explicit cancellation of the still-running predecessor is not
        // information completion. Stop it without discarding prepared G2.
        scan_planner_msgs::msg::ExecutionCommand command;
        command.command = scan_planner_msgs::msg::ExecutionCommand::COMMAND_RETIRE_TASK;
        command.task_id = incoming.first; command.task_version = incoming.second;
        command.reason = "ROLLING_PREDECESSOR_CANCELLED";
        execution_command_pub_->publish(command);
        rolling_task_handoff_.store(false);
      }
      retired_task_ = std::max(retired_task_, incoming);
    }
    return;
  }
  const auto finite_point=[](const geometry_msgs::msg::Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  };
  const char* invalid = nullptr;
  if (msg->coarse_route.size() > 4096) invalid = "coarse_route_size";
  else if (msg->target_cells.empty() || msg->target_cells.size() > 256 ||
           msg->target_cell_count < msg->target_cells.size()) invalid = "target_cells_size";
  else if (msg->header.frame_id != self_inflation_frame_id_) invalid = "frame_id";
  else if (!std::all_of(msg->coarse_route.begin(), msg->coarse_route.end(), finite_point)) invalid = "coarse_route_nonfinite";
  else if (!std::all_of(msg->target_cells.begin(), msg->target_cells.end(), finite_point)) invalid = "target_cells_nonfinite";
  else if (msg->lifecycle != scan_planner_msgs::msg::ObservationTask::ACTIVE) invalid = "lifecycle";
  else if (incoming < current) invalid = "stale_task_version";
  else if (!msg->request_id) invalid = "request_id";
  else if ((msg->predecessor_task_id == 0) != (msg->predecessor_task_version == 0)) invalid = "predecessor_identity";
  else if (msg->predecessor_task_id && incoming != current &&
           TaskVersion{msg->predecessor_task_id,msg->predecessor_task_version} != current)
    invalid = "predecessor_identity";
  else if (!std::isfinite(msg->resolution) || msg->resolution <= 0) invalid = "resolution";
  else if (!std::isfinite(msg->corridor_radius) || msg->corridor_radius <= 0 || msg->corridor_radius > 10) invalid = "corridor_radius";
  else if (!std::isfinite(msg->observation_radius) || msg->observation_radius <= 0 || msg->observation_radius > 20) invalid = "observation_radius";
  else if (!msg->max_station_adjustments || msg->max_station_adjustments > 32) invalid = "station_budget";
  else if (msg->representative_selection_version != 1) invalid = "representative_selection_version";
  else if (!finite_point(msg->goal)) invalid = "goal_nonfinite";
  else if (msg->has_directional_gate && (!finite_point(msg->gate_origin) ||
      !std::isfinite(msg->gate_direction.x) || !std::isfinite(msg->gate_direction.y) ||
      std::hypot(msg->gate_direction.x,msg->gate_direction.y)<1e-6 ||
      !std::isfinite(msg->gate_min_progress))) invalid = "directional_gate";
  else if (!std::all_of(msg->terminal_cells.begin(), msg->terminal_cells.end(), finite_point)) invalid = "terminal_cells_nonfinite";
  else if (msg->route_portals.size()>256 || (!msg->route_portals.empty() &&
      (!msg->local_geometry_autonomy || !msg->route_graph_revision))) invalid = "route_portals";
  if(!invalid) for(const auto& portal:msg->route_portals) {
    if(portal.source_node_id==portal.target_node_id || !finite_point(portal.center) ||
       !std::isfinite(portal.direction.x) || !std::isfinite(portal.direction.y) ||
       std::hypot(portal.direction.x,portal.direction.y)<1e-6 ||
       !std::isfinite(portal.radius) || portal.radius<=0 || portal.radius>msg->corridor_radius) {
      invalid="route_portal_geometry";break;
    }
  }
  if(!invalid && incoming==current && observation_task_ &&
      (msg->local_geometry_autonomy!=observation_task_->local_geometry_autonomy ||
       msg->route_graph_revision!=observation_task_->route_graph_revision ||
       msg->route_portals!=observation_task_->route_portals)) invalid="route_contract_changed_without_version";
  if(!invalid) for(size_t i=1;i<msg->route_portals.size();++i)
    if(msg->route_portals[i-1].target_node_id!=msg->route_portals[i].source_node_id) {
      invalid="disconnected_route_portals";break;
    }
  if (invalid) {
    RCLCPP_WARN(node_->get_logger(), "[OBSERVATION_TASK_REJECTED] invalid contract task=%llu version=%llu request=%llu field=%s",
                static_cast<unsigned long long>(msg->task_id), static_cast<unsigned long long>(msg->task_version),
                static_cast<unsigned long long>(msg->request_id), invalid);
    // Use incoming lineage, not the currently executing task's lineage. A
    // rejected message is not geometric evidence against its Viewpoint.
    publishStatus("REFERENCE_PATH_REJECTED reason=INVALID_TASK_CONTRACT field=" + std::string(invalid) +
        " request_id=" + std::to_string(msg->request_id) +
        " task_id=" + std::to_string(msg->task_id) +
        " task_version=" + std::to_string(msg->task_version));
    return;
  }
  if (incoming == current && msg->request_id <= active_reference_request_id_.load()) return;
  if (deferred_task_ && std::make_tuple(msg->task_id, msg->task_version, msg->request_id) <=
      std::make_tuple(deferred_task_->task_id, deferred_task_->task_version, deferred_task_->request_id)) return;
  deferred_task_ = msg;
  if(task_turn_stage_active_.load() && msg->predecessor_task_id) {
    // This is the queued successor's phase, not the active stage's identity.
    // It grants no motion or information completion. Explorer must allow
    // the finite straight leg and confirmed stopping before its ready ACK.
    scan_planner_msgs::msg::ObservationFeedback feedback;
    feedback.header.stamp=node_->now();feedback.header.frame_id=msg->header.frame_id;
    feedback.task_id=msg->task_id;feedback.task_version=msg->task_version;
    feedback.request_id=msg->request_id;
    feedback.state=scan_planner_msgs::msg::ObservationFeedback::TRANSITION_WAIT_STOP;
    feedback.reason="WAITING_FOR_TURN_STAGE";
    task_feedback_pub_->publish(feedback);
  }
  if (current.first != 0 && incoming != current && !msg->predecessor_task_id)
    retireObservationTask(current);
}

bool SCANReplanFSM::processObservationTaskTransitions() {
  if (!task_mode_) return false;
  if(task_forward_stage_failed_.exchange(false)) {
    task_turn_stage_active_=false;task_turn_stage_trajectory_id_=0;
    task_turn_stage_rest_gate_={};task_goal_available_=false;
  }
  std::vector<TaskVersion> confirmations;
  std::vector<std::pair<TaskVersion,Eigen::Vector2d>> failed;
  {
    std::lock_guard<std::mutex> lock(task_event_mutex_);
    confirmations.swap(task_retirement_confirmations_);
    failed.swap(task_failed_goals_);
  }
  for (const auto& confirmation : confirmations)
    confirmObservationTaskRetirement(confirmation);
  for(const auto& item:failed)
    if(item.first==TaskVersion{task_id_.load(),task_version_.load()})
      task_rejected_stations_.push_back(item.second);
  refreshPlanningOdomFromSafety();
  if(observation_task_ && observation_task_->local_geometry_autonomy && !task_retirement_pending_) {
    if(task_route_position_valid_)
      task_route_cursor_=advanceRoutePortal(task_route_portals_,task_route_cursor_,
          task_route_last_position_,odom_pos_.head<2>());
    task_route_last_position_=odom_pos_.head<2>(); task_route_position_valid_=true;
  }
  if (task_retirement_pending_) {
    const int64_t now = node_->now().nanoseconds();
    if (!task_retirement_unconfirmed_ &&
        now-task_retirement_started_ns_ >= static_cast<int64_t>(task_retirement_timeout_*1e9)) {
      task_retirement_unconfirmed_ = true;
      publishTaskCancellationFeedback(retiring_task_,
          scan_planner_msgs::msg::ObservationFeedback::CANCEL_UNCONFIRMED,
          "CONTROLLER_RETIREMENT_TIMEOUT");
      RCLCPP_ERROR(node_->get_logger(),
          "[TASK_CANCEL_UNCONFIRMED] task_id=%llu task_version=%llu; motion remains fenced",
          static_cast<unsigned long long>(retiring_task_.first),
          static_cast<unsigned long long>(retiring_task_.second));
    }
    if (last_task_retirement_ns_ == 0 || now-last_task_retirement_ns_ >= 250000000LL) {
      scan_planner_msgs::msg::ExecutionCommand command;
      command.command = scan_planner_msgs::msg::ExecutionCommand::COMMAND_RETIRE_TASK;
      command.task_id = retiring_task_.first; command.task_version = retiring_task_.second;
      command.reason = "OBSERVATION_TASK_SUPERSEDED";
      execution_command_pub_->publish(command);
      last_task_retirement_ns_ = now;
    }
    return true;
  }
  if (!have_odom_) return false;
  // A prepared successor waits for the certified straight staging leg to
  // finish and settle. It must not reintroduce an early turn in the narrow
  // approach. Explicit cancellation still fences execution immediately.
  if (deferred_task_ && !task_turn_stage_active_.load()) {
    // Do not consume the finite candidate budget before the first accepted
    // sensor update. Keep the request queued while startup safety is holding.
    const auto map = planner_manager_->grid_map_;
    if (map->getMapRevision() == 0 || !std::isfinite(map->getMapAgeSeconds()) ||
        map->getMapAgeSeconds() > 0.25) return true;
    { std::lock_guard<std::mutex> lock(pending_handoff_mutex_);
      if (pending_handoff_.valid) return false; }
    const auto msg = deferred_task_; deferred_task_.reset();
    if (TaskVersion{msg->task_id, msg->task_version} <= retired_task_) return false;
    const bool same_task = task_id_.load() == msg->task_id && task_version_.load() == msg->task_version;
    const bool preserve_objective=same_task && task_goal_available_;
    if (!same_task && msg->predecessor_task_id) {
      // Several information tasks may finish before any successor ACK.
      // Keep ownership of the actual oldest execution until the swap.
      if (!rolling_predecessor_id_.load()) {
        rolling_predecessor_id_.store(msg->predecessor_task_id);
        rolling_predecessor_version_.store(msg->predecessor_task_version);
      }
      rolling_task_handoff_.store(true);
    }
    refreshPlanningOdomFromSafety();
    if (msg->coarse_route.size() < 2 &&
        (odom_pos_.head<2>()-Eigen::Vector2d(msg->goal.x,msg->goal.y)).norm() > msg->observation_radius) {
      publishStatus("REFERENCE_PATH_REJECTED reason=INVALID_TASK_CONTRACT field=approach_route_required request_id=" +
          std::to_string(msg->request_id) + " task_id=" + std::to_string(msg->task_id) +
          " task_version=" + std::to_string(msg->task_version));
      return true;
    }
    observation_task_ = msg;
    task_id_.store(msg->task_id); task_version_.store(msg->task_version);
    // A request refresh is not a new motion attempt or a new station grant.
    if(!same_task) {
      task_rejected_stations_.clear();task_rejected_departures_.clear();
      task_departure_plan_.reset();
    }
    // Rest evidence belongs to this request's exact terminal execution.
    // A same-task observation retry must also collect three new samples.
    task_station_rest_gate_={};
    task_lateral_repair_authorized_.store(false);
    task_turn_stage_active_=false;task_turn_stage_trajectory_id_=0;task_turn_stage_rest_gate_={};
    if (!same_task) {
      { std::lock_guard<std::mutex> lock(task_failure_trace_mutex_);
        task_failure_trigger_.clear();task_failure_recovery_.clear();task_failure_planning_.clear(); }
      task_turn_stage_used_=false;
      task_restart_from_rest_requested_.store(false);
      task_restart_from_rest_used_.store(false);
      task_restart_current_turn_.store(false);
      {
        std::lock_guard<std::mutex> lock(departure_recovery_mutex_);
        const auto& memory=departure_recovery_memory_;
        const bool blocked=memory.used && memory.blocks(odom_pos_.head<2>(),getOdomYaw(),
            observationStationExitMask({memory.position.x(),memory.position.y(),odom_pos_.z()},memory.yaw));
        task_lateral_repair_used_.store(blocked);
        if(!blocked) departure_recovery_memory_.used=false;
      }
      task_restart_wait_started_ = 0;
      task_restart_rest_gate_ = {};
      task_visited_stations_.clear(); task_station_adjustments_ = 0;
      execution_no_progress_exhausted_.store(false);
      task_route_cursor_=0; task_route_portals_.clear();
      for(const auto& portal:msg->route_portals)
        task_route_portals_.push_back({{portal.center.x,portal.center.y},
            {portal.direction.x,portal.direction.y},portal.radius});
      task_route_last_position_=odom_pos_.head<2>(); task_route_position_valid_=true;
    }
    task_terminal_request_ = 0;
    task_departure_blocked_ = false;
    if(!preserve_objective) {
      task_station_target_=false;task_goal_available_=false;
      task_station_={msg->goal.x,msg->goal.y,odom_pos_.z()};
    }
    if (same_task && task_station_adjustments_ >= msg->max_station_adjustments) {
      active_reference_request_id_.store(msg->request_id);
      observationTaskRejected("OBSERVATION_STATION_BUDGET_EXHAUSTED");
      return true;
    }
    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->header.stamp.sec = static_cast<int32_t>(msg->request_id/1000000000ULL);
    path->header.stamp.nanosec = static_cast<uint32_t>(msg->request_id%1000000000ULL);
    for (const auto& point : msg->coarse_route) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path->header; pose.pose.position = point; pose.pose.orientation.w = 1;
      path->poses.push_back(pose);
    }
    if (path->poses.empty()) {
      geometry_msgs::msg::PoseStamped anchor;
      anchor.pose.position.x=odom_pos_.x(); anchor.pose.position.y=odom_pos_.y();
      anchor.pose.orientation.w=1; path->poses.push_back(anchor);
    }
    pathCallback(path);
    if (!have_target_) observationTaskRejected("TASK_REFERENCE_NOT_ACTIVATABLE");
  }
  if (task_restart_from_rest_requested_.load()) {
    const int64_t now = node_->now().nanoseconds();
    if (!task_restart_wait_started_) {
      task_restart_wait_started_ = now;
      task_restart_rest_gate_ = {};
      publishObservationFeedback(scan_planner_msgs::msg::ObservationFeedback::TRANSITION_WAIT_STOP,
                                 "WAITING_FOR_CONFIRMED_PREDECESSOR_HOLD");
    }
    // ACK timeout handling must remain runnable for any unresolved candidate.
    { std::lock_guard<std::mutex> lock(pending_handoff_mutex_);
      if (pending_handoff_.valid) return false; }
    double speed, rate; int64_t pose_stamp;
    { std::lock_guard<std::mutex> lock(safety_odom_mutex_);
      speed=safety_odom_vel_.head<2>().norm(); rate=safety_yaw_rate_;
      pose_stamp=safety_odom_velocity_stamp_ns_; }
    const int64_t hold_stamp=task_transition_hold_stamp_.load();
    const bool fresh=pose_stamp>0 && now>=pose_stamp && now-pose_stamp<=250000000LL;
    if (task_restart_rest_gate_.observe(hold_stamp>0 && now>=hold_stamp && now-hold_stamp<=500000000LL,
                                       fresh,speed,rate,pose_stamp)) {
      task_restart_from_rest_requested_.store(false);
      const bool current_turn=task_restart_current_turn_.load();
      if(!current_turn) {
        task_restart_from_rest_used_.store(true);
        rolling_task_handoff_.store(false);
        task_rejected_stations_.clear();task_rejected_departures_.clear();
        task_goal_available_=false;replan_fail_count_=0;
      }
      // A turn restart keeps the same endpoint, exclusions and replacement
      // budget. Only its stopped initial state is recertified.
      predictive_replan_requested_.store(current_turn);
      refreshPlanningOdomFromSafety();
      bool have_active_execution=false;
      { std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
        have_active_execution=execution_snapshot_.valid; }
      // A first grant has no predecessor curve or execution clock to replan.
      changeFSMExecState(current_turn && have_active_execution ? REPLAN_TRAJ : GEN_NEW_TRAJ,
                         "TASK_RESTART_FROM_REST");
      publishObservationFeedback(scan_planner_msgs::msg::ObservationFeedback::TRANSITION_RESTARTING,
                                 "CONFIRMED_REST_NEW_DEPARTURE_CERTIFICATION");
      RCLCPP_INFO(node_->get_logger(),"[TASK_RESTART_FROM_REST] task=%llu request=%llu speed=%.3f yaw_rate=%.3f; safety and turn certification unchanged",
          static_cast<unsigned long long>(task_id_.load()),
          static_cast<unsigned long long>(active_reference_request_id_.load()),speed,rate);
      return false;
    }
    if (now-task_restart_wait_started_>5000000000LL) {
      task_restart_from_rest_requested_.store(false);
      const bool current_turn=task_restart_current_turn_.exchange(false);
      if(!current_turn) task_restart_from_rest_used_.store(true);
      observationTaskRejected(current_turn ? "TURN_STOP_UNCONFIRMED" : "TRANSITION_STOP_UNCONFIRMED");
    }
    return true;
  }
  if (observation_task_ && task_terminal_request_ == active_reference_request_id_.load()) return true;
  // A turn staging endpoint is not an observation station or information
  // completion. Retain its exact spline through FINISHED and measured rest,
  // then recertify the remaining original task from the actual safe location.
  if(task_turn_stage_active_ && observation_task_ && have_target_) {
    bool current=false,capturing=false;uint8_t state=scan_planner_msgs::msg::ExecutionState::STATE_IDLE;
    { std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
      current=execution_snapshot_.valid && task_turn_stage_trajectory_id_.load()>0 &&
          execution_snapshot_.trajectory_id==task_turn_stage_trajectory_id_.load() &&
          execution_snapshot_.task_id==task_id_.load() &&
          execution_snapshot_.task_version==task_version_.load() &&
          execution_snapshot_.request_id==active_reference_request_id_.load() &&
          execution_snapshot_.candidate_id==task_candidate_id_.load();
      if(current) { state=execution_snapshot_.controller_state;capturing=execution_snapshot_.terminal_settling; } }
    // An unresolved ACK must keep the ordinary timeout/reconciliation runnable.
    if(!current) return false;
    if(safety_stop_active_.load() || state==scan_planner_msgs::msg::ExecutionState::STATE_STALLED ||
        state==scan_planner_msgs::msg::ExecutionState::STATE_SOFT_HOLD ||
        state==scan_planner_msgs::msg::ExecutionState::STATE_HARD_STOP) {
      task_turn_stage_active_=false;task_turn_stage_trajectory_id_=0;
      task_turn_stage_rest_gate_={};task_goal_available_=false;
      return false; // Existing bounded failure/recovery arbitration owns interruption.
    }
    if(capturing) terminal_repair_requested_.store(false);
    if(!capturing && terminal_repair_requested_.exchange(false)) {
      task_turn_stage_active_=false;task_turn_stage_trajectory_id_=0;
      task_turn_stage_rest_gate_={};task_goal_available_=false;
      double error=terminal_repair_error_.load();
      handleExecutionNoProgress(active_reference_request_id_.load(),task_id_.load(),task_version_.load(),
          planner_manager_->local_data_.traj_id_,error,"FORWARD_STAGE_TRACKING_DEVIATION");
      return false; // The same bounded stop/restart arbitration must remain runnable.
    }
    const bool terminal=state==scan_planner_msgs::msg::ExecutionState::STATE_FINISHED;
    double speed,rate;int64_t stamp;
    { std::lock_guard<std::mutex> lock(safety_odom_mutex_);
      speed=safety_odom_vel_.head<2>().norm();rate=safety_yaw_rate_;stamp=safety_odom_velocity_stamp_ns_; }
    const int64_t now=node_->now().nanoseconds();
    if(terminal && !task_turn_stage_wait_started_) task_turn_stage_wait_started_=now;
    if(task_turn_stage_rest_gate_.observe(terminal,stamp>0 && now>=stamp && now-stamp<=250000000LL,
                                       speed,rate,stamp)) {
      task_turn_stage_active_=false;task_turn_stage_trajectory_id_=0;task_turn_stage_rest_gate_={};
      task_goal_available_=false;
      refreshPlanningOdomFromSafety();
      if((odom_pos_-task_station_).head<2>().norm()>kForwardStageCaptureRadius ||
          !forwardStageRestartCertified(odom_pos_,getOdomYaw(),false)) {
        observationTaskRejected("TURN_STAGE_CAPTURE_FAILED");return true;
      }
      RCLCPP_INFO(node_->get_logger(),"[TASK_TURN_STAGE_REACHED] request=%llu actual=(%.3f,%.3f) yaw=%.3f; certifying next departure for same task",
          static_cast<unsigned long long>(active_reference_request_id_.load()),odom_pos_.x(),odom_pos_.y(),getOdomYaw());
      task_rejected_departures_.clear(); // New measured departure pose, same task budget.
      changeFSMExecState(REPLAN_TRAJ,"TASK_TURN_STAGE_REACHED");
    } else if(terminal && now-task_turn_stage_wait_started_>=5000000000LL) {
      observationTaskRejected("TURN_STAGE_STOP_UNCONFIRMED");
    }
    return true; // Never roll a turn candidate before actual staging arrival.
  }
  // A recovery anchor is not an observation station. Finish recovery and
  // re-certify the task approach before evaluating station capture.
  if (structured_local_repair_active_.load() || stopped_local_repair_pending_.load() ||
      stopped_local_repair_active_.load()) return false;
  if (observation_task_ && task_station_target_ && have_target_ &&
      taskStationCaptureReady(observation_task_->local_geometry_autonomy,
          go2_heading_aligning_.load(),(odom_pos_-task_station_).head<2>().norm())) {
    bool acknowledged=false,settling=false;
    { std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
      acknowledged = execution_snapshot_.valid && execution_snapshot_.task_id == task_id_.load() &&
          execution_snapshot_.task_version == task_version_.load() &&
          execution_snapshot_.request_id == active_reference_request_id_.load() &&
          execution_snapshot_.candidate_id == task_candidate_id_.load() &&
          execution_snapshot_.controller_state==scan_planner_msgs::msg::ExecutionState::STATE_FINISHED;
      settling=execution_snapshot_.valid && execution_snapshot_.task_id==task_id_.load() &&
          execution_snapshot_.task_version==task_version_.load() &&
          execution_snapshot_.request_id==active_reference_request_id_.load() &&
          execution_snapshot_.candidate_id==task_candidate_id_.load() &&
          execution_snapshot_.terminal_settling; }
    if(settling) return true; // Preserve this stopped capture; no sibling fitting while settling.
    double speed,rate;int64_t stamp;
    { std::lock_guard<std::mutex> lock(safety_odom_mutex_);
      speed=safety_odom_vel_.head<2>().norm();rate=safety_yaw_rate_;
      stamp=safety_odom_velocity_stamp_ns_; }
    const int64_t now=node_->now().nanoseconds();
    const bool terminal=acknowledged; // Exact current trajectory terminal feedback, not a prior ACK/state.
    if(task_station_rest_gate_.observe(terminal,
        stamp>0 && now>=stamp && now-stamp<=250000000LL,speed,rate,stamp)) {
      if(!observationStationRestartCertified(odom_pos_,getOdomYaw())) {
        task_rejected_stations_.push_back(task_station_.head<2>());
        task_rejected_stations_.push_back(odom_pos_.head<2>());
        task_goal_available_=false;task_station_target_=false;
        task_station_rest_gate_={};
        RCLCPP_WARN(node_->get_logger(),"[TASK_STATION_EXIT_REJECTED] actual stopped pose has no certified departure; retaining information task");
        changeFSMExecState(REPLAN_TRAJ,"TASK_STATION_EXIT_REJECTED");
        return true;
      }
      finishGoalReached("OBSERVATION_STATION");return true;
    }
    // Preserve this exact terminal execution while collecting distinct rest
    // samples; ordinary end-of-segment replanning must not replace it first.
    if(terminal) return true;
  }
  return false;
}

bool SCANReplanFSM::selectObservationTaskTarget(bool report_failure, bool preferred_only, size_t* geometry_remaining) {
    const bool autonomy=observation_task_ && observation_task_->local_geometry_autonomy;
    // Observation purpose survives curve revisions. A transit point extends
    // before capture rather than being rebuilt until its reference runs out.
    bool retain_goal = task_goal_available_ && (autonomy ?
        taskKeepLocalTarget(task_station_target_,
            (task_station_.head<2>()-start_pt_.head<2>()).norm(),
            start_vel_.head<2>().norm(),rolling_replan_prepare_time_) :
        (task_station_.head<2>()-start_pt_.head<2>()).norm()>.6);
    task_selection_retry_=false;
    // The stopped conversion owns its objective until actual capture/failure.
    // A spline retry must not consume a new stage or silently change it.
    if(task_turn_stage_active_ && task_departure_plan_ &&
        task_departure_plan_->kind==TaskDepartureKind::FORWARD_STAGE &&
        task_goal_available_ && task_approach_.size()>1)
      return true;
    const bool had_retained_goal=retain_goal;
    const Eigen::Vector2d previous_goal = task_station_.head<2>();
    task_goal_available_ = false;
    task_transit_speed_limited_=false;
    task_approach_.clear();
  if (!observation_task_ || reference_path_.size()<2) return false;
  auto map=planner_manager_->grid_map_;
  const auto sample=projectReferencePathLookahead(reference_path_,start_pt_,
      reference_progress_idx_,reference_progress_ratio_, std::max(0.5,reference_path_lookahead_));
  const bool in_observation = (start_pt_.head<2>()-Eigen::Vector2d(observation_task_->goal.x,
      observation_task_->goal.y)).norm() <= observation_task_->observation_radius;
  if (!sample.valid && !in_observation) return false;
  reference_progress_idx_=sample.progress_segment; reference_progress_ratio_=sample.progress_ratio;
  LocalTaskDomain domain=observationTaskDomain();
  const auto clearance=autonomy ? map->capturePlanarClearance(start_pt_.z()) : nullptr;
  const auto snapshot=map->captureInflatedOccupancySnapshot();
  // Refit the approach with the retained objective as first preference.
  // Compare wider arrivals before hard-margin alternatives; a new map/odom
  // sample alone does not revoke the observation task or its information need.
  Eigen::Vector2d preferred=retain_goal ? previous_goal :
      (sample.valid ? Eigen::Vector2d(sample.target.head<2>()) : domain.observation_center);
  if(autonomy && !retain_goal && task_route_cursor_<task_route_portals_.size()) {
    const auto& portal=task_route_portals_[task_route_cursor_];
    const Eigen::Vector2d checkpoint=portal.center+0.3*portal.direction.normalized();
    if((checkpoint-start_pt_.head<2>()).norm()<=reference_path_lookahead_)
      preferred=checkpoint;
  }
  const double stopped_distance=min_forward_command_*(command_stop_latency_+predictive_replan_reaction_time_)+
      min_forward_command_*min_forward_command_/(2*guaranteed_braking_deceleration_);
  const double stage_margin=planning_clearance_margin_+kHandoffPositionAllowance+
      kForwardStageCrossTrackBound+
      (kHandoffYawAllowance+kForwardStageYawBound+2.5*M_PI/180.0)*
          (self_double_cylinder_radius_+self_double_cylinder_offset_);
  const auto departure_safe=[&](const Eigen::Vector2d& p,double yaw) {
    if(autonomy) for(size_t i=task_route_cursor_+1;i<task_route_portals_.size();++i)
      if(crossesRoutePortal(task_route_portals_[i],start_pt_.head<2>(),p)) return false;
    return taskDomainContains(domain,p) && !footprintOccupiedWithMargin(map,
        {p.x(),p.y(),start_pt_.z()},yaw,stage_margin);
  };
  const auto turn_stage_safe=[&](const Eigen::Vector2d& p,double yaw) {
    // Certify the whole capture region, not just the mathematical endpoint.
    for(int i=0;i<16;++i) {
      const double angle=2.*M_PI*i/16.;
      if(!taskDomainContains(domain,p+kForwardStageCaptureRadius*
          Eigen::Vector2d(std::cos(angle),std::sin(angle)))) return false;
    }
    return taskDomainContains(domain,p) && !footprintOccupiedWithMargin(map,
        {p.x(),p.y(),start_pt_.z()},yaw,stage_margin+kForwardStageCaptureRadius);
  };
  // A short rolling lookahead may lie behind the staging point. Certify
  // the future turn toward the retained station/information objective,
  // rather than an artificial return to that consumed lookahead sample.
  const Eigen::Vector2d stage_target=retain_goal ? preferred :
      Eigen::Vector2d(observation_task_->goal.x,observation_task_->goal.y);
  const Eigen::Vector2d toward=stage_target-start_pt_.head<2>();
  // Only a stopped new departure can use this rest-to-rest staging action.
  // Ordinary rolling search retains its existing measured-motion contract.
  if(!preferred_only && !task_turn_stage_used_ && !structured_local_repair_active_.load() &&
      start_vel_.head<2>().norm()<=.05 && toward.norm()>.4) {
    const auto departure=taskPlanDeparture(start_pt_.head<2>(),getOdomYaw(),stage_target,
                                         stopped_distance,departure_safe,turn_stage_safe);
    if(departure && departure->kind==TaskDepartureKind::FORWARD_STAGE) {
      task_departure_plan_=departure;
      const Eigen::Vector2d stage=departure->target;
      task_turn_stage_active_=true;task_turn_stage_used_=true;
      task_turn_stage_objective_=stage_target;
      task_turn_stage_trajectory_id_=0;task_turn_stage_wait_started_=0;
      task_turn_stage_rest_gate_={};
      task_approach_={start_pt_,Eigen::Vector3d(stage.x(),stage.y(),start_pt_.z())};
      local_target_pt_=task_approach_.back();local_target_vel_.setZero();
      task_station_=local_target_pt_;task_station_target_=false;task_goal_available_=true;
      const auto x=static_cast<uint32_t>(static_cast<int64_t>(std::floor(stage.x()/.2)));
      const auto y=static_cast<uint32_t>(static_cast<int64_t>(std::floor(stage.y()/.2)));
      task_candidate_id_.store((uint64_t(x)<<32)|y);
      RCLCPP_INFO(node_->get_logger(),"[TASK_FORWARD_TURN_STAGE] request=%llu stage=(%.3f,%.3f) yaw=%.3f retained_target=(%.3f,%.3f); straight incoming motion and future turn/braking certified",
          static_cast<unsigned long long>(active_reference_request_id_.load()),stage.x(),stage.y(),getOdomYaw(),preferred.x(),preferred.y());
      return true;
    }
  }
  double search_margin=preferred_clearance_margin_;
  const bool narrow_start=footprintOccupiedWithMargin(map,start_pt_,getOdomYaw(),preferred_clearance_margin_);
  const double initial_clearance=clearance && snapshot ?
      clearance->footprintDistance(start_pt_.head<2>(),getOdomYaw(),snapshot->body_offset) : 0.;
  const auto point_safe=[&](const Eigen::Vector2d& p,double yaw) {
    if(!taskDomainContains(domain,p)) return false;
    const bool escape_prefix=search_margin>planning_clearance_margin_ && narrow_start &&
        (p-start_pt_.head<2>()).norm()<=.50;
    const double margin=escape_prefix ? planning_clearance_margin_ : search_margin;
    if(footprintOccupiedWithMargin(map,{p.x(),p.y(),start_pt_.z()},yaw,margin)) return false;
    if(escape_prefix && clearance && snapshot &&
        clearance->footprintDistance(p,yaw,snapshot->body_offset)+1e-6<initial_clearance) return false;
    return true;
  };
  const auto turn_safe=[&](const Eigen::Vector2d& p,double before,double after) {
    return taskDepartureCertified(p,before,after,0.,point_safe);
  };
  // Arrival and braking never inherit the narrow-start escape allowance.
  const auto terminal_point_safe=[&](const Eigen::Vector2d& p,double yaw) {
    return taskDomainContains(domain,p) && !footprintOccupiedWithMargin(
        map,{p.x(),p.y(),start_pt_.z()},yaw,search_margin);
  };
  const auto terminal_safe=[&](const Eigen::Vector2d& p,double yaw,bool station) {
    const Eigen::Vector2d return_direction=domain.route.empty() ? Eigen::Vector2d::Zero() :
        Eigen::Vector2d(domain.route.front()-p);
    const double return_yaw=return_direction.norm()>.20 ?
        std::atan2(return_direction.y(),return_direction.x()) : yaw;
    if(station) return taskTerminalRestartCertified(p,yaw,stopped_distance,terminal_point_safe,return_yaw);
    return taskTransitTerminalSpeed(p,yaw,planner_manager_->pp_.max_vel_,min_forward_command_,
        command_stop_latency_+predictive_replan_reaction_time_,guaranteed_braking_deceleration_,terminal_point_safe)>0. ||
        taskTerminalRestartCertified(p,yaw,stopped_distance,terminal_point_safe,return_yaw);
  };
  bool certified_departure_available=false;
  size_t departure_checks=0, departure_certificates=0;
  auto edge_safe=[&](const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
    if(autonomy) for(size_t i=task_route_cursor_+1;i<task_route_portals_.size();++i)
      if(crossesRoutePortal(task_route_portals_[i],a,b)) return false;
    const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
    if ((a-start_pt_.head<2>()).norm()<0.01) {
      ++departure_checks;
      if (taskDeparturePreviouslyRejected(yaw,task_rejected_departures_)) return false;
      // Keep command speed as a conservative bound, not as calibrated actual
      // speed. Do not weaken the existing stopping contract in recovery.
      const double stop_distance=min_forward_command_*
          (command_stop_latency_+predictive_replan_reaction_time_)+
          min_forward_command_*min_forward_command_/(2*guaranteed_braking_deceleration_);
      if (!taskDepartureCertified(start_pt_.head<2>(),getOdomYaw(),yaw,stop_distance,
          point_safe)) return false;
      certified_departure_available=true;
      ++departure_certificates;
    }
    if ((a-start_pt_.head<2>()).norm()<0.05 &&
        !taskRollingDirectionAllowed(b-a,getOdomYaw(),start_vel_.head<2>().norm())) return false;
    const int samples=std::max(1,static_cast<int>(std::ceil((b-a).norm()/0.05)));
    double progress=autonomy ? 0.0 : taskRouteProgress(domain.route,a);
    for(int i=0;i<=samples;++i) {
      const Eigen::Vector2d p=a+(b-a)*(static_cast<double>(i)/samples);
      if(!autonomy) {
        const double next_progress=taskRouteProgress(domain.route,p);
        if(next_progress<progress-0.10) return false;
        progress=std::max(progress,next_progress);
      }
      if (!taskDomainContains(domain,p)) return false;
      if (!point_safe(p,yaw)) return false;
    }
    return true;
  };
  const auto sees=[&](const Eigen::Vector2d& p,const Eigen::Vector2d& endpoint) {
      if ((endpoint-p).norm()>observation_task_->observation_radius) return false;
      // Observation hypothesis, not a footprint-clear motion segment.
      // Unknown space can contain information; known obstacles occlude it.
      // Motion goal/edge/departure certification above remains strict.
      return map->observationRayMayReach(
          {p.x(),p.y(),start_pt_.z()}, {endpoint.x(),endpoint.y(),start_pt_.z()});
  };
  auto visible=[&](const Eigen::Vector2d& p) {
    for(const auto& rejected:task_rejected_stations_) if ((p-rejected).norm()<0.35) return false;
    if(!autonomy) for(const auto& visited:task_visited_stations_) if ((p-visited).norm()<0.35) return false;
    for(const auto& target:observation_task_->target_cells) {
      const Eigen::Vector2d endpoint(target.x,target.y);
      if (!sees(p,endpoint)) continue;
      // Explorer sends only unresolved target representatives. Visibility
      // from a past pose is not evidence that its sensor observed this cell.
      return true;
    }
    return false;
  };
  const auto goal_allowed=[&](const Eigen::Vector2d& p) {
    if ((p-start_pt_.head<2>()).norm()<(autonomy && retain_goal ? 0.20 : 0.4)) return false;
    if (!autonomy && !task_visited_stations_.empty() && domain.route.size()>1) {
      if (taskRouteProgress(domain.route,p) <
          taskRouteProgress(domain.route,task_visited_stations_.back()) - 0.1)
        return false;
    }
    for(const auto& rejected:task_rejected_stations_) if((p-rejected).norm()<0.35) return false;
    if(!autonomy) for(const auto& visited:task_visited_stations_) if((p-visited).norm()<0.35) return false;
    return (p-domain.observation_center).norm()>domain.observation_radius || visible(p);
  };
  const auto cost=[&](const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
    const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
    if(autonomy && clearance && snapshot) {
      const int count=std::max(1,static_cast<int>(std::ceil((b-a).norm()/0.05)));
      double sum=0;
      for(int i=0;i<=count;++i) {
        const auto p=a+(b-a)*(static_cast<double>(i)/count);
        const double d=clearance->footprintDistance(p,yaw,snapshot->body_offset);
        sum+=scan_planner::PlanarClearanceField::preferenceCost(d,preferred_clearance_margin_);
      }
      return (b-a).norm()*sum/(count+1)*50.0;
    }
    double route_distance=std::numeric_limits<double>::infinity();
    for(size_t i=1;i<domain.route.size();++i)
      route_distance=std::min(route_distance,taskSegmentDistance(b,domain.route[i-1],domain.route[i]));
    if(!std::isfinite(route_distance)) route_distance=0;
    const bool narrow=footprintOccupiedWithMargin(map,{b.x(),b.y(),start_pt_.z()},yaw,preferred_clearance_margin_);
    return (b-a).norm()*(0.5*route_distance+(narrow ? 2.0 : 0.0));
  };
  // Compare wide arrivals/turns before the hard-margin family. All passes
  // share the existing 1024-pop local geometry budget, independent of Global.
  size_t remaining=geometry_remaining ? *geometry_remaining : 1024;
  size_t wide_expanded=0,hard_expanded=0;
  const auto search=[&](bool fixed,size_t limit) {
    size_t used=0;
    auto result=selectLocalTaskCandidates(domain,start_pt_.head<2>(),preferred,
        edge_safe,visible,.2,std::min(4.,std::max(1.,reference_path_lookahead_)),
        std::min(limit,remaining),8,goal_allowed,getOdomYaw(),cost,
        fixed ? &previous_goal : nullptr,turn_safe,terminal_safe,&used);
    remaining-=std::min(remaining,used);
    if(geometry_remaining) *geometry_remaining=remaining;
    if(search_margin>planning_clearance_margin_) wide_expanded+=used;else hard_expanded+=used;
    return result;
  };
  std::vector<LocalTaskCandidate> candidates;
  // The retained endpoint may be withdrawn only by endpoint evidence. Failure
  // to reconnect within a bounded search is an attempt result, not that evidence.
  search_margin=planning_clearance_margin_;
  const bool retained_terminal_safe=had_retained_goal &&
      terminal_safe(previous_goal,task_station_arrival_yaw_,task_station_target_);
  search_margin=preferred_clearance_margin_;
  const bool retained_admissible=autonomy && had_retained_goal && retained_terminal_safe &&
      taskDomainContains(domain,previous_goal) &&
      !footprintOccupiedWithMargin(map,{previous_goal.x(),previous_goal.y(),start_pt_.z()},
          task_station_arrival_yaw_,planning_clearance_margin_) &&
      ((previous_goal-domain.observation_center).norm()>domain.observation_radius || visible(previous_goal));
  if(autonomy && retain_goal) candidates=search(true,256);
  if(candidates.empty() && !retained_admissible) {
    retain_goal=false;
    candidates=search(false,preferred_only ? remaining : std::min<size_t>(remaining,512));
  }
  if(candidates.empty() && !preferred_only && remaining) {
    search_margin=planning_clearance_margin_;
    // Keep the previous endpoint only if it also meets the restart contract.
    if(autonomy && had_retained_goal) {
      retain_goal=true;
      candidates=search(true,std::min<size_t>(remaining,128));
      if(candidates.empty()) retain_goal=false;
    }
    if(candidates.empty() && !retained_admissible) candidates=search(false,remaining);
  }
  RCLCPP_INFO(node_->get_logger(),"[TASK_CLEARANCE_FAMILIES] request=%llu wide_pops=%zu hard_pops=%zu remaining=%zu selected_margin=%.2f candidates=%zu",
      static_cast<unsigned long long>(active_reference_request_id_.load()),wide_expanded,
      hard_expanded,remaining,search_margin,candidates.size());
  RCLCPP_INFO(node_->get_logger(),
      "[TASK_DEPARTURE_SEARCH] request=%llu heading=%.3f checked=%zu certified=%zu excluded=%zu candidates=%zu retained_goal=%d",
      static_cast<unsigned long long>(active_reference_request_id_.load()),getOdomYaw(),
      departure_checks,departure_certificates,task_rejected_departures_.size(),candidates.size(),retain_goal);
  if(candidates.empty()) {
    if(retained_admissible) {
      task_goal_available_=true;task_selection_retry_=true;
      RCLCPP_INFO(node_->get_logger(),
          "[TASK_OBJECTIVE_RETAINED] request=%llu goal=(%.3f,%.3f) reason=BOUNDED_CONNECTION_SEARCH_FAILED; same attempt budget, no station blacklist",
          static_cast<unsigned long long>(active_reference_request_id_.load()),previous_goal.x(),previous_goal.y());
      return false;
    }
    if(!report_failure) return false;
    if (!certified_departure_available ||
        footprintOccupiedWithMargin(map,start_pt_,getOdomYaw(),planning_clearance_margin_))
      observationTaskRejected("DEPARTURE_UNSAFE");
    else
      // An empty motion/visibility search does not prove the remote viewpoint
      // is exhausted. Only the explicit station budget may terminate it.
      observationTaskRejected("LOCAL_PATH_FAMILY_EXHAUSTED");
    return false;
  }
  // Compare alternate approaches to the SAME retained endpoint within the
  // existing attempt budget. Curve failure never grants a fresh station.
  const size_t candidate_index=retain_goal ? std::min(candidates.size()-1,
      static_cast<size_t>(std::max(0,replan_fail_count_))) : 0;
  const auto& selected=candidates[candidate_index];
  const auto simple_approach=simplifyTaskApproach(selected.approach,edge_safe,autonomy ? cost :
      std::function<double(const Eigen::Vector2d&,const Eigen::Vector2d&)>{});
  for(const auto& point:simple_approach) task_approach_.push_back({point.x(),point.y(),start_pt_.z()});
  local_target_pt_={selected.goal.x(),selected.goal.y(),start_pt_.z()};
  task_station_=local_target_pt_;
  task_station_target_=selected.observation_station;
  // Packed world-anchored grid coordinates, scoped to task/version. Not a
  // topology node ID, and not inferred from the nearest graph node.
  const auto x=static_cast<uint32_t>(static_cast<int64_t>(std::floor(selected.goal.x()/0.2)));
  const auto y=static_cast<uint32_t>(static_cast<int64_t>(std::floor(selected.goal.y()/0.2)));
  task_candidate_id_.store((static_cast<uint64_t>(x)<<32)|y);
  const Eigen::Vector2d delta=simple_approach.back()-simple_approach[simple_approach.size()-2];
  task_station_arrival_yaw_=std::atan2(delta.y(),delta.x());
  task_departure_plan_=TaskDeparturePlan{TaskDepartureKind::DIRECT,selected.goal,
      selected.goal,getOdomYaw()};
  local_target_vel_.setZero();
  if(!task_station_target_ && delta.norm()>1e-6) {
    const double yaw=std::atan2(delta.y(),delta.x());
    // A short last edge is still a rolling transit endpoint when its forward
    // continuation and stopping horizon are certified. Do not force rest
    // merely because lattice simplification left less than 0.4 m here.
    const double speed=taskTransitTerminalSpeed(selected.goal,yaw,planner_manager_->pp_.max_vel_,
        min_forward_command_,command_stop_latency_+predictive_replan_reaction_time_,
        guaranteed_braking_deceleration_,terminal_point_safe);
    if(speed>0.) local_target_vel_.head<2>()=delta.normalized()*speed;
    task_transit_speed_limited_=speed>0. && speed+1e-6<planner_manager_->pp_.max_vel_;
  }
  task_goal_available_=true;
  return true;
}

void SCANReplanFSM::refreshDepartureRecoveryBudget() {
  if(!observation_task_ || structured_local_repair_active_.load() ||
      stopped_local_repair_pending_.load() || structured_failed_rejoin_pending_.load()) return;
  refreshPlanningOdomFromSafety();
  std::lock_guard<std::mutex> lock(departure_recovery_mutex_);
  auto& memory=departure_recovery_memory_;
  if(!memory.used) return;
  const auto exits=observationStationExitMask(
      {memory.position.x(),memory.position.y(),odom_pos_.z()},memory.yaw);
  const bool blocked=memory.blocks(odom_pos_.head<2>(),getOdomYaw(),exits);
  task_lateral_repair_used_.store(blocked);
  if(!blocked) {
    memory.used=false;
    RCLCPP_INFO(node_->get_logger(),"[DEPARTURE_RECOVERY_REOPENED] task=%llu actual=(%.3f,%.3f) yaw=%.3f; physical departure changed, no clock or goal reset",
        static_cast<unsigned long long>(task_id_.load()),odom_pos_.x(),odom_pos_.y(),getOdomYaw());
  }
}

void SCANReplanFSM::observationTaskRejected(const std::string& requested_reason) {
  if (!observation_task_ || task_terminal_request_==active_reference_request_id_.load()) return;
  task_turn_stage_active_=false;task_turn_stage_trajectory_id_=0;task_turn_stage_rest_gate_={};
  std::string reason=requested_reason;
  { std::lock_guard<std::mutex> lock(task_failure_trace_mutex_);
    task_failure_planning_=requested_reason;
    if(task_failure_trigger_.empty()) task_failure_trigger_=requested_reason; }
  const bool failed_rejoin=structured_failed_rejoin_pending_.exchange(false);
  if (failed_rejoin) {
    if (reason.rfind("LATERAL_RECOVERY_REJOIN_",0)!=0)
      reason="LATERAL_RECOVERY_REJOIN_FAILED";
    RCLCPP_WARN(node_->get_logger(),
        "[FAILED_RECOVERY_REJOIN_EXHAUSTED] request=%llu cause=%s terminal=%s; no additional escape",
        static_cast<unsigned long long>(active_reference_request_id_.load()),
        requested_reason.c_str(),reason.c_str());
  }
  if (failed_rejoin || reason.rfind("LATERAL_RECOVERY_",0)==0) {
    // Terminal lateral failures, including unconfirmed rest, release motion
    // ownership. They retain physical departure memory and consumed budget.
    structured_local_repair_active_.store(false);
    structured_local_repair_executing_.store(false);
    structured_local_repair_finished_.store(false);
    structured_local_repair_rejoin_pending_.store(false);
    stopped_local_repair_pending_.store(false);
    stopped_local_repair_active_.store(false);
  }
  // A same-stack failed search cannot retire recovery just scheduled by
  // another search. The FSM owns its stop/validation/ACK until it completes.
  if ((reason=="DEPARTURE_UNSAFE" || reason=="LOCAL_PATH_FAMILY_EXHAUSTED") &&
      (task_restart_from_rest_requested_.load() || stopped_local_repair_pending_.load())) return;
  // Empty moving-departure search has the same fallback as a rolling ACK
  // rejection. Do not terminalize G2 before solving the stopped initial state.
  if ((reason=="DEPARTURE_UNSAFE" || reason=="LOCAL_PATH_FAMILY_EXHAUSTED") &&
      requestTaskRestartFromRest(reason)) return;
  // Defer recovery to the FSM rather than recursively planning inside the
  // failed target search. The task, information demand and candidate budget
  // survive HOLD; only an exact trajectory activation ACK releases motion.
  if(!failed_rejoin) refreshDepartureRecoveryBudget();
  if (enable_holonomic_lateral_repair_ && !task_lateral_repair_used_.load() &&
      !structured_local_repair_active_.load() && reference_path_active_ && have_target_ &&
      (reason=="DEPARTURE_UNSAFE" || reason=="LOCAL_PATH_FAMILY_EXHAUSTED")) {
    task_lateral_repair_used_.store(true);
    task_lateral_repair_authorized_.store(true);
    { std::lock_guard<std::mutex> lock(departure_recovery_mutex_);
      departure_recovery_memory_={true,odom_pos_.head<2>(),getOdomYaw(),
          observationStationExitMask(odom_pos_,getOdomYaw())}; }
    stopped_local_repair_pending_.store(true);
    stopped_local_repair_active_.store(false);
    safety_stop_active_.store(true);
    safety_generation_.fetch_add(1);
    predictive_replan_requested_.store(false);
    replan_fail_count_=0;
    requestExecutionStop("TASK_LATERAL_RECOVERY_WAIT_STOP",false);
    RCLCPP_WARN(node_->get_logger(),
        "[TASK_LATERAL_RECOVERY_WAIT_STOP] task=%llu request=%llu reason=%s; retaining task and observation demand",
        static_cast<unsigned long long>(task_id_.load()),
        static_cast<unsigned long long>(active_reference_request_id_.load()),reason.c_str());
    return;
  }
  task_lateral_repair_authorized_.store(false);
  task_terminal_request_=active_reference_request_id_.load();
  const bool spatial = reason == "OBSERVATION_STATION_BUDGET_EXHAUSTED" ||
                       reason == "NO_NEW_OBSERVATION_STATION";
  const bool path_failure = reason == "LOCAL_PATH_FAMILY_EXHAUSTED" ||
                            reason == "TASK_REFERENCE_NOT_ACTIVATABLE";
  const bool departure_failure = reason == "DEPARTURE_UNSAFE" || (path_failure && (task_departure_blocked_ ||
      footprintOccupiedWithMargin(planner_manager_->grid_map_,odom_pos_,
                                  getOdomYaw(),planning_clearance_margin_)));
  const bool path_failed = path_failure && !departure_failure;
  const bool autonomy=observation_task_->local_geometry_autonomy;
  RCLCPP_WARN(node_->get_logger(),
      "[OBSERVATION_LOCAL_REJECTED] task=%llu request=%llu reason=%s",
      static_cast<unsigned long long>(task_id_.load()),
      static_cast<unsigned long long>(task_terminal_request_),
      (departure_failure ? "DEPARTURE_UNSAFE" : reason.c_str()));
  if (spatial && !autonomy)
    publishFailureEvidence(task_terminal_request_,planner_manager_->local_data_.traj_id_,
        "OBSERVATION_LOCAL_PLANNING",reason,task_station_,static_cast<uint32_t>(replan_fail_count_),
        planning_clearance_margin_,ClearanceFailure::NONE,
        scan_planner_msgs::msg::FailureEvidence::SCOPE_VIEWPOINT);
  publishObservationFeedback(autonomy
      ? scan_planner_msgs::msg::ObservationFeedback::LOCAL_MOTION_EXHAUSTED
      : spatial
      ? scan_planner_msgs::msg::ObservationFeedback::TASK_EXHAUSTED
      : (path_failed ? scan_planner_msgs::msg::ObservationFeedback::PATH_REJECTED
                     : scan_planner_msgs::msg::ObservationFeedback::EXECUTION_BLOCKED),
                     departure_failure ? "DEPARTURE_UNSAFE" : reason);
  have_target_=false; have_new_target_=false; reference_path_active_=false;
  reference_path_update_pending_=false; predictive_replan_requested_.store(false);
  stopped_local_repair_pending_.store(false); terminal_local_repair_hold_.store(true);
  requestExecutionStop(spatial ? "VIEWPOINT_LOCAL_REJECTED" :
                       (path_failed ? "LOCAL_PATH_REJECTED" : "LOCAL_EXECUTION_BLOCKED"),false);
  {std::lock_guard<std::mutex> lock(execution_snapshot_mutex_); execution_snapshot_.valid=false;}
  changeFSMExecState(WAIT_TARGET,"OBSERVATION_LOCAL_REJECTED");
}

bool SCANReplanFSM::observationStationRestartCertified(const Eigen::Vector3d& position,double yaw) const {
  const auto domain=observationTaskDomain();const auto map=planner_manager_->grid_map_;
  const double stop=min_forward_command_*(command_stop_latency_+predictive_replan_reaction_time_)+
      min_forward_command_*min_forward_command_/(2*guaranteed_braking_deceleration_);
  const Eigen::Vector2d return_direction=domain.route.empty() ? Eigen::Vector2d::Zero() :
      Eigen::Vector2d(domain.route.front()-position.head<2>());
  const double return_yaw=return_direction.norm()>.20 ?
      std::atan2(return_direction.y(),return_direction.x()) : yaw;
  return taskTerminalRestartCertified(position.head<2>(),yaw,stop,
      [&](const Eigen::Vector2d& p,double heading) {
        return taskDomainContains(domain,p) && !footprintOccupiedWithMargin(
            map,{p.x(),p.y(),position.z()},heading,planning_clearance_margin_);
      },return_yaw);
}

uint16_t SCANReplanFSM::observationStationExitMask(const Eigen::Vector3d& position,double yaw) const {
  const auto domain=observationTaskDomain();
  const auto map=planner_manager_->grid_map_;
  const double forward_stop=min_forward_command_*(command_stop_latency_+predictive_replan_reaction_time_)+
      min_forward_command_*min_forward_command_/(2*guaranteed_braking_deceleration_);
  const double lateral_stop=local_repair_lateral_speed_*(command_stop_latency_+predictive_replan_reaction_time_)+
      local_repair_lateral_speed_*local_repair_lateral_speed_/(2*guaranteed_braking_deceleration_);
  return taskStationExitMask(position.head<2>(),yaw,std::max(.30,forward_stop),
      .15+lateral_stop,enable_holonomic_lateral_repair_,
      [&](const Eigen::Vector2d& point,double heading) {
        return taskDomainContains(domain,point) && !footprintOccupiedWithMargin(
            map,{point.x(),point.y(),position.z()},heading,planning_clearance_margin_);
      });
}

LocalTaskDomain SCANReplanFSM::observationTaskDomain() const {
  LocalTaskDomain domain;
  if (!observation_task_) return domain;
  for(const auto& point:observation_task_->coarse_route) domain.route.push_back({point.x,point.y});
  domain.observation_center={observation_task_->goal.x,observation_task_->goal.y};
  domain.corridor_radius=observation_task_->corridor_radius;
  domain.observation_radius=observation_task_->observation_radius;
  domain.has_directional_gate=observation_task_->has_directional_gate &&
      !observation_task_->local_geometry_autonomy;
  domain.gate_origin={observation_task_->gate_origin.x,observation_task_->gate_origin.y};
  domain.gate_direction={observation_task_->gate_direction.x,observation_task_->gate_direction.y};
  domain.gate_min_progress=observation_task_->gate_min_progress;
  return domain;
}

}  // namespace scan_planner

namespace scan_planner {
bool SCANReplanFSM::forwardStageRestartCertified(const Eigen::Vector3d& position,double yaw,bool capture_region) const {
  const auto domain=observationTaskDomain();const auto map=planner_manager_->grid_map_;
  const double radius=capture_region ? kForwardStageCaptureRadius : 0.;
  const double margin=planning_clearance_margin_+kHandoffPositionAllowance+
      (kHandoffYawAllowance+kForwardStageYawBound+2.5*M_PI/180.)*
          (self_double_cylinder_radius_+self_double_cylinder_offset_)+radius;
  const double stop=min_forward_command_*(command_stop_latency_+predictive_replan_reaction_time_)+
      min_forward_command_*min_forward_command_/(2.*guaranteed_braking_deceleration_);
  const Eigen::Vector2d next=task_turn_stage_objective_-position.head<2>();
  if(next.norm()<.01) return false;
  return taskDepartureCertified(position.head<2>(),yaw,std::atan2(next.y(),next.x()),stop,
      [&](const Eigen::Vector2d& p,double heading) {
        for(int i=0;i<16 && radius>0.;++i) {
          const double angle=2.*M_PI*i/16.;
          if(!taskDomainContains(domain,p+radius*Eigen::Vector2d(std::cos(angle),std::sin(angle)))) return false;
        }
        return taskDomainContains(domain,p) && !footprintOccupiedWithMargin(map,
            {p.x(),p.y(),position.z()},heading,margin);
      });
}
} // namespace scan_planner
