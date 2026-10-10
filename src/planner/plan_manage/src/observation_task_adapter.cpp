#include <plan_manage/scan_replan_fsm.h>

namespace scan_planner {

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

void SCANReplanFSM::retireObservationTask(TaskVersion version) {
  if (version.first == 0 || version.second == 0) return;
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
  retiring_task_ = std::max(retiring_task_, version);
  task_retirement_pending_ = true;
  last_task_retirement_ns_ = 0;
  have_target_ = false;
  reference_path_active_ = false;
  reference_path_update_pending_ = false;
  predictive_replan_requested_.store(false);
  stopped_local_repair_pending_.store(false);
  terminal_local_repair_hold_.store(true);
  changeFSMExecState(WAIT_TARGET, "TASK_RETIRING");
}

void SCANReplanFSM::confirmObservationTaskRetirement(TaskVersion version) {
  if (!task_retirement_pending_ || version != retiring_task_) return;
  retired_task_ = std::max(retired_task_, version);
  task_retirement_pending_ = false;
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
  // Cancellation may name a queued successor, not observation_task_.
  // Confirm the cancelled identity so Explorer can finish its transaction.
  scan_planner_msgs::msg::ObservationFeedback feedback;
  feedback.header.stamp = node_->now();
  feedback.header.frame_id = self_inflation_frame_id_;
  feedback.task_id = version.first; feedback.task_version = version.second;
  feedback.state = scan_planner_msgs::msg::ObservationFeedback::CANCEL_ACK;
  feedback.reason = "CONTROLLER_TASK_RETIRED";
  task_feedback_pub_->publish(feedback);
  observation_task_.reset();
  task_id_.store(0); task_version_.store(0);
}

void SCANReplanFSM::observationTaskCallback(
    const scan_planner_msgs::msg::ObservationTask::ConstSharedPtr& msg) {
  if (!task_mode_ || !msg || !msg->task_id || !msg->task_version) return;
  const TaskVersion incoming{msg->task_id, msg->task_version};
  if (incoming <= retired_task_) return;
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
  if (current.first != 0 && incoming != current && !msg->predecessor_task_id)
    retireObservationTask(current);
}

bool SCANReplanFSM::processObservationTaskTransitions() {
  if (!task_mode_) return false;
  TaskVersion confirmation;
  std::vector<std::pair<TaskVersion,Eigen::Vector2d>> failed;
  {
    std::lock_guard<std::mutex> lock(task_event_mutex_);
    confirmation=task_retirement_confirmation_;
    task_retirement_confirmation_={0,0};
    failed.swap(task_failed_goals_);
  }
  if(confirmation.first) confirmObservationTaskRetirement(confirmation);
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
  if (deferred_task_) {
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
    task_rejected_stations_.clear(); // Candidate exclusions belong to one request.
    task_rejected_departures_.clear();
    if (!same_task) {
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
    task_station_target_ = false; task_goal_available_ = false;
    task_station_={msg->goal.x,msg->goal.y,odom_pos_.z()};
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
  if (observation_task_ && task_terminal_request_ == active_reference_request_id_.load()) return true;
  if (observation_task_ && task_station_target_ && have_target_ &&
      taskStationCaptureReady(observation_task_->local_geometry_autonomy,
          go2_heading_aligning_.load(),(odom_pos_-task_station_).head<2>().norm())) {
    bool acknowledged=false;
    { std::lock_guard<std::mutex> lock(execution_snapshot_mutex_);
      acknowledged = execution_snapshot_.valid && execution_snapshot_.task_id == task_id_.load() &&
          execution_snapshot_.task_version == task_version_.load() &&
          execution_snapshot_.request_id == active_reference_request_id_.load() &&
          execution_snapshot_.candidate_id == task_candidate_id_.load(); }
    if (acknowledged) { finishGoalReached("OBSERVATION_STATION"); return true; }
  }
  return false;
}

bool SCANReplanFSM::selectObservationTaskTarget() {
    const bool autonomy=observation_task_ && observation_task_->local_geometry_autonomy;
    bool retain_goal = task_goal_available_ &&
        (task_station_.head<2>()-start_pt_.head<2>()).norm() > (autonomy ? 0.20 : 0.6);
    const Eigen::Vector2d previous_goal = task_station_.head<2>();
    task_goal_available_ = false;
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
  // Refit the approach while retaining a still-valid local objective. A
  // new map/odom sample alone is not a reason to choose another station.
  Eigen::Vector2d preferred=retain_goal ? previous_goal :
      (sample.valid ? Eigen::Vector2d(sample.target.head<2>()) : domain.observation_center);
  if(autonomy && !retain_goal && task_route_cursor_<task_route_portals_.size()) {
    const auto& portal=task_route_portals_[task_route_cursor_];
    const Eigen::Vector2d checkpoint=portal.center+0.3*portal.direction.normalized();
    if((checkpoint-start_pt_.head<2>()).norm()<=reference_path_lookahead_)
      preferred=checkpoint;
  }
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
          [&](const Eigen::Vector2d& p,double heading) {
            return taskDomainContains(domain,p) && !footprintOccupiedWithMargin(
                map,{p.x(),p.y(),start_pt_.z()},heading,planning_clearance_margin_);
          })) return false;
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
      if (footprintOccupiedWithMargin(map,{p.x(),p.y(),start_pt_.z()},yaw,planning_clearance_margin_)) return false;
    }
    return true;
  };
  const auto sees=[&](const Eigen::Vector2d& p,const Eigen::Vector2d& endpoint) {
      if ((endpoint-p).norm()>observation_task_->observation_radius) return false;
      const int samples=std::max(1,static_cast<int>(std::ceil((endpoint-p).norm()/0.1)));
      bool clear=true;
      // The target boundary itself may be unknown/occupied. Do not look
      // through an occupied inflated cell before the final boundary band.
      for(int i=0;i<samples-3;++i) {
        const Eigen::Vector2d q=p+(endpoint-p)*(static_cast<double>(i)/samples);
        if(map->getInflateOccupancy({q.x(),q.y(),start_pt_.z()},getOdomYaw())!=0) {clear=false;break;}
      }
      return clear;
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
  auto candidates=selectLocalTaskCandidates(domain,start_pt_.head<2>(),preferred,
      edge_safe,visible,0.2,std::min(4.0,std::max(1.0,reference_path_lookahead_)),1024,8,goal_allowed,
      getOdomYaw(),cost,autonomy && retain_goal ? &previous_goal : nullptr);
  if (candidates.empty() && autonomy && retain_goal) {
    // A target changes only if its current approach is genuinely infeasible,
    // never just because the moving search lattice ranks a neighbour higher.
    retain_goal=false;
    candidates=selectLocalTaskCandidates(domain,start_pt_.head<2>(),preferred,
        edge_safe,visible,0.2,std::min(4.0,std::max(1.0,reference_path_lookahead_)),1024,8,
        goal_allowed,getOdomYaw(),cost);
  }
  RCLCPP_INFO(node_->get_logger(),
      "[TASK_DEPARTURE_SEARCH] request=%llu heading=%.3f checked=%zu certified=%zu excluded=%zu candidates=%zu retained_goal=%d",
      static_cast<unsigned long long>(active_reference_request_id_.load()),getOdomYaw(),
      departure_checks,departure_certificates,task_rejected_departures_.size(),candidates.size(),retain_goal);
  if(candidates.empty()) {
    if (!certified_departure_available ||
        footprintOccupiedWithMargin(map,start_pt_,getOdomYaw(),planning_clearance_margin_))
      observationTaskRejected("DEPARTURE_UNSAFE");
    else
      // An empty motion/visibility search does not prove the remote viewpoint
      // is exhausted. Only the explicit station budget may terminate it.
      observationTaskRejected("LOCAL_PATH_FAMILY_EXHAUSTED");
    return false;
  }
  const auto& selected=candidates.front();
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
  local_target_vel_.setZero();
  if(!task_station_target_ && delta.norm()>0.4)
    local_target_vel_.head<2>()=delta.normalized()*planner_manager_->pp_.max_vel_;
  task_goal_available_=true;
  return true;
}

void SCANReplanFSM::observationTaskRejected(const std::string& reason) {
  if (!observation_task_ || task_terminal_request_==active_reference_request_id_.load()) return;
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
