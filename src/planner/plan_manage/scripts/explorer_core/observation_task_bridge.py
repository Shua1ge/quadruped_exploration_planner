"""ROS task bridge; Explorer retains objective and region ownership."""
import math
from geometry_msgs.msg import Point
from scan_planner_msgs.msg import ObservationTask as TaskMessage, ObservationFeedback
from std_msgs.msg import String
from .observation_contract import bind_task_identity, representative_cells


class ObservationTaskBridge:
    @property
    def active_observation(self):
        return getattr(self, "_active_observation", None)

    @active_observation.setter
    def active_observation(self, value):
        old = getattr(self, "_active_observation", None)
        if old is not None and old is not value and getattr(self, "task_mode", False):
            lifecycle = (TaskMessage.SUPERSEDED if value is not None
                         else TaskMessage.CANCELLED)
            self.retire_observation_task(old, lifecycle)
        self._active_observation = value
        if old is not value:
            self.active_task_elapsed = 0.0
            self.blocked_execution_context = None
            revoke = getattr(self, "revoke_task_path_constraints", None)
            if revoke is not None:
                revoke(old)
            if old is not None:
                self.pending_observation_station = None
                self.pending_path_publish_ns = 0
                self.pending_path_request_generation = None
                self.pending_path_deferred = False
                self.active_path_request_generation = None
                self.active_path_trajectory_id = None

    def cancel_observation_task(self, task):
        return self.retire_observation_task(task, TaskMessage.CANCELLED)

    def retire_observation_task(self, task, lifecycle):
        """Publish one terminal lifecycle event for an objective identity."""
        if (not getattr(self, "task_mode", False) or not task.task_id
                or getattr(task, "lifecycle", "ACTIVE") in
                ("CANCELLED", "SUPERSEDED", "COMPLETED")):
            return False
        message = TaskMessage()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.task_id, message.task_version = task.task_id, task.task_version
        message.lifecycle = lifecycle
        task.lifecycle = ("SUPERSEDED" if lifecycle == TaskMessage.SUPERSEDED
                          else "CANCELLED")
        self.observation_task_pub.publish(message)
        self.task_cancellations += 1
        return True

    def publish_observation_task(self, path, request_id):
        task = self.active_observation
        if task is None:
            raise RuntimeError("task mode requires an observation objective")
        if not task.task_id:
            generation = max(request_id, self.last_observation_task_id + 1)
            bind_task_identity(task, generation, self.grid)
            self.last_observation_task_id = generation
            self.tasks_created += 1
        message = TaskMessage()
        message.header = path.header
        message.task_id, message.task_version = task.task_id, task.task_version
        message.request_id, message.lifecycle = request_id, TaskMessage.ACTIVE
        message.has_region, message.region_id = task.region_id is not None, int(task.region_id or 0)
        message.goal = Point(x=float(task.goal[0]), y=float(task.goal[1]), z=0.0)
        message.coarse_route = [pose.pose.position for pose in path.poses]
        message.corridor_radius, message.observation_radius = self.task_corridor_radius, self.observation_radius
        remaining = {cell for cell in task.target_cells
                     if not self.grid.in_bounds(cell) or
                     self.grid.sensor_observed[cell[1], cell[0]] <= task.initial_sensor_revision}
        message.target_cells = [Point(x=float(x), y=float(y), z=0.0) for x, y in
            (self.grid.cell_to_world(cell) for cell in representative_cells(remaining or task.target_cells, self.task_representative_limit))]
        message.target_cell_count, message.resolution = len(task.target_cells), self.grid.resolution
        message.representative_selection_version = 1
        message.terminal_cells = [Point(x=float(x), y=float(y), z=0.0) for x, y in
            (self.grid.cell_to_world(cell) for cell in task.terminal_cells)]
        message.max_station_adjustments = self.task_max_station_adjustments
        origin = getattr(self, "commitment_direction_origin", None)
        direction = getattr(self, "commitment_direction_vector", None)
        if origin is not None and direction is not None:
            length = math.hypot(*direction)
            if length > 1e-6:
                message.has_directional_gate = True
                message.gate_origin = Point(x=float(origin[0]),y=float(origin[1]),z=0.0)
                message.gate_direction.x, message.gate_direction.y = direction[0]/length, direction[1]/length
                tolerance = (self.region_recovery_backtrack_tolerance if self.directional_recovery_pending
                             else self.region_backtrack_tolerance)
                message.gate_min_progress = self.commitment_direction_progress-tolerance
        self.observation_task_pub.publish(message)
        self.task_requests_published += 1
        self.pending_observation_station = None
        self.blocked_execution_context = None

    def observation_feedback_callback(self, message):
        if not self.task_mode:
            return
        task = self.active_observation
        if (task is None or message.task_id != task.task_id or message.task_version != task.task_version
                or message.request_id != self.active_path_request_generation):
            self.task_stale_feedback += 1
            return
        if not all(math.isfinite(v) for v in (message.station.x, message.station.y, message.station.z)):
            return
        key = (message.task_id, message.task_version, message.request_id, message.candidate_id, message.state)
        if key in self.task_feedback_seen:
            return
        self.task_feedback_seen.append(key)
        if message.state == ObservationFeedback.STATION_REACHED:
            task.station_adjustments = int(message.station_adjustments)
            self.task_stations_reached += 1
            self.pending_path_publish_ns = 0
            self.pending_path_request_generation = None
            self.pending_path_deferred = False
            self.scan_waiting_for_target = True
            completed = self.evaluate_active_observation()
            if self.observation_complete(completed):
                self.pending_observation_station = None
                self.handoff_satisfied_observation("OBSERVATION_TASK_COMPLETE_AT_STATION")
                return
            self.pending_observation_station = (self.map_update_count, self.get_clock().now().nanoseconds, message)
        elif message.state == ObservationFeedback.EXECUTION_BLOCKED:
            self.pending_observation_station = None
            self.pending_path_publish_ns = 0
            self.pending_path_request_generation = None
            self.pending_path_deferred = False
            self.blocked_execution_context = self.local_execution_context()
            self.publish_status("LOCAL_EXECUTION_BLOCKED")
        elif message.state == ObservationFeedback.PATH_REJECTED:
            self.pending_observation_station = None
            self.pending_path_publish_ns = 0
            self.pending_path_request_generation = None
            self.pending_path_deferred = False
            self.blocked_execution_context = None
            # No second progress ledger: retain task/coverage and try a route
            # avoiding its typed path evidence. Never repeat an identical route.
            failed_route = tuple(self.active_raw_path)
            if self.reroute_active_goal(excluded_route=failed_route):
                self.publish_status("LOCAL_PATH_REPAIRED")
                return
            status = String()
            status.data = (f"VIEWPOINT_LOCAL_REJECTED request_id={message.request_id} "
                           f"task_id={message.task_id} task_version={message.task_version}")
            self.planning_status_callback(status)
        elif message.state in (ObservationFeedback.VIEWPOINT_REJECTED, ObservationFeedback.TASK_EXHAUSTED):
            self.task_local_rejections += 1
            self.pending_observation_station = None
            if message.state == ObservationFeedback.VIEWPOINT_REJECTED:
                self.apply_global_viewpoint_constraint(message.request_id, (message.station.x, message.station.y),
                                                      self.blacklist_radius, message.reason)
            # Existing arbitration first tries siblings in the committed scope.
            status = String()
            status.data = (f"VIEWPOINT_LOCAL_REJECTED request_id={message.request_id} "
                           f"task_id={message.task_id} task_version={message.task_version}")
            self.planning_status_callback(status)

    def continue_observation_station(self):
        if self.pending_observation_station is None:
            return False
        revision, stamp, message = self.pending_observation_station
        task = self.evaluate_active_observation()
        if self.observation_complete(task):
            self.pending_observation_station = None
            self.handoff_satisfied_observation("OBSERVATION_TASK_COMPLETE_AT_STATION")
            return True
        # Await accepted post-arrival scans, with a bounded wait if clouds stop.
        if (self.map_update_count-revision < self.observation_done_updates
                and self.get_clock().now().nanoseconds-stamp < 2_000_000_000):
            return True
        self.pending_observation_station = None
        if not self.reroute_active_goal():
            self.publish_reference_path(self.active_raw_path)
        return True

    def local_execution_context(self):
        """Only real departure-state changes authorize retry, not map revision."""
        x, y = self.grid.world_to_cell(*self.position)
        radius = max(1, int(math.ceil(1.0/self.grid.resolution)))
        cells = tuple(self.grid.value((i,j)) if self.grid.in_bounds((i,j)) else -1
                      for j in range(y-radius,y+radius+1)
                      for i in range(x-radius,x+radius+1))
        return (x, y, round(getattr(self, "body_yaw", 0.0)/0.2), cells)

    def continue_blocked_execution(self):
        context = getattr(self, "blocked_execution_context", None)
        if context is None:
            return False
        if self.local_execution_context() != context:
            self.blocked_execution_context = None
            self.healthy_task_elapsed = 0.0
            self.publish_reference_path(self.active_raw_path)
            self.publish_status("LOCAL_EXECUTION_RETRY_CHANGED_STATE")
        # Holding is a durable explicit state, not an exploration timeout and
        # not permission to blacklist successive distant viewpoints.
        return True
