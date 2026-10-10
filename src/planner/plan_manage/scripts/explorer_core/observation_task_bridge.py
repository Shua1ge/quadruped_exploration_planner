"""ROS task bridge; Explorer retains objective and region ownership."""
import math
from dataclasses import dataclass
from geometry_msgs.msg import Point
from scan_planner_msgs.msg import ObservationTask as TaskMessage, ObservationFeedback, RoutePortal
from std_msgs.msg import String
from .observation_contract import bind_task_identity, representative_cells
from .grid import FREE, OCCUPIED


@dataclass
class PendingObservationHandoff:
    predecessor: object
    task: object
    path: object
    candidate: object
    request_id: int = 0
    published_ns: int = 0
    state: str = "PREPARING"
    cancel_publish_ns: int = 0


@dataclass(frozen=True)
class LocalMotionAttempt:
    position: tuple
    reason: str
    blocked_cells: frozenset
    departure_yaw: float = math.nan


class ObservationTaskBridge:
    @property
    def active_observation(self):
        return getattr(self, "_active_observation", None)

    @active_observation.setter
    def active_observation(self, value):
        old = getattr(self, "_active_observation", None)
        if old is not value and getattr(self, "pending_observation_handoff", None) is not None \
                and not getattr(self, "completion_handoff_in_progress", False):
            self.cancel_pending_observation_handoff("ACTIVE_TASK_CANCELLED")
        if old is not None and old is not value and getattr(self, "task_mode", False):
            if (value is not None and old.task_id and
                    getattr(self, "completion_handoff_in_progress", False)):
                if not value.task_id:
                    value.predecessor_task_id = old.task_id
                    value.predecessor_task_version = old.task_version
                # Information completion does not revoke safe motion. Local
                # fences the predecessor only after the replacement ACK.
                old.lifecycle = ("SUPERSEDED" if old.motion_released and
                                 not old.information_satisfied else "COMPLETED")
            else:
                lifecycle = (TaskMessage.SUPERSEDED if value is not None
                             else TaskMessage.CANCELLED)
                self.retire_observation_task(old, lifecycle)
        self._active_observation = value
        if old is not value:
            self.active_task_elapsed = 0.0
            self.blocked_execution_context = None
            self.departure_recovery_key = None
            self.departure_recovery_attempts = 0
            self.departure_recovery_terminal = False
            self.departure_last_attempt_ns = None
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

    def retire_observation_task(self, task, lifecycle, repeat=False):
        """Publish one terminal lifecycle event for an objective identity."""
        if (not getattr(self, "task_mode", False) or not task.task_id
                or (not repeat and getattr(task, "lifecycle", "ACTIVE") in
                ("CANCELLED", "SUPERSEDED", "COMPLETED"))):
            return False
        message = TaskMessage()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.task_id, message.task_version = task.task_id, task.task_version
        message.lifecycle = lifecycle
        task.lifecycle = ("SUPERSEDED" if lifecycle == TaskMessage.SUPERSEDED
                          else "CANCELLED")
        self.observation_task_pub.publish(message)
        if not repeat:
            self.task_cancellations += 1
        return True

    def publish_observation_task(self, path, request_id, task=None):
        task = self.active_observation if task is None else task
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
        message.predecessor_task_id = task.predecessor_task_id
        message.predecessor_task_version = task.predecessor_task_version
        message.has_region, message.region_id = task.region_id is not None, int(task.region_id or 0)
        message.goal = Point(x=float(task.goal[0]), y=float(task.goal[1]), z=0.0)
        message.coarse_route = [pose.pose.position for pose in path.poses]
        message.local_geometry_autonomy = getattr(self,"task_geometry_autonomy",False)
        if message.local_geometry_autonomy:
            if not task.route_contract_frozen:
                task.route_portals,task.route_graph_revision=self.build_route_portals(task,path)
                task.route_contract_frozen=True
            message.route_portals=list(task.route_portals)
            message.route_graph_revision=task.route_graph_revision
        message.corridor_radius, message.observation_radius = self.task_corridor_radius, self.observation_radius
        remaining = {cell for cell in task.target_cells
                     if not self.grid.in_bounds(cell) or
                     self.grid.sensor_observed[cell[1], cell[0]] <= task.initial_sensor_revision}
        message.target_cells = [Point(x=float(x), y=float(y), z=0.0) for x, y in
            (self.grid.cell_to_world(cell) for cell in representative_cells(remaining, self.task_representative_limit))]
        message.target_cell_count, message.resolution = len(task.target_cells), self.grid.resolution
        message.representative_selection_version = 1
        message.terminal_cells = [Point(x=float(x), y=float(y), z=0.0) for x, y in
            (self.grid.cell_to_world(cell) for cell in task.terminal_cells)]
        message.max_station_adjustments = self.task_max_station_adjustments
        origin = getattr(self, "commitment_direction_origin", None)
        direction = getattr(self, "commitment_direction_vector", None)
        preparing = task is not self.active_observation
        if preparing:
            origin = self.position
            target = next((pose.pose.position for pose in path.poses[1:]
                if math.hypot(pose.pose.position.x-origin[0],pose.pose.position.y-origin[1])
                   >= self.region_direction_lookahead),path.poses[-1].pose.position)
            direction = (target.x-origin[0],target.y-origin[1])
        if not message.local_geometry_autonomy and origin is not None and direction is not None:
            length = math.hypot(*direction)
            if length > 1e-6:
                message.has_directional_gate = True
                message.gate_origin = Point(x=float(origin[0]),y=float(origin[1]),z=0.0)
                message.gate_direction.x, message.gate_direction.y = direction[0]/length, direction[1]/length
                tolerance = (self.region_recovery_backtrack_tolerance if self.directional_recovery_pending
                             else self.region_backtrack_tolerance)
                message.gate_min_progress = (0.0 if preparing else self.commitment_direction_progress)-tolerance
        self.observation_task_pub.publish(message)
        self.task_requests_published += 1
        if not preparing:
            self.pending_observation_station = None
            self.blocked_execution_context = None

    def cancel_pending_observation_handoff(self, reason):
        pending = getattr(self,"pending_observation_handoff",None)
        if pending is None or pending.state == "CANCELLING":
            return
        pending.state = "CANCELLING"
        pending.cancel_publish_ns = self.get_clock().now().nanoseconds
        if getattr(self, "task_geometry_autonomy", False):
            self.remember_local_motion_attempt(pending.task.goal, reason)
        self.retire_observation_task(pending.task,TaskMessage.CANCELLED)
        # Failed connection is not permanent viewpoint/region evidence.
        excluded = list(getattr(self,"handoff_excluded_goals",()))
        if pending.task.goal not in excluded:
            excluded.append(pending.task.goal)
        self.handoff_excluded_goals = excluded[-16:]
        self.publish_status("OBSERVATION_HANDOFF_CANCELLING reason="+reason)

    def continue_pending_observation_handoff(self):
        pending = getattr(self,"pending_observation_handoff",None)
        if pending is None:
            return False
        now = self.get_clock().now().nanoseconds
        limit = int(self.path_request_timeout*1e9)
        if pending.state == "PREPARING" and now-pending.published_ns >= limit:
            self.cancel_pending_observation_handoff("EXECUTION_READY_TIMEOUT")
        elif pending.state == "CANCELLING" and now-pending.cancel_publish_ns >= limit:
            self.retire_observation_task(pending.task,TaskMessage.CANCELLED,repeat=True)
            pending.cancel_publish_ns = now
        self.publish_status("OBSERVATION_HANDOFF_"+pending.state)
        return True

    def pending_observation_feedback(self, message):
        pending = getattr(self,"pending_observation_handoff",None)
        if pending is None or (message.task_id,message.task_version) != \
                (pending.task.task_id,pending.task.task_version):
            return False
        if message.state == ObservationFeedback.CANCEL_ACK:
            if pending.state == "CANCELLING":
                self.pending_observation_handoff = None
                # Both predecessor and successor are fenced after cancel.
                # G1 remains the satisfied information ledger, not a resumable
                # execution. A later replacement starts without a predecessor.
                self.handoff_execution_retired = True
                self.active_path_request_generation = None
                self.active_path_trajectory_id = None
                self.pending_path_publish_ns = 0
                self.pending_path_request_generation = None
                self.pending_observation_station = None
                self.scan_waiting_for_target = True
                self.last_handoff_attempt_update = -1
                self.publish_status("OBSERVATION_HANDOFF_CANCELLED_WAITING_FOR_REPLACEMENT")
            return True
        if message.request_id != pending.request_id or pending.state != "PREPARING":
            self.task_stale_feedback += 1
            return True
        if message.state == ObservationFeedback.EXECUTION_READY:
            if message.trajectory_id <= 0 or not message.candidate_id:
                self.task_stale_feedback += 1
                return True
            if self.active_observation is not pending.predecessor:
                self.cancel_pending_observation_handoff("PREDECESSOR_CHANGED")
            else:
                self.commit_observation_handoff(pending,message)
        elif message.state in (ObservationFeedback.PATH_REJECTED,
                ObservationFeedback.VIEWPOINT_REJECTED,ObservationFeedback.TASK_EXHAUSTED,
                ObservationFeedback.EXECUTION_BLOCKED,ObservationFeedback.LOCAL_MOTION_EXHAUSTED):
            self.cancel_pending_observation_handoff(message.reason)
        return True

    def pending_observation_status(self, status, request_id, fields):
        pending = getattr(self,"pending_observation_handoff",None)
        if pending is None or fields.get("task_id") != str(pending.task.task_id) \
                or fields.get("task_version") != str(pending.task.task_version):
            return False
        if request_id != pending.request_id:
            self.task_stale_feedback += 1
            return True
        if pending.state == "PREPARING" and status in (
                "REFERENCE_PATH_REJECTED","INVALID_REFERENCE_PATH","BLOCKED",
                "TRACKING_RECOVERY_FAILED","LOCAL_REPAIR_EXHAUSTED","VIEWPOINT_LOCAL_REJECTED"):
            self.cancel_pending_observation_handoff(fields.get("reason",status))
        # PATH_ACCEPTED/PATH_TRAJECTORY_READY are not the typed commit event.
        return True

    def observation_feedback_callback(self, message):
        if not self.task_mode:
            return
        if self.pending_observation_feedback(message):
            return
        task = self.active_observation
        if task is not None and (task.information_satisfied or task.motion_released) and \
                (message.task_id,message.task_version)==(task.task_id,task.task_version):
            # G1 arrival/stall is no longer a reason to reopen its information
            # objective or issue another G1 route while G2 is being prepared.
            return
        if (task is None or message.task_id != task.task_id or message.task_version != task.task_version
                or message.request_id != self.active_path_request_generation
                or getattr(self, "departure_recovery_terminal", False)):
            self.task_stale_feedback += 1
            return
        if message.state == ObservationFeedback.EXECUTION_READY:
            # A replay of the committed handoff is not station/progress feedback.
            return
        if not all(math.isfinite(v) for v in (message.station.x, message.station.y, message.station.z)):
            return
        key = (message.task_id, message.task_version, message.request_id, message.candidate_id, message.state)
        if key in self.task_feedback_seen:
            return
        self.task_feedback_seen.append(key)
        if (getattr(self,"task_geometry_autonomy",False) and message.state in
                (ObservationFeedback.LOCAL_MOTION_EXHAUSTED,ObservationFeedback.PATH_REJECTED,
                 ObservationFeedback.EXECUTION_BLOCKED,ObservationFeedback.TASK_EXHAUSTED)):
            self.handle_local_motion_exhausted(message)
            return
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
            self.pending_observation_station = (self.grid.sensor_revision, self.get_clock().now().nanoseconds, message)
        elif message.state == ObservationFeedback.EXECUTION_BLOCKED:
            self.pending_observation_station = None
            self.pending_path_publish_ns = 0
            self.pending_path_request_generation = None
            self.pending_path_deferred = False
            self.latch_blocked_execution(message.reason)
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
            self.ensure_departure_recovery_budget()
            if not self.departure_retry_ready():
                self.latch_blocked_execution("PATH_WAITING_FOR_SETTLING")
                self.publish_status("LOCAL_PATH_WAITING_FOR_SETTLING")
                return
            if (self.departure_recovery_attempts < getattr(self, "task_departure_retry_limit", 2)
                    and self.reroute_active_goal(excluded_route=failed_route)):
                self.departure_recovery_attempts += 1
                self.departure_last_attempt_ns = self.get_clock().now().nanoseconds
                self.publish_status("LOCAL_PATH_REPAIRED")
                return
            # Failure to synthesize a spline is not evidence against its
            # remote viewpoint. Only a validated, task-scoped failed segment
            # away from the departure cell can authorize sibling arbitration.
            departure = self.grid.world_to_cell(*self.position)
            edges = getattr(self, "task_path_constraints", {}).get(
                (int(task.task_id), int(task.task_version)), set())
            has_route_evidence = any(departure not in edge for edge in edges)
            if not has_route_evidence:
                self.latch_blocked_execution("UNRESOLVED_PATH")
                self.publish_status("LOCAL_PATH_UNRESOLVED_NO_SPATIAL_EVIDENCE")
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
        # Completion requires several observations. One new scan is not an
        # instruction to move again; allow the same confirmation window here.
        required = (max(1, getattr(self, "observation_done_updates", 3))
                    if getattr(self, "task_geometry_autonomy", False) else 1)
        if (self.grid.sensor_revision-revision < required
                and self.get_clock().now().nanoseconds-stamp < 2_000_000_000):
            return True
        self.pending_observation_station = None
        if getattr(self, "task_geometry_autonomy", False):
            # The station was reached and held by Local. Release motion once,
            # retain real information evidence, and prepare an ACKed successor.
            task.motion_released = True
            self.observation_motion_released_count += 1
            self.last_observation_released_reason = "STATION_CAPTURE_INCOMPLETE"
            self.last_observation_released_progress = task.progress
            self.remember_local_motion_attempt(task.goal, "STATION_CAPTURE_INCOMPLETE")
            self.handoff_satisfied_observation("OBSERVATION_STATION_MOTION_RELEASED")
            return True
        if not self.reroute_active_goal():
            self.publish_reference_path(self.active_raw_path)
        return True

    def remember_local_motion_attempt(self, goal, reason):
        """Keep a motion attempt out of arbitration until relevant conditions change."""
        if not hasattr(self, "local_motion_goal_cooldowns"):
            self.local_motion_goal_cooldowns = {}
        x, y = self.grid.world_to_cell(*self.position)
        radius = max(1, math.ceil(1.0/self.grid.resolution))
        blocked = frozenset((i, j) for j in range(y-radius, y+radius+1)
                            for i in range(x-radius, x+radius+1)
                            if self.grid.in_bounds((i, j)) and self.grid.value((i, j)) == OCCUPIED)
        self.local_motion_goal_cooldowns[goal] = LocalMotionAttempt(
            tuple(self.position), reason, blocked, getattr(self,"body_yaw",math.nan))

    def local_motion_retry_allowed(self, attempt):
        # Heading-bin jitter and unrelated map updates cannot reopen an
        # exhausted approach. A different departure requires leaving its area.
        if math.dist(self.position, attempt.position) >= max(
                1.0, 2.0*getattr(self, "task_corridor_radius", .8)):
            return True
        # A substantial externally measured reorientation is a new departure,
        # not a new task or a map revision. Local must certify it again.
        yaw=getattr(self,"body_yaw",math.nan)
        if (attempt.reason == "DEPARTURE_UNSAFE" and math.isfinite(yaw)
                and math.isfinite(attempt.departure_yaw)
                and abs(math.remainder(yaw-attempt.departure_yaw,2*math.pi))>=math.pi/4):
            return True
        # Only a departure-obstacle failure may be reconsidered on obstacle
        # removal. Other causes require a different physical departure.
        return (attempt.reason == "DEPARTURE_UNSAFE" and bool(attempt.blocked_cells)
                and any(self.grid.in_bounds(cell) and self.grid.value(cell) == FREE
                        for cell in attempt.blocked_cells))

    def handle_local_motion_exhausted(self, message):
        """Motion terminal, not geometric evidence or a second Local retry loop."""
        previous_goal=self.active_goal
        scope=getattr(self,"active_commitment_scope",None)
        if scope is None and self.active_region_id is not None:
            scope=("region",self.active_region_id)
        self.task_local_rejections+=1
        # Operational cooldown prevents immediate reissuing of an exhausted
        # motion problem. Do NOT insert an edge/viewpoint/region constraint or
        # mark information complete. CANCEL fences any late candidate/ACK.
        if previous_goal is not None:
            self.remember_local_motion_attempt(previous_goal, message.reason)
        self.terminate_active_observation("LOCAL_MOTION_EXHAUSTED")
        self.replan_gate.reset() # New task arbitration input, NOT new map/geometry evidence.
        excluded=() if previous_goal is None else (previous_goal,)
        if self.plan_from_current_position(excluded_goals=excluded,required_scope=scope,
                                           allow_region_release=False):
            self.publish_status("LOCAL_MOTION_EXHAUSTED_SIBLING_TASK_SELECTED")
        else:
            # Release commitment for task arbitration, not as a region failure.
            # No region/edge ban and no fabricated completed observation.
            self.terminate_region_option("local_motion_no_sibling_task")
            if self.plan_from_current_position(excluded_goals=excluded):
                self.publish_status("LOCAL_MOTION_EXHAUSTED_OTHER_TASK_SELECTED")
            else:
                self.publish_status("LOCAL_MOTION_EXHAUSTED_WAITING_FOR_ALTERNATIVE")

    def build_route_portals(self, task, path):
        """Freeze checkpoints from selected directed edges; never infer IDs by proximity."""
        estimate=task.route_estimate
        graph=getattr(self,"sparse_router",None)
        if (estimate is None or graph is None or not estimate.connections or estimate.graph_revision<=0 or
                estimate.graph_revision!=graph.graph_revision):
            return (),0
        points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
        def distance(point,a,b):
            dx,dy=b[0]-a[0],b[1]-a[1]
            t=max(0,min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy))) if dx*dx+dy*dy else 0
            return math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy)
        result=[]
        for edge_id,source,target in estimate.connections:
            edge=graph.edges.get(edge_id)
            if edge is None or {edge.source_id,edge.target_id}!={source,target}:
                return (),0
            polyline=graph._oriented_edge_polyline(edge_id,source,target)
            # A checkpoint on the final segment before this edge's exit.
            if len(polyline)<2:
                return (),0
            a,b=polyline[-2:]
            norm=math.hypot(b[0]-a[0],b[1]-a[1])
            if norm<1e-6:
                return (),0
            center=((a[0]+b[0])/2,(a[1]+b[1])/2)
            if not result and ((points[0][0]-center[0])*(b[0]-a[0])+
                               (points[0][1]-center[1])*(b[1]-a[1]))>=0:
                return (),0 # Robot already beyond the first checkpoint: do not backtrack to it.
            if not points or min((distance(center,u,v) for u,v in zip(points,points[1:])),default=math.inf)>self.task_corridor_radius:
                return (),0 # Dense/spliced geometry no longer represents these edges.
            portal=RoutePortal(connection_id=edge_id,source_node_id=source,target_node_id=target,
                               center=Point(x=center[0],y=center[1],z=0.0),radius=self.task_corridor_radius)
            portal.direction.x,portal.direction.y=(b[0]-a[0])/norm,(b[1]-a[1])/norm
            result.append(portal)
        return tuple(result),estimate.graph_revision

    def local_execution_context(self):
        """Only real departure-state changes authorize retry, not map revision."""
        x, y = self.grid.world_to_cell(*self.position)
        radius = max(1, int(math.ceil(1.0/self.grid.resolution)))
        cells = tuple(self.grid.value((i,j)) if self.grid.in_bounds((i,j)) else -1
                      for j in range(y-radius,y+radius+1)
                      for i in range(x-radius,x+radius+1))
        return (x, y, round(getattr(self, "body_yaw", 0.0)/0.2), cells)

    def ensure_departure_recovery_budget(self):
        task = self.active_observation
        key = (task.task_id, task.task_version) if task is not None else None
        if getattr(self, "departure_recovery_key", None) != key:
            self.departure_recovery_key = key
            self.departure_recovery_attempts = 0
            self.departure_recovery_terminal = False
            self.departure_last_attempt_ns = None

    def record_departure_motion(self, linear_speed, angular_speed):
        """Measured odometry, not map/yaw revision, authorizes rest retries."""
        now = self.get_clock().now().nanoseconds
        previous = getattr(self, "departure_odom_ns", None)
        since = getattr(self, "departure_rest_since_ns", None)
        continuous = previous is not None and 0 <= now - previous <= 500_000_000
        resting = (math.isfinite(linear_speed) and math.isfinite(angular_speed)
                   and linear_speed <= 0.05 and abs(angular_speed) <= 0.10)
        self.departure_odom_ns = now
        self.departure_rest_since_ns = (since if continuous and since is not None
                                       else now) if resting else None

    def departure_is_settled(self):
        now = self.get_clock().now().nanoseconds
        stamp = getattr(self, "departure_odom_ns", None)
        since = getattr(self, "departure_rest_since_ns", None)
        return (stamp is not None and since is not None
                and 0 <= now - stamp <= 500_000_000
                and now - since >= 500_000_000)

    def departure_retry_ready(self):
        now = self.get_clock().now().nanoseconds
        last = getattr(self, "departure_last_attempt_ns", None)
        return self.departure_is_settled() and (last is None or
            now - last >= int(getattr(self, "task_departure_retry_period", 2.0) * 1e9))

    def latch_blocked_execution(self, reason):
        """Task-scoped retry ownership; request/ACK refreshes cannot refill it."""
        self.ensure_departure_recovery_budget()
        if getattr(self, "blocked_execution_context", None) is None:
            self.departure_recovery_started_ns = self.get_clock().now().nanoseconds
        self.blocked_execution_context = self.local_execution_context()
        self.departure_recovery_reason = reason

    def continue_blocked_execution(self):
        if getattr(self,"task_geometry_autonomy",False):
            return False # Bounded candidate retries are owned exclusively by Local.
        context = getattr(self, "blocked_execution_context", None)
        if context is None:
            return False
        if getattr(self, "departure_recovery_terminal", False):
            return True
        now = self.get_clock().now().nanoseconds
        started = getattr(self, "departure_recovery_started_ns", now)
        self.departure_recovery_started_ns = min(started, now)
        changed = self.local_execution_context() != context
        expired = now - self.departure_recovery_started_ns >= int(
            getattr(self, "task_departure_retry_period", 2.0) * 1e9)
        # A changed cell/yaw during braking must not bypass the minimum wait.
        if not expired:
            return True
        if (getattr(self, "departure_recovery_reason", "") == "INVALID_TASK_CONTRACT"
                or getattr(self, "departure_recovery_attempts", 0) >=
                getattr(self, "task_departure_retry_limit", 2)):
            # A terminal result, not a silent wait or a remote viewpoint ban.
            self.departure_recovery_terminal = True
            task = self.active_observation
            if task is not None:
                self.cancel_observation_task(task)
            self.publish_status("DEPARTURE_RECOVERY_EXHAUSTED")
            # This execution is terminal, not the global exploration loop.
            # Cancel first so old candidates cannot restart, then arbitrate a
            # replacement exactly once using the failed task/request identity.
            if task is not None:
                request = getattr(self, "active_path_request_generation", None)
                terminal = String()
                terminal.data = ("LOCAL_RECOVERY_EXHAUSTED"
                    + (f" request_id={request}" if request is not None else "")
                    + f" task_id={task.task_id} task_version={task.task_version}")
                self.planning_status_callback(terminal)
            return True
        if self.departure_retry_ready():
            self.departure_recovery_attempts = getattr(self, "departure_recovery_attempts", 0) + 1
            self.departure_last_attempt_ns = now
            self.blocked_execution_context = None
            self.healthy_task_elapsed = 0.0
            # Ask for a different approach first; otherwise re-evaluate the
            # same objective from settled odometry. SCAN still certifies it.
            reroute = getattr(self, "reroute_active_goal", None)
            if reroute is None or not reroute(excluded_route=tuple(self.active_raw_path)):
                self.publish_reference_path(self.active_raw_path)
            self.publish_status("LOCAL_EXECUTION_RETRY_CHANGED_STATE" if changed
                                else "LOCAL_EXECUTION_RETRY_AFTER_SETTLING")
        return True
