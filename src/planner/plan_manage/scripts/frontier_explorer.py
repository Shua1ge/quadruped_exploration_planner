#!/usr/bin/env python3
"""Stage-one frontier explorer using only local world-frame LiDAR observations."""

import csv
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

from explorer_core.frontier_regions import (
    FrontierRegion, PersistentRegionTracker, RegionCommitmentUpdate,
    advance_completion_streak, advance_reroute_failure_streak, candidate_cells,
    cluster_frontiers, clustered_frontier_cells, observation_progress,
    observation_target_cells, partition_frontier_clusters, retain_region_commitment,
    region_information_efficiency, safe_viewpoint_cells, select_rolling_region,
    solve_open_held_karp, update_region_commitment,
)
from explorer_core.grid import (
    Cell, DirectedEdge, ExplorationGrid, FREE, GRID_MOVES, OCCUPIED, Point2, UNKNOWN,
    dilate_hit_ranges,
)
from explorer_core.path_planning import (
    RemainingPathCheck, ShortestPathTree, adjacent_grid_path_is_valid, astar_known,
    build_shortest_path_tree, match_blocked_path_edge, open_start_escape_corridor,
    path_length_cells, path_turn_cost, remaining_polyline_distance,
    segment_known_free, simplify_known_path, splice_prepared_path,
    validate_remaining_path,
)


REFERENCE_SCOPED_STATUSES = {
    "PATH_ACCEPTED", "PATH_TRAJECTORY_READY", "REACHED", "BLOCKED",
    "REFERENCE_PATH_REJECTED", "INVALID_REFERENCE_PATH",
}


def parse_planning_status(value: str) -> Tuple[str, Optional[int]]:
    """Decode SCAN status while retaining compatibility with plain statuses."""
    fields = value.split()
    if not fields:
        return "", None
    request_generation = None
    for field in fields[1:]:
        if not field.startswith("request_id="):
            continue
        try:
            parsed = int(field.partition("=")[2])
        except ValueError:
            continue
        if parsed > 0:
            request_generation = parsed
    return fields[0], request_generation


def planning_status_matches_request(
        status: str, request_generation: Optional[int],
        pending_generation: Optional[int],
        active_generation: Optional[int]) -> bool:
    """Return true only when a path-scoped status belongs to this request."""
    if status not in REFERENCE_SCOPED_STATUSES:
        return True
    expected = (pending_generation if status in (
        "PATH_ACCEPTED", "PATH_TRAJECTORY_READY",
        "REFERENCE_PATH_REJECTED", "INVALID_REFERENCE_PATH")
        else active_generation)
    return expected is not None and request_generation == expected


@dataclass
class FrontierCandidate:
    region_id: int
    cell: Cell
    frontier_cell: Cell
    goal: Point2
    path: List[Cell]
    path_length: float
    unknown_gain: int
    cluster_size: int
    turn_cost: float
    observation_cells: Set[Cell]


@dataclass
class ObservationTask:
    """A frozen information-gain objective; the goal pose is only guidance."""

    region_id: Optional[int]
    goal: Point2
    target_cells: Set[Cell]
    observed_cells: int = 0
    progress: float = 0.0
    completion_streak: int = 0
    last_evaluated_update: int = -1
    preparation_started: bool = False


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        resolution = float(self.declare_parameter("resolution", 0.20).value)
        size_x = float(self.declare_parameter("map_size_x", 64.0).value)
        size_y = float(self.declare_parameter("map_size_y", 40.0).value)
        self.grid = ExplorationGrid(size_x, size_y, resolution)
        self.mapping_range = float(self.declare_parameter("mapping_range", 7.5).value)
        self.no_return_range = float(self.declare_parameter("no_return_range", 3.0).value)
        self.ray_count = int(self.declare_parameter("ray_count", 720).value)
        self.hit_dilation_bins = int(self.declare_parameter("hit_dilation_bins", 3).value)
        self.obstacle_min_z = float(self.declare_parameter("obstacle_min_z", 0.08).value)
        self.obstacle_max_z = float(self.declare_parameter("obstacle_max_z", 0.85).value)
        self.inflation_radius = float(self.declare_parameter("inflation_radius", 0.65).value)
        self.viewpoint_standoff = float(
            self.declare_parameter("viewpoint_standoff", 1.0).value)
        self.min_frontier_size = int(self.declare_parameter("min_frontier_size", 6).value)
        self.min_goal_distance = float(self.declare_parameter("min_goal_distance", 2.0).value)
        self.blacklist_radius = float(self.declare_parameter("blacklist_radius", 1.5).value)
        self.goal_failure_cooldown_updates = int(
            self.declare_parameter("goal_failure_cooldown_updates", 6).value)
        if self.goal_failure_cooldown_updates < 1:
            raise ValueError("goal_failure_cooldown_updates must be at least one")
        self.global_reroute_failure_updates = int(
            self.declare_parameter("global_reroute_failure_updates", 3).value)
        if self.global_reroute_failure_updates < 1:
            raise ValueError("global_reroute_failure_updates must be at least one")
        self.map_update_period = float(self.declare_parameter("map_update_period", 0.5).value)
        self.goal_timeout = float(self.declare_parameter("goal_timeout", 120.0).value)
        self.blocked_edge_ttl = float(
            self.declare_parameter("blocked_edge_ttl", 30.0).value)
        self.frame_id = str(self.declare_parameter("frame_id", "world").value)
        self.auto_start = bool(self.declare_parameter("auto_start", True).value)
        self.selection_strategy = str(
            self.declare_parameter("selection_strategy", "hierarchical").value).lower()
        if self.selection_strategy not in ("greedy", "hierarchical"):
            raise ValueError("selection_strategy must be 'greedy' or 'hierarchical'")
        self.region_size = float(self.declare_parameter("region_size", 8.0).value)
        self.region_match_distance = float(
            self.declare_parameter("region_match_distance", 6.0).value)
        self.region_release_updates = int(
            self.declare_parameter("region_release_updates", 3).value)
        if self.region_release_updates < 1:
            raise ValueError("region_release_updates must be at least one")
        self.region_unreachable_timeout = float(
            self.declare_parameter("region_unreachable_timeout", 5.0).value)
        if self.region_unreachable_timeout <= 0.0:
            raise ValueError("region_unreachable_timeout must be positive")
        self.max_global_regions = int(
            self.declare_parameter("max_global_regions", 10).value)
        self.region_switch_ratio = float(
            self.declare_parameter("region_switch_ratio", 1.35).value)
        if self.region_switch_ratio < 1.0:
            raise ValueError("region_switch_ratio must be at least one")
        self.metrics_period = float(self.declare_parameter("metrics_period", 2.0).value)
        self.metrics_file = str(self.declare_parameter("metrics_file", "").value)
        self.observation_preplanning_enabled = bool(
            self.declare_parameter("observation_preplanning_enabled", True).value)
        self.planning_period = float(
            self.declare_parameter("planning_period", 0.25).value)
        self.observation_radius = float(
            self.declare_parameter("observation_radius", self.no_return_range).value)
        self.observation_prepare_ratio = float(
            self.declare_parameter("observation_prepare_ratio", 0.60).value)
        self.observation_done_ratio = float(
            self.declare_parameter("observation_done_ratio", 0.80).value)
        self.observation_done_updates = int(
            self.declare_parameter("observation_done_updates", 3).value)
        self.min_expected_observation_cells = int(
            self.declare_parameter("min_expected_observation_cells", 8).value)
        if not (0.0 <= self.observation_prepare_ratio
                < self.observation_done_ratio <= 1.0):
            raise ValueError(
                "observation ratios must satisfy 0 <= prepare < done <= 1")
        if self.observation_done_updates < 1:
            raise ValueError("observation_done_updates must be at least one")

        self.position: Optional[Point2] = None
        self.body_z = 0.3
        self.last_map_update_ns = 0
        self.map_update_count = 0
        self.active_goal: Optional[Point2] = None
        self.active_goal_cell: Optional[Cell] = None
        self.active_observation: Optional[ObservationTask] = None
        self.active_raw_path: List[Cell] = []
        self.active_path_progress_index = 0
        self.last_active_path_validation_update = -1
        self.prepared_candidate: Optional[FrontierCandidate] = None
        self.region_edge_cache: Dict[DirectedEdge, List[Cell]] = {}
        self.replacement_pending = False
        self.last_preparation_attempt_update = -1
        self.last_handoff_attempt_update = -1
        self.pipeline_latency_ewma = 0.0
        self.pending_path_publish_ns = 0
        self.last_path_request_generation = 0
        self.pending_path_request_generation: Optional[int] = None
        self.active_path_request_generation: Optional[int] = None
        self.global_reroute_failure_streak = 0
        self.active_since_ns = 0
        self.blacklist: List[Point2] = []
        self.completed_goals: List[Point2] = []
        self.goal_failure_cooldowns: Dict[Point2, int] = {}
        self.temporary_blocked_edges: Dict[DirectedEdge, int] = {}
        self.pending_blocked_edge: Optional[DirectedEdge] = None
        self.disabled_by_collision = False
        self.region_tracker = PersistentRegionTracker(
            self.region_match_distance / self.grid.resolution)
        self.region_sequence: List[int] = []
        self.active_region_id: Optional[int] = None
        self.active_region_missing_streak = 0
        self.active_region_unreachable_since: Optional[float] = None
        self.last_region_release_evaluation_update = -1
        self.last_selected_region_id: Optional[int] = None
        self.last_global_plan_ms = 0.0
        self.cumulative_planning_ms = 0.0
        self.region_switches = 0
        self.goals_reached = 0
        self.observations_satisfied = 0
        self.total_distance = 0.0
        self.revisit_distance = 0.0
        self.last_metric_position: Optional[Point2] = None
        self.last_motion_cell: Optional[Cell] = None
        self.visited_motion_cells: Set[Cell] = set()
        self.run_start_time = time.monotonic()
        self.run_id = str(time.time_ns())
        self.latest_frontier_count = 0
        self.latest_region_count = 0

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(PointCloud2, "cloud", self.cloud_callback, sensor_qos)
        self.create_subscription(Odometry, "body_pose", self.odom_callback, 10)
        self.create_subscription(String, "planning/status", self.planning_status_callback, 10)
        self.create_subscription(
            Path, "planning/blocked_segment", self.blocked_segment_callback, 10)
        self.create_subscription(Bool, "simulation/collision", self.collision_callback, 10)
        self.create_subscription(Bool, "explorer/enabled", self.enabled_callback, 10)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, "initial_path", 10)
        self.path_vis_pub = self.create_publisher(Path, "explorer/path", transient_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, "explorer/map", transient_qos)
        self.planning_map_pub = self.create_publisher(
            OccupancyGrid, "explorer/planning_map", transient_qos)
        self.frontier_pub = self.create_publisher(Marker, "explorer/frontiers", transient_qos)
        self.goal_pub = self.create_publisher(Marker, "explorer/goal", transient_qos)
        self.status_pub = self.create_publisher(String, "explorer/status", transient_qos)
        self.region_sequence_pub = self.create_publisher(
            String, "explorer/region_sequence", transient_qos)
        self.metrics_pub = self.create_publisher(String, "explorer/metrics", transient_qos)
        self.create_timer(self.planning_period, self.exploration_timer)
        self.create_timer(self.metrics_period, self.metrics_timer)
        self.publish_status("WAITING_FOR_LOCAL_MAP")
        self.get_logger().info(
            f"Explorer strategy={self.selection_strategy}, region_size={self.region_size:.1f} m, "
            f"observation_preplanning={self.observation_preplanning_enabled}, "
            f"observation_prepare={self.observation_prepare_ratio:.2f}, "
            f"observation_done={self.observation_done_ratio:.2f}")

    def publish_status(self, value: str):
        msg = String()
        msg.data = value
        self.status_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        new_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.last_metric_position is not None:
            distance = math.hypot(new_position[0] - self.last_metric_position[0],
                                  new_position[1] - self.last_metric_position[1])
            # Ignore simulator resets/teleports when accumulating benchmark distance.
            if distance <= 2.0:
                self.total_distance += distance
                cell = self.grid.world_to_cell(*new_position)
                if cell != self.last_motion_cell and self.grid.in_bounds(cell):
                    if cell in self.visited_motion_cells:
                        self.revisit_distance += distance
                    else:
                        self.visited_motion_cells.add(cell)
                    self.last_motion_cell = cell
        if self.last_motion_cell is None:
            cell = self.grid.world_to_cell(*new_position)
            if self.grid.in_bounds(cell):
                self.last_motion_cell = cell
                self.visited_motion_cells.add(cell)
        self.last_metric_position = new_position
        self.position = new_position
        self.body_z = msg.pose.pose.position.z

    def current_blocked_edges(self) -> Set[DirectedEdge]:
        now_ns = self.get_clock().now().nanoseconds
        expired = [edge for edge, expiry in self.temporary_blocked_edges.items()
                   if expiry <= now_ns]
        for edge in expired:
            del self.temporary_blocked_edges[edge]
        return set(self.temporary_blocked_edges)

    def blocked_segment_callback(self, msg: Path):
        if len(msg.poses) < 2 or len(self.active_raw_path) < 2:
            return
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warning(
                f"Ignoring blocked segment in frame '{msg.header.frame_id}', "
                f"expected '{self.frame_id}'")
            return

        free_pose = msg.poses[0].pose.position
        hit_pose = msg.poses[1].pose.position
        free_cell = self.grid.world_to_cell(free_pose.x, free_pose.y)
        hit_cell = self.grid.world_to_cell(hit_pose.x, hit_pose.y)
        edge = match_blocked_path_edge(self.active_raw_path, free_cell, hit_cell)
        if edge is None:
            return

        expiry_ns = (self.get_clock().now().nanoseconds
                     + int(self.blocked_edge_ttl * 1e9))
        self.temporary_blocked_edges[edge] = expiry_ns
        self.pending_blocked_edge = edge
        self.publish_status("LOCAL_BLOCKED_EDGE_RECORDED")
        self.get_logger().warning(
            f"Temporarily blocked directed edge {edge[0]} -> {edge[1]} "
            f"for {self.blocked_edge_ttl:.1f}s")

    def cloud_callback(self, msg: PointCloud2):
        if self.position is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_map_update_ns < int(self.map_update_period * 1e9):
            return
        points = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        if points.size == 0:
            return
        array = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        dx = array[:, 0] - self.position[0]
        dy = array[:, 1] - self.position[1]
        distances = np.hypot(dx, dy)
        mask = ((array[:, 2] >= self.obstacle_min_z)
                & (array[:, 2] <= self.obstacle_max_z)
                & (distances >= 0.35)
                & (distances < self.mapping_range))
        nearest = np.full(self.ray_count, np.inf, dtype=np.float64)
        if np.any(mask):
            angles = np.arctan2(dy[mask], dx[mask])
            bins = np.floor((angles + math.pi) * self.ray_count / (2.0 * math.pi)).astype(int)
            bins = np.clip(bins, 0, self.ray_count - 1)
            np.minimum.at(nearest, bins, distances[mask])
        nearest = dilate_hit_ranges(nearest, self.hit_dilation_bins)
        self.grid.integrate_ranges(
            self.position, nearest, self.mapping_range,
            no_return_range=self.no_return_range)
        # Ray endpoints are quantised by bearing; insert the original points as
        # occupied as well so a real wall cannot disappear between ray bins.
        if np.any(mask):
            self.grid.mark_occupied_points(array[mask, :2])
        self.last_map_update_ns = now_ns
        self.map_update_count += 1
        self.publish_maps()

    def evaluate_active_observation(self) -> Optional[ObservationTask]:
        """Update information progress once for every accepted map update."""
        task = self.active_observation
        if task is None or task.last_evaluated_update == self.map_update_count:
            return task
        observed, _, ratio = observation_progress(self.grid, task.target_cells)
        task.observed_cells = observed
        task.progress = ratio
        task.last_evaluated_update = self.map_update_count
        task.completion_streak = advance_completion_streak(
            ratio, self.observation_done_ratio, task.completion_streak)
        return task

    def prepare_next_observation(self) -> bool:
        """Select the next information objective without publishing its path."""
        task = self.active_observation
        if self.position is None or self.active_goal is None or task is None:
            return False
        if self.last_preparation_attempt_update == self.map_update_count:
            return self.prepared_candidate is not None
        self.last_preparation_attempt_update = self.map_update_count

        inflated = self.grid.inflated_obstacles(self.inflation_radius)
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        inflated = open_start_escape_corridor(
            self.grid, start, inflated,
            self.inflation_radius + self.grid.resolution)
        if inflated is None:
            self.publish_status("START_ESCAPE_UNAVAILABLE")
            return False
        frontiers = self.grid.frontier_cells(inflated)
        clusters = cluster_frontiers(frontiers, self.min_frontier_size)
        filtered_frontiers = clustered_frontier_cells(clusters)
        self.latest_frontier_count = len(filtered_frontiers)
        self.publish_frontiers(filtered_frontiers)
        candidate = self.choose_frontier(
            start, clusters, inflated, blocked_edges,
            excluded_goals=(self.active_goal,), allow_region_release=False)
        if candidate is None:
            self.publish_status("NEXT_OBSERVATION_NOT_YET_AVAILABLE")
            return False

        self.prepared_candidate = candidate
        task.preparation_started = True
        self.publish_status("NEXT_OBSERVATION_PREPARED")
        self.get_logger().info(
            f"Prepared next observation ({candidate.goal[0]:.2f}, "
            f"{candidate.goal[1]:.2f}), region={candidate.region_id}, "
            f"expected_gain={candidate.unknown_gain}")
        return True

    def activate_prepared_observation(self) -> bool:
        """Reuse the safe suffix of a prepared route from the current pose."""
        candidate = self.prepared_candidate
        if self.position is None or candidate is None:
            return False
        if self.is_goal_on_failure_cooldown(candidate.goal):
            self.prepared_candidate = None
            return False

        inflated = self.grid.inflated_obstacles(self.inflation_radius)
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        inflated = open_start_escape_corridor(
            self.grid, start, inflated,
            self.inflation_radius + self.grid.resolution)
        if inflated is None:
            self.prepared_candidate = None
            return False
        target_cells = observation_target_cells(
            self.grid, candidate.frontier_cell, self.observation_radius)
        if (not segment_known_free(
                    self.grid, candidate.cell, candidate.frontier_cell, inflated)
                or len(target_cells) < self.min_expected_observation_cells):
            self.prepared_candidate = None
            return False

        path = splice_prepared_path(
            self.grid, start, candidate.path, inflated, blocked_edges)
        if path is None:
            # The robot may have moved away from the prepared route.  Fall back
            # to one point-to-point search, never a fresh frontier enumeration.
            path = astar_known(
                self.grid, start, candidate.cell, inflated, blocked_edges)
        if not path:
            self.prepared_candidate = None
            return False
        simplified = simplify_known_path(
            self.grid, path, inflated, blocked_edges)
        if not simplified or not all(
                segment_known_free(self.grid, a, b, inflated, blocked_edges)
                for a, b in zip(simplified[:-1], simplified[1:])):
            self.prepared_candidate = None
            return False

        refreshed = FrontierCandidate(
            candidate.region_id, candidate.cell, candidate.frontier_cell,
            candidate.goal, path,
            path_length_cells(path, self.grid.resolution), len(target_cells),
            candidate.cluster_size, path_turn_cost(path), target_cells)
        self.prepared_candidate = None
        self.activate_candidate(simplified, refreshed)
        return True

    def activate_candidate(self, path: Sequence[Cell], candidate: FrontierCandidate):
        """Publish a candidate while keeping region accounting in one place."""
        self.global_reroute_failure_streak = 0
        if (self.last_selected_region_id is not None
                and candidate.region_id != self.last_selected_region_id):
            self.region_switches += 1
        self.last_selected_region_id = candidate.region_id
        self.active_region_id = candidate.region_id
        self.active_region_missing_streak = 0
        self.active_region_unreachable_since = None
        self.publish_path(path, candidate)

    def reroute_active_goal(self) -> bool:
        """Replan to the current observation pose without resetting its gain."""
        if (self.position is None or self.active_goal is None
                or self.active_goal_cell is None or self.active_observation is None):
            return False

        inflated = self.grid.inflated_obstacles(self.inflation_radius)
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        inflated = open_start_escape_corridor(
            self.grid, start, inflated,
            self.inflation_radius + self.grid.resolution)
        if inflated is None:
            return False
        raw_path = astar_known(
            self.grid, start, self.active_goal_cell, inflated, blocked_edges)
        if not raw_path:
            return False
        simplified = simplify_known_path(
            self.grid, raw_path, inflated, blocked_edges)
        if not simplified or not all(
                segment_known_free(self.grid, a, b, inflated, blocked_edges)
                for a, b in zip(simplified[:-1], simplified[1:])):
            return False

        self.publish_reference_path(simplified)
        self.active_raw_path = list(raw_path)
        self.active_path_progress_index = 0
        self.last_active_path_validation_update = self.map_update_count
        self.prepared_candidate = None
        self.last_preparation_attempt_update = -1
        self.last_handoff_attempt_update = -1
        self.active_since_ns = self.get_clock().now().nanoseconds
        self.publish_status("ACTIVE_GOAL_REROUTED")
        self.get_logger().info(
            f"Rerouted active observation to ({self.active_goal[0]:.2f}, "
            f"{self.active_goal[1]:.2f}) without resetting observation progress")
        return True

    def validate_and_repair_active_path(self) -> bool:
        """Validate one new map revision and atomically repair an invalid route.

        False means this timer iteration handled an invalid route and must not
        continue observation handoff logic using the stale task state.
        """
        if (self.position is None or self.active_goal is None
                or not self.active_raw_path):
            return True
        if self.last_active_path_validation_update == self.map_update_count:
            return True
        self.last_active_path_validation_update = self.map_update_count

        inflated = self.grid.inflated_obstacles(self.inflation_radius)
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        check = validate_remaining_path(
            self.grid, start, self.active_raw_path,
            self.active_path_progress_index, inflated, blocked_edges)
        self.active_path_progress_index = check.progress_index

        if check.valid:
            self.global_reroute_failure_streak = 0
            self.get_logger().debug(
                f"[GLOBAL_PATH_OK] map_update={self.map_update_count} "
                f"progress={check.progress_index + 1}/{len(self.active_raw_path)} "
                f"remaining={check.remaining_distance:.2f}m "
                f"path_error={check.lateral_error:.2f}m")
            return True

        invalid_world = (self.grid.cell_to_world(check.invalid_cell)
                         if check.invalid_cell is not None else None)
        invalid_text = ("none" if invalid_world is None else
                        f"({invalid_world[0]:.2f},{invalid_world[1]:.2f})")
        goal_text = f"({self.active_goal[0]:.2f},{self.active_goal[1]:.2f})"
        self.get_logger().warning(
            f"[GLOBAL_PATH_INVALID] map_update={self.map_update_count} "
            f"reason={check.reason} first_invalid={invalid_text} "
            f"progress={check.progress_index + 1}/{len(self.active_raw_path)} "
            f"remaining={check.remaining_distance:.2f}m "
            f"path_error={check.lateral_error:.2f}m goal={goal_text}",
            throttle_duration_sec=2.0)

        started = time.perf_counter()
        if self.reroute_active_goal():
            self.global_reroute_failure_streak = 0
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self.publish_status("GLOBAL_PATH_REROUTED_TO_SAME_OBSERVATION")
            self.get_logger().info(
                f"[GLOBAL_REROUTE_OK] mode=same_observation goal={goal_text} "
                f"path_cells={len(self.active_raw_path)} elapsed={elapsed_ms:.1f}ms; "
                "current safe local trajectory continues during replacement")
            return False

        previous_goal = self.active_goal
        if self.plan_from_current_position(excluded_goals=(previous_goal,)):
            self.global_reroute_failure_streak = 0
            self.add_goal_failure_cooldown(previous_goal, "GLOBAL_PATH_INVALID")
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            new_goal = self.active_goal
            new_goal_text = ("none" if new_goal is None else
                             f"({new_goal[0]:.2f},{new_goal[1]:.2f})")
            self.publish_status("GLOBAL_PATH_SWITCHED_OBSERVATION")
            self.get_logger().info(
                f"[GLOBAL_REROUTE_OK] mode=new_observation old_goal={goal_text} "
                f"new_goal={new_goal_text} elapsed={elapsed_ms:.1f}ms; "
                "current safe local trajectory continues during replacement")
            return False

        self.global_reroute_failure_streak, abandon_goal = (
            advance_reroute_failure_streak(
                self.global_reroute_failure_streak,
                self.global_reroute_failure_updates))
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if abandon_goal:
            self.add_goal_failure_cooldown(previous_goal, "GLOBAL_REROUTE_FAILED")
            self.active_goal = None
            self.active_goal_cell = None
            self.active_observation = None
            self.active_raw_path = []
            self.active_path_progress_index = 0
            self.pending_blocked_edge = None
            self.prepared_candidate = None
            self.replacement_pending = False
            self.active_path_request_generation = None
            self.global_reroute_failure_streak = 0
            self.publish_status("GLOBAL_PATH_ABANDONED_WAITING_FOR_ROUTE")
            self.get_logger().error(
                f"[GLOBAL_REROUTE_ABANDONED] goal={goal_text} "
                f"reason={check.reason} after "
                f"{self.global_reroute_failure_updates} failed map updates; "
                "goal placed on cooldown and a different route will be selected")
            return False

        self.publish_status("GLOBAL_PATH_INVALID_WAITING_FOR_ROUTE")
        self.get_logger().error(
            f"[GLOBAL_REROUTE_FAILED] goal={goal_text} reason={check.reason} "
            f"attempt={self.global_reroute_failure_streak}/"
            f"{self.global_reroute_failure_updates} elapsed={elapsed_ms:.1f}ms; "
            "robot remains stopped and the current observation task is retained "
            "for the next map update",
            throttle_duration_sec=2.0)
        return False

    def finish_active_observation(self, status: str):
        """Finish an information objective independently of pose arrival."""
        if self.active_goal is not None and not self.is_blacklisted(self.active_goal):
            self.completed_goals.append(self.active_goal)
        self.observations_satisfied += 1
        self.active_goal = None
        self.active_goal_cell = None
        self.active_observation = None
        self.active_raw_path = []
        self.pending_blocked_edge = None
        self.prepared_candidate = None
        self.replacement_pending = False
        self.active_path_request_generation = None
        self.pending_path_request_generation = None
        self.publish_status(status)

    def replace_unsatisfied_observation(self) -> bool:
        """Atomically replace a reached viewpoint that delivered too little gain."""
        if self.active_goal is None:
            self.replacement_pending = False
            return False
        if self.last_handoff_attempt_update == self.map_update_count:
            return False
        self.last_handoff_attempt_update = self.map_update_count
        previous_goal = self.active_goal

        switched = self.activate_prepared_observation()
        if not switched:
            switched = self.plan_from_current_position(
                excluded_goals=(previous_goal,))
        if switched:
            self.replacement_pending = False
            self.publish_status("VIEWPOINT_REACHED_GAIN_INSUFFICIENT_REROUTED")
            return True

        # Keep the old task object until a replacement is validated.  SCAN has
        # stopped at this pose, but the explorer can still consume map updates
        # and retry without creating an empty active/prepared state.
        self.publish_status("VIEWPOINT_REACHED_WAITING_FOR_SAFE_REPLACEMENT")
        return False

    def handoff_satisfied_observation(self, status: str) -> bool:
        """Switch atomically, retaining the safe current task on failure."""
        if self.active_goal is None:
            return False
        if self.last_handoff_attempt_update == self.map_update_count:
            return False
        self.last_handoff_attempt_update = self.map_update_count
        previous_goal = self.active_goal

        switched = self.activate_prepared_observation()
        if not switched:
            switched = self.plan_from_current_position(
                excluded_goals=(previous_goal,))
        if switched:
            if not self.is_blacklisted(previous_goal):
                self.completed_goals.append(previous_goal)
            self.observations_satisfied += 1
            self.publish_status(status)
            return True

        if self.latest_frontier_count == 0:
            self.finish_active_observation("EXPLORATION_COMPLETE")
            return True

        # The current goal is a known-free stand-off pose.  Keep its task
        # alive while waiting for a map update instead of creating a period
        # with neither an old task nor a validated new one.
        self.publish_status("WAITING_FOR_SAFE_HANDOFF")
        return False

    def planning_status_callback(self, msg: String):
        status, request_generation = parse_planning_status(msg.data)
        if not planning_status_matches_request(
                status, request_generation,
                self.pending_path_request_generation,
                self.active_path_request_generation):
            self.get_logger().warning(
                f"[STALE_PLANNING_STATUS_IGNORED] status={status} "
                f"request_id={request_generation} pending_id="
                f"{self.pending_path_request_generation} active_id="
                f"{self.active_path_request_generation}")
            return

        if status == "PATH_TRAJECTORY_READY" and self.pending_path_publish_ns:
            latency = ((self.get_clock().now().nanoseconds - self.pending_path_publish_ns) * 1e-9)
            if latency >= 0.0:
                self.pipeline_latency_ewma = (
                    latency if self.pipeline_latency_ewma <= 0.0 else
                    0.8 * self.pipeline_latency_ewma + 0.2 * latency)
                self.get_logger().info(
                    f"[LOCAL_TRAJECTORY_READY] pipeline_latency={latency:.2f}s, "
                    f"ewma={self.pipeline_latency_ewma:.2f}s")
            self.pending_path_publish_ns = 0
            self.pending_path_request_generation = None
        elif status in ("RUNNING", "PATH_ACCEPTED"):
            # A successful replacement trajectory resolves the pending local
            # failure.  Keep the short-lived edge record, but do not let it be
            # mistaken for the cause of a later unrelated BLOCKED status.
            self.pending_blocked_edge = None
        elif status == "REACHED" and self.active_goal is not None:
            task = self.evaluate_active_observation()
            self.goals_reached += 1
            if task is not None and task.progress >= self.observation_done_ratio:
                if task.completion_streak >= self.observation_done_updates:
                    self.handoff_satisfied_observation(
                        "OBSERVATION_HANDOFF_COMPLETE_AT_VIEWPOINT")
                else:
                    # The ratio is monotonic, so wait briefly for the configured
                    # number of map updates instead of misclassifying this as a
                    # failed viewpoint just because SCAN arrived first.
                    self.publish_status("VIEWPOINT_REACHED_WAITING_FOR_GAIN_CONFIRMATION")
            else:
                # Pose arrival is not task completion.  If this viewpoint did
                # not deliver enough information, atomically activate the
                # prepared route before discarding any active state.
                if not any(math.hypot(
                        self.active_goal[0] - old[0],
                        self.active_goal[1] - old[1]) < self.blacklist_radius
                        for old in self.blacklist):
                    self.blacklist.append(self.active_goal)
                self.replacement_pending = True
                self.replace_unsatisfied_observation()
        elif status == "BLOCKED":
            self.global_reroute_failure_streak = 0
            self.pending_path_publish_ns = 0
            self.pending_path_request_generation = None
            self.active_path_request_generation = None
            blocked_edge = self.pending_blocked_edge
            self.pending_blocked_edge = None
            if (blocked_edge is not None
                    and blocked_edge in self.current_blocked_edges()
                    and self.reroute_active_goal()):
                return

            previous_goal = self.active_goal
            self.active_goal = None
            self.active_goal_cell = None
            self.active_observation = None
            self.active_raw_path = []
            self.pending_blocked_edge = None
            self.prepared_candidate = None
            self.replacement_pending = False
            if previous_goal is not None:
                self.add_goal_failure_cooldown(previous_goal, "BLOCKED")
            if previous_goal is not None and self.plan_from_current_position():
                self.publish_status("LOCAL_BLOCKED_VIEWPOINT_CHANGED")
            else:
                self.publish_status("LOCAL_BLOCKED_WAITING_FOR_ROUTE")
        elif status in ("REFERENCE_PATH_REJECTED", "INVALID_REFERENCE_PATH"):
            self.global_reroute_failure_streak = 0
            self.pending_path_publish_ns = 0
            self.pending_path_request_generation = None
            self.active_path_request_generation = None
            if self.active_goal is not None:
                self.blacklist.append(self.active_goal)
            self.active_goal = None
            self.active_goal_cell = None
            self.active_observation = None
            self.active_raw_path = []
            self.pending_blocked_edge = None
            self.prepared_candidate = None
            self.replacement_pending = False
            self.publish_status(f"RECOVER_FROM_{status}")

    def collision_callback(self, msg: Bool):
        if msg.data:
            self.disabled_by_collision = True
            self.auto_start = False
            self.publish_status("STOPPED_BY_COLLISION")
            self.get_logger().error(
                "[PHYSICAL_COLLISION] simulator contact guard latched; Explorer "
                "disabled for this run (this is not a recoverable LOCAL_BLOCKED event)",
                throttle_duration_sec=2.0)

    def enabled_callback(self, msg: Bool):
        if self.disabled_by_collision and msg.data:
            self.get_logger().error("Cannot resume after a latched simulation collision")
            return
        self.auto_start = msg.data
        self.publish_status("ENABLED" if msg.data else "PAUSED")

    def is_blacklisted(self, point: Point2) -> bool:
        history = self.blacklist + self.completed_goals
        return any(math.hypot(point[0] - old[0], point[1] - old[1]) < self.blacklist_radius
                   for old in history)

    def add_goal_failure_cooldown(self, point: Point2, reason: str):
        release_update = self.map_update_count + self.goal_failure_cooldown_updates
        self.goal_failure_cooldowns[point] = max(
            release_update, self.goal_failure_cooldowns.get(point, 0))
        self.get_logger().warning(
            f"[GOAL_FAILURE_COOLDOWN] goal=({point[0]:.2f},{point[1]:.2f}) "
            f"reason={reason} release_update={release_update}")

    def is_goal_on_failure_cooldown(self, point: Point2) -> bool:
        expired = [goal for goal, release in self.goal_failure_cooldowns.items()
                   if self.map_update_count >= release]
        for goal in expired:
            del self.goal_failure_cooldowns[goal]
        return any(
            self.map_update_count < release
            and math.hypot(point[0] - goal[0], point[1] - goal[1])
            < self.blacklist_radius
            for goal, release in self.goal_failure_cooldowns.items())

    def exploration_timer(self):
        if not self.auto_start or self.position is None or self.map_update_count < 2:
            return
        if self.pending_path_publish_ns:
            request_age = ((self.get_clock().now().nanoseconds
                            - self.pending_path_publish_ns) * 1e-9)
            self.get_logger().info(
                f"[PATH_REQUEST_IN_FLIGHT] age={request_age:.2f}s; "
                "waiting for SCAN before publishing another reference path",
                throttle_duration_sec=2.0)
            return
        if self.active_goal is not None:
            if not self.validate_and_repair_active_path():
                return
            elapsed = (self.get_clock().now().nanoseconds - self.active_since_ns) * 1e-9
            if elapsed > self.goal_timeout:
                self.get_logger().warning("Frontier goal timed out; blacklisting it")
                self.blacklist.append(self.active_goal)
                self.active_goal = None
                self.active_goal_cell = None
                self.active_observation = None
                self.active_raw_path = []
                self.pending_blocked_edge = None
                self.prepared_candidate = None
                self.replacement_pending = False
            elif self.replacement_pending:
                self.replace_unsatisfied_observation()
                return
            else:
                task = self.evaluate_active_observation()
                if (task is not None
                        and task.completion_streak >= self.observation_done_updates):
                    self.get_logger().info(
                        f"Observation satisfied: {task.observed_cells}/"
                        f"{len(task.target_cells)} cells ({task.progress:.1%}); "
                        "switching before viewpoint arrival")
                    self.handoff_satisfied_observation(
                        "OBSERVATION_HANDOFF_COMPLETE")
                    return
                if (self.observation_preplanning_enabled and task is not None
                        and task.progress >= self.observation_prepare_ratio
                        and self.prepared_candidate is None):
                    self.prepare_next_observation()
                return

        self.plan_from_current_position()

    def plan_from_current_position(
            self, excluded_goals: Sequence[Point2] = ()) -> bool:
        """Select and publish a new terminal frontier from the robot position."""
        if self.position is None:
            return False

        inflated = self.grid.inflated_obstacles(self.inflation_radius)
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        inflated = open_start_escape_corridor(
            self.grid, start, inflated,
            self.inflation_radius + self.grid.resolution)
        if inflated is None:
            self.publish_status("START_ESCAPE_UNAVAILABLE")
            self.get_logger().warning(
                f"[FRONTIER_REACHABILITY] start={start} raw_start_free="
                f"{self.grid.in_bounds(start) and self.grid.value(start) == FREE} "
                "escape_corridor=unavailable",
                throttle_duration_sec=2.0)
            return False
        frontiers = self.grid.frontier_cells(inflated)
        clusters = cluster_frontiers(frontiers, self.min_frontier_size)
        filtered_frontiers = clustered_frontier_cells(clusters)
        self.latest_frontier_count = len(filtered_frontiers)
        self.publish_frontiers(filtered_frontiers)
        best = self.choose_frontier(
            start, clusters, inflated, blocked_edges,
            excluded_goals=excluded_goals)
        if best is None:
            status = ("EXPLORATION_COMPLETE" if not filtered_frontiers
                      else "NO_REACHABLE_FRONTIER")
            self.publish_status(status)
            self.get_logger().info(
                f"{status}: {len(filtered_frontiers)} usable frontier cells",
                throttle_duration_sec=5.0)
            return False
        raw_path = best.path
        goal_xy = best.goal
        simplified = simplify_known_path(
            self.grid, raw_path, inflated, blocked_edges)
        # Fail closed: never publish a simplified segment unless it is still
        # entirely known-free in the exact inflated map used by A*.
        if not simplified or not all(
                segment_known_free(self.grid, a, b, inflated, blocked_edges)
                for a, b in zip(simplified[:-1], simplified[1:])):
            self.blacklist.append(goal_xy)
            self.publish_status("REFERENCE_PATH_REJECTED")
            self.get_logger().warning("Rejected frontier path during final safety validation")
            return False
        self.activate_candidate(simplified, best)
        return True

    def choose_frontier(self, start: Cell, clusters: Sequence[Sequence[Cell]],
                        inflated: Set[Cell],
                        blocked_edges: Set[DirectedEdge],
                        excluded_goals: Sequence[Point2] = (),
                        allow_region_release: bool = True):
        planning_started = time.perf_counter()
        observations = partition_frontier_clusters(
            clusters, int(round(self.region_size / self.grid.resolution)))
        regions = self.region_tracker.update(observations, self.active_region_id)
        self.latest_region_count = len(regions)
        candidates = self.build_frontier_candidates(
            start, regions, inflated, blocked_edges, excluded_goals)

        if self.selection_strategy == "greedy":
            result = self.choose_greedy_candidate(candidates)
        else:
            result = self.choose_hierarchical_candidate(
                start, regions, candidates, inflated, blocked_edges,
                allow_region_release)

        self.last_global_plan_ms = (time.perf_counter() - planning_started) * 1000.0
        self.cumulative_planning_ms += self.last_global_plan_ms
        return result

    def build_frontier_candidates(self, start: Cell,
                                  regions: Sequence[FrontierRegion],
                                  inflated: Set[Cell],
                                  blocked_edges: Set[DirectedEdge],
                                  excluded_goals: Sequence[Point2] = ()) -> List[FrontierCandidate]:
        proposals = []
        for region in regions:
            for cluster in region.clusters:
                used_viewpoints: Set[Cell] = set()
                for frontier in candidate_cells(cluster):
                    target_cells = observation_target_cells(
                        self.grid, frontier, self.observation_radius)
                    unknown_gain = len(target_cells)
                    if unknown_gain < self.min_expected_observation_cells:
                        continue
                    viewpoints = safe_viewpoint_cells(
                        self.grid, frontier, inflated, self.viewpoint_standoff)
                    for viewpoint in viewpoints:
                        if viewpoint in used_viewpoints:
                            continue
                        used_viewpoints.add(viewpoint)
                        goal_xy = self.grid.cell_to_world(viewpoint)
                        direct_distance = math.hypot(
                            goal_xy[0] - self.position[0],
                            goal_xy[1] - self.position[1])
                        excluded = any(
                            math.hypot(goal_xy[0] - point[0], goal_xy[1] - point[1])
                            < self.blacklist_radius for point in excluded_goals)
                        if (direct_distance < self.min_goal_distance
                                or self.is_blacklisted(goal_xy)
                                or self.is_goal_on_failure_cooldown(goal_xy)
                                or excluded):
                            continue
                        proposals.append((
                            region.region_id, viewpoint, frontier, goal_xy,
                            unknown_gain, len(cluster), target_cells))

        # All candidate paths share one start and one planning map.  A single
        # shortest-path tree replaces one complete A* invocation per viewpoint.
        tree = build_shortest_path_tree(
            self.grid, start, inflated, blocked_edges,
            {proposal[1] for proposal in proposals})
        records = []
        for (region_id, viewpoint, frontier, goal_xy,
             unknown_gain, cluster_size, target_cells) in proposals:
            path = tree.path_to(viewpoint)
            if not path:
                continue
            records.append(FrontierCandidate(
                region_id, viewpoint, frontier, goal_xy, path,
                path_length_cells(path, self.grid.resolution), unknown_gain,
                cluster_size, path_turn_cost(path), target_cells))
        return records

    @staticmethod
    def candidate_utility(candidate: FrontierCandidate,
                          options: Sequence[FrontierCandidate]) -> float:
        max_gain = max(1, max(item.unknown_gain for item in options))
        max_cluster = max(1, max(item.cluster_size for item in options))
        max_length = max(1e-6, max(item.path_length for item in options))
        max_turn = max(1e-6, max(item.turn_cost for item in options))
        return (0.45 * candidate.unknown_gain / max_gain
                + 0.20 * candidate.cluster_size / max_cluster
                - 0.25 * candidate.path_length / max_length
                - 0.10 * candidate.turn_cost / max_turn)

    def choose_greedy_candidate(self, candidates: Sequence[FrontierCandidate]):
        best = None
        for candidate in candidates:
            direct_distance = math.hypot(candidate.goal[0] - self.position[0],
                                         candidate.goal[1] - self.position[1])
            score = self.candidate_utility(candidate, candidates)
            key = (score, direct_distance, -candidate.cell[0], -candidate.cell[1])
            if best is None or key > best[0]:
                best = (key, candidate)
        if best is None:
            return None
        return best[1]

    def choose_hierarchical_candidate(self, start: Cell,
                                      regions: Sequence[FrontierRegion],
                                      candidates: Sequence[FrontierCandidate],
                                      inflated: Set[Cell],
                                      blocked_edges: Set[DirectedEdge],
                                      allow_region_release: bool = True):
        by_region: Dict[int, List[FrontierCandidate]] = {}
        for candidate in candidates:
            by_region.setdefault(candidate.region_id, []).append(candidate)
        available = [region for region in regions if region.region_id in by_region]
        available_ids = {region.region_id for region in available}
        observed_ids = {region.region_id for region in regions}

        if (allow_region_release
                and self.last_region_release_evaluation_update != self.map_update_count):
            previous_active = self.active_region_id
            update = update_region_commitment(
                self.active_region_id, observed_ids, available_ids,
                self.active_region_missing_streak,
                self.active_region_unreachable_since, time.monotonic(),
                self.region_release_updates, self.region_unreachable_timeout)
            self.active_region_id = update.active_region_id
            self.active_region_missing_streak = update.missing_streak
            self.active_region_unreachable_since = update.unreachable_since
            self.last_region_release_evaluation_update = self.map_update_count
            if update.release_reason == "frontier_missing":
                self.get_logger().info(
                    f"[REGION_COMMITMENT_RELEASED] region={previous_active} "
                    f"after {self.region_release_updates} map updates with no "
                    "remaining frontier cluster")
            elif update.release_reason == "persistently_unreachable":
                self.get_logger().warning(
                    f"[REGION_COMMITMENT_RELEASED] region={previous_active} "
                    f"reason=persistently_unreachable timeout="
                    f"{self.region_unreachable_timeout:.1f}s; re-evaluating all "
                    "reachable regions")

        if (self.active_region_id is not None
                and self.active_region_id not in available_ids):
            if allow_region_release:
                frontier_cells = next((
                    len(region.cells) for region in regions
                    if region.region_id == self.active_region_id), 0)
                unreachable_duration = 0.0
                if self.active_region_unreachable_since is not None:
                    unreachable_duration = max(
                        0.0, time.monotonic()
                        - self.active_region_unreachable_since)
                self.get_logger().info(
                    f"[REGION_COMMITMENT_RETAINED] region={self.active_region_id} "
                    f"frontier_present={self.active_region_id in observed_ids} "
                    f"frontier_cells={frontier_cells} candidate_reachable=False "
                    f"other_reachable_regions="
                    f"{len(available_ids - {self.active_region_id})} "
                    f"unreachable_for={unreachable_duration:.1f}/"
                    f"{self.region_unreachable_timeout:.1f}s missing_frontier="
                    f"{self.active_region_missing_streak}/{self.region_release_updates}; "
                    "waiting for a reachable viewpoint in the committed region",
                    throttle_duration_sec=2.0)
            else:
                self.get_logger().info(
                    f"[REGION_PREPARATION_WAIT] region={self.active_region_id} "
                    "has no usable successor in this map update; commitment "
                    "was not released by background preparation",
                    throttle_duration_sec=2.0)
            return None

        if not available:
            self.region_sequence = []
            self.publish_region_sequence()
            return None

        # Bound exact Held-Karp work.  Keep the committed region, then prefer
        # regions with larger frontiers and lower real A* access cost.
        if len(available) > self.max_global_regions:
            def priority(region):
                access = min(item.path_length for item in by_region[region.region_id])
                gain = len(region.cells)
                committed = region.region_id == self.active_region_id
                return (not committed, -gain, access, region.region_id)
            available = sorted(available, key=priority)[:self.max_global_regions]

        self.region_sequence, need_global_plan = retain_region_commitment(
            self.region_sequence, available_ids, self.active_region_id)
        if need_global_plan:
            self.region_sequence = self.plan_region_sequence(
                start, available, by_region, inflated, blocked_edges)

        # The full tour is only a prediction for future preparation.  Immediate
        # control is a one-step rolling decision based on real A* access cost.
        best_by_region = {
            region_id: max(options, key=lambda item: (
                region_information_efficiency(
                    item.unknown_gain, item.cluster_size,
                    item.path_length, item.turn_cost),
                self.candidate_utility(item, options),
                -item.path_length, -item.cell[0], -item.cell[1]))
            for region_id, options in by_region.items() if options}
        region_scores = {
            region_id: region_information_efficiency(
                candidate.unknown_gain, candidate.cluster_size,
                candidate.path_length, candidate.turn_cost)
            for region_id, candidate in best_by_region.items()}
        selected_region = select_rolling_region(
            region_scores, self.active_region_id, self.region_switch_ratio)
        if selected_region is None:
            return None

        # Keep the prediction observable, but make its first item agree with
        # the rolling decision rather than letting Held-Karp choose the action.
        prediction = [selected_region] + [
            region_id for region_id in self.region_sequence
            if region_id != selected_region and region_id in available_ids]
        if prediction != self.region_sequence:
            self.region_sequence = prediction
            self.publish_region_sequence()
        elif need_global_plan:
            self.publish_region_sequence()

        selected = best_by_region[selected_region]
        active_score = region_scores.get(self.active_region_id)
        self.get_logger().info(
            f"[ROLLING_REGION_CHOICE] selected={selected_region} "
            f"active={self.active_region_id} score={region_scores[selected_region]:.3f} "
            f"active_score={'none' if active_score is None else f'{active_score:.3f}'} "
            f"switch_ratio={self.region_switch_ratio:.2f}",
            throttle_duration_sec=2.0)
        return selected

    def plan_region_sequence(self, start: Cell,
                             regions: Sequence[FrontierRegion],
                             by_region: Dict[int, List[FrontierCandidate]],
                             inflated: Set[Cell],
                             blocked_edges: Set[DirectedEdge]) -> List[int]:
        # Each region is represented by its cheapest currently reachable
        # frontier entry.  Every matrix edge is an actual known-free A* route.
        representatives = [
            max(by_region[region.region_id], key=lambda item: (
                self.candidate_utility(item, by_region[region.region_id]),
                -item.path_length, -item.cell[0], -item.cell[1]))
            for region in regions]
        count = len(representatives)
        costs = np.full((count + 1, count + 1), np.inf, dtype=np.float64)
        np.fill_diagonal(costs, 0.0)
        for index, representative in enumerate(representatives):
            costs[0, index + 1] = representative.path_length

        active_cache_keys = {
            (representatives[source].cell, representatives[target].cell)
            for source in range(count)
            for target in range(count) if source != target}
        self.region_edge_cache = {
            edge: path for edge, path in self.region_edge_cache.items()
            if edge in active_cache_keys}

        for source in range(count):
            source_cell = representatives[source].cell
            missing_targets: Set[Cell] = set()
            for target in range(count):
                if source == target:
                    continue
                target_cell = representatives[target].cell
                edge = (source_cell, target_cell)
                path = self.region_edge_cache.get(edge)
                if path is not None and not adjacent_grid_path_is_valid(
                        self.grid, path, inflated, blocked_edges):
                    del self.region_edge_cache[edge]
                    path = None
                if path:
                    costs[source + 1, target + 1] = path_length_cells(
                        path, self.grid.resolution)
                else:
                    missing_targets.add(target_cell)

            if not missing_targets:
                continue
            tree = build_shortest_path_tree(
                self.grid, source_cell, inflated, blocked_edges, missing_targets)
            for target in range(count):
                if source == target:
                    continue
                target_cell = representatives[target].cell
                if target_cell not in missing_targets:
                    continue
                path = tree.path_to(target_cell)
                if not path:
                    continue
                self.region_edge_cache[(source_cell, target_cell)] = path
                costs[source + 1, target + 1] = path_length_cells(
                    path, self.grid.resolution)

        forced_first = None
        if self.active_region_id is not None:
            for index, region in enumerate(regions):
                if region.region_id == self.active_region_id:
                    forced_first = index
                    break
        order = solve_open_held_karp(costs, forced_first)
        if order is None:
            # All representatives are robot-reachable in an undirected grid,
            # but keep a deterministic safe fallback for a changing map.
            return [region.region_id for region in sorted(
                regions,
                key=lambda region: min(
                    item.path_length for item in by_region[region.region_id]))]
        return [regions[index].region_id for index in order]

    def publish_region_sequence(self):
        msg = String()
        msg.data = json.dumps({
            "strategy": self.selection_strategy,
            "active_region": self.active_region_id,
            "sequence": self.region_sequence,
        }, separators=(",", ":"))
        self.region_sequence_pub.publish(msg)
        self.get_logger().info(f"Region sequence: {self.region_sequence}")

    def publish_reference_path(self, cells: Sequence[Cell]):
        msg = Path()
        now_ns = self.get_clock().now().nanoseconds
        request_generation = max(now_ns, self.last_path_request_generation + 1)
        self.last_path_request_generation = request_generation
        msg.header.stamp.sec = request_generation // 1_000_000_000
        msg.header.stamp.nanosec = request_generation % 1_000_000_000
        msg.header.frame_id = self.frame_id
        world_points = [self.grid.cell_to_world(cell) for cell in cells]
        world_points[0] = self.position
        for index, (x, y) in enumerate(world_points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            neighbour = (world_points[index + 1]
                         if index + 1 < len(world_points) else world_points[index - 1])
            yaw = math.atan2(neighbour[1] - y, neighbour[0] - x)
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            msg.poses.append(pose)
        self.path_pub.publish(msg)
        self.path_vis_pub.publish(msg)
        self.pending_path_publish_ns = now_ns
        self.pending_path_request_generation = request_generation
        self.active_path_request_generation = request_generation

    def publish_path(self, cells: Sequence[Cell], candidate: FrontierCandidate):
        goal_xy = candidate.goal
        region_id = candidate.region_id
        self.publish_reference_path(cells)
        self.active_goal = goal_xy
        self.active_goal_cell = cells[-1]
        self.active_raw_path = list(candidate.path)
        self.active_path_progress_index = 0
        self.last_active_path_validation_update = self.map_update_count
        self.pending_blocked_edge = None
        self.replacement_pending = False
        self.active_observation = ObservationTask(
            region_id=region_id,
            goal=goal_xy,
            target_cells=set(candidate.observation_cells))
        self.prepared_candidate = None
        self.last_preparation_attempt_update = -1
        self.last_handoff_attempt_update = -1
        self.active_since_ns = self.get_clock().now().nanoseconds
        self.publish_goal(goal_xy)
        self.publish_status("PATH_PUBLISHED")
        world_points = [self.grid.cell_to_world(cell) for cell in cells]
        world_points[0] = self.position
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(world_points[:-1], world_points[1:]))
        self.get_logger().info(
            f"Selected observation ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}), "
            f"region={region_id}, strategy={self.selection_strategy}; "
            f"expected_gain={len(candidate.observation_cells)}, "
            f"published {len(world_points)} waypoints, {length:.2f} m")

    def metrics_timer(self):
        known_cells = int(np.count_nonzero(self.grid.data != UNKNOWN))
        total_cells = int(self.grid.data.size)
        record = {
            "run_id": self.run_id,
            "elapsed_s": round(time.monotonic() - self.run_start_time, 3),
            "strategy": self.selection_strategy,
            "coverage": known_cells / total_cells if total_cells else 0.0,
            "known_cells": known_cells,
            "frontier_cells": self.latest_frontier_count,
            "regions": self.latest_region_count,
            "total_distance_m": self.total_distance,
            "revisit_distance_m": self.revisit_distance,
            "region_switches": self.region_switches,
            "goals_reached": self.goals_reached,
            "observations_satisfied": self.observations_satisfied,
            "observation_progress": (
                self.active_observation.progress if self.active_observation else 0.0),
            "observation_observed_cells": (
                self.active_observation.observed_cells if self.active_observation else 0),
            "observation_expected_cells": (
                len(self.active_observation.target_cells)
                if self.active_observation else 0),
            "next_observation_prepared": self.prepared_candidate is not None,
            "pipeline_latency_ewma_s": self.pipeline_latency_ewma,
            "last_planning_ms": self.last_global_plan_ms,
            "cumulative_planning_ms": self.cumulative_planning_ms,
            "collision": self.disabled_by_collision,
        }
        msg = String()
        msg.data = json.dumps(record, separators=(",", ":"))
        self.metrics_pub.publish(msg)
        if self.metrics_file:
            self.append_metrics_csv(record)

    def append_metrics_csv(self, record: Dict[str, object]):
        try:
            directory = os.path.dirname(os.path.abspath(self.metrics_file))
            os.makedirs(directory, exist_ok=True)
            write_header = not os.path.exists(self.metrics_file)
            with open(self.metrics_file, "a", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(record))
                if write_header:
                    writer.writeheader()
                writer.writerow(record)
        except OSError as error:
            self.get_logger().error(f"Unable to write metrics file: {error}")
            self.metrics_file = ""

    def occupancy_message(self, planning: bool = False) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.width
        msg.info.height = self.grid.height
        msg.info.origin.position.x = self.grid.origin_x
        msg.info.origin.position.y = self.grid.origin_y
        msg.info.origin.orientation.w = 1.0
        data = self.grid.data.copy()
        if planning:
            for x, y in self.grid.inflated_obstacles(self.inflation_radius):
                data[y, x] = OCCUPIED
        msg.data = data.reshape(-1).astype(int).tolist()
        return msg

    def publish_maps(self):
        self.map_pub.publish(self.occupancy_message(False))
        self.planning_map_pub.publish(self.occupancy_message(True))

    def publish_frontiers(self, cells: Iterable[Cell]):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = 0.10
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = 1.0
        for cell in cells:
            x, y = self.grid.cell_to_world(cell)
            marker.points.append(Point(x=x, y=y, z=0.08))
        self.frontier_pub.publish(marker)

    def publish_goal(self, goal: Point2):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "exploration_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal[0]
        marker.pose.position.y = goal[1]
        marker.pose.position.z = self.body_z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.goal_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
