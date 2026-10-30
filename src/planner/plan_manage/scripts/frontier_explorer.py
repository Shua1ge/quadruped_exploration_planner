#!/usr/bin/env python3
"""Stage-one frontier explorer using only local world-frame LiDAR observations."""

import array
import csv
import json
import math
import os
import statistics
import time
import traceback
from collections import deque
from dataclasses import dataclass, field, replace
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from scan_planner_msgs.msg import (
    FailureEvidence, LocomotionState, TopoGraphDelta, TrajectoryAck)
from scan_planner_msgs.msg import ObservationTask as TaskMessage, ObservationFeedback
from explorer_core.observation_task_bridge import ObservationTaskBridge, PendingObservationHandoff

from explorer_core.frontier_regions import (
    FrontierRegion, PersistentRegionTracker, RegionCommitmentUpdate,
    advance_completion_streak, advance_reroute_failure_streak, candidate_cells,
    cluster_frontiers, clustered_frontier_cells, frontier_cluster_present,
    observation_progress, observation_target_cells, observation_hypothesis_cells,
    partition_frontier_clusters,
    partition_frontier_clusters_by_topology, retain_region_commitment,
    region_commitment_scope, region_information_efficiency,
    double_cylinder_footprint_free, double_cylinder_path_free, diagnose_double_cylinder_path,
    footprint_aware_path,
    safe_viewpoint_cells, select_rolling_region,
    solve_open_held_karp, update_region_commitment,
)
from explorer_core.execution_protocol import (
    REFERENCE_SCOPED_STATUSES, completion_status_is_new,
    parse_local_execution_event, parse_planning_status,
    planning_status_matches_request,
)
from explorer_core.grid import (
    Cell, DirectedEdge, ExplorationGrid, FREE, GRID_MOVES, GROUND_FLOOR_TOLERANCE, OCCUPIED, Point2, UNKNOWN,
    PoseSample, bresenham, classify_planar_ranges, classify_planar_scan, project_spatial_rays,
    dilate_hit_ranges,
    interpolate_se3_pose,
    quaternion_rotation_matrix, rpy_rotation_matrix,
)
from explorer_core.path_planning import (
    CoarseRouteRepair, ExpansionBudget, RouteSearchStatus,
    RemainingPathCheck, ShortestPathTree, adjacent_grid_path_is_valid, astar_known,
    build_shortest_path_tree, densified_reference_path, match_blocked_path_edge, open_start_escape_corridor,
    path_length_cells, path_turn_cost, remaining_polyline_distance,
    segment_known_free, simplify_known_path, splice_prepared_path,
    validate_remaining_path,
)
from explorer_core.observation_lifecycle import (
    ObservationTask, evaluate_observation, observation_completion_record,
    observation_is_complete,
)
from explorer_core.planning_coordinator import (
    PlanningSnapshotCoordinator, with_planning_grid_snapshot,
)
from explorer_core.portal_connectivity import SafeRegionConnectivity
from explorer_core.replan_control import ReplanContext, ReplanGate, retry_epoch
from explorer_core.region_commitment import BranchCommitment, ResidualCommitmentGate
from explorer_core.region_overlay import build_region_overlay, region_rgb
from explorer_core.region_visit_planner import RegionVisitPlanner
from explorer_core.sparse_routing import (
    SparseEdge, SparseNode, SparseRouteEstimate, SparseRouteGraph,
)


SPARSE_MISS_REASONS = (
    "start_unattached", "target_unattached", "disconnected",
    "connector_rejected", "blocked_disabled",
)


def directional_route_allowed(
        origin: Point2, forward: Point2, progress_floor: float,
        route: Sequence[Point2], backtrack_tolerance: float) -> bool:
    """Reject a route that crosses behind the committed moving gate."""
    if not route:
        return False
    norm = math.hypot(forward[0], forward[1])
    if norm <= 1e-9:
        return True
    unit = (forward[0] / norm, forward[1] / norm)
    projections = [
        (point[0] - origin[0]) * unit[0]
        + (point[1] - origin[1]) * unit[1]
        for point in route]
    lower_bound = float(progress_floor) - max(
        0.0, float(backtrack_tolerance))
    return min(projections) >= lower_bound and projections[-1] >= lower_bound


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
    sparse_route: Optional[SparseRouteEstimate] = None
    terminal_cells: Tuple[Cell, ...] = ()
    terminal_clearance: float = 0.0
    frontier_cluster_cells: Set[Cell] = field(default_factory=set)
    connectivity_gain: int = 0
    provisional_portal: Optional[Tuple[int, int]] = None
    commitment_scope: Optional[Tuple[str, int]] = None
    option_target_cells: Set[Cell] = field(default_factory=set)
    excluded_goals: Tuple[Point2, ...] = ()
    observation_progress: float = 0.0
    route_source: str = "dense"


class FrontierExplorer(RegionVisitPlanner, ObservationTaskBridge, Node):
    @property
    def grid(self) -> ExplorationGrid:
        coordinator = getattr(self, "_planning_coordinator", None)
        if coordinator is not None:
            return coordinator.value("grid", self._live_grid)
        context = getattr(self, "_planning_context", None)
        planning_grid = (None if context is None
                         else getattr(context, "grid", None))
        if planning_grid is not None:
            return planning_grid
        return self._live_grid

    @grid.setter
    def grid(self, value: ExplorationGrid):
        self._live_grid = value

    @property
    def position(self) -> Optional[Point2]:
        coordinator = getattr(self, "_planning_coordinator", None)
        if coordinator is not None:
            return coordinator.value("position", self._live_position)
        context = getattr(self, "_planning_context", None)
        if context is not None and hasattr(context, "position"):
            return context.position
        return self._live_position

    @position.setter
    def position(self, value: Optional[Point2]):
        self._live_position = value

    @property
    def map_update_count(self) -> int:
        coordinator = getattr(self, "_planning_coordinator", None)
        if coordinator is not None:
            return coordinator.value(
                "map_update_count", self._live_map_update_count)
        context = getattr(self, "_planning_context", None)
        if context is not None and hasattr(context, "map_update_count"):
            return context.map_update_count
        return self._live_map_update_count

    @map_update_count.setter
    def map_update_count(self, value: int):
        self._live_map_update_count = value

    @property
    def map_content_revision(self) -> int:
        coordinator = getattr(self, "_planning_coordinator", None)
        if coordinator is not None:
            return coordinator.value(
                "map_content_revision", self._live_map_content_revision)
        context = getattr(self, "_planning_context", None)
        if context is not None and hasattr(context, "map_content_revision"):
            return context.map_content_revision
        return self._live_map_content_revision

    @map_content_revision.setter
    def map_content_revision(self, value: int):
        self._live_map_content_revision = value

    def planning_grid_snapshot(self):
        """Bind only this planning thread to a stable map copy."""
        return self._planning_coordinator.bind(
            self._live_grid, self._live_position, self._live_map_update_count,
            self._live_map_content_revision)

    def __init__(self):
        super().__init__("frontier_explorer")
        self._planning_coordinator = PlanningSnapshotCoordinator()
        # Compatibility aliases for map integration and focused unit tests.
        self._grid_lock = self._planning_coordinator.lock
        self._planning_context = self._planning_coordinator.context
        # Sensor callbacks must remain serviceable while the global search is
        # running.  Each group is internally serialized; the executor may run
        # the two groups concurrently, while planning reads an immutable grid.
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        # Locomotion validity is a safety contract for map fusion.  It must not
        # sit behind a point-cloud callback or a long Python planning callback,
        # otherwise a healthy high-rate supervisor appears stale and freezes
        # observation progress.
        self.locomotion_callback_group = MutuallyExclusiveCallbackGroup()
        self.planning_callback_group = MutuallyExclusiveCallbackGroup()
        resolution = float(self.declare_parameter("resolution", 0.20).value)
        size_x = float(self.declare_parameter("map_size_x", 64.0).value)
        size_y = float(self.declare_parameter("map_size_y", 40.0).value)
        self.grid = ExplorationGrid(size_x, size_y, resolution)
        self.use_laser_scan = bool(self.declare_parameter("use_laser_scan", False).value)
        self.use_ray_cloud = bool(self.declare_parameter("use_ray_cloud", False).value)
        if self.use_ray_cloud and self.use_laser_scan:
            raise ValueError("choose one identified sensor input, not both")
        self.mapping_range = float(self.declare_parameter("mapping_range", 7.5).value)
        self.no_return_range = float(self.declare_parameter("no_return_range", 3.0).value)
        self.ray_count = int(self.declare_parameter("ray_count", 720).value)
        self.hit_dilation_bins = int(self.declare_parameter("hit_dilation_bins", 3).value)
        self.obstacle_min_z = float(self.declare_parameter("obstacle_min_z", 0.08).value)
        # Body-relative obstacle band, mirroring Local's
        # low_return_z_below_body / high_return_z_above_body. Hits outside it
        # stay clearing evidence: a 3-D lidar sees the floor and the tunnel
        # beams, and neither may be projected into the planar obstacle grid.
        self.obstacle_max_z = float(self.declare_parameter("obstacle_max_z", 0.25).value)
        # Unknown-height XYZ remains a physical voxel. Only project it into
        # the body slice if Local's upward inflation can reach that slice.
        # NewMine derives this bound from planner.yaml; other inputs retain
        # their existing band unless the platform provides a slice bound.
        self.body_slice_min_z = float(self.declare_parameter(
            "body_slice_min_z", self.obstacle_min_z).value)
        # Clearing window, independent of the occupancy band. Occupancy stays
        # narrow so overhead structure is not a blocked cell, but a ray that
        # passes above the body still proves the cells under it are traversable.
        # Keeping one narrow band for both rejects every clearing prefix on a
        # pitched 3-D lidar and leaves only the cells around the robot free.
        self.clearing_max_z = float(
            self.declare_parameter("clearing_max_z", self.obstacle_max_z).value)
        # Ground-relative clearance volume for returns whose height above the
        # observed local ground is encoded in the return_type byte (shared wire
        # contract with plan_env/ground_height.hpp). Below
        # terrain_floor_tolerance the return is the ground; below
        # obstacle_min_height the legs handle the terrain; up to
        # obstacle_max_height the surface can reach the body and blocks; above
        # it is overhead structure. Unknown heights use body_slice_min_z up to
        # obstacle_max_z; the wider raw-return band still bounds clearing.
        self.terrain_floor_tolerance = float(self.declare_parameter(
            "terrain_floor_tolerance", GROUND_FLOOR_TOLERANCE).value)
        self.obstacle_min_height = float(
            self.declare_parameter("obstacle_min_height", 0.22).value)
        self.obstacle_max_height = float(
            self.declare_parameter("obstacle_max_height", 0.55).value)
        self.obstacle_z_relative_to_body = bool(
            self.declare_parameter("obstacle_z_relative_to_body", False).value)
        # Occupancy bounds do not prescribe the sensor origin or clearing band.
        self._validate_obstacle_band(
            self.obstacle_z_relative_to_body, self.obstacle_min_z, self.obstacle_max_z)
        self.cloud_is_world = bool(
            self.declare_parameter("cloud_is_world", True).value)
        self.cloud_frame_id = str(
            self.declare_parameter("cloud_frame_id", "world").value)
        self.need_extrinsic = bool(
            self.declare_parameter("need_extrinsic", False).value)
        self.lidar_extrinsic_x = float(
            self.declare_parameter("lidar_extrinsic_x", 0.0).value)
        self.lidar_extrinsic_y = float(
            self.declare_parameter("lidar_extrinsic_y", 0.0).value)
        self.lidar_extrinsic_z = float(
            self.declare_parameter("lidar_extrinsic_z", 0.0).value)
        self.lidar_extrinsic_roll = float(
            self.declare_parameter("lidar_extrinsic_roll", 0.0).value)
        self.lidar_extrinsic_pitch = float(
            self.declare_parameter("lidar_extrinsic_pitch", 0.0).value)
        self.lidar_extrinsic_yaw = float(
            self.declare_parameter("lidar_extrinsic_yaw", 0.0).value)
        self.lidar_extrinsic_rotation = rpy_rotation_matrix(
            self.lidar_extrinsic_roll, self.lidar_extrinsic_pitch,
            self.lidar_extrinsic_yaw)
        self.lidar_extrinsic_translation = np.array([
            self.lidar_extrinsic_x, self.lidar_extrinsic_y,
            self.lidar_extrinsic_z], dtype=np.float64)
        self.inflation_radius = float(self.declare_parameter("inflation_radius", 0.65).value)
        self.reference_path_max_segment = float(self.declare_parameter(
            "reference_path_max_segment", 0.75).value)
        if not math.isfinite(self.reference_path_max_segment) or self.reference_path_max_segment <= 0.0:
            raise ValueError("reference_path_max_segment must be finite and positive")
        self.footprint_radius = float(
            self.declare_parameter("footprint_radius", 0.35).value)
        self.footprint_offset = float(
            self.declare_parameter("footprint_offset", 0.18).value)
        self.get_logger().info(
            "[SENSOR_CONTRACT] "
            f"cloud_is_world={self.cloud_is_world} "
            f"cloud_frame={self.cloud_frame_id} "
            f"need_extrinsic={self.need_extrinsic} "
            f"lidar_xyz=({self.lidar_extrinsic_x:.3f},"
            f"{self.lidar_extrinsic_y:.3f},{self.lidar_extrinsic_z:.3f}) "
            f"footprint_radius={self.footprint_radius:.3f} "
            f"footprint_offset={self.footprint_offset:.3f}")
        self.viewpoint_standoff = float(
            self.declare_parameter("viewpoint_standoff", 1.0).value)
        self.viewpoint_relaxation = float(self.declare_parameter(
            "viewpoint_relaxation", 0.0).value)
        self.terminal_candidate_limit = int(
            self.declare_parameter("terminal_candidate_limit", 8).value)
        self.terminal_clearance_search_radius = float(self.declare_parameter(
            "terminal_clearance_search_radius", 1.0).value)
        self.terminal_approach_length = float(self.declare_parameter(
            "terminal_approach_length", 1.5).value)
        self.min_frontier_size = int(self.declare_parameter("min_frontier_size", 6).value)
        self.min_goal_distance = float(self.declare_parameter("min_goal_distance", 2.0).value)
        self.preferred_goal_path_length = float(
            self.declare_parameter("preferred_goal_path_length", 3.5).value)
        self.long_horizon_min_gain_ratio = float(
            self.declare_parameter("long_horizon_min_gain_ratio", 0.65).value)
        if self.preferred_goal_path_length < self.min_goal_distance:
            raise ValueError(
                "preferred_goal_path_length must not be shorter than min_goal_distance")
        if not 0.0 <= self.long_horizon_min_gain_ratio <= 1.0:
            raise ValueError("long_horizon_min_gain_ratio must be within [0, 1]")
        self.blacklist_radius = float(self.declare_parameter("blacklist_radius", 1.5).value)
        self.goal_failure_cooldown_updates = int(
            self.declare_parameter("goal_failure_cooldown_updates", 6).value)
        if self.goal_failure_cooldown_updates < 1:
            raise ValueError("goal_failure_cooldown_updates must be at least one")
        self.region_failure_cooldown_updates = int(self.declare_parameter(
            "region_failure_cooldown_updates", 20).value)
        if self.region_failure_cooldown_updates < 1:
            raise ValueError(
                "region_failure_cooldown_updates must be at least one")
        self.region_backtrack_tolerance = float(self.declare_parameter(
            "region_backtrack_tolerance", 0.60).value)
        self.region_recovery_backtrack_tolerance = float(
            self.declare_parameter(
                "region_recovery_backtrack_tolerance", 2.0).value)
        self.region_direction_lookahead = float(self.declare_parameter(
            "region_direction_lookahead", 1.0).value)
        if self.region_backtrack_tolerance < 0.0:
            raise ValueError("region_backtrack_tolerance must be non-negative")
        if (self.region_recovery_backtrack_tolerance
                < self.region_backtrack_tolerance):
            raise ValueError(
                "region_recovery_backtrack_tolerance must not be smaller "
                "than region_backtrack_tolerance")
        if self.region_direction_lookahead <= 0.0:
            raise ValueError("region_direction_lookahead must be positive")
        self.global_reroute_failure_updates = int(
            self.declare_parameter("global_reroute_failure_updates", 3).value)
        if self.global_reroute_failure_updates < 1:
            raise ValueError("global_reroute_failure_updates must be at least one")
        self.map_update_period = float(self.declare_parameter("map_update_period", 0.25).value)
        if not math.isfinite(self.map_update_period) or self.map_update_period <= 0.0:
            raise ValueError("map_update_period must be finite and positive")
        self._pending_fusion_cloud = None
        self.planning_map_publish_period = float(
            self.declare_parameter("planning_map_publish_period", 0.25).value)
        self.goal_timeout = float(self.declare_parameter("goal_timeout", 120.0).value)
        self.path_request_timeout = float(self.declare_parameter(
            "path_request_timeout", 5.0).value)
        if self.path_request_timeout <= 0.0:
            raise ValueError("path_request_timeout must be positive")
        # A deferred reference request hands the decision back to SCAN: Local
        # is inside a recovery transaction that owns the reference path, so the
        # Explorer must not consume its own planner timeout while it waits. The
        # wait is still bounded: a lost or unmapped transaction end would
        # otherwise stop every later planning entry point for good, silently
        # (no status message, no error) and the robot would simply stand there.
        self.deferred_path_timeout = float(self.declare_parameter(
            "deferred_path_timeout", 15.0).value)
        if not math.isfinite(self.deferred_path_timeout) or self.deferred_path_timeout <= 0.0:
            raise ValueError("deferred_path_timeout must be positive")
        self.task_cancel_timeout = float(self.declare_parameter(
            "task_cancel_timeout", 10.0).value)
        if not math.isfinite(self.task_cancel_timeout) or self.task_cancel_timeout <= 0.0:
            raise ValueError("task_cancel_timeout must be finite and positive")
        self.blocked_edge_ttl = float(
            self.declare_parameter("blocked_edge_ttl", 30.0).value)
        self.frame_id = str(self.declare_parameter("frame_id", "world").value)
        self.auto_start = bool(self.declare_parameter("auto_start", True).value)
        self.selection_strategy = str(
            self.declare_parameter("selection_strategy", "hierarchical").value).lower()
        if self.selection_strategy not in ("greedy", "hierarchical"):
            raise ValueError("selection_strategy must be 'greedy' or 'hierarchical'")
        self.region_visit_window_enabled = bool(self.declare_parameter(
            "region_visit_window_enabled", True).value)
        self.region_visit_frontiers = int(self.declare_parameter(
            "region_visit_frontiers", 4).value)
        self.region_visit_stations = int(self.declare_parameter(
            "region_visit_stations", 2).value)
        self.region_visit_route_expansions = int(self.declare_parameter(
            "region_visit_route_expansions", 4096).value)
        if not (1 <= self.region_visit_frontiers <= 8 and
                1 <= self.region_visit_stations <= 3 and
                1 <= self.region_visit_route_expansions <= 20000):
            raise ValueError("Region visit budgets must remain bounded")
        self.region_visit_deferred = False
        self.last_visit_window = []
        self.last_visit_stats = {}
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
        self.region_min_remaining_cells = int(self.declare_parameter(
            "region_min_remaining_cells", 24).value)
        self.region_exhausted_revisions = int(self.declare_parameter(
            "region_exhausted_revisions", 3).value)
        self.region_stagnation_revisions = int(self.declare_parameter(
            "region_stagnation_revisions", 20).value)
        if self.region_min_remaining_cells < 0:
            raise ValueError("region_min_remaining_cells must be non-negative")
        if self.region_exhausted_revisions < 1:
            raise ValueError("region_exhausted_revisions must be positive")
        if self.region_stagnation_revisions < 1:
            raise ValueError("region_stagnation_revisions must be positive")
        self.metrics_period = float(self.declare_parameter("metrics_period", 2.0).value)
        self.metrics_file = str(self.declare_parameter("metrics_file", "").value)
        self.observation_preplanning_enabled = bool(
            self.declare_parameter("observation_preplanning_enabled", True).value)
        self.task_mode = bool(self.declare_parameter("task_mode", False).value)
        self.task_geometry_autonomy = bool(self.declare_parameter("task_geometry_autonomy", True).value)
        self.task_corridor_radius = float(self.declare_parameter("task_corridor_radius", 0.8).value)
        self.task_representative_limit = int(self.declare_parameter("task_representative_limit", 64).value)
        self.task_max_station_adjustments = int(self.declare_parameter("task_max_station_adjustments", 4).value)
        self.task_departure_retry_period = float(self.declare_parameter("task_departure_retry_period", 2.0).value)
        self.task_departure_retry_limit = int(self.declare_parameter("task_departure_retry_limit", 2).value)
        if (not math.isfinite(self.task_departure_retry_period) or self.task_departure_retry_period <= 0
                or not 0 <= self.task_departure_retry_limit <= 8):
            raise ValueError("departure recovery requires a positive period and 0..8 retries")
        if (not math.isfinite(self.task_corridor_radius) or not 0 < self.task_corridor_radius <= 10
                or not 1 <= self.task_representative_limit <= 256
                or not 1 <= self.task_max_station_adjustments <= 32):
            raise ValueError("invalid observation task bounds")
        self.last_observation_task_id = 0
        self.tasks_created = self.task_requests_published = self.task_cancellations = 0
        self.task_stations_reached = self.task_local_rejections = self.task_stale_feedback = 0
        self.task_feedback_seen = deque(maxlen=256)
        self.pending_observation_station = None
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
        # How many consecutive map updates the tracked frontier must stay absent
        # before its disappearance counts as a confirmed closure.  Deliberately
        # longer than observation_done_updates: this signal replaces the old
        # "progress >= observation_prepare_ratio" floor, which could never be
        # reached when part of the frozen target set was structurally
        # unobservable, and deadlocked the task while SCAN held the robot.
        self.observation_closure_updates = int(
            self.declare_parameter("observation_closure_updates", 10).value)
        self.min_expected_observation_cells = int(
            self.declare_parameter("min_expected_observation_cells", 8).value)
        self.max_replans_per_context = int(
            self.declare_parameter("max_replans_per_context", 2).value)
        self.replan_retry_period = max(0.1, float(
            self.declare_parameter("replan_retry_period", 2.0).value))
        self.global_repair_search_limit = max(1, int(
            self.declare_parameter("global_repair_search_limit", 4).value))
        self.global_repair_max_expansions = max(1, int(
            self.declare_parameter("global_repair_max_expansions", 400).value))
        self.region_max_path_ratio = float(
            self.declare_parameter("region_max_path_ratio", 1.5).value)
        self.region_max_detour_ratio = float(
            self.declare_parameter("region_max_detour_ratio", 1.75).value)
        self.global_usage_mode = str(self.declare_parameter(
            "global_usage_mode", "shadow").value)
        if self.global_usage_mode not in ("off", "shadow", "route_cost"):
            raise ValueError("global_usage_mode must be off, shadow or route_cost")
        sparse_requested = bool(self.declare_parameter("sparse_routing_enabled", True).value)
        self.sparse_routing_enabled = self.global_usage_mode == "route_cost" and sparse_requested
        self.sparse_attachment_radius = float(
            self.declare_parameter("sparse_attachment_radius", 3.0).value)
        self.sparse_attachment_limit = int(
            self.declare_parameter("sparse_attachment_limit", 8).value)
        self.safe_region_attachment_radius = float(self.declare_parameter(
            "safe_region_attachment_radius", 3.0).value)
        if self.sparse_attachment_radius <= 0.0:
            raise ValueError("sparse_attachment_radius must be positive")
        if self.sparse_attachment_limit < 1:
            raise ValueError("sparse_attachment_limit must be at least one")
        if self.safe_region_attachment_radius <= 0.0:
            raise ValueError("safe_region_attachment_radius must be positive")
        if self.terminal_candidate_limit < 1:
            raise ValueError("terminal_candidate_limit must be at least one")
        if self.terminal_clearance_search_radius <= 0.0:
            raise ValueError("terminal_clearance_search_radius must be positive")
        if self.terminal_approach_length <= 0.0:
            raise ValueError("terminal_approach_length must be positive")
        if not (0.0 <= self.observation_prepare_ratio
                < self.observation_done_ratio <= 1.0):
            raise ValueError(
                "observation ratios must satisfy 0 <= prepare < done <= 1")
        if self.observation_done_updates < 1:
            raise ValueError("observation_done_updates must be at least one")
        if self.observation_closure_updates < 1:
            raise ValueError("observation_closure_updates must be at least one")

        self.position: Optional[Point2] = None
        self.body_z = 0.3
        self.body_yaw = 0.0
        self.validation_measured_pose = None  # One timestamped odom sample; diagnostics only.
        # Map throttling and timestamped pose lookup are independent concerns.
        # Keep an explicit initial value so the first accepted cloud cannot
        # read this member before cloud_callback() has assigned it.
        self.last_map_update_ns = 0
        self.last_integrated_cloud_stamp_ns = 0
        self.pose_history = deque(maxlen=200)
        self.max_interpolation_gap_ns = int(float(self.declare_parameter(
            "max_interpolation_gap", 0.05).value) * 1e9)
        self.max_nearest_pose_age_ns = int(float(self.declare_parameter(
            "max_nearest_pose_age", 0.02).value) * 1e9)
        self.cloud_pose_wait_timeout_ns = int(float(self.declare_parameter(
            "cloud_pose_wait_timeout", 0.10).value) * 1e9)
        if (self.max_interpolation_gap_ns <= 0
                or self.max_nearest_pose_age_ns < 0
                or self.cloud_pose_wait_timeout_ns <= 0):
            raise ValueError("pose history timing parameters are invalid")
        self.pending_clouds = deque(maxlen=8)
        self.stall_detection_seconds = float(self.declare_parameter(
            "stall_detection_seconds", 30.0).value)
        self.map_update_count = 0
        self.map_content_revision = 0
        self.last_decision_context = None
        self.last_decision_retry_epoch = -1
        self.skipped_decision_ticks = 0
        self.last_planning_map_ns = None
        # Sparse terrain annotation: the largest measured rise and deepest
        # measured drop per cell, relative to the observed local ground. Kept
        # so a future cost or capability layer can consume them without
        # re-deriving from the raw cloud. This is representation only --
        # nothing here becomes occupancy or inflation.
        self.terrain_rises: Dict[Cell, float] = {}
        self.terrain_drops: Dict[Cell, float] = {}
        self.map_published = False
        self.changed_cells: Set[Cell] = set()
        self.route_constraint_revision = 0
        self.replan_gate = ReplanGate(self.max_replans_per_context)
        self.replan_suppressed_count = 0
        self.scan_blocked_count = 0
        self.scan_reference_rejected_count = 0
        self.goal_timeout_count = 0
        # Tasks whose motion was released on arrival without the information
        # criterion being met.  Counted separately so the "arrival-triggered"
        # and "information-gated" termination strategies can be compared on the
        # same run: a high count with little revisit growth favours arrival.
        self.observation_motion_released_count = 0
        self.last_observation_released_reason = "none"
        self.last_observation_released_progress = 0.0
        self.no_active_goal_replan_count = 0
        self.last_active_goal_clear_reason = "startup"
        self.active_goal: Optional[Point2] = None
        self.active_goal_cell: Optional[Cell] = None
        self.active_observation: Optional[ObservationTask] = None
        self.active_raw_path: List[Cell] = []
        self.active_path_progress_index = 0
        self.last_active_path_validation_update = -1
        self.prepared_candidate: Optional[FrontierCandidate] = None
        self.region_edge_cache: Dict[Tuple[int, int], List[Cell]] = {}
        self.region_anchor_cells: Dict[int, Cell] = {}
        self.sparse_router = SparseRouteGraph(
            bucket_size=max(self.grid.resolution, 1.0))
        self.topology_messages_received = 0
        self.topology_messages_applied = 0
        self.topology_messages_rejected = 0
        self.topology_snapshot_recovery_count = 0
        self.last_rejected_topology_revision = 0
        self.safe_region_connectivity = SafeRegionConnectivity()
        self.region_edge_cache_hits = 0
        self.region_edge_cache_misses = 0
        self.replacement_pending = False
        self.last_preparation_attempt_update = -1
        self.last_handoff_attempt_update = -1
        self.pending_observation_handoff: Optional[PendingObservationHandoff] = None
        self.unconfirmed_observation_handoff: Optional[PendingObservationHandoff] = None
        self.handoff_execution_retired = False
        self.handoff_excluded_goals: List[Point2] = []
        self.completion_handoff_in_progress = False
        self.pipeline_latency_ewma = 0.0
        self.pending_path_publish_ns = 0
        self.pending_path_deferred = False
        self.last_path_request_generation = 0
        self.pending_path_request_generation: Optional[int] = None
        self.active_path_request_generation: Optional[int] = None
        self.active_path_trajectory_id: Optional[int] = None
        self.last_completed_request_generation: Optional[int] = None
        self.scan_waiting_for_target = False
        self.pending_local_execution_finish = None
        self.local_execution_finish_events = 0
        self.local_execution_finish_reroutes = 0
        self.global_reroute_failure_streak = 0
        self.active_since_ns = 0
        self.blacklist: List[Point2] = []
        self.completed_goals: List[Point2] = []
        # goal -> (minimum map update, map content revision at failure,
        #          route constraint revision at failure).  A timer/update
        # count alone is not new reachability evidence: a stationary robot
        # keeps receiving scans even when the failed corridor is unchanged.
        self.goal_failure_cooldowns: Dict[
            Point2, Tuple[int, int, int]] = {}
        self.region_failure_cooldowns: Dict[int, Tuple[int, int]] = {}
        self.dense_invalid_region_revisions: Dict[int, int] = {}
        self.temporary_blocked_edges: Dict[DirectedEdge, int] = {}
        self.task_path_constraints: Dict[
            Tuple[int, int], Set[DirectedEdge]] = {}
        self.pending_blocked_edge: Optional[DirectedEdge] = None
        self.disabled_by_collision = False
        self.region_tracker = PersistentRegionTracker(
            self.region_match_distance / self.grid.resolution)
        self.region_sequence: List[int] = []
        self.region_prediction_input_signature = None
        self.active_region_id: Optional[int] = None
        self.active_commitment_scope: Optional[Tuple[str, int]] = None
        self.commitment_target_cells: Set[Cell] = set()
        self.branch_commitment = None
        self.branch_option_serial = 0
        self.branch_association_update = -1
        self.branch_unresolved_since = None
        self.branch_unresolved_updates = 0
        self.branch_progress_remaining = None
        self.branch_continuation_present = False
        self.branch_progress_time = time.monotonic()
        self.commitment_direction_origin: Optional[Point2] = None
        self.commitment_direction_vector: Optional[Point2] = None
        self.commitment_direction_progress = 0.0
        self.directional_recovery_pending = False
        self.active_region_missing_streak = 0
        self.active_region_unreachable_since: Optional[float] = None
        self.last_region_release_evaluation_update = -1
        self.last_selected_region_id: Optional[int] = None
        self.last_selected_commitment_scope: Optional[Tuple[str, int]] = None
        self.region_commitment_gate = ResidualCommitmentGate(
            self.region_min_remaining_cells,
            self.region_exhausted_revisions,
            self.region_stagnation_revisions)
        self.commitment_state = "UNCOMMITTED"
        self.commitment_remaining_cells = 0
        self.commitment_stagnant_revisions = 0
        self.commitment_release_count = 0
        self.commitment_switch_suppressed_count = 0
        self.last_commitment_release_reason = "none"
        self.last_region_switch_reason = "startup"
        self.last_selection_reason = "startup"
        self.last_global_plan_ms = 0.0
        self.cumulative_planning_ms = 0.0
        self.planning_events = 0
        self.map_preprocess_calls = 0
        self.candidate_tree_searches = 0
        self.region_pair_tree_searches = 0
        self.expanded_grid_cells = 0
        self.last_map_preprocess_ms = 0.0
        self.last_region_partition_ms = 0.0
        self.last_topology_attached_clusters = 0
        self.last_topology_unattached_clusters = 0
        self.last_candidate_tree_ms = 0.0
        self.last_region_sequence_ms = 0.0
        self.last_candidate_tree_searches = 0
        self.last_region_pair_tree_searches = 0
        self.last_expanded_grid_cells = 0
        self.last_region_edge_cache_hits = 0
        self.last_region_edge_cache_misses = 0
        self.last_sparse_candidate_ms = 0.0
        self.last_sparse_region_ms = 0.0
        self.last_sparse_candidate_hits = 0
        self.last_sparse_candidate_fallbacks = 0
        self.last_sparse_region_pair_hits = 0
        self.last_sparse_region_pair_fallbacks = 0
        self.sparse_candidate_queries = 0
        self.sparse_candidate_hits = 0
        self.sparse_candidate_fallbacks = 0
        self.sparse_region_pair_queries = 0
        self.sparse_region_pair_hits = 0
        self.sparse_region_pair_fallbacks = 0
        self.sparse_candidate_miss_reasons = dict.fromkeys(
            SPARSE_MISS_REASONS, 0)
        self.sparse_region_miss_reasons = dict.fromkeys(
            SPARSE_MISS_REASONS, 0)
        self.last_sparse_candidate_miss_reasons = dict.fromkeys(
            SPARSE_MISS_REASONS, 0)
        self.last_sparse_region_miss_reasons = dict.fromkeys(
            SPARSE_MISS_REASONS, 0)
        self.dense_final_validation_searches = 0
        self.sparse_final_validation_failures = 0
        self.global_path_invalid_count = 0
        self.region_switches = 0
        self.intra_scope_region_switches = 0
        self.local_repair_exhausted_count = 0
        self.failure_evidence_received = 0
        self.failure_evidence_matched = 0
        self.global_viewpoint_constraint_requests: Set[int] = set()
        self.global_viewpoint_constraints: Dict[int, Tuple[float, float, float]] = {}
        self.last_failure_stage = "none"
        self.last_failure_reason = "none"
        self.last_failure_clearance = 0.0
        self.directional_candidate_suppressed_count = 0
        self.directional_recovery_switches = 0
        self.directional_suppression_revision = -1
        self.directional_suppression_cells: Set[Tuple[int, Cell]] = set()
        self.goals_reached = 0
        self.paths_published = 0
        self.short_paths_published = 0
        self.last_published_path_length = 0.0
        self.observations_satisfied = 0
        self.last_observation_completion_reason = "none"
        self.last_observation_completion_progress = 0.0
        self.terminal_relocations = 0
        self.last_terminal_candidate_count = 0
        self.last_terminal_clearance = 0.0
        self.last_approach_clearance = 0.0
        self.total_distance = 0.0
        self.revisit_distance = 0.0
        self.last_metric_position: Optional[Point2] = None
        self.last_motion_cell: Optional[Cell] = None
        self.visited_motion_cells: Set[Cell] = set()
        self.run_start_time = time.monotonic()
        self.run_id = str(time.time_ns())
        self.latest_frontier_count = 0
        self.latest_region_count = 0
        self.locomotion_actuation_ready = False
        # Running-state accounting.  A stalled or manually interrupted run keeps
        # publishing metrics with zero information gain, so every cumulative
        # counter grows while the map does not.  Classify the run and report
        # ACTIVE-phase medians, so comparisons stop depending on how long the run
        # happened to survive before the stall.
        self.running_state = "startup"
        self.running_state_since = self.run_start_time
        self.last_state_known_cells = 0
        self.last_cell_growth_time = self.run_start_time
        self.active_duration = 0.0
        self.active_sample_count = 0
        self.active_sparse_hit_rates: List[float] = []
        self.active_planning_ms: List[float] = []
        self.last_metrics_time = self.run_start_time
        self.rate_prev_known_cells = 0
        self.rate_prev_distance = 0.0
        self.rate_prev_planning_ms = 0.0
        self.perception_pose_valid = False
        self.locomotion_epoch = 0
        self.locomotion_reason = ""
        self.map_fusion_started = False
        self.last_locomotion_state_ns = 0
        self.last_locomotion_state_stamp_ns = 0
        self.locomotion_state_max_age = float(self.declare_parameter(
            "locomotion_state_max_age", 1.0).value)
        if self.locomotion_state_max_age <= 0.0:
            raise ValueError("locomotion_state_max_age must be positive")
        self.last_task_clock_ns = self.get_clock().now().nanoseconds
        self.healthy_task_elapsed = 0.0
        self.active_task_elapsed = 0.0
        self.unreachable_context = None
        self.last_candidate_outcomes = []
        self._region_overlay_last_publish = 0.0
        self.last_candidate_rejections = dict.fromkeys((
            "no_safe_viewpoint", "insufficient_gain", "too_close",
            "blacklisted", "cooldown", "excluded", "astar_unreachable",
            "final_validation_failed", "accepted"), 0)

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            LaserScan if self.use_laser_scan else PointCloud2,
            "scan" if self.use_laser_scan else "cloud", self.cloud_callback, sensor_qos,
            callback_group=self.sensor_callback_group)
        self.create_subscription(
            Odometry, "body_pose", self.odom_callback, 10,
            callback_group=self.sensor_callback_group)
        self.create_subscription(
            String, "planning/status", self.planning_status_callback, 10,
            callback_group=self.planning_callback_group)
        if self.global_usage_mode == "route_cost":
            self.create_subscription(
                FailureEvidence, "global_representation/failure_evidence",
                self.failure_evidence_callback, 20,
                callback_group=self.planning_callback_group)
        self.create_subscription(
            TrajectoryAck, "planning/trajectory_ack",
            self.trajectory_ack_callback, 20,
            callback_group=self.planning_callback_group)
        self.create_subscription(
            String, "planning/local_execution_event",
            self.local_execution_event_callback, 10,
            callback_group=self.planning_callback_group)
        self.create_subscription(
            Path, "planning/blocked_segment", self.blocked_segment_callback, 10,
            callback_group=self.planning_callback_group)
        self.create_subscription(
            Bool, "simulation/collision", self.collision_callback, 10,
            callback_group=self.planning_callback_group)
        self.create_subscription(
            Bool, "explorer/enabled", self.enabled_callback, 10,
            callback_group=self.planning_callback_group)
        locomotion_qos = QoSProfile(depth=1)
        locomotion_qos.reliability = ReliabilityPolicy.RELIABLE
        locomotion_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            LocomotionState, "/robot/locomotion_state",
            self.locomotion_state_callback, locomotion_qos,
            callback_group=self.locomotion_callback_group)
        if self.global_usage_mode == "route_cost":
            self.create_subscription(
                TopoGraphDelta, "global_representation/topology_delta",
                self.topology_delta_callback, 10,
                callback_group=self.planning_callback_group)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        if self.global_usage_mode == "route_cost":
            self.create_subscription(
                TopoGraphDelta, "global_representation/topology_snapshot",
                self.topology_snapshot_callback, transient_qos,
                callback_group=self.planning_callback_group)
        if self.global_usage_mode == "route_cost":
            self.create_subscription(
                String, "global_representation/safe_region_snapshot",
                self.safe_region_snapshot_callback, transient_qos,
                callback_group=self.planning_callback_group)
        self.path_pub = self.create_publisher(Path, "initial_path", 10)
        self.observation_task_pub = self.create_publisher(TaskMessage, "observation_task", 20)
        self.create_subscription(ObservationFeedback, "planning/observation_feedback",
                                 self.observation_feedback_callback, 20,
                                 callback_group=self.planning_callback_group)
        self.path_vis_pub = self.create_publisher(Path, "explorer/path", transient_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, "explorer/map", transient_qos)
        self.map_updates_pub = self.create_publisher(
            OccupancyGridUpdate, "map_updates", transient_qos)
        self.planning_map_pub = self.create_publisher(
            OccupancyGrid, "explorer/planning_map", transient_qos)
        self.frontier_pub = self.create_publisher(Marker, "explorer/frontiers", transient_qos)
        self.goal_pub = self.create_publisher(Marker, "explorer/goal", transient_qos)
        self.region_overlay_pub = self.create_publisher(
            MarkerArray, "explorer/region_overlay", transient_qos)
        self.status_pub = self.create_publisher(String, "explorer/status", transient_qos)
        self.region_sequence_pub = self.create_publisher(
            String, "explorer/region_sequence", transient_qos)
        self.metrics_pub = self.create_publisher(String, "explorer/metrics", transient_qos)
        self.create_timer(
            self.map_update_period, self.fusion_timer,
            callback_group=self.sensor_callback_group)
        self.create_timer(
            self.planning_period, self.exploration_timer,
            callback_group=self.planning_callback_group)
        self.create_timer(
            self.metrics_period, self.metrics_timer,
            callback_group=self.planning_callback_group)
        self.publish_status("WAITING_FOR_LOCAL_MAP")
        self.get_logger().info(
            f"Explorer strategy={self.selection_strategy}, region_size={self.region_size:.1f} m, "
            f"observation_preplanning={self.observation_preplanning_enabled}, "
            f"observation_prepare={self.observation_prepare_ratio:.2f}, "
            f"observation_done={self.observation_done_ratio:.2f}, "
            f"sparse_routing={self.sparse_routing_enabled} "
            f"global_usage_mode={self.global_usage_mode} "
            f"region_visit_window={self.bounded_visits_enabled()}")

    def locomotion_state_callback(self, msg: LocomotionState):
        self.locomotion_actuation_ready = msg.actuation_ready
        self.perception_pose_valid = msg.perception_pose_valid
        self.locomotion_epoch = msg.epoch
        self.locomotion_reason = msg.reason
        self.last_locomotion_state_ns = self.get_clock().now().nanoseconds
        self.last_locomotion_state_stamp_ns = (
            int(msg.stamp.sec) * 1000000000 + int(msg.stamp.nanosec))

    def trajectory_ack_callback(self, msg: TrajectoryAck):
        """Track only activated motion for the exact current task request."""
        task = getattr(self, "active_observation", None)
        if (msg.status != TrajectoryAck.STATUS_ACTIVATED
                or msg.trajectory_id <= 0
                or msg.request_id != self.active_path_request_generation
                or task is None
                or msg.task_id != task.task_id
                or msg.task_version != task.task_version):
            return
        current = getattr(self, "active_path_trajectory_id", None)
        if current is None or msg.trajectory_id > current:
            self.active_path_trajectory_id = int(msg.trajectory_id)
        self.record_route_source_activation(task, msg.request_id)

    def record_route_source_activation(self, task, request_id):
        """Count exact activated requests once, grouped by proposal source.

        Local may generate its own geometry; activation is conversion to a
        real trajectory, not proof of following every Global graph edge.
        """
        seen = getattr(self, "route_source_activation_seen", None)
        if seen is None:
            seen = self.route_source_activation_seen = deque(maxlen=256)
            self.route_source_activated_requests = {}
        key = (task.task_id, task.task_version, int(request_id))
        if key in seen:
            return
        seen.append(key)
        source = getattr(task, "route_source", "dense")
        counts = self.route_source_activated_requests
        counts[source] = counts.get(source,0)+1

    def failure_evidence_callback(self, msg: FailureEvidence):
        """Apply graph-mediated evidence without taking terminal ownership.

        The matching terminal planning/status message remains the sole state
        transition authority.  Viewpoint-scoped evidence may immediately
        change route scoring/cooldown, but it never clears the active request.
        """
        if getattr(self, "global_usage_mode", "route_cost") != "route_cost":
            return
        self.failure_evidence_received += 1
        if getattr(self, "task_mode", False):
            task = self.active_observation
            if task is None or msg.task_id != task.task_id or msg.task_version != task.task_version:
                return
        if (self.active_path_request_generation is None
                or int(msg.request_id) !=
                int(self.active_path_request_generation)):
            return
        self.failure_evidence_matched += 1
        self.last_failure_stage = str(msg.stage)
        self.last_failure_reason = str(msg.reason)
        self.last_failure_clearance = float(msg.required_clearance)
        failure_scope = int(getattr(
            msg, "failure_scope", FailureEvidence.SCOPE_PATH))
        accepted_trajectory = getattr(
            self, "active_path_trajectory_id", None)
        if (failure_scope == FailureEvidence.SCOPE_PATH
                and getattr(self, "task_mode", False)
                and accepted_trajectory is not None
                and int(msg.trajectory_id) < accepted_trajectory):
            return
        if failure_scope == FailureEvidence.SCOPE_VIEWPOINT:
            self.apply_global_viewpoint_constraint(
                int(msg.request_id),
                (float(msg.position.x), float(msg.position.y)),
                self.blacklist_radius,
                str(msg.reason))
        elif failure_scope == FailureEvidence.SCOPE_PATH and getattr(
                self, "task_mode", False):
            if not getattr(self,"task_geometry_autonomy",False):
                self.record_task_path_constraint(msg)
        self.get_logger().warn(
            "[LOCAL_FAILURE_EVIDENCE] "
            f"request_id={msg.request_id} trajectory={msg.trajectory_id} "
            f"stage={msg.stage} reason={msg.reason} "
            f"attempts={msg.attempted_candidates} "
            f"clearance={msg.required_clearance:.3f}m")

    def record_task_path_constraint(self, msg: FailureEvidence) -> bool:
        """Constrain one failed directed edge for this task version only."""
        # A bad spline or an inadmissible controller motion is not a blocked
        # road. Keep its diagnostics, but do not turn candidate failures into
        # persistent global geometry (including messages from older binaries).
        if getattr(msg, "stage", "") in (
                "LOCAL_PATH_SHAPE", "FORWARD_GAIT_STEERING"):
            return False
        task = self.active_observation
        if (task is None or not getattr(msg, "has_path_segment", False)
                or int(msg.trajectory_id) <= 0
                or len(self.active_raw_path) < 2):
            return False
        start, end = msg.path_start, msg.path_end
        if not all(math.isfinite(value) for value in (
                start.x, start.y, start.z, end.x, end.y, end.z)):
            return False
        edge = match_blocked_path_edge(
            self.active_raw_path,
            self.grid.world_to_cell(start.x, start.y),
            self.grid.world_to_cell(end.x, end.y))
        if edge is None:
            return False
        key = (int(task.task_id), int(task.task_version))
        constraints = getattr(self, "task_path_constraints", None)
        if constraints is None:
            constraints = {}
            self.task_path_constraints = constraints
        edges = constraints.setdefault(key, set())
        if edge in edges:
            return False
        edges.add(edge)
        self.route_constraint_revision += 1
        self.get_logger().warning(
            f"[TASK_PATH_CONSTRAINT] revision={self.route_constraint_revision} "
            f"task_id={task.task_id} task_version={task.task_version} "
            f"request_id={msg.request_id} trajectory_id={msg.trajectory_id} "
            f"edge={edge[0]}->{edge[1]}")
        return True

    def revoke_task_path_constraints(self, task) -> bool:
        """Revoke a task's path edges when its objective identity ends."""
        if (task is None or not hasattr(task, "task_id")
                or not hasattr(task, "task_version")):
            return False
        key = (int(task.task_id), int(task.task_version))
        constraints = getattr(self, "task_path_constraints", {})
        if constraints.pop(key, None) is None:
            return False
        self.route_constraint_revision += 1
        return True

    def locomotion_state_age_ns(self) -> Optional[int]:
        """Return publisher age, falling back to local receipt for old bags."""
        if self.last_locomotion_state_ns == 0:
            return None
        now_ns = self.get_clock().now().nanoseconds
        reference_ns = (self.last_locomotion_state_stamp_ns
                        if self.last_locomotion_state_stamp_ns > 0
                        else self.last_locomotion_state_ns)
        # Clock resets can occur when a simulation restarts.  A future stamp is
        # not stale; the next state sample will re-establish the epoch.
        return max(0, now_ns - reference_ns)

    def locomotion_state_is_fresh(self) -> bool:
        age_ns = self.locomotion_state_age_ns()
        return (age_ns is not None
                and age_ns <= int(self.locomotion_state_max_age * 1e9))

    def locomotion_ready(self) -> bool:
        return self.locomotion_actuation_ready and self.locomotion_state_is_fresh()

    def map_fusion_allowed(self) -> bool:
        """Whether a scan may be integrated into the occupancy grid.

        Only map fusion is gated here, never motion.  A scan taken while the
        robot is still lying, is getting up, or has lost its body pose gets
        projected through a planar pose that does not describe it, and because
        a ray endpoint is only ever added as occupied evidence, a single such
        scan can seal free space around the robot permanently.  Without this
        gate the grid is built from the first frame that has odometry -- several
        seconds before the stance exists.

        ``perception_pose_valid`` is the supervisor's map-fusion contract.
        ``actuation_ready`` answers a different question (may motion commands be
        accepted) and is only equal to it by accident in the current supervisor.
        """
        if not self.locomotion_state_is_fresh():
            return False
        return self.perception_pose_valid

    def report_fusion_blocked(self):
        age_ns = self.locomotion_state_age_ns()
        if not self.locomotion_state_is_fresh():
            age_detail = ("never received" if age_ns is None
                          else f"age={age_ns * 1e-9:.3f}s")
            detail = ("no fresh /robot/locomotion_state "
                      f"({age_detail}, limit={self.locomotion_state_max_age:.3f}s)")
        else:
            detail = (f"perception_pose_valid=false "
                      f"(supervisor reason={self.locomotion_reason})")
        self.get_logger().info(
            f"[MAP_FUSION_GATED] occupancy integration paused: {detail}; "
            "scans taken through an unverified stance would seal free space",
            throttle_duration_sec=5.0)

    def publish_status(self, value: str):
        msg = String()
        msg.data = value
        self.status_pub.publish(msg)

    def safe_region_snapshot_callback(self, msg: String):
        if getattr(self, "global_usage_mode", "route_cost") != "route_cost":
            return
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = None
        if isinstance(payload, dict):
            supports = payload.get("spatial_supports")
            if isinstance(supports, dict):
                try:
                    if self.sparse_router.update_supports(supports):
                        self._global_space_supports = supports
                except (TypeError, ValueError, KeyError) as error:
                    self.get_logger().warning(f"Rejected spatial support snapshot: {error}")
            for item in payload.get("viewpoint_constraints", ()):
                try:
                    self.apply_global_viewpoint_constraint(
                        int(item["request_id"]),
                        (float(item["x"]), float(item["y"])),
                        float(item.get("radius", self.blacklist_radius)),
                        str(item.get("reason", "GLOBAL_GRAPH_SNAPSHOT")))
                except (KeyError, TypeError, ValueError):
                    self.get_logger().warning(
                        "Rejected malformed viewpoint constraint",
                        throttle_duration_sec=2.0)
        if not self.safe_region_connectivity.update_json(msg.data):
            self.get_logger().warning(
                "Rejected malformed or stale safe-region snapshot",
                throttle_duration_sec=2.0)

    def apply_global_viewpoint_constraint(
            self, request_id: int, point: Point2, radius: float,
            reason: str) -> bool:
        """Install one graph-owned rejected-viewpoint constraint exactly once."""
        if getattr(self, "global_usage_mode", "route_cost") != "route_cost":
            return False
        requests = getattr(self, "global_viewpoint_constraint_requests", None)
        if requests is None:
            requests = set()
            self.global_viewpoint_constraint_requests = requests
        constraints = getattr(self, "global_viewpoint_constraints", None)
        if constraints is None:
            constraints = {}
            self.global_viewpoint_constraints = constraints
        if (request_id <= 0 or request_id in requests
                or radius <= 0.0
                or not all(math.isfinite(value)
                           for value in (point[0], point[1], radius))):
            return False
        requests.add(request_id)
        constraints[request_id] = (
            float(point[0]), float(point[1]), float(radius))
        self.route_constraint_revision += 1
        if (self.active_path_request_generation == request_id
                and self.active_goal is not None):
            self.add_goal_failure_cooldown(
                self.active_goal, "GLOBAL_GRAPH_VIEWPOINT_CONSTRAINT")
        self.get_logger().warning(
            "[GLOBAL_VIEWPOINT_CONSTRAINT_APPLIED] "
            f"revision={self.route_constraint_revision} "
            f"request_id={request_id} point=({point[0]:.2f},{point[1]:.2f}) "
            f"radius={radius:.2f}m reason={reason}")
        return True

    def attach_safe_region_commitment_scopes(
            self, candidates: Sequence[FrontierCandidate],
            inflated: Set[Cell]) -> None:
        """Promote only unambiguous, known-free safe-region attachments."""
        if getattr(self, "global_usage_mode", "route_cost") != "route_cost":
            return
        by_region: Dict[int, List[FrontierCandidate]] = {}
        for candidate in candidates:
            by_region.setdefault(candidate.region_id, []).append(candidate)

        def connector_allowed(source: Point2, target: Point2) -> bool:
            return segment_known_free(
                self.grid, self.grid.world_to_cell(*source),
                self.grid.world_to_cell(*target), inflated)

        for options in by_region.values():
            attachments = [
                self.safe_region_connectivity.region_for_point(
                    self.grid.cell_to_world(candidate.cell),
                    self.safe_region_attachment_radius, connector_allowed)
                for candidate in options]
            attached = {value for value in attachments if value is not None}
            if len(attached) != 1 or any(value is None for value in attachments):
                continue
            scope = ("safe_region", int(next(iter(attached))))
            for candidate in options:
                candidate.commitment_scope = scope

    @staticmethod
    def sparse_node_from_message(message) -> SparseNode:
        return SparseNode(
            int(message.id),
            (float(message.position.x), float(message.position.y)),
            float(message.clearance),
            float(getattr(message, "traversability", 1.0)),
            int(getattr(message, "terrain_state", 0)))

    @staticmethod
    def sparse_edge_from_message(message) -> SparseEdge:
        return SparseEdge(
            int(message.id), int(message.source_id), int(message.target_id),
            float(message.length), float(message.minimum_clearance),
            tuple((float(point.x), float(point.y))
                  for point in message.polyline),
            float(getattr(message, "traversability", 1.0)),
            int(getattr(message, "terrain_state", 0)),
            int(getattr(message, "traversals", 0)),
            float(getattr(message, "experience_score", 0.0)))

    def topology_delta_callback(self, msg: TopoGraphDelta):
        """Keep a lightweight query index of the independently maintained graph."""
        if (getattr(self, "global_usage_mode", "route_cost") != "route_cost"
                or not self.sparse_routing_enabled):
            return
        self.topology_messages_received += 1
        revision_before = self.sparse_router.graph_revision
        try:
            self.sparse_router.apply_delta(
                msg.graph_revision, msg.source_map_revision,
                [self.sparse_node_from_message(node) for node in msg.added_nodes],
                [self.sparse_node_from_message(node) for node in msg.updated_nodes],
                msg.removed_node_ids,
                [self.sparse_edge_from_message(edge) for edge in msg.added_edges],
                [self.sparse_edge_from_message(edge) for edge in msg.updated_edges],
                msg.removed_edge_ids)
        except Exception as error:  # ROS callback boundary: preserve the executor.
            self.topology_messages_rejected += 1
            self.last_rejected_topology_revision = int(msg.graph_revision)
            self.get_logger().error(
                "[SPARSE_GRAPH_REJECTED] "
                f"kind=delta revision={msg.graph_revision} reason={error}\n"
                f"{traceback.format_exc()}",
                throttle_duration_sec=1.0)
            return
        if self.sparse_router.graph_revision != revision_before:
            self.topology_messages_applied += 1
        self.get_logger().debug(
            f"[SPARSE_GRAPH_SYNC] revision={self.sparse_router.graph_revision} "
            f"nodes={len(self.sparse_router.nodes)} "
            f"edges={len(self.sparse_router.edges)}")

    def topology_snapshot_callback(self, msg: TopoGraphDelta):
        """Atomically restore a complete graph after launch races or restarts."""
        if (getattr(self, "global_usage_mode", "route_cost") != "route_cost"
                or not self.sparse_routing_enabled):
            return
        self.topology_messages_received += 1
        if msg.graph_revision < self.sparse_router.graph_revision:
            return
        try:
            replacement = SparseRouteGraph(
                bucket_size=max(self.grid.resolution, 1.0))
            replacement.apply_delta(
                msg.graph_revision, msg.source_map_revision,
                [self.sparse_node_from_message(node) for node in msg.added_nodes],
                [], [],
                [self.sparse_edge_from_message(edge) for edge in msg.added_edges],
                [], [])
        except Exception as error:  # Keep the last complete graph on bad input.
            self.topology_messages_rejected += 1
            self.last_rejected_topology_revision = int(msg.graph_revision)
            self.get_logger().error(
                "[SPARSE_GRAPH_REJECTED] "
                f"kind=snapshot revision={msg.graph_revision} reason={error}\n"
                f"{traceback.format_exc()}",
                throttle_duration_sec=1.0)
            return
        if getattr(self, "_global_space_supports", None) is not None:
            replacement.update_supports(self._global_space_supports)
        self.sparse_router = replacement
        self.topology_messages_applied += 1
        self.topology_snapshot_recovery_count += 1
        self.get_logger().info(
            f"[SPARSE_GRAPH_SNAPSHOT] revision={replacement.graph_revision} "
            f"nodes={len(replacement.nodes)} edges={len(replacement.edges)}",
            throttle_duration_sec=5.0)

    def sparse_route_estimates(
            self, start: Cell, targets: Sequence[Cell], inflated: Set[Cell],
            blocked_edges: Set[DirectedEdge]
            ) -> Tuple[List[Optional[SparseRouteEstimate]],
                       List[Optional[str]]]:
        """Estimate one-to-many routes, retaining dense safety at both connectors."""
        if not targets:
            return [], []
        if blocked_edges:
            return ([None] * len(targets),
                    ["blocked_disabled"] * len(targets))
        if (getattr(self, "global_usage_mode", "route_cost") != "route_cost"
                or not self.sparse_routing_enabled or not self.sparse_router.ready):
            return ([None] * len(targets),
                    ["start_unattached"] * len(targets))

        def connector_allowed(source: Point2, target: Point2) -> bool:
            source_cell = self.grid.world_to_cell(*source)
            target_cell = self.grid.world_to_cell(*target)
            return (self.grid.in_bounds(source_cell)
                    and self.grid.in_bounds(target_cell)
                    and segment_known_free(
                        self.grid, source_cell, target_cell, inflated)
                    and segment_known_free(
                        self.grid, target_cell, source_cell, inflated))

        return self.sparse_router.batch_estimates_with_reasons(
            self.grid.cell_to_world(start),
            [self.grid.cell_to_world(target) for target in targets],
            self.sparse_attachment_radius, self.sparse_attachment_limit,
            connector_allowed)

    @staticmethod
    def accumulate_sparse_miss_reasons(
            reasons: Sequence[Optional[str]], totals: Dict[str, int],
            latest: Dict[str, int]):
        """Accumulate one mutually exclusive reason for every sparse miss."""
        for reason in reasons:
            if reason is None:
                continue
            if reason not in totals:
                reason = "disconnected"
            totals[reason] += 1
            latest[reason] += 1

    def ensure_sparse_miss_counters(self):
        """Initialize diagnostics for lightweight test and replay instances."""
        if not hasattr(self, "sparse_candidate_miss_reasons"):
            self.sparse_candidate_miss_reasons = dict.fromkeys(
                SPARSE_MISS_REASONS, 0)
        if not hasattr(self, "sparse_region_miss_reasons"):
            self.sparse_region_miss_reasons = dict.fromkeys(
                SPARSE_MISS_REASONS, 0)
        if not hasattr(self, "last_sparse_candidate_miss_reasons"):
            self.last_sparse_candidate_miss_reasons = dict.fromkeys(
                SPARSE_MISS_REASONS, 0)
        if not hasattr(self, "last_sparse_region_miss_reasons"):
            self.last_sparse_region_miss_reasons = dict.fromkeys(
                SPARSE_MISS_REASONS, 0)

    def odom_callback(self, msg: Odometry):
        self.validation_measured_pose = None
        self.record_departure_motion(
            math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
            msg.twist.twist.angular.z)
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
        orientation = msg.pose.pose.orientation
        self.body_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z))
        stamp = msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns > 0:
            quaternion = np.array([
                orientation.x, orientation.y, orientation.z, orientation.w],
                dtype=np.float64)
            quaternion_norm = float(np.linalg.norm(quaternion))
            if not math.isfinite(quaternion_norm) or quaternion_norm < 1e-12:
                return
            quaternion /= quaternion_norm
            if all(math.isfinite(value) for value in new_position):
                qx, qy, qz, qw = quaternion
                measured_yaw = math.atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz))
                # Publish one immutable tuple, not an old planning position
                # paired with a newer live body_yaw from another callback.
                self.validation_measured_pose = (stamp_ns, *new_position, measured_yaw)
            sample: PoseSample = (
                stamp_ns, new_position[0], new_position[1], self.body_z,
                float(quaternion[0]), float(quaternion[1]),
                float(quaternion[2]), float(quaternion[3]))
            if not self.pose_history or stamp_ns >= self.pose_history[-1][0]:
                self.pose_history.append(sample)
            else:
                ordered = list(self.pose_history)
                ordered.append(sample)
                ordered.sort(key=lambda item: item[0])
                self.pose_history = deque(ordered[-self.pose_history.maxlen:],
                                          maxlen=self.pose_history.maxlen)
            self.drain_pending_clouds()

    @staticmethod
    def cloud_stamp_ns(msg: PointCloud2) -> int:
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def cloud_pose_wait_expired(self, cloud_stamp_ns: int) -> bool:
        return bool(
            self.pose_history
            and self.pose_history[-1][0] - cloud_stamp_ns
            > self.cloud_pose_wait_timeout_ns)

    def defer_cloud_for_pose(self, msg: PointCloud2):
        if len(self.pending_clouds) == self.pending_clouds.maxlen:
            self.pending_clouds.popleft()
            self.get_logger().warning(
                "Dropping oldest queued cloud while waiting for timestamped pose",
                throttle_duration_sec=2.0)
        self.pending_clouds.append(msg)

    def drain_pending_clouds(self):
        if not self.pending_clouds:
            return
        retained = deque(maxlen=self.pending_clouds.maxlen)
        while self.pending_clouds:
            msg = self.pending_clouds.popleft()
            stamp_ns = self.cloud_stamp_ns(msg)
            pose = interpolate_se3_pose(
                self.pose_history, stamp_ns,
                self.max_interpolation_gap_ns, self.max_nearest_pose_age_ns)
            if pose is not None:
                self.cloud_callback(msg)
            elif self.cloud_pose_wait_expired(stamp_ns):
                self.get_logger().warning(
                    "Dropping queued cloud after pose wait timeout",
                    throttle_duration_sec=2.0)
            else:
                retained.append(msg)
        self.pending_clouds = retained

    def current_blocked_edges(self) -> Set[DirectedEdge]:
        now_ns = self.get_clock().now().nanoseconds
        expired = [edge for edge, expiry in self.temporary_blocked_edges.items()
                   if expiry <= now_ns]
        for edge in expired:
            del self.temporary_blocked_edges[edge]
        if expired:
            self.route_constraint_revision += 1
        blocked = set(self.temporary_blocked_edges)
        task = getattr(self, "active_observation", None)
        if task is not None and getattr(self, "task_mode", False):
            blocked.update(getattr(self, "task_path_constraints", {}).get(
                (int(task.task_id), int(task.task_version)), ()))
        return blocked

    def blocked_segment_callback(self, msg: Path):
        if getattr(self, "task_mode", False):
            return
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
        self.route_constraint_revision += 1
        self.pending_blocked_edge = edge
        self.publish_status("LOCAL_BLOCKED_EDGE_RECORDED")
        self.get_logger().warning(
            f"Temporarily blocked directed edge {edge[0]} -> {edge[1]} "
            f"for {self.blocked_edge_ttl:.1f}s")

    def deferred_wait_expired(self) -> bool:
        """True once a deferred reference request has waited past its bound.

        The timer skips its own planner timeout while the deferral is active;
        without this bound a lost transaction-end message would skip every
        later planning entry point silently and forever.
        """
        if not self.pending_path_publish_ns or not self.pending_path_deferred:
            return False
        return ((self.get_clock().now().nanoseconds
                 - self.pending_path_publish_ns) * 1e-9
                >= self.deferred_path_timeout)

    def cloud_callback(self, msg: PointCloud2):
        """Keep the latest scan; fuse on the spatial timer, not a 10 Hz throttle.

        A 0.25 s throttle applied to 0.10 s callbacks actually runs every 0.30 s.
        The timer samples the latest *new* scan and never counts a replay as
        independent evidence. Pose wait retries enter the same bounded slot.
        """
        stats = self.fusion_stats()
        stats["clouds"] += 1
        if msg.header.frame_id != self.cloud_frame_id:
            stats["wrong_frame"] += 1
            self.get_logger().error(
                f"[SENSOR_CONTRACT_VIOLATION] dropping cloud in frame "
                f"'{msg.header.frame_id}', expected '{self.cloud_frame_id}'",
                throttle_duration_sec=2.0)
            return
        stamp = self.cloud_stamp_ns(msg)
        now_ns = self.get_clock().now().nanoseconds
        reset = now_ns < self.last_map_update_ns
        if not reset and stamp <= getattr(self, "last_integrated_cloud_stamp_ns", 0):
            stats["dropped_stamp"] += 1
            return
        pending = getattr(self, "_pending_fusion_cloud", None)
        if reset or pending is None or stamp > self.cloud_stamp_ns(pending):
            self._pending_fusion_cloud = msg
            stats["queued"] += 1

    def fusion_stats(self):
        if not hasattr(self, "_fusion_stats"):
            self._fusion_stats = dict(
                clouds=0, queued=0, integrated=0, dropped_stamp=0,
                wrong_frame=0, no_planar=0, deferred=0, dropped_pose=0,
                gated=0, no_grid_change=0)
        return self._fusion_stats

    @staticmethod
    def _validate_obstacle_band(relative: bool, minimum: float, maximum: float) -> None:
        """Validate coordinates, not an assumed sensor height of zero.

        A positive body-relative floor can contain a raised sensor. Whether a
        beam clears is decided using its capture-time origin and clearing band.
        """
        if not (math.isfinite(minimum) and math.isfinite(maximum)) or maximum <= minimum:
            raise ValueError("obstacle band must have finite bounds with max > min")

    @staticmethod
    def fusion_window_stalled(stats: dict, integrated_at_window_start: int,
                              clouds_at_window_start: int = 0) -> bool:
        """New clouds without integration; cumulative history is not input."""
        return (stats.get("clouds", 0) > clouds_at_window_start
                and stats.get("integrated", 0) == integrated_at_window_start)

    def report_fusion_stats(self, why=""):
        """One line per throttle window: which path is consuming the scans.

        The fusion path has several silent early returns. Without a counter a
        stalled map looks exactly like a still robot.
        """
        stats = self.fusion_stats()
        logger = getattr(self, "_logger", None)
        if logger is None:  # construction-time / unit-test probe: no ROS logger
            return
        # A window that consumed clouds but produced no integration is a stall,
        # not a quiet robot: the map cannot grow without it. Name the dominant
        # consumer so a field log says which path is eating the scans.
        stalled = self.fusion_window_stalled(
            stats, getattr(self, "_fusion_window_integrated", 0),
            getattr(self, "_fusion_window_clouds", 0))
        self._fusion_window_integrated = stats["integrated"]
        self._fusion_window_clouds = stats["clouds"]
        if stalled:
            logger.warning(
                f"[FUSION_STALLED]{why} no scan was integrated in the last window: "
                f"clouds={stats['clouds']} queued={stats['queued']} "
                f"integrated={stats['integrated']} "
                f"no_planar={stats['no_planar']} deferred={stats['deferred']} "
                f"dropped_stamp={stats['dropped_stamp']} gated={stats['gated']} "
                f"dropped_pose={stats['dropped_pose']}; the occupancy map is not growing")
        logger.info(
            f"[FUSION_STATS]{why} clouds={stats['clouds']} "
            f"queued={stats['queued']} integrated={stats['integrated']} "
            f"no_grid_change={stats['no_grid_change']} "
            f"dropped_stamp={stats['dropped_stamp']} "
            f"no_planar={stats['no_planar']} deferred={stats['deferred']} "
            f"dropped_pose={stats['dropped_pose']} gated={stats['gated']} "
            f"wrong_frame={stats['wrong_frame']} "
            f"known_cells={int(np.count_nonzero(self._live_grid.data != UNKNOWN))} "
            f"revision={self._live_map_content_revision}")

    def fusion_timer(self):
        msg = self._pending_fusion_cloud
        self._pending_fusion_cloud = None
        if msg is not None:
            self.integrate_cloud(msg)
        now = time.monotonic()
        if now - getattr(self, "_fusion_stats_wall", 0.0) >= 2.0:
            self._fusion_stats_wall = now
            self.report_fusion_stats()

    def integrate_cloud(self, msg: PointCloud2):
        stats = self.fusion_stats()
        if self.position is None:
            stats["deferred"] += 1
            return
        if msg.header.frame_id != self.cloud_frame_id:
            self.fusion_stats()["wrong_frame"] += 1
            self.get_logger().error(
                f"[SENSOR_CONTRACT_VIOLATION] dropping cloud in frame "
                f"'{msg.header.frame_id}', expected '{self.cloud_frame_id}'",
                throttle_duration_sec=2.0)
            return
        if not self.map_fusion_allowed():
            self.fusion_stats()["gated"] += 1
            self.report_fusion_blocked()
            return
        if not self.map_fusion_started:
            self.map_fusion_started = True
            self.get_logger().info(
                "[MAP_FUSION_STARTED] perception pose is valid; occupancy "
                "integration begins now, so the grid holds only scans taken "
                "from a verified stance")
        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.last_map_update_ns:
            # A simulation clock reset starts a new scan timestamp epoch.
            # Sensor revisions remain monotonic; old task evidence is not
            # relabelled. Do not permanently block fusion on the old stamp.
            self.last_integrated_cloud_stamp_ns = 0
            self.last_map_update_ns = 0
        cloud_stamp_ns = self.cloud_stamp_ns(msg)
        if cloud_stamp_ns <= getattr(self, "last_integrated_cloud_stamp_ns", 0):
            self.fusion_stats()["dropped_stamp"] += 1
            return  # Replayed/stale scans are not independent sensor evidence.
        pose_at_scan = interpolate_se3_pose(
            self.pose_history, cloud_stamp_ns,
            self.max_interpolation_gap_ns, self.max_nearest_pose_age_ns)
        if pose_at_scan is None:
            self.fusion_stats()["deferred"] += 1
            if self.cloud_pose_wait_expired(cloud_stamp_ns):
                self.get_logger().warning(
                    "Dropping cloud after pose wait timeout",
                    throttle_duration_sec=2.0)
            else:
                self.defer_cloud_for_pose(msg)
            return
        identified_scan = isinstance(msg, LaserScan)
        if identified_scan and (not msg.ranges or msg.time_increment != 0.):
            # Gazebo GPU rays share one capture pose. Sequential scanners need
            # per-beam pose synchronization, not this simultaneous-scan path.
            return
        identified_cloud = bool(getattr(self, "use_ray_cloud", False)) and not identified_scan
        if identified_cloud:
            fields = {field.name: field for field in msg.fields}
            if ("return_type" not in fields or fields["return_type"].datatype != 2
                    or fields["return_type"].count != 1):
                self.get_logger().warning("Dropping ray cloud without UINT8 return_type",
                                          throttle_duration_sec=2.)
                return
            points = point_cloud2.read_points(
                msg, field_names=("x","y","z","return_type"), skip_nans=False).reshape(-1)
            array = np.column_stack((points["x"],points["y"],points["z"]))
            return_types = points["return_type"]
        elif not identified_scan:
            xyz_fields = tuple(
                field for field in msg.fields if field.name in ("x", "y", "z"))
            uniform_xyz = (len(xyz_fields) == 3
                           and len({field.datatype for field in xyz_fields}) == 1
                           and len({field.datatype for field in msg.fields}) == 1)
            if uniform_xyz:
                points = point_cloud2.read_points_numpy(
                    msg, field_names=("x", "y", "z"), skip_nans=True)
                array = np.asarray(points, dtype=np.float64).reshape((-1, 3))
            else:
                # Gazebo PointCloudPacked adds heterogeneous metadata fields.  The
                # ROS helper's NumPy fast path rejects the complete schema even
                # when only x/y/z are requested, so select those structured fields
                # explicitly before forming an ordinary XYZ matrix.
                points = point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True)
                if points.size == 0:
                    return
                array = np.column_stack(
                    (points["x"], points["y"], points["z"])).astype(
                        np.float64, copy=False)
            if points.size == 0:
                return
        scan_x, scan_y, scan_z, qx, qy, qz, qw = pose_at_scan
        body_rotation = quaternion_rotation_matrix((qx, qy, qz, qw))
        if body_rotation is None:
            self.get_logger().warning(
                "Dropping cloud with an invalid interpolated orientation",
                throttle_duration_sec=2.0)
            return
        sensor_rotation = body_rotation
        sensor_position = np.array(
            [scan_x, scan_y, scan_z], dtype=np.float64)
        if self.need_extrinsic:
            sensor_position += body_rotation @ self.lidar_extrinsic_translation
            sensor_rotation = body_rotation @ self.lidar_extrinsic_rotation
        if (not np.all(np.isfinite(sensor_position))
                or not np.all(np.isfinite(sensor_rotation))):
            self.get_logger().warning(
                "Dropping cloud with a non-finite time-aligned sensor pose",
                throttle_duration_sec=2.0)
            return
        ray_x, ray_y = sensor_position[:2]
        z_reference = scan_z if self.obstacle_z_relative_to_body else 0.0
        clearing_ranges = None
        ground_cells, obstacle_cells = set(), set()
        if identified_scan:
            try:
                nearest, clearing_ranges = classify_planar_scan(
                    msg.ranges, msg.angle_min, msg.angle_increment,
                    msg.range_min, msg.range_max, sensor_position, sensor_rotation,
                    z_reference + self.obstacle_min_z, z_reference + self.obstacle_max_z,
                    self.mapping_range, self.no_return_range, self.ray_count,
                    self.hit_dilation_bins, self._live_grid.resolution)
            except ValueError as error:
                self.get_logger().warning(f"Dropping invalid laser scan: {error}",
                                          throttle_duration_sec=2.)
                return
        elif identified_cloud:
            try:
                # One decision per endpoint feeds both the azimuth ranges and
                # the exact-XY evidence. A no-return is never an obstacle, and
                # known leg-height terrain cannot be reintroduced by a second
                # body-band projection. All geometry uses the capture pose.
                projection = project_spatial_rays(
                    array, return_types, sensor_position, sensor_rotation,
                    z_reference+self.obstacle_min_z, z_reference+self.obstacle_max_z,
                    self.mapping_range, self.no_return_range, self.ray_count,
                    self.hit_dilation_bins, self._live_grid.resolution,
                    clear_min_z=z_reference+self.obstacle_min_z,
                    clear_max_z=z_reference+self.clearing_max_z,
                    unknown_min_z=z_reference+self.body_slice_min_z,
                    floor_tolerance=self.terrain_floor_tolerance,
                    obstacle_min_height=self.obstacle_min_height,
                    obstacle_max_height=self.obstacle_max_height,
                    body_position=(scan_x, scan_y, scan_z),
                    body_yaw=math.atan2(body_rotation[1, 0], body_rotation[0, 0]),
                    self_radius=self.footprint_radius, self_offset=self.footprint_offset)
                nearest, clearing_ranges = projection.ranges, projection.clearing
                self.get_logger().info(
                    f"[SPATIAL_RAY_EVIDENCE] body_z={scan_z:.3f} "
                    f"unknown_slice=[{z_reference+self.body_slice_min_z:.3f},"
                    f"{z_reference+self.obstacle_max_z:.3f}] "
                    f"obstacle={np.count_nonzero(projection.obstacle)} "
                    f"support={np.count_nonzero(projection.support)}; "
                    "invalid/no-return endpoints never occupy", throttle_duration_sec=2.)
                ground_cells = {self._live_grid.world_to_cell(*p[:2])
                                for p in projection.world[projection.support]}
                obstacle_cells = {self._live_grid.world_to_cell(*p[:2])
                                  for p in projection.world[projection.obstacle]}
                self.record_terrain(projection.world[projection.terrain],
                                    projection.heights[projection.terrain])
            except ValueError as error:
                self.get_logger().warning(f"Dropping invalid 3-D rays: {error}",
                                          throttle_duration_sec=2.)
                return
        else:
            if not self.cloud_is_world:
                array = array @ sensor_rotation.T + sensor_position
            nearest = classify_planar_ranges(
                array[:, 0] - ray_x, array[:, 1] - ray_y, array[:, 2],
                z_reference + self.obstacle_min_z, z_reference + self.obstacle_max_z,
                0.35, self.mapping_range, self.ray_count, self.hit_dilation_bins)
        if (identified_scan or identified_cloud) and not (np.isfinite(nearest).any() or
                                    np.any(clearing_ranges>=.35)):
            self.fusion_stats()["no_planar"] += 1
            return # no planar evidence: do not manufacture an observation revision
        with self._grid_lock:
            previous_grid = self._live_grid.data.copy()
            self._live_grid.integrate_ranges(
                (float(ray_x), float(ray_y)), nearest, self.mapping_range,
                no_return_range=self.no_return_range, clearing_ranges=clearing_ranges,
                ground_cells=ground_cells, obstacle_cells=obstacle_cells)
            changed_mask = self._live_grid.data != previous_grid
            if np.any(changed_mask):
                ys, xs = np.where(changed_mask)
                self.changed_cells.update(zip(xs.tolist(), ys.tolist()))
                self.map_content_revision += 1
            self.last_map_update_ns = now_ns
            self.last_integrated_cloud_stamp_ns = cloud_stamp_ns
            self.map_update_count += 1
            self.fusion_stats()["integrated"] += 1
            if not np.any(changed_mask):
                self.fusion_stats()["no_grid_change"] += 1
        self.publish_maps()

    def evaluate_active_observation(self) -> Optional[ObservationTask]:
        """Update information from new task-relevant sensor evidence."""
        return evaluate_observation(
            self.active_observation, self.grid, self.active_goal_cell,
            self.map_update_count, self.observation_done_ratio,
            self.observation_done_updates, self.observation_closure_updates,
            self.viewpoint_standoff)

    def observation_complete(self, task: Optional[ObservationTask]) -> bool:
        """Complete on positive information or confirmed boundary closure."""
        if task is not None and getattr(task,"information_satisfied",False):
            return True
        return observation_is_complete(
            task, self.observation_done_updates,
            self.observation_closure_updates)

    def record_observation_completion(self, task: Optional[ObservationTask]):
        """Freeze completion evidence before the next task replaces it."""
        (self.last_observation_completion_reason,
         self.last_observation_completion_progress) = (
            observation_completion_record(task))

    def prepare_next_observation(self, force: bool = False) -> bool:
        """Select the next information objective without publishing its path."""
        task = self.active_observation
        if self.position is None or self.active_goal is None or task is None:
            return False
        if (not force
                and self.last_preparation_attempt_update == self.map_update_count):
            return self.prepared_candidate is not None
        self.last_preparation_attempt_update = self.map_update_count

        preprocess_started = time.perf_counter()
        inflated = set()  # raw FREE coarse connection; Local owns body clearance
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        if self.grid.value(start) != FREE:
            self.publish_status("START_NOT_KNOWN_FREE")
            return False
        frontiers = self.grid.frontier_cells(inflated)
        clusters = cluster_frontiers(frontiers, self.min_frontier_size)
        filtered_frontiers = clustered_frontier_cells(clusters)
        self.latest_frontier_count = len(filtered_frontiers)
        preprocess_ms = (
            time.perf_counter() - preprocess_started) * 1000.0
        self.publish_frontiers(filtered_frontiers)
        candidate = self.choose_frontier(
            start, clusters, inflated, blocked_edges,
            excluded_goals=(self.active_goal,*self.current_handoff_exclusions()), allow_region_release=False,
            allow_cross_region_preparation=True,
            update_region_prediction=not force,
            map_preprocess_prefix_ms=preprocess_ms)
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

    def activate_or_prepare_next_observation(
            self, allow_commitment_transfer: bool = False) -> bool:
        """Activate a standby route, preparing one immediately if necessary."""
        had_standby = getattr(self, "prepared_candidate", None) is not None
        if self.activate_prepared_observation(allow_commitment_transfer):
            return True
        if self.coarse_repair_pending():
            return False
        if had_standby and self.prepared_candidate is not None:
            if getattr(self, "branch_commitment", None) is None:
                return False
            # An outside standby must not prevent a fresh search for a
            # successor in the committed branch after G1 finishes.
            self.prepared_candidate = None
        if not self.prepare_next_observation(force=True):
            return False
        return self.activate_prepared_observation(allow_commitment_transfer)

    def activate_prepared_observation(
            self, allow_commitment_transfer: bool = False) -> bool:
        """Reuse the safe suffix of a prepared route from the current pose."""
        candidate = self.prepared_candidate
        if self.position is None or candidate is None:
            return False
        candidate_scope = (candidate.commitment_scope
                           or ("region", candidate.region_id))
        active_scope = getattr(self, "active_commitment_scope", None)
        intent = getattr(self, "branch_commitment", None)
        if intent is not None:
            if intent.contains(self.grid, {candidate.frontier_cell}, self.branch_domain(intent)):
                candidate_scope = candidate.commitment_scope = active_scope
            elif candidate_scope == active_scope:
                return False  # A stale label is not current entrance evidence.
            # Task completion alone cannot authorize a regional transfer.
            allow_commitment_transfer = self.commitment_state != "COMMITTED"
        cross_scope = (active_scope is not None
                       and candidate_scope != active_scope)
        if (cross_scope and self.commitment_state == "COMMITTED"
                and not allow_commitment_transfer):
            # A standby route is speculative while the old task is running.
            # A completed-task handoff may transfer commitment, but only after
            # this method has validated and spliced the replacement route.
            self.get_logger().info(
                f"[PREPARED_HANDOFF_DEFERRED] prepared_region="
                f"{candidate.region_id} committed_region={self.active_region_id} "
                f"prepared_scope={candidate_scope} "
                f"committed_scope={active_scope}",
                throttle_duration_sec=2.0)
            return False
        if self.terminal_rejection_reason(
                candidate.cell, set(), candidate.excluded_goals) is not None:
            self.prepared_candidate = None
            return False

        inflated = set()  # raw FREE coarse connection; Local owns body clearance
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        if self.grid.value(start) != FREE:
            self.prepared_candidate = None
            return False
        self._repair_expansion_budget = ExpansionBudget(
            getattr(self, "global_repair_max_expansions", 400))
        target_cells = observation_target_cells(
            self.grid, candidate.frontier_cell, self.observation_radius,
            candidate.cell)
        hypotheses = observation_hypothesis_cells(
            self.grid, candidate.cell, target_cells, self.observation_radius)
        if (not hypotheses
                or len(target_cells) < self.min_expected_observation_cells):
            self.get_logger().info(
                f"[PREPARED_OBSERVATION_REJECTED] reason=NO_INFORMATION_HYPOTHESIS "
                f"region={candidate.region_id} terminal={candidate.cell} "
                f"targets={len(target_cells)} hypotheses={len(hypotheses)}")
            self.prepared_candidate = None
            return False

        witness = splice_prepared_path(
            self.grid, start, candidate.path, inflated, blocked_edges)
        if witness is None:
            # Discard already-traversed geometry; fix the current connector and
            # the remaining conflict segments, retaining their common suffix.
            if not candidate.path:
                self.prepared_candidate = None
                return False
            nearest = min(range(len(candidate.path)),
                          key=lambda i: math.dist(start, candidate.path[i]))
            witness = [start]+candidate.path[nearest:]
        result = self.repair_rejected_global_path(
            start, witness, candidate.cell, candidate.frontier_cell,
            inflated, blocked_edges)
        if result.status == RouteSearchStatus.INCOMPLETE:
            self.publish_status("WAITING_FOR_COARSE_ROUTE_REPAIR")
            return False
        if result.status != RouteSearchStatus.FOUND:
            self.prepared_candidate = None
            return False
        path = result.path
        route_source = "prepared_"+candidate.route_source
        if not path:
            self.prepared_candidate = None
            return False
        simplified = simplify_known_path(self.grid, path, inflated, blocked_edges)
        if not simplified or not all(
                segment_known_free(self.grid, a, b, inflated, blocked_edges)
                for a, b in zip(simplified[:-1], simplified[1:])):
            self.prepared_candidate = None
            return False

        refreshed = FrontierCandidate(
            candidate.region_id, candidate.cell, candidate.frontier_cell,
            candidate.goal, path,
            path_length_cells(path, self.grid.resolution), len(target_cells),
            candidate.cluster_size, path_turn_cost(path), target_cells,
            sparse_route=None, # Spliced/dense route must not claim the old graph transitions.
            terminal_cells=candidate.terminal_cells,
            terminal_clearance=candidate.terminal_clearance,
            frontier_cluster_cells=set(candidate.frontier_cluster_cells),
            connectivity_gain=candidate.connectivity_gain,
            provisional_portal=candidate.provisional_portal,
            commitment_scope=candidate.commitment_scope,
            option_target_cells=set(candidate.option_target_cells),
            excluded_goals=candidate.excluded_goals,
            observation_progress=candidate.observation_progress)
        refreshed.route_source = route_source
        if not self.directional_candidate_allowed(refreshed, candidate_scope):
            self.prepared_candidate = None
            self.get_logger().info(
                f"[DIRECTIONAL_PREPARED_ROUTE_REJECTED] region="
                f"{candidate.region_id}; prepared path crosses the active "
                "no-return gate",
                throttle_duration_sec=2.0)
            return False
        self.prepared_candidate = None
        self.get_logger().info(
            f"[PREPARED_OBSERVATION_VALIDATED] region={refreshed.region_id} "
            f"terminal={refreshed.cell} hypotheses={len(hypotheses)} "
            "motion=RAW_FREE_CONNECTION sensor_ray=MAY_CROSS_UNKNOWN "
            "requires_local_certification=true")
        if allow_commitment_transfer and cross_scope:
            task = self.active_observation
            self.last_selection_reason = ("station_capture_handoff" if task is not None and
                task.motion_released and not task.information_satisfied else "completed_task_handoff")
            self.get_logger().info(
                f"[REGION_COMMITMENT_TRANSFER_PREPARED] from={self.active_region_id} "
                f"to={refreshed.region_id} reason={self.last_selection_reason}")
        return self.activate_candidate(simplified, refreshed)

    def activate_candidate(self, path: Sequence[Cell], candidate: FrontierCandidate,
                           acknowledged_task=None):
        """Publish a candidate while keeping region accounting in one place."""
        if (getattr(self,"task_mode",False) and acknowledged_task is None
                and getattr(self,"completion_handoff_in_progress",False)):
            return self.stage_observation_handoff(path,candidate)
        if not self.publish_path(path, candidate, acknowledged_task=acknowledged_task):
            return False
        # Region/accounting commit follows successful publication (or the exact
        # Local installation ACK), never merely selecting a candidate.
        self.global_reroute_failure_streak = 0
        previous_scope = getattr(self, "active_commitment_scope", None)
        candidate_scope = (candidate.commitment_scope
                           or ("region", candidate.region_id))
        if self.bounded_visits_enabled():
            candidate_scope = self.visit_window().commit(candidate.region_id)
            candidate.commitment_scope = candidate_scope
            self.branch_commitment = None
        elif hasattr(self, "branch_option_serial"):
            intent = getattr(self, "branch_commitment", None)
            same_branch = (intent is not None and (
                candidate_scope == previous_scope or intent.contains(
                    self.grid, {candidate.frontier_cell}, self.branch_domain(intent))))
            # A successor already bound during preparation keeps that binding
            # at exact ACK, even if the current frame loses its witness.
            if same_branch:
                candidate_scope = previous_scope
            else:
                self.branch_option_serial += 1
                candidate_scope = ("branch_option", self.branch_option_serial)
                self.branch_commitment = BranchCommitment.initiate(
                    self.grid, path,
                    candidate.frontier_cluster_cells or {candidate.frontier_cell},
                    candidate.frontier_cell)
                self.branch_unresolved_since = None
                self.branch_unresolved_updates = 0
                self.branch_progress_remaining = None
                self.branch_progress_time = time.monotonic()
            candidate.commitment_scope = candidate_scope
        last_scope = getattr(self, "last_selected_commitment_scope", None)
        if last_scope is not None and candidate_scope != last_scope:
            self.region_switches += 1
            self.last_region_switch_reason = self.last_selection_reason
        elif (last_scope == candidate_scope
              and self.last_selected_region_id is not None
              and candidate.region_id != self.last_selected_region_id):
            self.intra_scope_region_switches += 1
            self.last_region_switch_reason = self.last_selection_reason
        self.last_selected_region_id = candidate.region_id
        self.last_selected_commitment_scope = candidate_scope
        self.active_region_id = candidate.region_id
        self.active_commitment_scope = candidate_scope
        self.active_region_missing_streak = 0
        self.active_region_unreachable_since = None
        if previous_scope != candidate_scope:
            self.commitment_target_cells = set(
                candidate.observation_cells if hasattr(self, "branch_option_serial")
                else candidate.option_target_cells or candidate.observation_cells)
            remaining = sum(
                self.grid.value(cell) == UNKNOWN
                for cell in self.commitment_target_cells)
            self.region_commitment_gate.reset(
                candidate_scope, (self.map_update_count if hasattr(self, "branch_option_serial")
                                  else self.map_content_revision),
                remaining)
        else:
            # New demand must come from a successor inside this entrance,
            # never all descendants of a Frontier lineage.
            self.commitment_target_cells.update(
                candidate.option_target_cells or candidate.observation_cells)
        self.commitment_state = "COMMITTED"
        self.commitment_remaining_cells = sum(
            self.grid.value(cell) == UNKNOWN
            for cell in self.commitment_target_cells)
        self.commitment_stagnant_revisions = 0
        if self.directional_recovery_pending:
            self.directional_recovery_switches += 1
        self.set_directional_commitment(path, candidate)
        self.directional_recovery_pending = False
        return True

    def stage_observation_handoff(self, path, candidate):
        if getattr(self,"pending_observation_handoff",None) is not None:
            return False
        predecessor = self.active_observation
        task = ObservationTask(candidate.region_id,candidate.goal,
            set(candidate.observation_cells),frontier_cell=candidate.frontier_cell,
            frontier_cluster_cells=set(candidate.frontier_cluster_cells),
            terminal_cells=candidate.terminal_cells or (candidate.cell,))
        if not getattr(self,"handoff_execution_retired",False):
            task.predecessor_task_id = predecessor.task_id
            task.predecessor_task_version = predecessor.task_version
        pending = PendingObservationHandoff(predecessor,task,list(path),candidate)
        self.pending_observation_handoff = pending
        task.route_estimate=candidate.sparse_route
        task.route_source=candidate.route_source
        if not self.publish_reference_path(path,preparing_task=task):
            self.pending_observation_handoff = None
            self.publish_status("OBSERVATION_HANDOFF_REFERENCE_REJECTED")
            return False
        self.publish_status("OBSERVATION_MOTION_RELEASED_PREPARING_SUCCESSOR" if
                            predecessor.motion_released and not predecessor.information_satisfied
                            else "OBSERVATION_SATISFIED_PREPARING_SUCCESSOR")
        return True

    def commit_observation_handoff(self,pending,feedback):
        # Same planning callback group as publication/feedback: no partially
        # committed region/task can be observed by the next decision tick.
        self.pending_observation_handoff = None
        self.completion_handoff_in_progress = True
        try:
            self.activate_candidate(pending.path,pending.candidate,
                                    acknowledged_task=pending.task)
        finally:
            self.completion_handoff_in_progress = False
        self.active_path_request_generation = pending.request_id
        self.active_path_trajectory_id = feedback.trajectory_id
        self.record_route_source_activation(pending.task, pending.request_id)
        self.pending_path_publish_ns = 0
        self.pending_path_request_generation = None
        self.pending_path_deferred = False
        self.scan_waiting_for_target = False
        self.handoff_execution_retired = False
        self.handoff_excluded_goals = []
        self.handoff_excluded_until = {}
        old = pending.predecessor
        if ((not old.motion_released or old.information_satisfied)
                and not self.is_blacklisted(old.goal)):
            self.completed_goals.append(old.goal)
        if not old.motion_released or old.information_satisfied:
            self.observations_satisfied += 1
        self.record_observation_completion(old)
        self.publish_status("OBSERVATION_HANDOFF_COMMITTED")

    def clear_directional_commitment(self):
        self.commitment_direction_origin = None
        self.commitment_direction_vector = None
        self.commitment_direction_progress = 0.0
        self.directional_recovery_pending = False

    def set_directional_commitment(
            self, path: Sequence[Cell], candidate: FrontierCandidate):
        """Move the no-return gate to the robot and follow the new route."""
        if self.position is None:
            self.clear_directional_commitment()
            return
        origin = self.position
        route = [self.grid.cell_to_world(cell) for cell in path]
        if not route or math.hypot(
                route[0][0] - origin[0], route[0][1] - origin[1]) > 0.5:
            route.insert(0, origin)
        route.append(candidate.goal)
        target = candidate.goal
        for point in route[1:]:
            if math.hypot(point[0] - origin[0], point[1] - origin[1]) \
                    >= self.region_direction_lookahead:
                target = point
                break
        vector = (target[0] - origin[0], target[1] - origin[1])
        if math.hypot(vector[0], vector[1]) <= 1e-6:
            self.clear_directional_commitment()
            return
        self.commitment_direction_origin = origin
        self.commitment_direction_vector = vector
        self.commitment_direction_progress = 0.0

    def directional_candidate_route(
            self, candidate: FrontierCandidate) -> List[Point2]:
        if candidate.path:
            route = [self.grid.cell_to_world(cell) for cell in candidate.path]
        elif candidate.sparse_route is not None:
            route = list(candidate.sparse_route.polyline)
        else:
            route = []
        if self.position is not None and (not route or math.hypot(
                route[0][0] - self.position[0],
                route[0][1] - self.position[1]) > 0.5):
            route.insert(0, self.position)
        if not route or route[-1] != candidate.goal:
            route.append(candidate.goal)
        return route

    def directional_candidate_allowed(
            self, candidate: FrontierCandidate,
            candidate_scope: Tuple[str, int]) -> bool:
        intent = getattr(self, "branch_commitment", None)
        if intent is not None and candidate_scope == self.active_commitment_scope:
            return intent.contains(self.grid, {candidate.frontier_cell}, self.branch_domain(intent))
        if getattr(self,"task_mode",False) and getattr(self,"task_geometry_autonomy",False):
            # Scope selection remains Global-owned. A whole-world half-plane
            # is not a branch constraint on a curved corridor.
            return True
        if (self.active_commitment_scope is None
                or candidate_scope != self.active_commitment_scope
                or self.commitment_direction_origin is None
                or self.commitment_direction_vector is None):
            return True
        origin = self.commitment_direction_origin
        forward = self.commitment_direction_vector
        norm = math.hypot(forward[0], forward[1])
        if norm <= 1e-9:
            return True
        if self.position is not None:
            current_progress = (
                (self.position[0] - origin[0]) * forward[0]
                + (self.position[1] - origin[1]) * forward[1]) / norm
            self.commitment_direction_progress = max(
                self.commitment_direction_progress, current_progress)
        tolerance = (
            self.region_recovery_backtrack_tolerance
            if self.directional_recovery_pending
            else self.region_backtrack_tolerance)
        allowed = directional_route_allowed(
            origin, forward, self.commitment_direction_progress,
            self.directional_candidate_route(candidate), tolerance)
        if not allowed:
            if self.directional_suppression_revision != self.map_content_revision:
                self.directional_suppression_revision = self.map_content_revision
                self.directional_suppression_cells = set()
            key = (candidate.region_id, candidate.cell)
            if key not in self.directional_suppression_cells:
                self.directional_suppression_cells.add(key)
                self.directional_candidate_suppressed_count += 1
        return allowed

    def reroute_active_goal(self, excluded_route=None) -> bool:
        """Replan to the current observation pose without resetting its gain."""
        if (self.position is None or self.active_goal is None
                or self.active_goal_cell is None or self.active_observation is None):
            return False

        inflated = set()  # raw FREE coarse connection; Local owns body clearance
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        if self.grid.value(start) != FREE:
            return False
        raw_path = astar_known(
            self.grid, start, self.active_goal_cell, inflated, blocked_edges)
        if not raw_path:
            return False
        if excluded_route is not None and tuple(raw_path) == excluded_route:
            return False
        simplified = simplify_known_path(
            self.grid, raw_path, inflated, blocked_edges)
        if not simplified or not all(
                segment_known_free(self.grid, a, b, inflated, blocked_edges)
                for a, b in zip(simplified[:-1], simplified[1:])):
            return False
        if (self.active_commitment_scope is not None
                and self.commitment_direction_origin is not None
                and self.commitment_direction_vector is not None):
            route = [self.grid.cell_to_world(cell) for cell in simplified]
            route.append(self.active_goal)
            tolerance = (
                self.region_recovery_backtrack_tolerance
                if self.directional_recovery_pending
                else self.region_backtrack_tolerance)
            if not directional_route_allowed(
                    self.commitment_direction_origin,
                    self.commitment_direction_vector,
                    self.commitment_direction_progress,
                    route, tolerance):
                self.get_logger().warning(
                    "[DIRECTIONAL_ACTIVE_REROUTE_REJECTED] repaired path "
                    "would cross behind the committed no-return gate")
                return False

        validated = self.validated_reference_path(simplified)
        if validated is None:
            self.get_logger().warning(
                "[REFERENCE_PATH_REJECTED] reason=SEGMENT_BLOCKED during reroute")
            return False
        self.publish_reference_path(validated)
        self.active_raw_path = list(raw_path)
        self.active_path_progress_index = 0
        self.last_active_path_validation_update = self.map_update_count
        self.prepared_candidate = None
        self.last_preparation_attempt_update = -1
        self.last_handoff_attempt_update = -1
        self.active_since_ns = self.get_clock().now().nanoseconds
        self.healthy_task_elapsed = 0.0
        self.publish_status("ACTIVE_GOAL_REROUTED")
        self.get_logger().info(
            f"Rerouted active observation to ({self.active_goal[0]:.2f}, "
            f"{self.active_goal[1]:.2f}) without resetting observation progress")
        return True

    def terminal_clearance(self, cell: Cell, inflated: Set[Cell]) -> float:
        """Soft raw-wall distance for ranking; never a body clearance veto."""
        radius_cells = max(1, int(math.ceil(
            self.terminal_clearance_search_radius / self.grid.resolution)))
        best = float(radius_cells + 1)
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                neighbour = (cell[0] + dx, cell[1] + dy)
                if self.grid.value(neighbour) == OCCUPIED:
                    best = min(best, math.hypot(dx, dy))
        return best * self.grid.resolution

    def approach_clearance(self, path: Sequence[Cell],
                           inflated: Set[Cell]) -> float:
        """Measure bottleneck clearance over the terminal path suffix."""
        if not path:
            return 0.0
        dense_path: List[Cell] = [path[0]]
        for first, second in zip(path[:-1], path[1:]):
            segment = bresenham(first, second)
            if segment and dense_path[-1] == segment[0]:
                segment = segment[1:]
            dense_path.extend(segment)
        remaining = max(self.grid.resolution, self.terminal_approach_length)
        suffix = [dense_path[-1]]
        for first, second in zip(
                reversed(dense_path[:-1]), reversed(dense_path[1:])):
            if remaining <= 0.0:
                break
            suffix.append(first)
            remaining -= math.hypot(
                second[0] - first[0], second[1] - first[1]
            ) * self.grid.resolution
        return min(self.terminal_clearance(item, inflated) for item in suffix)

    def validate_and_repair_active_path(self) -> bool:
        """Validate one new map revision and atomically repair an invalid route.

        False means this timer iteration handled an invalid route and must not
        continue observation handoff logic using the stale task state.
        """
        if getattr(self,"task_mode",False) and getattr(self,"task_geometry_autonomy",False):
            # This polyline is a motion-domain hint, NOT the executing path.
            # Local validates every real candidate and safety sweep. A changed
            # grid cell on the hint must not force a second Global rejoin loop.
            return True
        if (self.position is None or self.active_goal is None
                or not self.active_raw_path):
            return True
        if self.last_active_path_validation_update == self.map_update_count:
            return True
        self.last_active_path_validation_update = self.map_update_count

        inflated = set()  # raw FREE coarse connection; Local owns body clearance
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
        self.global_path_invalid_count += 1

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

    def terminate_active_observation(self, reason: str, status: Optional[str] = None,
                                     blacklist_goal: bool = False,
                                     release_region: bool = False,
                                     completed: bool = False) -> bool:
        """Retire one task and clear its in-flight execution identity once."""
        self.cancel_pending_observation_handoff(reason)
        task = self.active_observation
        goal = self.active_goal
        if task is None and goal is None:
            return False
        if task is not None:
            self.retire_observation_task(task, TaskMessage.CANCELLED)
            self.revoke_task_path_constraints(task)
        if goal is not None:
            if blacklist_goal:
                self.blacklist.append(goal)
            elif completed and not self.is_blacklisted(goal):
                self.record_completed_goal(goal)
        self.active_goal = None
        self.active_goal_cell = None
        self._active_observation = None
        self.active_raw_path = []
        self.active_path_progress_index = 0
        self.pending_blocked_edge = None
        self.prepared_candidate = None
        self.replacement_pending = False
        self.active_path_request_generation = None
        self.active_path_trajectory_id = None
        self.pending_path_request_generation = None
        self.pending_path_publish_ns = 0
        self.pending_path_deferred = False
        self.pending_observation_station = None
        self.blocked_execution_context = None
        self.scan_waiting_for_target = False
        self.active_task_elapsed = 0.0
        self.healthy_task_elapsed = 0.0
        self.last_active_goal_clear_reason = reason
        if release_region:
            self.terminate_region_option(reason)
        if status:
            self.publish_status(status)
        return True

    def finish_active_observation(self, status: str):
        """Finish an information objective independently of pose arrival."""
        self.record_observation_completion(self.active_observation)
        self.observations_satisfied += 1
        self.terminate_active_observation(
            "OBSERVATION_COMPLETE", status, completed=True)

    def terminate_region_option(self, reason: str) -> bool:
        """Terminate the single active semi-MDP option exactly once."""
        if self.active_commitment_scope is None and self.active_region_id is None:
            return False
        previous_region = self.active_region_id
        previous_scope = self.active_commitment_scope
        self.active_region_id = None
        self.active_commitment_scope = None
        self.commitment_target_cells = set()
        self.branch_commitment = None
        self.branch_unresolved_since = None
        self.branch_unresolved_updates = 0
        self.branch_continuation_present = False
        self.clear_directional_commitment()
        self.active_region_missing_streak = 0
        self.active_region_unreachable_since = None
        self.commitment_state = "RELEASABLE"
        self.commitment_release_count += 1
        self.last_commitment_release_reason = reason
        self.last_selection_reason = reason
        self.region_commitment_gate.reset()
        self.get_logger().info(
            f"[REGION_OPTION_TERMINATED] region={previous_region} "
            f"scope={previous_scope} reason={reason}")
        return True

    def branch_domain(self, intent):
        """One association computation per immutable fusion snapshot."""
        grid = self.grid
        key = (id(intent), getattr(self,"map_update_count",0),
               getattr(self,"map_content_revision",0), grid.origin_x,grid.origin_y,
               grid.width,grid.height,grid.resolution)
        cached = getattr(self,"branch_domain_cache",None)
        if cached is None or cached[0] != key:
            cached = self.branch_domain_cache = (key,intent.domain(grid))
        return cached[1]

    def evaluate_branch_commitment(self):
        """Assess the option before request/ACK/preparation early returns.

        Missing spatial association gets bounded recovery time. Unmet
        information stays in the existing observation task ledger.
        """
        intent = getattr(self, "branch_commitment", None)
        if intent is None or self.active_commitment_scope is None:
            return
        if self.branch_association_update == self.map_update_count:
            return
        self.branch_association_update = self.map_update_count
        domain = self.branch_domain(intent)
        inflated = set()  # raw FREE coarse connection; Local owns body clearance
        clusters = cluster_frontiers(
            self.grid.frontier_cells(inflated), self.min_frontier_size)
        associated = any(intent.contains(self.grid, {cell}, domain)
                         for cluster in clusters for cell in cluster)
        self.branch_continuation_present = associated
        remaining = sum(self.grid.value(c) == UNKNOWN for c in self.commitment_target_cells)
        self.commitment_remaining_cells = remaining
        now = time.monotonic()
        if remaining != getattr(self, "branch_progress_remaining", None):
            self.branch_progress_remaining = remaining
            self.branch_progress_time = now
        task = self.active_observation
        task_can_yield = (task is None or task.motion_released or task.information_satisfied
                          or self.observation_complete(task))
        can_release = (task_can_yield
                       and getattr(self,"pending_observation_handoff",None) is None
                       and getattr(self,"unconfirmed_observation_handoff",None) is None)
        resolving = (intent.entrance and domain is None) or (
            not associated and remaining > self.region_min_remaining_cells)
        successor_wait = (self.branch_unresolved_since is not None
                          and task is not None
                          and (task.motion_released or task.information_satisfied
                               or self.observation_complete(task)))
        if resolving or successor_wait:
            if self.branch_unresolved_since is None:
                self.branch_unresolved_since = time.monotonic()
                self.branch_unresolved_updates = 0
            self.branch_unresolved_updates += 1
            if (can_release and self.branch_unresolved_updates >= self.region_release_updates
                    and time.monotonic()-self.branch_unresolved_since
                    >= self.region_unreachable_timeout):
                self.terminate_region_option("branch_association_unresolved")
            return
        # Visible continuation prevents exhaustion of G1's frozen targets
        # from being mistaken for completion of the entire branch.
        decision = self.region_commitment_gate.evaluate(
            self.active_commitment_scope, self.map_update_count,
            remaining, continuation_present=associated)
        self.commitment_stagnant_revisions = decision.stagnant_revisions
        if (can_release and decision.release_reason and (decision.release_reason != "residual_stagnant"
                or now-self.branch_progress_time >= self.region_unreachable_timeout)):
            self.terminate_region_option(decision.release_reason)

    def release_commitment_for_idle_handoff(self) -> bool:
        """Remove stale region hysteresis only after SCAN confirms it is idle."""
        if getattr(self, "branch_commitment", None) is not None:
            return False  # Idle G1 motion is not termination of its branch.
        if not self.scan_waiting_for_target or self.active_region_id is None:
            return False
        return self.terminate_region_option("scan_wait_target")

    def replace_unsatisfied_observation(self) -> bool:
        """Atomically replace a reached viewpoint that delivered too little gain."""
        if self.active_goal is None:
            self.replacement_pending = False
            return False
        if self.last_handoff_attempt_update == self.map_update_count:
            return False
        self.last_handoff_attempt_update = self.map_update_count
        previous_goal = self.active_goal

        switched = self.activate_or_prepare_next_observation(
            allow_commitment_transfer=True)
        if not switched:
            released = self.release_commitment_for_idle_handoff()
            if released:
                switched = self.activate_prepared_observation(
                    allow_commitment_transfer=True)
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
        # Completed information or a released station motion may hand off.
        # Failure/cancellation must never claim this predecessor relationship.
        self.completion_handoff_in_progress = True
        try:
            return self._handoff_satisfied_observation(status)
        finally:
            self.completion_handoff_in_progress = False

    def _handoff_satisfied_observation(self, status: str) -> bool:
        """Switch atomically, retaining the safe current task on failure."""
        if getattr(self,"pending_observation_handoff",None) is not None:
            return True
        if self.active_goal is None:
            return False
        if self.last_handoff_attempt_update == self.map_update_count:
            return False
        self.last_handoff_attempt_update = self.map_update_count
        previous_goal = self.active_goal
        completed_task = self.active_observation
        if getattr(self,"task_mode",False) and completed_task is not None:
            if (not completed_task.information_satisfied and
                    (not completed_task.motion_released or self.observation_complete(completed_task))):
                completed_task.information_satisfied = True
                completed_task.satisfied_map_revision = self.map_content_revision
                completed_task.satisfied_sensor_revision = self.grid.sensor_revision
                completed_task.lifecycle = "SATISFIED"

        switched = self.activate_or_prepare_next_observation(
            allow_commitment_transfer=True)
        if not switched and self.coarse_repair_pending():
            self.publish_status("WAITING_FOR_COARSE_ROUTE_REPAIR")
            return False
        if not switched and not getattr(self,"task_mode",False):
            released = self.release_commitment_for_idle_handoff()
            if released:
                switched = self.activate_prepared_observation(
                    allow_commitment_transfer=True)
        if not switched:
            switched = self.plan_from_current_position(
                excluded_goals=(previous_goal,*self.current_handoff_exclusions()),
                allow_region_release=not getattr(self,"task_mode",False))
        if switched:
            if getattr(self,"pending_observation_handoff",None) is not None:
                return True  # Information satisfaction is not execution commit.
            if ((completed_task is None or not completed_task.motion_released or completed_task.information_satisfied)
                    and not self.is_blacklisted(previous_goal)):
                self.record_completed_goal(previous_goal)
            if completed_task is None or not completed_task.motion_released or completed_task.information_satisfied:
                self.observations_satisfied += 1
            self.record_observation_completion(completed_task)
            self.publish_status(status)
            return True

        if self.latest_frontier_count == 0:
            if getattr(self, "branch_commitment", None) is not None:
                self.publish_status("WAITING_FOR_BRANCH_ASSOCIATION")
                return False
            if completed_task is not None and completed_task.motion_released and not completed_task.information_satisfied:
                self.terminate_active_observation("STATION_CAPTURE_INCOMPLETE", "EXPLORATION_COMPLETE")
            else:
                self.finish_active_observation("EXPLORATION_COMPLETE")
            return True

        # The current goal is a known-free stand-off pose.  Keep its task
        # alive while waiting for a map update instead of creating a period
        # with neither an old task nor a validated new one.
        self.publish_status("WAITING_FOR_SAFE_HANDOFF")
        return False

    @staticmethod
    def parse_local_execution_event(value: str):
        return parse_local_execution_event(value)

    def local_execution_event_callback(self, msg: String):
        """Accept only the terminal event for the currently active request.

        The controller is a separate process, so this event remains observable
        when SCAN's internal FSM timer stops being scheduled.  Processing is
        deferred to the exploration timer to keep this callback idempotent and
        cheap.
        """
        event = self.parse_local_execution_event(msg.data)
        if (event is None or event["event"] not in (
                "LOCAL_TRAJECTORY_FINISHED", "LOCAL_TRAJECTORY_STALLED")):
            return
        if event["request_id"] != self.active_path_request_generation:
            return
        previous = self.pending_local_execution_finish
        if (previous is not None
                and previous["request_id"] == event["request_id"]
                and previous["trajectory_id"] >= event["trajectory_id"]):
            return
        self.pending_local_execution_finish = event
        self.local_execution_finish_events += 1
        self.get_logger().warning(
            f"[LOCAL_EXECUTION_TERMINAL] event={event['event']} "
            f"request_id={event['request_id']} "
            f"trajectory_id={event['trajectory_id']} "
            f"terminal_error={event['terminal_error']:.3f}m; "
            "waiting for the coordinator to continue or complete the option")

    def handle_local_execution_finish(self) -> bool:
        if getattr(self, "task_mode", False):
            self.pending_local_execution_finish = None
            return False
        event = self.pending_local_execution_finish
        if event is None:
            return False
        if (event["request_id"] != self.active_path_request_generation
                or self.active_goal is None or self.position is None):
            self.pending_local_execution_finish = None
            return False

        # A local spline ending is not itself a global/observation completion.
        # If the regional terminal is still far away, renew the same task with
        # a fresh request id.  This also wakes SCAN after an internal timer
        # chain has gone silent.
        terminal_distance = math.hypot(
            self.active_goal[0] - self.position[0],
            self.active_goal[1] - self.position[1])
        arrival_radius = max(0.30, 2.0 * self.grid.resolution)
        self.pending_local_execution_finish = None
        if terminal_distance > arrival_radius:
            if self.reroute_active_goal():
                self.local_execution_finish_reroutes += 1
                self.publish_status("LOCAL_SEGMENT_FINISHED_CONTINUING_OPTION")
                return True
            self.pending_local_execution_finish = event
            self.publish_status("LOCAL_SEGMENT_FINISHED_WAITING_FOR_ROUTE")
            return True

        # Reuse the reliable request-id completion path.  It evaluates the
        # observation predicate before terminating the regional option.
        reached = String()
        reached.data = f"REACHED request_id={event['request_id']}"
        self.planning_status_callback(reached)
        return True

    def planning_status_callback(self, msg: String):
        status, request_generation = parse_planning_status(msg.data)
        fields = dict(part.split("=",1) for part in msg.data.split() if "=" in part)
        if self.pending_observation_status(status,request_generation,fields):
            return
        task = self.active_observation
        if (getattr(self,"task_mode",False) and task is not None
                and (getattr(task,"information_satisfied",False) or
                     getattr(task,"motion_released",False)) and request_generation is not None
                and fields.get("task_id")==str(task.task_id)
                and fields.get("task_version")==str(task.task_version)):
            return
        if getattr(self, "task_mode", False):
            if status in ("REACHED", "WAIT_TARGET"):
                return  # Only measured typed station feedback may end motion.
            task = self.active_observation
            fields = dict(part.split("=", 1) for part in msg.data.split() if "=" in part)
            if request_generation is not None and task is not None:
                if (fields.get("task_id") != str(task.task_id)
                        or fields.get("task_version") != str(task.task_version)):
                    self.task_stale_feedback += 1
                    return
            if status == "REFERENCE_PATH_REJECTED" and fields.get("reason") == "INVALID_TASK_CONTRACT":
                if request_generation != self.pending_path_request_generation:
                    self.task_stale_feedback += 1
                    return
                self.pending_path_publish_ns = 0
                self.pending_path_deferred = False
                self.pending_path_request_generation = None
                self.scan_reference_rejected_count += 1
                self.get_logger().error(
                    f"[TASK_CONTRACT_REJECTED] request={request_generation} "
                    f"field={fields.get('field', 'unknown')}; retaining observation and region")
                self.publish_status("TASK_CONTRACT_REJECTED")
                if getattr(self,"task_geometry_autonomy",False):
                    self.retire_observation_task(task,TaskMessage.CANCELLED)
                    self.departure_recovery_terminal=True
                    self.blocked_execution_context=None
                    return
                self.latch_blocked_execution("INVALID_TASK_CONTRACT")
                return
            if status in ("BLOCKED", "TRACKING_RECOVERY_FAILED", "REFERENCE_PATH_REJECTED", "INVALID_REFERENCE_PATH"):
                if not planning_status_matches_request(status, request_generation,
                        self.pending_path_request_generation, self.active_path_request_generation):
                    self.task_stale_feedback += 1
                    return
                self.pending_path_publish_ns = 0
                self.pending_path_request_generation = None
                self.pending_path_deferred = False
                if getattr(self,"task_geometry_autonomy",False):
                    self.handle_local_motion_exhausted(reason=fields.get("reason", status))
                    return
                self.latch_blocked_execution(fields.get("reason", status))
                self.publish_status("LOCAL_EXECUTION_BLOCKED")
                return
        # WAIT_TARGET is a heartbeat.  Once this request has completed, later
        # repeats only reaffirm that SCAN is idle; they must not re-run the
        # completion transition or be reported as stale after the active task
        # has already been handed off/cleared.
        if (status == "WAIT_TARGET" and request_generation is not None and
                request_generation == self.last_completed_request_generation and
                self.active_path_request_generation in (None, request_generation)):
            self.scan_waiting_for_target = True
            return
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
            self.pending_path_deferred = False
            self.pending_path_request_generation = None
        elif status == "PATH_DEFERRED":
            self.pending_path_deferred = True
            self.scan_waiting_for_target = False
            self.get_logger().info(
                f"[PATH_REQUEST_DEFERRED] request_id={request_generation}; "
                "SCAN retains the current structured-recovery transaction")
        elif status in ("RUNNING", "PATH_ACCEPTED"):
            if status == "PATH_ACCEPTED" and self.pending_path_deferred:
                # Start the bounded planner-response timeout when SCAN really
                # activates the queued request, not while an older recovery
                # transaction still owns execution.
                self.pending_path_publish_ns = self.get_clock().now().nanoseconds
                self.pending_path_deferred = False
            self.scan_waiting_for_target = False
            # A successful replacement trajectory resolves the pending local
            # failure.  Keep the short-lived edge record, but do not let it be
            # mistaken for the cause of a later unrelated BLOCKED status.
            self.pending_blocked_edge = None
        elif status in ("REACHED", "WAIT_TARGET"):
            self.scan_waiting_for_target = True
            if not completion_status_is_new(
                    status, request_generation,
                    self.last_completed_request_generation):
                return
            self.last_completed_request_generation = request_generation
            if self.active_goal is None:
                return
            task = self.evaluate_active_observation()
            self.goals_reached += 1
            if self.observation_complete(task):
                self.get_logger().info(
                    f"[OBSERVATION_COMPLETE] reason={task.completion_reason} "
                    f"progress={task.progress:.1%} frontier_missing="
                    f"{task.frontier_missing_streak}/"
                    f"{self.observation_closure_updates}")
                self.handoff_satisfied_observation(
                    "OBSERVATION_HANDOFF_COMPLETE_AT_VIEWPOINT")
            else:
                # Arrival releases motion unconditionally.  The achieved ratio
                # is recorded for measurement but must never gate the switch:
                # completing a ratio that depends on the robot observing more
                # cannot terminate while the robot is held in place.  The
                # previous goal is excluded from the next selection through
                # completed_goals, so this cannot bounce back to the same pose.
                self.observation_motion_released_count += 1
                self.last_observation_released_reason = (
                    task.completion_reason if task is not None else "no_task")
                self.last_observation_released_progress = (
                    task.progress if task is not None else 0.0)
                self.get_logger().info(
                    f"[OBSERVATION_MOTION_RELEASED] reason="
                    f"{self.last_observation_released_reason} progress="
                    f"{self.last_observation_released_progress:.1%} "
                    f"frontier_missing="
                    f"{task.frontier_missing_streak if task is not None else 0}"
                    f"/{self.observation_closure_updates}; switching anyway")
                self.replace_unsatisfied_observation()
        elif status in ("VIEWPOINT_LOCAL_REJECTED", "LOCAL_RECOVERY_EXHAUSTED"):
            # SCAN exhausted a finite family of geometrically distinct local
            # paths for this exact viewpoint.  This is not evidence that the
            # committed region or portal is bad.  Cool only the rejected pose
            # and first ask the existing region commitment for a sibling
            # viewpoint.  Region ownership changes only if no sibling exists.
            self.local_repair_exhausted_count += 1
            # This terminal event is new route knowledge even when the global
            # occupancy grid did not change: SCAN proved that the exact local
            # approach to this viewpoint has exhausted its finite candidate
            # family.  Advance the route context before asking ReplanGate for
            # a sibling so it cannot suppress the replacement as an unchanged
            # BLOCKED retry.
            constraint_preapplied = (
                request_generation is not None
                and int(request_generation) in
                getattr(self, "global_viewpoint_constraint_requests", set()))
            if not constraint_preapplied:
                self.route_constraint_revision += 1
            previous_goal = self.active_goal
            failed_scope = (getattr(self, "active_commitment_scope", None)
                            or ("region", self.active_region_id))
            if status == "LOCAL_RECOVERY_EXHAUSTED":
                # CANCEL has already been sent. A sibling must get a NEW
                # task identity; reusing the frozen cancelled objective would
                # be rejected by Controller's task-retirement fence.
                self.terminate_active_observation("LOCAL_RECOVERY_EXHAUSTED")
            # Retain the frozen information objective while trying terminals.
            captured = self.clear_active_motion(
                status, keep_observation=True, reset_path_progress=True)
            failed_region = captured["region"]
            if previous_goal is not None and not constraint_preapplied:
                self.add_goal_failure_cooldown(previous_goal, status)
            excluded_goals = ((previous_goal,)
                              if previous_goal is not None else ())
            if self.plan_from_current_position(
                    excluded_goals=excluded_goals, required_scope=failed_scope,
                    allow_region_release=False):
                self.publish_status(
                    status + "_SIBLING_SELECTED")
                return

            # The viewpoint-level alternative set is empty.  Only now may the
            # failed child region be cooled and the enclosing commitment move
            # to a different child/portal.
            if failed_region is not None:
                self.add_region_failure_cooldown(
                    failed_region, "VIEWPOINT_ALTERNATIVES_EXHAUSTED")
                if self.active_region_id == failed_region:
                    self.terminate_region_option(
                        "viewpoint_alternatives_exhausted")
            if self.plan_from_current_position(excluded_goals=excluded_goals):
                self.publish_status(
                    status + "_REGION_CHANGED")
            else:
                self.publish_status(
                    status + "_WAITING_FOR_ROUTE")
        elif status == "LOCAL_REPAIR_EXHAUSTED":
            # SCAN has already held the robot and spent its local alternative
            # budget for this exact request.  Retire only the failed child
            # region/viewpoint first; the enclosing commitment remains active
            # so a sibling portal is preferred over an unrelated global jump.
            self.local_repair_exhausted_count += 1
            captured = self.clear_active_motion(
                "LOCAL_REPAIR_EXHAUSTED", reset_path_progress=True)
            failed_region = captured["region"]
            previous_goal = captured["goal"]
            if failed_region is not None:
                self.add_region_failure_cooldown(
                    failed_region, "LOCAL_REPAIR_EXHAUSTED")
            # Permit one bounded retreat to the nearest fork.  The failed
            # child remains cooled, so this cannot select the same branch.
            self.directional_recovery_pending = True
            if previous_goal is not None:
                self.add_goal_failure_cooldown(
                    previous_goal, "LOCAL_REPAIR_EXHAUSTED")
            if self.plan_from_current_position(
                    excluded_goals=((previous_goal,)
                                    if previous_goal is not None else ())):
                self.publish_status(
                    "LOCAL_REPAIR_EXHAUSTED_COMMITTED_CHILD_CHANGED")
            else:
                self.publish_status(
                    "LOCAL_REPAIR_EXHAUSTED_WAITING_FOR_COMMITTED_ROUTE")
        elif status == "BLOCKED":
            self.scan_blocked_count += 1
            # Preserve the current objective until rerouting has had a chance.
            self.global_reroute_failure_streak = 0
            self.pending_path_publish_ns = 0
            self.pending_path_deferred = False
            self.pending_path_request_generation = None
            self.active_path_request_generation = None
            blocked_edge = self.pending_blocked_edge
            self.pending_blocked_edge = None
            if (blocked_edge is not None
                    and blocked_edge in self.current_blocked_edges()
                    and self.reroute_active_goal()):
                return
            captured = self.clear_active_motion("BLOCKED")
            failed_region = captured["region"]
            previous_goal = captured["goal"]
            if previous_goal is not None:
                self.add_goal_failure_cooldown(previous_goal, "BLOCKED")
            excluded_goals = ((previous_goal,)
                              if previous_goal is not None else ())
            if (previous_goal is not None
                    and self.plan_from_current_position(
                        excluded_goals=excluded_goals)):
                self.publish_status("LOCAL_BLOCKED_VIEWPOINT_CHANGED")
                return

            # No sibling viewpoint in the committed option survived the same
            # dense checks.  Keeping that option while a frame-count cooldown
            # expires reproduces the identical rejected path forever.  Cool
            # the failed child region, release only this option, then permit a
            # different region/portal to take ownership.
            if failed_region is not None:
                self.add_region_failure_cooldown(
                    failed_region, "BLOCKED_NO_ALTERNATIVE")
                if self.active_region_id == failed_region:
                    self.terminate_region_option("blocked_no_alternative")
            if self.plan_from_current_position(
                    excluded_goals=excluded_goals):
                self.publish_status("LOCAL_BLOCKED_REGION_CHANGED")
            else:
                self.publish_status("LOCAL_BLOCKED_WAITING_FOR_ROUTE")
        elif status in ("REFERENCE_PATH_REJECTED", "INVALID_REFERENCE_PATH"):
            self.scan_reference_rejected_count += 1
            self.clear_active_motion(status, blacklist_goal=True)
            self.publish_status(f"RECOVER_FROM_{status}")

    def clear_active_motion(self, reason: str, *, keep_observation: bool = False,
                            reset_path_progress: bool = False,
                            blacklist_goal: bool = False) -> dict:
        """Invalidate the motion state that every local-motion terminal shares.

        The five failure reasons in planning_status_callback each keep their own
        policy (which cooldown, whether the child region is released, whether a
        sibling is preferred before a region change). This is only the state
        they must every one invalidate, extracted so the shared part appears
        once and the differing part stays visible at each call site.
        Returns the region, goal and blocked edge captured before clearing,
        because each policy needs them.
        """
        captured = {
            "region": self.active_region_id,
            "goal": self.active_goal,
            "blocked_edge": self.pending_blocked_edge,
        }
        self.global_reroute_failure_streak = 0
        self.pending_path_publish_ns = 0
        self.pending_path_deferred = False
        self.pending_path_request_generation = None
        self.active_path_request_generation = None
        if blacklist_goal and self.active_goal is not None:
            self.blacklist.append(self.active_goal)
        self.active_goal = None
        self.active_goal_cell = None
        if not keep_observation:
            self.active_observation = None
        self.active_raw_path = []
        if reset_path_progress:
            self.active_path_progress_index = 0
        self.pending_blocked_edge = None
        self.prepared_candidate = None
        self.replacement_pending = False
        self.last_active_goal_clear_reason = reason
        return captured

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

    def completion_neighbourhood(self, point: Point2):
        grid = self.grid
        x,y = grid.world_to_cell(*point)
        radius = int(math.ceil(2.0/grid.resolution))
        x0,x1=max(0,x-radius),min(grid.width,x+radius+1)
        y0,y1=max(0,y-radius),min(grid.height,y+radius+1)
        cells=grid.data[y0:y1,x0:x1]
        free={(int(cx+x0),int(cy+y0)) for cy,cx in np.argwhere(cells==FREE)}
        return free, int(np.count_nonzero(cells==UNKNOWN))

    def record_completed_goal(self, point: Point2):
        self.completed_goals.append(point)
        if not hasattr(self, "completed_goal_evidence"):
            self.completed_goal_evidence = {}
        self.completed_goal_evidence[point] = self.completion_neighbourhood(point)[0]

    def is_blacklisted(self, point: Point2) -> bool:
        if any(math.dist(point,old)<self.blacklist_radius for old in self.blacklist):
            return True
        for old in self.completed_goals:
            # Successful observation does not permanently exclude a whole
            # neighbourhood. Retain the exact station until new information.
            if math.dist(point,old)>=max(.10,self.grid.resolution*.5):
                continue
            previous=getattr(self,"completed_goal_evidence",{}).get(old)
            if previous is not None:
                current,unknown=self.completion_neighbourhood(old)
                if len(current-previous)>=8 and unknown>=8:
                    continue
            return True
        return False

    def no_frontier_status(self, inflated):
        if self.grid.frontier_cells(set()):
            return "FRONTIERS_BLOCKED_BY_CLEARANCE"
        # No visible boundary does not prove exploration complete while the
        # map still contains unobserved space behind projected obstacles.
        if np.any(self.grid.data==UNKNOWN):
            return "FRONTIER_OBSERVATION_INCOMPLETE"
        return "EXPLORATION_COMPLETE"

    def add_goal_failure_cooldown(self, point: Point2, reason: str):
        release_update = self.map_update_count + self.goal_failure_cooldown_updates
        previous = self.goal_failure_cooldowns.get(point)
        failure_content_revision = self.map_content_revision
        failure_route_revision = self.route_constraint_revision
        if previous is not None:
            release_update = max(release_update, previous[0])
            failure_content_revision = max(
                failure_content_revision, previous[1])
            failure_route_revision = max(
                failure_route_revision, previous[2])
        self.goal_failure_cooldowns[point] = (
            release_update, failure_content_revision,
            failure_route_revision)
        self.get_logger().warning(
            f"[GOAL_FAILURE_COOLDOWN] goal=({point[0]:.2f},{point[1]:.2f}) "
            f"reason={reason} release_update={release_update} "
            f"failure_content_revision={failure_content_revision} "
            f"failure_route_revision={failure_route_revision}")

    def is_goal_on_failure_cooldown(self, point: Point2) -> bool:
        operational=getattr(self,"local_motion_goal_cooldowns",{})
        if operational:
            for goal in list(operational):
                if self.local_motion_retry_allowed(operational[goal]):
                    del operational[goal]
            # Departure failure describes a motion from the current pose,
            # not a forbidden neighbourhood around the remote viewpoint.
            if any(math.hypot(point[0]-goal[0],point[1]-goal[1]) <
                   (self.blacklist_radius if attempt.reason=='STATION_CAPTURE_INCOMPLETE'
                    else max(0.05, self.grid.resolution*.5))
                   for goal,attempt in operational.items()):
                return True
        expired = [
            goal
            for goal, (release_update, failure_content_revision,
                       failure_route_revision)
            in self.goal_failure_cooldowns.items()
            if (self.map_update_count >= release_update
                and (self.map_content_revision > failure_content_revision
                     or self.route_constraint_revision
                     > failure_route_revision))]
        for goal in expired:
            del self.goal_failure_cooldowns[goal]
        locally_cooled = any(
            math.hypot(point[0] - goal[0], point[1] - goal[1])
            < self.blacklist_radius
            for goal in self.goal_failure_cooldowns)
        globally_constrained = any(
            math.hypot(point[0] - x, point[1] - y) < radius
            for x, y, radius in
            getattr(self, "global_viewpoint_constraints", {}).values())
        return locally_cooled or globally_constrained

    def add_region_failure_cooldown(self, region_id: int, reason: str):
        release_update = (
            self.map_update_count + self.region_failure_cooldown_updates)
        previous = self.region_failure_cooldowns.get(region_id)
        failure_revision = self.map_content_revision
        if previous is not None:
            release_update = max(release_update, previous[0])
            failure_revision = max(failure_revision, previous[1])
        self.region_failure_cooldowns[region_id] = (
            release_update, failure_revision)
        self.get_logger().warning(
            f"[REGION_FAILURE_COOLDOWN] region={region_id} reason={reason} "
            f"release_update={release_update} "
            f"failure_revision={failure_revision}")

    def prune_region_failure_cooldowns(self):
        expired = [
            region_id
            for region_id, (release_update, failure_revision)
            in self.region_failure_cooldowns.items()
            if (self.map_update_count >= release_update
                and self.map_content_revision > failure_revision)]
        for region_id in expired:
            del self.region_failure_cooldowns[region_id]

    def is_region_on_failure_cooldown(self, region_id: int) -> bool:
        self.prune_region_failure_cooldowns()
        return region_id in self.region_failure_cooldowns

    @with_planning_grid_snapshot
    def exploration_timer(self):
        now_ns = self.get_clock().now().nanoseconds
        dt = max(0.0, (now_ns - self.last_task_clock_ns) * 1e-9)
        self.last_task_clock_ns = now_ns
        self.evaluate_branch_commitment()
        if self.continue_pending_observation_handoff():
            return  # Lifecycle timeout/cancellation is independent of data/health.
        if not self.locomotion_ready():
            self.publish_status("PAUSED_ROBOT_UNHEALTHY")
            return
        if self.active_goal is not None:
            self.active_task_elapsed = getattr(
                self, "active_task_elapsed", 0.0) + dt
            if not self.scan_waiting_for_target:
                self.healthy_task_elapsed += dt
        if not self.auto_start or self.position is None or self.map_update_count < 2:
            return
        # Information completion owns task handoff. Neither an outstanding
        # Local request nor repair of G1's obsolete route may postpone G2.
        if self.active_goal is not None:
            task = self.evaluate_active_observation()
            if task is not None and getattr(task,"motion_released",False):
                # This return used to bypass goal_timeout forever on a static
                # map. Retire motion, not the frontier or its information need.
                if (self.active_task_elapsed > getattr(self, "goal_timeout", 120.0)
                        and not self.observation_complete(task)):
                    self.goal_timeout_count = getattr(self, "goal_timeout_count", 0) + 1
                    self.get_logger().warning(
                        f"[OBSERVATION_HANDOFF_WAIT_TIMEOUT] task_id={task.task_id} "
                        f"task_version={task.task_version} elapsed={self.active_task_elapsed:.2f}; "
                        "retiring held execution without blacklisting frontier or region")
                    self.terminate_active_observation(
                        "HANDOFF_WAIT_TIMEOUT", "REARBITRATING_FRONTIER")
                    return
                self.handoff_satisfied_observation("OBSERVATION_STATION_MOTION_RELEASED")
                return
            if self.observation_complete(task):
                self.handoff_satisfied_observation("OBSERVATION_HANDOFF_COMPLETE")
                return
            # Successor preparation depends on observation evidence, not on
            # a route-repair tick. A Local request/ACK wait must not postpone it.
            if (getattr(self, "observation_preplanning_enabled", False)
                    and task is not None
                    and task.progress >= self.observation_prepare_ratio
                    and self.prepared_candidate is None
                    and getattr(self, "blocked_execution_context", None) is None):
                self.prepare_next_observation()
        if (self.active_goal is not None
                and getattr(self, "blocked_execution_context", None) is None
                and self.active_task_elapsed > self.goal_timeout):
            self.goal_timeout_count += 1
            self.get_logger().warning("Frontier goal timed out; blacklisting it")
            self.terminate_active_observation(
                "GOAL_TIMEOUT", blacklist_goal=True)
            return
        if self.pending_path_publish_ns:
            if self.pending_path_deferred:
                deferred_age = ((self.get_clock().now().nanoseconds
                                 - self.pending_path_publish_ns) * 1e-9)
                if not self.deferred_wait_expired():
                    self.get_logger().info(
                        f"[PATH_REQUEST_DEFERRED_WAIT] request_id="
                        f"{self.pending_path_request_generation}; waiting for "
                        "the active SCAN recovery transaction to terminate",
                        throttle_duration_sec=2.0)
                    return
                request_id = self.pending_path_request_generation
                self.get_logger().error(
                    f"[PATH_REQUEST_DEFERRED_TIMEOUT] request_id={request_id} "
                    f"age={deferred_age:.2f}s limit={self.deferred_path_timeout:.2f}s; "
                    "retiring the timed-out transaction without geometric failure evidence")
                # No reply says nothing about the remote viewpoint or region.
                # Cancel/fence an active task; do not synthesize a path rejection
                # that would blacklist its goal or consume a motion family.
                self.pending_path_publish_ns = 0
                self.pending_path_deferred = False
                self.pending_path_request_generation = None
                self.active_path_request_generation = None
                self.terminate_active_observation("DEFERRED_REQUEST_TIMEOUT")
                self.publish_status("PATH_REQUEST_DEFERRED_TIMEOUT")
                return
            request_age = ((self.get_clock().now().nanoseconds
                            - self.pending_path_publish_ns) * 1e-9)
            request_limit=self.motion_request_timeout()
            if request_age >= request_limit:
                request_id = self.pending_path_request_generation
                self.get_logger().error(
                    f"[PATH_REQUEST_TIMEOUT] request_id={request_id} "
                    f"age={request_age:.2f}s limit={request_limit:.2f}s; "
                    "ending the request instead of waiting indefinitely")
                timeout = String()
                timeout.data = (
                    "REFERENCE_PATH_REJECTED"
                    + ("" if request_id is None
                       else f" request_id={request_id}")
                    + " reason=TIMEOUT")
                if getattr(self, "task_mode", False) and self.active_observation is not None:
                    task = self.active_observation
                    timeout.data += f" task_id={task.task_id} task_version={task.task_version}"
                self.planning_status_callback(timeout)
                return
            self.get_logger().info(
                f"[PATH_REQUEST_IN_FLIGHT] age={request_age:.2f}s; "
                "waiting for SCAN before publishing another reference path",
                throttle_duration_sec=2.0)
            return
        if getattr(self, "task_mode", False) and self.continue_blocked_execution():
            return
        if getattr(self, "task_mode", False) and self.continue_observation_station():
            return
        if self.handle_local_execution_finish():
            return
        retry_deadline = retry_epoch(now_ns, self.replan_retry_period)
        decision_context = (
            self.map_content_revision, self.route_constraint_revision,
            self.grid.world_to_cell(*self.position),
            getattr(self.active_observation, "task_id", 0),
            getattr(self.active_observation, "task_version", 0),
            self.active_path_request_generation, self.replacement_pending)
        if (decision_context == self.last_decision_context
                and retry_deadline == self.last_decision_retry_epoch):
            self.skipped_decision_ticks += 1
            return
        self.last_decision_context = decision_context
        self.last_decision_retry_epoch = retry_deadline
        if self.active_goal is not None:
            if not self.validate_and_repair_active_path():
                return
            if self.replacement_pending:
                self.replace_unsatisfied_observation()
                return
            else:
                task = self.evaluate_active_observation()
                if self.observation_complete(task):
                    self.get_logger().info(
                        f"Observation satisfied ({task.completion_reason}): "
                        f"{task.observed_cells}/"
                        f"{len(task.target_cells)} cells ({task.progress:.1%}); "
                        "switching before viewpoint arrival")
                    self.handoff_satisfied_observation(
                        "OBSERVATION_HANDOFF_COMPLETE")
                    return
                return

        self.no_active_goal_replan_count += 1
        self.plan_from_current_position()

    def plan_from_current_position(
            self, excluded_goals: Sequence[Point2] = (),
            required_scope: Optional[Tuple[str, int]] = None,
            allow_region_release: bool = True) -> bool:
        """Select and publish a new terminal frontier from the robot position."""
        if self.position is None:
            return False
        if self.coarse_repair_pending():
            return self.activate_prepared_observation()
        # Recovery failure is not an Explorer body/turn certificate.
        if not self.local_departure_replan_allowed():
            return False

        departure_context=(self.local_execution_context(),self.map_content_revision,
                           self.route_constraint_revision)
        if getattr(self,"exhausted_departure_context",None)==departure_context:
            self.publish_status("LOCAL_NAVIGATION_BLOCKED waiting_for=related_input_or_new_work")
            return False
        now_ns = self.get_clock().now().nanoseconds
        context = ReplanContext(
            self.grid.world_to_cell(*self.position),
            self.map_content_revision,
            self.route_constraint_revision,
            retry_epoch(now_ns, self.replan_retry_period))
        frontier_signature = (self.latest_frontier_count, self.map_content_revision)
        unreachable_key = (
            context.robot_cell, context.map_content_revision,
            context.route_constraint_revision, self.locomotion_epoch,
            frontier_signature)
        # Diagnostic memo only. Retry cadence stays owned by ReplanGate, whose
        # context carries a deadline bucket independent of sensor-frame count. A
        # robot that is healthy but stationary cannot change map_content_revision,
        # so suppressing on this latch alone would block every retry and idle
        # forever instead of reporting a stall.
        if self.unreachable_context == unreachable_key:
            self.publish_status("FRONTIERS_PRESENT_BUT_UNREACHABLE")
        else:
            self.unreachable_context = None
        if not self.replan_gate.allow(context):
            self.replan_suppressed_count += 1
            self.publish_status("WAITING_FOR_PLANNING_INPUT_CHANGE")
            self.get_logger().info(
                f"[GLOBAL_REPLAN_SUPPRESSED] reason="
                f"{self.last_active_goal_clear_reason} cell={context.robot_cell} "
                f"map_content_revision={context.map_content_revision} "
                f"route_constraint_revision={context.route_constraint_revision}",
                throttle_duration_sec=2.0)
            return False
        self.unreachable_context = None

        preprocess_started = time.perf_counter()
        inflated = set()  # raw FREE coarse connection; Local owns body clearance
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        if self.grid.value(start) != FREE:
            self.publish_status("START_NOT_KNOWN_FREE")
            self.get_logger().warning(
                f"[FRONTIER_REACHABILITY] start={start} raw_start_free="
                f"{self.grid.in_bounds(start) and self.grid.value(start) == FREE} "
                "coarse_connection=unconfirmed",
                throttle_duration_sec=2.0)
            return False
        frontiers = self.grid.frontier_cells(inflated)
        clusters = cluster_frontiers(frontiers, self.min_frontier_size)
        filtered_frontiers = clustered_frontier_cells(clusters)
        self.latest_frontier_count = len(filtered_frontiers)
        preprocess_ms = (
            time.perf_counter() - preprocess_started) * 1000.0
        self.publish_frontiers(filtered_frontiers)
        best = self.choose_frontier(
            start, clusters, inflated, blocked_edges,
            excluded_goals=excluded_goals,
            required_scope=required_scope,
            allow_region_release=allow_region_release,
            # Completion permits selecting a standby outside the old scope;
            # activate_candidate still stages it and waits for Local readiness.
            allow_cross_region_preparation=(
                getattr(self, 'task_mode', False) and
                getattr(self, 'completion_handoff_in_progress', False) and
                getattr(self, 'branch_commitment', None) is None),
            map_preprocess_prefix_ms=preprocess_ms)
        if best is None:
            if getattr(self, 'region_visit_deferred', False):
                self.publish_status("REGION_VISIT_SEARCH_PENDING")
                return False
            if any(a.reason=="DEPARTURE_EPISODE_EXHAUSTED" and
                   not self.local_motion_retry_allowed(a)
                   for a in getattr(self,"local_motion_goal_cooldowns",{}).values()):
                self.exhausted_departure_context=departure_context
                self.publish_status("LOCAL_NAVIGATION_BLOCKED waiting_for=related_input_or_new_work")
                return False
            if self.coarse_repair_pending():
                self.publish_status("WAITING_FOR_COARSE_ROUTE_REPAIR")
                return False
            status = (self.no_frontier_status(inflated) if not filtered_frontiers
                      else "FRONTIERS_PRESENT_BUT_UNREACHABLE")
            if filtered_frontiers:
                self.unreachable_context = (
                    context.robot_cell, context.map_content_revision,
                    context.route_constraint_revision, self.locomotion_epoch,
                    (len(filtered_frontiers), self.map_content_revision))
            self.publish_status(status)
            self.get_logger().info(
                f"{status}: {len(filtered_frontiers)} usable frontier cells",
                throttle_duration_sec=5.0)
            return False
        raw_path = best.path
        goal_xy = best.goal
        simplified = simplify_known_path(
            self.grid, raw_path, inflated, blocked_edges)
        # Every published segment needs a raw FREE witness, not a body
        # certificate. Unknown and occupied cells still fail closed.
        if not simplified or not all(
                segment_known_free(self.grid, a, b, inflated, blocked_edges)
                for a, b in zip(simplified[:-1], simplified[1:])):
            self.publish_status("REFERENCE_PATH_REJECTED")
            self.get_logger().warning("Rejected frontier path during final safety validation")
            return False
        return self.activate_candidate(simplified, best)

    def choose_frontier(self, start: Cell, clusters: Sequence[Sequence[Cell]],
                        inflated: Set[Cell],
                        blocked_edges: Set[DirectedEdge],
                        excluded_goals: Sequence[Point2] = (),
                        allow_region_release: bool = True,
                        allow_cross_region_preparation: bool = False,
                        update_region_prediction: bool = False,
                        map_preprocess_prefix_ms: float = 0.0,
                        required_scope: Optional[Tuple[str, int]] = None):
        inflated = set()  # caller inflation is not a publication veto
        selection_started = time.perf_counter()
        self.last_candidate_tree_ms = 0.0
        self.last_region_sequence_ms = 0.0
        self.last_candidate_tree_searches = 0
        self.last_region_pair_tree_searches = 0
        self.last_expanded_grid_cells = 0
        self.last_region_edge_cache_hits = 0
        self.last_region_edge_cache_misses = 0
        self.last_sparse_candidate_ms = 0.0
        self.last_sparse_region_ms = 0.0
        self.last_sparse_candidate_hits = 0
        self.last_sparse_candidate_fallbacks = 0
        self.last_sparse_region_pair_hits = 0
        self.last_sparse_region_pair_fallbacks = 0
        self.last_sparse_candidate_miss_reasons = dict.fromkeys(
            SPARSE_MISS_REASONS, 0)
        self.last_sparse_region_miss_reasons = dict.fromkeys(
            SPARSE_MISS_REASONS, 0)
        self.ensure_sparse_miss_counters()

        partition_started = time.perf_counter()

        def connector_allowed(source: Point2, target: Point2) -> bool:
            source_cell = self.grid.world_to_cell(*source)
            target_cell = self.grid.world_to_cell(*target)
            return segment_known_free(
                self.grid, source_cell, target_cell, inflated)
        topology_keys = []
        for cluster in clusters:
            if not cluster or not self.sparse_routing_enabled:
                topology_keys.append(None)
                continue
            centroid_x = sum(cell[0] for cell in cluster) / len(cluster)
            centroid_y = sum(cell[1] for cell in cluster) / len(cluster)
            frontier = min(cluster, key=lambda cell: (
                (cell[0] - centroid_x) ** 2 + (cell[1] - centroid_y) ** 2,
                cell[0], cell[1]))
            viewpoints = safe_viewpoint_cells(
                self.grid, frontier, inflated,
                self.viewpoint_standoff, limit=1,
                clearance_search_radius=self.terminal_clearance_search_radius,
                relaxation=self.viewpoint_relaxation,
                footprint_radius=0.0,
                footprint_offset=0.0)
            representative = viewpoints[0] if viewpoints else frontier
            attachment = self.sparse_router.topology_attachment(
                self.grid.cell_to_world(representative),
                self.sparse_attachment_radius,
                self.sparse_attachment_limit,
                connector_allowed)
            topology_keys.append(
                (attachment.component_id, attachment.branch_id)
                if attachment is not None else None)
        self.last_topology_attached_clusters = sum(
            key is not None for key in topology_keys)
        self.last_topology_unattached_clusters = (
            len(topology_keys) - self.last_topology_attached_clusters)
        observations = partition_frontier_clusters_by_topology(
            clusters, topology_keys,
            int(round(self.region_size / self.grid.resolution)),
            self.grid, inflated, self.region_max_path_ratio,
            self.region_max_detour_ratio)
        regions = self.region_tracker.update(observations, self.active_region_id)
        self.last_region_partition_ms = (
            time.perf_counter() - partition_started) * 1000.0
        self.last_map_preprocess_ms = (
            map_preprocess_prefix_ms
            + self.last_region_partition_ms)
        self.map_preprocess_calls += 1
        self.planning_events += 1
        self.latest_region_count = len(regions)
        if self.bounded_visits_enabled():
            result = self.choose_window_visit(
                start, regions, blocked_edges, excluded_goals,
                allow_region_release=allow_region_release,
                allow_cross_region_preparation=allow_cross_region_preparation,
                required_scope=required_scope)
            self.last_global_plan_ms = map_preprocess_prefix_ms + (
                time.perf_counter()-selection_started)*1000.
            self.cumulative_planning_ms += self.last_global_plan_ms
            self.get_logger().info(
                f"[GLOBAL_PLAN_BASELINE] revision={self.map_update_count} "
                f"task_id={getattr(self.active_observation, 'task_id', 0)} "
                f"regions={len(regions)} candidates={self.last_visit_stats.get('candidates',0)} "
                f"total_ms={self.last_global_plan_ms:.1f} "
                f"region_partition_ms={self.last_region_partition_ms:.1f} "
                f"expanded_grid_cells={self.last_expanded_grid_cells} "
                f"planner=bounded_region_visits window={self.last_visit_window}")
            return result
        candidates = self.build_frontier_candidates(
            start, regions, inflated, blocked_edges, excluded_goals)
        self.attach_safe_region_commitment_scopes(candidates, inflated)
        for candidate in candidates:
            hypothesis = self.safe_region_connectivity.hypothesis(
                self.grid.cell_to_world(candidate.frontier_cell))
            if hypothesis is None:
                continue
            # Convert avoided travel metres to avoided dense-grid steps.  This
            # keeps coverage information and connectivity information in the
            # same planning-effort unit without a free weighting constant.
            candidate.connectivity_gain = int(round(
                hypothesis.route_savings / self.grid.resolution))
            candidate.provisional_portal = (
                hypothesis.source_id, hypothesis.target_id)

        def select_candidate(options: Sequence[FrontierCandidate]):
            if required_scope is not None:
                options = [candidate for candidate in options
                           if candidate.commitment_scope == required_scope]
                # This is a bounded sibling attempt, not a region ownership
                # decision. Prefer the frozen task before other same-scope tasks.
                task = getattr(self, "active_observation", None)
                same_task = [candidate for candidate in options
                             if isinstance(task, ObservationTask)
                             and candidate.frontier_cell == task.frontier_cell
                             and candidate.region_id == task.region_id]
                return self.choose_greedy_candidate(same_task or options)
            if self.selection_strategy == "greedy":
                return self.choose_greedy_candidate(options)
            return self.choose_hierarchical_candidate(
                start, regions, options, inflated, blocked_edges,
                allow_region_release, allow_cross_region_preparation,
                update_region_prediction)

        result = select_candidate(candidates)
        while result is not None and not result.path:
            rejected_region_id = result.region_id
            materialized = self.materialize_sparse_candidate(
                start, result, inflated, blocked_edges)
            if materialized is None and self.coarse_repair_pending():
                result = None
                break  # capacity deferral is not candidate/region rejection
            materialized_scope = (
                result.commitment_scope
                or next((region_commitment_scope(region)
                         for region in regions
                         if region.region_id == result.region_id),
                        ("region", result.region_id)))
            if (materialized is not None
                    and self.directional_candidate_allowed(
                        materialized, materialized_scope)):
                result = materialized
                break
            if materialized is not None:
                self.get_logger().info(
                    f"[DIRECTIONAL_DENSE_ROUTE_REJECTED] region="
                    f"{result.region_id}; dense route crosses the active "
                    "no-return gate",
                    throttle_duration_sec=2.0)
            candidates = [candidate for candidate in candidates
                          if candidate is not result]
            if allow_region_release and required_scope is None:
                self.handle_dense_invalid_region(
                    rejected_region_id, candidates)
            result = select_candidate(candidates)

        self.last_global_plan_ms = (
            map_preprocess_prefix_ms
            + (time.perf_counter() - selection_started) * 1000.0)
        self.cumulative_planning_ms += self.last_global_plan_ms
        self.get_logger().info(
            f"[GLOBAL_PLAN_BASELINE] revision={self.map_update_count} "
            f"task_id={getattr(self.active_observation, 'task_id', 0)} "
            f"regions={len(regions)} candidates={len(candidates)} "
            f"map_preprocess_ms={self.last_map_preprocess_ms:.1f} "
            f"inflation_ms={self.grid.last_inflation_ms:.2f} "
            f"inflation_cache_hits={self.grid.inflation_cache_hits} "
            f"inflation_cache_misses={self.grid.inflation_cache_misses} "
            f"region_partition_ms={self.last_region_partition_ms:.1f} "
            f"sparse_candidate_ms={self.last_sparse_candidate_ms:.1f} "
            f"candidate_tree_ms={self.last_candidate_tree_ms:.1f} "
            f"sparse_region_ms={self.last_sparse_region_ms:.1f} "
            f"region_sequence_ms={self.last_region_sequence_ms:.1f} "
            f"total_ms={self.last_global_plan_ms:.1f} "
            f"candidate_tree_searches={self.last_candidate_tree_searches} "
            f"region_pair_tree_searches={self.last_region_pair_tree_searches} "
            f"region_cache_hits={self.last_region_edge_cache_hits} "
            f"region_cache_misses={self.last_region_edge_cache_misses} "
            f"sparse_candidate_hits={self.last_sparse_candidate_hits} "
            f"sparse_candidate_fallbacks={self.last_sparse_candidate_fallbacks} "
            f"sparse_candidate_miss="
            f"{self.last_sparse_candidate_miss_reasons} "
            f"sparse_region_hits={self.last_sparse_region_pair_hits} "
            f"sparse_region_fallbacks={self.last_sparse_region_pair_fallbacks} "
            f"sparse_region_miss={self.last_sparse_region_miss_reasons} "
            f"expanded_grid_cells={self.last_expanded_grid_cells}")
        return result

    def handle_dense_invalid_region(
            self, rejected_region_id: int,
            remaining_candidates: Sequence[FrontierCandidate]) -> bool:
        """Remember a region whose candidates all failed this dense revision."""
        if any(candidate.region_id == rejected_region_id
               for candidate in remaining_candidates):
            return False
        self.get_logger().warning(
            f"[FRONTIER_CONNECTION_UNCONFIRMED] region={rejected_region_id} "
            "frontier_retained=true region_blocked=false",
            throttle_duration_sec=2.0)
        return False

    def materialize_sparse_candidate(
            self, start: Cell, candidate: FrontierCandidate,
            inflated: Set[Cell], blocked_edges: Set[DirectedEdge]
            ) -> Optional[FrontierCandidate]:
        """Use a safe sparse route, falling back to dense search when needed."""
        inflated = set()  # caller inflation is not a publication veto
        self.dense_final_validation_searches += 1
        selected_cell = candidate.cell
        selected_goal = candidate.goal
        selected_clearance = candidate.terminal_clearance
        selected_sparse_route = None
        path = None
        task = getattr(self, "active_observation", None)
        same_task = (isinstance(task, ObservationTask)
                     and task.region_id == candidate.region_id
                     and task.frontier_cell == candidate.frontier_cell)
        observation_cells = (set(task.target_cells) if same_task
                             else candidate.observation_cells)
        terminals = tuple(dict.fromkeys((candidate.cell,) + candidate.terminal_cells))
        for cell in terminals:
            if candidate.terminal_cells and self.terminal_rejection_reason(
                    cell, inflated, candidate.excluded_goals) is not None:
                continue
            trial = None
            selected_sparse_route = None
            if cell == candidate.cell and candidate.sparse_route and candidate.sparse_route.polyline:
                sparse_cells = [self.grid.world_to_cell(*point)
                                for point in candidate.sparse_route.polyline]
                sparse_cells[0], sparse_cells[-1] = start, cell
                recovered = []
                for first, second in zip(sparse_cells[:-1], sparse_cells[1:]):
                    segment = bresenham(first, second)
                    if recovered and segment and recovered[-1] == segment[0]:
                        segment = segment[1:]
                    recovered.extend(segment)
                if len(sparse_cells) == 1 and start == cell:
                    recovered = [start]
                if adjacent_grid_path_is_valid(self.grid, recovered, inflated, blocked_edges):
                    trial = recovered
                    selected_sparse_route = candidate.sparse_route
                else:
                    result = self.repair_rejected_global_path(
                        start, recovered, cell, candidate.frontier_cell, inflated, blocked_edges)
                    if result.status == RouteSearchStatus.INCOMPLETE:
                        candidate.path = recovered
                        self.prepared_candidate = candidate
                        return None
                    if result.status == RouteSearchStatus.FOUND:
                        trial = result.path
            if not trial:
                trial = astar_known(self.grid, start, cell, inflated, blocked_edges)
            if not trial:
                continue
            targets = observation_cells
            if same_task:
                visible_remaining = sum(
                    self.grid.value(target) == UNKNOWN and
                    not any(self.grid.value(ray) == OCCUPIED
                            for ray in bresenham(cell, target)[1:-1])
                    for target in targets)
                if visible_remaining == 0:
                    continue
            elif cell != candidate.cell:
                targets = observation_target_cells(
                    self.grid, candidate.frontier_cell, self.observation_radius, cell)
                if len(targets) < self.min_expected_observation_cells:
                    continue
            selected_cell, path = cell, trial
            selected_goal = self.grid.cell_to_world(cell)
            if cell != candidate.cell:
                selected_clearance = self.terminal_clearance(cell, inflated)
            observation_cells = targets
            break
        if not path:
            self.sparse_final_validation_failures += 1
            # A failed geometric realization is not persistent evidence
            # against this viewpoint or its frontier.
            self.get_logger().warning(
                f"[SPARSE_ROUTE_REJECTED] goal={candidate.goal}; all eligible "
                "terminals lack a known-free coarse connection",
                throttle_duration_sec=2.0)
            return None
        if candidate.sparse_route is not None:
            metric = ("sparse_materialized_dense_rescues" if selected_sparse_route is None
                      else "sparse_materialized_connector_routes"
                      if selected_sparse_route.route_source == "direct_connector"
                      else "sparse_materialized_graph_routes")
            setattr(self, metric, getattr(self, metric, 0)+1)
        return FrontierCandidate(
            candidate.region_id, selected_cell, candidate.frontier_cell,
            selected_goal, path,
            path_length_cells(path, self.grid.resolution),
            candidate.unknown_gain, candidate.cluster_size,
            path_turn_cost(path), observation_cells,
            sparse_route=selected_sparse_route,
            terminal_cells=tuple(dict.fromkeys((selected_cell,) + candidate.terminal_cells)),
            terminal_clearance=selected_clearance,
            frontier_cluster_cells=set(candidate.frontier_cluster_cells),
            connectivity_gain=candidate.connectivity_gain,
            provisional_portal=candidate.provisional_portal,
            commitment_scope=candidate.commitment_scope,
            option_target_cells=set(candidate.option_target_cells),
            excluded_goals=candidate.excluded_goals,
            observation_progress=candidate.observation_progress,
            route_source=(selected_sparse_route.route_source if selected_sparse_route is not None
                          else "dense_rescue" if candidate.sparse_route is not None else "dense"))

    def terminal_rejection_reason(
            self, cell: Cell, inflated: Set[Cell],
            excluded_goals: Sequence[Point2] = ()) -> Optional[str]:
        """Apply the same pose constraints to primary and sibling terminals."""
        if self.grid.value(cell) != FREE:
            return "no_safe_viewpoint"
        goal = self.grid.cell_to_world(cell)
        exclusion_radius = (max(.05, self.grid.resolution*.5)
                            if getattr(self, 'task_geometry_autonomy', False)
                            else self.blacklist_radius)
        if any(math.hypot(goal[0] - point[0], goal[1] - point[1])
               < exclusion_radius for point in excluded_goals):
            return "excluded"
        if self.is_blacklisted(goal):
            return "blacklisted"
        if self.is_goal_on_failure_cooldown(goal):
            return "cooldown"
        if (math.hypot(goal[0] - self.position[0], goal[1] - self.position[1])
                < self.min_goal_distance):
            return "too_close"
        return None

    def build_frontier_candidates(self, start: Cell,
                                  regions: Sequence[FrontierRegion],
                                  inflated: Set[Cell],
                                  blocked_edges: Set[DirectedEdge],
                                  excluded_goals: Sequence[Point2] = (), *,
                                  frontier_limit=None, station_limit=None,
                                  route_provider=None, preview_regions=()) -> List[FrontierCandidate]:
        inflated = set()  # caller inflation is not a publication veto
        rejected = dict.fromkeys(self.last_candidate_rejections, 0)
        self.last_candidate_rejections = rejected
        outcomes = []
        self._repair_expansion_budget = ExpansionBudget(
            getattr(self, "global_repair_search_limit", 4)*getattr(self, "global_repair_max_expansions", 400))
        proposals = []
        for region in regions:
            clusters = region.clusters
            sampled = None
            if frontier_limit is not None:
                window = self.visit_window()
                choices = [candidate_cells(cluster) for cluster in clusters]
                flat = [f for i in range(max((len(c) for c in choices), default=0))
                        for c in choices if i < len(c) for f in [c[i]]]
                retained = list(window.prefix if window.prefix_region == region.region_id else ())
                task = getattr(self, "active_observation", None)
                if isinstance(task, ObservationTask) and task.region_id == region.region_id:
                    retained.insert(0, task.frontier_cell)
                valid = set().union(*(set(c) for c in clusters))
                retained = [f for f in retained if f in valid]
                cursor = window.sample_cursor.get(region.region_id, 0)
                rotated = flat[cursor % len(flat):]+flat[:cursor % len(flat)] if flat else []
                sampled = list(dict.fromkeys(retained+rotated))[:frontier_limit]
                window.sample_cursor[region.region_id] = cursor+frontier_limit
            for cluster in clusters:
                used_viewpoints: Set[Cell] = set()
                task = getattr(self, "active_observation", None)
                frontiers = list(candidate_cells(cluster))
                # Sampling may change as a boundary shrinks. Keep the frozen
                # task represented while its cluster still belongs to this region.
                if (isinstance(task, ObservationTask)
                        and task.region_id == region.region_id
                        and task.frontier_cell is not None
                        and (task.frontier_cell in cluster
                             or task.frontier_cluster_cells.intersection(cluster))):
                    frontiers = list(dict.fromkeys([task.frontier_cell] + frontiers))
                if sampled is not None:
                    frontiers = [f for f in sampled if f in cluster]
                for frontier in frontiers:
                    viewpoints = safe_viewpoint_cells(
                        self.grid, frontier, inflated, self.viewpoint_standoff,
                        limit=self.terminal_candidate_limit,
                        clearance_search_radius=(
                            self.terminal_clearance_search_radius),
                        relaxation=self.viewpoint_relaxation,
                        footprint_radius=0.0,
                        footprint_offset=0.0)
                    same_task = (isinstance(task, ObservationTask)
                                 and task.frontier_cell == frontier
                                 and task.region_id == region.region_id)
                    if same_task:
                        # Previously safe terminals are frozen task alternatives,
                        # not trusted geometry: recheck LOS, pose and route below.
                        retained = [cell for cell in task.terminal_cells
                                    if segment_known_free(
                                        self.grid, cell, frontier, inflated)]
                        viewpoints = list(dict.fromkeys(viewpoints + retained))
                    if not viewpoints:
                        rejected["no_safe_viewpoint"] += 1
                        outcomes.append({"region": region.region_id,
                                         "viewpoint": None,
                                         "frontier": frontier,
                                         "reason": "no_safe_viewpoint"})
                        continue
                    # Filter every terminal before sparse/dense routing. A rejected
                    # safest pose must not hide a safe sibling of this frontier.
                    terminals = []
                    for viewpoint in viewpoints:
                        reason = self.terminal_rejection_reason(
                            viewpoint, inflated, excluded_goals)
                        if reason is not None:
                            if reason in rejected:
                                rejected[reason] += 1
                            outcomes.append({"region": region.region_id,
                                             "viewpoint": viewpoint,
                                             "frontier": frontier,
                                             "reason": reason})
                            continue
                        terminals.append(viewpoint)
                    if station_limit is not None:
                        terminals = terminals[:station_limit]
                    for viewpoint in terminals:
                        if viewpoint in used_viewpoints:
                            continue
                        used_viewpoints.add(viewpoint)
                        target_cells = (set(task.target_cells) if same_task else
                                        observation_target_cells(
                                            self.grid, frontier,
                                            self.observation_radius, viewpoint))
                        unknown_gain = sum(self.grid.value(cell) == UNKNOWN
                                           for cell in target_cells)
                        if same_task:
                            visible_remaining = sum(
                                self.grid.value(cell) == UNKNOWN and
                                not any(self.grid.value(ray) == OCCUPIED
                                        for ray in bresenham(viewpoint, cell)[1:-1])
                                for cell in target_cells)
                            if visible_remaining == 0:
                                rejected["insufficient_gain"] += 1
                                outcomes.append({
                                    "region": region.region_id,
                                    "viewpoint": viewpoint,
                                    "frontier": frontier,
                                    "reason": "insufficient_gain"})
                                continue
                        if (not same_task and unknown_gain
                                < self.min_expected_observation_cells):
                            rejected["insufficient_gain"] += 1
                            outcomes.append({
                                "region": region.region_id,
                                "viewpoint": viewpoint,
                                "frontier": frontier,
                                "reason": "insufficient_gain"})
                            continue
                        proposals.append((
                            region.region_id, region_commitment_scope(region),
                            viewpoint, frontier, self.grid.cell_to_world(viewpoint),
                            unknown_gain, len(cluster), target_cells,
                            tuple(terminals),
                            self.terminal_clearance(viewpoint, inflated),
                            set(cluster)))

        visit_paths = {}
        if route_provider is not None:
            estimate_by_cell = {}
            for proposal in proposals:
                rid, _, cell = proposal[:3]
                if rid in preview_regions:
                    continue
                if cell in visit_paths:
                    continue
                result = route_provider.query(start, cell)
                visit_paths[cell] = result
                if result.status == RouteSearchStatus.INCOMPLETE:
                    self._visit_pending_regions.add(rid)
                    if result.reason == 'SEARCH_CACHE_CAPACITY':
                        self._visit_capacity_regions.add(rid)
        else:
            proposal_cells = list(dict.fromkeys(proposal[2] for proposal in proposals))
            sparse_started = time.perf_counter()
            sparse_estimates, sparse_miss_reasons = self.sparse_route_estimates(
                start, proposal_cells, inflated, blocked_edges)
            sparse_elapsed = (time.perf_counter() - sparse_started) * 1000.0
            self.last_sparse_candidate_ms += sparse_elapsed
            estimate_by_cell = dict(zip(proposal_cells, sparse_estimates))
            sparse_hits = sum(estimate is not None for estimate in sparse_estimates)
            fallback_cells = {cell for cell, estimate in estimate_by_cell.items()
                              if estimate is None}
            self.sparse_candidate_queries += len(proposal_cells)
            self.sparse_candidate_hits += sparse_hits
            self.sparse_candidate_fallbacks += len(fallback_cells)
            self.last_sparse_candidate_hits += sparse_hits
            self.last_sparse_candidate_fallbacks += len(fallback_cells)
            self.ensure_sparse_miss_counters()
            self.accumulate_sparse_miss_reasons(
                sparse_miss_reasons, self.sparse_candidate_miss_reasons,
                self.last_sparse_candidate_miss_reasons)

            tree = ShortestPathTree(start, {}, {}, 0)
            if fallback_cells:
                # Sparse coverage is deliberately non-authoritative.  Missing graph
                # attachments retain the original dense behaviour for completeness.
                tree_started = time.perf_counter()
                tree = build_shortest_path_tree(
                    self.grid, start, inflated, blocked_edges, fallback_cells)
                self.last_candidate_tree_ms += (
                    time.perf_counter() - tree_started) * 1000.0
                self.candidate_tree_searches += 1
                self.last_candidate_tree_searches += 1
                self.expanded_grid_cells += tree.expanded_cells
                self.last_expanded_grid_cells += tree.expanded_cells
        records = []
        task = getattr(self, "active_observation", None)
        recorded_tasks = set()
        for (region_id, commitment_scope, viewpoint, frontier, goal_xy,
             unknown_gain, cluster_size, target_cells, terminal_cells,
             terminal_clearance, frontier_cluster_cells) in proposals:
            estimate = estimate_by_cell.get(viewpoint)
            if route_provider is not None and region_id in preview_regions:
                # These are only bounded lookahead options. No claim of robot
                # reachability until one is promoted and its route is checked.
                path = []
                path_length = math.dist(start, viewpoint)*self.grid.resolution
                turn_cost = 0.
            elif route_provider is not None:
                result = visit_paths[viewpoint]
                if result.status == RouteSearchStatus.INCOMPLETE:
                    outcomes.append({"region": region_id, "viewpoint": viewpoint,
                                     "frontier": frontier, "reason": "search_pending"})
                    continue
                if result.status != RouteSearchStatus.FOUND:
                    rejected["astar_unreachable"] += 1
                    outcomes.append({"region": region_id, "viewpoint": viewpoint,
                                     "frontier": frontier, "reason": "astar_unreachable"})
                    continue
                path = result.path
                path_length = path_length_cells(path, self.grid.resolution)
                turn_cost = path_turn_cost(path)
            elif estimate is None:
                path = tree.path_to(viewpoint)
                if not path:
                    rejected["astar_unreachable"] += 1
                    outcomes.append({"region": region_id,
                                     "viewpoint": viewpoint,
                                     "frontier": frontier,
                                     "reason": "astar_unreachable"})
                    continue
                if not adjacent_grid_path_is_valid(self.grid, path, inflated, blocked_edges):
                    rejected["final_validation_failed"] += 1
                    continue
                path_length = path_length_cells(path, self.grid.resolution)
                turn_cost = path_turn_cost(path)
            else:
                path = []
                path_length = estimate.distance
                turn_cost = estimate.turn_cost
            # One frontier remains one ranked objective, with its safest
            # usable terminal first. Dense fallback has already checked siblings.
            if station_limit is None and (region_id, frontier) in recorded_tasks:
                continue
            recorded_tasks.add((region_id, frontier))
            records.append(FrontierCandidate(
                region_id, viewpoint, frontier, goal_xy, path,
                path_length, unknown_gain, cluster_size, turn_cost, target_cells,
                sparse_route=estimate, terminal_cells=terminal_cells,
                terminal_clearance=terminal_clearance,
                frontier_cluster_cells=frontier_cluster_cells,
                commitment_scope=commitment_scope,
                excluded_goals=tuple(excluded_goals),
                route_source=('visit_preview' if region_id in preview_regions else
                              'visit_bounded' if route_provider is not None else 'dense'),
                observation_progress=(
                    task.progress if isinstance(task, ObservationTask)
                    and task.region_id == region_id
                    and task.frontier_cell == frontier else 0.0)))
            if region_id not in preview_regions:
                rejected["accepted"] += 1
        self.get_logger().info(
            f"[FRONTIER_CANDIDATE_SUMMARY] {rejected}",
            throttle_duration_sec=2.0)
        self.last_candidate_outcomes = outcomes[-400:]
        if route_provider is None:
            self.publish_region_overlay(regions, records)
        return records

    def region_color_map(self, regions):
        """Use the persistent region ID as the sole colour identity.

        The tracker keeps one ID on a split and gives the other children new
        IDs. Sharing lineage or a nearby centroid does not share a colour.
        Reuse the overlay's unbounded deterministic palette so refresh order,
        temporary absence and regions beyond the first twelve cannot recolour
        an existing region. This is display-only; it never changes ownership.
        """
        return {int(region.region_id): region_rgb(int(region.region_id))
                for region in regions}

    def publish_region_overlay(self, regions, accepted):
        """Publish the current region/candidate decision for RViz only."""
        now = time.monotonic()
        if now - getattr(self, "_region_overlay_last_publish", 0.0) < 0.4:
            return
        self._region_overlay_last_publish = now
        if getattr(self, "position", None) is None:
            return
        try:
            colors = self.region_color_map(regions)
        except Exception:  # colour allocation is visualization only
            colors = {}
        try:
            overlay = build_region_overlay(
                frame_id=self.frame_id,
                stamp=self.get_clock().now().to_msg(),
                regions=regions,
                outcomes=getattr(self, "last_candidate_outcomes", ()),
                accepted=accepted,
                robot=self.position,
                active_goal=getattr(self, "active_goal", None),
                prepared=getattr(self, "prepared_candidate", None),
                active_region=getattr(self, "active_region_id", None),
                region_sequence=getattr(self, "region_sequence", ()),
                region_colors=colors,
                resolution=float(self.grid.resolution),
                cell_to_world=self.grid.cell_to_world,
                previous_marker_keys=getattr(self, "_region_overlay_marker_keys", None))
        except Exception as error:  # overlay must never break planning
            self.get_logger().warning(
                f"[REGION_OVERLAY_FAILED] {error}", throttle_duration_sec=5.0)
            return
        self.region_overlay_pub.publish(overlay)
        self._region_overlay_marker_keys = {
            (marker.ns, marker.id) for marker in overlay.markers
            if marker.action == marker.ADD}

    def coarse_repair_pending(self):
        result = getattr(self, "last_coarse_route_result", None)
        return (getattr(self, "prepared_candidate", None) is not None
                and result is not None
                and result.status == RouteSearchStatus.INCOMPLETE)

    def repair_rejected_global_path(self, start, path, terminal, frontier,
                                    inflated, blocked_edges):
        """Repair raw connection conflicts; compute capacity is not geometry."""
        if not hasattr(self, "_coarse_route_repair"):
            self._coarse_route_repair = CoarseRouteRepair()
        # Shared across all repairs in this candidate pass. A later pass resumes
        # the same search rather than resetting its open/closed sets.
        budget = getattr(self, "_repair_expansion_budget", None)
        if budget is None:
            budget = ExpansionBudget(getattr(self, "global_repair_max_expansions", 400))
        result = self._coarse_route_repair.repair(self.grid, path, blocked_edges, budget)
        self.last_coarse_route_result = result
        self.get_logger().info(
            f"[GLOBAL_PATH_CORRECTION] strategy=RAW_CONNECTION_SEGMENT "
            f"frontier={frontier} terminal={terminal} result={result.status.value} "
            f"reason={result.reason} expansions={result.expanded} "
            f"map_content_revision={self.map_content_revision} "
            "requires_local_certification=true", throttle_duration_sec=2.0)
        return result

    @staticmethod
    def planning_gain(candidate) -> int:
        return int(candidate.unknown_gain) + int(
            getattr(candidate, "connectivity_gain", 0))

    @staticmethod
    def candidate_utility(candidate: FrontierCandidate,
                          options: Sequence[FrontierCandidate]) -> float:
        max_gain = max(1, max(
            FrontierExplorer.planning_gain(item) for item in options))
        max_cluster = max(1, max(item.cluster_size for item in options))
        max_length = max(1e-6, max(item.path_length for item in options))
        max_turn = max(1e-6, max(item.turn_cost for item in options))
        return (0.45 * FrontierExplorer.planning_gain(candidate) / max_gain
                + 0.20 * candidate.cluster_size / max_cluster
                - 0.25 * candidate.path_length / max_length
                - 0.10 * candidate.turn_cost / max_turn)

    def preferred_horizon_pool(
            self, options: Sequence[FrontierCandidate]) -> List[FrontierCandidate]:
        """Prefer a useful longer observation path, retaining short fallback goals."""
        if not options:
            return []
        best_gain = max(
            self.planning_gain(candidate)
            for candidate in options)
        minimum_gain = best_gain * self.long_horizon_min_gain_ratio
        longer_useful = [
            candidate for candidate in options
            if candidate.path_length >= self.preferred_goal_path_length
            and self.planning_gain(candidate) >= minimum_gain]
        return longer_useful or list(options)

    def candidate_departure_penalty(self, candidate):
        """Soft access cost from measured heading, not a world-space no-return gate."""
        history=self.local_departure_history_penalty(candidate)
        if not (getattr(self, "task_mode", False) and
                getattr(self, "task_geometry_autonomy", False)):
            return history
        yaw = getattr(self, "body_yaw", None)
        if yaw is None or not math.isfinite(yaw):
            return 0.0
        points = ([self.grid.cell_to_world(cell) for cell in candidate.path]
                  if candidate.path else list(getattr(candidate.sparse_route, "polyline", ())))
        # Use the route's actual departure, not goal minus start on a bent route.
        for point in points:
            dx, dy = point[0]-self.position[0], point[1]-self.position[1]
            if math.hypot(dx, dy) >= 0.5:
                angle = abs(math.remainder(math.atan2(dy, dx)-yaw, 2*math.pi))
                return angle/math.pi+history
        return history

    def choose_greedy_candidate(self, candidates: Sequence[FrontierCandidate]):
        best = None
        for candidate in candidates:
            direct_distance = math.hypot(candidate.goal[0] - self.position[0],
                                         candidate.goal[1] - self.position[1])
            score = self.candidate_utility(candidate, candidates) - 0.25*self.candidate_departure_penalty(candidate)
            key = (score, direct_distance, -candidate.cell[0], -candidate.cell[1])
            if best is None or key > best[0]:
                best = (key, candidate)
        if best is None:
            return None
        selected = best[1]
        selected.option_target_cells = {
            cell
            for candidate in candidates
            if candidate.commitment_scope == selected.commitment_scope
            for cell in candidate.observation_cells}
        return selected

    def choose_hierarchical_candidate(self, start: Cell,
                                      regions: Sequence[FrontierRegion],
                                      candidates: Sequence[FrontierCandidate],
                                      inflated: Set[Cell],
                                      blocked_edges: Set[DirectedEdge],
                                      allow_region_release: bool = True,
                                      allow_cross_region_preparation: bool = False,
                                      update_region_prediction: bool = False):
        intent = getattr(self, "branch_commitment", None)
        if intent is not None:
            domain = self.branch_domain(intent)
            for candidate in candidates:
                if intent.contains(self.grid, {candidate.frontier_cell}, domain):
                    candidate.commitment_scope = self.active_commitment_scope
        all_by_region: Dict[int, List[FrontierCandidate]] = {}
        for candidate in candidates:
            all_by_region.setdefault(candidate.region_id, []).append(candidate)
        self.prune_region_failure_cooldowns()
        by_region = {
            region_id: options
            for region_id, options in all_by_region.items()
            if region_id not in self.region_failure_cooldowns}
        fallback_scope_by_region = {
            region.region_id: region_commitment_scope(region)
            for region in regions}
        scope_by_region = {}
        for region in regions:
            scopes = {
                candidate.commitment_scope
                for candidate in all_by_region.get(region.region_id, [])
                if candidate.commitment_scope is not None}
            scope_by_region[region.region_id] = (
                next(iter(scopes)) if len(scopes) == 1
                else fallback_scope_by_region[region.region_id])
            if intent is not None and self.active_commitment_scope in scopes:
                scope_by_region[region.region_id] = self.active_commitment_scope
                by_region[region.region_id] = [c for c in by_region.get(region.region_id, [])
                    if c.commitment_scope == self.active_commitment_scope]
        by_region = {
            region_id: [
                candidate for candidate in options
                if self.directional_candidate_allowed(
                    candidate,
                    candidate.commitment_scope
                    or scope_by_region[region_id])]
            for region_id, options in by_region.items()}
        by_region = {
            region_id: options for region_id, options in by_region.items()
            if options}
        available = [region for region in regions if region.region_id in by_region]
        available_ids = {region.region_id for region in available}
        if (getattr(self, "active_commitment_scope", None) is None
                and self.active_region_id in scope_by_region):
            self.active_commitment_scope = scope_by_region[self.active_region_id]
        observed_scopes = set(scope_by_region.values())
        observed_scopes.update(
            ("safe_region", int(region_id))
            for region_id in self.safe_region_connectivity.regions)
        available_scopes = {
            scope_by_region[region_id] for region_id in available_ids}
        self.last_selection_reason = "rolling_uncommitted"

        if (allow_region_release and intent is None
                and self.last_region_release_evaluation_update != self.map_update_count):
            previous_active = self.active_region_id
            previous_scope = self.active_commitment_scope
            update = update_region_commitment(
                self.active_commitment_scope, observed_scopes, available_scopes,
                self.active_region_missing_streak,
                self.active_region_unreachable_since, time.monotonic(),
                self.region_release_updates, self.region_unreachable_timeout)
            self.active_commitment_scope = update.active_region_id
            if self.active_commitment_scope is None:
                self.active_region_id = None
            elif self.active_region_id not in available_ids or (
                    scope_by_region.get(self.active_region_id)
                    != self.active_commitment_scope):
                self.active_region_id = next((
                    region.region_id for region in available
                    if scope_by_region[region.region_id]
                    == self.active_commitment_scope), self.active_region_id)
            self.active_region_missing_streak = update.missing_streak
            self.active_region_unreachable_since = update.unreachable_since
            self.last_region_release_evaluation_update = self.map_update_count
            if update.release_reason is not None:
                self.commitment_target_cells = set()
                self.clear_directional_commitment()
                self.commitment_release_count += 1
                self.last_commitment_release_reason = update.release_reason
                self.commitment_state = "RELEASABLE"
                self.region_commitment_gate.reset()
            if update.release_reason == "frontier_missing":
                self.get_logger().info(
                    f"[REGION_COMMITMENT_RELEASED] region={previous_active} "
                    f"scope={previous_scope} "
                    f"after {self.region_release_updates} map updates with no "
                    "remaining frontier cluster")
            elif update.release_reason == "persistently_unreachable":
                self.get_logger().warning(
                    f"[REGION_COMMITMENT_RELEASED] region={previous_active} "
                    f"scope={previous_scope} "
                    f"reason=persistently_unreachable timeout="
                    f"{self.region_unreachable_timeout:.1f}s; re-evaluating all "
                    "reachable regions")

        committed_ids = {
            region_id for region_id in available_ids
            if scope_by_region[region_id] == self.active_commitment_scope}
        if (allow_region_release and intent is None and self.active_commitment_scope is not None
                and committed_ids):
            if not self.commitment_target_cells:
                self.commitment_target_cells = {
                    cell
                    for region_id in committed_ids
                    for candidate in by_region[region_id]
                    for cell in candidate.observation_cells}
            remaining_cells = sum(
                self.grid.value(cell) == UNKNOWN
                for cell in self.commitment_target_cells)
            decision = self.region_commitment_gate.evaluate(
                self.active_commitment_scope, self.map_content_revision,
                remaining_cells)
            self.commitment_remaining_cells = decision.remaining_cells
            self.commitment_stagnant_revisions = decision.stagnant_revisions
            if decision.release_reason is not None:
                previous_active = self.active_region_id
                self.active_region_id = None
                self.active_commitment_scope = None
                self.commitment_target_cells = set()
                self.clear_directional_commitment()
                self.active_region_missing_streak = 0
                self.active_region_unreachable_since = None
                self.commitment_state = "RELEASABLE"
                self.commitment_release_count += 1
                self.last_commitment_release_reason = decision.release_reason
                self.last_selection_reason = decision.release_reason
                self.get_logger().info(
                    f"[REGION_COMMITMENT_RELEASED] region={previous_active} "
                    f"reason={decision.release_reason} remaining_cells="
                    f"{decision.remaining_cells} exhausted_streak="
                    f"{decision.exhausted_streak}/{self.region_exhausted_revisions} "
                    f"stagnant_revisions={decision.stagnant_revisions}/"
                    f"{self.region_stagnation_revisions}")
                self.region_commitment_gate.reset()
            else:
                self.commitment_state = "COMMITTED"

        if intent is not None:
            if committed_ids:
                self.branch_unresolved_since = None
                self.branch_unresolved_updates = 0
            elif self.branch_unresolved_since is None:
                self.branch_unresolved_since = time.monotonic()
                self.branch_unresolved_updates = 0

        if (self.active_commitment_scope is not None
                and self.active_commitment_scope not in available_scopes):
            if allow_cross_region_preparation and available_ids:
                self.get_logger().info(
                    f"[CROSS_REGION_FALLBACK_PREPARATION] committed_region="
                    f"{self.active_region_id} has no reachable successor; "
                    f"preparing standby from {len(available_ids)} other "
                    "reachable region(s) without releasing commitment",
                    throttle_duration_sec=2.0)
            elif allow_region_release:
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
                    f"scope={self.active_commitment_scope} "
                    f"frontier_present="
                    f"{self.active_commitment_scope in observed_scopes} "
                    f"frontier_cells={frontier_cells} candidate_reachable=False "
                    f"other_reachable_regions="
                    f"{len(available_scopes - {self.active_commitment_scope})} "
                    f"unreachable_for={unreachable_duration:.1f}/"
                    f"{self.region_unreachable_timeout:.1f}s missing_frontier="
                    f"{self.active_region_missing_streak}/{self.region_release_updates}; "
                    "waiting for a reachable viewpoint in the committed region",
                    throttle_duration_sec=2.0)
                return None
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
                committed = (scope_by_region[region.region_id]
                             == self.active_commitment_scope)
                return (not committed, -gain, access, region.region_id)
            available = sorted(available, key=priority)[:self.max_global_regions]

        self.region_sequence, need_global_plan = retain_region_commitment(
            self.region_sequence, available_ids, self.active_region_id)
        if update_region_prediction:
            prediction_signature = self.build_region_prediction_signature(
                start, available_ids)
            if (need_global_plan
                    or prediction_signature
                    != self.region_prediction_input_signature):
                sequence_started = time.perf_counter()
                self.region_sequence = self.plan_region_sequence(
                    start, available, by_region, inflated, blocked_edges)
                self.last_region_sequence_ms += (
                    time.perf_counter() - sequence_started) * 1000.0
                self.region_prediction_input_signature = prediction_signature

        # The full tour is only a prediction for future preparation.  Immediate
        # control is a one-step rolling decision based on real A* access cost.
        selection_options = {
            region_id: self.preferred_horizon_pool(options)
            for region_id, options in by_region.items()}
        best_by_region = {
            region_id: max(preferred, key=lambda item: (
                region_information_efficiency(
                    self.planning_gain(item),
                    item.cluster_size,
                    item.path_length, item.turn_cost)/(1+self.candidate_departure_penalty(item)),
                self.candidate_utility(item, preferred),
                -item.path_length, -item.cell[0], -item.cell[1]))
            for region_id, preferred in selection_options.items() if preferred}
        region_scores = {
            region_id: region_information_efficiency(
                self.planning_gain(candidate),
                candidate.cluster_size,
                candidate.path_length, candidate.turn_cost)/(1+self.candidate_departure_penalty(candidate))
            for region_id, candidate in best_by_region.items()}
        committed_ids = {
            region_id for region_id in best_by_region
            if (self.active_commitment_scope is not None
                and scope_by_region[region_id] == self.active_commitment_scope)}
        if committed_ids:
            # A committed option keeps its current child while that child has a
            # reachable terminal.  Frontier score jitter may rank another child
            # higher, but it cannot steer the robot back and forth inside one
            # persistent safe region or terminate the option.
            incumbent = self.active_region_id
            if incumbent in committed_ids:
                selected_region = incumbent
            else:
                selected_region = max(committed_ids, key=lambda region_id: (
                    region_scores[region_id], -region_id))
            self.last_selection_reason = "residual_commitment"
            challenger = select_rolling_region(
                region_scores, selected_region, self.region_switch_ratio)
            if challenger is not None and challenger not in committed_ids:
                self.commitment_switch_suppressed_count += 1
        else:
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
        selected_scope = (selected.commitment_scope
                          or scope_by_region[selected_region])
        if intent is not None and selected_scope == self.active_commitment_scope:
            demand_candidates = [c for c in candidates if c.commitment_scope == selected_scope]
        elif hasattr(self, "branch_option_serial"):
            demand_candidates = [c for c in candidates if c.region_id == selected.region_id]
        else:
            demand_candidates = [c for c in candidates if (
                c.commitment_scope or scope_by_region[c.region_id]) == selected_scope]
        selected.option_target_cells = {cell for c in demand_candidates for cell in c.observation_cells}
        if not selected.option_target_cells:
            selected.option_target_cells = set(selected.observation_cells)
        active_score = max(
            (region_scores[region_id] for region_id in committed_ids),
            default=None)
        self.get_logger().info(
            f"[ROLLING_REGION_CHOICE] selected={selected_region} "
            f"active={self.active_region_id} score={region_scores[selected_region]:.3f} "
            f"active_score={'none' if active_score is None else f'{active_score:.3f}'} "
            f"switch_ratio={self.region_switch_ratio:.2f}",
            throttle_duration_sec=2.0)
        return selected

    def build_region_prediction_signature(
            self, start: Cell, available_region_ids: Set[int]):
        """Describe material ATSP inputs without using every map revision.

        The tour is only a background prediction.  Recompute it when the
        region set, coarse robot location, route constraints, or sparse graph
        structure materially changes; ordinary frontier-cell jitter does not
        justify blocking this callback with another exact tour solve.
        """
        coarse_stride = max(1, int(round(1.0 / self.grid.resolution)))
        position_quantum = max(self.grid.resolution, 0.5)
        quantized_nodes = {
            node_id: (
                int(round(node.position[0] / position_quantum)),
                int(round(node.position[1] / position_quantum)))
            for node_id, node in self.sparse_router.nodes.items()}
        topology_nodes = tuple(sorted(set(quantized_nodes.values())))
        topology_edges = []
        for edge in self.sparse_router.edges.values():
            source = quantized_nodes.get(edge.source_id)
            target = quantized_nodes.get(edge.target_id)
            if source is None or target is None:
                continue
            topology_edges.append(tuple(sorted((source, target))))
        return (
            start[0] // coarse_stride,
            start[1] // coarse_stride,
            tuple(sorted(available_region_ids)),
            self.route_constraint_revision,
            topology_nodes,
            tuple(sorted(topology_edges)),
        )

    def plan_region_sequence(self, start: Cell,
                             regions: Sequence[FrontierRegion],
                             by_region: Dict[int, List[FrontierCandidate]],
                             inflated: Set[Cell],
                             blocked_edges: Set[DirectedEdge]) -> List[int]:
        # This tour predicts later regions only.  It must not launch dense
        # searches that delay the next executable goal.  Sparse estimates are
        # used when available; uncovered pairs receive a conservative geometric
        # estimate.  Every path actually published is still dense-validated.
        representative_options = {
            region.region_id: self.preferred_horizon_pool(
                by_region[region.region_id])
            for region in regions}
        representatives = []
        active_region_ids = {region.region_id for region in regions}
        self.region_anchor_cells = {
            region_id: cell for region_id, cell in self.region_anchor_cells.items()
            if region_id in active_region_ids}
        for region in regions:
            options = representative_options[region.region_id]
            anchor = self.region_anchor_cells.get(region.region_id)
            representative = next(
                (item for item in options if item.cell == anchor), None)
            if representative is None:
                representative = max(options, key=lambda item: (
                    self.candidate_utility(item, options),
                    -item.path_length, -item.cell[0], -item.cell[1]))
            representatives.append(representative)
            self.region_anchor_cells[region.region_id] = representative.cell
        count = len(representatives)
        costs = np.full((count + 1, count + 1), np.inf, dtype=np.float64)
        np.fill_diagonal(costs, 0.0)
        for index, representative in enumerate(representatives):
            costs[0, index + 1] = representative.path_length

        for source in range(count):
            source_cell = representatives[source].cell
            target_indices = [target for target in range(count)
                              if target != source]
            sparse_started = time.perf_counter()
            sparse_estimates, sparse_miss_reasons = self.sparse_route_estimates(
                source_cell,
                [representatives[target].cell for target in target_indices],
                inflated, blocked_edges)
            self.last_sparse_region_ms += (
                time.perf_counter() - sparse_started) * 1000.0
            sparse_by_target = dict(zip(target_indices, sparse_estimates))
            sparse_hits = sum(
                estimate is not None for estimate in sparse_estimates)
            sparse_fallbacks = len(target_indices) - sparse_hits
            self.sparse_region_pair_queries += len(target_indices)
            self.sparse_region_pair_hits += sparse_hits
            self.sparse_region_pair_fallbacks += sparse_fallbacks
            self.last_sparse_region_pair_hits += sparse_hits
            self.last_sparse_region_pair_fallbacks += sparse_fallbacks
            self.ensure_sparse_miss_counters()
            self.accumulate_sparse_miss_reasons(
                sparse_miss_reasons, self.sparse_region_miss_reasons,
                self.last_sparse_region_miss_reasons)
            for target in range(count):
                if source == target:
                    continue
                sparse_estimate = sparse_by_target.get(target)
                if sparse_estimate is not None:
                    costs[source + 1, target + 1] = sparse_estimate.distance
                    continue
                target_cell = representatives[target].cell
                grid_distance = math.hypot(
                    source_cell[0] - target_cell[0],
                    source_cell[1] - target_cell[1])
                costs[source + 1, target + 1] = (
                    1.5 * grid_distance * self.grid.resolution)

        forced_first = None
        if getattr(self, "active_commitment_scope", None) is not None:
            for index, region in enumerate(regions):
                if any(
                        (candidate.commitment_scope
                         or region_commitment_scope(region))
                        == self.active_commitment_scope
                        for candidate in by_region[region.region_id]):
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
            "visit_window": getattr(self, "last_visit_window", []),
            "visit_stats": getattr(self, "last_visit_stats", {}),
        }, separators=(",", ":"))
        self.region_sequence_pub.publish(msg)
        self.get_logger().info(f"Region sequence: {self.region_sequence}")

    def validated_reference_path(self, cells: Sequence[Cell]):
        """Densify and validate a reference route before Local sees it.

        A sparse route polyline may join two distant points; Local then cannot
        connect its departure to that straight segment at a bend. Reject any
        path whose original segments leave known-free space.
        """
        if not cells or self.position is None:
            return None
        try:
            blocked_edges = self.current_blocked_edges()
        except AttributeError:  # partial test/replay instance
            blocked_edges = set()
        inflated = set()  # reference paths have zero hard erosion
        start = self.grid.world_to_cell(*self.position)
        if self.grid.value(start) != FREE:
            return None
        max_cells = max(1, int(math.ceil(
            float(getattr(self, "reference_path_max_segment", 0.75))
            / self.grid.resolution)))
        return densified_reference_path(
            self.grid, cells, inflated, blocked_edges, max_cells)

    def publish_reference_path(self, cells: Sequence[Cell], preparing_task=None, *, task=None):
        validated = self.validated_reference_path(cells)
        if validated is None:
            self.get_logger().warning(
                "[REFERENCE_PATH_REJECTED] reason=SEGMENT_BLOCKED "
                f"cells={len(cells)}; not sending an unconnectable route to Local",
                throttle_duration_sec=2.0)
            return False
        cells = validated
        msg = Path()
        now_ns = self.get_clock().now().nanoseconds
        request_generation = max(now_ns, self.last_path_request_generation + 1)
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
        if getattr(self, "task_mode", False):
            if preparing_task is None:
                self.publish_observation_task(msg, request_generation, task=task)
            else:
                pending = self.pending_observation_handoff
                pending.request_id = request_generation
                pending.published_ns = now_ns
                self.publish_observation_task(msg,request_generation,task=preparing_task)
        else:
            self.path_pub.publish(msg)
        self.last_path_request_generation = request_generation
        self.path_vis_pub.publish(msg)
        if preparing_task is not None or task is not None:
            return True
        self.record_reference_request(request_generation, now_ns)
        return True

    def record_reference_request(self, request_generation, published_ns=None):
        """Start ACK waiting only for a reference that was actually published."""
        self.pending_path_publish_ns = (self.get_clock().now().nanoseconds
                                        if published_ns is None else published_ns)
        self.pending_path_deferred = False
        self.local_recovery_request = None
        self.pending_path_request_generation = request_generation
        self.active_path_request_generation = request_generation
        self.active_path_trajectory_id = None
        self.scan_waiting_for_target = False
        self.pending_local_execution_finish = None
        return True

    def publish_path(self, cells: Sequence[Cell], candidate: FrontierCandidate,
                     acknowledged_task=None):
        """Publish first, then commit task/goal state in this callback group.

        A matched installation ACK commits its already-published intent. Map
        changes since publication belong to Local safety, not a second ACK veto.
        """
        previous = self.active_observation
        same_objective = (isinstance(previous, ObservationTask)
                          and previous.region_id == candidate.region_id
                          and previous.frontier_cell == candidate.frontier_cell)
        if acknowledged_task is not None:
            task = acknowledged_task
        elif same_objective:
            task = replace(previous, goal=candidate.goal,
                           terminal_cells=candidate.terminal_cells or (candidate.cell,))
        else:
            task = ObservationTask(
                region_id=candidate.region_id, goal=candidate.goal,
                target_cells=set(candidate.observation_cells),
                frontier_cell=candidate.frontier_cell,
                frontier_cluster_cells=set(candidate.frontier_cluster_cells),
                terminal_cells=candidate.terminal_cells or (candidate.cell,))
        if acknowledged_task is None:
            task.route_estimate = candidate.sparse_route
            task.route_source = candidate.route_source
            if not self.publish_reference_path(cells, task=task):
                # A simplified chord may be stale while the original adjacent
                # route still has a witness. Neither attempt changes active state.
                if (list(cells) == list(candidate.path) or not candidate.path
                        or not self.publish_reference_path(candidate.path, task=task)):
                    return False
        if acknowledged_task is None and same_objective:
            # Preserve the existing information object and its measured progress.
            # Identity binding and route metadata were staged on the copy above.
            previous.__dict__.update(task.__dict__)
            task = previous
        self.active_observation = task
        self.active_goal = candidate.goal
        self.active_goal_cell = candidate.cell
        self.active_raw_path = list(candidate.path)
        self.active_path_progress_index = 0
        self.last_active_path_validation_update = self.map_update_count
        self.pending_blocked_edge = None
        self.replacement_pending = False
        self.last_terminal_candidate_count = len(task.terminal_cells)
        if acknowledged_task is None:
            # The task setter may retire an old identity and clear request fields;
            # register the new, published request only after that commit.
            self.record_reference_request(self.last_path_request_generation)
        return True

    def update_running_state(self, known_cells: int) -> str:
        """Classify the run and accrue ACTIVE-phase time.

        ``complete`` means no frontier is left to visit.  ``stalled`` means the
        map stopped growing while frontier cells remain -- what a run looks like
        when the robot is boxed in or blocked: it keeps publishing paths and
        metrics with zero information gain, inflating every cumulative counter.
        """
        now = time.monotonic()
        if known_cells > self.last_state_known_cells:
            self.last_state_known_cells = known_cells
            self.last_cell_growth_time = now
            state = "active"
        elif self.latest_frontier_count == 0 and known_cells > 0:
            state = "complete"
        elif now - self.last_cell_growth_time > self.stall_detection_seconds:
            state = "stalled"
        else:
            # Growth paused, but not long enough to call it a stall yet.
            state = "active"
        if state != self.running_state:
            self.running_state = state
            self.running_state_since = now
        if state == "active":
            self.active_duration += now - self.last_metrics_time
        self.last_metrics_time = now
        return state

    def record_path_validation_failure(self, path, terminal, frontier, inflated):
        """Diagnostics only; do not publish persistent corridor failure evidence."""
        measured = getattr(self, "validation_measured_pose", None)
        measured_position = tuple(measured[1:3]) if measured is not None else None
        measured_yaw = measured[3] if measured is not None else None
        result = diagnose_double_cylinder_path(
            self.grid, path, terminal, frontier, inflated,
            getattr(self, "footprint_radius", 0.0),
            getattr(self, "footprint_offset", 0.0), measured_yaw,
            self.grid.world_to_cell(*measured_position) if measured_position is not None else None)
        if result.accepted:
            return  # A mocked/custom boolean reject has no footprint evidence.
        counts = getattr(self, "path_validation_failure_counts", {})
        counts[result.location] = counts.get(result.location, 0) + 1
        self.path_validation_failure_counts = counts
        task = getattr(self, "active_observation", None)
        self.last_path_validation_failure = {
            "location": result.location, "reason": result.reason,
            "task_id": getattr(task, "task_id", 0),
            "task_version": getattr(task, "task_version", 0),
            "map_content_revision": self.map_content_revision,
            "sample_index": result.sample_index, "sample_cell": result.sample_cell,
            "sample_world": self.grid.cell_to_world(result.sample_cell) if result.sample_cell else None,
            "hit_cell": result.hit_cell,
            "hit_world": self.grid.cell_to_world(result.hit_cell) if result.hit_cell else None,
            "sampled_distance": result.sampled_distance, "sample_yaw": result.sample_yaw,
            "current_margin_blocked": result.current_margin_blocked,
            "measured_position": measured_position, "measured_yaw": measured_yaw,
            "measured_stamp_ns": measured[0] if measured is not None else None,
            "departure_window": 0.5, "goal": self.grid.cell_to_world(terminal),
        }
        self.get_logger().warning(
            "[GLOBAL_PATH_VALIDATION_FAILURE] " + json.dumps(self.last_path_validation_failure),
            throttle_duration_sec=2.0)

    @with_planning_grid_snapshot
    def metrics_timer(self):
        known_cells = int(np.count_nonzero(self.grid.data != UNKNOWN))
        total_cells = int(self.grid.data.size)
        option_state = self.region_commitment_gate.state
        state = self.update_running_state(known_cells)
        sparse_attempts = (self.last_sparse_candidate_hits
                           + self.last_sparse_candidate_fallbacks)
        sparse_hit_rate_inst = (self.last_sparse_candidate_hits / sparse_attempts
                                if sparse_attempts else 0.0)
        if state == "active":
            self.active_sample_count += 1
            self.active_sparse_hit_rates.append(sparse_hit_rate_inst)
            self.active_planning_ms.append(self.last_global_plan_ms)
        # Per-interval rates: numerator and denominator come from the same
        # window, so they keep their meaning at any point of the exploration
        # instead of drifting with elapsed time.
        delta_cells = known_cells - self.rate_prev_known_cells
        delta_ms = self.cumulative_planning_ms - self.rate_prev_planning_ms
        delta_distance = self.total_distance - self.rate_prev_distance
        coverage_gain_per_m = (delta_cells / delta_distance
                               if delta_distance > 1e-6 else 0.0)
        planning_ms_per_new_cell = (delta_ms / delta_cells
                                    if delta_cells > 0 else 0.0)
        self.rate_prev_known_cells = known_cells
        self.rate_prev_planning_ms = self.cumulative_planning_ms
        self.rate_prev_distance = self.total_distance
        record = {
            "run_id": self.run_id,
            "elapsed_s": round(time.monotonic() - self.run_start_time, 3),
            "strategy": self.selection_strategy,
            "coverage": known_cells / total_cells if total_cells else 0.0,
            "known_cells": known_cells,
            "frontier_cells": self.latest_frontier_count,
            "locomotion_epoch": self.locomotion_epoch,
            "locomotion_ready": self.locomotion_ready(),
            "unreachable_context_latched": self.unreachable_context is not None,
            "last_global_path_validation_failure": json.dumps(
                getattr(self, "last_path_validation_failure", {})),
            **{f"global_path_validation_{location.lower()}":
               getattr(self, "path_validation_failure_counts", {}).get(location, 0)
               for location in ("CURRENT_FOOTPRINT_INVALID", "DEPARTURE_CONNECTION_INVALID",
                                "PATH_INVALID", "TERMINAL_INVALID", "UNASSESSED")},
            **{
                f"candidate_rejected_{reason}": count
                for reason, count in self.last_candidate_rejections.items()
            },
            "regions": self.latest_region_count,
            "total_distance_m": self.total_distance,
            "revisit_distance_m": self.revisit_distance,
            "region_visit_window_enabled": self.bounded_visits_enabled(),
            "region_visit_window": getattr(self, "last_visit_window", []),
            "region_visit_stats": getattr(self, "last_visit_stats", {}),
            "region_switches": self.region_switches,
            "intra_scope_region_switches": self.intra_scope_region_switches,
            "local_repair_exhausted_count": self.local_repair_exhausted_count,
            "failure_evidence_received": self.failure_evidence_received,
            "failure_evidence_matched": self.failure_evidence_matched,
            "last_failure_stage": self.last_failure_stage,
            "last_failure_reason": self.last_failure_reason,
            "last_failure_clearance_m": self.last_failure_clearance,
            "region_failure_cooldowns": len(self.region_failure_cooldowns),
            "directional_candidate_suppressed_count": (
                self.directional_candidate_suppressed_count),
            "directional_recovery_switches": self.directional_recovery_switches,
            "directional_recovery_pending": self.directional_recovery_pending,
            "commitment_direction_progress_m": (
                self.commitment_direction_progress),
            "active_region_id": (
                self.active_region_id if self.active_region_id is not None else -1),
            "active_commitment_kind": (
                self.active_commitment_scope[0]
                if self.active_commitment_scope is not None else "none"),
            "active_commitment_id": (
                self.active_commitment_scope[1]
                if self.active_commitment_scope is not None else -1),
            "commitment_state": self.commitment_state,
            "commitment_remaining_cells": self.commitment_remaining_cells,
            "commitment_stagnant_revisions": self.commitment_stagnant_revisions,
            "commitment_release_count": self.commitment_release_count,
            "commitment_switch_suppressed_count": (
                self.commitment_switch_suppressed_count),
            "last_commitment_release_reason": self.last_commitment_release_reason,
            "region_option_elapsed_revisions": option_state.elapsed_revisions,
            "region_option_collected_cells": option_state.collected_cells,
            "region_option_frozen_target_cells": len(
                self.commitment_target_cells),
            "region_option_score_switch_enabled": 0,
            "last_region_switch_reason": self.last_region_switch_reason,
            "goals_reached": self.goals_reached,
            "paths_published": self.paths_published,
            "short_paths_published": self.short_paths_published,
            "short_path_ratio": (
                self.short_paths_published / self.paths_published
                if self.paths_published else 0.0),
            "last_published_path_length_m": self.last_published_path_length,
            "observations_satisfied": self.observations_satisfied,
            "observation_motion_released_count": (
                self.observation_motion_released_count),
            "task_mode": getattr(self, "task_mode", False),
            "task_id": getattr(self.active_observation, "task_id", 0),
            "task_version": getattr(self.active_observation, "task_version", 0),
            "tasks_created": getattr(self, "tasks_created", 0),
            "task_requests_published": getattr(self, "task_requests_published", 0),
            "task_stations_reached": getattr(self, "task_stations_reached", 0),
            "task_local_rejections": getattr(self, "task_local_rejections", 0),
            "departure_recovery_attempts": getattr(self, "departure_recovery_attempts", 0),
            "departure_recovery_terminal": getattr(self, "departure_recovery_terminal", False),
            "departure_recovery_reason": getattr(self, "departure_recovery_reason", "none"),
            "task_stale_feedback": getattr(self, "task_stale_feedback", 0),
            "last_observation_released_reason": (
                self.last_observation_released_reason),
            "last_observation_released_progress": (
                self.last_observation_released_progress),
            "terminal_relocations": self.terminal_relocations,
            "terminal_candidate_count": self.last_terminal_candidate_count,
            "terminal_clearance_m": self.last_terminal_clearance,
            "approach_min_clearance_m": self.last_approach_clearance,
            "observation_progress": (
                self.active_observation.progress if self.active_observation else 0.0),
            "observation_observed_cells": (
                self.active_observation.observed_cells if self.active_observation else 0),
            "observation_expected_cells": (
                len(self.active_observation.target_cells)
                if self.active_observation else 0),
            "observation_frontier_missing_streak": (
                self.active_observation.frontier_missing_streak
                if self.active_observation else 0),
            "observation_completion_reason": (
                self.last_observation_completion_reason),
            "observation_completion_progress": (
                self.last_observation_completion_progress),
            "next_observation_prepared": self.prepared_candidate is not None,
            "pipeline_latency_ewma_s": self.pipeline_latency_ewma,
            "local_execution_finish_events": self.local_execution_finish_events,
            "local_execution_finish_reroutes": self.local_execution_finish_reroutes,
            "map_updates": self.map_update_count,
            "terrain_rise_cells": len(self.terrain_rises),
            "terrain_drop_cells": len(self.terrain_drops),
            "planning_events": self.planning_events,
            "map_preprocess_calls": self.map_preprocess_calls,
            "candidate_tree_searches": self.candidate_tree_searches,
            "region_pair_tree_searches": self.region_pair_tree_searches,
            "region_edge_cache_hits": self.region_edge_cache_hits,
            "region_edge_cache_misses": self.region_edge_cache_misses,
            "expanded_grid_cells": self.expanded_grid_cells,
            "sparse_graph_revision": self.sparse_router.graph_revision,
            "sparse_graph_nodes": len(self.sparse_router.nodes),
            "sparse_graph_edges": len(self.sparse_router.edges),
            "topology_messages_received": self.topology_messages_received,
            "topology_messages_applied": self.topology_messages_applied,
            "topology_messages_rejected": self.topology_messages_rejected,
            "topology_snapshot_recovery_count": (
                self.topology_snapshot_recovery_count),
            "last_rejected_topology_revision": (
                self.last_rejected_topology_revision),
            "sparse_terrain_penalized_edges": sum(
                1 for edge in self.sparse_router.edges.values()
                if edge.traversability > 1.0001),
            "sparse_terrain_preferred_edges": sum(
                1 for edge in self.sparse_router.edges.values()
                if edge.traversability < 0.9999),
            "sparse_terrain_states": {
                str(state): sum(
                    1 for edge in self.sparse_router.edges.values()
                    if edge.terrain_state == state)
                for state in sorted({edge.terrain_state
                                     for edge in self.sparse_router.edges.values()})},
            "sparse_candidate_queries": self.sparse_candidate_queries,
            "sparse_candidate_hits": self.sparse_candidate_hits,
            "sparse_candidate_fallbacks": self.sparse_candidate_fallbacks,
            **{
                f"sparse_candidate_miss_{reason}": count
                for reason, count in self.sparse_candidate_miss_reasons.items()
            },
            "sparse_region_pair_queries": self.sparse_region_pair_queries,
            "sparse_region_pair_hits": self.sparse_region_pair_hits,
            "sparse_region_pair_fallbacks": self.sparse_region_pair_fallbacks,
            **{
                f"sparse_region_miss_{reason}": count
                for reason, count in self.sparse_region_miss_reasons.items()
            },
            "dense_final_validation_searches": self.dense_final_validation_searches,
            "sparse_final_validation_failures": self.sparse_final_validation_failures,
            "sparse_materialized_graph_routes": getattr(self, "sparse_materialized_graph_routes", 0),
            "sparse_materialized_dense_rescues": getattr(self, "sparse_materialized_dense_rescues", 0),
            "sparse_materialized_connector_routes": getattr(self, "sparse_materialized_connector_routes", 0),
            "route_source_activated_requests": getattr(self, "route_source_activated_requests", {}),
            "branch_option_serial": getattr(self, "branch_option_serial", 0),
            "branch_entrance_verified": bool(getattr(self, "branch_commitment", None)
                                            and self.branch_commitment.entrance),
            "branch_association_unresolved_updates": getattr(self, "branch_unresolved_updates", 0),
            "branch_continuation_present": getattr(self, "branch_continuation_present", False),
            "global_path_invalid_count": self.global_path_invalid_count,
            "replan_suppressed_count": self.replan_suppressed_count,
            "scan_blocked_count": self.scan_blocked_count,
            "scan_reference_rejected_count": self.scan_reference_rejected_count,
            "goal_timeout_count": self.goal_timeout_count,
            "no_active_goal_replan_count": self.no_active_goal_replan_count,
            "last_active_goal_clear_reason": self.last_active_goal_clear_reason,
            "map_content_revision": self.map_content_revision,
            "last_map_preprocess_ms": self.last_map_preprocess_ms,
            "last_region_partition_ms": self.last_region_partition_ms,
            "last_topology_attached_clusters": (
                self.last_topology_attached_clusters),
            "last_topology_unattached_clusters": (
                self.last_topology_unattached_clusters),
            "last_region_edge_cache_hits": self.last_region_edge_cache_hits,
            "last_region_edge_cache_misses": self.last_region_edge_cache_misses,
            "last_sparse_candidate_ms": self.last_sparse_candidate_ms,
            "last_sparse_region_ms": self.last_sparse_region_ms,
            "last_sparse_candidate_hits": self.last_sparse_candidate_hits,
            "last_sparse_candidate_fallbacks": self.last_sparse_candidate_fallbacks,
            **{
                f"last_sparse_candidate_miss_{reason}": count
                for reason, count
                in self.last_sparse_candidate_miss_reasons.items()
            },
            "last_sparse_region_pair_hits": self.last_sparse_region_pair_hits,
            "last_sparse_region_pair_fallbacks": (
                self.last_sparse_region_pair_fallbacks),
            **{
                f"last_sparse_region_miss_{reason}": count
                for reason, count in self.last_sparse_region_miss_reasons.items()
            },
            "last_candidate_tree_ms": self.last_candidate_tree_ms,
            "last_region_sequence_ms": self.last_region_sequence_ms,
            "last_planning_ms": self.last_global_plan_ms,
            "cumulative_planning_ms": self.cumulative_planning_ms,
            # ACTIVE-phase view.  Compare runs on these, never on the cumulative
            # counters above: those keep growing while the run is stalled, or
            # up to whatever moment it was stopped.
            "running_state": state,
            "state_age_s": round(time.monotonic() - self.running_state_since, 1),
            "since_map_growth_s": round(
                time.monotonic() - self.last_cell_growth_time, 1),
            "active_duration_s": round(self.active_duration, 1),
            "active_sample_count": self.active_sample_count,
            "sparse_hit_rate_inst": round(sparse_hit_rate_inst, 4),
            "sparse_hit_rate_active_median": (
                round(statistics.median(self.active_sparse_hit_rates), 4)
                if self.active_sparse_hit_rates else 0.0),
            "planning_ms_active_median": (
                round(statistics.median(self.active_planning_ms), 3)
                if self.active_planning_ms else 0.0),
            "coverage_gain_per_m": round(coverage_gain_per_m, 4),
            "planning_ms_per_new_cell": round(planning_ms_per_new_cell, 4),
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
            fieldnames = list(record)
            existing_header = None
            if os.path.exists(self.metrics_file):
                with open(self.metrics_file, "r", newline="",
                          encoding="utf-8") as stream:
                    existing_header = stream.readline().rstrip("\r\n")
            if (existing_header is not None
                    and existing_header != ",".join(fieldnames)):
                # Reusing one metrics_file across builds with a different field
                # set would silently misalign every later row under the stale
                # header: DictWriter rejects extra keys but quietly fills
                # missing ones.  Archive rather than corrupt the series.
                archive = f"{self.metrics_file}.{self.run_id}.bak"
                os.replace(self.metrics_file, archive)
                self.get_logger().warning(
                    "metrics field set changed; previous file archived to "
                    f"{archive}")
            write_header = not os.path.exists(self.metrics_file)
            with open(self.metrics_file, "a", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                if write_header:
                    writer.writeheader()
                writer.writerow(record)
        except OSError as error:
            self.get_logger().error(f"Unable to write metrics file: {error}")
            self.metrics_file = ""

    def record_terrain(self, world, heights):
        """Sparse terrain annotation: measured step heights, never obstacles.

        Each cell keeps the largest measured rise and the deepest measured drop
        relative to the observed local ground. The Explorer represents terrain;
        it does not decide whether the robot may cross it. This layer therefore
        feeds diagnostics (and, once the execution capability is integrated,
        cost or capability reasoning) while occupancy and inflation stay
        reserved for geometry that reaches the trunk's clearance volume.
        """
        for point, height in zip(world, heights):
            if not math.isfinite(height):
                continue
            cell = self._live_grid.world_to_cell(point[0], point[1])
            if height > self.terrain_floor_tolerance:
                self.terrain_rises[cell] = max(self.terrain_rises.get(cell, 0.0),
                                               float(height))
            elif height < -self.terrain_floor_tolerance:
                self.terrain_drops[cell] = min(self.terrain_drops.get(cell, 0.0),
                                               float(height))

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
            data[self.grid.inflated_mask(self.inflation_radius)] = OCCUPIED
        msg.data = array.array('b', data.reshape(-1).tobytes())
        return msg

    def publish_maps(self):
        if not self.changed_cells:
            return
        if not self.map_published:
            self.map_pub.publish(self.occupancy_message(False))
        else:
            xs = [cell[0] for cell in self.changed_cells]
            ys = [cell[1] for cell in self.changed_cells]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            update = OccupancyGridUpdate()
            update.header.stamp = self.get_clock().now().to_msg()
            update.header.frame_id = self.frame_id
            update.x = min_x
            update.y = min_y
            update.width = max_x - min_x + 1
            update.height = max_y - min_y + 1
            update.data = array.array(
                'b', self.grid.data[
                    min_y:max_y + 1, min_x:max_x + 1].reshape(-1).tobytes())
            self.map_updates_pub.publish(update)
        # Publish changed planning data on the configured spatial cadence.
        # An unchanged map is not a new observation and needs no full resend.
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if (self.last_planning_map_ns is None
                or now_s < self.last_planning_map_ns
                or (now_s - self.last_planning_map_ns)
                >= self.planning_map_publish_period - 1e-3):
            self.last_planning_map_ns = now_s
            self.planning_map_pub.publish(self.occupancy_message(True))
        self.changed_cells.clear()
        self.map_published = True

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
    # Point cloud, locomotion contract, planning/status, and executor overhead
    # each have an independent lane.  This prevents a valid stance heartbeat
    # from expiring while a Python global-planning callback is running.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
