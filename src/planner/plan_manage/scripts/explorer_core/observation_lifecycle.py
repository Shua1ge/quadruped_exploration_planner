"""Information-objective state independent of ROS scheduling and selection."""

from dataclasses import dataclass, field
from typing import Optional, Set, Tuple

from .frontier_regions import (
    advance_completion_streak, frontier_cluster_present,
    observation_progress,
)
from .grid import Cell, ExplorationGrid, Point2, UNKNOWN


@dataclass
class ObservationTask:
    """Frozen information objective with stable alternative terminals."""

    region_id: Optional[int]
    goal: Point2
    target_cells: Set[Cell]
    frontier_cell: Optional[Cell] = None
    frontier_cluster_cells: Set[Cell] = field(default_factory=set)
    terminal_cells: Tuple[Cell, ...] = ()
    observed_cells: int = 0
    progress: float = 0.0
    completion_streak: int = 0
    frontier_missing_streak: int = 0
    completion_reason: str = "none"
    last_evaluated_update: int = -1
    preparation_started: bool = False
    task_id: int = 0
    task_version: int = 1
    lifecycle: str = "ACTIVE"
    # Legacy objectives retain their old denominator. Task mode requires
    # actual sensor evidence, never robot-centred free-space seeding.
    require_sensor_evidence: bool = False
    sensor_observed_cells: int = 0
    initial_sensor_revision: int = 0
    station_adjustments: int = 0
    predecessor_task_id: int = 0
    predecessor_task_version: int = 0
    information_satisfied: bool = False
    # Arrival ends this motion attempt, not its information objective. Unseen
    # cells remain in the map and may be selected by a later observation task.
    motion_released: bool = False
    satisfied_map_revision: int = 0
    satisfied_sensor_revision: int = 0
    # Motion contract metadata; never a second observation progress ledger.
    route_estimate: object = None
    route_source: str = "dense"
    route_portals: tuple = ()
    route_graph_revision: int = 0
    route_contract_frozen: bool = False
    last_evaluated_sensor_revision: int = 0
    last_closure_sensor_revision: int = 0
    boundary_closure_supported: bool = False


def evaluate_observation(
        task: Optional[ObservationTask], grid: ExplorationGrid,
        active_goal_cell: Optional[Cell], map_update_count: int,
        done_ratio: float, done_updates: int, closure_updates: int,
        viewpoint_standoff: float) -> Optional[ObservationTask]:
    """Update coverage, counting confirmations only from new relevant evidence.

    Occupancy computation revisions are not independent observations. Legacy
    mode keeps its old update semantics; sensor-only tasks use per-cell source
    revisions so unrelated scans cannot manufacture confirmation streaks.
    """
    if task is None or task.information_satisfied:
        return task
    tracked_frontier = (task.frontier_cluster_cells
                        or ({task.frontier_cell}
                            if task.frontier_cell is not None else set()))
    relevant_cells = task.target_cells | tracked_frontier
    frontier_present = bool(
        tracked_frontier and frontier_cluster_present(
            grid, tracked_frontier,
            max(viewpoint_standoff, 2.0 * grid.resolution)))
    if task.require_sensor_evidence and (frontier_present or any(
            not grid.in_bounds(cell) or grid.value(cell) == UNKNOWN
            for cell in tracked_frontier)):
        # Evidence invalidation must run even on an otherwise idle compute
        # revision. It may withdraw a closure proof, never add confirmation.
        task.frontier_missing_streak = 0
        task.boundary_closure_supported = False
        task.completion_reason = "none"
    sensor_revision = max((int(grid.sensor_observed[y, x])
        for x, y in relevant_cells if grid.in_bounds((x, y))), default=0)
    if task.require_sensor_evidence:
        if sensor_revision <= max(task.initial_sensor_revision,
                                  task.last_evaluated_sensor_revision):
            return task
        task.last_evaluated_sensor_revision = sensor_revision
    elif task.last_evaluated_update == map_update_count:
        return task
    observed, _, ratio = observation_progress(
        grid, task.target_cells, active_goal_cell)
    task.observed_cells = observed
    task.sensor_observed_cells = sum(
        1 for x, y in task.target_cells
        if grid.in_bounds((x, y))
        and grid.sensor_observed[y, x] > task.initial_sensor_revision)
    if task.require_sensor_evidence:
        observed = task.sensor_observed_cells
        ratio = observed / max(1, len(task.target_cells))
        task.observed_cells = observed
    task.progress = ratio
    task.last_evaluated_update = map_update_count
    task.completion_streak = advance_completion_streak(
        ratio, done_ratio, task.completion_streak)
    if not task.require_sensor_evidence:
        task.frontier_missing_streak = (
            0 if frontier_present else task.frontier_missing_streak + 1)
        task.boundary_closure_supported = not frontier_present
    elif frontier_present:
        task.frontier_missing_streak = 0
        task.boundary_closure_supported = False
    else:
        # Closure is a proof about the original boundary, not every unknown
        # behind it. Each confirmation requires a new observed cover of that
        # boundary; seeded FREE and window loss never provide this cover.
        baseline = max(task.initial_sensor_revision, task.last_closure_sensor_revision)
        supported = bool(tracked_frontier) and all(
            grid.in_bounds(cell) and grid.value(cell) != UNKNOWN
            and int(grid.sensor_observed[cell[1], cell[0]]) > baseline
            for cell in tracked_frontier)
        if supported:
            task.last_closure_sensor_revision = max(
                int(grid.sensor_observed[y, x]) for x, y in tracked_frontier)
            task.frontier_missing_streak += 1
            task.boundary_closure_supported = True
    if task.completion_streak >= done_updates:
        task.completion_reason = "visible_information_resolved"
    elif (task.frontier_missing_streak >= closure_updates
          and (not task.require_sensor_evidence
               or task.boundary_closure_supported)):
        task.completion_reason = "frontier_closed"
    else:
        task.completion_reason = "none"
    return task


def observation_is_complete(
        task: Optional[ObservationTask], done_updates: int,
        closure_updates: int) -> bool:
    """Complete on positive information or confirmed boundary closure."""
    return bool(task is not None and (
        task.completion_streak >= done_updates
        or (task.frontier_missing_streak >= closure_updates
            and (not task.require_sensor_evidence
                 or task.boundary_closure_supported))))


def observation_completion_record(
        task: Optional[ObservationTask]) -> Tuple[str, float]:
    """Return immutable evidence to retain after replacing the active task."""
    if task is None:
        return "none", 0.0
    return task.completion_reason, task.progress
