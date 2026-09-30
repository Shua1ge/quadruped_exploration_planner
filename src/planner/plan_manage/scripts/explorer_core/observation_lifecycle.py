"""Information-objective state independent of ROS scheduling and selection."""

from dataclasses import dataclass, field
from typing import Optional, Set, Tuple

from .frontier_regions import (
    advance_completion_streak, frontier_cluster_present,
    observation_progress,
)
from .grid import Cell, ExplorationGrid, Point2


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


def evaluate_observation(
        task: Optional[ObservationTask], grid: ExplorationGrid,
        active_goal_cell: Optional[Cell], map_update_count: int,
        done_ratio: float, done_updates: int, closure_updates: int,
        viewpoint_standoff: float) -> Optional[ObservationTask]:
    """Apply one accepted map revision to an active information objective."""
    if task is None or task.last_evaluated_update == map_update_count:
        return task
    observed, _, ratio = observation_progress(
        grid, task.target_cells, active_goal_cell)
    task.observed_cells = observed
    task.progress = ratio
    task.last_evaluated_update = map_update_count
    task.completion_streak = advance_completion_streak(
        ratio, done_ratio, task.completion_streak)
    tracked_frontier = (task.frontier_cluster_cells
                        or ({task.frontier_cell}
                            if task.frontier_cell is not None else set()))
    frontier_present = bool(
        tracked_frontier and frontier_cluster_present(
            grid, tracked_frontier,
            max(viewpoint_standoff, 2.0 * grid.resolution)))
    task.frontier_missing_streak = (
        0 if frontier_present else task.frontier_missing_streak + 1)
    if task.completion_streak >= done_updates:
        task.completion_reason = "visible_information_resolved"
    elif task.frontier_missing_streak >= closure_updates:
        # This objective exists to resolve its frozen frontier cluster.  Once
        # that cluster has remained absent for a confirmed run, its purpose is
        # complete even if not every neighbouring unknown became observable.
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
        or task.frontier_missing_streak >= closure_updates))


def observation_completion_record(
        task: Optional[ObservationTask]) -> Tuple[str, float]:
    """Return immutable evidence to retain after replacing the active task."""
    if task is None:
        return "none", 0.0
    return task.completion_reason, task.progress
