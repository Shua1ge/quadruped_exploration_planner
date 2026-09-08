"""Known-space path search, validation, and rolling-path repair."""

import heapq
import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set

from .grid import (
    Cell, DirectedEdge, ExplorationGrid, FREE, GRID_MOVES, OCCUPIED, Point2,
    UNKNOWN, bresenham,
)

def open_start_escape_corridor(
        grid: ExplorationGrid, start: Cell, inflated: Set[Cell],
        max_distance: float) -> Optional[Set[Cell]]:
    """Open only the shortest raw-free corridor out of start inflation.

    The robot pose can legitimately lie inside the conservative obstacle
    inflation near a wall. Clearing only ``start`` leaves A* in an isolated
    cell, while clearing a disk would remove unrelated safety clearance.
    """
    if not grid.in_bounds(start) or grid.value(start) != FREE:
        return None
    adjusted = set(inflated)
    if start not in inflated:
        return adjusted

    max_steps = max(1, int(math.ceil(max_distance / grid.resolution)))
    queue = deque([start])
    parents: Dict[Cell, Optional[Cell]] = {start: None}
    distances = {start: 0}
    exit_cell: Optional[Cell] = None
    while queue:
        current = queue.popleft()
        if current not in inflated:
            exit_cell = current
            break
        if distances[current] >= max_steps:
            continue
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nxt = (current[0] + dx, current[1] + dy)
            if (nxt in parents or not grid.in_bounds(nxt)
                    or grid.value(nxt) != FREE):
                continue
            parents[nxt] = current
            distances[nxt] = distances[current] + 1
            queue.append(nxt)

    if exit_cell is None:
        return None
    current: Optional[Cell] = exit_cell
    while current is not None:
        adjusted.discard(current)
        current = parents[current]
    return adjusted


def astar_known(grid: ExplorationGrid, start: Cell, goal: Cell,
                inflated: Set[Cell],
                blocked_edges: Optional[Set[DirectedEdge]] = None) -> Optional[List[Cell]]:
    if not grid.planning_free(start, inflated) or not grid.planning_free(goal, inflated):
        return None
    queue = [(0.0, 0.0, start)]
    forbidden = blocked_edges or set()
    costs: Dict[Cell, float] = {start: 0.0}
    parents: Dict[Cell, Cell] = {}
    closed = set()
    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in parents:
                current = parents[current]
                path.append(current)
            return list(reversed(path))
        closed.add(current)
        for dx, dy, step_cost in GRID_MOVES:
            nxt = (current[0] + dx, current[1] + dy)
            if (current, nxt) in forbidden:
                continue
            if not grid.planning_free(nxt, inflated):
                continue
            if dx and dy and (
                    not grid.planning_free((current[0] + dx, current[1]), inflated)
                    or not grid.planning_free((current[0], current[1] + dy), inflated)):
                continue
            candidate = current_cost + step_cost
            if candidate >= costs.get(nxt, float("inf")):
                continue
            costs[nxt] = candidate
            parents[nxt] = current
            heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            heapq.heappush(queue, (candidate + heuristic, candidate, nxt))
    return None


@dataclass
class ShortestPathTree:
    """One known-free shortest-path search shared by many candidate goals."""

    start: Cell
    costs: Dict[Cell, float]
    parents: Dict[Cell, Cell]

    def path_to(self, goal: Cell) -> Optional[List[Cell]]:
        if goal not in self.costs:
            return None
        current = goal
        path = [current]
        while current != self.start:
            current = self.parents[current]
            path.append(current)
        return list(reversed(path))


def build_shortest_path_tree(
        grid: ExplorationGrid, start: Cell, inflated: Set[Cell],
        blocked_edges: Optional[Set[DirectedEdge]] = None,
        targets: Optional[Set[Cell]] = None) -> ShortestPathTree:
    """Run Dijkstra once and retain paths to every requested reachable goal."""
    if not grid.planning_free(start, inflated):
        return ShortestPathTree(start, {}, {})

    forbidden = blocked_edges or set()
    remaining = None if targets is None else {
        target for target in targets if grid.planning_free(target, inflated)}
    queue = [(0.0, start)]
    costs: Dict[Cell, float] = {start: 0.0}
    parents: Dict[Cell, Cell] = {}
    closed: Set[Cell] = set()
    while queue:
        current_cost, current = heapq.heappop(queue)
        if current in closed:
            continue
        closed.add(current)
        if remaining is not None:
            remaining.discard(current)
            if not remaining:
                break
        for dx, dy, step_cost in GRID_MOVES:
            nxt = (current[0] + dx, current[1] + dy)
            if (current, nxt) in forbidden or not grid.planning_free(nxt, inflated):
                continue
            if dx and dy and (
                    not grid.planning_free((current[0] + dx, current[1]), inflated)
                    or not grid.planning_free((current[0], current[1] + dy), inflated)):
                continue
            candidate = current_cost + step_cost
            if candidate >= costs.get(nxt, float("inf")):
                continue
            costs[nxt] = candidate
            parents[nxt] = current
            heapq.heappush(queue, (candidate, nxt))
    return ShortestPathTree(start, costs, parents)


def adjacent_grid_path_is_valid(
        grid: ExplorationGrid, path: Sequence[Cell], inflated: Set[Cell],
        blocked_edges: Optional[Set[DirectedEdge]] = None) -> bool:
    """Validate a raw adjacent-cell path against the current planning map."""
    if not path or not all(grid.planning_free(cell, inflated) for cell in path):
        return False
    forbidden = blocked_edges or set()
    for first, second in zip(path[:-1], path[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        if max(abs(dx), abs(dy)) != 1 or (first, second) in forbidden:
            return False
        if dx and dy and (
                not grid.planning_free((first[0] + dx, first[1]), inflated)
                or not grid.planning_free((first[0], first[1] + dy), inflated)):
            return False
    return True


def splice_prepared_path(
        grid: ExplorationGrid, start: Cell, prepared_path: Sequence[Cell],
        inflated: Set[Cell],
        blocked_edges: Optional[Set[DirectedEdge]] = None) -> Optional[List[Cell]]:
    """Join the current pose to the reusable valid suffix of a prepared path."""
    if not prepared_path or not grid.planning_free(start, inflated):
        return None

    suffix_start = len(prepared_path) - 1
    if not grid.planning_free(prepared_path[-1], inflated):
        return None
    for index in range(len(prepared_path) - 2, -1, -1):
        if not adjacent_grid_path_is_valid(
                grid, prepared_path[index:index + 2], inflated, blocked_edges):
            break
        suffix_start = index

    best = None
    for index in range(suffix_start, len(prepared_path)):
        connector = bresenham(start, prepared_path[index])
        combined = connector + list(prepared_path[index + 1:])
        if not adjacent_grid_path_is_valid(grid, combined, inflated, blocked_edges):
            continue
        key = (path_length_cells(combined, grid.resolution), index)
        if best is None or key < best[0]:
            best = (key, combined)
    return None if best is None else best[1]


def segment_known_free(grid: ExplorationGrid, first: Cell, second: Cell,
                       inflated: Set[Cell],
                       blocked_edges: Optional[Set[DirectedEdge]] = None) -> bool:
    cells = bresenham(first, second)
    if not all(grid.planning_free(cell, inflated) for cell in cells):
        return False
    forbidden = blocked_edges or set()
    return all((a, b) not in forbidden for a, b in zip(cells[:-1], cells[1:]))


def simplify_known_path(grid: ExplorationGrid, path: Sequence[Cell],
                        inflated: Set[Cell],
                        blocked_edges: Optional[Set[DirectedEdge]] = None) -> List[Cell]:
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        farthest = anchor + 1
        for candidate in range(anchor + 2, len(path)):
            if segment_known_free(
                    grid, path[anchor], path[candidate], inflated, blocked_edges):
                farthest = candidate
            else:
                break
        result.append(path[farthest])
        anchor = farthest
    return result


def match_blocked_path_edge(path: Sequence[Cell], free_cell: Cell,
                            hit_cell: Cell) -> Optional[DirectedEdge]:
    """Match a local free-to-collision segment to one directed global path edge."""
    if len(path) < 2:
        return None
    for edge in zip(path[:-1], path[1:]):
        if edge == (free_cell, hit_cell):
            return edge

    direction = (hit_cell[0] - free_cell[0], hit_cell[1] - free_cell[1])
    norm = math.hypot(*direction)
    best = None
    for index, (start, end) in enumerate(zip(path[:-1], path[1:])):
        midpoint = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5)
        distance = math.hypot(midpoint[0] - hit_cell[0], midpoint[1] - hit_cell[1])
        alignment_penalty = 0.0
        if norm > 1e-6:
            edge_direction = (end[0] - start[0], end[1] - start[1])
            edge_norm = math.hypot(*edge_direction)
            alignment = ((edge_direction[0] * direction[0]
                          + edge_direction[1] * direction[1]) / (edge_norm * norm))
            alignment_penalty = 2.0 * max(0.0, -alignment)
        key = (distance + alignment_penalty, index)
        if best is None or key < best[0]:
            best = (key, (start, end))
    return None if best is None else best[1]


def path_length_cells(path: Sequence[Cell], resolution: float) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) * resolution
               for a, b in zip(path[:-1], path[1:]))


def path_turn_cost(path: Sequence[Cell]) -> float:
    """Total absolute heading change of a grid path in radians."""
    headings = [math.atan2(b[1] - a[1], b[0] - a[0])
                for a, b in zip(path[:-1], path[1:]) if a != b]
    return sum(abs(math.atan2(math.sin(b - a), math.cos(b - a)))
               for a, b in zip(headings[:-1], headings[1:]))


@dataclass
class RemainingPathCheck:
    """Result of validating only the not-yet-traversed global path suffix."""

    valid: bool
    progress_index: int
    remaining_distance: float
    lateral_error: float
    reason: str = ""
    invalid_cell: Optional[Cell] = None
    invalid_edge: Optional[DirectedEdge] = None


def validate_remaining_path(
        grid: ExplorationGrid, start: Cell, path: Sequence[Cell],
        previous_progress_index: int, inflated: Set[Cell],
        blocked_edges: Optional[Set[DirectedEdge]] = None) -> RemainingPathCheck:
    """Validate the active path ahead without reconsidering passed cells.

    The closest path cell at or after the previous progress index advances a
    monotonic cursor.  Its occupancy is deliberately not checked here: local
    execution owns the robot's immediate footprint, while this check detects
    newly invalidated future global-path cells and edges.
    """
    if not path:
        return RemainingPathCheck(False, 0, 0.0, 0.0, "empty_path")

    first = min(max(0, int(previous_progress_index)), len(path) - 1)
    progress_index = min(
        range(first, len(path)),
        key=lambda index: (
            (path[index][0] - start[0]) ** 2 + (path[index][1] - start[1]) ** 2,
            index))
    lateral_error = math.hypot(
        path[progress_index][0] - start[0],
        path[progress_index][1] - start[1]) * grid.resolution
    remaining_distance = path_length_cells(
        path[progress_index:], grid.resolution)
    forbidden = blocked_edges or set()

    for index in range(progress_index + 1, len(path)):
        previous = path[index - 1]
        current = path[index]
        edge = (previous, current)
        if edge in forbidden:
            return RemainingPathCheck(
                False, progress_index, remaining_distance, lateral_error,
                "temporarily_blocked_edge", invalid_cell=current,
                invalid_edge=edge)
        if not grid.planning_free(current, inflated):
            value = grid.value(current)
            reason = ("occupied_cell" if value == OCCUPIED else
                      "unknown_cell" if value == UNKNOWN else
                      "inflated_cell")
            return RemainingPathCheck(
                False, progress_index, remaining_distance, lateral_error,
                reason, invalid_cell=current, invalid_edge=edge)

        dx = current[0] - previous[0]
        dy = current[1] - previous[1]
        if dx and dy:
            side_a = (previous[0] + dx, previous[1])
            side_b = (previous[0], previous[1] + dy)
            if (not grid.planning_free(side_a, inflated)
                    or not grid.planning_free(side_b, inflated)):
                return RemainingPathCheck(
                    False, progress_index, remaining_distance, lateral_error,
                    "diagonal_corner_blocked", invalid_cell=current,
                    invalid_edge=edge)

    return RemainingPathCheck(
        True, progress_index, remaining_distance, lateral_error)


def remaining_polyline_distance(position: Point2,
                                points: Sequence[Point2]) -> float:
    """Arc length remaining after projecting position onto the closest segment."""
    if len(points) < 2:
        return 0.0
    best = None
    suffix = 0.0
    suffix_lengths = [0.0] * len(points)
    for index in range(len(points) - 2, -1, -1):
        suffix += math.hypot(points[index + 1][0] - points[index][0],
                             points[index + 1][1] - points[index][1])
        suffix_lengths[index] = suffix
    for index, (first, second) in enumerate(zip(points[:-1], points[1:])):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length2 = dx * dx + dy * dy
        ratio = 0.0 if length2 < 1e-9 else max(0.0, min(
            1.0, ((position[0] - first[0]) * dx +
                  (position[1] - first[1]) * dy) / length2))
        projected = (first[0] + ratio * dx, first[1] + ratio * dy)
        distance2 = ((position[0] - projected[0]) ** 2 +
                     (position[1] - projected[1]) ** 2)
        remaining = ((1.0 - ratio) * math.sqrt(length2) + suffix_lengths[index + 1])
        key = (distance2, remaining)
        if best is None or key < best:
            best = key
    return best[1]
