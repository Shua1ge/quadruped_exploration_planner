"""Known-space path search, validation, and rolling-path repair."""

import heapq
import math
from collections import deque
from dataclasses import dataclass
from enum import Enum
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


@dataclass
class ExpansionBudget:
    limit: int
    used: int = 0

    @property
    def remaining(self):
        return max(0, self.limit-self.used)

    def consume(self):
        if self.remaining == 0:
            return False
        self.used += 1
        return True


class RouteSearchStatus(str, Enum):
    FOUND = "COARSE_CONNECTED"
    INCOMPLETE = "SEARCH_INCOMPLETE"
    NO_ROUTE = "NO_KNOWN_FREE_ROUTE"


@dataclass
class RouteSearchResult:
    status: RouteSearchStatus
    path: Optional[List[Cell]] = None
    expanded: int = 0
    reason: str = ""


class KnownFreeSearch:
    """Resumable A* facts, invalidated only by inspected cells/edges.

    This is a raw FREE connection witness, never a body certificate. Capacity
    exhaustion preserves the frontier rather than caching a false no-path.
    """

    def __init__(self, start, goal):
        self.start, self.goal = start, goal
        self.queue = [(0., 0., start)]
        self.costs = {start: 0.}
        self.parents = {}
        self.closed = set()
        self.cells = {}
        self.edges = {}
        self.result = None

    def current(self, grid, forbidden):
        return (all(grid.value(c) == value for c, value in self.cells.items())
                and all((edge in forbidden) == blocked
                        for edge, blocked in self.edges.items()))

    def advance(self, grid, forbidden, budget):
        if self.result is not None:
            return self.result

        def free(cell):
            value = grid.value(cell)
            self.cells[cell] = value
            return grid.in_bounds(cell) and value == FREE

        if not free(self.start) or not free(self.goal):
            self.result = RouteSearchResult(RouteSearchStatus.NO_ROUTE,
                                            reason="ENDPOINT_NOT_KNOWN_FREE")
            return self.result
        while self.queue:
            _, cost, cell = self.queue[0]
            if cell in self.closed or cost != self.costs.get(cell):
                heapq.heappop(self.queue)
                continue
            if cell == self.goal:
                path = [cell]
                while cell in self.parents:
                    cell = self.parents[cell]
                    path.append(cell)
                self.result = RouteSearchResult(RouteSearchStatus.FOUND,
                                                list(reversed(path)))
                return self.result
            if not budget.consume():
                return RouteSearchResult(RouteSearchStatus.INCOMPLETE,
                                         reason="EXPANSION_BUDGET")
            heapq.heappop(self.queue)
            self.closed.add(cell)
            for dx, dy, step in GRID_MOVES:
                nxt = (cell[0]+dx, cell[1]+dy)
                edge = (cell, nxt)
                self.edges[edge] = edge in forbidden
                if self.edges[edge] or not free(nxt):
                    continue
                if dx and dy and (not free((cell[0]+dx, cell[1]))
                                  or not free((cell[0], cell[1]+dy))):
                    continue
                candidate = cost+step
                if candidate >= self.costs.get(nxt, float("inf")):
                    continue
                self.costs[nxt] = candidate
                self.parents[nxt] = cell
                heapq.heappush(self.queue, (
                    candidate+math.dist(nxt, self.goal), candidate, nxt))
        self.result = RouteSearchResult(RouteSearchStatus.NO_ROUTE,
                                        reason="KNOWN_FREE_GRAPH_EXHAUSTED")
        return self.result


class CoarseRouteRepair:
    """Replace conflicting route segments, sharing search across candidates.

    Prefix/suffix cells are revalidated; unchanged anchors reuse their search
    even if the target, sensor revision or unrelated map content changed.
    """

    def __init__(self):
        self.searches = {}
        self.geometry = None

    def repair(self, grid, path, blocked_edges, budget):
        used = budget.used
        geometry = (grid.origin_x, grid.origin_y, grid.width, grid.height,
                    grid.resolution)
        if self.geometry != geometry:
            self.searches.clear()
            self.geometry = geometry
        forbidden = blocked_edges or set()
        route = []
        for cell in path:
            if not route:
                route.append(cell)
            elif cell != route[-1]:
                route.extend(bresenham(route[-1], cell)[1:])
        empty = set()  # no body inflation in the exploration connection model
        if not route or not grid.planning_free(route[0], empty) or not grid.planning_free(route[-1], empty):
            return RouteSearchResult(RouteSearchStatus.NO_ROUTE,
                                     reason="ENDPOINT_NOT_KNOWN_FREE")
        while True:
            conflict = next((i for i in range(len(route)-1)
                             if not segment_known_free(grid, route[i], route[i+1],
                                                       empty, forbidden)), None)
            if conflict is None:
                return RouteSearchResult(RouteSearchStatus.FOUND, route,
                                         budget.used-used)
            # Keep the first valid anchor on each side of this conflict. Extend
            # over newly occupied cells, but leave distant good geometry alone.
            left = max(0, conflict-2)
            while left and not grid.planning_free(route[left], empty):
                left -= 1
            right = min(len(route)-1, conflict+3)
            while right < len(route)-1 and not grid.planning_free(route[right], empty):
                right += 1
            key = (route[left], route[right])
            search = self.searches.get(key)
            if search is None or not search.current(grid, forbidden):
                search = KnownFreeSearch(*key)
                if len(self.searches) >= 64:
                    self.searches.pop(next(iter(self.searches)))
                self.searches[key] = search
            result = search.advance(grid, forbidden, budget)
            if result.status != RouteSearchStatus.FOUND:
                return RouteSearchResult(result.status, expanded=budget.used-used,
                                         reason=result.reason)
            route = route[:left]+result.path+route[right+1:]


def astar_known(grid: ExplorationGrid, start: Cell, goal: Cell,
                inflated: Set[Cell],
                blocked_edges: Optional[Set[DirectedEdge]] = None,
                max_cost_cells: Optional[float] = None,
                edge_allowed=None, max_expansions: Optional[int] = None,
                expansion_budget: Optional[ExpansionBudget] = None) -> Optional[List[Cell]]:
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
        if max_cost_cells is not None and current_cost > max_cost_cells:
            continue
        if current == goal:
            path = [current]
            while current in parents:
                current = parents[current]
                path.append(current)
            return list(reversed(path))
        if max_expansions is not None and len(closed) >= max_expansions:
            return None
        if expansion_budget is not None and not expansion_budget.consume():
            return None
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
            if edge_allowed is not None and not edge_allowed(current, nxt):
                continue
            candidate = current_cost + step_cost
            if max_cost_cells is not None and candidate > max_cost_cells:
                continue
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
    expanded_cells: int = 0

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
        return ShortestPathTree(start, {}, {}, 0)

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
    return ShortestPathTree(start, costs, parents, len(closed))


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
    return adjacent_grid_path_is_valid(grid, cells, inflated, blocked_edges)


def densified_reference_path(grid: ExplorationGrid, path: Sequence[Cell],
                             inflated: Set[Cell],
                             blocked_edges: Optional[Set[DirectedEdge]] = None,
                             max_segment_cells: int = 1) -> Optional[List[Cell]]:
    """Return a densified reference path, or None if any segment is blocked.

    A sparse route polyline may join two distant points; sending that straight
    segment to Local makes its departure/path search fail at a bend even when
    the current pose is safe. Densify while preserving the original waypoints,
    and reject the whole path if any original segment crosses non-free cells.
    """
    cells = [tuple(int(value) for value in cell) for cell in path]
    if not cells:
        return []
    if not all(grid.planning_free(cell, inflated) for cell in cells):
        return None
    step = max(1, int(max_segment_cells))
    result = [cells[0]]
    for first, second in zip(cells, cells[1:]):
        if first == second:
            continue
        if not segment_known_free(grid, first, second, inflated, blocked_edges):
            return None
        # Choose waypoints from the checked raster. Rounded interpolation can
        # change the raster on a second validation, even with an unchanged map.
        raster = bresenham(first, second)
        anchor = 0
        while anchor < len(raster)-1:
            end = min(anchor+step, len(raster)-1)
            while end > anchor+1 and (
                    math.dist(raster[anchor], raster[end]) > step or
                    bresenham(raster[anchor], raster[end]) != raster[anchor:end+1]):
                end -= 1
            if raster[end] != result[-1]:
                result.append(raster[end])
            anchor = end
    return result


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
