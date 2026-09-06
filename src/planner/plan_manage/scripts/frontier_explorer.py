#!/usr/bin/env python3
"""Stage-one frontier explorer using only local world-frame LiDAR observations."""

import csv
import heapq
import json
import math
import os
import time
from collections import deque
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


Cell = Tuple[int, int]
Point2 = Tuple[float, float]
DirectedEdge = Tuple[Cell, Cell]
UNKNOWN = -1
FREE = 0
OCCUPIED = 100
GRID_MOVES = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
)


def dilate_hit_ranges(ranges: Sequence[float], bins: int) -> np.ndarray:
    """Conservatively widen finite returns in bearing space.

    A point-cloud wall is sampled at discrete bearings.  Treating every empty
    bearing bin as a maximum-range miss creates artificial one-cell doorways.
    Copying the nearest return into neighbouring bins closes those sampling
    gaps without giving the explorer access to the simulator's truth map.
    """
    source = np.asarray(ranges, dtype=np.float64)
    result = source.copy()
    for offset in range(1, max(0, int(bins)) + 1):
        result = np.minimum(result, np.roll(source, offset))
        result = np.minimum(result, np.roll(source, -offset))
    return result


def bresenham(start: Cell, end: Cell) -> List[Cell]:
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    error = dx - dy
    cells = []
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        twice_error = 2 * error
        if twice_error > -dy:
            error -= dy
            x0 += sx
        if twice_error < dx:
            error += dx
            y0 += sy


class ExplorationGrid:
    """Fixed-size unknown/free/occupied map updated from planar LiDAR rays."""

    def __init__(self, size_x: float, size_y: float, resolution: float,
                 origin_x: Optional[float] = None, origin_y: Optional[float] = None):
        self.resolution = float(resolution)
        self.width = int(math.ceil(size_x / self.resolution))
        self.height = int(math.ceil(size_y / self.resolution))
        self.origin_x = -0.5 * size_x if origin_x is None else float(origin_x)
        self.origin_y = -0.5 * size_y if origin_y is None else float(origin_y)
        self.data = np.full((self.height, self.width), UNKNOWN, dtype=np.int8)

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height

    def world_to_cell(self, x: float, y: float) -> Cell:
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def cell_to_world(self, cell: Cell) -> Point2:
        return (self.origin_x + (cell[0] + 0.5) * self.resolution,
                self.origin_y + (cell[1] + 0.5) * self.resolution)

    def value(self, cell: Cell) -> int:
        if not self.in_bounds(cell):
            return OCCUPIED
        return int(self.data[cell[1], cell[0]])

    def integrate_ranges(self, position: Point2, ranges: Sequence[float],
                         max_range: float, min_range: float = 0.35,
                         no_return_range: Optional[float] = None):
        origin = self.world_to_cell(*position)
        if not self.in_bounds(origin):
            return
        free_cells: Set[Cell] = set()
        occupied_cells: Set[Cell] = set()
        ray_count = len(ranges)
        for index, measured_range in enumerate(ranges):
            hit = math.isfinite(measured_range) and measured_range < max_range
            miss_range = (max_range if no_return_range is None
                          else min(max_range, no_return_range))
            ray_range = min(measured_range, max_range) if hit else miss_range
            if ray_range < min_range:
                continue
            angle = -math.pi + (index + 0.5) * (2.0 * math.pi / ray_count)
            endpoint = (position[0] + ray_range * math.cos(angle),
                        position[1] + ray_range * math.sin(angle))
            line = [cell for cell in bresenham(origin, self.world_to_cell(*endpoint))
                    if self.in_bounds(cell)]
            if not line:
                continue
            if hit:
                free_cells.update(line[:-1])
                occupied_cells.add(line[-1])
            else:
                free_cells.update(line)

        for x, y in free_cells - occupied_cells:
            if self.data[y, x] != OCCUPIED:
                self.data[y, x] = FREE
        for x, y in occupied_cells:
            self.data[y, x] = OCCUPIED

        seed_radius = max(1, int(math.ceil(0.35 / self.resolution)))
        for dx in range(-seed_radius, seed_radius + 1):
            for dy in range(-seed_radius, seed_radius + 1):
                cell = (origin[0] + dx, origin[1] + dy)
                if self.in_bounds(cell) and math.hypot(dx, dy) <= seed_radius:
                    self.data[cell[1], cell[0]] = FREE

    def mark_occupied_points(self, points_xy: np.ndarray):
        """Insert every observed obstacle point; occupied evidence is sticky."""
        points = np.asarray(points_xy, dtype=np.float64).reshape((-1, 2))
        if points.size == 0:
            return
        xs = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(int)
        ys = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(int)
        valid = ((xs >= 0) & (xs < self.width) & (ys >= 0) & (ys < self.height))
        self.data[ys[valid], xs[valid]] = OCCUPIED

    def inflated_obstacles(self, radius: float) -> Set[Cell]:
        radius_cells = int(math.ceil(radius / self.resolution))
        offsets = [(dx, dy)
                   for dx in range(-radius_cells, radius_cells + 1)
                   for dy in range(-radius_cells, radius_cells + 1)
                   if math.hypot(dx * self.resolution, dy * self.resolution) <= radius + 1e-9]
        ys, xs = np.where(self.data == OCCUPIED)
        result: Set[Cell] = set()
        for x, y in zip(xs.tolist(), ys.tolist()):
            for dx, dy in offsets:
                cell = (x + dx, y + dy)
                if self.in_bounds(cell):
                    result.add(cell)
        return result

    def planning_free(self, cell: Cell, inflated: Set[Cell]) -> bool:
        return self.in_bounds(cell) and self.value(cell) == FREE and cell not in inflated

    def frontier_cells(self, inflated: Set[Cell]) -> Set[Cell]:
        result = set()
        for y, x in np.argwhere(self.data == FREE):
            cell = (int(x), int(y))
            if cell in inflated:
                continue
            neighbours = ((cell[0] - 1, cell[1]), (cell[0] + 1, cell[1]),
                          (cell[0], cell[1] - 1), (cell[0], cell[1] + 1))
            if any(self.in_bounds(n) and self.value(n) == UNKNOWN for n in neighbours):
                result.add(cell)
        return result


def cluster_frontiers(frontiers: Set[Cell], minimum_size: int) -> List[List[Cell]]:
    remaining = set(frontiers)
    clusters = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        queue = deque([seed])
        cluster = []
        while queue:
            current = queue.popleft()
            cluster.append(current)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbour = (current[0] + dx, current[1] + dy)
                    if (dx or dy) and neighbour in remaining:
                        remaining.remove(neighbour)
                        queue.append(neighbour)
        if len(cluster) >= minimum_size:
            clusters.append(cluster)
    return clusters


def observation_target_cells(grid: ExplorationGrid, center: Cell,
                             radius: float) -> Set[Cell]:
    """Freeze the currently unknown cells associated with an observation task.

    The set is deliberately captured when a task becomes active.  Recomputing
    it after every map update would move the denominator and make progress
    ratios meaningless.
    """
    radius_cells = int(math.ceil(max(0.0, radius) / grid.resolution))
    result: Set[Cell] = set()
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            if math.hypot(dx * grid.resolution, dy * grid.resolution) > radius:
                continue
            cell = (center[0] + dx, center[1] + dy)
            if grid.in_bounds(cell) and grid.value(cell) == UNKNOWN:
                result.add(cell)
    return result


def observation_progress(grid: ExplorationGrid,
                         target_cells: Set[Cell]) -> Tuple[int, int, float]:
    """Return realised cells, expected cells and their fixed-denominator ratio."""
    expected = len(target_cells)
    if expected == 0:
        return 0, 0, 1.0
    observed = sum(1 for cell in target_cells if grid.value(cell) != UNKNOWN)
    return observed, expected, observed / expected


def advance_completion_streak(progress: float, done_ratio: float,
                              current_streak: int) -> int:
    """Require consecutive map updates above threshold before completion."""
    return current_streak + 1 if progress >= done_ratio else 0


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


def candidate_cells(cluster: Sequence[Cell]) -> List[Cell]:
    if not cluster:
        return []
    cx = sum(cell[0] for cell in cluster) / len(cluster)
    cy = sum(cell[1] for cell in cluster) / len(cluster)
    choices = {
        min(cluster, key=lambda cell: (cell[0], cell[1])),
        max(cluster, key=lambda cell: (cell[0], cell[1])),
        min(cluster, key=lambda cell: (cell[1], cell[0])),
        max(cluster, key=lambda cell: (cell[1], cell[0])),
        min(cluster, key=lambda cell: ((cell[0] - cx) ** 2 + (cell[1] - cy) ** 2,
                                       cell[0], cell[1])),
    }
    return sorted(choices)


def safe_viewpoint_cells(grid: ExplorationGrid, frontier: Cell,
                         inflated: Set[Cell], stand_off: float,
                         limit: int = 3) -> List[Cell]:
    """Return known-free stand-off poses that look toward a frontier.

    A frontier is an information boundary, not a place the robot should stand.
    Candidate viewpoints therefore sit behind the frontier, on its known side,
    while the frozen observation objective remains centred on the frontier.
    """
    unknown_neighbours = []
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        neighbour = (frontier[0] + dx, frontier[1] + dy)
        if grid.in_bounds(neighbour) and grid.value(neighbour) == UNKNOWN:
            unknown_neighbours.append((dx, dy))
    if not unknown_neighbours:
        return []

    unknown_dx = sum(offset[0] for offset in unknown_neighbours)
    unknown_dy = sum(offset[1] for offset in unknown_neighbours)
    norm = math.hypot(unknown_dx, unknown_dy)
    if norm < 1e-9:
        return []
    inward = (-unknown_dx / norm, -unknown_dy / norm)

    desired_cells = max(1.0, stand_off / grid.resolution)
    tolerance_cells = max(2.0, 0.4 / grid.resolution)
    minimum_cells = max(1.0, desired_cells - tolerance_cells)
    maximum_cells = desired_cells + tolerance_cells
    search_cells = int(math.ceil(maximum_cells))
    ranked = []
    for dx in range(-search_cells, search_cells + 1):
        for dy in range(-search_cells, search_cells + 1):
            distance = math.hypot(dx, dy)
            if not minimum_cells <= distance <= maximum_cells:
                continue
            cell = (frontier[0] + dx, frontier[1] + dy)
            if not grid.planning_free(cell, inflated):
                continue
            # Require the pose to lie on the known side of the boundary and
            # retain a known-free line of sight to the observed frontier.
            inward_progress = dx * inward[0] + dy * inward[1]
            if inward_progress <= 0.25 * distance:
                continue
            if not segment_known_free(grid, cell, frontier, inflated):
                continue
            desired_x = inward[0] * desired_cells
            desired_y = inward[1] * desired_cells
            desired_error = math.hypot(dx - desired_x, dy - desired_y)
            ranked.append(((desired_error, abs(distance - desired_cells),
                            cell[0], cell[1]), cell))
    return [item[1] for item in sorted(ranked)[:max(1, int(limit))]]


def path_length_cells(path: Sequence[Cell], resolution: float) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) * resolution
               for a, b in zip(path[:-1], path[1:]))


def path_turn_cost(path: Sequence[Cell]) -> float:
    """Total absolute heading change of a grid path in radians."""
    headings = [math.atan2(b[1] - a[1], b[0] - a[0])
                for a, b in zip(path[:-1], path[1:]) if a != b]
    return sum(abs(math.atan2(math.sin(b - a), math.cos(b - a)))
               for a, b in zip(headings[:-1], headings[1:]))


@dataclass(frozen=True)
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


@dataclass
class FrontierRegion:
    """A spatial group of frontier clusters with an ID stable across updates."""

    region_id: int
    clusters: List[List[Cell]]
    centroid: Tuple[float, float]
    cells: Set[Cell]


def partition_frontier_clusters(clusters: Sequence[Sequence[Cell]],
                                region_size_cells: int) -> List[FrontierRegion]:
    """Group frontier clusters by spatial adjacency, not fixed map buckets.

    A fixed world-aligned grid can split two neighbouring frontiers merely
    because their centroids lie on opposite sides of a bucket boundary.  The
    resulting artificial region change defeats commitment.  Here clusters are
    grouped by centroid proximity, so grouping is invariant to the map origin
    and bucket edges.  Complete-linkage keeps every pair of cluster centroids
    within the configured span, preventing a long frontier chain from merging
    half of the map into one oversized ATSP node.
    """
    nonempty = [list(cluster) for cluster in clusters if cluster]
    if not nonempty:
        return []

    link_distance = max(1.0, float(region_size_cells))
    centroids = [
        (sum(cell[0] for cell in cluster) / len(cluster),
         sum(cell[1] for cell in cluster) / len(cluster))
        for cluster in nonempty]
    groups: List[List[int]] = [[index] for index in range(len(nonempty))]
    while True:
        best_pair = None
        for first in range(len(groups)):
            for second in range(first + 1, len(groups)):
                pair_distances = [
                    math.hypot(centroids[a][0] - centroids[b][0],
                               centroids[a][1] - centroids[b][1])
                    for a in groups[first] for b in groups[second]]
                diameter = max(pair_distances)
                if diameter > link_distance:
                    continue
                key = (min(pair_distances), diameter, first, second)
                if best_pair is None or key < best_pair[0]:
                    best_pair = (key, first, second)
        if best_pair is None:
            break
        _, first, second = best_pair
        groups[first].extend(groups[second])
        del groups[second]

    observations = []
    for group in groups:
        region_clusters = [nonempty[index] for index in sorted(group)]
        cells = {cell for cluster in region_clusters for cell in cluster}
        centroid = (sum(cell[0] for cell in cells) / len(cells),
                    sum(cell[1] for cell in cells) / len(cells))
        observations.append(FrontierRegion(-1, region_clusters, centroid, cells))
    return observations


class PersistentRegionTracker:
    """Associate dynamic frontier regions without relying on list indices."""

    def __init__(self, match_distance_cells: float):
        self.match_distance_cells = float(match_distance_cells)
        self.next_id = 0
        self.regions: Dict[int, FrontierRegion] = {}

    def update(self, observations: Sequence[FrontierRegion],
               preferred_region_id: Optional[int] = None) -> List[FrontierRegion]:
        previous = dict(self.regions)
        matched_observations: Dict[int, int] = {}
        matched_regions: Set[int] = set()

        # Match globally instead of letting observation iteration order decide
        # which child keeps an old ID after a split.  The committed region gets
        # first choice of its best-overlapping successor, then all other pairs
        # are assigned by overlap and distance.
        pairs = []
        for observation_index, observation in enumerate(observations):
            for region_id, old in previous.items():
                distance = math.hypot(observation.centroid[0] - old.centroid[0],
                                      observation.centroid[1] - old.centroid[1])
                if distance > self.match_distance_cells:
                    continue
                union = observation.cells | old.cells
                overlap = len(observation.cells & old.cells) / len(union) if union else 0.0
                preferred = region_id == preferred_region_id
                pairs.append((not preferred, -overlap, distance,
                              region_id, observation_index))

        for _, _, _, region_id, observation_index in sorted(pairs):
            if (observation_index in matched_observations
                    or region_id in matched_regions):
                continue
            matched_observations[observation_index] = region_id
            matched_regions.add(region_id)

        assigned = []
        for observation_index, observation in enumerate(observations):
            best_id = matched_observations.get(observation_index)
            if best_id is None:
                best_id = self.next_id
                self.next_id += 1
            assigned.append(FrontierRegion(
                best_id, observation.clusters, observation.centroid, observation.cells))
        self.regions = {region.region_id: region for region in assigned}
        return sorted(assigned, key=lambda region: region.region_id)


def advance_region_release(active_region_id: Optional[int],
                           available_region_ids: Set[int],
                           missing_streak: int,
                           release_updates: int
                           ) -> Tuple[Optional[int], int, bool]:
    """Debounce release of a committed region with no usable candidate."""
    if active_region_id is None or active_region_id in available_region_ids:
        return active_region_id, 0, False
    missing_streak += 1
    if missing_streak >= max(1, release_updates):
        return None, 0, True
    return active_region_id, missing_streak, False


def solve_open_held_karp(cost_matrix: Sequence[Sequence[float]],
                         forced_first: Optional[int] = None) -> Optional[List[int]]:
    """Minimum-cost open tour starting at matrix node 0.

    Returned values are zero-based region indices; unlike a normal TSP no cost
    is added from the final region back to the robot.  The matrix may be
    asymmetric.  ``forced_first`` is also a zero-based region index and is used
    to preserve commitment to the current region across global replans.
    """
    costs = np.asarray(cost_matrix, dtype=np.float64)
    if costs.ndim != 2 or costs.shape[0] != costs.shape[1] or costs.shape[0] < 1:
        raise ValueError("cost matrix must be square and contain the start node")
    region_count = costs.shape[0] - 1
    if region_count == 0:
        return []
    if forced_first is not None and not 0 <= forced_first < region_count:
        raise ValueError("forced_first is outside the region range")

    # (visited mask, last region) -> (cost, predecessor)
    dynamic: Dict[Tuple[int, int], Tuple[float, Optional[int]]] = {}
    initial = range(region_count) if forced_first is None else (forced_first,)
    for region in initial:
        edge = float(costs[0, region + 1])
        if math.isfinite(edge):
            dynamic[(1 << region, region)] = (edge, None)

    for mask in range(1, 1 << region_count):
        for last in range(region_count):
            state = dynamic.get((mask, last))
            if state is None:
                continue
            for nxt in range(region_count):
                if mask & (1 << nxt):
                    continue
                edge = float(costs[last + 1, nxt + 1])
                if not math.isfinite(edge):
                    continue
                new_mask = mask | (1 << nxt)
                candidate = state[0] + edge
                old = dynamic.get((new_mask, nxt))
                if old is None or candidate < old[0]:
                    dynamic[(new_mask, nxt)] = (candidate, last)

    full_mask = (1 << region_count) - 1
    endings = [(value[0], last) for (mask, last), value in dynamic.items()
               if mask == full_mask]
    if not endings:
        return None
    _, last = min(endings)
    route = []
    mask = full_mask
    while True:
        route.append(last)
        predecessor = dynamic[(mask, last)][1]
        if predecessor is None:
            break
        mask ^= 1 << last
        last = predecessor
    route.reverse()
    return route


def retain_region_commitment(sequence: Sequence[int],
                             available_region_ids: Set[int],
                             active_region_id: Optional[int]) -> Tuple[List[int], bool]:
    """Keep a usable region tour until the committed region is exhausted.

    Newly observed regions are considered at the next event-driven global
    replan; their appearance alone must not reshuffle an executable tour.
    """
    retained = [region_id for region_id in sequence
                if region_id in available_region_ids]
    active_lost = (active_region_id is not None
                   and active_region_id not in available_region_ids)
    return retained, not retained or active_lost


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
        self.max_global_regions = int(
            self.declare_parameter("max_global_regions", 10).value)
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
        self.global_path_hold_active = False
        self.global_path_hold_started_ns = 0
        self.global_path_hold_reason = ""
        self.active_since_ns = 0
        self.blacklist: List[Point2] = []
        self.completed_goals: List[Point2] = []
        self.temporary_blocked_edges: Dict[DirectedEdge, int] = {}
        self.pending_blocked_edge: Optional[DirectedEdge] = None
        self.disabled_by_collision = False
        self.region_tracker = PersistentRegionTracker(
            self.region_match_distance / self.grid.resolution)
        self.region_sequence: List[int] = []
        self.active_region_id: Optional[int] = None
        self.active_region_missing_streak = 0
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
        self.execution_stop_pub = self.create_publisher(
            Bool, "planning/emergency_stop", 10)
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
        inflated.discard(start)
        frontiers = self.grid.frontier_cells(inflated)
        clusters = cluster_frontiers(frontiers, self.min_frontier_size)
        self.latest_frontier_count = len(frontiers)
        self.publish_frontiers(frontiers)
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

        inflated = self.grid.inflated_obstacles(self.inflation_radius)
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        inflated.discard(start)
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
        if (self.last_selected_region_id is not None
                and candidate.region_id != self.last_selected_region_id):
            self.region_switches += 1
        self.last_selected_region_id = candidate.region_id
        self.active_region_id = candidate.region_id
        self.publish_path(path, candidate)

    def reroute_active_goal(self) -> bool:
        """Replan to the current observation pose without resetting its gain."""
        if (self.position is None or self.active_goal is None
                or self.active_goal_cell is None or self.active_observation is None):
            return False

        inflated = self.grid.inflated_obstacles(self.inflation_radius)
        blocked_edges = self.current_blocked_edges()
        start = self.grid.world_to_cell(*self.position)
        inflated.discard(start)
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

    def request_execution_stop(self, reason: str):
        """Freeze the old local trajectory before replacing an invalid path."""
        if self.global_path_hold_active:
            self.get_logger().debug(
                f"[GLOBAL_PATH_HOLD_ACTIVE] original_reason="
                f"{self.global_path_hold_reason} latest_reason={reason}")
            return
        msg = Bool()
        msg.data = True
        self.execution_stop_pub.publish(msg)
        self.global_path_hold_active = True
        self.global_path_hold_started_ns = self.get_clock().now().nanoseconds
        self.global_path_hold_reason = reason
        self.publish_status("GLOBAL_PATH_HOLD_REQUESTED")
        self.get_logger().warning(
            f"[GLOBAL_PATH_HOLD] reason={reason}; old local trajectory stopped "
            "before global-route replacement")

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

        self.request_execution_stop(check.reason)
        started = time.perf_counter()
        if self.reroute_active_goal():
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            self.publish_status("GLOBAL_PATH_REROUTED_TO_SAME_OBSERVATION")
            self.get_logger().info(
                f"[GLOBAL_REROUTE_OK] mode=same_observation goal={goal_text} "
                f"path_cells={len(self.active_raw_path)} elapsed={elapsed_ms:.1f}ms; "
                "waiting for a replacement local trajectory")
            return False

        previous_goal = self.active_goal
        if self.plan_from_current_position(excluded_goals=(previous_goal,)):
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            new_goal = self.active_goal
            new_goal_text = ("none" if new_goal is None else
                             f"({new_goal[0]:.2f},{new_goal[1]:.2f})")
            self.publish_status("GLOBAL_PATH_SWITCHED_OBSERVATION")
            self.get_logger().info(
                f"[GLOBAL_REROUTE_OK] mode=new_observation old_goal={goal_text} "
                f"new_goal={new_goal_text} elapsed={elapsed_ms:.1f}ms; "
                "waiting for a replacement local trajectory")
            return False

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.publish_status("GLOBAL_PATH_INVALID_WAITING_FOR_ROUTE")
        self.get_logger().error(
            f"[GLOBAL_REROUTE_FAILED] goal={goal_text} reason={check.reason} "
            f"elapsed={elapsed_ms:.1f}ms; robot remains stopped and the current "
            "observation task is retained for the next map update",
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
        if msg.data == "PATH_TRAJECTORY_READY" and self.pending_path_publish_ns:
            latency = ((self.get_clock().now().nanoseconds - self.pending_path_publish_ns) * 1e-9)
            if latency >= 0.0:
                self.pipeline_latency_ewma = (
                    latency if self.pipeline_latency_ewma <= 0.0 else
                    0.8 * self.pipeline_latency_ewma + 0.2 * latency)
                self.get_logger().info(
                    f"[LOCAL_TRAJECTORY_READY] pipeline_latency={latency:.2f}s, "
                    f"ewma={self.pipeline_latency_ewma:.2f}s")
            self.pending_path_publish_ns = 0
            if self.global_path_hold_active:
                hold_time = ((self.get_clock().now().nanoseconds
                              - self.global_path_hold_started_ns) * 1e-9)
                self.get_logger().info(
                    f"[GLOBAL_PATH_RESUME_READY] replacement local trajectory "
                    f"ready after {hold_time:.2f}s; original_reason="
                    f"{self.global_path_hold_reason}")
                self.global_path_hold_active = False
                self.global_path_hold_started_ns = 0
                self.global_path_hold_reason = ""
        elif msg.data in ("RUNNING", "PATH_ACCEPTED"):
            # A successful replacement trajectory resolves the pending local
            # failure.  Keep the short-lived edge record, but do not let it be
            # mistaken for the cause of a later unrelated BLOCKED status.
            self.pending_blocked_edge = None
        elif msg.data == "REACHED" and self.active_goal is not None:
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
        elif msg.data == "BLOCKED":
            self.pending_path_publish_ns = 0
            self.global_path_hold_active = False
            self.global_path_hold_started_ns = 0
            self.global_path_hold_reason = ""
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
            if previous_goal is not None and self.plan_from_current_position(
                    excluded_goals=(previous_goal,)):
                self.publish_status("LOCAL_BLOCKED_VIEWPOINT_CHANGED")
            else:
                self.publish_status("LOCAL_BLOCKED_WAITING_FOR_ROUTE")
        elif msg.data in ("REFERENCE_PATH_REJECTED", "INVALID_REFERENCE_PATH"):
            self.pending_path_publish_ns = 0
            self.global_path_hold_active = False
            self.global_path_hold_started_ns = 0
            self.global_path_hold_reason = ""
            if self.active_goal is not None:
                self.blacklist.append(self.active_goal)
            self.active_goal = None
            self.active_goal_cell = None
            self.active_observation = None
            self.active_raw_path = []
            self.pending_blocked_edge = None
            self.prepared_candidate = None
            self.replacement_pending = False
            self.publish_status(f"RECOVER_FROM_{msg.data}")

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
        inflated.discard(start)
        frontiers = self.grid.frontier_cells(inflated)
        clusters = cluster_frontiers(frontiers, self.min_frontier_size)
        self.latest_frontier_count = len(frontiers)
        self.publish_frontiers(frontiers)
        best = self.choose_frontier(
            start, clusters, inflated, blocked_edges,
            excluded_goals=excluded_goals)
        if best is None:
            status = "EXPLORATION_COMPLETE" if not frontiers else "NO_REACHABLE_FRONTIER"
            self.publish_status(status)
            self.get_logger().info(
                f"{status}: {len(frontiers)} frontier cells", throttle_duration_sec=5.0)
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
                                or self.is_blacklisted(goal_xy) or excluded):
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

        if (allow_region_release
                and self.last_region_release_evaluation_update != self.map_update_count):
            previous_active = self.active_region_id
            self.active_region_id, self.active_region_missing_streak, released = (
                advance_region_release(
                    self.active_region_id, available_ids,
                    self.active_region_missing_streak,
                    self.region_release_updates))
            self.last_region_release_evaluation_update = self.map_update_count
            if released:
                self.get_logger().info(
                    f"[REGION_COMMITMENT_RELEASED] region={previous_active} "
                    f"after {self.region_release_updates} map updates without "
                    "a usable observation candidate")

        if (self.active_region_id is not None
                and self.active_region_id not in available_ids):
            if allow_region_release:
                self.get_logger().info(
                    f"[REGION_COMMITMENT_RETAINED] region={self.active_region_id} "
                    f"missing={self.active_region_missing_streak}/"
                    f"{self.region_release_updates}; waiting for a stable candidate",
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
            self.publish_region_sequence()

        # If a region has no usable goal, remove it and continue along the
        # current global sequence rather than falling back across all regions.
        while self.region_sequence:
            region_id = self.region_sequence[0]
            options = by_region.get(region_id, [])
            if options:
                selected = max(options, key=lambda item: (
                    self.candidate_utility(item, options),
                    -item.path_length, -item.cell[0], -item.cell[1]))
                return selected
            self.region_sequence.pop(0)
        return None

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
        msg.header.stamp = self.get_clock().now().to_msg()
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
        self.pending_path_publish_ns = self.get_clock().now().nanoseconds

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
