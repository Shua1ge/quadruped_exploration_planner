"""Occupancy-grid construction primitives for frontier exploration."""

import bisect
import math
from typing import List, Optional, Sequence, Set, Tuple

import numpy as np

Cell = Tuple[int, int]
Point2 = Tuple[float, float]
DirectedEdge = Tuple[Cell, Cell]
PoseSample = Tuple[int, float, float, float, float]
UNKNOWN = -1
FREE = 0
OCCUPIED = 100
GRID_MOVES = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
)

def interpolate_planar_pose(history: Sequence[PoseSample], stamp_ns: int,
                             max_age_ns: int) -> Optional[Tuple[float, float, float, float]]:
    """Interpolate x/y/yaw at a sensor timestamp from an ordered pose history."""
    if not history:
        return None
    times = [sample[0] for sample in history]
    if stamp_ns < times[0] or stamp_ns > times[-1]:
        nearest_age = min(abs(stamp_ns - times[0]), abs(stamp_ns - times[-1]))
        if nearest_age > max_age_ns:
            return None
        sample = history[0] if stamp_ns < times[0] else history[-1]
        return sample[1], sample[2], sample[3], sample[4]
    index = bisect.bisect_left(times, stamp_ns)
    if index == 0:
        sample = history[0]
        return sample[1], sample[2], sample[3], sample[4]
    if index == len(history):
        sample = history[-1]
        return sample[1], sample[2], sample[3], sample[4]
    before, after = history[index - 1], history[index]
    span = max(1, after[0] - before[0])
    ratio = min(1.0, max(0.0, (stamp_ns - before[0]) / span))
    yaw_delta = (after[3] - before[3] + math.pi) % (2.0 * math.pi) - math.pi
    return (
        before[1] + ratio * (after[1] - before[1]),
        before[2] + ratio * (after[2] - before[2]),
        before[3] + ratio * yaw_delta,
        before[4] + ratio * (after[4] - before[4]),
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
        # Occupancy is derived from bounded temporal evidence rather than being
        # sticky forever after one point-cloud hit.  A single hit remains
        # immediately useful to the planner, while repeated misses can revoke
        # a transient or time-misaligned wall return.
        self.evidence = np.zeros((self.height, self.width), dtype=np.int16)
        self.occupied_threshold = 1
        self.free_threshold = -1
        self.evidence_limit = 32

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

    def _apply_evidence(self, free_cells: Set[Cell], occupied_cells: Set[Cell]):
        """Apply one scan's evidence with hysteresis and reversible occupancy."""
        for x, y in free_cells - occupied_cells:
            evidence = int(self.evidence[y, x]) - 1
            self.evidence[y, x] = max(-self.evidence_limit, evidence)
            if self.evidence[y, x] <= self.free_threshold:
                self.data[y, x] = FREE
            elif self.evidence[y, x] < self.occupied_threshold:
                self.data[y, x] = UNKNOWN
        for x, y in occupied_cells:
            evidence = int(self.evidence[y, x]) + 1
            self.evidence[y, x] = min(self.evidence_limit, evidence)
            if self.evidence[y, x] >= self.occupied_threshold:
                self.data[y, x] = OCCUPIED

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

        self._apply_evidence(free_cells, occupied_cells)

        seed_radius = max(1, int(math.ceil(0.35 / self.resolution)))
        for dx in range(-seed_radius, seed_radius + 1):
            for dy in range(-seed_radius, seed_radius + 1):
                cell = (origin[0] + dx, origin[1] + dy)
                if self.in_bounds(cell) and math.hypot(dx, dy) <= seed_radius:
                    self.evidence[cell[1], cell[0]] = min(
                        self.evidence[cell[1], cell[0]], self.free_threshold)
                    self.data[cell[1], cell[0]] = FREE

    def mark_occupied_points(self, points_xy: np.ndarray):
        """Add one hit of evidence per raw endpoint; never latch occupancy directly."""
        points = np.asarray(points_xy, dtype=np.float64).reshape((-1, 2))
        if points.size == 0:
            return
        xs = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(int)
        ys = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(int)
        valid = ((xs >= 0) & (xs < self.width) & (ys >= 0) & (ys < self.height))
        for x, y in zip(xs[valid].tolist(), ys[valid].tolist()):
            evidence = min(self.evidence_limit, int(self.evidence[y, x]) + 1)
            self.evidence[y, x] = evidence
            if evidence >= self.occupied_threshold:
                self.data[y, x] = OCCUPIED

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
