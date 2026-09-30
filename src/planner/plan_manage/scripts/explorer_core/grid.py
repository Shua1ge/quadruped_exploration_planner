"""Occupancy-grid construction primitives for frontier exploration."""

import bisect
import math
from typing import List, Optional, Sequence, Set, Tuple

import numpy as np

Cell = Tuple[int, int]
Point2 = Tuple[float, float]
DirectedEdge = Tuple[Cell, Cell]
# timestamp, body position xyz, body quaternion xyzw
PoseSample = Tuple[int, float, float, float, float, float, float, float]
UNKNOWN = -1
FREE = 0
OCCUPIED = 100
GRID_MOVES = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
)

def _normalized_quaternion(values: Sequence[float]) -> Optional[np.ndarray]:
    quaternion = np.asarray(values, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm < 1e-12:
        return None
    return quaternion / norm


def slerp_quaternion(before: Sequence[float], after: Sequence[float],
                     ratio: float) -> Optional[np.ndarray]:
    """Interpolate xyzw quaternions along the shortest unit-sphere arc."""
    first = _normalized_quaternion(before)
    second = _normalized_quaternion(after)
    if first is None or second is None:
        return None
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    alpha = min(1.0, max(0.0, float(ratio)))
    if dot > 0.9995:
        return _normalized_quaternion(first + alpha * (second - first))
    angle = math.acos(dot)
    sine = math.sin(angle)
    if abs(sine) < 1e-12:
        return first
    return ((math.sin((1.0 - alpha) * angle) / sine) * first
            + (math.sin(alpha * angle) / sine) * second)


def quaternion_rotation_matrix(quaternion: Sequence[float]) -> Optional[np.ndarray]:
    """Return the 3-D rotation matrix for an xyzw quaternion."""
    normalized = _normalized_quaternion(quaternion)
    if normalized is None:
        return None
    x, y, z, w = normalized
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def rpy_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return Rz(yaw) * Ry(pitch) * Rx(roll), matching GridMap."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def interpolate_se3_pose(
        history: Sequence[PoseSample], stamp_ns: int,
        max_interpolation_gap_ns: int,
        max_nearest_age_ns: int) -> Optional[Tuple[float, float, float,
                                                   float, float, float, float]]:
    """Match GridMap's timestamp lookup and interpolate full body SE(3)."""
    if not history:
        return None
    times = [sample[0] for sample in history]
    index = bisect.bisect_left(times, stamp_ns)
    if index < len(history) and history[index][0] == stamp_ns:
        return history[index][1:]
    if index == 0:
        if history[0][0] - stamp_ns > max_nearest_age_ns:
            return None
        return history[0][1:]
    if index == len(history):
        if stamp_ns - history[-1][0] > max_nearest_age_ns:
            return None
        return history[-1][1:]
    before, after = history[index - 1], history[index]
    span = after[0] - before[0]
    if span <= 0 or span > max_interpolation_gap_ns:
        return None
    ratio = min(1.0, max(0.0, (stamp_ns - before[0]) / span))
    orientation = slerp_quaternion(before[4:8], after[4:8], ratio)
    if orientation is None:
        return None
    position = tuple(
        before[axis] + ratio * (after[axis] - before[axis])
        for axis in range(1, 4))
    return position + tuple(float(value) for value in orientation)


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


def classify_planar_ranges(
        dx: np.ndarray, dy: np.ndarray, point_z: np.ndarray,
        min_z: float, max_z: float, min_range: float, max_range: float,
        ray_count: int, dilation_bins: int) -> np.ndarray:
    """Build three-state planar rays from finite 3-D returns.

    Finite values are accepted obstacle hits, ``inf`` means a genuine empty
    bearing that may clear conservative near-field free space, and ``nan``
    means a real return was observed but rejected by the height projection.
    Rejected returns are deliberately non-clearing: on a pitched robot they
    are commonly the wall base or terrain, not evidence of free space.
    """
    dx = np.asarray(dx, dtype=np.float64)
    dy = np.asarray(dy, dtype=np.float64)
    point_z = np.asarray(point_z, dtype=np.float64)
    if dx.shape != dy.shape or dx.shape != point_z.shape:
        raise ValueError("dx, dy and point_z must have matching shapes")
    if ray_count <= 0:
        raise ValueError("ray_count must be positive")

    distances = np.hypot(dx, dy)
    finite_return = (np.isfinite(dx) & np.isfinite(dy)
                     & np.isfinite(point_z) & np.isfinite(distances))
    projectable = (finite_return & (distances >= min_range)
                   & (distances < max_range))
    angles = np.arctan2(dy[projectable], dx[projectable])
    bins = np.floor(
        (angles + math.pi) * ray_count / (2.0 * math.pi)).astype(int)
    bins = np.clip(bins, 0, ray_count - 1)

    return_seen = np.zeros(ray_count, dtype=bool)
    return_seen[bins] = True
    accepted = projectable & (point_z >= min_z) & (point_z <= max_z)
    accepted_angles = np.arctan2(dy[accepted], dx[accepted])
    accepted_bins = np.floor(
        (accepted_angles + math.pi) * ray_count / (2.0 * math.pi)).astype(int)
    accepted_bins = np.clip(accepted_bins, 0, ray_count - 1)
    nearest = np.full(ray_count, np.inf, dtype=np.float64)
    if accepted_bins.size:
        np.minimum.at(nearest, accepted_bins, distances[accepted])
    nearest = dilate_hit_ranges(nearest, dilation_bins)

    # Conservative dilation may turn a filtered bearing into a neighbouring
    # valid hit.  A finite obstacle hit wins; only remaining filtered bearings
    # become non-clearing NaNs.
    filtered_return = return_seen & ~np.isfinite(nearest)
    nearest[filtered_return] = np.nan
    return nearest


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
        # An occupied cell remains inflated until three consecutive valid free
        # observations overcome its first hit.  Ignored/height-filtered rays
        # never contribute negative evidence.
        self.occupied_clear_threshold = -2
        self.evidence_limit = 32

    def snapshot(self) -> "ExplorationGrid":
        """Return an independent map image for a planning cycle.

        Planning may take hundreds of milliseconds.  Copy the two dense arrays
        once so sensor callbacks can continue updating the live grid without
        changing the occupancy revision underneath A* and frontier scoring.
        """
        result = ExplorationGrid(
            self.width * self.resolution,
            self.height * self.resolution,
            self.resolution,
            self.origin_x,
            self.origin_y)
        result.data = self.data.copy()
        result.evidence = self.evidence.copy()
        result.occupied_threshold = self.occupied_threshold
        result.free_threshold = self.free_threshold
        result.occupied_clear_threshold = self.occupied_clear_threshold
        result.evidence_limit = self.evidence_limit
        return result

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
            if (self.data[y, x] == OCCUPIED
                    and self.evidence[y, x] > self.occupied_clear_threshold):
                continue
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
            # NaN is an observed but deliberately ignored 3-D return.  It is
            # neither an obstacle hit in the 2-D projection nor free-space
            # evidence.  Infinity alone represents a genuine no-return ray.
            if math.isnan(measured_range):
                continue
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
                # A missing return is free-space evidence only up to the first
                # wall already present when this scan began.  Let it weaken
                # that cell by one evidence step, but do not clear cells behind
                # the wall in the same observation.  Once repeated valid rays
                # have actually revoked the wall, a later scan may continue
                # into the newly visible space.
                first_occluder = next((offset for offset, cell in enumerate(line)
                                       if self.value(cell) == OCCUPIED), None)
                if first_occluder is None:
                    free_cells.update(line)
                else:
                    free_cells.update(line[:first_occluder + 1])

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
