"""Occupancy-grid construction primitives for frontier exploration."""

import bisect
import math
import time
from typing import List, Optional, Sequence, Set, Tuple

import numpy as np
from scipy.ndimage import binary_dilation

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
    """Build observed-hit / unknown planar bearings from finite 3-D returns.

    Finite values are accepted obstacle hits. All other bearings are ``nan``:
    XYZ alone cannot distinguish a missing sample from a measured no-return.
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
    accepted = projectable & (point_z >= min_z) & (point_z <= max_z)
    accepted_angles = np.arctan2(dy[accepted], dx[accepted])
    accepted_bins = np.floor(
        (accepted_angles + math.pi) * ray_count / (2.0 * math.pi)).astype(int)
    accepted_bins = np.clip(accepted_bins, 0, ray_count - 1)
    nearest = np.full(ray_count, np.inf, dtype=np.float64)
    if accepted_bins.size:
        np.minimum.at(nearest, accepted_bins, distances[accepted])
    nearest = dilate_hit_ranges(nearest, dilation_bins)

    # Empty and filtered bearings are both unobserved for clearing purposes.
    # Genuine no-return evidence requires original beam identities, which
    # this XYZ-only projection does not have.
    nearest[~np.isfinite(nearest)] = np.nan
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
        # Last accepted sensor revision per cell (not robot-centred seeding).
        self.sensor_observed = np.zeros((self.height, self.width), dtype=np.uint64)
        self.sensor_revision = 0
        self.occupied_threshold = 1
        self.free_threshold = -1
        # An occupied cell remains inflated until three consecutive valid free
        # observations overcome its first hit.  Ignored/height-filtered rays
        # never contribute negative evidence.
        self.occupied_clear_threshold = -2
        self.evidence_limit = 32
        self._inflation_cache = [None]
        self.inflation_cache_hits = 0
        self.inflation_cache_misses = 0
        self.last_inflation_ms = 0.0

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
        result.sensor_observed = self.sensor_observed.copy()
        result.sensor_revision = self.sensor_revision
        result.occupied_threshold = self.occupied_threshold
        result.free_threshold = self.free_threshold
        result.occupied_clear_threshold = self.occupied_clear_threshold
        result.evidence_limit = self.evidence_limit
        # Share one bounded cache slot, including results generated by a
        # planning snapshot. Entries are immutable and atomically replaced;
        # every lookup checks its own occupancy before using an entry.
        result._inflation_cache = self._inflation_cache
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
        self.sensor_revision += 1
        for x, y in free_cells - occupied_cells:
            self.sensor_observed[y, x] = self.sensor_revision
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
            self.sensor_observed[y, x] = self.sensor_revision
            evidence = int(self.evidence[y, x]) + 1
            self.evidence[y, x] = min(self.evidence_limit, evidence)
            if self.evidence[y, x] >= self.occupied_threshold:
                self.data[y, x] = OCCUPIED

    def integrate_ranges(self, position: Point2, ranges: Sequence[float],
                         max_range: float, min_range: float = 0.35,
                         no_return_range: Optional[float] = None):
        """Fuse one planar scan.

        The batched implementation below is cell-for-cell equivalent to
        ``_integrate_ranges_reference``; equivalence is locked by
        test_grid_vectorized.py.
        """
        return self._integrate_ranges_vectorized(
            position, ranges, max_range, min_range, no_return_range)

    def _integrate_ranges_reference(self, position: Point2, ranges: Sequence[float],
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
                occupied_cells.add(line[-1])
            free_line = line[:-1] if hit else line
            if free_line:
                # A missing return is free-space evidence only up to the first
                # wall already present when this scan began.  Let it weaken
                # that cell by one evidence step, but do not clear cells behind
                # the wall in the same observation.  Once repeated valid rays
                # have actually revoked the wall, a later scan may continue
                # into the newly visible space.
                first_occluder = next((offset for offset, cell in enumerate(free_line)
                                       if self.value(cell) == OCCUPIED), None)
                if first_occluder is None:
                    free_cells.update(free_line)
                else:
                    free_cells.update(free_line[:first_occluder + 1])

        self._apply_evidence(free_cells, occupied_cells)

        seed_radius = max(1, int(math.ceil(0.35 / self.resolution)))
        for dx in range(-seed_radius, seed_radius + 1):
            for dy in range(-seed_radius, seed_radius + 1):
                cell = (origin[0] + dx, origin[1] + dy)
                if (self.in_bounds(cell) and math.hypot(dx, dy) <= seed_radius
                        and self.value(cell) != OCCUPIED):
                    self.evidence[cell[1], cell[0]] = min(
                        self.evidence[cell[1], cell[0]], self.free_threshold)
                    self.data[cell[1], cell[0]] = FREE

    def _integrate_ranges_vectorized(self, position: Point2, ranges: Sequence[float],
                                     max_range: float, min_range: float = 0.35,
                                     no_return_range: Optional[float] = None):
        """Exact batched port of _integrate_ranges_reference.

        The integer-Bresenham recurrence runs as a short loop over ray steps
        and is vectorised across rays; the evidence update is fully
        vectorised.  Equivalence with the reference implementation is locked
        by test_grid_vectorized.py.
        """
        origin = self.world_to_cell(*position)
        if not self.in_bounds(origin):
            return
        ray_count = len(ranges)
        if ray_count == 0:
            return
        ranges_arr = np.asarray(ranges, dtype=np.float64)
        is_nan = np.isnan(ranges_arr)
        # NaN is an observed but deliberately ignored 3-D return: neither an
        # obstacle hit in the 2-D projection nor free-space evidence.  A
        # genuine no-return ray is an infinity (or any finite range at/over
        # max_range) and frees up to miss_range.
        hit = ~is_nan & (ranges_arr < max_range)
        miss = ~is_nan & ~hit
        miss_range = (max_range if no_return_range is None
                      else min(max_range, no_return_range))
        ray_range = np.where(hit, np.minimum(ranges_arr, max_range), miss_range)
        valid = (hit | miss) & (ray_range >= min_range)
        if not valid.any():
            return
        idx = np.nonzero(valid)[0]
        is_hit = hit[idx]
        angles = -math.pi + (idx + 0.5) * (2.0 * math.pi / ray_count)
        ex = position[0] + ray_range[idx] * np.cos(angles)
        ey = position[1] + ray_range[idx] * np.sin(angles)
        x1 = np.floor((ex - self.origin_x) / self.resolution).astype(np.int64)
        y1 = np.floor((ey - self.origin_y) / self.resolution).astype(np.int64)
        x0 = np.full(idx.size, origin[0], dtype=np.int64)
        y0 = np.full(idx.size, origin[1], dtype=np.int64)
        dx = np.abs(x1 - x0)
        dy = np.abs(y1 - y0)
        sx = np.where(x0 < x1, 1, -1).astype(np.int64)
        sy = np.where(y0 < y1, 1, -1).astype(np.int64)
        error = dx - dy
        steps = int(np.max(np.maximum(dx, dy))) + 1
        # -1 sentinel: finished rays stop recording, and the sentinel cell can
        # never pass the in-bounds test below.
        px = np.full((steps, idx.size), -1, dtype=np.int64)
        py = np.full((steps, idx.size), -1, dtype=np.int64)
        active = np.ones(idx.size, dtype=bool)
        cx, cy = x0.copy(), y0.copy()
        for k in range(steps):
            px[k, active] = cx[active]
            py[k, active] = cy[active]
            active &= ~((cx == x1) & (cy == y1))
            if not active.any():
                break
            twice = 2 * error
            x_adv = active & (twice > -dy)
            y_adv = active & (twice < dx)
            error[x_adv] -= dy[x_adv]
            cx[x_adv] += sx[x_adv]
            error[y_adv] += dx[y_adv]
            cy[y_adv] += sy[y_adv]
        in_bounds = ((px >= 0) & (px < self.width) &
                     (py >= 0) & (py < self.height))
        any_inb = in_bounds.any(axis=0)
        k_axis = np.arange(steps, dtype=np.int64)[:, None]
        # The reference filters the whole path to in-bounds cells and uses
        # line[-1] as the hit endpoint, so the endpoint of a hit ray is its
        # last *in-bounds* cell, not necessarily the Bresenham terminus.
        last_ib = np.max(np.where(in_bounds, k_axis, -1), axis=0)
        occ_lookup = (self.data == OCCUPIED).reshape(-1)
        flat = np.where(in_bounds, py * self.width + px, 0)
        occ_along = np.zeros_like(in_bounds)
        occ_along[in_bounds] = occ_lookup[flat[in_bounds]]
        # Hit rays free everything except their endpoint cell.
        free_mask = in_bounds & np.where(is_hit[None, :],
                                         k_axis < last_ib[None, :], True)
        # A missing return is free-space evidence only up to the first wall
        # already present when this scan began; the wall cell itself keeps one
        # weakening step and cells behind it are untouched in this scan.
        first_occ = np.argmax(free_mask & occ_along, axis=0)
        has_occ = (free_mask & occ_along).any(axis=0)
        free_mask &= np.where(has_occ[None, :],
                              k_axis <= first_occ[None, :], True)
        free_flat = np.unique(flat[free_mask])
        occ_mask = np.zeros_like(in_bounds)
        hit_rays = np.nonzero(is_hit & any_inb)[0]
        occ_mask[last_ib[hit_rays], hit_rays] = True
        occ_flat = np.unique(flat[occ_mask])
        free_apply = np.setdiff1d(free_flat, occ_flat)

        self.sensor_revision += 1
        data_flat = self.data.reshape(-1)
        ev_flat = self.evidence.reshape(-1)
        seen_flat = self.sensor_observed.reshape(-1)
        if free_apply.size:
            new_ev = np.maximum(ev_flat[free_apply].astype(np.int64) - 1,
                                -self.evidence_limit)
            d = data_flat[free_apply]
            keep_occ = (d == OCCUPIED) & (new_ev > self.occupied_clear_threshold)
            to_free = (~keep_occ) & (new_ev <= self.free_threshold)
            to_unknown = (~keep_occ) & ~to_free & (new_ev < self.occupied_threshold)
            ev_flat[free_apply] = new_ev
            seen_flat[free_apply] = self.sensor_revision
            data_flat[free_apply[to_free]] = FREE
            data_flat[free_apply[to_unknown]] = UNKNOWN
        if occ_flat.size:
            new_ev = np.minimum(ev_flat[occ_flat].astype(np.int64) + 1,
                                self.evidence_limit)
            ev_flat[occ_flat] = new_ev
            seen_flat[occ_flat] = self.sensor_revision
            data_flat[occ_flat[new_ev >= self.occupied_threshold]] = OCCUPIED

        seed_radius = max(1, int(math.ceil(0.35 / self.resolution)))
        span = np.arange(-seed_radius, seed_radius + 1)
        gx = origin[0] + span
        gy = origin[1] + span
        mx = (gx >= 0) & (gx < self.width)
        my = (gy >= 0) & (gy < self.height)
        yy, xx = np.meshgrid(gy[my], gx[mx], indexing="ij")
        disk = np.hypot(xx - origin[0], yy - origin[1]) <= seed_radius
        yy, xx = yy[disk], xx[disk]
        not_occ = self.data[yy, xx] != OCCUPIED
        yy, xx = yy[not_occ], xx[not_occ]
        self.evidence[yy, xx] = np.minimum(self.evidence[yy, xx],
                                           self.free_threshold)
        self.data[yy, xx] = FREE

    def mark_occupied_points(self, points_xy: np.ndarray):
        """Add one hit of evidence per raw endpoint; never latch occupancy directly."""
        points = np.asarray(points_xy, dtype=np.float64).reshape((-1, 2))
        if points.size == 0:
            return
        xs = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(int)
        ys = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(int)
        valid = ((xs >= 0) & (xs < self.width) & (ys >= 0) & (ys < self.height))
        self.sensor_revision += 1
        for x, y in zip(xs[valid].tolist(), ys[valid].tolist()):
            self.sensor_observed[y, x] = self.sensor_revision
            evidence = min(self.evidence_limit, int(self.evidence[y, x]) + 1)
            self.evidence[y, x] = evidence
            if evidence >= self.occupied_threshold:
                self.data[y, x] = OCCUPIED

    def inflated_obstacles(self, radius: float) -> Set[Cell]:
        started = time.perf_counter()
        occupied = self.data == OCCUPIED
        key = (float(radius), self.resolution, self.width, self.height,
               self.origin_x, self.origin_y)
        cached = self._inflation_cache[0]
        if cached is not None and cached[0] == key and np.array_equal(cached[1], occupied):
            self.inflation_cache_hits += 1
            self.last_inflation_ms = (time.perf_counter() - started) * 1000.0
            return cached[2]
        radius_cells = int(math.ceil(radius / self.resolution))
        offsets = np.arange(-radius_cells, radius_cells + 1) * self.resolution
        structure = np.hypot(offsets[:, None], offsets[None, :]) <= radius + 1e-9
        inflated = binary_dilation(occupied, structure=structure, border_value=0)
        ys, xs = np.nonzero(inflated)
        result = frozenset(zip(xs.tolist(), ys.tolist()))
        occupied.flags.writeable = False
        self._inflation_cache[0] = (key, occupied, result, inflated)
        self.inflation_cache_misses += 1
        self.last_inflation_ms = (time.perf_counter() - started) * 1000.0
        return result

    def inflated_mask(self, radius: float) -> np.ndarray:
        """Boolean inflated-occupancy mask sharing the inflated_obstacles
        cache; lets vectorised consumers skip the per-cell frozenset."""
        self.inflated_obstacles(radius)
        return self._inflation_cache[0][3]

    def planning_free(self, cell: Cell, inflated: Set[Cell]) -> bool:
        return self.in_bounds(cell) and self.value(cell) == FREE and cell not in inflated

    def frontier_cells(self, inflated: Set[Cell]) -> Set[Cell]:
        """FREE cells with an UNKNOWN 4-neighbour, minus the inflated set.

        Candidate cells come from array shifts; inflated membership (a set
        lookup) is only tested on the small candidate set."""
        free = self.data == FREE
        unknown = self.data == UNKNOWN
        has_unknown_neighbour = np.zeros_like(free)
        has_unknown_neighbour[1:, :] |= unknown[:-1, :]
        has_unknown_neighbour[:-1, :] |= unknown[1:, :]
        has_unknown_neighbour[:, 1:] |= unknown[:, :-1]
        has_unknown_neighbour[:, :-1] |= unknown[:, 1:]
        ys, xs = np.nonzero(free & has_unknown_neighbour)
        result = set()
        for x, y in zip(xs.tolist(), ys.tolist()):
            if (x, y) not in inflated:
                result.add((x, y))
        return result
