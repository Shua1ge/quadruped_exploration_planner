"""Occupancy-grid construction primitives for frontier exploration."""

import bisect
import math
import time
from typing import List, NamedTuple, Optional, Sequence, Set, Tuple

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
# Return-type wire contract (single source of truth: plan_env/ground_height.hpp).
# 0 invalid; 1 measured return without a height estimate; 2 measured no-return;
# 3..252 measured return whose height above the observed local ground is
# kHeightBase + (value - 3) * kHeightStep, clamped to [-0.50, +1.99] m at 1 cm
# resolution; 253..255 reserved and treated as unknown.
HEIGHT_FIRST = 3
HEIGHT_LAST = 252
HEIGHT_BASE = -0.50
HEIGHT_STEP = 0.01
GROUND_FLOOR_TOLERANCE = 0.035


def decode_ground_height(return_types):
    """Height above the observed local ground per return; NaN where unknown."""
    kinds = np.asarray(return_types, dtype=np.int64)
    heights = np.full(kinds.shape, np.nan)
    known = (kinds >= HEIGHT_FIRST) & (kinds <= HEIGHT_LAST)
    heights[known] = HEIGHT_BASE + (kinds[known] - HEIGHT_FIRST) * HEIGHT_STEP
    return heights

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



def _project_identified_rays(values, hits, misses, origin, directions,
        min_z, max_z, mapping_range, no_return_range, ray_count, dilation_bins,
        resolution, spatial=False, clear_min_z=None, clear_max_z=None,
        support=None, endpoint_obstacles=None):
    """Project identified rays into 2-D obstacle and clearing evidence.

    ``min_z``/``max_z`` bound the untyped endpoint fallback. Identified 3-D
    rays supply their authoritative ``endpoint_obstacles`` mask instead.
    ``clear_min_z``/``clear_max_z``
    bound how far a ray may travel before its prefix stops counting as free
    space; they may be deliberately wider. The two are different decisions: an
    overhead beam is not an obstacle, yet a ray passing just above the body
    still proves the cells beneath it are traversable. Sharing one narrow band
    for both rejects every clearing prefix on a pitched 3-D lidar and leaves
    only the cells around the robot free, which is how a frontier disappears.
    """
    if clear_min_z is None:
        clear_min_z = min_z
    if clear_max_z is None:
        clear_max_z = max_z
    planar = np.linalg.norm(directions[:, :2], axis=1)
    valid = (hits | misses) & (planar > 1e-6)
    extent = np.where(hits, values, no_return_range)
    extent = np.minimum(extent, mapping_range / np.maximum(planar, 1e-6))
    endpoint_z = origin[2] + directions[:, 2] * np.where(hits, values, 0.)
    obstacle = (valid & hits & (endpoint_z >= min_z) & (endpoint_z <= max_z)
                & (values * planar < mapping_range))
    if endpoint_obstacles is not None:
        # The identified 3-D path owns the endpoint classification. Reusing its
        # mask here prevents the radial and exact-XY channels from disagreeing.
        obstacle = valid & hits & endpoint_obstacles & (values * planar < mapping_range)
    elif support is not None:
        obstacle &= ~support
    bearings = np.arctan2(directions[:, 1], directions[:, 0])
    bins = np.clip(np.floor((bearings + math.pi) * ray_count / (2 * math.pi)).astype(int),
                   0, ray_count - 1)
    nearest = np.full(ray_count, np.inf)
    np.minimum.at(nearest, bins[obstacle], values[obstacle] * planar[obstacle])
    nearest = dilate_hit_ranges(nearest, dilation_bins)
    prefix = np.zeros(values.size)
    if clear_min_z <= origin[2] <= clear_max_z:
        exit_distance = np.full(values.size, np.inf)
        up = directions[:, 2] > 1e-9
        down = directions[:, 2] < -1e-9
        exit_distance[up] = (clear_max_z - origin[2]) / directions[up, 2]
        exit_distance[down] = (clear_min_z - origin[2]) / directions[down, 2]
        # A low/ground hit terminates its own ray. Do not clear past the true
        # endpoint or outside the clearing window.
        guard = resolution * math.sqrt(2.) / np.maximum(planar, 1e-6)
        endpoint_limit = np.where(hits & ~obstacle, extent - guard, extent)
        band_limit = np.where(np.isfinite(exit_distance), exit_distance - guard, exit_distance)
        prefix[valid] = np.maximum(0., np.minimum(endpoint_limit, band_limit)[valid]) * planar[valid]
    if support is not None:
        # Supported, measured ground retains its own visible endpoint even
        # where it leaves a body-relative height window. Never extend past it.
        prefix[support & valid] = extent[support & valid] * planar[support & valid]
    if spatial:
        # Rays at different elevations have different valid prefixes. A downward
        # ray leaving the band immediately must not erase a measured horizontal
        # free prefix. Actual accepted obstacle hits still occlude every prefix.
        clearing = np.zeros(ray_count)
        np.maximum.at(clearing, bins[valid], prefix[valid])
    else:
        clearing = np.full(ray_count, np.inf)
        np.minimum.at(clearing, bins[valid], prefix[valid])
        clearing[~np.isfinite(clearing)] = 0.
    clearing = np.minimum(clearing, nearest)
    nearest[~np.isfinite(nearest) & (clearing <= 0.)] = np.nan
    return nearest, clearing


def classify_planar_scan(ranges: Sequence[float], angle_min: float,
        angle_increment: float, range_min: float, range_max: float,
        sensor_position: np.ndarray, sensor_rotation: np.ndarray,
        min_z: float, max_z: float, mapping_range: float,
        no_return_range: float, ray_count: int, dilation_bins: int,
        resolution: float) -> Tuple[np.ndarray, np.ndarray]:
    """Project simultaneous planar rays, preserving their evidence semantics."""
    values = np.asarray(ranges, dtype=np.float64)
    origin = np.asarray(sensor_position, dtype=np.float64)
    rotation = np.asarray(sensor_rotation, dtype=np.float64)
    if (values.ndim != 1 or origin.shape != (3,) or rotation.shape != (3, 3)
            or ray_count <= 0 or resolution <= 0
            or not np.all(np.isfinite(origin)) or not np.all(np.isfinite(rotation))
            or not all(math.isfinite(x) for x in (angle_min, angle_increment,
                range_min, range_max, min_z, max_z, mapping_range, no_return_range))
            or angle_increment == 0 or range_min < 0 or range_max <= range_min
            or min_z >= max_z or mapping_range <= 0 or no_return_range <= 0):
        raise ValueError("invalid planar scan geometry")
    angles = angle_min + np.arange(values.size) * angle_increment
    local = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(values.size)))
    directions = local @ rotation.T
    finite = np.isfinite(values) & (values >= range_min) & (values <= range_max)
    hits = finite & (values < range_max)
    misses = np.isposinf(values) | (finite & (values == range_max))
    return _project_identified_rays(values, hits, misses, origin, directions,
        min_z, max_z, mapping_range, min(range_max, no_return_range), ray_count,
        dilation_bins, resolution)


class SpatialRayProjection(NamedTuple):
    """One endpoint classification shared by radial and exact-XY fusion."""
    ranges: np.ndarray
    clearing: np.ndarray
    world: np.ndarray
    heights: np.ndarray
    support: np.ndarray
    obstacle: np.ndarray
    terrain: np.ndarray


def project_spatial_rays(endpoints, return_types, sensor_position, sensor_rotation,
        min_z, max_z, mapping_range, no_return_range, ray_count, dilation_bins,
        resolution, clear_min_z=None, clear_max_z=None, *, unknown_min_z=None,
        floor_tolerance=GROUND_FLOOR_TOLERANCE, obstacle_min_height=.22,
        obstacle_max_height=.55, body_position=None, body_yaw=0.,
        self_radius=0., self_offset=0., self_z_down=.45, self_z_up=.20):
    """Project physical XYZ and typed heights into consistent planar evidence.

    Type 0 is invalid, 2 is a measured no-return, 1 has unknown height, and
    3..252 carry height above the observed local ground. Invalid and no-return
    endpoints never create occupied cells. Known heights
    use the same ground-relative body-clearance interval as Local, including
    deep drops; low terrain and overhead returns keep their measured heights.
    Unknown heights use the physical body-slice band supplied by the platform.
    Clearing retains its independent window and always stops at a measured hit.
    """
    points = np.asarray(endpoints, dtype=np.float64)
    kinds = np.asarray(return_types)
    origin = np.asarray(sensor_position, dtype=np.float64)
    rotation = np.asarray(sensor_rotation, dtype=np.float64)
    if unknown_min_z is None:
        unknown_min_z = min_z
    if (points.ndim != 2 or points.shape[1] != 3 or kinds.shape != (len(points),)
            or origin.shape != (3,) or rotation.shape != (3, 3)
            or not np.isfinite(origin).all() or not np.isfinite(rotation).all()
            or ray_count <= 0 or resolution <= 0 or min_z >= max_z
            or unknown_min_z >= max_z or mapping_range <= 0 or no_return_range <= 0
            or not 0. < floor_tolerance < obstacle_min_height < obstacle_max_height
            or not all(math.isfinite(x) for x in (min_z, max_z, unknown_min_z,
                mapping_range, no_return_range, resolution, floor_tolerance,
                obstacle_min_height, obstacle_max_height))):
        raise ValueError("invalid identified 3-D ray geometry")
    lengths = np.linalg.norm(points, axis=1)
    valid = np.isfinite(points).all(axis=1) & (lengths >= .1) & (kinds >= 1)
    points = points[valid]; kinds = kinds[valid]; lengths = lengths[valid]
    world = points @ rotation.T + origin
    directions = (points / lengths[:, None]) @ rotation.T
    hits, misses = kinds != 2, kinds == 2
    heights = decode_ground_height(kinds)
    measured = hits.copy()
    if body_position is not None and self_radius > 0.:
        body = np.asarray(body_position, dtype=np.float64)
        if body.shape != (3,) or not np.isfinite(body).all() or not math.isfinite(body_yaw):
            raise ValueError("invalid capture-time body pose")
        heading = np.array([math.cos(body_yaw), math.sin(body_yaw)])
        xy = world[:, :2] - body[:2]
        self_hit = ((np.minimum(np.linalg.norm(xy-self_offset*heading, axis=1),
                               np.linalg.norm(xy+self_offset*heading, axis=1)) <= self_radius)
                    & (world[:, 2] >= body[2]-self_z_down)
                    & (world[:, 2] <= body[2]+self_z_up))
        measured &= ~self_hit
    known = measured & np.isfinite(heights)
    support = known & (np.abs(heights) <= floor_tolerance)
    terrain = known & ~support
    hazard = known & (((heights >= obstacle_min_height) & (heights <= obstacle_max_height))
                      | (heights <= -obstacle_min_height))
    unknown_body = (measured & ~known & (world[:, 2] >= unknown_min_z)
                    & (world[:, 2] <= max_z))
    obstacle = hazard | unknown_body
    nearest, clearing = _project_identified_rays(
        lengths, hits, misses, origin, directions, min_z, max_z, mapping_range,
        np.minimum(lengths, no_return_range), ray_count, dilation_bins, resolution,
        spatial=True, clear_min_z=clear_min_z, clear_max_z=clear_max_z,
        support=support, endpoint_obstacles=obstacle)
    within_map = np.linalg.norm(world[:, :2]-origin[:2], axis=1) < mapping_range
    return SpatialRayProjection(nearest, clearing, world, heights,
        support & within_map, obstacle & within_map, terrain & within_map)


def classify_spatial_rays(endpoints, return_types, sensor_position, sensor_rotation,
        min_z, max_z, mapping_range, no_return_range, ray_count, dilation_bins,
        resolution, clear_min_z=None, clear_max_z=None, **options):
    """Range-only interface to the shared identified-ray projection."""
    result = project_spatial_rays(endpoints, return_types, sensor_position,
        sensor_rotation, min_z, max_z, mapping_range, no_return_range, ray_count,
        dilation_bins, resolution, clear_min_z, clear_max_z, **options)
    return result.ranges, result.clearing



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
                         no_return_range: Optional[float] = None,
                         clearing_ranges: Optional[Sequence[float]] = None,
                         ground_cells=(), obstacle_cells=()):
        """Fuse one planar scan.

        The batched implementation below is cell-for-cell equivalent to
        ``_integrate_ranges_reference``; equivalence is locked by
        test_grid_vectorized.py.
        """
        return self._integrate_ranges_vectorized(
            position, ranges, max_range, min_range, no_return_range, clearing_ranges, ground_cells, obstacle_cells)

    def _integrate_ranges_reference(self, position: Point2, ranges: Sequence[float],
                                    max_range: float, min_range: float = 0.35,
                                    no_return_range: Optional[float] = None,
                                    clearing_ranges: Optional[Sequence[float]] = None,
                         ground_cells=(), obstacle_cells=()):
        origin = self.world_to_cell(*position)
        if not self.in_bounds(origin):
            return
        free_cells: Set[Cell] = {c for c in ground_cells if self.in_bounds(c)}
        occupied_cells: Set[Cell] = {c for c in obstacle_cells if self.in_bounds(c)}
        ray_count = len(ranges)
        if clearing_ranges is not None:
            clearing_ranges=np.asarray(clearing_ranges,dtype=np.float64)
            if clearing_ranges.shape!=(ray_count,):
                raise ValueError("clearing_ranges must match ranges")
            clearing_ranges=np.where(np.isfinite(clearing_ranges),
                                      np.clip(clearing_ranges,0.,max_range),0.)
        for index, measured_range in enumerate(ranges):
            # NaN is an observed but deliberately ignored 3-D return.  It is
            # neither an obstacle hit in the 2-D projection nor free-space
            # evidence.  Infinity alone represents a genuine no-return ray.
            if math.isnan(measured_range):
                continue
            hit = math.isfinite(measured_range) and measured_range < max_range
            miss_range = (max_range if no_return_range is None
                          else min(max_range, no_return_range))
            clear_limit = (None if clearing_ranges is None else float(clearing_ranges[index]))
            ray_range = min(measured_range, max_range) if hit else (
                miss_range if clear_limit is None else min(max_range, max(0., clear_limit)))
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
            if clear_limit is not None:
                free_line = [cell for cell in free_line if clear_limit>0.
                             and math.dist(self.cell_to_world(cell), position) <= clear_limit]
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
                                     no_return_range: Optional[float] = None,
                                     clearing_ranges: Optional[Sequence[float]] = None,
                         ground_cells=(), obstacle_cells=()):
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
            return self._integrate_ranges_reference(
                position, ranges, max_range, min_range, no_return_range, clearing_ranges, ground_cells, obstacle_cells)
        ranges_arr = np.asarray(ranges, dtype=np.float64)
        is_nan = np.isnan(ranges_arr)
        # NaN is an observed but deliberately ignored 3-D return: neither an
        # obstacle hit in the 2-D projection nor free-space evidence.  A
        # genuine no-return ray is an infinity (or any finite range at/over
        # max_range) and frees up to miss_range.
        hit = np.isfinite(ranges_arr) & (ranges_arr < max_range)
        miss = ~is_nan & ~hit
        miss_range = (max_range if no_return_range is None
                      else min(max_range, no_return_range))
        clear_limits = None
        if clearing_ranges is not None:
            clear_limits = np.asarray(clearing_ranges, dtype=np.float64)
            if clear_limits.shape != ranges_arr.shape:
                raise ValueError("clearing_ranges must match ranges")
            clear_limits = np.where(np.isfinite(clear_limits),
                                    np.clip(clear_limits, 0., max_range), 0.)
        ray_range = np.where(hit, np.minimum(ranges_arr, max_range),
                             miss_range if clear_limits is None else clear_limits)
        valid = (hit | miss) & (ray_range >= min_range)
        if not valid.any():
            return self._integrate_ranges_reference(
                position, ranges, max_range, min_range, no_return_range, clearing_ranges, ground_cells, obstacle_cells)
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
        if clear_limits is not None:
            wx = self.origin_x + (px + .5) * self.resolution
            wy = self.origin_y + (py + .5) * self.resolution
            free_mask &= (clear_limits[idx][None,:]>0.) & (
                np.hypot(wx-position[0],wy-position[1])<=clear_limits[idx][None,:])
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
        if ground_cells:
            extra = [y*self.width+x for x,y in ground_cells if self.in_bounds((x,y))]
            free_flat = np.union1d(free_flat, np.asarray(extra, dtype=np.int64))
        if obstacle_cells:
            extra = [y*self.width+x for x,y in obstacle_cells if self.in_bounds((x,y))]
            occ_flat = np.union1d(occ_flat, np.asarray(extra, dtype=np.int64))
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
