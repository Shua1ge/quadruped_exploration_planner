"""Global terrain tiles, aggregation onto graph elements, and route costs.

Local measures ground facts per 2 m tile; Global stores those summaries and
aggregates them onto nodes, edges and portals.  Terrain never hard-blocks a
route -- Explorer must not decide by itself whether the body can pass.  It only
scales the route cost in [0.8, 2.0] so measured, experienced corridors are
preferred over rough, stepped, overhead or stale ones.
"""

from dataclasses import dataclass
import math
from typing import Dict, Mapping, Sequence, Tuple

UNKNOWN, FLAT, ROUGH, STEPS, OVERHEAD, STALE = range(6)
STATE_NAMES = ("unknown", "flat", "rough", "steps", "overhead", "stale")

Point2 = Tuple[float, float]


@dataclass(frozen=True)
class TerrainTile:
    tx: int
    ty: int
    size: float
    ground_min: float
    ground_median: float
    ground_max: float
    slope: float
    roughness: float
    support: int
    rise: int
    drop: int
    overhead: int
    columns: int
    last_scan: int
    content_revision: int
    occupancy_revision: int = 0


class TerrainTileStore:
    """World-aligned terrain summaries with age-based invalidation."""

    def __init__(self, tile_size: float = 2.0, stale_scans: int = 60,
                 drop_scans: int = 300, rough_slope: float = 0.20,
                 rough_roughness: float = 0.06, max_tiles: int = 8192):
        if not math.isfinite(tile_size) or tile_size <= 0.0:
            raise ValueError("terrain tile_size must be finite and positive")
        if stale_scans <= 0 or drop_scans <= stale_scans:
            raise ValueError("terrain scans must satisfy 0 < stale < drop")
        self.tile_size = float(tile_size)
        self.stale_scans = int(stale_scans)
        self.drop_scans = int(drop_scans)
        self.rough_slope = float(rough_slope)
        self.rough_roughness = float(rough_roughness)
        self.max_tiles = int(max_tiles)
        self.scan = 0
        self.revision = 0
        self.tiles: Dict[Tuple[int, int], TerrainTile] = {}
        self.occupancy: Dict[Tuple[int, int], int] = {}
        self.changed_tiles = 0
        self.dropped_tiles = 0

    def clear(self):
        self.scan = 0
        self.revision = 0
        self.tiles.clear()
        self.occupancy.clear()
        self.changed_tiles = 0
        self.dropped_tiles = 0

    def key(self, x: float, y: float) -> Tuple[int, int]:
        return (math.floor(x / self.tile_size), math.floor(y / self.tile_size))

    def observe_scan(self, scan: int):
        self.scan = max(self.scan, int(scan))

    def update(self, scan: int, summaries: Sequence[object]) -> int:
        """Ingest one patch's tile summaries; return changed tile count."""
        self.observe_scan(scan)
        self.changed_tiles = 0
        for item in summaries:
            key = (int(item.tile_x), int(item.tile_y))
            tile = TerrainTile(
                key[0], key[1], float(item.tile_size),
                float(item.ground_min), float(item.ground_median),
                float(item.ground_max), float(item.slope),
                float(item.roughness), int(item.support_cells),
                int(item.rise_cells), int(item.drop_cells),
                int(item.overhead_cells), int(item.measured_columns),
                int(item.last_scan), int(item.content_revision),
                self.occupancy.get(key, 0))
            previous = self.tiles.get(key)
            if previous is None or previous.content_revision != tile.content_revision:
                self.changed_tiles += 1
            self.tiles[key] = tile
        drop_before = self.scan - self.drop_scans
        for key, tile in list(self.tiles.items()):
            if tile.last_scan < drop_before:
                del self.tiles[key]
                self.occupancy.pop(key, None)
                self.dropped_tiles += 1
        while len(self.tiles) > self.max_tiles:
            oldest = min(self.tiles, key=lambda key: self.tiles[key].last_scan)
            del self.tiles[oldest]
            self.occupancy.pop(oldest, None)
            self.dropped_tiles += 1
        if self.changed_tiles:
            self.revision += 1
        return self.changed_tiles

    def observe_occupancy(self, key: Tuple[int, int], revision: int, scan: int):
        """Occupancy changed: keep terrain only if it was measured this scan."""
        previous = self.occupancy.get(key)
        self.occupancy[key] = int(revision)
        if previous is None or previous == int(revision):
            return
        tile = self.tiles.get(key)
        if tile is not None and tile.last_scan < int(scan):
            del self.tiles[key]
            self.dropped_tiles += 1

    def aggregate(self, points: Sequence[Point2]) -> dict:
        """Aggregate every known tile touched by the given sample points."""
        keys = {self.key(x, y) for x, y in points
                if math.isfinite(x) and math.isfinite(y)}
        tiles = [self.tiles[key] for key in keys
                 if key in self.tiles and self.tiles[key].columns > 0]
        result = {
            "ground_min": math.nan, "ground_median": math.nan,
            "ground_max": math.nan, "slope": math.nan,
            "roughness": math.nan, "rise_cells": 0, "drop_cells": 0,
            "overhead_cells": 0, "measured_columns": 0, "last_scan": 0,
            "state": UNKNOWN, "tiles": tuple(sorted(keys)),
        }
        if not tiles:
            return result
        result["ground_min"] = min(tile.ground_min for tile in tiles)
        result["ground_max"] = max(tile.ground_max for tile in tiles)
        result["ground_median"] = _weighted_median(tiles)
        slopes = [tile.slope for tile in tiles if math.isfinite(tile.slope)]
        roughs = [tile.roughness for tile in tiles if math.isfinite(tile.roughness)]
        result["slope"] = max(slopes) if slopes else math.nan
        result["roughness"] = max(roughs) if roughs else math.nan
        result["rise_cells"] = sum(tile.rise for tile in tiles)
        result["drop_cells"] = sum(tile.drop for tile in tiles)
        result["overhead_cells"] = sum(tile.overhead for tile in tiles)
        result["measured_columns"] = sum(tile.columns for tile in tiles)
        result["last_scan"] = min(tile.last_scan for tile in tiles)
        result["state"] = self.state_for(result)
        return result

    def state_for(self, aggregate: Mapping[str, object]) -> int:
        if aggregate.get("measured_columns", 0) <= 0:
            return UNKNOWN
        if self.scan - int(aggregate.get("last_scan", 0)) > self.stale_scans:
            return STALE
        if (int(aggregate.get("rise_cells", 0))
                + int(aggregate.get("drop_cells", 0))) > 0:
            return STEPS
        if int(aggregate.get("overhead_cells", 0)) > 0:
            return OVERHEAD
        slope = float(aggregate.get("slope", math.nan))
        roughness = float(aggregate.get("roughness", math.nan))
        if ((math.isfinite(slope) and slope > self.rough_slope)
                or (math.isfinite(roughness)
                    and roughness > self.rough_roughness)):
            return ROUGH
        return FLAT

    def traversability(self, aggregate: Mapping[str, object],
                       traversals: int = 0,
                       experience_score: float = 0.0) -> float:
        """Route-cost multiplier in [0.8, 2.0]; never infinite, never a wall."""
        penalty = {ROUGH: 0.35, STEPS: 0.55, OVERHEAD: 0.15,
                   STALE: 0.25, UNKNOWN: 0.30}.get(
                       int(aggregate.get("state", UNKNOWN)), 0.0)
        slope = float(aggregate.get("slope", math.nan))
        roughness = float(aggregate.get("roughness", math.nan))
        if math.isfinite(slope):
            penalty += min(0.40, 1.5 * max(0.0, slope - self.rough_slope))
        if math.isfinite(roughness):
            penalty += min(0.30, 2.0 * max(0.0,
                                            roughness - self.rough_roughness))
        discount = min(0.20, 0.10 * max(0.0, float(experience_score)))
        if traversals >= 3:
            discount += 0.05
        return round(min(2.0, max(0.8, 1.0 + penalty - discount)), 3)


def _weighted_median(tiles: Sequence[TerrainTile]) -> float:
    ordered = sorted(tiles, key=lambda tile: tile.ground_median)
    total = sum(max(1, tile.columns) for tile in ordered)
    cumulative = 0
    for tile in ordered:
        cumulative += max(1, tile.columns)
        if cumulative * 2 >= total:
            return float(tile.ground_median)
    return float(ordered[-1].ground_median)


def decayed_experience(count: float, stamp: float, now: float,
                       half_life: float) -> float:
    if count <= 0.0 or not math.isfinite(stamp) or half_life <= 0.0:
        return 0.0
    age = max(0.0, float(now) - float(stamp))
    return float(count) * 0.5 ** (age / float(half_life))


def experience_summary(records: Mapping[Tuple[int, int], Sequence[float]],
                       keys: Sequence[Tuple[int, int]], now: float,
                       half_life: float) -> Tuple[int, float]:
    """Return (undecayed traversal count, decayed preference in [0, 1])."""
    traversals = 0
    best = 0.0
    for key in keys:
        record = records.get(key)
        if not record:
            continue
        count = float(record[0])
        traversals += int(count)
        best = max(best, decayed_experience(
            count, float(record[1]), now, half_life))
    return traversals, min(1.0, best / 3.0)
