"""Planning-preserving safe-region graph extracted from a local 2D patch.

This module is deliberately independent from frontier selection and route
execution.  It provides a shadow representation that can be compared with the
existing skeleton graph before either planner consumes it.
"""

import heapq
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set, Tuple

import numpy as np

from .grid import ExplorationGrid, FREE, OCCUPIED, UNKNOWN
from .path_planning import astar_known, path_length_cells
from .topology import clearance_field

Cell = Tuple[int, int]
WorldCell = Tuple[int, int]
TileKey = Tuple[int, int]

NEIGHBOURS_8 = tuple(
    (dx, dy) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy)


@dataclass(frozen=True)
class SafeRegion:
    region_id: int
    tile: TileKey
    anchor: Cell
    cells: frozenset
    area: float
    minimum_clearance: float


@dataclass(frozen=True)
class Portal:
    portal_id: int
    source_id: int
    target_id: int
    cell: Cell
    width: float
    minimum_clearance: float
    bottleneck: bool


@dataclass
class SafeRegionGraph:
    regions: Dict[int, SafeRegion]
    portals: Dict[int, Portal]
    cell_to_region: Dict[Cell, int]
    free_cell_count: int


def _fnv1a(values: Iterable[int]) -> int:
    value = 1469598103934665603
    for item in values:
        encoded = int(item) & 0xFFFFFFFFFFFFFFFF
        for shift in range(0, 64, 8):
            value ^= (encoded >> shift) & 0xFF
            value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return value


def _world_cell(cell: Cell, origin: Tuple[float, float],
                resolution: float) -> WorldCell:
    return (
        int(math.floor(origin[0] / resolution + cell[0] + 1e-6)),
        int(math.floor(origin[1] / resolution + cell[1] + 1e-6)),
    )


def _tile_key(cell: Cell, origin: Tuple[float, float], resolution: float,
              tile_cells: int) -> TileKey:
    gx, gy = _world_cell(cell, origin, resolution)
    return gx // tile_cells, gy // tile_cells


def _components(cells: Set[Cell]) -> List[Set[Cell]]:
    remaining = set(cells)
    result = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        component = {seed}
        stack = [seed]
        while stack:
            x, y = stack.pop()
            for dx, dy in NEIGHBOURS_8:
                neighbour = (x + dx, y + dy)
                if neighbour in remaining:
                    remaining.remove(neighbour)
                    component.add(neighbour)
                    stack.append(neighbour)
        result.append(component)
    return result


def _region_id(tile: TileKey, component: Set[Cell],
               origin: Tuple[float, float], resolution: float) -> int:
    """Use a world-aligned tile and component anchor for deterministic IDs."""
    anchor_world = min(_world_cell(cell, origin, resolution)
                       for cell in component)
    return _fnv1a((0x525347, tile[0], tile[1],
                   anchor_world[0], anchor_world[1]))


def extract_safe_region_graph(
        occupancy: np.ndarray, resolution: float,
        origin: Tuple[float, float], tile_size: float = 2.0,
        bottleneck_width: float = 1.2,
        shared_clearance: Optional[np.ndarray] = None) -> SafeRegionGraph:
    """Partition known free space into stable world-aligned safe regions.

    World-aligned tiles prevent a sliding local-map origin from renumbering the
    entire graph.  Connected components inside a tile preserve walls and other
    barriers instead of merging cells solely because they are nearby.  Region
    interfaces are explicit portals; narrow interfaces are labelled as likely
    bottlenecks but are not removed or treated as unsafe in shadow mode.
    """
    values = np.asarray(occupancy, dtype=np.int16)
    if values.ndim != 2:
        raise ValueError("occupancy must be two-dimensional")
    if resolution <= 0.0 or tile_size <= 0.0:
        raise ValueError("resolution and tile_size must be positive")

    free = values == FREE
    free_cells = {(int(x), int(y)) for y, x in np.argwhere(free)}
    if not free_cells:
        return SafeRegionGraph({}, {}, {}, 0)
    clearance = (clearance_field(free, resolution)
                 if shared_clearance is None
                 else np.asarray(shared_clearance, dtype=np.float64))
    if clearance.shape != free.shape:
        raise ValueError("shared_clearance shape must match occupancy")
    tile_cells = max(1, int(round(tile_size / resolution)))
    cells_by_tile: Dict[TileKey, Set[Cell]] = {}
    for cell in free_cells:
        cells_by_tile.setdefault(
            _tile_key(cell, origin, resolution, tile_cells), set()).add(cell)

    regions: Dict[int, SafeRegion] = {}
    cell_to_region: Dict[Cell, int] = {}
    for tile in sorted(cells_by_tile):
        for component in _components(cells_by_tile[tile]):
            region_id = _region_id(tile, component, origin, resolution)
            anchor = max(component,
                         key=lambda cell: (clearance[cell[1], cell[0]],
                                           -cell[1], -cell[0]))
            region = SafeRegion(
                region_id=region_id,
                tile=tile,
                anchor=anchor,
                cells=frozenset(component),
                area=len(component) * resolution * resolution,
                minimum_clearance=min(
                    float(clearance[y, x]) for x, y in component),
            )
            regions[region_id] = region
            for cell in component:
                cell_to_region[cell] = region_id

    contacts: Dict[Tuple[int, int], Set[Cell]] = {}
    for (x, y), source_id in cell_to_region.items():
        for dx, dy in ((1, 0), (0, 1), (1, 1), (-1, 1)):
            neighbour = (x + dx, y + dy)
            target_id = cell_to_region.get(neighbour)
            if target_id is None or target_id == source_id:
                continue
            pair = tuple(sorted((source_id, target_id)))
            contacts.setdefault(pair, set()).update(((x, y), neighbour))

    portals = {}
    for pair, interface_cells in sorted(contacts.items()):
        # A contact set can include a diagonal corner.  Count its span rather
        # than its area so broad room interfaces are distinguished from doors.
        xs = [cell[0] for cell in interface_cells]
        ys = [cell[1] for cell in interface_cells]
        width = max(max(xs) - min(xs) + 1,
                    max(ys) - min(ys) + 1) * resolution
        centroid_x = sum(xs) / len(xs)
        centroid_y = sum(ys) / len(ys)
        portal_cell = min(
            interface_cells,
            key=lambda cell: ((cell[0] - centroid_x) ** 2
                              + (cell[1] - centroid_y) ** 2,
                              cell[1], cell[0]))
        portal_id = _fnv1a((0x505254, pair[0], pair[1]))
        portals[portal_id] = Portal(
            portal_id=portal_id,
            source_id=pair[0],
            target_id=pair[1],
            cell=portal_cell,
            width=float(width),
            minimum_clearance=min(
                float(clearance[y, x]) for x, y in interface_cells),
            bottleneck=width <= bottleneck_width,
        )
    return SafeRegionGraph(regions, portals, cell_to_region, len(free_cells))


def _region_distance(graph: SafeRegionGraph, source_id: int,
                     target_id: int, resolution: float) -> Optional[float]:
    adjacency: Dict[int, List[Tuple[int, float]]] = {}
    for portal in graph.portals.values():
        source = graph.regions[portal.source_id].anchor
        target = graph.regions[portal.target_id].anchor
        weight = max(resolution, math.hypot(
            target[0] - source[0], target[1] - source[1]) * resolution)
        adjacency.setdefault(portal.source_id, []).append(
            (portal.target_id, weight))
        adjacency.setdefault(portal.target_id, []).append(
            (portal.source_id, weight))
    queue = [(0.0, source_id)]
    costs = {source_id: 0.0}
    while queue:
        cost, current = heapq.heappop(queue)
        if current == target_id:
            return cost
        if cost != costs[current]:
            continue
        for neighbour, weight in adjacency.get(current, []):
            candidate = cost + weight
            if candidate < costs.get(neighbour, float("inf")):
                costs[neighbour] = candidate
                heapq.heappush(queue, (candidate, neighbour))
    return None


def safe_region_oracle_metrics(
        occupancy: np.ndarray, resolution: float, graph: SafeRegionGraph,
        max_queries: int = 8) -> Dict[str, float]:
    """Compare region-graph reachability with the existing dense A* oracle."""
    values = np.asarray(occupancy, dtype=np.int16)
    grid = ExplorationGrid(
        values.shape[1] * resolution, values.shape[0] * resolution,
        resolution, 0.0, 0.0)
    grid.data[:, :] = np.where(values == FREE, FREE,
                               np.where(values == OCCUPIED, OCCUPIED, UNKNOWN))
    region_ids = sorted(graph.regions)
    pairs = []
    for index, source_id in enumerate(region_ids):
        for target_id in reversed(region_ids[index + 1:]):
            pairs.append((source_id, target_id))
            if len(pairs) >= max_queries:
                break
        if len(pairs) >= max_queries:
            break

    dense_connected = matched = false_positive = 0
    absolute_errors = []
    for source_id, target_id in pairs:
        source = graph.regions[source_id].anchor
        target = graph.regions[target_id].anchor
        dense_path = astar_known(grid, source, target, set())
        dense_cost = (path_length_cells(dense_path, resolution)
                      if dense_path else None)
        sparse_cost = _region_distance(graph, source_id, target_id, resolution)
        dense_connected += dense_cost is not None
        matched += dense_cost is not None and sparse_cost is not None
        false_positive += dense_cost is None and sparse_cost is not None
        if dense_cost is not None and sparse_cost is not None:
            absolute_errors.append(abs(sparse_cost - dense_cost)
                                   / max(dense_cost, resolution))
    return {
        "safe_region_queries": float(len(pairs)),
        "safe_region_dense_connected": float(dense_connected),
        "safe_region_connectivity_recall": (
            matched / dense_connected if dense_connected else 1.0),
        "safe_region_false_positive_rate": (
            false_positive / len(pairs) if pairs else 0.0),
        "safe_region_mean_abs_cost_error": (
            float(np.mean(absolute_errors)) if absolute_errors else 0.0),
        "safe_region_max_abs_cost_error": (
            float(max(absolute_errors)) if absolute_errors else 0.0),
    }


class ShadowStabilityTracker:
    """Measure ID retention without influencing the extracted graph."""

    def __init__(self):
        self.previous_regions: Set[int] = set()
        self.previous_portals: Set[int] = set()

    def update(self, graph: SafeRegionGraph) -> Dict[str, float]:
        region_ids = set(graph.regions)
        portal_ids = set(graph.portals)
        region_retention = (
            len(region_ids & self.previous_regions) / len(self.previous_regions)
            if self.previous_regions else 1.0)
        portal_retention = (
            len(portal_ids & self.previous_portals) / len(self.previous_portals)
            if self.previous_portals else 1.0)
        self.previous_regions = region_ids
        self.previous_portals = portal_ids
        return {
            "safe_region_id_retention": region_retention,
            "portal_id_retention": portal_retention,
        }


class PersistentSafeRegionTracker:
    """Preserve region identity across consecutive overlapping map patches.

    Matching is deliberately local to a world-aligned tile.  A region can only
    inherit an ID from a previous component with actual free-cell overlap, so a
    wall-separated component can never steal the identity of its neighbour.
    """

    def __init__(self, minimum_overlap: float = 0.25):
        self.minimum_overlap = minimum_overlap
        self.previous: Dict[TileKey, Dict[int, Set[WorldCell]]] = {}

    def update(self, graph: SafeRegionGraph, origin: Tuple[float, float],
               resolution: float) -> SafeRegionGraph:
        by_tile: Dict[TileKey, List[Tuple[SafeRegion, Set[WorldCell]]]] = {}
        for region in graph.regions.values():
            world_cells = {
                _world_cell(cell, origin, resolution) for cell in region.cells}
            by_tile.setdefault(region.tile, []).append((region, world_cells))

        assignments: Dict[int, int] = {}
        current: Dict[TileKey, Dict[int, Set[WorldCell]]] = {}
        for tile, observations in by_tile.items():
            old = self.previous.get(tile, {})
            candidates = []
            for region, world_cells in observations:
                for previous_id, previous_cells in old.items():
                    overlap = len(world_cells & previous_cells)
                    score = overlap / max(1, min(
                        len(world_cells), len(previous_cells)))
                    if score >= self.minimum_overlap:
                        candidates.append(
                            (score, overlap, region.region_id, previous_id))
            used_new: Set[int] = set()
            used_old: Set[int] = set()
            for _, _, temporary_id, previous_id in sorted(
                    candidates, reverse=True):
                if temporary_id in used_new or previous_id in used_old:
                    continue
                assignments[temporary_id] = previous_id
                used_new.add(temporary_id)
                used_old.add(previous_id)

            tile_state = {}
            for region, world_cells in observations:
                persistent_id = assignments.get(region.region_id,
                                                region.region_id)
                # A deterministic new ID can collide with an inherited one
                # after a component split.  Salt only that genuinely new part.
                salt = 1
                while persistent_id in tile_state:
                    persistent_id = _fnv1a(
                        (region.region_id, tile[0], tile[1], salt))
                    salt += 1
                assignments[region.region_id] = persistent_id
                tile_state[persistent_id] = world_cells
            current[tile] = tile_state

        regions = {}
        cell_to_region = {}
        for region in graph.regions.values():
            persistent_id = assignments[region.region_id]
            replacement = SafeRegion(
                persistent_id, region.tile, region.anchor, region.cells,
                region.area, region.minimum_clearance)
            regions[persistent_id] = replacement
            for cell in region.cells:
                cell_to_region[cell] = persistent_id

        portals = {}
        for portal in graph.portals.values():
            source_id = assignments[portal.source_id]
            target_id = assignments[portal.target_id]
            portal_id = _fnv1a(
                (0x505254, min(source_id, target_id),
                 max(source_id, target_id)))
            portals[portal_id] = Portal(
                portal_id, source_id, target_id, portal.cell, portal.width,
                portal.minimum_clearance, portal.bottleneck)
        self.previous = current
        return SafeRegionGraph(
            regions, portals, cell_to_region, graph.free_cell_count)
