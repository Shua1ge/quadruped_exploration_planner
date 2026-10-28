"""Conservative 2D topology extraction and dense-grid oracle evaluation."""

import heapq
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

from .grid import ExplorationGrid, FREE, OCCUPIED, UNKNOWN
from .path_planning import astar_known, path_length_cells

Cell = Tuple[int, int]


@dataclass(frozen=True)
class TopologyNode:
    node_id: int
    cell: Cell
    degree: int
    clearance: float


@dataclass(frozen=True)
class TopologyEdge:
    edge_id: int
    source_id: int
    target_id: int
    cells: Tuple[Cell, ...]
    length: float
    minimum_clearance: float


@dataclass
class TopologyGraph:
    nodes: Dict[int, TopologyNode]
    edges: Dict[int, TopologyEdge]
    skeleton_cells: Set[Cell]


NEIGHBOURS_8 = tuple(
    (dx, dy) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy)


def thin_free_space(free_mask: np.ndarray) -> np.ndarray:
    """Zhang-Suen thinning; connectivity is retained while interiors shrink."""
    image = np.asarray(free_mask, dtype=np.uint8).copy()
    if image.ndim != 2:
        raise ValueError("free_mask must be two-dimensional")
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0
    changed = True
    while changed:
        changed = False
        for first_step in (True, False):
            p2, p3, p4 = image[:-2, 1:-1], image[:-2, 2:], image[1:-1, 2:]
            p5, p6, p7 = image[2:, 2:], image[2:, 1:-1], image[2:, :-2]
            p8, p9 = image[1:-1, :-2], image[:-2, :-2]
            ring = (p2, p3, p4, p5, p6, p7, p8, p9)
            count = sum(ring)
            transitions = sum(
                ((ring[index] == 0) & (ring[(index + 1) % 8] == 1)).astype(np.uint8)
                for index in range(8))
            if first_step:
                keep_a, keep_b = p2 * p4 * p6 == 0, p4 * p6 * p8 == 0
            else:
                keep_a, keep_b = p2 * p4 * p8 == 0, p2 * p6 * p8 == 0
            remove = ((image[1:-1, 1:-1] != 0) & (count >= 2) &
                      (count <= 6) & (transitions == 1) & keep_a & keep_b)
            if np.any(remove):
                changed = True
                image[1:-1, 1:-1][remove] = 0
    return image.astype(bool)


def clearance_field(free_mask: np.ndarray, resolution: float) -> np.ndarray:
    """Exact Euclidean cell-centre distance to non-free (including unknown)."""
    free = np.asarray(free_mask, dtype=bool)
    if np.all(free):
        return np.full(free.shape, max(free.shape) * resolution, dtype=np.float64)
    return distance_transform_edt(free, sampling=resolution)


def stable_node_id(cell: Cell, origin: Tuple[float, float], resolution: float) -> int:
    """Pack globally quantized signed coordinates into a stable uint64 id."""
    world_x = origin[0] + (cell[0] + 0.5) * resolution
    world_y = origin[1] + (cell[1] + 0.5) * resolution
    gx = int(math.floor(world_x / resolution + 1e-9))
    gy = int(math.floor(world_y / resolution + 1e-9))
    return ((gx & 0xFFFFFFFF) << 32) | (gy & 0xFFFFFFFF)


def stable_edge_id(source_id: int, target_id: int) -> int:
    """FNV-1a id for an undirected pair of stable node ids."""
    first, second = sorted((source_id, target_id))
    value = 1469598103934665603
    for item in (first, second):
        for shift in range(0, 64, 8):
            value ^= (item >> shift) & 0xFF
            value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return value


def skeleton_neighbours(cell: Cell, skeleton: Set[Cell]) -> List[Cell]:
    return sorted((cell[0] + dx, cell[1] + dy) for dx, dy in NEIGHBOURS_8
                  if (cell[0] + dx, cell[1] + dy) in skeleton)


def extract_topology(occupancy: np.ndarray, resolution: float,
                     origin: Tuple[float, float],
                     shared_clearance: Optional[np.ndarray] = None,
                     timings: Optional[Dict[str, float]] = None
                     ) -> TopologyGraph:
    """Extract endpoints/junctions and compress degree-two skeleton chains."""
    values = np.asarray(occupancy, dtype=np.int16)
    free = values == FREE
    thinning_started = time.perf_counter()
    skeleton_mask = thin_free_space(free)
    thinning_ms = (time.perf_counter() - thinning_started) * 1000.0
    compression_started = time.perf_counter()
    skeleton = {(int(x), int(y)) for y, x in np.argwhere(skeleton_mask)}
    clearance = (clearance_field(free, resolution)
                 if shared_clearance is None
                 else np.asarray(shared_clearance, dtype=np.float64))
    if clearance.shape != free.shape:
        raise ValueError("shared_clearance shape must match occupancy")
    node_cells = {cell for cell in skeleton
                  if len(skeleton_neighbours(cell, skeleton)) != 2}
    if skeleton and not node_cells:
        node_cells.add(min(skeleton))

    nodes = {}
    cell_to_id = {}
    for cell in sorted(node_cells):
        node_id = stable_node_id(cell, origin, resolution)
        cell_to_id[cell] = node_id
        nodes[node_id] = TopologyNode(
            node_id, cell, len(skeleton_neighbours(cell, skeleton)),
            float(clearance[cell[1], cell[0]]))

    edges = {}
    visited_links = set()
    for start in sorted(node_cells):
        for neighbour in skeleton_neighbours(start, skeleton):
            initial_link = tuple(sorted((start, neighbour)))
            if initial_link in visited_links:
                continue
            path = [start, neighbour]
            visited_links.add(initial_link)
            previous, current = start, neighbour
            while current not in node_cells:
                onward = [cell for cell in skeleton_neighbours(current, skeleton)
                          if cell != previous]
                if not onward:
                    break
                nxt = onward[0]
                visited_links.add(tuple(sorted((current, nxt))))
                path.append(nxt)
                previous, current = current, nxt
            if current not in node_cells or current == start:
                continue
            source_id, target_id = cell_to_id[start], cell_to_id[current]
            edge_id = stable_edge_id(source_id, target_id)
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) * resolution
                         for a, b in zip(path[:-1], path[1:]))
            minimum_clearance = min(float(clearance[y, x]) for x, y in path)
            candidate = TopologyEdge(
                edge_id, source_id, target_id, tuple(path), length,
                minimum_clearance)
            previous_edge = edges.get(edge_id)
            if previous_edge is None or candidate.length < previous_edge.length:
                edges[edge_id] = candidate
    if timings is not None:
        timings.update(thinning_ms=thinning_ms,
                       graph_compression_ms=(time.perf_counter() - compression_started) * 1000.0)
    return TopologyGraph(nodes, edges, skeleton)


def graph_distance(graph: TopologyGraph, source_id: int,
                   target_id: int) -> Optional[float]:
    adjacency: Dict[int, List[Tuple[int, float]]] = {}
    for edge in graph.edges.values():
        adjacency.setdefault(edge.source_id, []).append((edge.target_id, edge.length))
        adjacency.setdefault(edge.target_id, []).append((edge.source_id, edge.length))
    queue = [(0.0, source_id)]
    costs = {source_id: 0.0}
    while queue:
        cost, current = heapq.heappop(queue)
        if current == target_id:
            return cost
        if cost != costs[current]:
            continue
        for nxt, length in adjacency.get(current, []):
            candidate = cost + length
            if candidate < costs.get(nxt, float("inf")):
                costs[nxt] = candidate
                heapq.heappush(queue, (candidate, nxt))
    return None


def oracle_metrics(occupancy: np.ndarray, resolution: float,
                   graph: TopologyGraph, max_queries: int = 8) -> Dict[str, float]:
    """Compare sparse graph queries with the existing known-free dense A*."""
    values = np.asarray(occupancy, dtype=np.int16)
    grid = ExplorationGrid(
        np.nextafter(values.shape[1] * resolution, 0.0), np.nextafter(values.shape[0] * resolution, 0.0),
        resolution, 0.0, 0.0)
    grid.data[:, :] = np.where(values == FREE, FREE,
                               np.where(values == OCCUPIED, OCCUPIED, UNKNOWN))
    node_ids = sorted(graph.nodes)
    pairs = []
    for first_index, source_id in enumerate(node_ids):
        for target_id in reversed(node_ids[first_index + 1:]):
            pairs.append((source_id, target_id))
            if len(pairs) >= max_queries:
                break
        if len(pairs) >= max_queries:
            break

    connected_dense = connected_graph = matched = 0
    errors = []
    for source_id, target_id in pairs:
        source = graph.nodes[source_id].cell
        target = graph.nodes[target_id].cell
        dense_path = astar_known(grid, source, target, set())
        dense_cost = (path_length_cells(dense_path, resolution)
                      if dense_path else None)
        sparse_cost = graph_distance(graph, source_id, target_id)
        connected_dense += dense_cost is not None
        connected_graph += sparse_cost is not None
        if dense_cost is not None and sparse_cost is not None:
            matched += 1
            errors.append(max(0.0, sparse_cost - dense_cost) /
                          max(dense_cost, resolution))
    return {
        "queries": float(len(pairs)),
        "dense_connected": float(connected_dense),
        "graph_connected": float(connected_graph),
        "connectivity_recall": (matched / connected_dense
                                if connected_dense else 1.0),
        "mean_cost_error": float(np.mean(errors)) if errors else 0.0,
        "max_cost_error": float(max(errors)) if errors else 0.0,
    }


def routing_oracle_metrics(occupancy, resolution, graph, max_queries=8,
                          origin=(0., 0.), attachment_radius=3.0):
    """Probe actual query/attachment/realization from independent free cells.

    No dense rescue is credited to the graph. Misses and unsafe polylines are
    counted. Empty cohorts have no recall rather than a fabricated 100%.
    Graph cells use the world lattice; origin applies to the dense array only.
    """
    from .grid import bresenham
    from .path_planning import adjacent_grid_path_is_valid, segment_known_free
    from .sparse_routing import SparseRouteGraph, SparseNode, SparseEdge
    values = np.asarray(occupancy, dtype=np.int8)
    grid = ExplorationGrid(np.nextafter(values.shape[1]*resolution, 0.0), np.nextafter(values.shape[0]*resolution, 0.0),
                           resolution, *origin)
    grid.data[:] = values
    router = SparseRouteGraph()
    def world(cell):
        return ((cell[0]+.5)*resolution, (cell[1]+.5)*resolution)
    for n in graph.nodes.values():
        router.upsert_node(SparseNode(n.node_id, world(n.cell), n.clearance))
    for e in graph.edges.values():
        router.upsert_edge(SparseEdge(e.edge_id, e.source_id, e.target_id,
            e.length, e.minimum_clearance, tuple(world(c) for c in e.cells)))
    free = np.argwhere(values == FREE)
    pairs = []
    if len(free) >= 2 and max_queries > 0:
        # The extent pair exposes coverage loss; seeded independent samples
        # exercise interiors, rather than selecting existing graph nodes.
        pairs.append((free[0], free[-1]))
        rng = np.random.default_rng(0)
        for _ in range(max_queries-1):
            indices = rng.choice(len(free), 2, replace=False)
            pairs.append((free[indices[0]], free[indices[1]]))
    dense_connected = estimate_hits = validated = false_positives = matched = 0
    graph_hits = graph_validated = direct_validated = 0
    errors = []
    def connector(a,b):
        return (segment_known_free(grid, grid.world_to_cell(*a), grid.world_to_cell(*b), set())
                and segment_known_free(grid, grid.world_to_cell(*b), grid.world_to_cell(*a), set()))
    for a,b in pairs:
        source, target = (int(a[1]), int(a[0])), (int(b[1]), int(b[0]))
        dense = astar_known(grid, source, target, set())
        dense_connected += bool(dense)
        estimate = router.batch_estimates(grid.cell_to_world(source),
            [grid.cell_to_world(target)], attachment_radius, 8, connector)[0]
        if estimate is None:
            continue
        estimate_hits += 1
        graph_hits += estimate.route_source == "graph"
        cells = []
        vertices = [grid.world_to_cell(*p) for p in estimate.polyline]
        for p,q in zip(vertices, vertices[1:]):
            cells.extend(bresenham(p,q)[:-1])
        if vertices:
            cells.append(vertices[-1])
        valid = (bool(cells) and cells[0] == source and cells[-1] == target
                 and adjacent_grid_path_is_valid(grid,cells,set(),set()))
        validated += valid
        graph_validated += valid and estimate.route_source == "graph"
        direct_validated += valid and estimate.route_source == "direct_connector"
        false_positives += not valid or not dense
        if valid and dense:
            matched += 1
            cost = path_length_cells(dense, resolution)
            errors.append(abs(estimate.distance-cost)/max(cost,resolution))
    return dict(queries=len(pairs), dense_connected=dense_connected,
        graph_connected=graph_validated, graph_estimate_hits=graph_hits,
        graph_validated_routes=graph_validated, query_estimate_hits=estimate_hits,
        query_validated_routes=validated, direct_connector_routes=direct_validated,
        connectivity_recall=matched/dense_connected if dense_connected else None,
        false_positive_rate=false_positives/estimate_hits if estimate_hits else None,
        mean_cost_error=float(np.mean(errors)) if errors else None,
        max_cost_error=max(errors) if errors else None,
        oracle_endpoint_sampling="free_cells", oracle_representation="published_query_graph")
