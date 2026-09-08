"""Conservative 2D topology extraction and dense-grid oracle evaluation."""

import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

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
            remove = []
            for y in range(1, image.shape[0] - 1):
                for x in range(1, image.shape[1] - 1):
                    if image[y, x] == 0:
                        continue
                    p2, p3, p4 = image[y - 1, x], image[y - 1, x + 1], image[y, x + 1]
                    p5, p6, p7 = (
                        image[y + 1, x + 1], image[y + 1, x],
                        image[y + 1, x - 1])
                    p8, p9 = image[y, x - 1], image[y - 1, x - 1]
                    ring = (p2, p3, p4, p5, p6, p7, p8, p9)
                    count = int(sum(ring))
                    transitions = sum(
                        ring[index] == 0 and ring[(index + 1) % 8] == 1
                        for index in range(8))
                    if not 2 <= count <= 6 or transitions != 1:
                        continue
                    if first_step:
                        keep_a = p2 * p4 * p6 == 0
                        keep_b = p4 * p6 * p8 == 0
                    else:
                        keep_a = p2 * p4 * p8 == 0
                        keep_b = p2 * p6 * p8 == 0
                    if keep_a and keep_b:
                        remove.append((x, y))
            if remove:
                changed = True
                for x, y in remove:
                    image[y, x] = 0
    return image.astype(bool)


def clearance_field(free_mask: np.ndarray, resolution: float) -> np.ndarray:
    """Conservative 8-neighbour distance to non-free space."""
    free = np.asarray(free_mask, dtype=bool)
    distance = np.full(free.shape, np.inf, dtype=np.float64)
    queue = []
    for y, x in np.argwhere(~free):
        distance[y, x] = 0.0
        heapq.heappush(queue, (0.0, int(x), int(y)))
    if not queue:
        distance.fill(max(free.shape) * resolution)
        return distance
    while queue:
        current, x, y = heapq.heappop(queue)
        if current != distance[y, x]:
            continue
        for dx, dy in NEIGHBOURS_8:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < free.shape[1] and 0 <= ny < free.shape[0]):
                continue
            candidate = current + math.hypot(dx, dy) * resolution
            if candidate < distance[ny, nx]:
                distance[ny, nx] = candidate
                heapq.heappush(queue, (candidate, nx, ny))
    return distance


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
                     origin: Tuple[float, float]) -> TopologyGraph:
    """Extract endpoints/junctions and compress degree-two skeleton chains."""
    values = np.asarray(occupancy, dtype=np.int16)
    free = values == FREE
    skeleton_mask = thin_free_space(free)
    skeleton = {(int(x), int(y)) for y, x in np.argwhere(skeleton_mask)}
    clearance = clearance_field(free, resolution)
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
        values.shape[1] * resolution, values.shape[0] * resolution,
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
