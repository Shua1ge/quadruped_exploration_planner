"""Frontier extraction, viewpoint generation, and persistent region policy."""

import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

import numpy as np

from .grid import Cell, ExplorationGrid, UNKNOWN
from .path_planning import astar_known, path_length_cells, segment_known_free


def region_information_efficiency(unknown_gain: int, frontier_size: int,
                                  path_length: float, turn_cost: float) -> float:
    """Return useful observable frontier information per unit travel effort."""
    information = max(0.0, float(unknown_gain)) + 0.25 * math.sqrt(
        max(0.0, float(frontier_size)))
    travel_effort = 1.0 + max(0.0, float(path_length)) + 0.25 * max(
        0.0, float(turn_cost))
    return information / travel_effort


def select_rolling_region(region_scores: Dict[int, float],
                          active_region_id: Optional[int],
                          switch_ratio: float) -> Optional[int]:
    """Choose only the next region, retaining the active one with hysteresis.

    The current region remains selected unless another reachable region is
    better by ``switch_ratio``.  This avoids letting a predicted full tour
    control the immediate exploration action.
    """
    if not region_scores:
        return None
    best_region = max(region_scores, key=lambda region_id: (
        region_scores[region_id], -region_id))
    if active_region_id not in region_scores:
        return best_region
    if best_region == active_region_id:
        return active_region_id
    active_score = max(0.0, region_scores[active_region_id])
    threshold = active_score * max(1.0, float(switch_ratio))
    return best_region if region_scores[best_region] > threshold else active_region_id


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


def clustered_frontier_cells(clusters: Sequence[Sequence[Cell]]) -> Set[Cell]:
    """Return only frontier cells that survived cluster-size filtering."""
    return {cell for cluster in clusters for cell in cluster}


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


def advance_reroute_failure_streak(current_streak: int,
                                   failure_limit: int) -> Tuple[int, bool]:
    """Bound how long an invalid active route can keep the robot stopped."""
    next_streak = current_streak + 1
    return next_streak, next_streak >= failure_limit


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


@dataclass
class FrontierRegion:
    """A spatial group of frontier clusters with an ID stable across updates."""

    region_id: int
    clusters: List[List[Cell]]
    centroid: Tuple[float, float]
    cells: Set[Cell]
    topology_key: Optional[Tuple[int, int]] = None


def partition_frontier_clusters(clusters: Sequence[Sequence[Cell]],
                                region_size_cells: int,
                                grid: Optional[ExplorationGrid] = None,
                                inflated: Optional[Set[Cell]] = None,
                                max_path_ratio: float = 1.5,
                                max_detour_ratio: float = 1.75
                                ) -> List[FrontierRegion]:
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
    representatives = [
        min(cluster, key=lambda cell: (
            (cell[0] - centroids[index][0]) ** 2
            + (cell[1] - centroids[index][1]) ** 2,
            cell[0], cell[1]))
        for index, cluster in enumerate(nonempty)]

    # A local known-free route acts as the portal test. Frontiers that are
    # close in XY but sit across a wall or behind different entrances have
    # no short local connection and must remain separate topology nodes.
    path_distances: Dict[Tuple[int, int], float] = {}
    if grid is not None:
        obstacles = inflated or set()
        maximum_path = link_distance * max(1.0, float(max_path_ratio))
        for source, source_cell in enumerate(representatives):
            for target in range(source + 1, len(representatives)):
                euclidean = math.hypot(
                    centroids[source][0] - centroids[target][0],
                    centroids[source][1] - centroids[target][1])
                if euclidean > link_distance:
                    continue
                path = astar_known(
                    grid, source_cell, representatives[target], obstacles,
                    max_cost_cells=maximum_path)
                if path:
                    path_distances[(source, target)] = path_length_cells(path, 1.0)
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
                if grid is not None:
                    topology_ok = True
                    for a in groups[first]:
                        for b in groups[second]:
                            key = (min(a, b), max(a, b))
                            path_distance = path_distances.get(key)
                            euclidean = math.hypot(
                                centroids[a][0] - centroids[b][0],
                                centroids[a][1] - centroids[b][1])
                            if (path_distance is None
                                    or path_distance > link_distance * max_path_ratio
                                    or path_distance > max(1.0, euclidean) * max_detour_ratio):
                                topology_ok = False
                                break
                        if not topology_ok:
                            break
                    if not topology_ok:
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


def partition_frontier_clusters_by_topology(
        clusters: Sequence[Sequence[Cell]],
        topology_keys: Sequence[Optional[Tuple[int, int]]],
        region_size_cells: int, grid: Optional[ExplorationGrid] = None,
        inflated: Optional[Set[Cell]] = None, max_path_ratio: float = 1.5,
        max_detour_ratio: float = 1.75) -> List[FrontierRegion]:
    """Partition attached frontiers by component/corridor before proximity.

    Attached clusters never run pairwise dense A*. Unattached clusters retain
    the conservative legacy test, so incomplete topology cannot invent a
    connection or make an exploration target disappear.
    """
    if len(clusters) != len(topology_keys):
        raise ValueError("topology_keys must correspond one-to-one with clusters")
    keyed: Dict[Tuple[int, int], List[Sequence[Cell]]] = {}
    fallback: List[Sequence[Cell]] = []
    for cluster, key in zip(clusters, topology_keys):
        if not cluster:
            continue
        if key is None:
            fallback.append(cluster)
        else:
            keyed.setdefault(key, []).append(cluster)

    observations: List[FrontierRegion] = []
    for key in sorted(keyed):
        for region in partition_frontier_clusters(
                keyed[key], region_size_cells):
            observations.append(FrontierRegion(
                region.region_id, region.clusters, region.centroid,
                region.cells, key))
    if fallback:
        observations.extend(partition_frontier_clusters(
            fallback, region_size_cells, grid, inflated,
            max_path_ratio, max_detour_ratio))
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
                if (observation.topology_key is not None
                        and old.topology_key is not None
                        and observation.topology_key[1] != old.topology_key[1]):
                    # A component's canonical id may decrease as exploration
                    # connects new graph nodes.  The corridor branch id is the
                    # stable identity that must never cross during tracking.
                    continue
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
                best_id, observation.clusters, observation.centroid,
                observation.cells, observation.topology_key))
        self.regions = {region.region_id: region for region in assigned}
        return sorted(assigned, key=lambda region: region.region_id)


@dataclass
class RegionCommitmentUpdate:
    active_region_id: Optional[int]
    missing_streak: int
    unreachable_since: Optional[float]
    release_reason: Optional[str] = None


def update_region_commitment(
        active_region_id: Optional[int], observed_region_ids: Set[int],
        candidate_region_ids: Set[int], missing_streak: int,
        unreachable_since: Optional[float], now: float,
        missing_release_updates: int,
        unreachable_timeout: float) -> RegionCommitmentUpdate:
    """Debounce transient map changes without retaining an unusable region forever."""
    if active_region_id is None:
        return RegionCommitmentUpdate(None, 0, None)

    if active_region_id not in observed_region_ids:
        missing_streak += 1
        if missing_streak >= max(1, missing_release_updates):
            return RegionCommitmentUpdate(None, 0, None, "frontier_missing")
        return RegionCommitmentUpdate(active_region_id, missing_streak, None)

    if active_region_id in candidate_region_ids:
        return RegionCommitmentUpdate(active_region_id, 0, None)

    if unreachable_since is None:
        unreachable_since = now
    if now - unreachable_since >= max(0.0, unreachable_timeout):
        return RegionCommitmentUpdate(
            None, 0, None, "persistently_unreachable")
    return RegionCommitmentUpdate(active_region_id, 0, unreachable_since)


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
