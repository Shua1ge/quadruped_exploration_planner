"""Read-only safe-region connectivity used to value frontier observations."""

import heapq
import json
import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple


Point2 = Tuple[float, float]


@dataclass(frozen=True)
class RegionView:
    region_id: int
    anchor: Point2


@dataclass(frozen=True)
class ProvisionalPortal:
    source_id: int
    target_id: int
    known_distance: float
    hypothesized_distance: float
    route_savings: float


class SafeRegionConnectivity:
    """A compact snapshot; hypotheses value tasks but never add route edges."""

    def __init__(self):
        self.map_revision = 0
        self.regions: Dict[int, RegionView] = {}
        self.adjacency: Dict[int, Dict[int, float]] = {}

    def update_json(self, value: str) -> bool:
        try:
            payload = json.loads(value)
            revision = int(payload["map_revision"])
            regions = {
                int(item["id"]): RegionView(
                    int(item["id"]),
                    (float(item["x"]), float(item["y"])))
                for item in payload.get("regions", [])}
            adjacency: Dict[int, Dict[int, float]] = {
                region_id: {} for region_id in regions}
            for item in payload.get("portals", []):
                source = int(item["source"])
                target = int(item["target"])
                if source not in regions or target not in regions:
                    continue
                weight = max(1e-6, float(item["length"]))
                adjacency[source][target] = min(
                    weight, adjacency[source].get(target, math.inf))
                adjacency[target][source] = min(
                    weight, adjacency[target].get(source, math.inf))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return False
        if revision < self.map_revision:
            return False
        self.map_revision = revision
        self.regions = regions
        self.adjacency = adjacency
        return True

    def distance(self, source: int, target: int) -> Optional[float]:
        queue = [(0.0, source)]
        costs = {source: 0.0}
        while queue:
            cost, current = heapq.heappop(queue)
            if current == target:
                return cost
            if cost != costs.get(current):
                continue
            for neighbour, weight in self.adjacency.get(current, {}).items():
                candidate = cost + weight
                if candidate < costs.get(neighbour, math.inf):
                    costs[neighbour] = candidate
                    heapq.heappush(queue, (candidate, neighbour))
        return None

    def hypothesis(self, frontier: Point2, neighbourhood: float = 2.5,
                   minimum_savings: float = 4.0
                   ) -> Optional[ProvisionalPortal]:
        """Return the best nearby missing connection without mutating the graph."""
        nearby = sorted(
            ((math.hypot(region.anchor[0] - frontier[0],
                         region.anchor[1] - frontier[1]), region)
             for region in self.regions.values()),
            key=lambda item: (item[0], item[1].region_id))
        nearby = [item for item in nearby if item[0] <= neighbourhood][:4]
        best = None
        for first_index, (first_leg, first) in enumerate(nearby):
            for second_leg, second in nearby[first_index + 1:]:
                if second.region_id in self.adjacency.get(first.region_id, {}):
                    continue
                known = self.distance(first.region_id, second.region_id)
                if known is None:
                    continue
                hypothesized = first_leg + second_leg
                savings = known - hypothesized
                if savings < minimum_savings:
                    continue
                hypothesis = ProvisionalPortal(
                    first.region_id, second.region_id,
                    known, hypothesized, savings)
                key = (savings, -hypothesized,
                       -min(first.region_id, second.region_id))
                if best is None or key > best[0]:
                    best = (key, hypothesis)
        return None if best is None else best[1]
