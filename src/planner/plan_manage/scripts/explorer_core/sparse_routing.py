"""Incremental sparse-topology route queries for global exploration."""

import heapq
import math
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple


Point2 = Tuple[float, float]
Attachment = Tuple[int, float]


@dataclass(frozen=True)
class SparseNode:
    node_id: int
    position: Point2
    clearance: float = 0.0


@dataclass(frozen=True)
class SparseEdge:
    edge_id: int
    source_id: int
    target_id: int
    length: float
    minimum_clearance: float
    polyline: Tuple[Point2, ...]


@dataclass(frozen=True)
class SparseRouteEstimate:
    distance: float
    turn_cost: float
    source_node_id: int
    target_node_id: int


@dataclass(frozen=True)
class EdgeAttachment:
    edge_id: int
    along: float
    connector_cost: float
    position: Point2


@dataclass(frozen=True)
class TopologyAttachment:
    """Component and corridor identity for a point near the sparse graph."""

    component_id: int
    branch_id: int
    connector_cost: float


@dataclass
class SparseAttachments:
    node_options: List[Attachment]
    edge_options: Dict[int, List[EdgeAttachment]]


class SparseRouteGraph:
    """Apply topology deltas and answer one-to-many route queries.

    Query points attach to nearby graph nodes or sampled edge polylines.  An
    edge attachment contributes the along-edge distance to both endpoints, so
    a robot in the middle of a long compressed corridor does not need to be
    close to one of its endpoint nodes.
    """

    def __init__(self, bucket_size: float = 1.0):
        self.bucket_size = max(0.1, float(bucket_size))
        self.nodes: Dict[int, SparseNode] = {}
        self.edges: Dict[int, SparseEdge] = {}
        self.adjacency: Dict[int, Dict[int, Tuple[float, int]]] = {}
        self.graph_revision = 0
        self.source_map_revision = 0
        self._node_buckets: Dict[Tuple[int, int], Dict[int, Point2]] = {}
        self._node_bucket_keys: Dict[int, Tuple[int, int]] = {}
        self._sample_buckets: Dict[
            Tuple[int, int], Dict[Tuple[int, int], Tuple[Point2, float]]] = {}
        self._edge_sample_keys: Dict[int, List[Tuple[Tuple[int, int], Tuple[int, int]]]] = {}
        self._component_ids: Dict[int, int] = {}
        self._components_dirty = True

    @property
    def ready(self) -> bool:
        return bool(self.nodes and self.edges)

    def clear(self):
        self.nodes.clear()
        self.edges.clear()
        self.adjacency.clear()
        self._node_buckets.clear()
        self._node_bucket_keys.clear()
        self._sample_buckets.clear()
        self._edge_sample_keys.clear()
        self._component_ids.clear()
        self._components_dirty = True
        self.graph_revision = 0
        self.source_map_revision = 0

    def _bucket(self, point: Point2) -> Tuple[int, int]:
        return (math.floor(point[0] / self.bucket_size),
                math.floor(point[1] / self.bucket_size))

    def upsert_node(self, node: SparseNode):
        key = self._node_bucket_keys.pop(node.node_id, None)
        if key is not None:
            bucket = self._node_buckets.get(key)
            if bucket is not None:
                bucket.pop(node.node_id, None)
                if not bucket:
                    self._node_buckets.pop(key, None)
        self.nodes[node.node_id] = node
        key = self._bucket(node.position)
        self._node_buckets.setdefault(key, {})[node.node_id] = node.position
        self._node_bucket_keys[node.node_id] = key
        self.adjacency.setdefault(node.node_id, {})
        self._components_dirty = True

    def remove_node(self, node_id: int, remove_edges: bool = True):
        key = self._node_bucket_keys.pop(node_id, None)
        if key is not None:
            bucket = self._node_buckets.get(key)
            if bucket is not None:
                bucket.pop(node_id, None)
                if not bucket:
                    self._node_buckets.pop(key, None)
        self.nodes.pop(node_id, None)
        self.adjacency.pop(node_id, None)
        for neighbours in self.adjacency.values():
            neighbours.pop(node_id, None)
        if remove_edges:
            for edge_id, edge in list(self.edges.items()):
                if edge.source_id == node_id or edge.target_id == node_id:
                    self.remove_edge(edge_id)
        self._components_dirty = True

    def _remove_edge_samples(self, edge_id: int):
        for bucket_key, sample_key in self._edge_sample_keys.pop(edge_id, []):
            bucket = self._sample_buckets.get(bucket_key)
            if bucket is None:
                continue
            bucket.pop(sample_key, None)
            if not bucket:
                self._sample_buckets.pop(bucket_key, None)

    def remove_edge(self, edge_id: int):
        edge = self.edges.pop(edge_id, None)
        self._remove_edge_samples(edge_id)
        if edge is None:
            return
        source_neighbours = self.adjacency.get(edge.source_id)
        if (source_neighbours is not None
                and source_neighbours.get(edge.target_id, (None, None))[1] == edge_id):
            source_neighbours.pop(edge.target_id, None)
        target_neighbours = self.adjacency.get(edge.target_id)
        if (target_neighbours is not None
                and target_neighbours.get(edge.source_id, (None, None))[1] == edge_id):
            target_neighbours.pop(edge.source_id, None)
        self._components_dirty = True

    def upsert_edge(self, edge: SparseEdge):
        self.remove_edge(edge.edge_id)
        if edge.source_id not in self.nodes or edge.target_id not in self.nodes:
            return
        self.edges[edge.edge_id] = edge
        self.adjacency.setdefault(edge.source_id, {})[edge.target_id] = (
            edge.length, edge.edge_id)
        self.adjacency.setdefault(edge.target_id, {})[edge.source_id] = (
            edge.length, edge.edge_id)

        samples = []
        along = 0.0
        previous = None
        for index, point in enumerate(edge.polyline):
            if previous is not None:
                along += math.hypot(point[0] - previous[0],
                                    point[1] - previous[1])
            previous = point
            bucket_key = self._bucket(point)
            sample_key = (edge.edge_id, index)
            self._sample_buckets.setdefault(bucket_key, {})[sample_key] = (
                point, min(max(0.0, along), edge.length))
            samples.append((bucket_key, sample_key))
        self._edge_sample_keys[edge.edge_id] = samples
        self._components_dirty = True

    def _refresh_components(self):
        if not self._components_dirty:
            return
        component_ids: Dict[int, int] = {}
        remaining = set(self.nodes)
        while remaining:
            seed = min(remaining)
            stack = [seed]
            members = set()
            while stack:
                current = stack.pop()
                if current in members or current not in self.nodes:
                    continue
                members.add(current)
                stack.extend(self.adjacency.get(current, {}))
            component_id = min(members)
            for node_id in members:
                component_ids[node_id] = component_id
            remaining.difference_update(members)
        self._component_ids = component_ids
        self._components_dirty = False

    def topology_attachment(
            self, point: Point2, radius: float, limit: int = 8,
            connector_allowed: Optional[Callable[[Point2, Point2], bool]] = None
            ) -> Optional[TopologyAttachment]:
        """Attach a point to one connected component and corridor branch."""
        if not self.ready:
            return None
        details = self.attachment_details(
            point, radius, limit, connector_allowed)
        self._refresh_components()
        edge_options = [
            attachment
            for options in details.edge_options.values()
            for attachment in options]
        if edge_options:
            attachment = min(edge_options, key=lambda item: (
                item.connector_cost, item.edge_id, item.along))
            edge = self.edges.get(attachment.edge_id)
            if edge is not None:
                component_id = self._component_ids.get(edge.source_id)
                if component_id is not None:
                    return TopologyAttachment(
                        component_id, attachment.edge_id,
                        attachment.connector_cost)
        if not details.node_options:
            return None
        node_id, connector_cost = details.node_options[0]
        component_id = self._component_ids.get(node_id)
        if component_id is None:
            return None
        return TopologyAttachment(component_id, -node_id - 1, connector_cost)

    def apply_delta(self, graph_revision: int, source_map_revision: int,
                    added_nodes: Iterable[SparseNode],
                    updated_nodes: Iterable[SparseNode],
                    removed_node_ids: Iterable[int],
                    added_edges: Iterable[SparseEdge],
                    updated_edges: Iterable[SparseEdge],
                    removed_edge_ids: Iterable[int]):
        graph_revision = int(graph_revision)
        if graph_revision <= 0:
            return
        if self.graph_revision and graph_revision < self.graph_revision:
            self.clear()
        if graph_revision <= self.graph_revision:
            return
        for edge_id in removed_edge_ids:
            self.remove_edge(int(edge_id))
        for node_id in removed_node_ids:
            self.remove_node(int(node_id))
        for node in list(added_nodes) + list(updated_nodes):
            self.upsert_node(node)
        for edge in list(added_edges) + list(updated_edges):
            self.upsert_edge(edge)
        self.graph_revision = graph_revision
        self.source_map_revision = int(source_map_revision)

    def _nearby_buckets(self, point: Point2, radius: float):
        first_x = math.floor((point[0] - radius) / self.bucket_size)
        last_x = math.floor((point[0] + radius) / self.bucket_size)
        first_y = math.floor((point[1] - radius) / self.bucket_size)
        last_y = math.floor((point[1] + radius) / self.bucket_size)
        for bx in range(first_x, last_x + 1):
            for by in range(first_y, last_y + 1):
                yield bx, by

    def attachment_details(
            self, point: Point2, radius: float, limit: int = 8,
            connector_allowed: Optional[Callable[[Point2, Point2], bool]] = None
            ) -> SparseAttachments:
        """Return nearby node and along-edge attachment options."""
        radius = max(0.0, float(radius))
        allowed = connector_allowed or (lambda _source, _target: True)
        options: Dict[int, float] = {}
        edge_options: Dict[int, List[EdgeAttachment]] = {}
        for key in self._nearby_buckets(point, radius):
            for node_id, position in self._node_buckets.get(key, {}).items():
                distance = math.hypot(position[0] - point[0],
                                      position[1] - point[1])
                if distance <= radius and allowed(point, position):
                    options[node_id] = min(options.get(node_id, math.inf), distance)

            for (edge_id, _), (position, along) in self._sample_buckets.get(
                    key, {}).items():
                distance = math.hypot(position[0] - point[0],
                                      position[1] - point[1])
                if distance > radius or not allowed(point, position):
                    continue
                edge = self.edges.get(edge_id)
                if edge is None:
                    continue
                edge_options.setdefault(edge_id, []).append(EdgeAttachment(
                    edge_id, along, distance, position))
                source_cost = distance + along
                target_cost = distance + max(0.0, edge.length - along)
                options[edge.source_id] = min(
                    options.get(edge.source_id, math.inf), source_cost)
                options[edge.target_id] = min(
                    options.get(edge.target_id, math.inf), target_cost)
        node_options = sorted(
            options.items(), key=lambda item: (item[1], item[0]))[
                :max(1, int(limit))]
        for edge_id, attachments in edge_options.items():
            edge_options[edge_id] = sorted(
                attachments,
                key=lambda item: (item.connector_cost, item.along))[
                    :max(1, int(limit))]
        return SparseAttachments(node_options, edge_options)

    def attachments(
            self, point: Point2, radius: float, limit: int = 8,
            connector_allowed: Optional[Callable[[Point2, Point2], bool]] = None
            ) -> List[Attachment]:
        """Return graph-node attachments ordered by connector cost."""
        return self.attachment_details(
            point, radius, limit, connector_allowed).node_options

    def batch_estimates(
            self, start: Point2, targets: Sequence[Point2],
            attachment_radius: float, attachment_limit: int = 8,
            connector_allowed: Optional[Callable[[Point2, Point2], bool]] = None
            ) -> List[Optional[SparseRouteEstimate]]:
        """Run one multi-source Dijkstra and estimate routes to all targets."""
        if not self.ready or not targets:
            return [None] * len(targets)
        start_details = self.attachment_details(
            start, attachment_radius, attachment_limit, connector_allowed)
        target_details = [self.attachment_details(
            target, attachment_radius, attachment_limit, connector_allowed)
            for target in targets]
        start_attachments = start_details.node_options
        target_attachments = [details.node_options for details in target_details]
        if not start_attachments:
            return [None] * len(targets)

        queue = []
        costs: Dict[int, float] = {}
        parents: Dict[int, Tuple[int, int]] = {}
        roots: Dict[int, int] = {}
        for node_id, connector_cost in start_attachments:
            if connector_cost >= costs.get(node_id, math.inf):
                continue
            costs[node_id] = connector_cost
            roots[node_id] = node_id
            heapq.heappush(queue, (connector_cost, node_id))

        target_nodes = {node_id for attachments in target_attachments
                        for node_id, _ in attachments}
        if not target_nodes:
            return [None] * len(targets)
        settled_targets = set()
        while queue:
            current_cost, current = heapq.heappop(queue)
            if current_cost != costs.get(current):
                continue
            if current in target_nodes:
                settled_targets.add(current)
                if settled_targets == target_nodes:
                    break
            for neighbour, (length, edge_id) in self.adjacency.get(
                    current, {}).items():
                candidate = current_cost + length
                if candidate >= costs.get(neighbour, math.inf):
                    continue
                costs[neighbour] = candidate
                parents[neighbour] = (current, edge_id)
                roots[neighbour] = roots[current]
                heapq.heappush(queue, (candidate, neighbour))

        results: List[Optional[SparseRouteEstimate]] = []
        for target, details in zip(targets, target_details):
            attachments = details.node_options
            choices = [(costs[node_id] + connector_cost, node_id)
                       for node_id, connector_cost in attachments
                       if node_id in costs]
            direct_choices = []
            for edge_id, start_edges in start_details.edge_options.items():
                for start_edge in start_edges:
                    for target_edge in details.edge_options.get(edge_id, []):
                        direct_choices.append((
                            start_edge.connector_cost
                            + abs(target_edge.along - start_edge.along)
                            + target_edge.connector_cost,
                            start_edge, target_edge))
            graph_choice = min(choices) if choices else None
            direct_choice = min(
                direct_choices, key=lambda item: item[0]) if direct_choices else None
            if graph_choice is None and direct_choice is None:
                results.append(None)
                continue
            if (direct_choice is not None
                    and (graph_choice is None or direct_choice[0] < graph_choice[0])):
                total, start_edge, target_edge = direct_choice
                edge = self.edges[start_edge.edge_id]
                points = [start, start_edge.position,
                          target_edge.position, target]
                headings = [math.atan2(second[1] - first[1],
                                       second[0] - first[0])
                            for first, second in zip(points[:-1], points[1:])
                            if first != second]
                turn_cost = sum(abs(math.atan2(math.sin(second - first),
                                               math.cos(second - first)))
                                for first, second in zip(
                                    headings[:-1], headings[1:]))
                results.append(SparseRouteEstimate(
                    total, turn_cost, edge.source_id, edge.target_id))
                continue

            total, target_node = graph_choice
            source_node = roots[target_node]
            points = [target]
            current = target_node
            points.append(self.nodes[current].position)
            while current in parents:
                previous, _ = parents[current]
                current = previous
                points.append(self.nodes[current].position)
            points.append(start)
            points.reverse()
            headings = [math.atan2(second[1] - first[1],
                                   second[0] - first[0])
                        for first, second in zip(points[:-1], points[1:])
                        if first != second]
            turn_cost = sum(abs(math.atan2(math.sin(second - first),
                                           math.cos(second - first)))
                            for first, second in zip(headings[:-1], headings[1:]))
            results.append(SparseRouteEstimate(
                total, turn_cost, source_node, target_node))
        return results
