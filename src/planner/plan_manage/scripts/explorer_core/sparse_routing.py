"""Incremental sparse-topology route queries for global exploration."""

import heapq
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple


Point2 = Tuple[float, float]
Attachment = Tuple[int, float]


@dataclass(frozen=True)
class SparseNode:
    node_id: int
    position: Point2
    clearance: float = 0.0
    traversability: float = 1.0
    terrain_state: int = 0


@dataclass(frozen=True)
class SparseEdge:
    edge_id: int
    source_id: int
    target_id: int
    length: float
    minimum_clearance: float
    polyline: Tuple[Point2, ...]
    # Global's route-cost multiplier ([0.8, 2.0]); never a hard block.
    traversability: float = 1.0
    terrain_state: int = 0
    traversals: int = 0
    experience_score: float = 0.0


@dataclass(frozen=True)
class SparseRouteEstimate:
    distance: float
    turn_cost: float
    source_node_id: int
    target_node_id: int
    polyline: Tuple[Point2, ...] = ()
    graph_revision: int = 0
    # Directed graph transitions, not nearest-polyline/arc-length labels.
    connections: Tuple[Tuple[int, int, int], ...] = ()
    route_source: str = "graph"
    # Extra cost above geometric length from Global's terrain/experience.
    terrain_cost: float = 0.0


@dataclass(frozen=True)
class EdgeAttachment:
    edge_id: int
    along: float
    connector_cost: float
    position: Point2
    sample_index: int = 0
    connector: Tuple[Point2, ...] = ()


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
    nearby_count: int = 0
    rejected_count: int = 0
    node_paths: Dict[int, Tuple[Point2, ...]] = field(default_factory=dict)
    node_connections: Dict[int, Tuple[Tuple[int,int,int], ...]] = field(default_factory=dict)
    # Terrain surcharge that belongs to each node option's weighted connector.
    node_penalties: Dict[int, float] = field(default_factory=dict)


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
        self._support_rows = {}
        self._support_anchors = {}
        self._support_edge_refs = {}
        self._support_trees = {}
        self._support_resolution = None
        self._support_map_revision = -1
        self._support_epoch = -1

    @property
    def ready(self) -> bool:
        return bool(self.nodes and (self.edges or self._support_rows))

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
        self._support_rows.clear()
        self._support_anchors.clear()
        self._support_edge_refs.clear()
        self._support_trees.clear()
        self._support_resolution = None
        self._support_map_revision = -1
        self._support_epoch = -1

    def _bucket(self, point: Point2) -> Tuple[int, int]:
        return (math.floor(point[0] / self.bucket_size),
                math.floor(point[1] / self.bucket_size))

    @staticmethod
    def _validate_point(point: Point2, label: str):
        if len(point) != 2 or not all(math.isfinite(value) for value in point):
            raise ValueError(f"{label} must contain two finite coordinates")

    @classmethod
    def _validate_price(cls, value: float, label: str):
        if (not math.isfinite(value) or value < 0.5 or value > 4.0):
            raise ValueError(f"{label} must be finite and within [0.5, 4.0]")

    @classmethod
    def _validate_node(cls, node: SparseNode):
        if int(node.node_id) < 0:
            raise ValueError("sparse node id must be non-negative")
        cls._validate_point(node.position, "sparse node position")
        if not math.isfinite(node.clearance) or node.clearance < 0.0:
            raise ValueError("sparse node clearance must be finite and non-negative")
        cls._validate_price(node.traversability, "sparse node traversability")

    @classmethod
    def _validate_edge(cls, edge: SparseEdge):
        if int(edge.edge_id) < 0:
            raise ValueError("sparse edge id must be non-negative")
        if int(edge.source_id) < 0 or int(edge.target_id) < 0:
            raise ValueError("sparse edge endpoints must be non-negative")
        if int(edge.source_id) == int(edge.target_id):
            raise ValueError("sparse edge endpoints must be distinct")
        if not math.isfinite(edge.length) or edge.length < 0.0:
            raise ValueError("sparse edge length must be finite and non-negative")
        if (not math.isfinite(edge.minimum_clearance)
                or edge.minimum_clearance < 0.0):
            raise ValueError(
                "sparse edge minimum clearance must be finite and non-negative")
        for index, point in enumerate(edge.polyline):
            cls._validate_point(point, f"sparse edge polyline point {index}")
        cls._validate_price(edge.traversability, "sparse edge traversability")

    def upsert_node(self, node: SparseNode):
        self._validate_node(node)
        new_key = self._bucket(node.position)
        old_key = self._node_bucket_keys.get(node.node_id)
        if old_key is not None:
            bucket = self._node_buckets.get(old_key)
            if bucket is not None:
                bucket.pop(node.node_id, None)
                if not bucket:
                    self._node_buckets.pop(old_key, None)
        self.nodes[node.node_id] = node
        self._node_buckets.setdefault(new_key, {})[node.node_id] = node.position
        self._node_bucket_keys[node.node_id] = new_key
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
        self._validate_edge(edge)
        # Resolve every bucket before mutating the existing edge.  This keeps
        # a direct update atomic even if a future bucket implementation adds
        # stricter coordinate checks.
        sample_buckets = [self._bucket(point) for point in edge.polyline]
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
            bucket_key = sample_buckets[index]
            sample_key = (edge.edge_id, index)
            self._sample_buckets.setdefault(bucket_key, {})[sample_key] = (
                point, min(max(0.0, along), edge.length))
            samples.append((bucket_key, sample_key))
        self._edge_sample_keys[edge.edge_id] = samples
        self._components_dirty = True

    def edge_cost(self, edge_id: int) -> float:
        """Weighted length: Global's traversability scales the geometric length."""
        edge = self.edges.get(edge_id)
        return 1.0 if edge is None else float(edge.traversability)

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
        source_map_revision = int(source_map_revision)
        if graph_revision <= 0:
            return
        if source_map_revision < 0:
            raise ValueError("source map revision must be non-negative")
        added_nodes = tuple(added_nodes)
        updated_nodes = tuple(updated_nodes)
        removed_node_ids = tuple(int(value) for value in removed_node_ids)
        added_edges = tuple(added_edges)
        updated_edges = tuple(updated_edges)
        removed_edge_ids = tuple(int(value) for value in removed_edge_ids)

        # Validate the complete message before touching any graph container.
        # Once this phase succeeds, every coordinate-to-bucket conversion used
        # by the commit phase is guaranteed to be finite.
        for node in added_nodes + updated_nodes:
            self._validate_node(node)
            self._bucket(node.position)
        for edge in added_edges + updated_edges:
            self._validate_edge(edge)
            for point in edge.polyline:
                self._bucket(point)
        if any(value < 0 for value in removed_node_ids + removed_edge_ids):
            raise ValueError("removed topology ids must be non-negative")

        if self.graph_revision and graph_revision < self.graph_revision:
            self.clear()
        if graph_revision <= self.graph_revision:
            return
        for edge_id in removed_edge_ids:
            self.remove_edge(int(edge_id))
        for node_id in removed_node_ids:
            self.remove_node(int(node_id))
        for node in added_nodes + updated_nodes:
            self.upsert_node(node)
        for edge in added_edges + updated_edges:
            self.upsert_edge(edge)
        self.graph_revision = graph_revision
        self.source_map_revision = source_map_revision

    def _nearby_buckets(self, point: Point2, radius: float):
        first_x = math.floor((point[0] - radius) / self.bucket_size)
        last_x = math.floor((point[0] + radius) / self.bucket_size)
        first_y = math.floor((point[1] - radius) / self.bucket_size)
        last_y = math.floor((point[1] + radius) / self.bucket_size)
        for bx in range(first_x, last_x + 1):
            for by in range(first_y, last_y + 1):
                yield bx, by

    def update_supports(self, payload):
        """Replace the derived spatial support index, independently of tasks.

        World-grid row intervals are geometry references, not executable space.
        Far connectors ALWAYS require the caller's current map check. Node
        anchors must still match; stale snapshot data cannot move an anchor.
        """
        resolution = float(payload.get('resolution') or 0.)
        domains = payload.get('domains', ())
        revision = int(payload.get('source_map_revision', 0))
        epoch = int(payload.get('world_epoch', 0))
        if epoch < self._support_epoch or (epoch == self._support_epoch and revision < self._support_map_revision):
            return False
        edge_refs, trees = {}, {}
        if not domains:
            rows, anchors = {}, {}
        else:
            if not math.isfinite(resolution) or resolution <= 0:
                raise ValueError('invalid spatial support resolution')
            rows, anchors = {}, {}
            for item in domains:
                node_id = int(item['id']); anchor = tuple(map(float, item['anchor']))
                self._validate_point(anchor, 'support anchor')
                if node_id < 0 or node_id in anchors:
                    raise ValueError('invalid/duplicate support node')
                anchors[node_id] = anchor
                if 'edge_attachment' in item:
                    ref = item['edge_attachment']
                    if (len(ref) != 2 or any(int(v) != v or v < 0 for v in ref)):
                        raise ValueError('invalid support edge attachment')
                    edge_refs[node_id] = tuple(map(int, ref))
                if 'connector_tree' in item:
                    tree = item['connector_tree']
                    width = int(tree['width']); origin = tuple(tree['origin'])
                    steps = tree['steps']
                    if (width < 1 or width != tree['width'] or len(origin) != 2 or
                            any(int(v) != v for v in origin) or
                            not isinstance(steps, str) or len(steps) != width*width or
                            any(c not in '.012345678' for c in steps)):
                        raise ValueError('invalid support connector tree')
                    trees[node_id] = tuple(map(int, origin)), width, steps
                for row in item['rows']:
                    if len(row) != 3 or any(int(v) != v for v in row):
                        raise ValueError('support intervals must use integer world cells')
                    y, x0, x1 = map(int, row)
                    if x1 < x0:
                        raise ValueError('invalid support interval')
                    rows.setdefault(y, []).append((x0, x1, node_id))
        self._support_rows = {y: sorted(v) for y, v in rows.items()}
        self._support_anchors = anchors
        self._support_edge_refs = edge_refs
        self._support_trees = trees
        self._support_resolution = resolution
        self._support_map_revision = revision
        self._support_epoch = epoch
        return True

    def _support_domains(self, point):
        if not self._support_rows:
            return ()
        x, y = (math.floor(v/self._support_resolution) for v in point)
        result = []
        for lo, hi, domain in self._support_rows.get(y, ()):
            if not lo <= x <= hi:
                continue
            anchor = self._support_anchors[domain]
            if domain in self._support_edge_refs:
                eid, index = self._support_edge_refs[domain]
                edge = self.edges.get(eid)
                if edge and index < len(edge.polyline) and math.dist(edge.polyline[index], anchor) < 1e-5:
                    result.append(domain)
            elif domain in self.nodes and math.dist(self.nodes[domain].position, anchor) < 1e-5:
                result.append(domain)
        return tuple(result)

    def support_nodes(self, point):
        nodes = set()
        for domain in self._support_domains(point):
            if domain in self._support_edge_refs:
                edge = self.edges[self._support_edge_refs[domain][0]]
                nodes.update((edge.source_id, edge.target_id))
            else:
                nodes.add(domain)
        return tuple(sorted(nodes))

    def _support_connector(self, point, domain, allowed):
        anchor = self._support_anchors[domain]
        if allowed(point, anchor) and allowed(anchor, point):
            return (point, anchor)
        tree = self._support_trees.get(domain)
        if tree is None:
            return None
        (x0, y0), width, steps = tree
        resolution = self._support_resolution
        def chain(position):
            cell = tuple(math.floor(v/resolution) for v in position)
            result, seen = [], set()
            while cell not in seen:
                x, y = cell[0]-x0, cell[1]-y0
                if not (0 <= x < width and 0 <= y < width):
                    return None
                code = steps[y*width+x]
                if code == '.':
                    return None
                seen.add(cell); result.append(cell)
                if code == '4':
                    return result
                step = int(code)
                cell = cell[0]+step % 3-1, cell[1]+step//3-1
            return None  # malformed/cyclic witness never triggers a search
        left, right = chain(point), chain(anchor)
        if left is None or right is None or left[-1] != right[-1]:
            return None
        # Trim their shared tail, so a connector need not detour to the root.
        while len(left) > 1 and len(right) > 1 and left[-2] == right[-2]:
            left.pop(); right.pop()
        cells = left+list(reversed(right[:-1]))
        path = self._join_points((point,), tuple(
            ((x+.5)*resolution, (y+.5)*resolution) for x, y in cells), (anchor,))
        if all(allowed(a, b) and allowed(b, a) for a, b in zip(path, path[1:])):
            return path
        return None

    def attachment_details(
            self, point: Point2, radius: float, limit: int = 8,
            connector_allowed: Optional[Callable[[Point2, Point2], bool]] = None
            ) -> SparseAttachments:
        """Return nearby node and along-edge attachment options."""
        radius = max(0.0, float(radius))
        allowed = connector_allowed or (lambda _source, _target: True)
        options: Dict[int, float] = {}
        penalties: Dict[int, float] = {}
        node_paths: Dict[int, Tuple[Point2, ...]] = {}
        node_connections = {}
        edge_options: Dict[int, List[EdgeAttachment]] = {}
        nearby_count = 0
        rejected_count = 0
        def add_edge(edge_id, sample_index, position, along, distance, connector=None):
            edge = self.edges[edge_id]
            connector = connector or (point, position)
            edge_options.setdefault(edge_id, []).append(EdgeAttachment(
                edge_id, along, distance, position, sample_index, connector))
            multiplier = self.edge_cost(edge_id)
            source_along = along
            target_along = max(0.0, edge.length - along)
            source_cost = distance + source_along * multiplier
            target_cost = distance + target_along * multiplier
            if source_cost < options.get(edge.source_id, math.inf):
                options[edge.source_id] = source_cost
                penalties[edge.source_id] = (
                    source_along * (multiplier - 1.0))
                node_paths[edge.source_id] = self._join_points(
                    connector,
                    tuple(reversed(edge.polyline[:sample_index + 1])))
                node_connections[edge.source_id]=((edge_id,edge.target_id,edge.source_id),) if sample_index>0 else ()
            if target_cost < options.get(edge.target_id, math.inf):
                options[edge.target_id] = target_cost
                penalties[edge.target_id] = (
                    target_along * (multiplier - 1.0))
                node_paths[edge.target_id] = self._join_points(
                    connector, edge.polyline[sample_index:])
                node_connections[edge.target_id]=((edge_id,edge.source_id,edge.target_id),) if sample_index<len(edge.polyline)-1 else ()
        for key in self._nearby_buckets(point, radius):
            for node_id, position in self._node_buckets.get(key, {}).items():
                distance = math.hypot(position[0] - point[0],
                                      position[1] - point[1])
                if distance > radius:
                    continue
                nearby_count += 1
                if not allowed(point, position):
                    rejected_count += 1
                    continue
                if distance < options.get(node_id, math.inf):
                    options[node_id] = distance
                    penalties[node_id] = 0.0
                    node_paths[node_id] = (point, position)
                    node_connections[node_id] = ()

            for (edge_id, sample_index), (position, along) in self._sample_buckets.get(
                    key, {}).items():
                distance = math.hypot(position[0] - point[0],
                                      position[1] - point[1])
                if distance > radius:
                    continue
                nearby_count += 1
                if not allowed(point, position):
                    rejected_count += 1
                    continue
                edge = self.edges.get(edge_id)
                if edge is None:
                    continue
                add_edge(edge_id, sample_index, position, along, distance)
        # Spatial support removes the fixed-radius coverage requirement. It
        # does not authorize an unobserved shortcut through a concave domain.
        if connector_allowed is not None:
            for domain in self._support_domains(point):
                position = self._support_anchors[domain]
                nearby_count += 1
                connector = self._support_connector(point, domain, allowed)
                if connector is None:
                    rejected_count += 1
                    continue
                distance = sum(math.dist(a, b) for a, b in zip(connector, connector[1:]))
                if domain in self._support_edge_refs:
                    eid, index = self._support_edge_refs[domain]
                    edge = self.edges[eid]
                    # Reuse the along-distance already indexed at edge ingest;
                    # compression must not make every target rescan the edge.
                    along = self._sample_buckets[self._bucket(position)][(eid, index)][1]
                    add_edge(eid, index, position, along, distance, connector)
                    continue
                multiplier = self.nodes[domain].traversability
                cost = distance*multiplier
                if cost < options.get(domain, math.inf):
                    options[domain] = cost
                    penalties[domain] = distance*(multiplier-1.)
                    node_paths[domain] = connector
                    node_connections[domain] = ()
        node_options = sorted(
            options.items(), key=lambda item: (item[1], item[0]))[
                :max(1, int(limit))]
        for edge_id, attachments in edge_options.items():
            edge_options[edge_id] = sorted(
                attachments,
                key=lambda item: (item.connector_cost, item.along))[
                    :max(1, int(limit))]
        retained_paths = {
            node_id: node_paths[node_id] for node_id, _ in node_options
            if node_id in node_paths}
        return SparseAttachments(
            node_options, edge_options, nearby_count, rejected_count,
            retained_paths,
            {node_id: node_connections.get(node_id, ())
             for node_id, _ in node_options},
            {node_id: penalties.get(node_id, 0.0)
             for node_id, _ in node_options})

    @staticmethod
    def _join_points(*parts: Sequence[Point2]) -> Tuple[Point2, ...]:
        result: List[Point2] = []
        for part in parts:
            for point in part:
                if not result or point != result[-1]:
                    result.append(point)
        return tuple(result)

    @staticmethod
    def _turn_cost(points: Sequence[Point2]) -> float:
        headings = [
            math.atan2(second[1] - first[1], second[0] - first[0])
            for first, second in zip(points[:-1], points[1:])
            if first != second]
        return sum(abs(math.atan2(math.sin(second - first),
                                  math.cos(second - first)))
                   for first, second in zip(headings[:-1], headings[1:]))

    def _oriented_edge_polyline(
            self, edge_id: int, source_id: int, target_id: int
            ) -> Tuple[Point2, ...]:
        edge = self.edges[edge_id]
        if edge.source_id == source_id and edge.target_id == target_id:
            return edge.polyline
        return tuple(reversed(edge.polyline))

    def _graph_polyline(
            self, target_node: int,
            parents: Dict[int, Tuple[int, int]]) -> Tuple[Point2, ...]:
        transitions = []
        current = target_node
        while current in parents:
            previous, edge_id = parents[current]
            transitions.append((previous, current, edge_id))
            current = previous
        points: Tuple[Point2, ...] = ()
        for source_id, target_id, edge_id in reversed(transitions):
            points = self._join_points(
                points,
                self._oriented_edge_polyline(edge_id, source_id, target_id))
        if not points:
            points = (self.nodes[target_node].position,)
        return points

    @staticmethod
    def _graph_connections(target_node, parents):
        transitions=[]
        current=target_node
        while current in parents:
            previous,edge_id=parents[current]
            transitions.append((edge_id,previous,current))
            current=previous
        return tuple(reversed(transitions))

    def _same_edge_polyline(
            self, first: EdgeAttachment, second: EdgeAttachment
            ) -> Tuple[Point2, ...]:
        edge = self.edges[first.edge_id]
        if first.sample_index <= second.sample_index:
            return edge.polyline[first.sample_index:second.sample_index + 1]
        return tuple(reversed(
            edge.polyline[second.sample_index:first.sample_index + 1]))

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
        estimates, _ = self.batch_estimates_with_reasons(
            start, targets, attachment_radius, attachment_limit,
            connector_allowed)
        return estimates

    def batch_estimates_with_reasons(
            self, start: Point2, targets: Sequence[Point2],
            attachment_radius: float, attachment_limit: int = 8,
            connector_allowed: Optional[
                Callable[[Point2, Point2], bool]] = None
            ) -> Tuple[List[Optional[SparseRouteEstimate]],
                       List[Optional[str]]]:
        """Estimate routes and identify why each missing route fell back."""
        if not self.ready or not targets:
            return ([None] * len(targets),
                    ["start_unattached"] * len(targets))
        start_details = self.attachment_details(
            start, attachment_radius, attachment_limit, connector_allowed)
        target_details = [self.attachment_details(
            target, attachment_radius, attachment_limit, connector_allowed)
            for target in targets]
        start_attachments = start_details.node_options
        target_attachments = [details.node_options for details in target_details]
        if not start_attachments:
            reason = ("connector_rejected"
                      if start_details.rejected_count else "start_unattached")
            return [None] * len(targets), [reason] * len(targets)

        queue = []
        costs: Dict[int, float] = {}
        penalties: Dict[int, float] = {}
        parents: Dict[int, Tuple[int, int]] = {}
        roots: Dict[int, int] = {}
        for node_id, connector_cost in start_attachments:
            if connector_cost >= costs.get(node_id, math.inf):
                continue
            costs[node_id] = connector_cost
            penalties[node_id] = start_details.node_penalties.get(node_id, 0.0)
            roots[node_id] = node_id
            heapq.heappush(queue, (connector_cost, node_id))

        target_nodes = {node_id for attachments in target_attachments
                        for node_id, _ in attachments}
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
                multiplier = self.edge_cost(edge_id)
                candidate = current_cost + length * multiplier
                if candidate >= costs.get(neighbour, math.inf):
                    continue
                costs[neighbour] = candidate
                penalties[neighbour] = (
                    penalties[current] + length * (multiplier - 1.0))
                parents[neighbour] = (current, edge_id)
                roots[neighbour] = roots[current]
                heapq.heappush(queue, (candidate, neighbour))

        results: List[Optional[SparseRouteEstimate]] = []
        reasons: List[Optional[str]] = []
        for target, details in zip(targets, target_details):
            common_support = set(self._support_domains(start)) & set(self._support_domains(target))
            if (common_support and connector_allowed is not None
                    and connector_allowed(start, target) and connector_allowed(target, start)):
                domain = min(common_support)
                if domain in self._support_edge_refs:
                    edge = self.edges[self._support_edge_refs[domain][0]]
                    node_id, price = edge.source_id, edge.traversability
                else:
                    node_id, price = domain, self.nodes[domain].traversability
                distance = math.dist(start, target)
                results.append(SparseRouteEstimate(distance*price, 0., node_id, node_id,
                    (start, target), self.graph_revision, (), "space_connector",
                    distance*(price-1.)))
                reasons.append(None)
                continue
            if (connector_allowed is not None and math.dist(start,target) <= attachment_radius
                    and connector_allowed(start,target) and connector_allowed(target,start)):
                # Short visible motion needs no detour through a graph anchor.
                # Keep its provenance explicit; this is not a graph-edge hit.
                results.append(SparseRouteEstimate(math.dist(start,target),0.,
                    start_attachments[0][0],start_attachments[0][0],
                    (start,target),self.graph_revision,(),"direct_connector"))
                reasons.append(None)
                continue
            attachments = details.node_options
            if not attachments:
                results.append(None)
                reasons.append(
                    "connector_rejected"
                    if details.rejected_count else "target_unattached")
                continue
            choices = [(costs[node_id] + connector_cost, node_id)
                       for node_id, connector_cost in attachments
                       if node_id in costs]
            direct_choices = []
            for edge_id, start_edges in start_details.edge_options.items():
                multiplier = self.edge_cost(edge_id)
                for start_edge in start_edges:
                    for target_edge in details.edge_options.get(edge_id, []):
                        along = abs(target_edge.along - start_edge.along)
                        direct_choices.append((
                            start_edge.connector_cost + along * multiplier
                            + target_edge.connector_cost,
                            start_edge, target_edge,
                            along * (multiplier - 1.0)))
            graph_choice = min(choices) if choices else None
            direct_choice = min(
                direct_choices, key=lambda item: item[0]) if direct_choices else None
            if graph_choice is None and direct_choice is None:
                results.append(None)
                reasons.append("disconnected")
                continue
            if (direct_choice is not None
                    and (graph_choice is None or direct_choice[0] < graph_choice[0])):
                total, start_edge, target_edge, terrain_surcharge = direct_choice
                edge = self.edges[start_edge.edge_id]
                points = self._join_points(
                    start_edge.connector or (start, start_edge.position),
                    self._same_edge_polyline(start_edge, target_edge),
                    tuple(reversed(target_edge.connector)) if target_edge.connector
                    else (target_edge.position, target))
                results.append(SparseRouteEstimate(
                    total, self._turn_cost(points), edge.source_id,
                    edge.target_id, points, self.graph_revision,
                    terrain_cost=terrain_surcharge))
                reasons.append(None)
                continue

            total, target_node = graph_choice
            source_node = roots[target_node]
            points = self._join_points(
                start_details.node_paths[source_node],
                self._graph_polyline(target_node, parents),
                tuple(reversed(details.node_paths[target_node])))
            results.append(SparseRouteEstimate(
                total, self._turn_cost(points), source_node, target_node,
                points,self.graph_revision,
                start_details.node_connections.get(source_node,())+
                self._graph_connections(target_node,parents)+
                tuple((edge,target,source) for edge,source,target in
                      reversed(details.node_connections.get(target_node,()))),
                terrain_cost=(penalties.get(target_node, 0.0)
                              + details.node_penalties.get(target_node, 0.0))))
            reasons.append(None)
        return results, reasons
