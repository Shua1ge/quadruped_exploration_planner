"""World-anchored, dirty-tile wavefront connection graph.

Each observed-free component of a fixed world tile owns one representative.
A metric, eight-connected wavefront (without corner cutting) certifies paths to tile
portals. Occupied evidence changes structure; an UNKNOWN rolling patch does
not erase history. Recomputed tiles REPLACE their old expressions, not union
successive skeletons. Execution constraints are deliberately not an input.
"""

from collections import deque
from dataclasses import dataclass
import math
import heapq
import time
from typing import Dict, Set, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

from .grid import FREE, OCCUPIED, UNKNOWN, bresenham
from .topology import TopologyEdge, TopologyGraph, TopologyNode, stable_edge_id
from .spatial_domains import SpatialDomainView

Cell = Tuple[int, int]
STEPS = ((-1, 0), (0, -1), (0, 1), (1, 0))


@dataclass
class Tile:
    components: Dict[int, Set[Cell]]
    owners: Dict[Cell, int]
    parents: Dict[Cell, Cell]
    distances: Dict[Cell, float]
    clearance: Dict[Cell, float]


@dataclass
class GraphUpdate:
    added_nodes: Dict[int, TopologyNode]
    updated_nodes: Dict[int, TopologyNode]
    removed_nodes: Set[int]
    added_edges: Dict[int, TopologyEdge]
    updated_edges: Dict[int, TopologyEdge]
    removed_edges: Set[int]
    lineage: Dict[int, Tuple[int, ...]]
    metrics: dict


class IncrementalWavefrontGraph:
    def __init__(self, tile_size=2.0, attachment_radius=3.0):
        if not math.isfinite(tile_size) or tile_size <= 0:
            raise ValueError('wavefront tile_size must be finite and positive')
        if not math.isfinite(attachment_radius) or attachment_radius <= 0:
            raise ValueError('wavefront attachment_radius must be finite and positive')
        self.attachment_radius = float(attachment_radius)
        self.tile_size = float(tile_size)
        self.resolution = None
        self.tile_cells = None
        self.occupancy = {}
        self.tiles = {}
        self.nodes = {}
        self.edges = {}
        self.adjacency = {}
        self.used_ids = set()
        # Successful measured travel is a preference, never occupancy evidence.
        # Cell -> (accepted traversal count, newest accepted stamp in seconds).
        self.traversed_cells = dict()
        # 2 m tile -> [total traversal count, newest stamp]; matches Local's
        # terrain tile origin so Global can aggregate both on one element.
        self.experience_tiles = dict()
        # Compatibility diagnostic: body motion never creates spatial structure.
        self.complex_traversal_cells = set()
        self.traversal_direction = None
        self.spatial_view = SpatialDomainView()
        self.spatial_dirty = set()
        self.experience_dirty = set()
        self.experience_revision = 0
        self.routing_revision = 0
        self._routing_cache = None

    def record_traversal(self, start, end, body_height_change=0., stamp=None):
        """Accept an actual measured segment only if every crossed cell is FREE.

        Diagonal side cells also have to be free. This cannot create a road or
        override a failure overlay; final execution certification remains Local.
        """
        if self.resolution is None or not all(math.isfinite(v) for v in (*start, *end)):
            return False
        a, b = (tuple(math.floor(v/self.resolution) for v in p) for p in (start, end))
        cells = bresenham(a, b)
        if any(self.occupancy.get(p) != FREE for p in cells):
            self.traversal_direction = None
            return False
        if any(x[0] != y[0] and x[1] != y[1] and
               (self.occupancy.get((x[0], y[1])) != FREE or
                self.occupancy.get((y[0], x[1])) != FREE)
               for x, y in zip(cells, cells[1:])):
            self.traversal_direction = None
            return False
        new = [p for p in cells if p not in self.traversed_cells]
        revisited = any(p in self.traversed_cells for p in cells)
        if not new and not revisited:
            return False
        stamp = time.time() if stamp is None else float(stamp)
        if not math.isfinite(stamp):
            return False
        if not new:
            # Reuse of an already-accepted corridor refreshes recency only; it
            # is not new experience and must not rebuild graph structure.
            for p in cells:
                record = self.traversed_cells.get(p)
                if record is not None:
                    self.traversed_cells[p] = (
                        record[0] + 1, max(record[1], stamp))
                    self._experience_add(p, 1, stamp)
            return False
        for p in cells:
            record = self.traversed_cells.get(p)
            if record is None:
                self.traversed_cells[p] = (1, stamp)
            else:
                self.traversed_cells[p] = (record[0] + 1, max(record[1], stamp))
            self._experience_add(p, 1, stamp)
        # FIFO budget, with explicit invalidation of evicted preferences.
        while len(self.traversed_cells) > 8192:
            p = next(iter(self.traversed_cells))
            count, _ = self.traversed_cells.pop(p)
            self._experience_add(p, -count, 0.0)
            self.complex_traversal_cells.discard(p)
        self.experience_revision += 1
        return True

    def _experience_add(self, cell, count_delta, stamp):
        """Maintain per-tile counts. A surviving tile keeps its newest stamp."""
        key = self.tile_key(cell)
        record = self.experience_tiles.get(key)
        if record is None:
            if count_delta > 0:
                self.experience_tiles[key] = [int(count_delta), float(stamp)]
            return
        record[0] = max(0, record[0] + int(count_delta))
        if count_delta > 0:
            record[1] = max(record[1], float(stamp))
        if record[0] == 0:
            self.experience_tiles.pop(key, None)

    def tile_key(self, cell):
        return cell[0] // self.tile_cells, cell[1] // self.tile_cells

    @staticmethod
    def neighbours(cell):
        return ((cell[0] + dx, cell[1] + dy) for dx, dy in STEPS)

    def _new_id(self, cells):
        # World anchors are immutable identities, not the movable seed pose.
        for x, y in sorted(cells):
            candidate = ((x & 0xffffffff) << 32) | (y & 0xffffffff)
            if candidate not in self.used_ids:
                self.used_ids.add(candidate)
                return candidate
        # A cell may have been consumed by an earlier split/merge generation.
        # Monotonic spare IDs do not resurrect an unrelated retired object.
        candidate = 0xffffffffffffffff
        while candidate in self.used_ids:
            candidate -= 1
        self.used_ids.add(candidate)
        return candidate

    def _build_tile(self, key, previous, before_nodes):
        size = self.tile_cells
        x0, y0 = key[0]*size, key[1]*size
        # A one-tile halo bounds distance work and provides neighbouring walls.
        mask = np.zeros((3*size+2, 3*size+2), dtype=bool)
        for y in range(y0-size, y0+2*size):
            for x in range(x0-size, x0+2*size):
                mask[y-y0+size+1, x-x0+size+1] = self.occupancy.get((x, y)) == FREE
        field = distance_transform_edt(mask) * self.resolution
        free = {(x, y) for y in range(y0, y0+size) for x in range(x0, x0+size)
                if self.occupancy.get((x, y)) == FREE}
        clearance = {p: float(field[p[1]-y0+size+1, p[0]-x0+size+1]) for p in free}
        components = []
        remaining = set(free)
        while remaining:
            first = min(remaining)
            remaining.remove(first)
            component, queue = {first}, deque([first])
            while queue:
                for nxt in self.neighbours(queue.popleft()):
                    if nxt in remaining:
                        remaining.remove(nxt)
                        component.add(nxt)
                        queue.append(nxt)
            components.append(component)
        # Largest overlap wins identity on splits and merges. New components
        # get new identities; position/degree changes keep the matched identity.
        old = previous.components if previous else {}
        matches = sorted((-len(cells & old_cells), old_id, index)
                         for index, cells in enumerate(components)
                         for old_id, old_cells in old.items() if cells & old_cells)
        assigned, claimed = {}, set()
        for _, old_id, index in matches:
            if old_id not in claimed and index not in assigned:
                assigned[index] = old_id
                claimed.add(old_id)
        result = Tile({}, {}, {}, {}, clearance)
        created = {}
        for index, cells in enumerate(components):
            node_id = assigned.get(index)
            if node_id is None:
                node_id = self._new_id(cells)
            cx = sum(p[0] for p in cells) / len(cells)
            cy = sum(p[1] for p in cells) / len(cells)
            peak = max(clearance[p] for p in cells)
            central = lambda p: (p[0]-cx)**2+(p[1]-cy)**2
            best = min((p for p in cells if clearance[p] >= .95*peak),
                       key=lambda p: (central(p), -clearance[p], p))
            old_node = before_nodes.get(node_id)
            seed = (old_node.cell if old_node and old_node.cell in cells
                    and clearance[old_node.cell] >= .95*peak
                    and central(old_node.cell) <= central(best)+.5 else best)
            created[node_id] = TopologyNode(node_id, seed, 0, clearance[seed])
            result.components[node_id] = cells
            result.parents[seed] = seed
            result.distances[seed] = 0
            result.owners[seed] = node_id
            # Metric wavefront, not a neighbour-order Manhattan staircase.
            # Diagonals require BOTH side cells: corner contact is not a road.
            costs = {seed: 0.0}
            queue = [(0.0, seed)]
            while queue:
                cost, p = heapq.heappop(queue)
                if cost != costs[p]:
                    continue
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nxt = (p[0]+dx, p[1]+dy)
                        if (not (dx or dy) or nxt not in cells or
                                (dx and dy and ((p[0]+dx, p[1]) not in cells
                                                or (p[0], p[1]+dy) not in cells))):
                            continue
                        step = math.hypot(dx, dy)
                        new_cost = cost + step * (1.+.1*self.resolution/clearance[nxt]
                                                 )
                        if new_cost < costs.get(nxt, math.inf)-1e-10:
                            costs[nxt] = new_cost
                            result.owners[nxt] = node_id
                            result.parents[nxt] = p
                            result.distances[nxt] = result.distances[p]+step
                            heapq.heappush(queue, (new_cost, nxt))
        lineage = {old_id: tuple(sorted(node_id for node_id, cells in result.components.items()
                                        if old_cells & cells))
                   for old_id, old_cells in old.items()}
        return result, created, lineage

    @staticmethod
    def _to_seed(tile, cell):
        path = [cell]
        while tile.parents[path[-1]] != path[-1]:
            path.append(tile.parents[path[-1]])
        return path

    def _border_edges(self, first_key, second_key):
        first, second = self.tiles.get(first_key), self.tiles.get(second_key)
        if first is None or second is None:
            return {}
        options = {}
        for p, source in first.owners.items():
            for q in self.neighbours(p):
                target = second.owners.get(q)
                if target is None:
                    continue
                score = (first.distances[p]+second.distances[q]+1,
                         -min(first.clearance[p], second.clearance[q]), p, q)
                pair = (source, target)
                if pair not in options or score < options[pair][0]:
                    options[pair] = score, p, q
        edges = {}
        for (source, target), (_, p, q) in options.items():
            path = tuple(reversed(self._to_seed(first, p))) + tuple(self._to_seed(second, q))
            edge_id = stable_edge_id(source, target)
            minimum = min((first.clearance.get(c, second.clearance.get(c, 0.)) for c in path))
            length = sum(math.dist(a, b) for a, b in zip(path, path[1:])) * self.resolution
            edges[edge_id] = TopologyEdge(edge_id, source, target, path, length, minimum)
        return edges

    def update(self, occupancy, resolution, origin):
        values = np.asarray(occupancy)
        if (values.ndim != 2 or not values.size or not math.isfinite(resolution)
                or resolution <= 0 or len(origin) != 2 or not all(map(math.isfinite, origin))
                or not np.isin(values, (UNKNOWN, FREE, OCCUPIED)).all()):
            raise ValueError('invalid wavefront geometry patch')
        if self.resolution is not None and not math.isclose(resolution, self.resolution, rel_tol=1e-6):
            raise ValueError('wavefront resolution cannot change within a map epoch')
        origin_cell = tuple(round(v/resolution) for v in origin)
        if any(abs(v/resolution-i) > 1e-4 for v, i in zip(origin, origin_cell)):
            raise ValueError('wavefront patches must be world-grid aligned')
        if self.resolution is None:
            self.resolution = float(resolution)
            self.tile_cells = max(2, round(self.tile_size/resolution))
        changed, dirty = 0, set()
        self.experience_dirty.clear()
        for y, x in np.argwhere(values != UNKNOWN):
            cell = (origin_cell[0]+int(x), origin_cell[1]+int(y))
            state = int(values[y, x])
            if self.occupancy.get(cell) == state:
                continue
            self.occupancy[cell] = state
            if state != FREE and cell in self.traversed_cells:
                count, _ = self.traversed_cells.pop(cell)
                self._experience_add(cell, -count, 0.0)
                self.complex_traversal_cells.discard(cell)
                self.experience_revision += 1
            changed += 1
            tx, ty = self.tile_key(cell)
            dirty.update((tx+dx, ty+dy) for dy in (-1, 0, 1) for dx in (-1, 0, 1))
        before_nodes, before_edges = {}, {}
        old_ids = set()
        for key in dirty:
            tile = self.tiles.get(key)
            if tile:
                old_ids.update(tile.components)
        for node_id in old_ids:
            before_nodes[node_id] = self.nodes[node_id]
            for edge_id in self.adjacency.get(node_id, ()):
                before_edges[edge_id] = self.edges[edge_id]
        for edge_id, edge in before_edges.items():
            self.edges.pop(edge_id)
            for endpoint in (edge.source_id, edge.target_id):
                before_nodes.setdefault(endpoint, self.nodes[endpoint])
                self.adjacency[endpoint].discard(edge_id)
        lineage, rebuilt = {}, {}
        expanded = 0
        for key in sorted(dirty):
            tile, nodes, links = self._build_tile(key, self.tiles.get(key), before_nodes)
            expanded += len(tile.owners)
            lineage.update(links)
            rebuilt.update(nodes)
            if tile.components:
                self.tiles[key] = tile
            else:
                self.tiles.pop(key, None)
        removed_nodes = old_ids - rebuilt.keys()
        for node_id in removed_nodes:
            self.nodes.pop(node_id)
            self.adjacency.pop(node_id, None)
        for node_id, node in rebuilt.items():
            self.nodes[node_id] = node
            self.adjacency.setdefault(node_id, set())
        pairs = {tuple(sorted((key, (key[0]+dx, key[1]+dy))))
                 for key in dirty for dx, dy in STEPS}
        rebuilt_edges = {}
        for a, b in sorted(pairs):
            rebuilt_edges.update(self._border_edges(a, b))
        for edge_id, edge in rebuilt_edges.items():
            self.edges[edge_id] = edge
            for endpoint in (edge.source_id, edge.target_id):
                if endpoint not in rebuilt or endpoint in old_ids:
                    before_nodes.setdefault(endpoint, self.nodes[endpoint])
                self.adjacency[endpoint].add(edge_id)
        touched = set(before_nodes) | set(rebuilt)
        for node_id in touched - removed_nodes:
            node = self.nodes[node_id]
            self.nodes[node_id] = TopologyNode(node_id, node.cell, len(self.adjacency[node_id]), node.clearance)
        if dirty:
            self.spatial_dirty.update(dirty)
            self.routing_revision += 1
            self._routing_cache = None
        return GraphUpdate(
            {k: self.nodes[k] for k in rebuilt if k not in old_ids},
            {k: self.nodes[k] for k in touched-removed_nodes if k in before_nodes and self.nodes[k] != before_nodes[k]},
            removed_nodes,
            {k: v for k, v in rebuilt_edges.items() if k not in before_edges},
            {k: v for k, v in rebuilt_edges.items() if k in before_edges and v != before_edges[k]},
            set(before_edges)-rebuilt_edges.keys(), lineage,
            {'wavefront_changed_cells': changed, 'wavefront_dirty_tiles': len(dirty),
             'wavefront_expanded_cells': expanded, 'wavefront_cached_cells': len(self.occupancy),
             'wavefront_tiles': len(self.tiles),
             'wavefront_experience_revision': self.experience_revision,
             'wavefront_traversed_cells': len(self.traversed_cells),
             'wavefront_experience_tiles': len(self.experience_tiles),
             'wavefront_complex_traversal_cells': len(self.complex_traversal_cells)})

    def _free_hull(self, points, interior=None, max_span=None):
        """A local loop is redundant only inside fully observed free space.

        Unknown/occupied holes preserve alternate routes. The complete local
        neighbourhood, including its interior, must be interchangeable.
        """
        points = sorted(set(points))
        if len(points) < 3:
            return False
        if math.hypot(points[-1][0]-points[0][0],
                max(p[1] for p in points)-min(p[1] for p in points))*self.resolution > (4*self.tile_size if max_span is None else max_span):
            return False
        def cross(a, b, c):
            return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        lower, upper = [], []
        for sequence, half in ((points, lower), (reversed(points), upper)):
            for p in sequence:
                while len(half) >= 2 and cross(half[-2], half[-1], p) <= 0:
                    half.pop()
                half.append(p)
        hull = lower[:-1]+upper[:-1]
        if len(hull) < 3:
            return False
        sides = tuple(zip(hull, hull[1:]+hull[:1]))
        if interior is not None and any(cross(a,b,interior) < 0 for a,b in sides):
            return False
        # The bounded hull can contain thousands of cells. Vectorize the
        # half-plane tests; occupancy checks remain exact, including boundary.
        yy,xx=np.mgrid[min(p[1] for p in hull):max(p[1] for p in hull)+1,
                      min(p[0] for p in hull):max(p[0] for p in hull)+1]
        inside=np.ones(xx.shape,dtype=bool)
        for a,b in sides:
            inside &= (b[0]-a[0])*(yy-a[1])-(b[1]-a[1])*(xx-a[0]) >= 0
        return all(self.occupancy.get((int(x),int(y))) == FREE
                   for x,y in zip(xx[inside],yy[inside]))

    def _free_segment(self, a, b):
        cells = tuple(bresenham(a, b))
        if any(self.occupancy.get(c) != FREE for c in cells):
            return None
        if any(p[0] != q[0] and p[1] != q[1] and (
                self.occupancy.get((p[0],q[1])) != FREE or
                self.occupancy.get((q[0],p[1])) != FREE) for p,q in zip(cells,cells[1:])):
            return None
        return cells

    def routing_view(self):
        """Publish spatial decisions and support domains from dirty geometry.

        Calculation tiles stay internal. Support-domain attachment replaces
        the old fixed-radius coverage mesh; odometry cannot pin topology.
        """
        if self._routing_cache is None:
            self._routing_cache = self.spatial_view.update(self, self.spatial_dirty)
            self.spatial_dirty.clear()
        return self._routing_cache

    def local_view(self, shape, origin):
        x0, y0 = (round(v/self.resolution) for v in origin)
        h, w = shape
        def inside(p):
            return x0 <= p[0] < x0+w and y0 <= p[1] < y0+h
        def local(p):
            return p[0]-x0, p[1]-y0
        nodes = {k: TopologyNode(k, local(n.cell), n.degree, n.clearance)
                 for k, n in self.nodes.items() if inside(n.cell)}
        edges = {k: TopologyEdge(k, e.source_id, e.target_id, tuple(map(local, e.cells)), e.length, e.minimum_clearance)
                 for k, e in self.edges.items() if e.source_id in nodes and e.target_id in nodes
                 and all(map(inside, e.cells))}
        return TopologyGraph(nodes, edges, {c for e in edges.values() for c in e.cells})
