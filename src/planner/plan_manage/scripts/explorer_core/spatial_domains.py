"""Incremental spatial domains derived from the one fine wavefront graph.

Tiles are calculation units, not public landmarks. Convex observed-free tile
unions may share an anchor. Geometry junctions, necks and route alternatives
remain distinct. No odometry/IMU or information-task identity enters this view.
"""
from collections import defaultdict
from dataclasses import dataclass, replace
import heapq
import math

from .grid import FREE, OCCUPIED
from .topology import TopologyNode, TopologyEdge, TopologyGraph, stable_edge_id


def convex_hull(points):
    points = sorted(set(points))
    if len(points) < 3:
        return tuple(points)
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    halves = []
    for sequence in (points, reversed(points)):
        half = []
        for p in sequence:
            while len(half) > 1 and cross(half[-2], half[-1], p) <= 0:
                half.pop()
            half.append(p)
        halves.append(half)
    return tuple(halves[0][:-1]+halves[1][:-1])


def lattice_count(hull):
    """Exact integer-cell count in a convex hull (Pick's theorem)."""
    if len(hull) < 2:
        return len(hull)
    if len(hull) == 2:
        return math.gcd(abs(hull[1][0]-hull[0][0]), abs(hull[1][1]-hull[0][1]))+1
    pairs = tuple(zip(hull, hull[1:]+hull[:1]))
    area2 = abs(sum(a[0]*b[1]-a[1]*b[0] for a, b in pairs))
    boundary = sum(math.gcd(abs(a[0]-b[0]), abs(a[1]-b[1])) for a, b in pairs)
    return (area2+boundary)//2+1


def row_spans(cells):
    rows = defaultdict(list)
    for x, y in cells:
        rows[y].append(x)
    result = []
    for y in sorted(rows):
        xs = sorted(rows[y]); first = last = xs[0]
        for x in xs[1:]:
            if x != last+1:
                result.append((y, first, last)); first = x
            last = x
        result.append((y, first, last))
    return tuple(result)


def span_rectangles(rows):
    """Lossless vertical run-length encoding of equal FREE row intervals."""
    grouped = defaultdict(set)
    for y, x0, x1 in rows:
        grouped[y].add((x0, x1))
    active, result, previous_y = {}, [], None
    for y in sorted(grouped):
        spans = grouped[y]
        for span in tuple(active):
            if span not in spans or previous_y != y-1:
                y0, y1 = active.pop(span)
                result.append((span[0], y0, span[1]+1, y1+1))
        for span in spans:
            active[span] = (active[span][0], y) if span in active else (y, y)
        previous_y = y
    for span, (y0, y1) in active.items():
        result.append((span[0], y0, span[1]+1, y1+1))
    return tuple(sorted(result))


@dataclass(frozen=True)
class SpaceDomain:
    domain_id: int
    place_id: int
    members: frozenset
    cells: frozenset
    hull: tuple
    anchor: tuple
    reason: str
    rows: tuple
    rectangles: tuple


class SpatialDomainView:
    """A cached quotient of the existing fine graph, not another occupancy map."""
    def __init__(self, max_span=20.):
        self.max_span = float(max_span)
        self.domains = {}
        self.owners = {}
        self.used_ids = set()
        self.place_used_ids = set()
        self.place_birth = {}
        self.birth = {}
        self.edges = {}
        self.edge_dependencies = {}
        self._pruned_edges = {}
        self._neck_cache = {}
        self._support_edges = {}
        self._connector_trees = {}
        self.revision = 0
        self.floor_revision = 0
        self.metrics = {}
        self.graph = TopologyGraph({}, {}, set())

    @staticmethod
    def _cells(engine, members):
        return frozenset(c for k in members
                        for c in engine.tiles[engine.tile_key(engine.nodes[k].cell)].components[k])

    def _new_id(self, members):
        for candidate in sorted(members):
            if candidate not in self.used_ids:
                break
        else:
            candidate = 0xffffffffffffffff
            while candidate in self.used_ids:
                candidate -= 1
        self.used_ids.add(candidate)
        self.birth[candidate] = len(self.birth)
        return candidate

    def _new_place_id(self, preferred):
        candidate = preferred
        if candidate in self.place_used_ids:
            candidate = 0xffffffffffffffff
            while candidate in self.place_used_ids:
                candidate -= 1
        self.place_used_ids.add(candidate)
        self.place_birth[candidate] = len(self.place_birth)
        return candidate

    def _protected(self, engine, members):
        reasons = {}
        for key in members:
            node = engine.nodes[key]
            edges = [engine.edges[k] for k in engine.adjacency[key]]
            if any(self._neck_cache[e.edge_id][1] for e in edges):
                reasons[key] = 'neck'
                continue
            if len(edges) <= 1:
                reasons[key] = 'endpoint' if edges else 'coverage'
                continue
            neighbours = [e.target_id if e.source_id == key else e.source_id for e in edges]
            # A grid degree of three is not a physical junction. Check the
            # actual surrounding free hull; an unknown/occupied wedge matters.
            if len(edges) >= 3:
                points = [node.cell]+[engine.nodes[k].cell for k in neighbours]
                points += [p for e in edges for p in e.cells]
                if not engine._free_hull(points):
                    reasons[key] = 'junction'
                    continue
        return reasons

    @staticmethod
    def _measured_neck(engine, edge):
        """Relative contraction bounded by measured walls on BOTH sides.

        Unknown and the EDT halo cannot establish a physical separator. Test
        only narrow path samples; a cross-section must close within one tile
        on either side. This is a geometric partition, not a body-size gate.
        """
        def context(node_id):
            neighbours = [engine.nodes[node_id].clearance]
            for eid in engine.adjacency[node_id]:
                other = engine.edges[eid]
                neighbour = other.target_id if other.source_id == node_id else other.source_id
                neighbours.append(engine.nodes[neighbour].clearance)
            return max(neighbours)
        width_limit = 1.2*min(context(edge.source_id), context(edge.target_id))
        for a, p, b in zip(edge.cells, edge.cells[1:], edge.cells[2:]):
            tile = engine.tiles[engine.tile_key(p)]
            if (tile.clearance[p]*2 >= width_limit or
                    tile.clearance_sources[p] != 1):
                continue
            normal = (0, 1) if abs(b[0]-a[0]) >= abs(b[1]-a[1]) else (1, 0)
            distances, walls = [], []
            for sign in (-1, 1):
                for step in range(1, engine.tile_cells+1):
                    q = p[0]+sign*step*normal[0], p[1]+sign*step*normal[1]
                    state = engine.occupancy.get(q)
                    if state != FREE:
                        if state == OCCUPIED:
                            distances.append(step)
                            walls.append(q)
                        break
            if len(distances) == 2 and (sum(distances)-1)*engine.resolution < width_limit:
                return dict(sample=p, walls=walls,
                            width=(sum(distances)-1)*engine.resolution,
                            neighbour_width_reference=width_limit/.6,
                            contraction_ratio=.6)
        return None

    def update(self, engine, dirty_tiles):
        if not dirty_tiles and self.revision:
            return self.graph
        # Revisit only domains touched by the dirty tile halo and their
        # immediate domain neighbours. Unrelated domains retain their objects.
        touched = {k for tile in dirty_tiles for k in
                   engine.tiles.get(tile, _EMPTY_TILE).components}
        affected = {self.owners[k] for k in touched if k in self.owners}
        affected.update(d for k, d in self.owners.items() if k not in engine.nodes)
        for d in tuple(affected):
            for k in self.domains[d].members & engine.nodes.keys():
                for edge_id in engine.adjacency[k]:
                    edge = engine.edges[edge_id]
                    for n in (edge.source_id, edge.target_id):
                        if n in self.owners:
                            affected.add(self.owners[n])
        old = {d: self.domains[d] for d in affected}
        floor_before = {d: (v.place_id, v.cells) for d, v in self.domains.items()}
        for eid in self._neck_cache.keys()-engine.edges.keys():
            del self._neck_cache[eid]
        for eid, edge in engine.edges.items():
            # Width probes and one-hop context stay inside this dependency halo.
            endpoints = [engine.tile_key(engine.nodes[k].cell) for k in (edge.source_id, edge.target_id)]
            dependencies = {(x+dx, y+dy) for x, y in endpoints
                            for dx in (-1, 0, 1) for dy in (-1, 0, 1)}
            cached = self._neck_cache.get(eid)
            if cached is None or cached[0] != edge or dependencies & dirty_tiles:
                self._neck_cache[eid] = edge, self._measured_neck(engine, edge)
        reconsider = set()
        for eid, (edge, dependencies, alternative_domains) in list(self._pruned_edges.items()):
            if dependencies & dirty_tiles or alternative_domains & affected:
                del self._pruned_edges[eid]
                if edge.source_id not in affected and edge.target_id not in affected:
                    self.edges[eid] = edge
                    self.edge_dependencies[eid] = dependencies
                    reconsider.add(eid)
        members = set(touched)
        for domain in old.values():
            members.update(domain.members & engine.nodes.keys())
        if not self.revision:
            members.update(engine.nodes)
        protected = self._protected(engine, members)
        neck_neighbours = defaultdict(set)
        for eid, (edge, evidence) in self._neck_cache.items():
            if evidence:
                neck_neighbours[edge.source_id].add(edge.target_id)
                neck_neighbours[edge.target_id].add(edge.source_id)
        # A measured neck prevents merging ACROSS that interface. It does not
        # require keeping both adjacent calculation nodes as public landmarks.
        fixed = {k for k, reason in protected.items() if reason != 'neck'}
        groups, cell_sets, hulls = {}, {}, {}
        next_group = 0
        for domain in old.values():
            surviving = domain.members & members
            # Preserve an unchanged domain and its hue on new observations.
            # Destructive geometry or a newly discovered structural decision
            # repartitions only this domain; largest overlap inherits its ID.
            cells = self._cells(engine, surviving)
            keep = (surviving and domain.cells <= cells and
                    (not (surviving & protected.keys()) or len(surviving) == 1))
            parts = [surviving] if keep else [frozenset((k,)) for k in surviving]
            for part in parts:
                groups[next_group] = set(part); next_group += 1
        claimed = set().union(*groups.values()) if groups else set()
        for k in sorted(members-claimed):
            groups[next_group] = {k}; next_group += 1
        owner = {k: d for d, group in groups.items() for k in group}
        for d, group in groups.items():
            cell_sets[d] = self._cells(engine, group)
            hulls[d] = convex_hull(cell_sets[d])
        neighbours = {d: set() for d in groups}
        for k in members:
            for edge_id in engine.adjacency[k]:
                e = engine.edges[edge_id]
                a, b = owner.get(e.source_id), owner.get(e.target_id)
                if a is not None and b is not None and a != b:
                    neighbours[a].add(b); neighbours[b].add(a)
        queue = [(min(groups[a]), min(groups[b]), a, b) for a in groups
                 for b in neighbours[a] if a < b]
        heapq.heapify(queue)
        merge_checks = 0
        while queue:
            _, _, a, b = heapq.heappop(queue)
            if a not in groups or b not in groups or b not in neighbours[a]:
                continue
            if (groups[a] | groups[b]) & fixed:
                continue
            if any(neck_neighbours[k] & groups[b] for k in groups[a]):
                continue
            hull = convex_hull(hulls[a]+hulls[b]); merge_checks += 1
            if (max(p[0] for p in hull)-min(p[0] for p in hull))*engine.resolution > self.max_span or \
               (max(p[1] for p in hull)-min(p[1] for p in hull))*engine.resolution > self.max_span:
                continue
            # Equality proves every integer cell in the union hull already
            # belongs to these FREE components. No wall/unknown is filled.
            if lattice_count(hull) != len(cell_sets[a])+len(cell_sets[b]):
                continue
            # The wire edge identity is an endpoint pair. Do not contract a
            # real loop into two parallel edges that this protocol cannot name.
            if any(not engine._free_hull(hull+hulls[n], max_span=2*self.max_span)
                   for n in neighbours[a] & neighbours[b]):
                continue
            groups[a].update(groups.pop(b))
            cell_sets[a] = cell_sets[a] | cell_sets.pop(b)
            hulls[a] = hull; del hulls[b]
            for n in neighbours[b]:
                neighbours[n].discard(b)
                if n != a:
                    neighbours[n].add(a)
            neighbours[a] = (neighbours[a] | neighbours.pop(b))-{a, b}
            for n in neighbours[a]:
                heapq.heappush(queue, (min(groups[a]), min(groups[n]), a, n))
        # Largest geometric overlap wins identity. Mere frontier motion never
        # participates. Overlap wins; equal overlap prefers the older identity.
        matches = sorted((-len(cells & domain.cells), self.birth[d], d, g)
                         for g, cells in cell_sets.items() for d, domain in old.items()
                         if cells & domain.cells)
        assigned, used = {}, set()
        for _, _, d, g in matches:
            if g not in assigned and d not in used:
                assigned[g] = d; used.add(d)
        for d in old:
            del self.domains[d]
            for k in old[d].members:
                self.owners.pop(k, None)
        changed_domains = set(old)
        for g in sorted(groups):
            group, cells = groups[g], cell_sets[g]
            d = assigned.get(g)
            if d is None:
                d = self._new_id(group)
            prior = old.get(d)
            # Keep the existing world anchor while it remains supported.
            anchor = prior.anchor if prior and prior.anchor in cells else engine.nodes[max(
                group, key=lambda k: (engine.nodes[k].clearance, -k))].cell
            reason = next((protected[k] for k in sorted(group) if k in protected),
                          'coverage' if len(groups) == 1 else 'interface')
            rows = row_spans(cells)
            overlapping = [v for v in old.values() if cells & v.cells]
            if overlapping:
                inherited = max(overlapping, key=lambda v: (len(cells & v.cells), -self.birth[v.domain_id]))
                place_id = inherited.place_id
            else:
                place_id = self._new_place_id(d)
            domain = SpaceDomain(d, place_id, frozenset(group), cells, hulls[g], anchor,
                                 reason, rows, span_rectangles(rows))
            self.domains[d] = prior if prior == domain else domain
            self.owners.update((k, d) for k in group)
            changed_domains.add(d)
        # Replace only incident coarse edges. Their paths retain exact fine
        # witnesses; no global A* or trajectory feasibility search is involved.
        for edge_id, (edge, _, _) in list(self._pruned_edges.items()):
            if edge.source_id in changed_domains or edge.target_id in changed_domains:
                del self._pruned_edges[edge_id]
        for edge_id, edge in list(self.edges.items()):
            if edge.source_id in changed_domains or edge.target_id in changed_domains:
                del self.edges[edge_id]; self.edge_dependencies.pop(edge_id, None)
        fine_edges = {eid for d in changed_domains if d in self.domains
                      for k in self.domains[d].members for eid in engine.adjacency[k]}
        trees = {}
        for edge_id in sorted(fine_edges):
            edge = engine.edges[edge_id]
            a, b = self.owners[edge.source_id], self.owners[edge.target_id]
            if a == b:
                continue
            left = self._path(engine, self.domains[a], edge.source_id, trees)
            right = self._path(engine, self.domains[b], edge.target_id, trees)
            path = left+edge.cells[1:]+tuple(reversed(right))[1:]
            length = sum(math.dist(p, q) for p, q in zip(path, path[1:]))*engine.resolution
            clearance = min(engine.tiles[engine.tile_key(p)].clearance[p] for p in path)
            candidate = TopologyEdge(stable_edge_id(a, b), a, b, path, length, clearance)
            previous = self.edges.get(candidate.edge_id)
            if previous is None or (candidate.length, -candidate.minimum_clearance, candidate.cells) < \
                    (previous.length, -previous.minimum_clearance, previous.cells):
                self.edges[candidate.edge_id] = candidate
                self.edge_dependencies[candidate.edge_id] = frozenset(engine.tile_key(p) for p in path)
        self._assign_places(engine, floor_before, affected)
        self._prune_redundant_edges(engine, changed_domains, reconsider)
        if floor_before != {d: (v.place_id, v.cells) for d, v in self.domains.items()}:
            self.floor_revision += 1
        degrees = defaultdict(int)
        for edge in self.edges.values():
            degrees[edge.source_id] += 1; degrees[edge.target_id] += 1
        nodes = {d: TopologyNode(d, v.anchor, degrees[d],
                 engine.tiles[engine.tile_key(v.anchor)].clearance[v.anchor]) for d, v in self.domains.items()}
        nodes, route_edges, absorbed = self._compress_connections(nodes)
        published_edges = {}
        self._support_edges = {}
        for eid, edge in route_edges.items():
            required = {self.domains[d].anchor for d in absorbed[eid]}
            compact = [edge.cells[0]]
            for previous, current, following in zip(edge.cells, edge.cells[1:], edge.cells[2:]):
                if ((current[0]-previous[0], current[1]-previous[1]) !=
                        (following[0]-current[0], following[1]-current[1]) or
                        math.dist(compact[-1], following)*engine.resolution > .5 or current in required):
                    compact.append(current)
            compact.append(edge.cells[-1])
            published_edges[eid] = TopologyEdge(eid, edge.source_id, edge.target_id,
                tuple(compact), edge.length, edge.minimum_clearance)
            positions = {p: i for i, p in enumerate(compact) if p in required}
            for d in absorbed[eid]:
                self._support_edges[d] = (eid, positions[self.domains[d].anchor])
        self.revision += 1
        self._connector_trees = {}
        for d, domain in self.domains.items():
            if len(domain.members) != 1 or lattice_count(domain.hull) == len(domain.cells):
                continue
            # A single tile component can itself bend around an obstacle.
            # Reuse its existing parent tree instead of exporting a dense map
            # or making every future attachment run A*. One byte per tile cell.
            tile = engine.tiles[engine.tile_key(domain.anchor)]
            size = engine.tile_cells
            x0, y0 = (v*size for v in engine.tile_key(domain.anchor))
            steps = ['.']*(size*size)
            for x, y in domain.cells:
                px, py = tile.parents[(x, y)]
                steps[(y-y0)*size+x-x0] = str((py-y+1)*3+(px-x+1))
            self._connector_trees[d] = dict(origin=(x0, y0), width=size, steps=''.join(steps))
        payload = self.payload(engine.resolution)
        payload['geometry_revision'] = engine.routing_revision
        payload['measured_necks'] = [dict(evidence, fine_edge=eid,
            source_domain=self.owners[edge.source_id], target_domain=self.owners[edge.target_id])
            for eid, (edge, evidence) in sorted(self._neck_cache.items()) if evidence]
        payload['clearance'] = dict(
            definition='sampled_2d_nonfree_cell_area_lower_bound',
            projection='local_uninflated_body_space',
            cell_error=engine.resolution/math.sqrt(2.),
            field_halo_tiles=1, geometry_revision=engine.routing_revision,
            nodes={str(k): engine.clearance_evidence(n.cell) for k, n in nodes.items()},
            edges={str(k): self._edge_evidence(engine, e) for k, e in route_edges.items()})
        self.graph = TopologyGraph(nodes, published_edges, set(), payload)
        self.metrics = dict(spatial_domains=len(self.domains), spatial_route_nodes=len(nodes),
                            spatial_route_edges=len(published_edges),
                            spatial_places=len({v.place_id for v in self.domains.values()}), spatial_rebuilt_domains=len(changed_domains),
                            spatial_touched_fine_nodes=len(members), spatial_merge_checks=merge_checks,
                            spatial_rebuilt_fine_edges=len(fine_edges), spatial_revision=self.revision,
                            spatial_compressed_nodes=len(self.domains)-len(nodes),
                            spatial_measured_neck_edges=sum(bool(v[1]) for v in self._neck_cache.values()))
        return self.graph

    @staticmethod
    def _edge_evidence(engine, edge):
        samples = [p for p in edge.cells if abs(
            engine.tiles[engine.tile_key(p)].clearance[p]-edge.minimum_clearance) < 1e-9]
        return dict(radius=edge.minimum_clearance, sample=samples[0],
                    sources=sorted({s for p in samples for s in engine.clearance_evidence(p)['sources']}),
                    support_tiles=sorted({engine.tile_key(p) for p in edge.cells}),
                    clearance_tiles=sorted({(x+dx, y+dy)
                        for x, y in {engine.tile_key(p) for p in edge.cells}
                        for dx in (-1, 0, 1) for dy in (-1, 0, 1)}))

    def _assign_places(self, engine, previous, affected):
        """Partition observed interfaces, separately from convex route supports.

        The graph uses certified FREE boundary crossings. Only a contraction
        closed by measured walls cuts an interface. Obstacles remain holes;
        neither a tile boundary nor a non-convex hull creates a new place.
        Largest surviving overlap inherits the old identity, one component per
        identity. Reconciliation also handles a new doorway in an old place.
        """
        adjacency = defaultdict(set)
        for eid, edge in engine.edges.items():
            a, b = self.owners[edge.source_id], self.owners[edge.target_id]
            if a != b and not self._neck_cache[eid][1]:
                adjacency[a].add(b); adjacency[b].add(a)
        remaining, components = set(self.domains), []
        while remaining:
            first = min(remaining); remaining.remove(first)
            group, queue = {first}, [first]
            while queue:
                for nxt in adjacency[queue.pop()] & remaining:
                    remaining.remove(nxt); group.add(nxt); queue.append(nxt)
            components.append(group)
        # Existing exact support sets are reused, not rasterized a second time.
        matches = []
        for index, group in enumerate(components):
            votes = defaultdict(int)
            for d in group:
                cells = self.domains[d].cells
                if d in previous and cells is previous[d][1]:
                    votes[previous[d][0]] += len(cells)
                    continue
                for old in affected:
                    pid, old_cells = previous[old]
                    votes[pid] += len(cells & old_cells)
            matches.extend((-overlap, self.place_birth[pid], pid, index)
                           for pid, overlap in votes.items() if overlap)
        assigned, claimed = {}, set()
        for _, _, pid, index in sorted(matches):
            if index not in assigned and pid not in claimed:
                assigned[index] = pid; claimed.add(pid)
        for index, group in enumerate(components):
            pid = assigned.get(index)
            if pid is None:
                pid = min((self.domains[d].place_id for d in group if
                           self.domains[d].place_id not in claimed),
                          key=lambda k: self.place_birth[k], default=None)
                if pid is None:
                    pid = self._new_place_id(min(group))
                claimed.add(pid)
            for d in group:
                if self.domains[d].place_id != pid:
                    self.domains[d] = replace(self.domains[d], place_id=pid)

    def _compress_connections(self, nodes):
        """Move degree-two geometry into edges without losing support access.

        Keep a third anchor when removing it would collapse a real loop into
        parallel endpoint-pair IDs. Each removed domain attaches to an exact
        vertex of the resulting edge; query clients need no dense rescue.
        """
        nodes, edges = dict(nodes), dict(self.edges)
        absorbed = {eid: set() for eid in edges}
        adjacency = defaultdict(set)
        for eid, e in edges.items():
            adjacency[e.source_id].add(eid); adjacency[e.target_id].add(eid)
        queue = list(nodes); heapq.heapify(queue)
        while queue:
            node = heapq.heappop(queue)
            if node not in nodes or len(adjacency[node]) != 2:
                continue
            first, second = (edges[k] for k in sorted(adjacency[node]))
            a = first.target_id if first.source_id == node else first.source_id
            b = second.target_id if second.source_id == node else second.source_id
            eid = stable_edge_id(a, b)
            if a == b or eid in edges:
                continue
            left = first.cells if first.target_id == node else tuple(reversed(first.cells))
            right = second.cells if second.source_id == node else tuple(reversed(second.cells))
            edges[eid] = TopologyEdge(eid, a, b, left+right[1:],
                first.length+second.length, min(first.minimum_clearance, second.minimum_clearance))
            absorbed[eid] = absorbed.pop(first.edge_id) | absorbed.pop(second.edge_id) | {node}
            for edge in (first, second):
                del edges[edge.edge_id]
                adjacency[edge.source_id].remove(edge.edge_id)
                adjacency[edge.target_id].remove(edge.edge_id)
            del nodes[node]
            adjacency[a].add(eid); adjacency[b].add(eid)
            heapq.heappush(queue, a); heapq.heappush(queue, b)
        nodes = {k: replace(n, degree=len(adjacency[k])) for k, n in nodes.items()}
        return nodes, edges, absorbed

    def _prune_redundant_edges(self, engine, changed, reconsider):
        """Keep real alternatives, remove local free-space shortcut cliques."""
        adjacency = defaultdict(set)
        for eid, e in self.edges.items():
            adjacency[e.source_id].add(eid); adjacency[e.target_id].add(eid)
        candidates = [eid for eid, e in self.edges.items()
                      if e.source_id in changed or e.target_id in changed or eid in reconsider]
        for eid in sorted(candidates, key=lambda k: (-self.edges[k].length, k)):
            edge = self.edges[eid]
            costs, queue, alternative = {edge.source_id: 0.}, [(0., edge.source_id, ())], None
            while queue:
                cost, node, trail = heapq.heappop(queue)
                if cost != costs[node]:
                    continue
                if node == edge.target_id:
                    alternative = trail; break
                for nxt_id in sorted(adjacency[node]-{eid}):
                    nxt = self.edges[nxt_id]
                    if nxt.minimum_clearance+1e-8 < edge.minimum_clearance:
                        continue
                    target = nxt.target_id if nxt.source_id == node else nxt.source_id
                    new_cost = cost+nxt.length
                    if new_cost <= edge.length*1.15 and new_cost < costs.get(target, math.inf):
                        costs[target] = new_cost
                        heapq.heappush(queue, (new_cost, target, trail+(nxt_id,)))
            if alternative and engine._free_hull(
                    list(edge.cells)+[p for k in alternative for p in self.edges[k].cells],
                    max_span=2*self.max_span):
                witness = list(edge.cells)+[p for k in alternative for p in self.edges[k].cells]
                x0, y0 = engine.tile_key((min(p[0] for p in witness), min(p[1] for p in witness)))
                x1, y1 = engine.tile_key((max(p[0] for p in witness), max(p[1] for p in witness)))
                dependencies = frozenset((x, y) for x in range(x0, x1+1) for y in range(y0, y1+1))
                alternative_domains = frozenset(n for k in alternative
                    for n in (self.edges[k].source_id, self.edges[k].target_id))
                self._pruned_edges[eid] = edge, dependencies, alternative_domains
                del self.edges[eid]; self.edge_dependencies.pop(eid, None)
                adjacency[edge.source_id].remove(eid); adjacency[edge.target_id].remove(eid)

    @staticmethod
    def _path(engine, domain, target, trees):
        direct = engine._free_segment(domain.anchor, engine.nodes[target].cell)
        if direct is not None:
            return direct
        # Rare non-convex single-tile components retain the original wavefront
        # witness. Other cases use only this domain's small fine graph.
        if domain.domain_id not in trees:
            tile = engine.tiles[engine.tile_key(domain.anchor)]
            root = tile.owners[domain.anchor]
            prefix = tuple(engine._to_seed(tile, domain.anchor))
            costs, parent, queue = {root: 0.}, {}, [(0., root)]
            while queue:
                cost, k = heapq.heappop(queue)
                if cost != costs[k]:
                    continue
                for eid in engine.adjacency[k]:
                    e = engine.edges[eid]; nxt = e.target_id if e.source_id == k else e.source_id
                    if nxt not in domain.members or cost+e.length >= costs.get(nxt, math.inf):
                        continue
                    costs[nxt] = cost+e.length; parent[nxt] = (k, eid)
                    heapq.heappush(queue, (costs[nxt], nxt))
            trees[domain.domain_id] = prefix, parent
        prefix, parent = trees[domain.domain_id]
        parts, current = [], target
        while current in parent:
            previous, eid = parent[current]; edge = engine.edges[eid]
            parts.append(edge.cells if edge.source_id == previous else tuple(reversed(edge.cells)))
            current = previous
        path = prefix
        for part in reversed(parts):
            path += part[1:]
        return path

    def payload(self, resolution):
        return {'resolution': resolution, 'spatial_revision': self.revision, 'domains': [
            {'id': d, 'place_id': v.place_id, 'anchor': [(v.anchor[0]+.5)*resolution, (v.anchor[1]+.5)*resolution],
             'rows': v.rows, 'reason': v.reason,
             **({'connector_tree': self._connector_trees[d]} if d in self._connector_trees else {}),
             **({'edge_attachment': self._support_edges[d]} if d in self._support_edges else {})}
            for d, v in sorted(self.domains.items())]}


class _EmptyTile:
    components = {}
_EMPTY_TILE = _EmptyTile()
