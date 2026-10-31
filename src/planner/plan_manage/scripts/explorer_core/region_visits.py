"""Bounded 1+2 Region visiting; no motion authority or second map.

The full sequence uses cheap spatial summaries. Only its three-entry window
produces stations. Route capacity is INCOMPLETE, never geometric rejection.
"""
from collections import OrderedDict
from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.ndimage import binary_propagation

from .grid import FREE, UNKNOWN, bresenham
from .frontier_regions import observation_hypothesis_cells
from .path_planning import (
    ExpansionBudget, KnownFreeSearch, RouteSearchResult, RouteSearchStatus,
    adjacent_grid_path_is_valid, path_length_cells, path_turn_cost,
)


@dataclass
class VisitDemand:
    """Bounded selection reference, not another task/completion ledger.

    Targets freeze when the alternatives are first compared. A later boundary
    can supply another view of these targets, without claiming Frontier lineage.
    """
    demand_id: int
    region_id: int
    frontier: tuple
    targets: frozenset


def same_visit_demand(a, b):
    if a is None or b is None or a.region_id != b.region_id:
        return False
    left = getattr(a, 'visit_demand_id', None)
    right = getattr(b, 'visit_demand_id', None)
    if left is not None or right is not None:
        return left is not None and left == right
    return a.frontier_cell == b.frontier_cell


def visit_demand_key(candidate):
    identity = getattr(candidate, 'visit_demand_id', None)
    return candidate.frontier_cell if identity is None else identity


def station_alternatives(cells, limit):
    """Keep the safest first choice and spatially distinct bounded siblings."""
    cells = list(dict.fromkeys(cells))
    if not cells or limit <= 0:
        return []
    selected = [cells.pop(0)]
    while cells and len(selected) < limit:
        cell = max(cells, key=lambda c: min(math.dist(c, old) for old in selected))
        selected.append(cell)
        cells.remove(cell)
    return selected


class RegionVisitWindow:
    """One retained coarse order; changing scores do not reorder every tick."""
    width = 3  # one current Region, two lookahead Regions

    def __init__(self):
        self.order = []
        self.scope_serial = 0
        self.committed_region = None
        self.committed_scope = None
        self.deferred = {}
        self.prefix = ()
        self.prefix_region = None
        self.sample_cursor = {}
        self.demands = {}
        self.demand_serial = 0
        self.demand_prefix = ()
        self.sampled_demands = {}

    def reconcile(self, regions, start, revision):
        by_id = {r.region_id: r for r in regions}
        self.order = [rid for rid in self.order if rid in by_id]
        self.sample_cursor = {rid: cursor for rid,cursor in self.sample_cursor.items() if rid in by_id}
        self.demands = {rid: pool for rid, pool in self.demands.items() if rid in by_id}
        self.sampled_demands = {}
        self.deferred = {rid: rev for rid, rev in self.deferred.items()
                         if rid in by_id and rev == revision}
        for rid in sorted(set(by_id)-set(self.order)):
            center = by_id[rid].centroid
            # Insert once using a distance LOWER BOUND, not an invented route.
            # Existing relative order survives ordinary frontier/map updates.
            choices = []
            for i in range(len(self.order)+1):
                before = start if i == 0 else by_id[self.order[i-1]].centroid
                after = None if i == len(self.order) else by_id[self.order[i]].centroid
                extra = math.dist(before, center)
                if after is not None:
                    extra += math.dist(center, after)-math.dist(before, after)
                choices.append((extra, i))
            i = min(choices)[1]
            # A newly discovered Region cannot displace the committed current.
            if self.committed_region in self.order:
                i = max(1, i)
            self.order.insert(i, rid)
        if self.committed_region in self.order:
            self.order.remove(self.committed_region)
            self.order.insert(0, self.committed_region)
        ready = [rid for rid in self.order if rid not in self.deferred]
        return [by_id[rid] for rid in ready[:self.width]]

    def sample_demands(self, region, choices, grid, limit, radius, task=None):
        """Keep a small demand batch; obtain views from this Region only.

        Reprojection tests visibility of fixed target facts, not overlap/IoU
        between moving Frontier shapes. The actual stations are checked later.
        """
        rid = region.region_id
        valid = set().union(*(set(c) for c in region.clusters))
        pool = self.demands.setdefault(rid, OrderedDict())
        active_id = getattr(task, 'visit_demand_id', None) if task is not None and task.region_id == rid else None
        if active_id is not None and not task.information_satisfied:
            # Reuse the existing task's frozen domain and measurement baseline;
            # selection never edits them or manufactures task completion.
            pool[active_id] = VisitDemand(active_id, rid, task.frontier_cell,
                                         frozenset(task.target_cells))
            self.demand_serial = max(self.demand_serial, active_id)
        elif active_id is not None:
            pool.pop(active_id, None)
        order = list(dict.fromkeys(
            ([active_id] if active_id in pool else [])
            + list(self.demand_prefix if self.prefix_region == rid else ())
            + list(pool)))
        chosen = []
        used = set()
        kept = OrderedDict()
        for identity in order:
            demand = pool.get(identity)
            if demand is None:
                continue
            remaining = sorted(c for c in demand.targets if grid.value(c) == UNKNOWN)
            if not remaining:
                continue
            frontier = demand.frontier if demand.frontier in valid else None
            if frontier is None:
                # At most eight target representatives and eight candidate
                # boundaries. This does not enumerate stand-off poses globally.
                stride = max(1, math.ceil(len(remaining)/8))
                representatives = set(remaining[::stride][:8])
                nearby = sorted(choices, key=lambda f: (
                    min(math.dist(f, c) for c in representatives), f))[:8]
                frontier = next((f for f in nearby if f not in used and
                    observation_hypothesis_cells(grid, f, representatives, radius)), None)
            if frontier is None or frontier in used:
                continue
            demand.frontier = frontier
            chosen.append(frontier)
            used.add(frontier)
            kept[identity] = demand
            self.sampled_demands[(rid, frontier)] = demand
            if len(chosen) == limit:
                break
        cursor = self.sample_cursor.get(rid, 0)
        rotated = choices[cursor % len(choices):]+choices[:cursor % len(choices)] if choices else []
        for frontier in rotated:
            if len(chosen) >= limit:
                break
            if frontier not in used:
                chosen.append(frontier)
                used.add(frontier)
        self.sample_cursor[rid] = cursor+max(0, len(chosen)-len(kept))
        # Unselected demand records are disposable scheduling caches. Their
        # unknown cells remain in the map and receive no completion/blacklist.
        self.demands[rid] = kept
        return chosen

    def freeze_demand(self, rid, frontier, targets):
        demand = self.sampled_demands.get((rid, frontier))
        if demand is None:
            self.demand_serial += 1
            demand = VisitDemand(self.demand_serial, rid, frontier, frozenset(targets))
            self.sampled_demands[(rid, frontier)] = demand
            self.demands.setdefault(rid, OrderedDict())[demand.demand_id] = demand
        return demand

    def defer(self, rid, revision):
        """No usable station this revision; retain residual work, no blacklist."""
        self.deferred[rid] = revision
        if rid in self.order:
            self.order.remove(rid)
            self.order.append(rid)
        if self.prefix_region == rid:
            self.prefix = ()
            self.demand_prefix = ()

    def scope(self, rid):
        if rid == self.committed_region and self.committed_scope is not None:
            return self.committed_scope
        # Reserved internal identity. Only commit() makes it active.
        return ('region_visit', self.scope_serial+1)

    def commit(self, rid):
        if rid != self.committed_region or self.committed_scope is None:
            self.scope_serial += 1
            self.committed_scope = ('region_visit', self.scope_serial)
            self.committed_region = rid
        if rid in self.order:
            self.order.remove(rid)
            self.order.insert(0, rid)
        return self.committed_scope


class BoundedVisitRoutes:
    """Reuse raw FREE witnesses/searches in a bounded cache, with one budget."""
    def __init__(self, max_entries=48, max_retained_expansions=20000):
        self.searches = OrderedDict()
        self.paths = OrderedDict()
        self.max_entries = max_entries
        self.max_retained_expansions = max_retained_expansions
        self.geometry = None
        self.used = 0
        self.pending = set()
        self.capacity_yields = set()
        self.cache_hits = 0
        self.estimated_transitions = 0
        self.reachable = None
        self.reachability_key = None

    def begin(self, grid, blocked, limit, revision=None):
        shape = (grid.origin_x, grid.origin_y, grid.width, grid.height, grid.resolution)
        if shape != self.geometry:
            self.searches.clear()
            self.paths.clear()
            self.geometry = shape
        self.grid, self.blocked = grid, blocked
        key = (shape, revision)
        if revision is None or key != self.reachability_key:
            self.reachable = None
            self.reachability_key = key
        self.reachability_ms = 0.
        self.budget = ExpansionBudget(limit)
        self.pending = set()
        self.capacity_yields = set()
        self.cache_hits = 0
        self.estimated_transitions = 0

    def query(self, start, goal, quota=512):
        grid, forbidden = self.grid, self.blocked
        key = (start, goal)
        path = self.paths.get(key)
        if path and adjacent_grid_path_is_valid(grid, path, set(), forbidden):
            self.paths.move_to_end(key)
            self.cache_hits += 1
            return RouteSearchResult(RouteSearchStatus.FOUND, path)
        self.paths.pop(key, None)
        # Reuse a nearby witnessed suffix after odometry moves to the next
        # grid cell. Requiring an identical start would lose rolling reuse.
        for old_key, old_path in reversed(list(self.paths.items())):
            if old_key[1] != goal:
                continue
            joined = self._join(start, old_path)
            if joined is not None:
                self.paths[key] = joined
                self.cache_hits += 1
                self._trim()
                return RouteSearchResult(RouteSearchStatus.FOUND, joined)
        direct = bresenham(start, goal)
        if direct and adjacent_grid_path_is_valid(grid, direct, set(), forbidden):
            self.paths[key] = direct
            self._trim()
            return RouteSearchResult(RouteSearchStatus.FOUND, direct)
        if grid.value(start) != FREE or grid.value(goal) != FREE:
            return RouteSearchResult(RouteSearchStatus.NO_ROUTE, reason='ENDPOINT_NOT_KNOWN_FREE')
        # A one-byte reachability index prevents an unreachable endpoint from
        # exhausting/restarting a Python search over an entire large map. It
        # contains no occupancy/terrain authority. Four-connectivity has the
        # same components as 8-connectivity without diagonal corner cutting.
        if self.reachable is None:
            began = time.perf_counter()
            seed = np.zeros(grid.data.shape, dtype=bool)
            seed[start[1],start[0]] = True
            self.reachable = binary_propagation(seed, mask=grid.data == FREE)
            self.reachability_ms += (time.perf_counter()-began)*1000.
        if bool(self.reachable[start[1],start[0]]) != bool(self.reachable[goal[1],goal[0]]):
            return RouteSearchResult(RouteSearchStatus.NO_ROUTE, reason='RAW_COMPONENT_DISCONNECTED')
        search_key = key
        search = self.searches.get(key)
        if search is None:
            for old_key, old_search in reversed(list(self.searches.items())):
                if old_key[1] != goal or math.dist(start,old_key[0])*grid.resolution > 1.:
                    continue
                connector = bresenham(start,old_key[0])
                if (adjacent_grid_path_is_valid(grid,connector,set(),forbidden)
                        and old_search.current(grid,forbidden)):
                    search_key, search = old_key, old_search
                    break
        if search is None or not search.current(grid, forbidden):
            search_key = key
            search = KnownFreeSearch(start, goal)
            self.searches[search_key] = search
        self.searches.move_to_end(search_key)
        local = ExpansionBudget(min(max(0, quota), self.budget.remaining))
        result = search.advance(grid, forbidden, local)
        self.budget.used += local.used
        if result.status == RouteSearchStatus.FOUND:
            if search_key != key:
                joined = self._join(start,result.path)
                if joined is None:
                    # Changed departure geometry requires a fresh bounded
                    # search; failure to splice does not prove no route.
                    self.searches.pop(search_key,None)
                    return RouteSearchResult(RouteSearchStatus.INCOMPLETE,
                                             reason='START_CONNECTION_PENDING')
                result = RouteSearchResult(RouteSearchStatus.FOUND,joined)
            self.paths[key] = result.path
            self.searches.pop(search_key, None)
        elif result.status == RouteSearchStatus.INCOMPLETE:
            self.pending.add(goal)
        evicted = self._trim()
        if search_key in evicted and result.status == RouteSearchStatus.INCOMPLETE:
            result.reason = 'SEARCH_CACHE_CAPACITY'
            self.capacity_yields.add(goal)
        return result

    def _join(self, start, path):
        if not adjacent_grid_path_is_valid(self.grid,path,set(),self.blocked):
            return None
        indices = sorted(range(len(path)),key=lambda i:math.dist(start,path[i]))[:4]
        options = []
        for i in indices:
            if math.dist(start,path[i])*self.grid.resolution > 1.:
                continue
            connector = bresenham(start,path[i])
            if adjacent_grid_path_is_valid(self.grid,connector,set(),self.blocked):
                joined = connector+path[i+1:]
                options.append((path_length_cells(joined,self.grid.resolution),joined))
        return min(options,key=lambda item:item[0])[1] if options else None

    def _trim(self):
        evicted = set()
        while len(self.paths) > self.max_entries:
            self.paths.popitem(last=False)
        while (len(self.searches) > self.max_entries or
               sum(len(s.closed) for s in self.searches.values()) > self.max_retained_expansions):
            key, _ = self.searches.popitem(last=False)
            evicted.add(key)
        return evicted

    def transition(self, a, b):
        if a == b:
            return 0., None
        result = self.query(a, b, quota=96)
        if result.status == RouteSearchStatus.FOUND:
            return path_length_cells(result.path, self.grid.resolution), result.path
        if result.status == RouteSearchStatus.NO_ROUTE:
            return float('inf'), None
        # A future leg may use an explicitly provisional lower bound. It is
        # NEVER published or counted as a known-free connection to the current.
        self.estimated_transitions += 1
        return math.dist(a, b)*self.grid.resolution, None


@dataclass
class JointVisitChoice:
    candidate: object
    prefix: tuple
    score: float
    continuation_estimated: bool = False
    demand_prefix: tuple = ()


def _heading(points, at_end=False):
    pairs = list(zip(points[:-1], points[1:]))
    if at_end:
        pairs.reverse()
    for a, b in pairs:
        if a != b:
            return math.atan2(b[1]-a[1], b[0]-a[0])
    return None


def _angle(a, b):
    return 0. if a is None or b is None else abs(math.remainder(b-a, 2*math.pi))


def choose_joint_visit(current, lookahead, routes, *, body_yaw=None,
                      depth=3, beam_width=8, retained_prefix=(), retained_demands=()):
    """Short joint Frontier/station visit, then a two-Region exit preview.

    The current Region owns all executable choices. Future legs are estimates;
    only candidate.path (already checked raw FREE) can be handed to Local.
    A frontier is visited once in a prefix, regardless of station alternatives.
    Expected observations are unioned, avoiding reward for counting the same
    unknown cells twice. No station yaw requirement is imposed on 360deg LiDAR.
    """
    if not current:
        return None
    transitions = {}
    def leg(a, b):
        key = (a.cell, b.cell)
        if key not in transitions:
            transitions[key] = routes.transition(*key)
        return transitions[key]

    # A retained next objective remains the next objective while it has a valid
    # station. Recalculate its station/route; do not freeze stale geometry.
    def demand(c):
        return getattr(c, 'visit_remaining_cells', c.observation_cells)
    current = [c for c in current if demand(c)]
    if not current:
        return None
    keys = {visit_demand_key(c) for c in current}
    retained = tuple(k for k in (retained_demands or retained_prefix) if k in keys)
    first = [c for c in current if not retained or visit_demand_key(c) == retained[0]]

    # prefix, expected remaining cells, length, turn, final heading
    beam = []
    for c in first:
        heading = _heading(c.path)
        beam.append(((c,), set(demand(c)), c.path_length,
                     c.turn_cost+_angle(body_yaw, heading), _heading(c.path, True)))
    complete = list(beam)
    for _ in range(1, min(depth, len(keys))):
        expanded = []
        for prefix, seen, length, turn, heading in beam:
            used = {visit_demand_key(c) for c in prefix}
            next_retained = next((k for k in retained if k not in used), None)
            for c in current:
                if (visit_demand_key(c) in used or not demand(c)-seen or
                        (next_retained is not None and visit_demand_key(c) != next_retained)):
                    continue
                distance, path = leg(prefix[-1], c)
                if not math.isfinite(distance):
                    continue
                arrival = _heading(path, True) if path else math.atan2(
                    c.cell[1]-prefix[-1].cell[1], c.cell[0]-prefix[-1].cell[0])
                entry = _heading(path) if path else arrival
                expanded.append((prefix+(c,), seen | demand(c),
                    length+distance, turn+_angle(heading, entry)+(path_turn_cost(path) if path else 0.), arrival))
        expanded.sort(key=lambda s: (-len(s[1])/(1+s[2]+.25*s[3]),
                                     tuple((c.frontier_cell,c.cell) for c in s[0])))
        beam = expanded[:beam_width]
        complete.extend(beam)
        if not beam:
            break

    best = None
    # When another independently useful demand exists, compare the access to
    # the first station AND its next visit. A one-station reward ratio would
    # otherwise defeat joint selection by winning before that leg is priced.
    joint = [state for state in complete if len(state[0]) > 1]
    if joint:
        complete = joint
    for prefix, seen, length, turn, heading in complete:
        # Visit B then C in coarse order, with only their bounded stations.
        tails = [(prefix[-1], 0., heading, False)]
        for pool in lookahead[:2]:
            if not pool:
                continue
            next_tails = []
            for previous, cost, tail_heading, estimated in tails:
                for option in pool:
                    distance, path = leg(previous, option)
                    if not math.isfinite(distance):
                        continue
                    entry = _heading(path) if path else math.atan2(
                        option.cell[1]-previous.cell[1], option.cell[0]-previous.cell[0])
                    exit_heading = _heading(path, True) if path else entry
                    next_tails.append((option, cost+distance+.25*_angle(tail_heading,entry),
                                       exit_heading, estimated or path is None))
            tails = sorted(next_tails, key=lambda s:(s[1],s[0].cell))[:beam_width]
            if not tails:
                break
        if not tails:
            continue
        tail = min(tails, key=lambda s:(s[1],s[0].cell))
        score = len(seen)/(1+length+tail[1]+.25*turn)
        key = (score, -length, tuple((-c.cell[0],-c.cell[1]) for c in prefix))
        if best is None or key > best[0]:
            best = (key, JointVisitChoice(prefix[0], tuple(c.frontier_cell for c in prefix),score,tail[3],
                                         tuple(visit_demand_key(c) for c in prefix)))
    # Uncertain/blocked preview cannot veto an independently valid current leg.
    if best is None:
        c = max(first, key=lambda c:(len(demand(c))/(1+c.path_length+.25*c.turn_cost),
                                     -c.path_length,-c.cell[0],-c.cell[1]))
        return JointVisitChoice(c,(c.frontier_cell,),0.,True,(visit_demand_key(c),))
    result = best[1]
    result.demand_prefix = tuple(dict.fromkeys(
        result.demand_prefix+tuple(k for k in retained
                                  if k not in result.demand_prefix)))[:depth]
    return result
