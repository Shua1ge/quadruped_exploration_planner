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
    adjacent_grid_path_is_valid, path_length_cells,
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
    retired: bool = False  # cache invalidation from the real task's completion


@dataclass(frozen=True)
class VisitPlan:
    """The priced current-Region prefix carried by one publication candidate.

    Future Region legs remain estimates. This snapshot grants no motion rights.
    Demand references live only as long as the bounded cache/tasks/candidates.
    """
    plan_id: int
    region_id: int
    demand_ids: tuple
    frontiers: tuple
    stations: tuple
    demands: tuple


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
        self.plan_serial = 0
        self.installed_plan = None
        self.installed_request = None
        self.events = []
        self.event_overflow = 0

    def note(self, action, rid, identity=None, **facts):
        # One decision's evidence, not a permanent failure/completion ledger.
        if len(self.events) < 64:
            self.events.append(dict(action=action, region=rid, demand=identity, **facts))
        else:
            self.event_overflow += 1

    def retain_regions(self, region_ids, reason):
        for rid, pool in self.demands.items():
            if rid not in region_ids:
                for identity in pool:
                    self.note('CACHE_RELEASED', rid, identity, reason=reason)
        self.demands = {rid: pool for rid, pool in self.demands.items() if rid in region_ids}

    def draft(self, choice, rid):
        self.plan_serial += 1
        pool = self.demands.get(rid, {})
        return VisitPlan(self.plan_serial, rid, choice.demand_prefix,
                         choice.prefix, choice.station_prefix,
                         tuple(pool[k] for k in choice.demand_prefix if k in pool))

    def install(self, plan, request):
        """Called only after the bridge matches an actual installation ACK."""
        if plan is None or request == self.installed_request:
            return False
        self.installed_plan, self.installed_request = plan, request
        self.prefix_region = plan.region_id
        live = {d.demand_id for d in plan.demands if not d.retired}
        self.demand_prefix = tuple(k for k in plan.demand_ids if k in live)
        self.prefix = tuple(f for k, f in zip(plan.demand_ids, plan.frontiers) if k in live)
        return True

    def retire(self, task):
        """Release scheduling references after measured completion, not arrival."""
        if not task.information_satisfied:
            return False
        identity = getattr(task, 'visit_demand_id', None)
        if identity is None:
            return False
        pool = self.demands.get(task.region_id, {})
        ref = getattr(task, 'visit_demand_ref', None) or pool.get(identity)
        changed = ref is not None and not ref.retired
        if ref is not None:
            ref.retired = True
        pool.pop(identity, None)
        self.sampled_demands = {key: d for key, d in self.sampled_demands.items()
                                if d.demand_id != identity}
        if self.prefix_region == task.region_id:
            self.prefix = tuple(f for k, f in zip(self.demand_prefix, self.prefix)
                                if k != identity)
            self.demand_prefix = tuple(k for k in self.demand_prefix if k != identity)
        return changed

    def retained_stations(self, rid):
        plan = self.installed_plan
        if plan is None or plan.region_id != rid:
            return ()
        return tuple((k, station) for k, station in zip(plan.demand_ids, plan.stations)
                     if k in self.demand_prefix)

    def reconcile(self, regions, start, revision):
        self.events = []
        self.event_overflow = 0
        by_id = {r.region_id: r for r in regions}
        self.order = [rid for rid in self.order if rid in by_id]
        self.sample_cursor = {rid: cursor for rid,cursor in self.sample_cursor.items() if rid in by_id}
        self.retain_regions(by_id, 'REGION_ABSENT')
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
        # Installed work outlives its disposable sampling cache. Reattach only
        # this bounded prefix, not every historical demand or a second ledger.
        if self.installed_plan is not None and self.installed_plan.region_id == rid:
            for ref in self.installed_plan.demands:
                if not ref.retired and ref.demand_id in self.demand_prefix:
                    pool.setdefault(ref.demand_id, ref)
        active_id = getattr(task, 'visit_demand_id', None) if task is not None and task.region_id == rid else None
        if active_id is not None and not task.information_satisfied:
            # Reuse the existing task's frozen domain and measurement baseline;
            # selection never edits them or manufactures task completion.
            ref = getattr(task, 'visit_demand_ref', None) or pool.get(active_id)
            if ref is None:
                ref = VisitDemand(active_id, rid, task.frontier_cell, frozenset(task.target_cells))
            if not ref.retired:
                pool[active_id] = ref
            self.demand_serial = max(self.demand_serial, active_id)
        elif active_id is not None:
            self.retire(task)
        order = list(dict.fromkeys(
            ([active_id] if active_id in pool else [])
            + list(self.demand_prefix if self.prefix_region == rid else ())
            + list(pool)))
        chosen = []
        used = set()
        kept = OrderedDict()
        for identity in order:
            demand = pool.get(identity)
            if demand is None or demand.retired:
                continue
            remaining = sorted(c for c in demand.targets if grid.value(c) == UNKNOWN)
            if not remaining:
                self.note('CACHE_RELEASED', rid, identity, reason='NO_UNKNOWN_TARGETS')
                continue
            if len(chosen) >= limit:
                self.note('CACHE_RELEASED', rid, identity, reason='DEMAND_SAMPLE_LIMIT',
                          remaining=len(remaining))
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
                self.note('CACHE_RELEASED', rid, identity,
                          reason=('BOUNDARY_REPROJECTION_UNAVAILABLE' if frontier is None
                                  else 'BOUNDARY_ALREADY_USED'), remaining=len(remaining))
                continue
            if frontier != demand.frontier:
                self.note('BOUNDARY_REPROJECTED', rid, identity,
                          previous=demand.frontier, current=frontier)
            demand.frontier = frontier
            chosen.append(frontier)
            used.add(frontier)
            kept[identity] = demand
            self.sampled_demands[(rid, frontier)] = demand
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
        # A failed preparation is not evidence that an installed prefix ended.

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

    def query(self, start, goal, quota=None, *, preview=False):
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
        if preview:
            return RouteSearchResult(RouteSearchStatus.INCOMPLETE, reason='PREVIEW_ESTIMATE_ONLY')
        # One unfinished connection owns the bounded search working set.
        # Other candidates can use cheap/cache witnesses, but cannot evict it.
        if self.searches and not any(k[1] == goal for k in self.searches):
            self.pending.add(goal)
            return RouteSearchResult(RouteSearchStatus.INCOMPLETE, reason='CONNECTION_OWNER_BUSY')
        if self.budget.remaining <= 0:
            self.pending.add(goal)
            return RouteSearchResult(RouteSearchStatus.INCOMPLETE, reason='EXPANSION_BUDGET')
        # A one-byte reachability index prevents an unreachable endpoint from
        # exhausting/restarting a Python search over an entire large map. It
        # contains no occupancy/terrain authority. Four-connectivity has the
        # same components as 8-connectivity without diagonal corner cutting.
        if self.reachable is None and not self.searches:
            began = time.perf_counter()
            seed = np.zeros(grid.data.shape, dtype=bool)
            seed[start[1],start[0]] = True
            self.reachable = binary_propagation(seed, mask=grid.data == FREE)
            self.reachability_ms += (time.perf_counter()-began)*1000.
        if self.reachable is not None and bool(self.reachable[start[1],start[0]]) != bool(self.reachable[goal[1],goal[0]]):
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
            self.searches.clear()  # old start/evidence cannot own a second search
            search = KnownFreeSearch(start, goal)
            self.searches[search_key] = search
        self.searches.move_to_end(search_key)
        # Current terminal connections may use the remaining decision budget.
        # Preview transitions retain their small quota and cannot displace one.
        local = ExpansionBudget(self.budget.remaining if quota is None else
                                min(max(0, quota), self.budget.remaining))
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

    def retain_goals(self, goals):
        """Release an owner only when its problem is no longer requested."""
        for key in list(self.searches):
            if key[1] not in goals:
                self.searches.pop(key)

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
        result = self.query(a, b, preview=True)
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
    estimated_time_s: float = 0.
    travel_time_s: float = 0.
    turn_time_s: float = 0.
    stop_turn_time_s: float = 0.
    continuation_time_s: float = 0.
    preview_regions_priced: int = 0
    preview_unavailable: bool = False
    reordered: bool = False
    predicted_gain: int = 0
    station_prefix: tuple = ()
    comparison: object = None


@dataclass(frozen=True)
class VisitCostModel:
    """Ranking estimates in seconds; never motion limits or certificates."""
    linear_speed: float = .5
    yaw_rate: float = .8
    stop_turn_angle: float = .8
    stop_restart_time: float = 1.
    reorder_ratio: float = 1.15

    def __post_init__(self):
        values = (self.linear_speed, self.yaw_rate, self.stop_turn_angle,
                  self.stop_restart_time, self.reorder_ratio)
        if (not all(math.isfinite(v) for v in values)
                or min(values[:3]) <= 0 or self.stop_turn_angle > math.pi
                or self.stop_restart_time < 0 or self.reorder_ratio < 1):
            raise ValueError('Invalid Region visit ranking estimates')

    def seconds(self, distance, turn, stops):
        return distance/self.linear_speed + turn/self.yaw_rate + stops*self.stop_restart_time


def _motion_shape(points, resolution):
    """Metre-scale cost geometry, not a replacement executable route.

    Consecutive grid steps on a diagonal alternate by 45 degrees. Resampling
    the witnessed polyline at 1 m avoids pricing each raster step as a body
    turn. Always retain the original route for validation and publication.
    """
    if points is None or len(points) < 2:
        return None, None, ()
    p = np.asarray(points, dtype=float)*resolution
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    if arc[-1] <= 1e-9:
        return None, None, ()
    samples = np.arange(0., arc[-1], 1.)
    # Fold a short remainder into the last metre-scale chord rather than
    # introducing a tiny final segment with a quantized heading.
    if len(samples) > 1 and arc[-1]-samples[-1] < .5:
        samples = samples[:-1]
    samples = np.r_[samples, arc[-1]]
    q = np.c_[np.interp(samples, arc, p[:,0]), np.interp(samples, arc, p[:,1])]
    d = np.diff(q, axis=0)
    d = d[np.linalg.norm(d, axis=1) > 1e-9]
    if not len(d):
        return None, None, ()
    heading = np.arctan2(d[:,1], d[:,0])
    bends = tuple(abs(math.remainder(float(b-a), 2*math.pi))
                  for a,b in zip(heading[:-1], heading[1:]))
    return float(heading[0]), float(heading[-1]), bends


def _angle(a, b):
    return 0. if a is None or b is None else abs(math.remainder(b-a, 2*math.pi))


def choose_joint_visit(current, lookahead, routes, *, body_yaw=None,
                      depth=3, beam_width=8, retained_prefix=(), retained_demands=(),
                      retained_stations=(), cost_model=None, resolution=1.):
    """Short joint Frontier/station visit, then a two-Region exit preview.

    The current Region owns all executable choices. Future legs are estimates;
    only candidate.path (already checked raw FREE) can be handed to Local.
    A frontier is visited once in a prefix, regardless of station alternatives.
    Expected observations are unioned, avoiding reward for counting the same
    unknown cells twice. No station yaw requirement is imposed on 360deg LiDAR.
    """
    if not current:
        return None
    model = cost_model or VisitCostModel()
    transitions = {}
    def leg(a, b):
        key = (a.cell, b.cell)
        if key not in transitions:
            distance, path = routes.transition(*key)
            if path:
                entry, arrival, bends = _motion_shape(path, resolution)
            else:
                entry = arrival = math.atan2(b.cell[1]-a.cell[1], b.cell[0]-a.cell[0])
                bends = ()
            transitions[key] = (distance, entry, arrival, bends, path is None)
        return transitions[key]

    # Retain the information identity, not an unconditional execution order.
    def demand(c):
        return getattr(c, 'visit_remaining_cells', c.observation_cells)
    current = [c for c in current if demand(c)]
    if not current:
        return None
    keys = {visit_demand_key(c) for c in current}
    requested = tuple(retained_demands or retained_prefix)
    retained = tuple(k for k in requested if k in keys)
    available = {(visit_demand_key(c), c.cell) for c in current}
    stations = {k: s for k, s in retained_stations if (k, s) in available}
    def follows_retained(prefix):
        count = min(len(prefix), len(retained))
        return (bool(retained)
                and tuple(visit_demand_key(c) for c in prefix[:count]) == retained[:count]
                and all(c.cell == stations.get(visit_demand_key(c), c.cell)
                        for c in prefix[:count]))

    def state_time(state):
        return model.seconds(state[2], state[3], state[4])

    def partial_rank(state):
        return (-len(state[1])/(1.+state_time(state)),
                tuple((c.frontier_cell,c.cell) for c in state[0]))

    # prefix, expected remaining cells, length, turn, large turns, final heading
    beam = []
    for c in current:
        heading, arrival, bends = _motion_shape(c.path, resolution)
        initial_turn = _angle(body_yaw, heading)
        beam.append(((c,), set(demand(c)), c.path_length,
                     sum(bends)+initial_turn,
                     sum(b > model.stop_turn_angle for b in bends)
                     + int(initial_turn > model.stop_turn_angle), arrival))
    complete = list(beam)
    for _ in range(1, min(depth, len(keys))):
        expanded = []
        for prefix, seen, length, turn, stops, heading in beam:
            used = {visit_demand_key(c) for c in prefix}
            for c in current:
                if visit_demand_key(c) in used or not demand(c)-seen:
                    continue
                distance, entry, arrival, bends, _ = leg(prefix[-1], c)
                if not math.isfinite(distance):
                    continue
                entry_turn = _angle(heading, entry)
                expanded.append((prefix+(c,), seen | demand(c),
                    length+distance, turn+entry_turn+sum(bends),
                    stops+int(entry_turn > model.stop_turn_angle)
                    + sum(b > model.stop_turn_angle for b in bends), arrival))
        expanded.sort(key=partial_rank)
        beam = expanded[:beam_width]
        # One incumbent slot inside the SAME beam budget allows a fair final
        # hysteresis comparison even if a small gain increase prunes it early.
        incumbent = next((s for s in expanded if follows_retained(s[0])), None)
        if incumbent is not None and not any(s is incumbent for s in beam):
            beam[-1:] = [incumbent]
        complete.extend(beam)
        if not beam:
            break

    # When another independently useful demand exists, compare the access to
    # the first station AND its next visit. A one-station reward ratio would
    # otherwise defeat joint selection by winning before that leg is priced.
    joint = [state for state in complete if len(state[0]) > 1]
    if joint:
        complete = joint
    evaluations = []
    pools = [pool for pool in lookahead[:2] if pool]
    for state in complete:
        prefix, seen, length, turn, stops, heading = state
        # Visit B then C in coarse order, with only their bounded stations.
        tails = [(prefix[-1], 0., heading, False)]
        levels = [tails]
        for pool in pools:
            next_tails = []
            for previous, cost, tail_heading, estimated in tails:
                for option in pool:
                    distance, entry, exit_heading, bends, provisional = leg(previous, option)
                    if not math.isfinite(distance):
                        continue
                    angle = _angle(tail_heading, entry)
                    next_tails.append((option, cost+model.seconds(distance, angle+sum(bends),
                        int(angle > model.stop_turn_angle)+sum(b > model.stop_turn_angle for b in bends)),
                        exit_heading, estimated or provisional))
            tails = sorted(next_tails, key=lambda s:(s[1],s[0].cell))[:beam_width]
            if not tails:
                break
            levels.append(tails)
        evaluations.append((state, levels))
    # All current prefixes are ranked against the SAME available horizon.
    # An unavailable future must neither erase a valid joint visit nor give
    # that prefix a free tail compared with others priced through B/C.
    priced = min(len(levels)-1 for _,levels in evaluations)
    unavailable = priced < len(lookahead[:2])
    ranked = []
    for state, levels in evaluations:
        prefix, seen, length, turn, stops, _ = state
        tail = min(levels[priced], key=lambda s:(s[1],s[0].cell))
        total = state_time(state)+tail[1]
        score = len(seen)/(1.+total)
        key = (score, -total, tuple((-c.cell[0],-c.cell[1]) for c in prefix))
        choice = JointVisitChoice(prefix[0], tuple(c.frontier_cell for c in prefix),score,
            tail[3] or unavailable, tuple(visit_demand_key(c) for c in prefix),
            estimated_time_s=total, travel_time_s=length/model.linear_speed,
            turn_time_s=turn/model.yaw_rate, stop_turn_time_s=stops*model.stop_restart_time,
            continuation_time_s=tail[1], preview_regions_priced=priced,
            preview_unavailable=unavailable, predicted_gain=len(seen),
            station_prefix=tuple(c.cell for c in prefix))
        ranked.append((key, choice, follows_retained(prefix)))
    best = max(ranked, key=lambda item:item[0])
    incumbents = [item for item in ranked if item[2]]
    if incumbents:
        incumbent = max(incumbents, key=lambda item:item[0])
        if best[0][0] <= incumbent[0][0]*model.reorder_ratio:
            best = incumbent
    result = best[1]
    result.reordered = bool(retained) and not best[2]
    incumbent = max(incumbents, key=lambda item:item[0])[1] if incumbents else None
    result.comparison = dict(
        retained=list(requested), available=list(retained),
        missing_demands=[k for k in requested if k not in keys],
        missing_stations=[(k,s) for k,s in retained_stations if (k,s) not in available],
        reason=('NO_RETAINED_PREFIX' if not requested else
                'RETAINED_CANDIDATES_UNAVAILABLE' if not retained else
                'RETAINED_ORDER_UNAVAILABLE' if incumbent is None else
                'GAIN_TIME_IMPROVEMENT' if result.reordered else 'RETAINED_WITHIN_HYSTERESIS'),
        threshold=model.reorder_ratio,
        winner_gain=result.predicted_gain, winner_time_s=result.estimated_time_s,
        winner_score=result.score,
        incumbent_gain=incumbent.predicted_gain if incumbent else None,
        incumbent_time_s=incumbent.estimated_time_s if incumbent else None,
        incumbent_score=incumbent.score if incumbent else None,
        score_ratio=(result.score/incumbent.score if incumbent and incumbent.score>0 else None))
    return result
