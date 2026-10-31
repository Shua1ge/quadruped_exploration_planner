"""Explorer integration of bounded Region visiting, using existing contracts."""
import math
import time

from .grid import UNKNOWN
from .path_planning import RouteSearchStatus, path_length_cells, path_turn_cost
from .region_visits import (BoundedVisitRoutes, RegionVisitWindow, choose_joint_visit,
                           same_visit_demand, visit_demand_key)


class RegionVisitPlanner:
    def bounded_visits_enabled(self):
        return (getattr(self, 'region_visit_window_enabled', False)
                and self.selection_strategy == 'hierarchical'
                and not self.sparse_routing_enabled
                and getattr(self, 'task_geometry_autonomy', True))

    def visit_window(self):
        if not hasattr(self, '_region_visits'):
            self._region_visits = RegionVisitWindow()
            self._visit_routes = BoundedVisitRoutes()
        return self._region_visits

    def visit_candidate_available(self, candidate):
        ref = getattr(candidate, 'visit_demand_ref', None)
        return ref is None or not ref.retired

    def retire_visit_demand(self, task):
        window = getattr(self, '_region_visits', None)
        if window is not None and task is not None and window.retire(task):
            getattr(self, 'last_visit_stats', {}).update(
                installed_demand_prefix=list(window.demand_prefix))
            self.log_visit_plan('SATISFIED', task)

    def log_visit_plan(self, state, task, request_id=0, trajectory_id=0):
        plan = getattr(task, 'visit_plan', None)
        if plan is None:
            return
        window = self.visit_window()
        self.get_logger().info(
            f"[REGION_VISIT_PLAN] state={state} plan={plan.plan_id} "
            f"task={task.task_id} version={task.task_version} request={request_id} "
            f"trajectory={trajectory_id} "
            f"region={task.region_id} demand={task.visit_demand_id} "
            f"priced={plan.demand_ids} remaining={window.demand_prefix}")

    def install_visit_plan(self, task, request_id, trajectory_id):
        plan = getattr(task, 'visit_plan', None)
        if plan is None:
            return
        self.retire_visit_demand(task)
        key = (task.task_id, task.task_version, request_id)
        if self.visit_window().install(plan, key):
            getattr(self, 'last_visit_stats', {}).update(
                installed_plan_id=plan.plan_id,
                installed_demand_prefix=list(self.visit_window().demand_prefix))
            self.log_visit_plan('INSTALLED', task, request_id, trajectory_id)

    def choose_window_visit(self, start, regions, blocked_edges, excluded_goals,
                            *, allow_region_release, allow_cross_region_preparation,
                            required_scope):
        began = time.perf_counter()
        window = self.visit_window()
        selected_regions = window.reconcile(regions, start, self.map_content_revision)
        if required_scope is not None:
            selected_regions = [r for r in selected_regions
                                if r.region_id == window.committed_region
                                and required_scope == window.committed_scope]
        self.last_visit_window = [r.region_id for r in selected_regions]
        window.demands = {rid: pool for rid, pool in window.demands.items()
                          if rid in self.last_visit_window}
        self.region_visit_deferred = False
        self._visit_pending_regions = set()
        self._visit_capacity_regions = set()
        if not selected_regions:
            self.region_visit_deferred = bool(window.deferred)
            self.last_visit_stats = {'window': [], 'deferred': self.region_visit_deferred,
                'reason': 'WAITING_FOR_RELATED_INPUT_OR_CAPACITY' if window.deferred else 'NO_CURRENT_WORK'}
            self.publish_region_overlay(regions, [])
            return None
        routes = self._visit_routes
        routes.begin(self.grid, blocked_edges, self.region_visit_route_expansions,
                     revision=self.map_content_revision)
        records = self.build_frontier_candidates(
            start, selected_regions, set(), blocked_edges, excluded_goals,
            frontier_limit=self.region_visit_frontiers,
            station_limit=self.region_visit_stations,
            route_provider=routes,
            preview_regions={r.region_id for r in selected_regions[1:]})
        self.publish_region_overlay(regions, [c for c in records if c.path])
        for c in records:
            # Separate remaining-reward estimate from the frozen task ledger.
            predicted = (c.visit_visible_cells if c.visit_visible_cells is not None
                         else c.observation_cells)
            c.visit_remaining_cells = {cell for cell in predicted
                                      if self.grid.value(cell) == UNKNOWN}
        by_region = {r.region_id: [c for c in records if c.region_id == r.region_id]
                     for r in selected_regions}
        task = getattr(self, 'active_observation', None)
        chosen_region = None
        current = []
        for r in selected_regions:
            rid = r.region_id
            options = by_region[rid]
            if rid in self._visit_pending_regions:
                # Valid siblings are usable, but no sibling means pending work,
                # not permission to silently skip a Region whose search is busy.
                if not any(c.path for c in options):
                    self.region_visit_deferred = True
                    if rid in self._visit_capacity_regions:
                        window.defer(rid, self.map_content_revision)
                        continue
                    break
            if rid != selected_regions[0].region_id:
                if required_scope is not None:
                    break
                if task is not None and not (allow_region_release or allow_cross_region_preparation):
                    break
                # Preview paths were lower bounds. Certify the first actual
                # departure only when this Region is promoted for selection.
                ready = []
                for c in options:
                    result = routes.query(start, c.cell)
                    if result.status == RouteSearchStatus.FOUND:
                        c.path = result.path
                        c.path_length = path_length_cells(c.path, self.grid.resolution)
                        c.turn_cost = path_turn_cost(c.path)
                        c.route_source = 'visit_bounded'
                        ready.append(c)
                    elif result.status == RouteSearchStatus.INCOMPLETE:
                        self._visit_pending_regions.add(rid)
                        if result.reason == 'SEARCH_CACHE_CAPACITY':
                            self._visit_capacity_regions.add(rid)
                options = ready
                if not options and rid in self._visit_pending_regions:
                    self.region_visit_deferred = True
                    if rid in self._visit_capacity_regions:
                        window.defer(rid, self.map_content_revision)
                        continue
                    break
            if options:
                chosen_region, current = rid, options
                break
            window.defer(rid, self.map_content_revision)
        if chosen_region is None:
            self.last_visit_stats = dict(window=self.last_visit_window,
                candidates=len(records), preview_candidates=sum(not c.path for c in records), expansions=routes.budget.used,
                deferred=self.region_visit_deferred, reason='SEARCH_INCOMPLETE' if
                self.region_visit_deferred else 'NO_USABLE_STATION')
            self._log_visit(began)
            return None

        self.region_visit_deferred = False
        prefix = window.prefix if window.prefix_region == chosen_region else ()
        demand_prefix = window.demand_prefix if window.prefix_region == chosen_region else ()
        if (task is not None and excluded_goals and task.region_id == chosen_region):
            # Prepare the next objective, not a different pose of the same
            # still-executing objective, when an independent sibling exists.
            siblings = [c for c in current if not same_visit_demand(c, task)]
            if siblings:
                current = siblings
                prefix = tuple(f for f in prefix if f != task.frontier_cell)
                demand_prefix = tuple(k for k in demand_prefix if k != visit_demand_key(task))
        remaining_ids = [r.region_id for r in selected_regions]
        after = remaining_ids[remaining_ids.index(chosen_region)+1:]
        lookahead = [by_region[rid] for rid in after[:2]]
        choice = choose_joint_visit(current, lookahead, routes,
            body_yaw=getattr(self, 'body_yaw', None), retained_prefix=prefix,
            retained_demands=demand_prefix,
            retained_stations=window.retained_stations(chosen_region),
            cost_model=getattr(self, 'region_visit_cost_model', None),
            resolution=self.grid.resolution)
        if choice is None:
            self.last_visit_stats = dict(window=self.last_visit_window,
                candidates=len(records), deferred=False, reason='NO_REMAINING_VISIBLE_TARGETS')
            self._log_visit(began)
            return None
        candidate = choice.candidate
        candidate.visit_plan = window.draft(choice, chosen_region)
        candidate.commitment_scope = window.scope(chosen_region)
        candidate.option_target_cells = set().union(*(c.observation_cells for c in current))
        self.last_selection_reason = 'bounded_joint_visit'
        self.last_visit_stats = dict(window=self.last_visit_window,
            selected=chosen_region, candidates=len(records),
            preview_candidates=sum(not c.path for c in records),
            frontier_objectives=len({(c.region_id,c.frontier_cell) for c in records}),
            prefix=[list(p) for p in choice.prefix], score=choice.score,
            demand_prefix=list(choice.demand_prefix),
            plan_id=candidate.visit_plan.plan_id, plan_state='DRAFT',
            installed_plan_id=(window.installed_plan.plan_id if window.installed_plan else None),
            installed_demand_prefix=list(window.demand_prefix),
            station_prefix=[list(p) for p in choice.station_prefix],
            selected_demand=candidate.visit_demand_id,
            selected_visible_cells=len(candidate.visit_remaining_cells),
            selected_target_cells=len(candidate.observation_cells),
            predicted_gain=choice.predicted_gain,
            estimated_time_s=choice.estimated_time_s,
            travel_time_s=choice.travel_time_s,
            turn_time_s=choice.turn_time_s,
            stop_turn_time_s=choice.stop_turn_time_s,
            continuation_time_s=choice.continuation_time_s,
            preview_regions_priced=choice.preview_regions_priced,
            preview_unavailable=choice.preview_unavailable,
            reordered=choice.reordered,
            expansions=routes.budget.used, cache_hits=routes.cache_hits,
            estimated_transitions=routes.estimated_transitions,
            preview_estimated=choice.continuation_estimated, deferred=False)
        old_sequence = self.region_sequence
        self.region_sequence = list(window.order)
        if self.region_sequence != old_sequence:
            self.publish_region_sequence()
        self.last_candidate_tree_ms = 0.  # no all-map Dijkstra in this mode
        self._log_visit(began)
        return candidate

    def _log_visit(self, began):
        self.last_expanded_grid_cells = self._visit_routes.budget.used
        self.expanded_grid_cells += self.last_expanded_grid_cells
        if self.region_sequence != self._region_visits.order:
            self.region_sequence = list(self._region_visits.order)
            self.publish_region_sequence()
        self.last_visit_stats['capacity_yields'] = len(self._visit_routes.capacity_yields)
        self.last_visit_stats['reachability_ms'] = self._visit_routes.reachability_ms
        self.last_visit_stats['reachability_bytes'] = (0 if self._visit_routes.reachable is None
                                                       else self._visit_routes.reachable.nbytes)
        self.last_visit_stats['elapsed_ms'] = (time.perf_counter()-began)*1000
        # Do not keep the full planning snapshot alive between callbacks.
        self._visit_routes.grid = None
        self.get_logger().info(
            f"[REGION_VISIT_WINDOW] {self.last_visit_stats}", throttle_duration_sec=2.)
