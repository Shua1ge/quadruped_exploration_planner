"""Explorer coarse connections and incremental conflict-segment repair."""
from types import SimpleNamespace

import numpy as np
import pytest

from test_frontier_explorer import MODULE, candidate_test_explorer
from explorer_core.path_planning import (
    CoarseRouteRepair, ExpansionBudget, KnownFreeSearch, RouteSearchStatus,
    adjacent_grid_path_is_valid, path_length_cells,
)
from explorer_core.grid import ExplorationGrid


def no_physical_veto(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('Explorer cannot certify Local body motion')
    for name in ('double_cylinder_path_free', 'footprint_aware_path',
                 'diagnose_double_cylinder_path'):
        monkeypatch.setattr(MODULE, name, forbidden)


def candidate_scene(monkeypatch):
    ex = candidate_test_explorer()
    ex.footprint_radius, ex.footprint_offset = .35, 1.
    path = [(x, 2) for x in range(1, 7)]
    ex.grid.data[2, 0] = MODULE.OCCUPIED  # rear proxy overlaps; centreline remains FREE
    ex.grid.data[2, 9] = MODULE.UNKNOWN
    monkeypatch.setattr(MODULE, 'candidate_cells', lambda cluster: [(8, 2)])
    monkeypatch.setattr(MODULE, 'safe_viewpoint_cells', lambda *a, **kw: [(6, 2)])
    monkeypatch.setattr(MODULE, 'observation_target_cells', lambda *a: {(9, 2)})
    region = MODULE.FrontierRegion(3, [[(8, 2)]], (8, 2), {(8, 2)})
    return ex, path, region


def test_dense_raw_connection_survives_inflation_and_body_proxy_failure(monkeypatch):
    ex, path, region = candidate_scene(monkeypatch)
    assert not MODULE.double_cylinder_path_free(ex.grid, path, path[-1], (8, 2), set(), .35, 1.)
    no_physical_veto(monkeypatch)
    records = ex.build_frontier_candidates(path[0], [region], set(path), set())
    assert len(records) == 1
    assert records[0].path == path
    assert ex.last_candidate_rejections['final_validation_failed'] == 0
    assert records[0].observation_cells == {(9, 2)}


def test_sparse_materialization_uses_same_raw_check(monkeypatch):
    ex, path, _ = candidate_scene(monkeypatch)
    no_physical_veto(monkeypatch)
    ex.dense_final_validation_searches = ex.sparse_final_validation_failures = 0
    c = MODULE.FrontierCandidate(3, path[-1], (8, 2), ex.grid.cell_to_world(path[-1]),
                                 [], 5., 1, 1, 0., {(9, 2)}, terminal_cells=(path[-1],))
    out = ex.materialize_sparse_candidate(path[0], c, set(path), set())
    assert out and out.path == path


@pytest.mark.parametrize('value', [MODULE.UNKNOWN, MODULE.OCCUPIED])
def test_actual_cut_never_publishes_a_reference_through_it(monkeypatch, value):
    ex, path, region = candidate_scene(monkeypatch)
    ex.grid.data[:, 4] = value
    no_physical_veto(monkeypatch)
    assert ex.build_frontier_candidates(path[0], [region], set(), set()) == []
    assert ex.validated_reference_path(path) is None


def test_terminal_centre_must_be_free_but_margin_is_not_a_veto():
    ex = candidate_test_explorer()
    cell = (6, 2)
    assert ex.terminal_rejection_reason(cell, {cell}) is None
    ex.grid.data[2, 6] = MODULE.OCCUPIED
    assert ex.terminal_rejection_reason(cell, set()) == 'no_safe_viewpoint'


def test_reference_densification_cannot_restore_inflation_veto(monkeypatch):
    ex = candidate_test_explorer()
    ex.inflation_radius = 1.
    ex.grid.data[1, 3] = MODULE.OCCUPIED
    no_physical_veto(monkeypatch)
    path = ex.validated_reference_path([(1, 2), (6, 2)])
    assert path and path[0] == (1, 2) and path[-1] == (6, 2)


def long_scene():
    g = ExplorationGrid(730., 10., 1., 0., 0.)
    g.data.fill(MODULE.FREE)
    path = [(x, 3) for x in range(1, 711)]
    g.data[3, 4] = MODULE.OCCUPIED
    return g, path


def test_conflict_repair_preserves_long_prefix_and_suffix():
    g, path = long_scene()
    before = g.data.copy()
    budget = ExpansionBudget(400)
    out = CoarseRouteRepair().repair(g, path, set(), budget)
    assert out.status == RouteSearchStatus.FOUND
    assert out.path != path and out.path[0] == path[0] and out.path[-1] == path[-1]
    assert out.path[-650:] == path[-650:]
    assert budget.used < 100  # conflict near start; no whole 710 m search
    assert adjacent_grid_path_is_valid(g, out.path, set())
    np.testing.assert_array_equal(before, g.data)


def test_budget_miss_resumes_same_search_instead_of_becoming_no_path():
    g, path = long_scene()
    repair = CoarseRouteRepair()
    statuses = []
    for _ in range(100):
        out = repair.repair(g, path, set(), ExpansionBudget(1))
        statuses.append(out.status)
        if out.status == RouteSearchStatus.FOUND:
            break
    assert statuses[0] == RouteSearchStatus.INCOMPLETE
    assert statuses[-1] == RouteSearchStatus.FOUND
    assert RouteSearchStatus.NO_ROUTE not in statuses
    assert len(repair.searches) == 1
    search = next(iter(repair.searches.values()))
    count = len(search.closed)
    cached = repair.repair(g, path, set(), ExpansionBudget(0))
    assert cached.status == RouteSearchStatus.FOUND and cached.expanded == 0
    assert len(search.closed) == count


def test_other_goal_with_same_conflict_reuses_segment_search():
    g, path = long_scene()
    repair = CoarseRouteRepair()
    assert repair.repair(g, path, set(), ExpansionBudget(1)).status == RouteSearchStatus.INCOMPLETE
    shared = next(iter(repair.searches.values()))
    other = path[:500]
    result = repair.repair(g, other, set(), ExpansionBudget(400))
    assert result.status == RouteSearchStatus.FOUND
    assert next(iter(repair.searches.values())) is shared
    assert result.path[-1] == other[-1]


def test_unrelated_map_change_preserves_progress_but_relevant_change_restarts():
    g, path = long_scene()
    repair = CoarseRouteRepair()
    repair.repair(g, path, set(), ExpansionBudget(1))
    first = next(iter(repair.searches.values()))
    g.data[9, 700] = MODULE.OCCUPIED
    repair.repair(g, path, set(), ExpansionBudget(1))
    assert next(iter(repair.searches.values())) is first
    watched = next(c for c, value in first.cells.items() if value == MODULE.FREE and c[1] != 3)
    g.data[watched[1], watched[0]] = MODULE.OCCUPIED
    repair.repair(g, path, set(), ExpansionBudget(1))
    assert next(iter(repair.searches.values())) is not first


def test_measured_cut_is_distinct_from_unfinished_search():
    g, path = long_scene()
    g.data[:, 4] = MODULE.OCCUPIED
    repair = CoarseRouteRepair()
    out = repair.repair(g, path, set(), ExpansionBudget(1))
    assert out.status == RouteSearchStatus.INCOMPLETE
    for _ in range(100):
        out = repair.repair(g, path, set(), ExpansionBudget(10))
        if out.status != RouteSearchStatus.INCOMPLETE:
            break
    assert out.status == RouteSearchStatus.NO_ROUTE
    assert out.reason == 'KNOWN_FREE_GRAPH_EXHAUSTED'
    assert out.path is None


def test_opening_in_measured_cut_invalidates_negative_result():
    g, path = long_scene()
    g.data[:, 4] = MODULE.OCCUPIED
    repair = CoarseRouteRepair()
    assert repair.repair(g, path, set(), ExpansionBudget(400)).status == RouteSearchStatus.NO_ROUTE
    g.data[3, 4] = MODULE.FREE
    assert repair.repair(g, path, set(), ExpansionBudget(400)).status == RouteSearchStatus.FOUND


def test_repair_checks_all_conflicts_and_does_not_keep_a_blocked_suffix():
    g, path = long_scene()
    blocked = {(path[-5], path[-4])}
    result = CoarseRouteRepair().repair(g, path, blocked, ExpansionBudget(400))
    assert result.status == RouteSearchStatus.FOUND
    assert adjacent_grid_path_is_valid(g, result.path, set(), blocked)
    assert not blocked.intersection(zip(result.path, result.path[1:]))


def test_search_forbids_unknown_and_diagonal_corner_cutting():
    g = ExplorationGrid(5., 5., 1., 0., 0.)
    g.data.fill(MODULE.UNKNOWN)
    g.data[1, 1] = g.data[2, 2] = MODULE.FREE
    out = KnownFreeSearch((1, 1), (2, 2)).advance(g, set(), ExpansionBudget(100))
    assert out.status == RouteSearchStatus.NO_ROUTE
    g.data[1, 2] = MODULE.FREE
    out = KnownFreeSearch((1, 1), (2, 2)).advance(g, set(), ExpansionBudget(100))
    assert out.path == [(1, 1), (2, 1), (2, 2)]


def test_changed_constraint_restarts_only_affected_search():
    g, path = long_scene()
    repair = CoarseRouteRepair()
    repair.repair(g, path, set(), ExpansionBudget(1))
    first = next(iter(repair.searches.values()))
    unrelated = {((600, 3), (601, 3))}
    repair.repair(g, path, unrelated, ExpansionBudget(1))
    assert next(iter(repair.searches.values())) is first
    edge = next(edge for edge in first.edges if not first.edges[edge])
    repair.repair(g, path, {edge}, ExpansionBudget(1))
    assert next(iter(repair.searches.values())) is not first


def test_pending_prepared_repair_does_not_clear_candidate_or_reenumerate():
    ex = candidate_test_explorer()
    ex.prepared_candidate = object()
    candidate = ex.prepared_candidate
    ex.last_coarse_route_result = SimpleNamespace(status=RouteSearchStatus.INCOMPLETE)
    ex.activate_prepared_observation = lambda *a: False
    ex.prepare_next_observation = lambda **kw: pytest.fail('must resume, not enumerate')
    assert not ex.activate_or_prepare_next_observation(True)
    assert ex.prepared_candidate is candidate


def test_budget_result_is_typed_and_does_not_blacklist_information():
    ex = candidate_test_explorer()
    ex.grid.data[2, 4] = MODULE.OCCUPIED
    path = [(x, 2) for x in range(1, 8)]
    before = ex.grid.data.copy()
    ex._repair_expansion_budget = ExpansionBudget(1)
    first = ex.repair_rejected_global_path(path[0], path, path[-1], (8, 2), set(), set())
    assert first.status == RouteSearchStatus.INCOMPLETE
    ex._repair_expansion_budget = ExpansionBudget(400)
    ex.map_content_revision += 1  # unrelated revision does not reset the session
    last = ex.repair_rejected_global_path(path[0], path, path[-1], (8, 2), set(), set())
    assert last.status == RouteSearchStatus.FOUND
    assert not ex.dense_invalid_region_revisions
    np.testing.assert_array_equal(before, ex.grid.data)


def test_reference_repair_densifies_long_segment_without_crossing_wall():
    g, path = long_scene()
    result = CoarseRouteRepair().repair(g, [path[0], path[-1]], set(), ExpansionBudget(400))
    assert result.status == RouteSearchStatus.FOUND
    assert adjacent_grid_path_is_valid(g, result.path, set())


def test_no_route_does_not_mark_an_information_region_unobservable():
    ex = candidate_test_explorer()
    ex.active_region_id = 3
    assert not ex.handle_dense_invalid_region(3, [])
    assert ex.active_region_id == 3 and ex.dense_invalid_region_revisions == {}


def test_random_repair_results_are_raw_free_and_respect_constraints():
    rng = np.random.default_rng(31)
    for _ in range(40):
        g = ExplorationGrid(12., 8., 1., 0., 0.)
        g.data.fill(MODULE.FREE)
        g.data[rng.random(g.data.shape) < .2] = MODULE.UNKNOWN
        path = [(x, 3) for x in range(1, 11)]
        g.data[3, 1] = g.data[3, 10] = MODULE.FREE
        out = CoarseRouteRepair().repair(g, path, set(), ExpansionBudget(400))
        if out.status == RouteSearchStatus.FOUND:
            assert adjacent_grid_path_is_valid(g, out.path, set())


def test_real_prepared_handoff_resumes_repair_and_activates_once():
    ex = candidate_test_explorer()
    ex.grid.data[2, 4] = MODULE.OCCUPIED
    ex.grid.data[2, 9] = MODULE.UNKNOWN
    ex.prepared_candidate = MODULE.FrontierCandidate(
        3, (7, 2), (8, 2), ex.grid.cell_to_world((7, 2)),
        [(x, 2) for x in range(1, 8)], 6., 1, 1, 0., {(9, 2)},
        terminal_cells=((7, 2),))
    original = ex.prepared_candidate
    ex.global_repair_max_expansions = 1
    ex.current_blocked_edges = lambda: set()
    ex.directional_candidate_allowed = lambda *args: True
    ex.inflation_radius = .25
    ex.publish_status = lambda *a: None
    ex.prepare_next_observation = lambda **kw: pytest.fail('deferred search must resume')
    installed = []
    ex.activate_candidate = lambda path, candidate: installed.append((path, candidate)) or True
    assert not ex.activate_or_prepare_next_observation(True)
    assert ex.prepared_candidate is original and ex.coarse_repair_pending()
    for _ in range(100):
        if ex.activate_or_prepare_next_observation(True):
            break
    assert len(installed) == 1
    assert (4, 2) not in installed[0][1].path
    assert installed[0][1].cell == (7, 2)


def test_handoff_does_not_fall_back_to_full_enumeration_on_pending_repair():
    ex = candidate_test_explorer()
    ex.prepared_candidate = object()
    ex.active_goal = (7., 2.)
    ex.active_observation = None
    ex.last_coarse_route_result = SimpleNamespace(status=RouteSearchStatus.INCOMPLETE)
    ex.map_update_count, ex.last_handoff_attempt_update = 2, 1
    ex.activate_or_prepare_next_observation = lambda **kw: False
    ex.plan_from_current_position = lambda **kw: pytest.fail('deferred handoff is not a failed goal')
    ex.publish_status = lambda *a: None
    assert not ex._handoff_satisfied_observation('test')
