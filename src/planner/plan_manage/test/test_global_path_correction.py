"""Rejected route correction: safety, finite work, observability and lifecycle."""

from types import SimpleNamespace

import numpy as np

from test_frontier_explorer import MODULE, candidate_test_explorer


def correction_scene():
    explorer = candidate_test_explorer()
    explorer.footprint_radius = .35
    explorer.footprint_offset = 1.0
    explorer.observation_radius = 6.0  # Synthetic map uses 1 m cells.
    explorer.grid.data[1, 3] = MODULE.UNKNOWN
    path = [(1, 2), (2, 2), (3, 2), (4, 2), (5, 3), (6, 3)]
    messages = []
    explorer.get_logger = lambda: SimpleNamespace(
        info=lambda value, **kw: messages.append(value),
        warning=lambda value, **kw: messages.append(value))
    return explorer, path, messages


def test_same_station_repair_changes_geometry_and_reuses_result(monkeypatch):
    explorer, failed, logs = correction_scene()
    before = explorer.grid.data.copy()
    assert not MODULE.double_cylinder_path_free(
        explorer.grid, failed, failed[-1], (8, 3), set(), .35, 1.)
    repaired = explorer.repair_rejected_global_path(
        failed[0], failed, failed[-1], (8, 3), set(), set())
    assert repaired and repaired != failed
    assert repaired[0] == failed[0] and repaired[-1] == failed[-1]
    assert MODULE.double_cylinder_path_free(
        explorer.grid, repaired, repaired[-1], (8, 3), set(), .35, 1.)
    monkeypatch.setattr(MODULE, "footprint_aware_path", lambda *a: (_ for _ in ()).throw(
        AssertionError("unchanged context must not repeat the alternative search")))
    assert explorer.repair_rejected_global_path(
        failed[0], failed, failed[-1], (8, 3), set(), set()) == repaired
    assert any("geometry_changed=True" in line for line in logs)
    assert any("cached=true" in line for line in logs)
    np.testing.assert_array_equal(before, explorer.grid.data)


def test_search_exhaustion_is_memoized_but_relevant_revision_reopens(monkeypatch):
    explorer, failed, logs = correction_scene()
    calls = []
    monkeypatch.setattr(MODULE, "footprint_aware_path", lambda *a: calls.append(a) or None)
    for _ in range(10):
        assert explorer.repair_rejected_global_path(
            failed[0], failed, failed[-1], (8, 3), set(), set()) is None
    assert len(calls) == 1
    explorer.map_content_revision += 1
    explorer.repair_rejected_global_path(failed[0], failed, failed[-1], (8, 3), set(), set())
    assert len(calls) == 2
    explorer.route_constraint_revision = 1
    explorer.repair_rejected_global_path(failed[0], failed, failed[-1], (8, 3), set(), set())
    assert len(calls) == 3
    assert not hasattr(explorer, "goal_failure_cooldowns")


def test_safe_approach_has_value_and_never_clears_unknown_or_exclusions():
    explorer, failed, logs = correction_scene()
    explorer.grid.data[3, 7] = MODULE.UNKNOWN
    targets = {(7, 3)}
    before = explorer.grid.data.copy()
    result = explorer.safe_observation_approach(failed, (8, 3), targets, set(), ())
    assert result is not None
    prefix, cell = result
    assert cell != failed[-1] and len(prefix) >= 2
    assert MODULE.double_cylinder_path_free(
        explorer.grid, prefix, cell, (8, 3), set(), .35, 1.)
    assert explorer.safe_observation_approach(failed, (8, 3), set(), set(), ()) is None
    explorer.global_approach_min_distance = 20.
    assert explorer.safe_observation_approach(failed, (8, 3), targets, set(), ()) is None
    explorer.global_approach_min_distance = 1.
    excluded = [explorer.grid.cell_to_world(point) for point in failed]
    assert explorer.safe_observation_approach(failed, (8, 3), targets, set(), excluded) is None
    np.testing.assert_array_equal(before, explorer.grid.data)


def test_dense_candidate_correction_preserves_frontier_and_observation_targets(monkeypatch):
    explorer, failed, logs = correction_scene()
    explorer.grid.data[3, 7] = MODULE.UNKNOWN
    frontier = (8, 3)
    monkeypatch.setattr(MODULE, "candidate_cells", lambda cluster: [frontier])
    monkeypatch.setattr(MODULE, "safe_viewpoint_cells", lambda *a, **kw: [failed[-1]])
    monkeypatch.setattr(MODULE, "observation_target_cells", lambda *a: {(7, 3)})
    monkeypatch.setattr(MODULE, "build_shortest_path_tree", lambda *a:
                        SimpleNamespace(path_to=lambda cell: failed, expanded_cells=6))
    region = MODULE.FrontierRegion(3, [[frontier]], frontier, {frontier})
    records = explorer.build_frontier_candidates(failed[0], [region], set(), set())
    assert len(records) == 1
    assert records[0].frontier_cell == frontier
    assert records[0].observation_cells == {(7, 3)}
    assert records[0].path != failed


def test_all_full_routes_fail_then_approach_is_a_candidate_not_completion(monkeypatch):
    explorer, failed, logs = correction_scene()
    explorer.grid.data[3, 7] = MODULE.UNKNOWN
    frontier = (8, 3)
    monkeypatch.setattr(MODULE, "candidate_cells", lambda cluster: [frontier])
    monkeypatch.setattr(MODULE, "safe_viewpoint_cells", lambda *a, **kw: [failed[-1]])
    monkeypatch.setattr(MODULE, "observation_target_cells", lambda *a: {(7, 3)})
    monkeypatch.setattr(MODULE, "build_shortest_path_tree", lambda *a:
                        SimpleNamespace(path_to=lambda cell: failed, expanded_cells=6))
    monkeypatch.setattr(MODULE, "footprint_aware_path", lambda *a: None)
    region = MODULE.FrontierRegion(3, [[frontier]], frontier, {frontier})
    records = explorer.build_frontier_candidates(failed[0], [region], set(), set())
    assert len(records) == 1 and records[0].cell != failed[-1]
    assert records[0].frontier_cell == frontier and records[0].observation_cells == {(7, 3)}
    assert records[0].terminal_cells == (records[0].cell,)
    assert any("SAFE_OBSERVATION_APPROACH" in line for line in logs)


def test_released_motion_timeout_runs_on_static_map_without_blacklisting():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.get_clock = lambda: SimpleNamespace(now=lambda:
                         SimpleNamespace(nanoseconds=20_000_000_000))
    explorer.last_task_clock_ns = 19_000_000_000
    explorer.locomotion_ready = lambda: True
    explorer.active_goal = (3., 0.)
    explorer.scan_waiting_for_target = True
    explorer.active_task_elapsed = 10.
    explorer.goal_timeout = 5.
    explorer.auto_start = True
    explorer.position = (1., 0.)
    explorer.map_update_count = 10
    task = MODULE.ObservationTask(3, (3., 0.), {(5, 1)}, task_id=99, motion_released=True)
    explorer.evaluate_active_observation = lambda: task
    explorer.observation_complete = lambda value: False
    explorer.get_logger = lambda: SimpleNamespace(warning=lambda *a, **kw: None)
    calls = []
    explorer.terminate_active_observation = lambda *a, **kw: calls.append((a, kw))
    explorer.handoff_satisfied_observation = lambda *a: (_ for _ in ()).throw(
        AssertionError("timeout must precede the released-motion early return"))
    MODULE.FrontierExplorer.exploration_timer.__wrapped__(explorer)
    assert calls == [(("HANDOFF_WAIT_TIMEOUT", "REARBITRATING_FRONTIER"), {})]
    assert explorer.goal_timeout_count == 1 and not task.information_satisfied


def test_autonomous_dense_failure_does_not_release_or_mark_region_unobservable():
    explorer, _, logs = correction_scene()
    explorer.task_mode = explorer.task_geometry_autonomy = True
    explorer.active_region_id = 3
    assert not explorer.handle_dense_invalid_region(3, [])
    assert explorer.active_region_id == 3 and explorer.dense_invalid_region_revisions == {}


def test_bounded_search_random_obstacles_never_returns_unsafe_path():
    rng = np.random.default_rng(11)
    for _ in range(40):
        explorer = candidate_test_explorer()
        explorer.grid.data[rng.random(explorer.grid.data.shape) < .18] = MODULE.UNKNOWN
        explorer.grid.data[2, 1] = explorer.grid.data[3, 6] = MODULE.FREE
        path = MODULE.footprint_aware_path(
            explorer.grid, (1, 2), (6, 3), (8, 3), set(), .35, 1., set(), 100)
        if path:
            assert MODULE.double_cylinder_path_free(
                explorer.grid, path, (6, 3), (8, 3), set(), .35, 1.)
            assert all(explorer.grid.value(cell) == MODULE.FREE for cell in path)


def test_shortcut_cannot_undo_a_footprint_repair():
    explorer, failed, logs = correction_scene()
    repaired = explorer.repair_rejected_global_path(
        failed[0], failed, failed[-1], (8, 3), set(), set())
    assert repaired
    # Explicit unsafe shortcut stands in for a point-only simplifier.
    preserved = explorer.preserve_footprint_route(
        repaired, failed, failed[-1], (8, 3), set())
    assert preserved == repaired
    assert any("SHORTCUT_REJECTED" in line for line in logs)


def test_sparse_candidate_uses_same_repair_without_persistent_rejection(monkeypatch):
    explorer, failed, logs = correction_scene()
    explorer.dense_final_validation_searches = explorer.sparse_final_validation_failures = 0
    monkeypatch.setattr(MODULE, "astar_known", lambda *a: failed)
    candidate = MODULE.FrontierCandidate(
        3, failed[-1], (8, 3), explorer.grid.cell_to_world(failed[-1]), [],
        5., 1, 1, 0., {(7, 3)}, terminal_cells=(failed[-1],))
    repaired = explorer.materialize_sparse_candidate(failed[0], candidate, set(), set())
    assert repaired is not None and repaired.path != failed
    assert repaired.frontier_cell == candidate.frontier_cell
    assert repaired.sparse_route is None


def test_search_budget_defers_untried_goal_instead_of_caching_failure(monkeypatch):
    explorer, failed, logs = correction_scene()
    explorer.global_repair_search_limit = 1
    monkeypatch.setattr(MODULE, "footprint_aware_path", lambda *a: None)
    explorer.repair_rejected_global_path(failed[0], failed, failed[-1], (8, 3), set(), set())
    explorer.repair_rejected_global_path(failed[0], failed, failed[-1], (9, 3), set(), set())
    assert (failed[-1], (9, 3)) not in explorer._global_repair_cache
    assert any("DEFERRED_SEARCH_BUDGET" in line for line in logs)
