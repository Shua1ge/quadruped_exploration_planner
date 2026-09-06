import importlib.util
import math
import pathlib

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "frontier_explorer", ROOT / "scripts" / "frontier_explorer.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_ray_integration_preserves_unknown_and_marks_hit():
    grid = MODULE.ExplorationGrid(20.0, 20.0, 0.5)
    ranges = [math.inf] * 72
    east_index = 36
    ranges[east_index] = 4.0
    grid.integrate_ranges((0.0, 0.0), ranges, 5.0)
    ray_angle = -math.pi + (east_index + 0.5) * (2.0 * math.pi / len(ranges))

    assert grid.value(grid.world_to_cell(0.0, 0.0)) == MODULE.FREE
    assert grid.value(grid.world_to_cell(
        3.0 * math.cos(ray_angle), 3.0 * math.sin(ray_angle))) == MODULE.FREE
    assert grid.value(grid.world_to_cell(
        4.0 * math.cos(ray_angle), 4.0 * math.sin(ray_angle))) == MODULE.OCCUPIED
    assert grid.value(grid.world_to_cell(8.0, 8.0)) == MODULE.UNKNOWN


def test_no_return_ray_only_clears_conservative_near_field():
    grid = MODULE.ExplorationGrid(20.0, 20.0, 0.25)
    ranges = [math.inf] * 360
    grid.integrate_ranges((0.0, 0.0), ranges, 7.5, no_return_range=3.0)

    assert grid.value(grid.world_to_cell(2.5, 0.0)) == MODULE.FREE
    assert grid.value(grid.world_to_cell(4.5, 0.0)) == MODULE.UNKNOWN


def test_hit_dilation_closes_neighbouring_bearing_gaps():
    ranges = [math.inf] * 16
    ranges[5] = 2.0
    dilated = MODULE.dilate_hit_ranges(ranges, 2)

    assert all(dilated[index] == 2.0 for index in (3, 4, 5, 6, 7))
    assert math.isinf(dilated[8])


def test_original_obstacle_points_are_always_inserted():
    grid = MODULE.ExplorationGrid(10.0, 10.0, 0.2)
    grid.mark_occupied_points([[1.03, -0.97], [1.06, -0.92]])

    assert grid.value(grid.world_to_cell(1.03, -0.97)) == MODULE.OCCUPIED


def test_frontiers_separate_known_free_from_unknown():
    grid = MODULE.ExplorationGrid(12.0, 12.0, 1.0)
    grid.data[4:8, 4:8] = MODULE.FREE
    frontiers = grid.frontier_cells(set())
    clusters = MODULE.cluster_frontiers(frontiers, minimum_size=2)

    assert frontiers
    assert len(clusters) == 1
    assert (5, 5) not in frontiers


def test_safe_viewpoint_stands_back_on_known_side_of_frontier():
    grid = MODULE.ExplorationGrid(12.0, 12.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.UNKNOWN
    grid.data[2:10, 1:7] = MODULE.FREE
    frontier = (6, 5)

    viewpoints = MODULE.safe_viewpoint_cells(
        grid, frontier, inflated=set(), stand_off=2.0)

    assert viewpoints
    assert all(cell[0] < frontier[0] for cell in viewpoints)
    assert all(grid.value(cell) == MODULE.FREE for cell in viewpoints)
    assert all(MODULE.segment_known_free(grid, cell, frontier, set())
               for cell in viewpoints)


def test_safe_viewpoint_respects_inflated_obstacle_clearance():
    grid = MODULE.ExplorationGrid(12.0, 12.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.UNKNOWN
    grid.data[2:10, 1:8] = MODULE.FREE
    grid.data[4, 4] = MODULE.OCCUPIED
    frontier = (7, 5)
    inflated = grid.inflated_obstacles(1.0)

    viewpoints = MODULE.safe_viewpoint_cells(
        grid, frontier, inflated=inflated, stand_off=2.0)

    assert viewpoints
    assert all(cell not in inflated for cell in viewpoints)
    assert all(MODULE.segment_known_free(grid, cell, frontier, inflated)
               for cell in viewpoints)


def test_known_astar_never_crosses_unknown_or_corner_cuts():
    grid = MODULE.ExplorationGrid(8.0, 8.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.UNKNOWN
    grid.data[1, 1:6] = MODULE.FREE
    grid.data[1:6, 5] = MODULE.FREE
    path = MODULE.astar_known(grid, (1, 1), (5, 5), set())

    assert path
    assert all(grid.value(cell) == MODULE.FREE for cell in path)
    assert (2, 2) not in path


def test_simplification_stays_inside_known_free_space():
    grid = MODULE.ExplorationGrid(10.0, 10.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    grid.data[4, 4] = MODULE.OCCUPIED
    inflated = grid.inflated_obstacles(0.0)
    raw = MODULE.astar_known(grid, (1, 4), (7, 4), inflated)
    simplified = MODULE.simplify_known_path(grid, raw, inflated)

    assert raw
    assert len(simplified) >= 3
    assert all(MODULE.segment_known_free(grid, a, b, inflated)
               for a, b in zip(simplified[:-1], simplified[1:]))


def test_known_astar_avoids_only_the_reported_directed_edge():
    grid = MODULE.ExplorationGrid(8.0, 5.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    blocked = {((2, 2), (3, 2))}

    forward = MODULE.astar_known(grid, (1, 2), (5, 2), set(), blocked)
    reverse = MODULE.astar_known(grid, (3, 2), (2, 2), set(), blocked)

    assert forward
    assert ((2, 2), (3, 2)) not in set(zip(forward[:-1], forward[1:]))
    assert reverse == [(3, 2), (2, 2)]


def test_single_source_tree_reuses_one_search_for_multiple_goals():
    grid = MODULE.ExplorationGrid(10.0, 8.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    grid.data[2:7, 4] = MODULE.OCCUPIED
    inflated = grid.inflated_obstacles(0.0)
    start = (1, 1)
    goals = {(7, 1), (7, 6)}

    tree = MODULE.build_shortest_path_tree(
        grid, start, inflated, targets=goals)

    for goal in goals:
        shared_path = tree.path_to(goal)
        astar_path = MODULE.astar_known(grid, start, goal, inflated)
        assert shared_path
        assert astar_path
        assert math.isclose(
            MODULE.path_length_cells(shared_path, grid.resolution),
            MODULE.path_length_cells(astar_path, grid.resolution))


def test_prepared_path_splices_to_current_pose_and_drops_invalid_prefix():
    grid = MODULE.ExplorationGrid(8.0, 6.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    prepared = [(1, 2), (2, 2), (3, 2), (4, 2), (5, 2)]
    grid.data[2, 2] = MODULE.OCCUPIED
    inflated = grid.inflated_obstacles(0.0)

    reused = MODULE.splice_prepared_path(
        grid, (3, 1), prepared, inflated)

    assert reused
    assert reused[0] == (3, 1)
    assert reused[-1] == (5, 2)
    assert (2, 2) not in reused
    assert MODULE.adjacent_grid_path_is_valid(grid, reused, inflated)


def test_prepared_path_rejects_newly_blocked_goal():
    grid = MODULE.ExplorationGrid(8.0, 6.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    prepared = [(1, 2), (2, 2), (3, 2)]
    grid.data[2, 3] = MODULE.OCCUPIED
    inflated = grid.inflated_obstacles(0.0)

    assert MODULE.splice_prepared_path(
        grid, (1, 1), prepared, inflated) is None


def test_simplification_does_not_restore_a_blocked_edge():
    grid = MODULE.ExplorationGrid(8.0, 5.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    blocked = {((2, 2), (3, 2))}
    raw = MODULE.astar_known(grid, (1, 2), (5, 2), set(), blocked)
    simplified = MODULE.simplify_known_path(grid, raw, set(), blocked)

    assert raw
    assert all(MODULE.segment_known_free(grid, a, b, set(), blocked)
               for a, b in zip(simplified[:-1], simplified[1:]))


def test_collision_segment_matches_one_edge_of_the_active_path():
    path = [(1, 2), (2, 2), (3, 2), (4, 2)]

    assert MODULE.match_blocked_path_edge(path, (2, 2), (3, 2)) == (
        (2, 2), (3, 2))
    assert MODULE.match_blocked_path_edge(path, (2, 2), (2, 2)) == (
        (1, 2), (2, 2))


def test_partition_and_tracking_preserve_region_ids():
    clusters_first = [[(2, 2), (2, 3)], [(22, 2), (22, 3)]]
    observations = MODULE.partition_frontier_clusters(clusters_first, 10)
    tracker = MODULE.PersistentRegionTracker(match_distance_cells=5.0)
    first = tracker.update(observations)

    clusters_second = [[(3, 2), (3, 3)], [(21, 2), (21, 3)]]
    second = tracker.update(MODULE.partition_frontier_clusters(clusters_second, 10))

    assert [region.region_id for region in first] == [0, 1]
    assert [region.region_id for region in second] == [0, 1]


def test_region_partition_merges_neighbours_across_old_bucket_boundary():
    clusters = [
        [(9, 2), (9, 3)],
        [(11, 2), (11, 3)],
        [(30, 2), (30, 3)],
    ]

    regions = MODULE.partition_frontier_clusters(clusters, 10)

    assert len(regions) == 2
    assert any(len(region.clusters) == 2 for region in regions)


def test_region_partition_does_not_chain_into_an_oversized_region():
    clusters = [[(0, 2)], [(8, 2)], [(16, 2)]]

    regions = MODULE.partition_frontier_clusters(clusters, 10)

    assert len(regions) == 2
    assert sorted(len(region.clusters) for region in regions) == [1, 2]


def test_committed_region_keeps_best_successor_when_region_splits():
    tracker = MODULE.PersistentRegionTracker(match_distance_cells=20.0)
    old = MODULE.FrontierRegion(
        -1, [[(x, 2) for x in range(2, 10)]], (5.5, 2.0),
        {(x, 2) for x in range(2, 10)})
    original = tracker.update([old])[0]

    small_child = MODULE.FrontierRegion(
        -1, [[(2, 2), (3, 2)]], (2.5, 2.0), {(2, 2), (3, 2)})
    large_child = MODULE.FrontierRegion(
        -1, [[(5, 2), (6, 2), (7, 2), (8, 2), (9, 2)]],
        (7.0, 2.0), {(5, 2), (6, 2), (7, 2), (8, 2), (9, 2)})
    updated = tracker.update(
        [small_child, large_child], preferred_region_id=original.region_id)

    successor = next(region for region in updated
                     if region.region_id == original.region_id)
    assert successor.cells == large_child.cells


def test_open_held_karp_does_not_charge_return_to_start():
    # Robot->A->B costs 2, while Robot->B->A costs 11.  A closed TSP would be
    # tempted by B->A because A->robot is cheap; the open solution is A then B.
    costs = np.array([
        [0.0, 1.0, 1.0],
        [100.0, 0.0, 1.0],
        [1.0, 10.0, 0.0],
    ])

    assert MODULE.solve_open_held_karp(costs) == [0, 1]


def test_open_held_karp_supports_asymmetry_and_forced_first():
    costs = np.array([
        [0.0, 1.0, 2.0, 3.0],
        [9.0, 0.0, 1.0, 8.0],
        [9.0, 5.0, 0.0, 1.0],
        [9.0, 1.0, 5.0, 0.0],
    ])

    assert MODULE.solve_open_held_karp(costs) == [0, 1, 2]
    forced = MODULE.solve_open_held_karp(costs, forced_first=2)
    assert forced[0] == 2
    assert sorted(forced) == [0, 1, 2]


def test_region_commitment_ignores_new_regions_until_current_tour_expires():
    retained, needs_replan = MODULE.retain_region_commitment(
        [1, 2], {1, 2, 3}, active_region_id=1)

    assert retained == [1, 2]
    assert not needs_replan


def test_region_commitment_replans_when_active_region_disappears():
    retained, needs_replan = MODULE.retain_region_commitment(
        [1, 2, 3], {2, 3, 4}, active_region_id=1)

    assert retained == [2, 3]
    assert needs_replan


def test_region_release_requires_consecutive_missing_updates():
    active, streak, released = MODULE.advance_region_release(7, {8}, 0, 3)
    assert (active, streak, released) == (7, 1, False)

    active, streak, released = MODULE.advance_region_release(7, {8}, streak, 3)
    assert (active, streak, released) == (7, 2, False)

    active, streak, released = MODULE.advance_region_release(7, {8}, streak, 3)
    assert (active, streak, released) == (None, 0, True)


def test_region_release_streak_resets_when_candidate_returns():
    assert MODULE.advance_region_release(7, {7, 8}, 2, 3) == (7, 0, False)


def test_path_length_uses_grid_resolution():
    path = [(0, 0), (1, 0), (2, 1)]
    expected = 0.2 * (1.0 + math.sqrt(2.0))

    assert math.isclose(MODULE.path_length_cells(path, 0.2), expected)


def test_remaining_polyline_distance_uses_arc_length_after_projection():
    points = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0)]

    assert math.isclose(MODULE.remaining_polyline_distance((2.0, 0.5), points), 5.0)
    assert math.isclose(MODULE.remaining_polyline_distance((4.0, 1.0), points), 2.0)


def test_path_turn_cost_penalizes_zigzags():
    straight = [(0, 0), (1, 0), (2, 0)]
    corner = [(0, 0), (1, 0), (1, 1)]

    assert MODULE.path_turn_cost(straight) == 0.0
    assert MODULE.path_turn_cost(corner) > 1.5


def test_remaining_path_validation_ignores_passed_obstacle_and_finds_future_one():
    grid = MODULE.ExplorationGrid(12.0, 6.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    path = [(1, 2), (2, 2), (3, 2), (4, 2), (5, 2), (6, 2)]
    grid.data[2, 1] = MODULE.OCCUPIED
    grid.data[2, 6] = MODULE.OCCUPIED
    inflated = grid.inflated_obstacles(0.0)

    check = MODULE.validate_remaining_path(
        grid, (4, 2), path, previous_progress_index=2, inflated=inflated)

    assert not check.valid
    assert check.progress_index == 3
    assert check.reason == "occupied_cell"
    assert check.invalid_cell == (6, 2)


def test_remaining_path_validation_progress_is_monotonic():
    grid = MODULE.ExplorationGrid(12.0, 6.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    path = [(1, 2), (2, 2), (3, 2), (4, 2), (5, 2), (6, 2)]

    check = MODULE.validate_remaining_path(
        grid, (2, 2), path, previous_progress_index=4, inflated=set())

    assert check.valid
    assert check.progress_index == 4


def test_remaining_path_validation_reports_temporary_blocked_edge():
    grid = MODULE.ExplorationGrid(12.0, 6.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    path = [(1, 2), (2, 2), (3, 2), (4, 2)]
    blocked = {((2, 2), (3, 2))}

    check = MODULE.validate_remaining_path(
        grid, (1, 2), path, previous_progress_index=0,
        inflated=set(), blocked_edges=blocked)

    assert not check.valid
    assert check.reason == "temporarily_blocked_edge"
    assert check.invalid_edge == ((2, 2), (3, 2))


def test_observation_target_is_frozen_and_progress_counts_newly_known_cells():
    grid = MODULE.ExplorationGrid(10.0, 10.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    grid.data[3:6, 3:6] = MODULE.UNKNOWN
    target = MODULE.observation_target_cells(grid, (4, 4), radius=2.0)

    assert len(target) == 9
    assert MODULE.observation_progress(grid, target) == (0, 9, 0.0)

    grid.data[3:5, 3:6] = MODULE.FREE
    observed, expected, ratio = MODULE.observation_progress(grid, target)

    assert observed == 6
    assert expected == 9
    assert math.isclose(ratio, 2.0 / 3.0)


def test_observation_progress_counts_free_and_occupied_as_valid_observations():
    grid = MODULE.ExplorationGrid(5.0, 5.0, 1.0, 0.0, 0.0)
    target = {(1, 1), (2, 1), (3, 1), (4, 1)}
    grid.data[1, 1] = MODULE.FREE
    grid.data[1, 2] = MODULE.OCCUPIED

    assert MODULE.observation_progress(grid, target) == (2, 4, 0.5)


def test_completion_streak_requires_consecutive_updates_above_threshold():
    streak = 0
    for progress in (0.81, 0.85, 0.79, 0.82, 0.83, 0.84):
        streak = MODULE.advance_completion_streak(progress, 0.80, streak)

    assert streak == 3
