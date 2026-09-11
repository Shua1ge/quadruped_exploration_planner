import importlib.util
import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
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


def test_small_frontier_fragments_are_excluded_from_visible_usable_cells():
    frontiers = {(1, 1), (1, 2), (8, 8), (8, 9), (9, 8), (9, 9)}

    clusters = MODULE.cluster_frontiers(frontiers, minimum_size=3)
    visible = MODULE.clustered_frontier_cells(clusters)

    assert visible == {(8, 8), (8, 9), (9, 8), (9, 9)}
    assert (1, 1) not in visible
    assert (1, 2) not in visible


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

    assert 0 < tree.expanded_cells <= grid.width * grid.height
    for goal in goals:
        shared_path = tree.path_to(goal)
        astar_path = MODULE.astar_known(grid, start, goal, inflated)
        assert shared_path
        assert astar_path
        assert math.isclose(
            MODULE.path_length_cells(shared_path, grid.resolution),
            MODULE.path_length_cells(astar_path, grid.resolution))


def test_start_escape_opens_only_shortest_free_corridor():
    grid = MODULE.ExplorationGrid(9.0, 7.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    start = (3, 3)
    inflated = {(3, 3), (4, 3), (5, 3), (3, 2), (3, 4), (2, 3), (7, 5)}

    adjusted = MODULE.open_start_escape_corridor(
        grid, start, inflated, max_distance=4.0)

    assert adjusted is not None
    tree = MODULE.build_shortest_path_tree(grid, start, adjusted)
    assert tree.path_to((6, 3))
    assert (7, 5) in adjusted
    assert inflated - adjusted == {(3, 3), (2, 3)}


def test_start_escape_rejects_raw_occupied_robot_cell():
    grid = MODULE.ExplorationGrid(5.0, 5.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    start = (2, 2)
    grid.data[start[1], start[0]] = MODULE.OCCUPIED

    assert MODULE.open_start_escape_corridor(
        grid, start, {start}, max_distance=2.0) is None


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


def test_region_partition_keeps_wall_separated_frontiers_apart():
    grid = MODULE.ExplorationGrid(12.0, 10.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    grid.data[:, 5] = MODULE.OCCUPIED
    clusters = [[(3, 4), (3, 5)], [(7, 4), (7, 5)]]

    regions = MODULE.partition_frontier_clusters(
        clusters, 10, grid, grid.inflated_obstacles(0.0))

    assert len(regions) == 2


def test_region_partition_merges_frontiers_with_short_known_free_connection():
    grid = MODULE.ExplorationGrid(12.0, 10.0, 1.0, 0.0, 0.0)
    grid.data[:, :] = MODULE.FREE
    clusters = [[(3, 4), (3, 5)], [(7, 4), (7, 5)]]

    regions = MODULE.partition_frontier_clusters(clusters, 10, grid, set())

    assert len(regions) == 1


def test_topology_partition_keeps_different_corridor_branches_separate():
    clusters = [[(2, 2), (2, 3)], [(4, 2), (4, 3)]]

    regions = MODULE.partition_frontier_clusters_by_topology(
        clusters, [(10, 101), (10, 202)], 10)

    assert len(regions) == 2


def test_topology_partition_merges_nearby_frontiers_on_same_branch():
    clusters = [[(2, 2), (2, 3)], [(4, 2), (4, 3)]]

    regions = MODULE.partition_frontier_clusters_by_topology(
        clusters, [(10, 101), (10, 101)], 10)

    assert len(regions) == 1


def test_region_tracker_never_moves_an_id_across_topology_branches():
    tracker = MODULE.PersistentRegionTracker(match_distance_cells=20.0)
    first = MODULE.FrontierRegion(
        -1, [[(2, 2)]], (2.0, 2.0), {(2, 2)}, (10, 101))
    original_id = tracker.update([first])[0].region_id
    moved = MODULE.FrontierRegion(
        -1, [[(3, 2)]], (3.0, 2.0), {(3, 2)}, (10, 202))

    updated_id = tracker.update([moved], original_id)[0].region_id

    assert updated_id != original_id


def test_region_tracker_retains_id_when_cells_overlap_despite_branch_change():
    tracker = MODULE.PersistentRegionTracker(match_distance_cells=20.0)
    first = MODULE.FrontierRegion(
        -1, [[(2, 2), (3, 2)]], (2.5, 2.0), {(2, 2), (3, 2)},
        (10, 101))
    original_id = tracker.update([first])[0].region_id
    changed_branch = MODULE.FrontierRegion(
        -1, [[(3, 2), (4, 2)]], (3.5, 2.0), {(3, 2), (4, 2)},
        (10, 202))

    updated_id = tracker.update([changed_branch], original_id)[0].region_id

    assert updated_id == original_id


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
    update = MODULE.update_region_commitment(
        7, {8}, {8}, 0, None, 10.0, 3, 5.0)
    assert (update.active_region_id, update.missing_streak,
            update.release_reason) == (7, 1, None)

    update = MODULE.update_region_commitment(
        7, {8}, {8}, update.missing_streak, None, 10.5, 3, 5.0)
    assert (update.active_region_id, update.missing_streak,
            update.release_reason) == (7, 2, None)

    update = MODULE.update_region_commitment(
        7, {8}, {8}, update.missing_streak, None, 11.0, 3, 5.0)
    assert (update.active_region_id, update.missing_streak,
            update.release_reason) == (None, 0, "frontier_missing")


def test_region_release_streak_resets_while_frontier_region_still_exists():
    update = MODULE.update_region_commitment(
        7, {7, 8}, {7, 8}, 2, None, 10.0, 3, 5.0)
    assert update == MODULE.RegionCommitmentUpdate(7, 0, None)


def test_region_commitment_releases_after_persistent_unreachability():
    update = MODULE.update_region_commitment(
        7, {7, 8}, {8}, 0, None, 10.0, 3, 5.0)
    assert update == MODULE.RegionCommitmentUpdate(7, 0, 10.0)

    update = MODULE.update_region_commitment(
        7, {7, 8}, {8}, 0, update.unreachable_since, 14.9, 3, 5.0)
    assert update == MODULE.RegionCommitmentUpdate(7, 0, 10.0)

    update = MODULE.update_region_commitment(
        7, {7, 8}, {8}, 0, update.unreachable_since, 15.0, 3, 5.0)
    assert update == MODULE.RegionCommitmentUpdate(
        None, 0, None, "persistently_unreachable")


def test_region_commitment_cancels_unreachable_timer_when_candidate_returns():
    update = MODULE.update_region_commitment(
        7, {7, 8}, {7, 8}, 0, 10.0, 12.0, 3, 5.0)
    assert update == MODULE.RegionCommitmentUpdate(7, 0, None)


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


def test_reroute_failure_streak_abandons_at_configured_map_update_limit():
    streak = 0

    streak, abandon = MODULE.advance_reroute_failure_streak(streak, 3)
    assert (streak, abandon) == (1, False)
    streak, abandon = MODULE.advance_reroute_failure_streak(streak, 3)
    assert (streak, abandon) == (2, False)
    streak, abandon = MODULE.advance_reroute_failure_streak(streak, 3)
    assert (streak, abandon) == (3, True)


def test_reference_status_generation_rejects_late_previous_request():
    status, request_id = MODULE.parse_planning_status(
        "BLOCKED request_id=101")

    assert status == "BLOCKED"
    assert request_id == 101
    assert not MODULE.planning_status_matches_request(
        status, request_id, pending_generation=202, active_generation=202)
    assert MODULE.planning_status_matches_request(
        "BLOCKED", 202, pending_generation=202, active_generation=202)


def test_ready_status_must_match_pending_not_only_active_request():
    assert not MODULE.planning_status_matches_request(
        "PATH_TRAJECTORY_READY", 101,
        pending_generation=202, active_generation=202)
    assert MODULE.planning_status_matches_request(
        "PATH_TRAJECTORY_READY", 202,
        pending_generation=202, active_generation=202)
    assert not MODULE.planning_status_matches_request(
        "PATH_TRAJECTORY_READY", 202,
        pending_generation=None, active_generation=202)


def test_failure_cooldown_expires_after_configured_map_revision():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.blacklist_radius = 1.5
    explorer.map_update_count = 10
    explorer.goal_failure_cooldowns = {(2.0, 3.0): 16}

    assert explorer.is_goal_on_failure_cooldown((2.5, 3.0))
    assert not explorer.is_goal_on_failure_cooldown((4.0, 3.0))

    explorer.map_update_count = 16
    assert not explorer.is_goal_on_failure_cooldown((2.5, 3.0))
    assert explorer.goal_failure_cooldowns == {}


def test_replan_gate_bounds_attempts_until_planning_context_changes():
    gate = MODULE.ReplanGate(max_attempts=2)
    context = MODULE.ReplanContext((4, 5), 10, 3, 7)

    assert gate.allow(context)
    assert gate.allow(context)
    assert not gate.allow(context)
    assert gate.allow(MODULE.ReplanContext((4, 5), 11, 3, 7))
    assert gate.allow(MODULE.ReplanContext((4, 5), 11, 3, 8))


def test_region_sequence_reuses_stable_region_id_edge_cache():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.grid = MODULE.ExplorationGrid(12.0, 8.0, 1.0, 0.0, 0.0)
    explorer.grid.data[:, :] = MODULE.FREE
    explorer.preferred_goal_path_length = 0.0
    explorer.long_horizon_min_gain_ratio = 0.0
    explorer.active_region_id = None
    explorer.region_edge_cache = {}
    explorer.region_anchor_cells = {}
    explorer.region_edge_cache_hits = 0
    explorer.region_edge_cache_misses = 0
    explorer.last_region_edge_cache_hits = 0
    explorer.last_region_edge_cache_misses = 0
    explorer.region_pair_tree_searches = 0
    explorer.last_region_pair_tree_searches = 0
    explorer.expanded_grid_cells = 0
    explorer.last_expanded_grid_cells = 0
    explorer.sparse_routing_enabled = False
    explorer.sparse_router = MODULE.SparseRouteGraph()
    explorer.sparse_region_pair_queries = 0
    explorer.sparse_region_pair_hits = 0
    explorer.sparse_region_pair_fallbacks = 0
    explorer.last_sparse_region_ms = 0.0
    explorer.last_sparse_region_pair_hits = 0
    explorer.last_sparse_region_pair_fallbacks = 0
    regions = [
        MODULE.FrontierRegion(10, [[(2, 2)]], (2.0, 2.0), {(2, 2)}),
        MODULE.FrontierRegion(20, [[(8, 2)]], (8.0, 2.0), {(8, 2)}),
    ]
    candidates = {
        10: [SimpleNamespace(cell=(2, 2), path_length=2.0,
                             unknown_gain=10, cluster_size=3, turn_cost=0.0)],
        20: [SimpleNamespace(cell=(8, 2), path_length=8.0,
                             unknown_gain=10, cluster_size=3, turn_cost=0.0)],
    }

    first = explorer.plan_region_sequence((0, 2), regions, candidates, set(), set())
    searches_after_first = explorer.region_pair_tree_searches
    second = explorer.plan_region_sequence((0, 2), regions, candidates, set(), set())

    assert first == second
    assert searches_after_first == 2
    assert explorer.region_pair_tree_searches == searches_after_first
    assert explorer.region_edge_cache_hits == 2

    explorer.grid.data[2, 5] = MODULE.OCCUPIED
    explorer.plan_region_sequence(
        (0, 2), regions, candidates,
        explorer.grid.inflated_obstacles(0.0), set())

    assert explorer.region_pair_tree_searches > searches_after_first


def test_sparse_candidate_is_materialized_by_dense_astar_before_use():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.grid = MODULE.ExplorationGrid(8.0, 6.0, 1.0, 0.0, 0.0)
    explorer.grid.data[:, :] = MODULE.FREE
    explorer.dense_final_validation_searches = 0
    explorer.sparse_final_validation_failures = 0
    explorer.sparse_router = MODULE.SparseRouteGraph()
    explorer.get_logger = lambda: SimpleNamespace(warning=lambda *args, **kwargs: None)
    candidate = MODULE.FrontierCandidate(
        3, (6, 2), (7, 2), (6.5, 2.5), [], 6.0, 20, 8, 0.0,
        {(7, 2)})

    materialized = explorer.materialize_sparse_candidate(
        (1, 2), candidate, set(), set())

    assert materialized is not None
    assert materialized.path[0] == (1, 2)
    assert materialized.path[-1] == candidate.cell
    assert explorer.dense_final_validation_searches == 1
    assert explorer.sparse_final_validation_failures == 0


def test_sparse_candidate_reuses_valid_sparse_polyline():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.grid = MODULE.ExplorationGrid(8.0, 6.0, 1.0, 0.0, 0.0)
    explorer.grid.data[:, :] = MODULE.FREE
    explorer.grid.data[2, 3] = MODULE.OCCUPIED
    explorer.dense_final_validation_searches = 0
    explorer.sparse_final_validation_failures = 0
    explorer.sparse_router = MODULE.SparseRouteGraph()
    route = MODULE.SparseRouteEstimate(
        7.0, 0.0, 1, 2,
        ((1.5, 2.5), (1.5, 3.5), (6.5, 3.5), (6.5, 2.5)))
    candidate = MODULE.FrontierCandidate(
        3, (6, 2), (7, 2), (6.5, 2.5), [], 7.0, 20, 8, 0.0,
        {(7, 2)}, route)

    materialized = explorer.materialize_sparse_candidate(
        (1, 2), candidate, explorer.grid.inflated_obstacles(0.0), set())

    assert materialized is not None
    assert materialized.path == [
        (1, 2), (1, 3), (2, 3), (3, 3),
        (4, 3), (5, 3), (6, 3), (6, 2)]


def test_invalid_sparse_polyline_falls_back_to_dense_route():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.grid = MODULE.ExplorationGrid(8.0, 6.0, 1.0, 0.0, 0.0)
    explorer.grid.data[:, :] = MODULE.FREE
    explorer.grid.data[2, 3] = MODULE.OCCUPIED
    explorer.dense_final_validation_searches = 0
    explorer.sparse_final_validation_failures = 0
    explorer.sparse_router = MODULE.SparseRouteGraph()
    route = MODULE.SparseRouteEstimate(
        5.0, 0.0, 1, 2, ((1.5, 2.5), (6.5, 2.5)))
    candidate = MODULE.FrontierCandidate(
        3, (6, 2), (7, 2), (6.5, 2.5), [], 5.0, 20, 8, 0.0,
        {(7, 2)}, route)

    materialized = explorer.materialize_sparse_candidate(
        (1, 2), candidate, explorer.grid.inflated_obstacles(0.0), set())

    assert materialized is not None
    assert (3, 2) not in materialized.path
    assert materialized.path[0] == (1, 2)
    assert materialized.path[-1] == (6, 2)
    assert explorer.sparse_final_validation_failures == 0


def test_sparse_candidate_is_rejected_when_dense_map_disconnects_goal():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.grid = MODULE.ExplorationGrid(8.0, 6.0, 1.0, 0.0, 0.0)
    explorer.grid.data[:, :] = MODULE.FREE
    explorer.grid.data[:, 4] = MODULE.OCCUPIED
    explorer.dense_final_validation_searches = 0
    explorer.sparse_final_validation_failures = 0
    explorer.map_update_count = 0
    explorer.goal_failure_cooldown_updates = 6
    explorer.goal_failure_cooldowns = {}
    explorer.blacklist_radius = 0.75
    explorer.sparse_router = MODULE.SparseRouteGraph()
    explorer.sparse_router.graph_revision = 9
    explorer.get_logger = lambda: SimpleNamespace(warning=lambda *args, **kwargs: None)
    candidate = MODULE.FrontierCandidate(
        3, (6, 2), (7, 2), (6.5, 2.5), [], 6.0, 20, 8, 0.0,
        {(7, 2)})

    materialized = explorer.materialize_sparse_candidate(
        (1, 2), candidate, explorer.grid.inflated_obstacles(0.0), set())

    assert materialized is None
    assert explorer.dense_final_validation_searches == 1
    assert explorer.sparse_final_validation_failures == 1
    assert explorer.is_goal_on_failure_cooldown(candidate.goal)


def test_dense_failures_release_only_an_exhausted_active_region():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.map_content_revision = 17
    explorer.dense_invalid_region_revisions = {}
    explorer.active_region_id = 3
    explorer.active_region_missing_streak = 2
    explorer.active_region_unreachable_since = 5.0
    explorer.commitment_state = "COMMITTED"
    explorer.commitment_release_count = 0
    explorer.last_commitment_release_reason = "none"
    explorer.last_selection_reason = "residual_commitment"
    explorer.region_sequence = [3, 7]
    gate = SimpleNamespace(reset_calls=0)
    gate.reset = lambda: setattr(gate, "reset_calls", gate.reset_calls + 1)
    explorer.region_commitment_gate = gate
    explorer.get_logger = lambda: SimpleNamespace(warning=lambda *args, **kwargs: None)
    remaining = [MODULE.FrontierCandidate(
        7, (6, 2), (7, 2), (6.5, 2.5), [], 6.0, 20, 8, 0.0,
        {(7, 2)})]

    released = explorer.handle_dense_invalid_region(3, remaining)

    assert released
    assert explorer.active_region_id is None
    assert explorer.region_sequence == [7]
    assert explorer.commitment_release_count == 1
    assert explorer.last_commitment_release_reason == "dense_validation_failed"
    assert gate.reset_calls == 1
    assert explorer.dense_invalid_region_revisions == {3: 17}


def test_region_information_efficiency_prefers_near_useful_frontier():
    nearby = MODULE.region_information_efficiency(40, 20, 4.0, 1.0)
    distant = MODULE.region_information_efficiency(45, 20, 18.0, 1.0)

    assert nearby > distant


def test_preferred_horizon_uses_longer_candidate_with_sufficient_gain():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.preferred_goal_path_length = 3.5
    explorer.long_horizon_min_gain_ratio = 0.65
    short = SimpleNamespace(path_length=2.1, unknown_gain=100)
    longer = SimpleNamespace(path_length=3.8, unknown_gain=70)

    assert explorer.preferred_horizon_pool([short, longer]) == [longer]


def test_preferred_horizon_retains_short_fallback_when_long_gain_is_too_low():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.preferred_goal_path_length = 3.5
    explorer.long_horizon_min_gain_ratio = 0.65
    short = SimpleNamespace(path_length=2.1, unknown_gain=100)
    weak_longer = SimpleNamespace(path_length=3.8, unknown_gain=60)

    assert explorer.preferred_horizon_pool(
        [short, weak_longer]) == [short, weak_longer]


def test_rolling_region_keeps_active_region_within_switch_margin():
    selected = MODULE.select_rolling_region(
        {3: 10.0, 7: 12.0}, active_region_id=3, switch_ratio=1.35)

    assert selected == 3


def test_rolling_region_switches_for_clearly_better_challenger():
    selected = MODULE.select_rolling_region(
        {3: 10.0, 7: 14.0}, active_region_id=3, switch_ratio=1.35)

    assert selected == 7


def test_rolling_region_selects_reachable_standby_when_committed_is_unavailable():
    selected = MODULE.select_rolling_region(
        {7: 12.0, 9: 10.0}, active_region_id=3, switch_ratio=1.35)

    assert selected == 7


def test_cross_region_preparation_does_not_release_current_commitment():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.active_region_id = 3

    # Preparing another region is speculative: activation, not preparation,
    # is the only operation allowed to change active_region_id.
    MODULE.select_rolling_region(
        {7: 12.0, 9: 10.0}, explorer.active_region_id, 1.35)

    assert explorer.active_region_id == 3


def test_handoff_activates_existing_standby_without_replanning():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.activate_prepared_observation = lambda allow=False: True
    explorer.prepare_next_observation = lambda force=False: (_ for _ in ()).throw(
        AssertionError("existing standby must be reused"))

    assert explorer.activate_or_prepare_next_observation()


def test_handoff_forces_preparation_when_no_standby_exists():
    explorer = object.__new__(MODULE.FrontierExplorer)
    activations = iter((False, True))
    force_arguments = []
    explorer.activate_prepared_observation = lambda allow=False: next(activations)
    explorer.prepare_next_observation = lambda force=False: (
        force_arguments.append(force) or True)

    assert explorer.activate_or_prepare_next_observation()
    assert force_arguments == [True]


def test_completed_task_handoff_allows_commitment_transfer():
    explorer = object.__new__(MODULE.FrontierExplorer)
    transfer_arguments = []
    explorer.activate_prepared_observation = lambda allow=False: (
        transfer_arguments.append(allow) or True)

    assert explorer.activate_or_prepare_next_observation(
        allow_commitment_transfer=True)
    assert transfer_arguments == [True]


def test_region_prediction_signature_tracks_material_inputs():
    explorer = object.__new__(MODULE.FrontierExplorer)
    explorer.grid = SimpleNamespace(resolution=0.2)
    explorer.route_constraint_revision = 4
    explorer.sparse_router = MODULE.SparseRouteGraph()
    explorer.sparse_router.upsert_node(MODULE.SparseNode(1, (2.0, 3.0)))
    explorer.sparse_router.upsert_node(MODULE.SparseNode(2, (4.0, 3.0)))
    explorer.sparse_router.upsert_edge(MODULE.SparseEdge(
        5, 1, 2, 2.0, 1.0, ((2.0, 3.0), (4.0, 3.0))))

    baseline = explorer.build_region_prediction_signature((10, 10), {3, 7})

    assert explorer.build_region_prediction_signature((11, 10), {7, 3}) == baseline
    explorer.sparse_router.remove_edge(5)
    explorer.sparse_router.upsert_edge(MODULE.SparseEdge(
        9, 1, 2, 2.0, 1.0, ((2.0, 3.0), (4.0, 3.0))))
    assert explorer.build_region_prediction_signature((10, 10), {3, 7}) == baseline
    assert explorer.build_region_prediction_signature((15, 10), {3, 7}) != baseline
    assert explorer.build_region_prediction_signature((10, 10), {3, 8}) != baseline
