import pathlib
import sys

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from explorer_core.safe_region_graph import (  # noqa: E402
    PersistentSafeRegionTracker, ShadowStabilityTracker,
    extract_safe_region_graph,
    safe_region_oracle_metrics)


def test_open_space_is_fully_covered_and_connected():
    occupancy = np.full((30, 40), 100, dtype=np.int8)
    occupancy[2:28, 2:38] = 0

    graph = extract_safe_region_graph(
        occupancy, 0.2, (0.0, 0.0), tile_size=2.0)
    metrics = safe_region_oracle_metrics(occupancy, 0.2, graph, 12)

    assert len(graph.cell_to_region) == int(np.count_nonzero(occupancy == 0))
    assert len(graph.regions) < len(graph.cell_to_region) / 20
    assert metrics["safe_region_connectivity_recall"] == 1.0
    assert metrics["safe_region_false_positive_rate"] == 0.0


def test_wall_separates_nearby_free_spaces():
    occupancy = np.full((30, 40), 100, dtype=np.int8)
    occupancy[3:27, 3:18] = 0
    occupancy[3:27, 22:37] = 0

    graph = extract_safe_region_graph(
        occupancy, 0.2, (0.0, 0.0), tile_size=2.0)
    left_ids = {graph.cell_to_region[(x, y)]
                for y in range(3, 27) for x in range(3, 18)}
    right_ids = {graph.cell_to_region[(x, y)]
                 for y in range(3, 27) for x in range(22, 37)}
    portal_pairs = {
        frozenset((portal.source_id, portal.target_id))
        for portal in graph.portals.values()}

    assert left_ids.isdisjoint(right_ids)
    assert not any(pair & left_ids and pair & right_ids for pair in portal_pairs)


def test_narrow_door_creates_a_bottleneck_portal():
    occupancy = np.full((30, 40), 100, dtype=np.int8)
    occupancy[2:28, 2:38] = 0
    occupancy[:, 19:21] = 100
    occupancy[13:17, 19:21] = 0

    graph = extract_safe_region_graph(
        occupancy, 0.2, (0.0, 0.0), tile_size=2.0,
        bottleneck_width=1.2)
    metrics = safe_region_oracle_metrics(occupancy, 0.2, graph, 16)

    assert any(portal.bottleneck for portal in graph.portals.values())
    assert metrics["safe_region_connectivity_recall"] == 1.0
    assert metrics["safe_region_false_positive_rate"] == 0.0


def test_ids_follow_world_coordinates_when_patch_origin_slides():
    first = np.full((20, 30), 100, dtype=np.int8)
    first[2:18, 2:28] = 0
    second = np.full((20, 30), 100, dtype=np.int8)
    second[2:18, 1:27] = 0

    first_graph = extract_safe_region_graph(
        first, 0.2, (0.0, 0.0), tile_size=2.0)
    second_graph = extract_safe_region_graph(
        second, 0.2, (0.2, 0.0), tile_size=2.0)

    assert set(first_graph.regions) == set(second_graph.regions)
    assert set(first_graph.portals) == set(second_graph.portals)


def test_stability_tracker_reports_removed_visible_ids():
    occupancy = np.full((20, 30), 100, dtype=np.int8)
    occupancy[2:18, 2:28] = 0
    tracker = ShadowStabilityTracker()
    graph = extract_safe_region_graph(
        occupancy, 0.2, (0.0, 0.0), tile_size=2.0)

    first = tracker.update(graph)
    second = tracker.update(graph)

    assert first["safe_region_id_retention"] == 1.0
    assert first["portal_id_retention"] == 1.0
    assert second["safe_region_id_retention"] == 1.0
    assert second["portal_id_retention"] == 1.0


def test_persistent_tracker_keeps_id_when_free_space_grows_within_tile():
    first = np.full((20, 20), 100, dtype=np.int8)
    first[5:15, 7:15] = 0
    second = first.copy()
    second[5:15, 5:7] = 0
    tracker = PersistentSafeRegionTracker()

    first_graph = tracker.update(
        extract_safe_region_graph(first, 0.2, (0.0, 0.0), 4.0),
        (0.0, 0.0), 0.2)
    second_graph = tracker.update(
        extract_safe_region_graph(second, 0.2, (0.0, 0.0), 4.0),
        (0.0, 0.0), 0.2)

    assert set(first_graph.regions) == set(second_graph.regions)
