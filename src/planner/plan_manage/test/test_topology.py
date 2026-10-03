import pathlib
import sys

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from explorer_core.topology import (  # noqa: E402
    clearance_field, extract_topology, oracle_metrics, thin_free_space)
from explorer_core.grid import ExplorationGrid  # noqa: E402


def scalar_thinning_reference(mask):
    image = mask.astype(np.uint8).copy()
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0
    changed = True
    while changed:
        changed = False
        for first in (True, False):
            remove = []
            for y in range(1, image.shape[0] - 1):
                for x in range(1, image.shape[1] - 1):
                    ring = tuple(image[y + dy, x + dx] for dx, dy in
                                 ((0, -1), (1, -1), (1, 0), (1, 1),
                                  (0, 1), (-1, 1), (-1, 0), (-1, -1)))
                    transitions = sum(ring[i] == 0 and ring[(i + 1) % 8] == 1
                                      for i in range(8))
                    p2, _, p4, _, p6, _, p8, _ = ring
                    keep = ((p2 * p4 * p6 == 0 and p4 * p6 * p8 == 0)
                            if first else
                            (p2 * p4 * p8 == 0 and p2 * p6 * p8 == 0))
                    if image[y, x] and 2 <= sum(ring) <= 6 and transitions == 1 and keep:
                        remove.append((y, x))
            for y, x in remove:
                image[y, x] = 0
            changed |= bool(remove)
    return image.astype(bool)


def test_vectorized_thinning_matches_scalar_for_random_and_solid_maps():
    rng = np.random.default_rng(73)
    masks = [np.ones((51, 51), dtype=bool)]
    masks += [rng.random((17, 23)) < density
              for density in (0.1, 0.4, 0.7, 0.95) for _ in range(5)]
    for mask in masks:
        np.testing.assert_array_equal(thin_free_space(mask), scalar_thinning_reference(mask))


def test_euclidean_clearance_obeys_unknown_and_full_free_conventions():
    free = np.ones((7, 7), dtype=bool)
    free[0, 0] = False
    distance = clearance_field(free, 0.2)
    assert abs(distance[1, 2] - np.sqrt(5) * 0.2) < 1e-12
    assert distance[0, 0] == 0
    np.testing.assert_array_equal(clearance_field(np.zeros((3, 4), bool), .2), 0)
    np.testing.assert_allclose(clearance_field(np.ones((3, 4), bool), .2), .8)


def test_inflation_matches_disc_and_invalidates_for_mutation_and_snapshot():
    grid = ExplorationGrid(4, 3, .2)
    rng = np.random.default_rng(4)
    grid.data[rng.random(grid.data.shape) < .1] = 100
    for radius in (0, .2, .35, .6):
        expected = set()
        for y, x in np.argwhere(grid.data == 100):
            for dy in range(-4, 5):
                for dx in range(-4, 5):
                    if np.hypot(dx * .2, dy * .2) <= radius + 1e-9 and grid.in_bounds((x + dx, y + dy)):
                        expected.add((x + dx, y + dy))
        result = grid.inflated_obstacles(radius)
        assert result == expected
        assert grid.inflated_obstacles(radius) is result
    snapshot = grid.snapshot()
    original = snapshot.inflated_obstacles(.6)
    assert original is grid.inflated_obstacles(.6)
    grid.data[:] = -1
    assert grid.inflated_obstacles(.6) == set()
    assert snapshot.inflated_obstacles(.6) == original
    next_snapshot = snapshot.snapshot()
    assert next_snapshot.inflated_obstacles(.6) is snapshot.inflated_obstacles(.6)
    snapshot.resolution = .1
    assert snapshot.inflated_obstacles(.6) is not original


def test_straight_corridor_compresses_to_one_edge_without_losing_connectivity():
    occupancy = np.full((15, 25), 100, dtype=np.int8)
    occupancy[5:10, 2:23] = 0

    graph = extract_topology(occupancy, 0.2, (0.0, 0.0))
    metrics = oracle_metrics(occupancy, 0.2, graph)

    assert len(graph.nodes) == 2
    assert len(graph.edges) == 1
    assert metrics["connectivity_recall"] == 1.0
    assert metrics["max_cost_error"] < 1e-9


def test_junction_survives_thinning_and_dense_oracle_queries_remain_connected():
    occupancy = np.full((21, 21), 100, dtype=np.int8)
    occupancy[8:13, 2:19] = 0
    occupancy[2:11, 8:13] = 0

    graph = extract_topology(occupancy, 0.2, (-2.0, -2.0))
    metrics = oracle_metrics(occupancy, 0.2, graph)

    assert any(node.degree >= 3 for node in graph.nodes.values())
    assert metrics["queries"] > 0
    assert metrics["connectivity_recall"] == 1.0


def test_stable_node_ids_follow_world_coordinates_across_patch_origins():
    first = np.full((11, 21), 100, dtype=np.int8)
    first[3:8, 2:19] = 0
    second = np.full((11, 21), 100, dtype=np.int8)
    second[3:8, 1:18] = 0

    first_graph = extract_topology(first, 0.2, (0.0, 0.0))
    second_graph = extract_topology(second, 0.2, (0.2, 0.0))

    assert set(first_graph.nodes) == set(second_graph.nodes)
