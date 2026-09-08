import pathlib
import sys

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from explorer_core.topology import extract_topology, oracle_metrics  # noqa: E402


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
