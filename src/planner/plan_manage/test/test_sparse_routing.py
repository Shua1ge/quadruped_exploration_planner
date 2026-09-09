import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from explorer_core.sparse_routing import (  # noqa: E402
    SparseEdge, SparseNode, SparseRouteGraph)


def straight_graph():
    graph = SparseRouteGraph(bucket_size=1.0)
    graph.upsert_node(SparseNode(1, (0.0, 0.0), 1.0))
    graph.upsert_node(SparseNode(2, (10.0, 0.0), 1.0))
    graph.upsert_edge(SparseEdge(
        11, 1, 2, 10.0, 1.0,
        tuple((float(x), 0.0) for x in range(11))))
    graph.graph_revision = 1
    return graph


def test_mid_corridor_queries_attach_to_edge_not_only_endpoint_nodes():
    graph = straight_graph()

    result = graph.batch_estimates(
        (4.0, 0.2), [(7.0, -0.2)], 0.5,
        connector_allowed=lambda _first, _second: True)[0]

    assert result is not None
    assert abs(result.distance - 3.4) < 1e-9


def test_batch_query_returns_connected_targets_and_rejects_disconnected_one():
    graph = straight_graph()
    graph.upsert_node(SparseNode(3, (20.0, 0.0), 1.0))

    results = graph.batch_estimates(
        (1.0, 0.0), [(9.0, 0.0), (20.0, 0.0)], 0.5)

    assert results[0] is not None
    assert abs(results[0].distance - 8.0) < 1e-9
    assert results[1] is None


def test_node_position_update_preserves_existing_edge_adjacency():
    graph = straight_graph()
    graph.upsert_node(SparseNode(1, (0.0, 0.1), 1.0))

    result = graph.batch_estimates((0.0, 0.1), [(10.0, 0.0)], 0.2)[0]

    assert result is not None
    assert abs(result.distance - 10.0) < 1e-9


def test_delta_removal_disconnects_the_route():
    graph = straight_graph()
    graph.apply_delta(
        2, 5, [], [], [], [], [], [11])

    result = graph.batch_estimates((0.0, 0.0), [(10.0, 0.0)], 0.2)[0]

    assert result is None
