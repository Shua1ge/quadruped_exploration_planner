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
    assert result.polyline[0] == (4.0, 0.2)
    assert result.polyline[-1] == (7.0, -0.2)


def test_same_edge_route_preserves_curved_corridor_polyline():
    graph = SparseRouteGraph(bucket_size=1.0)
    graph.upsert_node(SparseNode(1, (0.0, 0.0), 1.0))
    graph.upsert_node(SparseNode(2, (2.0, 2.0), 1.0))
    graph.upsert_edge(SparseEdge(
        11, 1, 2, 4.0, 1.0,
        ((0.0, 0.0), (0.0, 1.0), (0.0, 2.0),
         (1.0, 2.0), (2.0, 2.0))))
    graph.graph_revision = 1

    result = graph.batch_estimates(
        (0.1, 1.0), [(1.9, 2.0)], 0.25)[0]

    assert result is not None
    assert result.polyline == (
        (0.1, 1.0), (0.0, 1.0), (0.0, 2.0),
        (1.0, 2.0), (2.0, 2.0), (1.9, 2.0))


def test_graph_route_preserves_each_edge_polyline_and_connectors():
    graph = SparseRouteGraph(bucket_size=1.0)
    graph.upsert_node(SparseNode(1, (0.0, 0.0), 1.0))
    graph.upsert_node(SparseNode(2, (0.0, 2.0), 1.0))
    graph.upsert_node(SparseNode(3, (2.0, 2.0), 1.0))
    graph.upsert_edge(SparseEdge(
        11, 1, 2, 2.0, 1.0,
        ((0.0, 0.0), (0.0, 1.0), (0.0, 2.0))))
    graph.upsert_edge(SparseEdge(
        12, 2, 3, 2.0, 1.0,
        ((0.0, 2.0), (1.0, 2.0), (2.0, 2.0))))
    graph.graph_revision = 1

    result = graph.batch_estimates(
        (-0.1, 0.0), [(2.1, 2.0)], 0.25)[0]

    assert result is not None
    assert result.polyline == (
        (-0.1, 0.0), (0.0, 0.0), (0.0, 1.0),
        (0.0, 2.0), (1.0, 2.0), (2.0, 2.0), (2.1, 2.0))


def test_batch_query_returns_connected_targets_and_rejects_disconnected_one():
    graph = straight_graph()
    graph.upsert_node(SparseNode(3, (20.0, 0.0), 1.0))

    results = graph.batch_estimates(
        (1.0, 0.0), [(9.0, 0.0), (20.0, 0.0)], 0.5)

    assert results[0] is not None
    assert abs(results[0].distance - 8.0) < 1e-9
    assert results[1] is None


def test_batch_diagnostics_distinguish_unattached_rejected_and_disconnected():
    graph = straight_graph()
    graph.upsert_node(SparseNode(3, (20.0, 0.0), 1.0))
    graph.upsert_node(SparseNode(4, (22.0, 0.0), 1.0))
    graph.upsert_edge(SparseEdge(
        12, 3, 4, 2.0, 1.0,
        ((20.0, 0.0), (21.0, 0.0), (22.0, 0.0))))

    estimates, reasons = graph.batch_estimates_with_reasons(
        (0.0, 0.0), [(7.0, 0.0), (50.0, 0.0), (20.0, 0.0)], 0.5,
        connector_allowed=lambda source, target: source[0] != 7.0)

    assert estimates == [None, None, None]
    assert reasons == ["connector_rejected", "target_unattached", "disconnected"]


def test_batch_diagnostics_report_start_attachment_failure():
    graph = straight_graph()

    estimates, reasons = graph.batch_estimates_with_reasons(
        (50.0, 0.0), [(7.0, 0.0)], 0.5)

    assert estimates == [None]
    assert reasons == ["start_unattached"]


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


def test_topology_attachment_identifies_component_and_corridor_branch():
    graph = straight_graph()

    attachment = graph.topology_attachment((4.0, 0.2), 0.5)

    assert attachment is not None
    assert attachment.component_id == 1
    assert attachment.branch_id == 11
