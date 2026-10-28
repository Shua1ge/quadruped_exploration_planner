import pathlib
import sys
from types import SimpleNamespace

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from explorer_core.terrain_graph import (  # noqa: E402
    FLAT, OVERHEAD, ROUGH, STALE, STEPS, UNKNOWN, TerrainTileStore,
    decayed_experience, experience_summary)
from explorer_core.sparse_routing import (  # noqa: E402
    SparseEdge, SparseNode, SparseRouteGraph)


def tile(tx, ty, **overrides):
    values = dict(
        tile_x=tx, tile_y=ty, tile_size=2.0,
        ground_min=0.0, ground_median=0.0, ground_max=0.0,
        slope=0.01, roughness=0.005, support_cells=10,
        rise_cells=0, drop_cells=0, overhead_cells=0,
        measured_columns=100, last_scan=10, content_revision=1)
    values.update(overrides)
    return SimpleNamespace(**values)


def store_with(*summaries, scan=10, **kwargs):
    store = TerrainTileStore(**kwargs)
    store.update(scan, summaries)
    return store


def test_flat_tile_aggregates_to_measured_ground():
    store = store_with(tile(0, 0, ground_max=0.04))
    aggregate = store.aggregate([(1.0, 1.0), (1.5, 0.5)])
    assert aggregate["state"] == FLAT
    assert aggregate["measured_columns"] == 100
    assert aggregate["ground_min"] == 0.0
    assert aggregate["ground_max"] == pytest.approx(0.04)
    assert aggregate["last_scan"] == 10


def test_empty_or_unmeasured_footprint_is_unknown():
    store = store_with()
    aggregate = store.aggregate([(5.0, 5.0)])
    assert aggregate["state"] == UNKNOWN
    assert aggregate["measured_columns"] == 0
    unmeasured = store_with(tile(0, 0, measured_columns=0))
    assert unmeasured.aggregate([(1.0, 1.0)])["state"] == UNKNOWN


@pytest.mark.parametrize("overrides,expected", [
    (dict(rise_cells=3), STEPS),
    (dict(drop_cells=1), STEPS),
    (dict(overhead_cells=2), OVERHEAD),
    (dict(slope=0.4), ROUGH),
    (dict(roughness=0.2), ROUGH),
])
def test_state_classification(overrides, expected):
    store = store_with(tile(0, 0, **overrides))
    assert store.aggregate([(1.0, 1.0)])["state"] == expected


def test_freshness_downgrades_then_drops_tiles():
    store = store_with(tile(0, 0), stale_scans=20, drop_scans=50)
    store.update(35, [])
    aggregate = store.aggregate([(1.0, 1.0)])
    assert aggregate["state"] == STALE
    assert aggregate["measured_columns"] == 100  # last known value survives
    store.update(70, [])
    aggregate = store.aggregate([(1.0, 1.0)])
    assert aggregate["state"] == UNKNOWN
    assert aggregate["measured_columns"] == 0


def test_occupancy_change_deletes_terrain_not_refreshed_this_scan():
    store = store_with(tile(0, 0), scan=10)
    store.observe_occupancy((0, 0), 1, 10)  # first observation: no invalidation
    assert (0, 0) in store.tiles
    store.observe_scan(11)
    store.observe_occupancy((0, 0), 2, 11)
    assert (0, 0) not in store.tiles  # geometry moved, facts were older
    refreshed = store_with(tile(0, 0, last_scan=11), scan=11)
    refreshed.observe_occupancy((0, 0), 1, 11)
    refreshed.observe_occupancy((0, 0), 2, 11)
    assert (0, 0) in refreshed.tiles  # measured by this same patch


def test_traversability_is_a_bounded_preference_never_a_wall():
    rough_store = store_with(tile(0, 0, slope=0.9, roughness=0.5))
    rough = rough_store.aggregate([(1.0, 1.0)])
    price = rough_store.traversability(rough)
    assert 1.0 < price <= 2.0
    flat_store = store_with(tile(0, 0))
    flat = flat_store.aggregate([(1.0, 1.0)])
    assert flat_store.traversability(flat) == pytest.approx(1.0)
    unknown_store = store_with()
    unknown_price = unknown_store.traversability(
        unknown_store.aggregate([(9.0, 9.0)]))
    assert 1.0 < unknown_price <= 2.0
    preferred = flat_store.traversability(
        flat, traversals=5, experience_score=1.0)
    assert 0.8 <= preferred < 1.0
    for state_aggregate, store in ((flat, flat_store), (rough, rough_store)):
        for traversals, score in ((0, 0.0), (100, 1.0)):
            value = store.traversability(state_aggregate, traversals, score)
            assert 0.8 <= value <= 2.0


def test_experience_decay_and_summary():
    assert decayed_experience(3, 100.0, 100.0, 300.0) == pytest.approx(3.0)
    assert decayed_experience(3, 0.0, 300.0, 300.0) == pytest.approx(1.5)
    records = {(0, 0): [6, 300.0], (1, 0): [1, 0.0]}
    traversals, score = experience_summary(
        records, [(0, 0), (1, 0)], now=300.0, half_life=300.0)
    assert traversals == 7
    assert 0.0 < score <= 1.0
    assert experience_summary({}, [(0, 0)], 0.0, 300.0) == (0, 0.0)


def _parallel_graph(direct_length, direct_trav, detour_trav):
    """Direct corridor versus an 8 m + 8 m detour through node 3."""
    graph = SparseRouteGraph(bucket_size=1.0)
    for node_id, point in ((1, (0.0, 0.0)), (2, (direct_length, 0.0)),
                           (3, (0.0, 8.0))):
        graph.upsert_node(SparseNode(node_id, point, 1.0))
    graph.upsert_edge(SparseEdge(
        11, 1, 2, direct_length, 1.0,
        ((0.0, 0.0), (direct_length, 0.0)), traversability=direct_trav))
    graph.upsert_edge(SparseEdge(
        12, 1, 3, 8.0, 1.0, ((0.0, 0.0), (0.0, 8.0)),
        traversability=detour_trav))
    graph.upsert_edge(SparseEdge(
        13, 3, 2, 8.0, 1.0,
        ((0.0, 8.0), (8.0, 0.0), (direct_length, 0.0))))
    return graph


def test_route_cost_neutral_edges_keep_geometric_distance():
    graph = _parallel_graph(10.0, 1.0, 1.0)
    estimate = graph.batch_estimates((0.0, 0.0), [(10.0, 0.0)], 0.5)[0]
    assert estimate.distance == pytest.approx(10.0)
    assert estimate.terrain_cost == pytest.approx(0.0)
    assert [connection[0] for connection in estimate.connections] == [11]


def test_route_cost_prefers_measured_low_cost_corridor():
    graph = _parallel_graph(16.0, 1.0, 0.8)
    estimate = graph.batch_estimates((0.0, 0.0), [(16.0, 0.0)], 0.5)[0]
    # 8 m at 0.8 + 8 m at 1.0 = 14.4 beats the 16 m direct edge.
    assert estimate.distance == pytest.approx(14.4)
    assert [connection[0] for connection in estimate.connections] == [12, 13]
    assert estimate.terrain_cost == pytest.approx(-1.6)


def test_route_cost_demotes_rough_direct_edge():
    graph = SparseRouteGraph(bucket_size=1.0)
    graph.upsert_node(SparseNode(1, (0.0, 0.0), 1.0))
    graph.upsert_node(SparseNode(2, (10.0, 0.0), 1.0))
    graph.upsert_edge(SparseEdge(
        11, 1, 2, 10.0, 1.0,
        tuple((float(x), 0.0) for x in range(11)), traversability=2.0))
    estimate = graph.batch_estimates((0.0, 0.0), [(10.0, 0.0)], 0.5)[0]
    assert estimate.distance == pytest.approx(20.0)
    assert estimate.terrain_cost == pytest.approx(10.0)


def test_out_of_range_traversability_is_rejected():
    graph = SparseRouteGraph(bucket_size=1.0)
    graph.upsert_node(SparseNode(1, (0.0, 0.0), 1.0))
    graph.upsert_node(SparseNode(2, (2.0, 0.0), 1.0))
    with pytest.raises(ValueError):
        graph.upsert_edge(SparseEdge(
            11, 1, 2, 2.0, 1.0, ((0.0, 0.0), (2.0, 0.0)),
            traversability=float("nan")))
    with pytest.raises(ValueError):
        graph.upsert_edge(SparseEdge(
            11, 1, 2, 2.0, 1.0, ((0.0, 0.0), (2.0, 0.0)),
            traversability=9.0))
