"""Geometry, identity and query regressions for the public spatial view."""
import math
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from explorer_core.wavefront_graph import IncrementalWavefrontGraph
from explorer_core.grid import FREE, OCCUPIED, UNKNOWN
from explorer_core.sparse_routing import SparseRouteGraph, SparseNode, SparseEdge
from explorer_core.topology import routing_oracle_metrics


def router_for(engine):
    view = engine.routing_view(); router = SparseRouteGraph()
    world = lambda c: tuple((v+.5)*engine.resolution for v in c)
    router.apply_delta(1, 1, [SparseNode(k, world(n.cell), n.clearance) for k, n in view.nodes.items()], [], [],
        [SparseEdge(k, e.source_id, e.target_id, e.length, e.minimum_clearance,
                    tuple(world(c) for c in e.cells)) for k, e in view.edges.items()], [], [])
    router.update_supports(view.supports)
    def allowed(a, b):
        first, last = (tuple(math.floor(v/engine.resolution) for v in p) for p in (a, b))
        return engine._free_segment(first, last) is not None
    return router, allowed


def floor_identity(engine):
    engine.routing_view()
    return {c: domain.place_id for domain in engine.spatial_view.domains.values() for c in domain.cells}


def test_flat_heave_turns_and_repeated_travel_do_not_change_geometry_or_hue():
    grid = np.zeros((40, 60), dtype=np.int8)
    engine = IncrementalWavefrontGraph(); engine.update(grid, .2, (0., 0.))
    view, colors = engine.routing_view(), floor_identity(engine)
    for i in range(45):
        x = .5+.2*i
        engine.record_traversal((x, 1.), (x+.2, 1.), .05 if i % 2 else -.05, stamp=100.+i)
    engine.record_traversal((2., 1.), (2., 2.), .4, stamp=150.)
    update = engine.update(grid, .2, (0., 0.))
    assert update.metrics['wavefront_dirty_tiles'] == 0
    assert engine.routing_view() is view
    assert floor_identity(engine) == colors
    assert engine.traversed_cells and not engine.complex_traversal_cells


def test_platform_corner_queries_use_support_not_an_unbounded_radius_or_dense_rescue():
    engine = IncrementalWavefrontGraph(); grid = np.zeros((40, 60), dtype=np.int8)
    engine.update(grid, .2, (0., 0.)); view = engine.routing_view()
    assert len(view.nodes) == 1 and not view.edges
    router, allowed = router_for(engine)
    assert router.ready
    start, target = (.1, .1), (11.9, 7.9)
    assert router.batch_estimates(start, [target], .1)[0] is None
    route = router.batch_estimates(start, [target], .1, connector_allowed=allowed)[0]
    assert route is not None and route.route_source == 'space_connector'
    assert route.distance == math.dist(start, target)
    assert route.polyline == (start, target)
    assert router.batch_estimates(start, [target], .1, connector_allowed=lambda a, b: False)[0] is None
    metrics = routing_oracle_metrics(grid, .2, view, 96, attachment_radius=.1)
    assert metrics['query_validated_routes'] == 96 and metrics['false_positive_rate'] == 0


def test_free_growth_keeps_existing_world_floor_identity_and_anchor():
    engine = IncrementalWavefrontGraph()
    engine.update(np.zeros((20, 20), dtype=np.int8), .2, (0., 0.))
    before = floor_identity(engine); old = dict(engine.routing_view().nodes)
    engine.update(np.zeros((20, 20), dtype=np.int8), .2, (2., 0.))
    after = floor_identity(engine)
    assert all(after[c] == d for c, d in before.items())
    assert engine.routing_view().nodes == old
    engine.update(np.full((20, 20), UNKNOWN, dtype=np.int8), .2, (0., 0.))
    assert floor_identity(engine) == after


def test_wall_split_inherits_one_identity_and_reopen_keeps_the_survivor():
    engine = IncrementalWavefrontGraph(); grid = np.zeros((20, 30), dtype=np.int8)
    engine.update(grid, .2, (0., 0.)); original = set(engine.routing_view().nodes)
    grid[:, 14] = OCCUPIED
    engine.update(grid, .2, (0., 0.)); view = engine.routing_view()
    assert original & view.nodes.keys()
    assert len(view.nodes) >= 2
    router, allowed = router_for(engine)
    assert router.batch_estimates((.5, 1.), [(5., 1.)], .5, connector_allowed=allowed)[0] is None
    assert all((14, y) not in floor_identity(engine) for y in range(20))
    grid[:, 14] = FREE
    engine.update(grid, .2, (0., 0.))
    assert original & engine.routing_view().nodes.keys()
    router, allowed = router_for(engine)
    assert router.batch_estimates((.5, 1.), [(5., 1.)], .5, connector_allowed=allowed)[0] is not None


def test_unrelated_component_update_keeps_domain_object_and_edge_witnesses():
    engine = IncrementalWavefrontGraph()
    grid = np.zeros((20, 30), dtype=np.int8)
    engine.update(grid, .2, (0., 0.)); engine.update(grid, .2, (40., 0.))
    before = floor_identity(engine)
    distant = {d: v for d, v in engine.spatial_view.domains.items() if v.anchor[0] >= 200}
    grid[5, 5] = OCCUPIED
    engine.update(grid, .2, (0., 0.)); after = floor_identity(engine)
    assert distant
    assert all(engine.spatial_view.domains[d] is v for d, v in distant.items())
    assert all(after[c] == d for c, d in before.items() if c[0] >= 200)
    assert engine.spatial_view.metrics['spatial_touched_fine_nodes'] < len(engine.nodes)


def test_local_dynamic_edits_match_dense_connectivity_without_wall_shortcuts():
    rng = np.random.default_rng(13)
    grid = np.zeros((30, 50), dtype=np.int8); grid[8:23, 20:30] = OCCUPIED
    engine = IncrementalWavefrontGraph()
    for step in range(10):
        if step:
            x, y = int(rng.integers(50)), int(rng.integers(30))
            grid[y, x] = OCCUPIED if grid[y, x] == FREE else FREE
        engine.update(grid, .2, (0., 0.)); view = engine.routing_view()
        assert len(floor_identity(engine)) == int((grid == FREE).sum())
        for edge in view.edges.values():
            assert view.nodes[edge.source_id].cell == edge.cells[0]
            assert view.nodes[edge.target_id].cell == edge.cells[-1]
            assert all(engine._free_segment(a, b) is not None for a, b in zip(edge.cells, edge.cells[1:]))
        metrics = routing_oracle_metrics(grid, .2, view, 48)
        assert metrics['query_validated_routes'] == metrics['dense_connected']
        assert metrics['false_positive_rate'] == 0


def test_outdated_support_anchor_and_snapshot_cannot_revive_old_connections():
    engine = IncrementalWavefrontGraph(); engine.update(np.zeros((20, 20), dtype=np.int8), .2, (0., 0.))
    router, allowed = router_for(engine); payload = dict(engine.routing_view().supports, source_map_revision=20)
    assert router.update_supports(payload)
    assert not router.update_supports(dict(payload, source_map_revision=19))
    node_id = next(iter(router.nodes)); node = router.nodes[node_id]
    router.upsert_node(SparseNode(node_id, (node.position[0]+.2, node.position[1]), node.clearance))
    assert not router.support_nodes((.1, .1))


def test_discovering_branch_adds_route_anchors_without_recolouring_old_platform():
    engine = IncrementalWavefrontGraph()
    platform = np.zeros((40, 60), dtype=np.int8)
    engine.update(platform, .2, (0., 0.))
    before = floor_identity(engine)
    node_ids = set(engine.routing_view().nodes)
    expanded = np.full((80, 60), UNKNOWN, dtype=np.int8)
    expanded[:40, :] = FREE
    expanded[40:, 25:35] = FREE
    engine.update(expanded, .2, (0., 0.))
    after = floor_identity(engine)
    assert len(engine.routing_view().nodes) > len(node_ids)
    assert all(after[c] == pid for c, pid in before.items())
    # A narrowing into UNKNOWN adds route coverage, not a measured doorway.
    assert set(after.values()) == set(before.values())


def test_pillar_changes_route_supports_but_does_not_split_connected_floor_identity():
    engine = IncrementalWavefrontGraph()
    grid = np.zeros((40, 60), dtype=np.int8)
    engine.update(grid, .2, (0., 0.))
    before = floor_identity(engine)
    grid[10:30, 25:35] = OCCUPIED
    engine.update(grid, .2, (0., 0.))
    after = floor_identity(engine)
    assert len(engine.routing_view().nodes) > 1
    assert all(after[c] == before[c] for c in after)


def test_coverage_limit_can_add_route_anchors_without_adding_floor_colours():
    engine = IncrementalWavefrontGraph()
    engine.update(np.zeros((80, 180), dtype=np.int8), .2, (0., 0.))
    assert len(engine.routing_view().nodes) > 1
    assert len(set(floor_identity(engine).values())) == 1
