"""Raw clearance provenance, observation-order invariance and sparse access."""
import copy
import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from explorer_core.grid import FREE, OCCUPIED, UNKNOWN
from explorer_core.wavefront_graph import IncrementalWavefrontGraph
from test_spatial_domains import router_for, floor_identity


def build(grid):
    e = IncrementalWavefrontGraph()
    e.update(grid, .2, (0., 0.))
    e.routing_view()
    return e


def partition(e):
    groups = {}
    for c, pid in floor_identity(e).items():
        groups.setdefault(pid, set()).add(c)
    return {frozenset(cells) for cells in groups.values()}


def test_clearance_is_a_cell_area_lower_bound_and_does_not_repeat_inflation():
    grid = np.full((20, 30), OCCUPIED, dtype=np.int8)
    grid[10, :] = FREE
    e = build(grid)
    facts = e.clearance_evidence((15, 10))
    assert facts['sources'] == ['occupied']
    assert facts['radius'] == pytest.approx(.2-.2/math.sqrt(2.))
    assert 0 < facts['radius'] <= .1  # true square boundary distance
    assert e.occupancy[(15, 10)] == FREE  # no body-radius dilation
    meta = e.routing_view().supports['clearance']
    assert meta['cell_error'] == pytest.approx(.2/math.sqrt(2.))
    assert meta['geometry_revision'] == e.routing_revision
    assert meta['projection'] == 'local_uninflated_body_space'


def test_unknown_and_field_limit_are_distinguished_and_cannot_make_wall_necks():
    grid = np.zeros((60, 100), dtype=np.int8)
    e = build(grid)
    assert e.clearance_evidence((50, 30))['sources'] == ['field_limit']
    assert e.clearance_evidence((0, 30))['sources'] == ['unknown']
    assert e.spatial_view.metrics['spatial_measured_neck_edges'] == 0
    meta = e.routing_view().supports['clearance']
    assert not any('occupied' in item['sources'] for item in meta['edges'].values())
    assert len(partition(e)) == 1


@pytest.mark.parametrize('pillars', [False, True])
def test_nonconvex_place_identity_depends_on_final_evidence_not_scan_order(pillars):
    grid = np.zeros((40, 60), dtype=np.int8)
    if pillars:
        grid[5::10, 5::10] = OCCUPIED
    else:
        grid[20, 30] = OCCUPIED
    first = build(grid)
    later = build(np.zeros_like(grid))
    old = floor_identity(later)
    later.update(grid, .2, (0., 0.))
    assert first.occupancy == later.occupancy
    assert partition(first) == partition(later)
    assert len(partition(first)) == 1
    assert all(floor_identity(later)[c] == pid for c, pid in old.items() if later.occupancy[c] == FREE)
    assert all(first.occupancy[c] == FREE for cells in partition(first) for c in cells)


def test_measured_doorway_splits_places_but_unknown_never_pretends_to_be_a_wall():
    grid = np.zeros((40, 60), dtype=np.int8)
    grid[:, 29:31] = OCCUPIED
    grid[18:22, 29:31] = FREE
    first = build(grid)
    later = build(np.zeros_like(grid)); later.update(grid, .2, (0., 0.))
    assert partition(first) == partition(later)
    assert len(partition(first)) == 2
    assert first.spatial_view.metrics['spatial_measured_neck_edges'] > 0
    router, allowed = router_for(first)
    route = router.batch_estimates((1., 1.), [(11., 1.)], .1, connector_allowed=allowed)[0]
    assert route is not None  # spatial separator is not a route veto
    assert all(allowed(a, b) for a, b in zip(route.polyline, route.polyline[1:]))
    grid[:, 29:31] = UNKNOWN; grid[18:22, 29:31] = FREE
    unknown = build(grid)
    assert len(partition(unknown)) == 1
    assert unknown.spatial_view.metrics['spatial_measured_neck_edges'] == 0


def test_bent_connection_keeps_geometry_and_far_support_attachment_without_dense_search():
    grid = np.full((40, 60), UNKNOWN, dtype=np.int8)
    grid[:20, :] = FREE; grid[:, 40:] = FREE
    e = build(grid); view = e.routing_view()
    assert len(view.nodes) == 2 and len(view.edges) == 1
    refs = [item for item in view.supports['domains'] if 'edge_attachment' in item]
    assert refs and e.spatial_view.metrics['spatial_compressed_nodes'] > 0
    router, allowed = router_for(e)
    start, target = (.1, .1), (11.9, 7.9)
    assert not allowed(start, target)
    route = router.batch_estimates(start, [target], .01, connector_allowed=allowed)[0]
    assert route is not None and len(route.polyline) > 2
    assert all(allowed(a, b) for a, b in zip(route.polyline, route.polyline[1:]))
    assert all(k in view.nodes for k in (route.source_node_id, route.target_node_id))
    # Query exclusively through a removed support anchor, outside all sample buckets.
    item = refs[0]
    cells = [(x, y) for y, x0, x1 in item['rows'] for x in range(x0, x1+1)]
    point = max((((x+.5)*.2, (y+.5)*.2) for x, y in cells),
                key=lambda p: min(math.dist(p, n.position) for n in router.nodes.values()))
    assert item['id'] not in router.nodes
    assert router.attachments(point, .001, connector_allowed=allowed)
    # Old references cannot reconnect a new edge whose anchor index has moved.
    eid, _ = item['edge_attachment']
    router.remove_edge(eid)
    assert item['id'] not in router._support_domains(point)


def test_real_routes_around_obstacle_survive_compression_and_one_side_closure():
    grid = np.zeros((40, 60), dtype=np.int8); grid[10:30, 25:35] = OCCUPIED
    e = build(grid); v = e.routing_view()
    # A real alternative remains a graph cycle. Endpoint-pair compression may
    # retain a third node instead of overwriting two homotopically distinct arcs.
    assert len(v.edges) >= len(v.nodes)
    def query():
        r, allowed = router_for(e)
        route = r.batch_estimates((1., 4.), [(11., 4.)], .1, connector_allowed=allowed)[0]
        assert route is not None
        assert all(allowed(a, b) for a, b in zip(route.polyline, route.polyline[1:]))
    query()
    grid[:10, 25:35] = OCCUPIED; e.update(grid, .2, (0., 0.)); query()
    grid[30:, 25:35] = OCCUPIED; e.update(grid, .2, (0., 0.))
    r, allowed = router_for(e)
    assert r.batch_estimates((1., 4.), [(11., 4.)], .1, connector_allowed=allowed)[0] is None


def test_evidence_revision_and_unchanged_geometry_cache_are_explicit():
    e = build(np.zeros((40, 60), dtype=np.int8)); view = e.routing_view()
    e.update(np.zeros((40, 60), dtype=np.int8), .2, (0., 0.))
    assert e.routing_view() is view
    grid = np.zeros((40, 60), dtype=np.int8); grid[20, 30] = OCCUPIED
    e.update(grid, .2, (0., 0.)); new = e.routing_view()
    assert new.supports['clearance']['geometry_revision'] > view.supports['clearance']['geometry_revision']
    assert new.supports['spatial_revision'] == e.spatial_view.revision


def test_single_tile_occluded_anchor_uses_existing_wavefront_witness_not_search():
    grid = np.zeros((10, 10), dtype=np.int8); grid[4:6, 4:6] = OCCUPIED
    e = build(grid); view = e.routing_view()
    assert len(view.nodes) == 1 and not view.edges
    payload = view.supports; domain = payload['domains'][0]
    tree = domain['connector_tree']
    assert len(tree['steps']) == 100
    router, allowed = router_for(e)
    points = [((x+.5)*.2, (y+.5)*.2) for y in range(10) for x in range(10) if grid[y, x] == FREE]
    anchor = tuple(domain['anchor'])
    blocked = [p for p in points if not allowed(p, anchor)]
    assert blocked
    for start in blocked:
        target = points[-1]
        route = router.batch_estimates(start, [target], .001, connector_allowed=allowed)[0]
        assert route is not None
        assert all(allowed(a, b) for a, b in zip(route.polyline, route.polyline[1:]))
    # A corrupt tree is bounded and cannot invent an observed path.
    malformed = copy.deepcopy(payload)
    malformed['domains'][0]['connector_tree']['steps'] = '0'*100
    router.update_supports(malformed)
    start = blocked[0]
    assert router.batch_estimates(start, [anchor], .001, connector_allowed=allowed)[0] is None


def test_removed_nonconvex_support_keeps_connector_shape_in_same_edge_queries():
    grid = np.zeros((10, 40), dtype=np.int8); grid[4:6, 14:16] = OCCUPIED
    e = build(grid); view = e.routing_view()
    domain = next(d for d in view.supports['domains']
                  if 'connector_tree' in d and 'edge_attachment' in d)
    router, allowed = router_for(e)
    points = [((x+.5)*.2, (y+.5)*.2) for y, x0, x1 in domain['rows']
              for x in range(x0, x1+1)]
    blocked = [p for p in points if not allowed(p, tuple(domain['anchor']))]
    assert blocked
    for start in blocked:
        targets = [p for p in points if not allowed(start, p)]
        assert targets
        routes = router.batch_estimates(start, targets, .001, connector_allowed=allowed)
        assert all(route is not None for route in routes)
        assert all(allowed(a, b) for route in routes
                   for a, b in zip(route.polyline, route.polyline[1:]))


def test_chunked_observation_orders_keep_equivalent_nonconvex_partition():
    grid = np.zeros((40, 60), dtype=np.int8); grid[5::10, 5::10] = OCCUPIED
    expected = partition(build(grid))
    for reverse in (False, True):
        e = IncrementalWavefrontGraph()
        offsets = list(range(0, 60, 10))
        if reverse:
            offsets.reverse()
        for x in offsets:
            e.update(grid[:, x:x+10], .2, (x*.2, 0.))
            e.routing_view()
        assert partition(e) == expected
