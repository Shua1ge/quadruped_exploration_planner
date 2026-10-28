import pathlib
import math
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / 'scripts'))
from explorer_core.wavefront_graph import IncrementalWavefrontGraph
from explorer_core.grid import FREE, OCCUPIED, UNKNOWN


def apply(engine, grid, origin=(0., 0.)):
    update = engine.update(grid, .2, origin)
    assert set(update.added_nodes).isdisjoint(update.updated_nodes)
    assert set(update.added_edges).isdisjoint(update.updated_edges)
    assert all(e.source_id in engine.nodes and e.target_id in engine.nodes for e in engine.edges.values())
    for edge in engine.edges.values():
        assert engine.nodes[edge.source_id].cell == edge.cells[0]
        assert engine.nodes[edge.target_id].cell == edge.cells[-1]
        assert all(engine.occupancy.get(p) == FREE for p in edge.cells)
        for a, b in zip(edge.cells, edge.cells[1:]):
            dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
            assert max(dx, dy) == 1
            if dx and dy:
                assert engine.occupancy[(a[0], b[1])] == FREE
                assert engine.occupancy[(b[0], a[1])] == FREE
        assert edge.length == pytest.approx(
            sum(math.dist(a, b) for a, b in zip(edge.cells, edge.cells[1:])) * .2)
    return update


def connected(engine, a, b):
    seen, queue = {a}, [a]
    while queue:
        node = queue.pop()
        for edge_id in engine.adjacency[node]:
            e = engine.edges[edge_id]
            nxt = e.target_id if e.source_id == node else e.source_id
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    return b in seen


def test_repeat_scans_do_no_wavefront_work_and_never_accumulate_graphs():
    e = IncrementalWavefrontGraph()
    grid = np.zeros((20, 50), dtype=np.int8)
    apply(e, grid)
    before = dict(e.nodes), dict(e.edges)
    for _ in range(100):
        result = apply(e, grid)
        assert result.metrics['wavefront_dirty_tiles'] == 0
        assert not result.added_nodes and not result.updated_nodes
    assert before == (e.nodes, e.edges)
    assert len(e.nodes) == 10
    assert len(e.edges) == 13


def test_routing_compression_preserves_corridor_cost_and_caches_static_geometry():
    e = IncrementalWavefrontGraph()
    apply(e, np.zeros((10, 70), dtype=np.int8))
    view = e.routing_view()
    assert len(e.nodes) == 7 and len(view.nodes) < len(e.nodes)
    assert len(view.edges) == len(view.nodes)-1
    assert sum(x.length for x in view.edges.values()) == pytest.approx(sum(x.length for x in e.edges.values()))
    assert sum(len(x.cells) for x in view.edges.values()) < sum(len(x.cells) for x in e.edges.values())
    for edge in view.edges.values():
        assert view.nodes[edge.source_id].cell == edge.cells[0]
        assert view.nodes[edge.target_id].cell == edge.cells[-1]
        from explorer_core.grid import bresenham
        assert all(e.occupancy.get(p) == FREE for a, b in zip(edge.cells, edge.cells[1:]) for p in bresenham(a,b))
        assert all(math.dist(a,b)*e.resolution <= .5+1e-8 for a,b in zip(edge.cells,edge.cells[1:]))
    apply(e, np.zeros((10, 70), dtype=np.int8))
    assert e.routing_view() is view


def test_actual_travel_reuses_safe_seed_without_new_occupancy_or_identity():
    e = IncrementalWavefrontGraph()
    grid = np.zeros((10, 30), dtype=np.int8)
    apply(e, grid)
    original_nodes, original_facts = dict(e.nodes), dict(e.occupancy)
    # y=1.1 is the equally safe central row not selected by the tie-break.
    assert e.record_traversal((.5, 1.1), (5.3, 1.1))
    update = apply(e, grid)
    assert update.metrics['wavefront_changed_cells'] == 0
    assert update.updated_nodes and update.updated_edges
    assert set(e.nodes) == set(original_nodes) and e.occupancy == original_facts
    assert all(n.cell[1] == 5 for n in e.nodes.values())
    assert not e.record_traversal((.5, 1.1), (5.3, 1.1))
    assert apply(e, grid).metrics['wavefront_dirty_tiles'] == 0
    grid[5, 12] = OCCUPIED
    apply(e, grid)
    assert (12,5) not in e.traversed_cells
    assert not e.record_traversal((.5, 1.1), (5.3, 1.1))


def test_travel_cannot_create_unknown_or_corner_connections():
    e = IncrementalWavefrontGraph()
    grid = np.full((20,20), UNKNOWN, dtype=np.int8)
    grid[9,9] = grid[10,10] = FREE
    apply(e, grid)
    assert not e.record_traversal((1.9,1.9),(2.1,2.1))
    assert not e.record_traversal((1.9,1.9),(3.,3.))
    assert not e.traversed_cells and not e.experience_dirty


def test_seed_position_changes_keep_identity_and_replace_incident_edges():
    e = IncrementalWavefrontGraph()
    grid = np.zeros((10, 30), dtype=np.int8)
    apply(e, grid)
    original = set(e.nodes)
    middle = next(n for n in e.nodes.values() if 10 <= n.cell[0] < 20)
    grid[middle.cell[1], middle.cell[0]] = OCCUPIED
    result = apply(e, grid)
    assert set(e.nodes) == original
    assert middle.node_id in result.updated_nodes
    assert e.nodes[middle.node_id].cell != middle.cell
    for _ in range(25):
        grid[0, 12] = OCCUPIED if grid[0, 12] == FREE else FREE
        apply(e, grid)
        assert len(e.nodes) == 3 and len(e.edges) == 2


def test_newly_observed_component_recenters_existing_identity():
    e = IncrementalWavefrontGraph()
    grid = np.full((10, 20), UNKNOWN, dtype=np.int8)
    grid[0:3, :20] = FREE
    apply(e, grid)
    initial = dict(e.nodes)
    grid[:, :] = FREE
    changed = apply(e, grid)
    assert set(e.nodes) == set(initial)
    assert changed.updated_nodes
    assert changed.updated_edges
    assert any(e.nodes[key].cell != node.cell for key, node in initial.items())
    for node in e.nodes.values():
        # Representative stays in the middle of its observed component.
        assert 4 <= node.cell[1] <= 5


def test_metric_edges_use_diagonals_without_inventing_connectivity():
    e = IncrementalWavefrontGraph()
    grid = np.full((10, 20), UNKNOWN, dtype=np.int8)
    grid[:7, :10] = FREE
    grid[3:, 10:] = FREE
    apply(e, grid)
    assert any(abs(a[0]-b[0]) == abs(a[1]-b[1]) == 1
               for edge in e.edges.values() for a, b in zip(edge.cells, edge.cells[1:]))


def test_closure_split_and_reopen_change_routes_not_just_revision():
    e = IncrementalWavefrontGraph()
    grid = np.zeros((10, 30), dtype=np.int8)
    apply(e, grid)
    left = next(n.node_id for n in e.nodes.values() if n.cell[0] < 10)
    right = next(n.node_id for n in e.nodes.values() if n.cell[0] >= 20)
    assert connected(e, left, right)
    grid[:, 14] = OCCUPIED
    result = apply(e, grid)
    assert not connected(e, left, right)
    assert result.removed_edges and result.added_nodes
    assert any(len(children) == 2 for children in result.lineage.values())
    grid[4, 14] = FREE
    result = apply(e, grid)
    assert connected(e, left, right)
    assert result.removed_nodes


def test_rolling_unknown_and_window_shift_do_not_destroy_history():
    e = IncrementalWavefrontGraph()
    apply(e, np.zeros((10, 20), dtype=np.int8), origin=(-2., -2.))
    original = set(e.nodes)
    result = apply(e, np.full((10, 20), UNKNOWN, dtype=np.int8), origin=(-2., -2.))
    assert not result.removed_nodes and set(e.nodes) == original
    apply(e, np.zeros((10, 20), dtype=np.int8), origin=(0., -2.))
    assert original <= set(e.nodes) and len(e.nodes) == 3


def test_corner_contact_does_not_invent_a_connection():
    e = IncrementalWavefrontGraph()
    grid = np.full((20, 20), OCCUPIED, dtype=np.int8)
    grid[9, 9] = grid[10, 10] = FREE
    apply(e, grid)
    assert len(e.nodes) == 2 and not e.edges


def test_small_change_does_not_propagate_through_entire_persistent_map():
    e = IncrementalWavefrontGraph()
    grid = np.zeros((100, 100), dtype=np.int8)
    initial = apply(e, grid)
    grid[50, 50] = OCCUPIED
    changed = apply(e, grid)
    assert changed.metrics['wavefront_dirty_tiles'] == 9
    assert changed.metrics['wavefront_expanded_cells'] < initial.metrics['wavefront_expanded_cells']/5


def test_invalid_patch_is_atomic_and_resolution_change_is_rejected():
    e = IncrementalWavefrontGraph()
    apply(e, np.zeros((5, 5), dtype=np.int8))
    before = dict(e.occupancy), dict(e.nodes), dict(e.edges)
    for grid, resolution, origin in ((np.array([[3]]), .2, (0., 0.)),
                                     (np.array([[FREE]]), .3, (0., 0.)),
                                     (np.array([[FREE]]), .2, (.13, 0.))):
        with pytest.raises(ValueError):
            e.update(grid, resolution, origin)
        assert before == (e.occupancy, e.nodes, e.edges)


def test_random_dynamic_edits_keep_dense_and_sparse_connectivity_equal():
    rng = np.random.default_rng(7)
    e = IncrementalWavefrontGraph(tile_size=1.)
    grid = np.where(rng.random((20, 20)) > .25, FREE, OCCUPIED).astype(np.int8)
    for _ in range(20):
        grid[rng.integers(20), rng.integers(20)] = rng.choice((FREE, OCCUPIED))
        apply(e, grid)
        from scipy.ndimage import label
        labels, _ = label(grid == FREE)
        nodes = list(e.nodes.values())
        for _ in range(12):
            a, b = rng.choice(len(nodes), size=2)
            p, q = nodes[a], nodes[b]
            assert connected(e, p.node_id, q.node_id) == (labels[p.cell[1], p.cell[0]] == labels[q.cell[1], q.cell[0]])


def test_real_corner_and_height_change_keep_fine_anchors_without_inventing_occupancy():
    engine = IncrementalWavefrontGraph()
    grid = np.zeros((40, 60), dtype=np.int8)
    apply(engine, grid)
    ordinary = engine.routing_view()
    facts, fine_ids = dict(engine.occupancy), set(engine.nodes)
    removed_id = next(k for k in engine.nodes if k not in ordinary.nodes)
    seed = engine.nodes[removed_id].cell
    x,y = ((v+.5)*engine.resolution for v in seed)
    assert engine.record_traversal((x-.4, y), (x, y))
    assert engine.record_traversal((x, y), (x, y+.4))
    apply(engine, grid)
    complex_view = engine.routing_view()
    assert engine.complex_traversal_cells
    assert removed_id in complex_view.nodes and removed_id not in ordinary.nodes
    retained = {k for k,n in engine.nodes.items()
                if engine.tiles[engine.tile_key(n.cell)].components[k] & engine.complex_traversal_cells}
    assert retained <= set(complex_view.nodes)
    assert engine.occupancy == facts and set(engine.nodes) == fine_ids
    assert engine.record_traversal((7.1, 5.1), (9.1, 5.1), .4)
    apply(engine, grid)
    assert (40,25) in engine.complex_traversal_cells
    grid[25,40] = OCCUPIED
    apply(engine,grid)
    assert (40,25) not in engine.complex_traversal_cells


def test_straight_flat_travel_does_not_pin_fine_density_or_promote_jitter():
    engine = IncrementalWavefrontGraph()
    grid = np.zeros((20,60),dtype=np.int8)
    apply(engine,grid)
    for start,end in [((.5,2.1),(3.5,2.1)),((3.5,2.1),(6.5,2.1))]:
        assert engine.record_traversal(start,end,.01)
    apply(engine,grid)
    assert not engine.complex_traversal_cells
    view=engine.routing_view()
    assert len(view.nodes)<len(engine.nodes)
    assert not engine.record_traversal((3.,10.),(6.,10.),.5)
    assert engine.traversal_direction is None
    assert not engine.complex_traversal_cells
