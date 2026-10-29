import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / 'scripts'))
from explorer_core.grid import FREE, OCCUPIED, UNKNOWN, bresenham
from explorer_core.topology import TopologyGraph, TopologyNode, TopologyEdge, routing_oracle_metrics
from explorer_core.wavefront_graph import IncrementalWavefrontGraph


def check_certified(engine, graph):
    for e in graph.edges.values():
        assert e.cells[0] == graph.nodes[e.source_id].cell
        assert e.cells[-1] == graph.nodes[e.target_id].cell
        raw = []
        for a,b in zip(e.cells,e.cells[1:]):
            raw.extend(bresenham(a,b)[:-1])
        raw.append(e.cells[-1])
        assert all(engine.occupancy.get(p) == FREE for p in raw)
        for a,b in zip(raw,raw[1:]):
            if a[0] != b[0] and a[1] != b[1]:
                assert engine.occupancy.get((a[0],b[1])) == FREE
                assert engine.occupancy.get((b[0],a[1])) == FREE


def test_room_compresses_tile_grid_and_retains_real_query_coverage():
    grid = np.zeros((40,60),dtype=np.int8)
    engine = IncrementalWavefrontGraph()
    engine.update(grid,.2,(0.,0.))
    view = engine.routing_view()
    assert len(view.nodes) <= len(engine.nodes)//2
    assert len(view.edges) < len(engine.edges)
    check_certified(engine,view)
    metrics = routing_oracle_metrics(grid,.2,view,32)
    assert metrics['queries'] == 32
    assert metrics['query_validated_routes'] == 32
    assert metrics['false_positive_rate'] == 0
    assert metrics['space_connector_routes'] == 32
    assert metrics['max_cost_error'] < .2


def test_obstacle_retains_both_sides_and_validates_queries_without_dense_rescue():
    grid = np.zeros((40,60),dtype=np.int8)
    grid[10:30,25:35] = OCCUPIED
    engine = IncrementalWavefrontGraph()
    engine.update(grid,.2,(0.,0.))
    graph = engine.routing_view()
    check_certified(engine,graph)
    assert any(any(25 <= x < 35 and y < 10 for x,y in e.cells) for e in graph.edges.values())
    assert any(any(25 <= x < 35 and y >= 30 for x,y in e.cells) for e in graph.edges.values())
    metrics = routing_oracle_metrics(grid,.2,graph,32)
    assert metrics['query_validated_routes'] == 32
    assert metrics['false_positive_rate'] == 0
    assert metrics['max_cost_error'] < .3
    grid[:,30] = OCCUPIED
    engine.update(grid,.2,(0.,0.))
    check_certified(engine,engine.routing_view())
    metrics = routing_oracle_metrics(grid,.2,engine.routing_view(),32)
    assert metrics['dense_connected'] < metrics['queries']
    assert metrics['false_positive_rate'] == 0


def test_narrow_t_junction_and_unknown_holes_cannot_be_filled_for_compression():
    grid = np.full((40,60),UNKNOWN,dtype=np.int8)
    grid[19:22,1:59] = FREE
    grid[1:22,29:32] = FREE
    engine = IncrementalWavefrontGraph()
    engine.update(grid,.2,(0.,0.))
    view = engine.routing_view()
    check_certified(engine,view)
    assert any(n.degree >= 3 for n in view.nodes.values())
    assert not engine._free_hull([(2,20),(30,2),(57,20)],(30,20))


def test_empty_or_uncovered_queries_do_not_report_perfect_recall():
    graph = TopologyGraph({}, {}, set())
    assert routing_oracle_metrics(np.full((4,4),UNKNOWN),1.,graph)['connectivity_recall'] is None
    metrics = routing_oracle_metrics(np.zeros((4,4)),1.,graph)
    assert metrics['queries'] > 0 and metrics['connectivity_recall'] == 0


def test_oracle_counts_underestimated_cost_and_unsafe_polyline():
    nodes = {1:TopologyNode(1,(0,0),1,1.),2:TopologyNode(2,(9,0),1,1.)}
    edge = TopologyEdge(3,1,2,((0,0),(9,0)),1.,1.)
    graph = TopologyGraph(nodes,{3:edge},set())
    metrics = routing_oracle_metrics(np.zeros((1,10)),1.,graph,1)
    assert metrics['max_cost_error'] == pytest.approx(8/9)
    values = np.zeros((3,10),dtype=np.int8)
    values[:,5] = OCCUPIED
    metrics = routing_oracle_metrics(values,1.,graph,1)
    assert metrics['graph_estimate_hits'] == 1
    assert metrics['graph_validated_routes'] == 0
    assert metrics['false_positive_rate'] == 1


@pytest.mark.parametrize('shape',[(20,20),(12,16)])
def test_free_room_removes_tile_loop_and_partial_boundary_anchors(shape):
    grid=np.zeros(shape,dtype=np.int8)
    engine=IncrementalWavefrontGraph();engine.update(grid,.2,(0.,0.))
    view=engine.routing_view()
    assert len(engine.nodes)==4 and len(engine.edges)==4
    assert len(view.nodes)==1 and not view.edges
    assert min(n.clearance for n in view.nodes.values())==pytest.approx(max(n.clearance for n in engine.nodes.values()))
    check_certified(engine,view)
    metrics=routing_oracle_metrics(grid,.2,view,64)
    assert metrics['query_validated_routes']==64
    assert metrics['false_positive_rate']==0


def test_collapse_keeps_meaningful_neck_and_real_routes_through_it():
    grid=np.zeros((40,60),dtype=np.int8)
    grid[:,20:30]=OCCUPIED;grid[18:22,20:30]=FREE
    engine=IncrementalWavefrontGraph();engine.update(grid,.2,(0.,0.))
    view=engine.routing_view();check_certified(engine,view)
    assert any(20<=n.cell[0]<30 and 18<=n.cell[1]<22 for n in view.nodes.values())
    assert any(e.minimum_clearance<=.4+1e-8 for e in view.edges.values())
    metrics=routing_oracle_metrics(grid,.2,view,64)
    assert metrics['query_validated_routes']==64
    assert metrics['graph_validated_routes']>0 and metrics['false_positive_rate']==0


def test_unknown_hole_keeps_upper_and_lower_route_geometry():
    grid=np.zeros((40,60),dtype=np.int8);grid[10:30,25:35]=UNKNOWN
    engine=IncrementalWavefrontGraph();engine.update(grid,.2,(0.,0.))
    view=engine.routing_view();check_certified(engine,view)
    assert any(any(25<=x<35 and y<10 for x,y in e.cells) for e in view.edges.values())
    assert any(any(25<=x<35 and y>=30 for x,y in e.cells) for e in view.edges.values())
    metrics=routing_oracle_metrics(grid,.2,view,64)
    assert metrics['query_validated_routes']==64 and metrics['false_positive_rate']==0


def test_attachment_radius_no_longer_forces_spatial_node_density():
    grid=np.zeros((20,20),dtype=np.int8)
    small=IncrementalWavefrontGraph(attachment_radius=1.)
    small.update(grid,.2,(0.,0.));small_view=small.routing_view()
    large=IncrementalWavefrontGraph();large.update(grid,.2,(0.,0.))
    assert small_view.nodes == large.routing_view().nodes
    assert small_view.supports == large.routing_view().supports
    small.update(grid,.2,(0.,0.));assert small.routing_view() is small_view
    with pytest.raises(ValueError):IncrementalWavefrontGraph(attachment_radius=0.)


def test_repeated_pruning_preserves_wide_room_corner_attachments():
    grid=np.zeros((40,60),dtype=np.int8)
    engine=IncrementalWavefrontGraph();engine.update(grid,.2,(0.,0.))
    view=engine.routing_view();check_certified(engine,view)
    metrics=routing_oracle_metrics(grid,.2,view,96)
    assert metrics['query_validated_routes']==96 and metrics['false_positive_rate']==0
    assert len(view.edges)<=len(engine.edges)
