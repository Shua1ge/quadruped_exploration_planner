import importlib.util
import json
import pathlib
import sys
import pytest
from types import SimpleNamespace


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location(
    "global_representation_node",
    ROOT / "scripts" / "global_representation_node.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_node():
    node = object.__new__(MODULE.GlobalRepresentationNode)
    node.last_map_revision = 17
    node.route_constraint_revision = 0
    node.viewpoint_constraint_radius = 0.6
    node.max_viewpoint_constraints = 4
    node.viewpoint_constraints = {}
    node.failure_evidence_received = 0
    node.failure_evidence_forwarded = 0
    node.global_safe_regions = {
        9: {"id": 9, "x": 4.0, "y": 2.0, "area": 3.0}}
    node.global_safe_portals = {}
    node.safe_region_pub = Publisher()
    node.failure_evidence_pub = Publisher()
    node.get_logger = lambda: SimpleNamespace(
        warning=lambda *args, **kwargs: None,
        info=lambda *args, **kwargs: None)
    return node


def evidence(request_id=42, scope=None, x=4.1, y=2.0):
    return SimpleNamespace(
        request_id=request_id,
        task_id=0, task_version=0, failed_object_id=0,
        trajectory_id=8,
        failure_scope=(MODULE.FailureEvidence.SCOPE_VIEWPOINT
                       if scope is None else scope),
        position=SimpleNamespace(x=x, y=y, z=0.4),
        required_clearance=0.1,
        stage="STRUCTURED_LOCAL_REPAIR",
        reason="FINITE_CANDIDATE_FAMILY_EXHAUSTED")


def test_same_task_viewpoint_is_idempotent_across_path_requests():
    node = make_node()
    first = evidence(42)
    first.task_id, first.task_version, first.failed_object_id = 100, 1, 7
    node.failure_evidence_callback(first)
    second = evidence(43)
    second.task_id, second.task_version, second.failed_object_id = 100, 1, 7
    node.failure_evidence_callback(second)
    assert node.route_constraint_revision == 1
    assert len(node.viewpoint_constraints) == 1


def test_viewpoint_failure_becomes_graph_constraint_before_forwarding():
    node = make_node()
    message = evidence()

    node.failure_evidence_callback(message)

    assert node.route_constraint_revision == 1
    assert node.viewpoint_constraints[42]["region_id"] == 9
    assert node.failure_evidence_pub.messages == [message]
    snapshot = json.loads(node.safe_region_pub.messages[-1].data)
    assert snapshot["route_constraint_revision"] == 1
    assert snapshot["viewpoint_constraints"][0]["request_id"] == 42


def test_duplicate_viewpoint_failure_is_idempotent_but_forwarded():
    node = make_node()
    message = evidence()

    node.failure_evidence_callback(message)
    node.failure_evidence_callback(message)

    assert node.route_constraint_revision == 1
    assert len(node.safe_region_pub.messages) == 1
    assert node.failure_evidence_pub.messages == [message, message]


def test_path_scoped_failure_is_forwarded_without_graph_constraint():
    node = make_node()
    message = evidence(scope=MODULE.FailureEvidence.SCOPE_PATH)
    message.task_id, message.task_version = 100, 2
    message.has_path_segment = True
    message.path_start = SimpleNamespace(x=1.0, y=2.0, z=0.0)
    message.path_end = SimpleNamespace(x=2.0, y=2.0, z=0.0)

    node.failure_evidence_callback(message)

    assert node.route_constraint_revision == 0
    assert node.viewpoint_constraints == {}
    assert node.safe_region_pub.messages == []
    assert node.failure_evidence_pub.messages == [message]


def test_path_scoped_failure_without_identity_or_segment_is_rejected():
    node = make_node()
    message = evidence(scope=MODULE.FailureEvidence.SCOPE_PATH)

    node.failure_evidence_callback(message)

    assert node.failure_evidence_pub.messages == []
    assert node.failure_evidence_forwarded == 0


def test_graph_markers_replace_stable_slots_and_remove_stale_evidence():
    from geometry_msgs.msg import Point
    from std_msgs.msg import Header
    from visualization_msgs.msg import Marker
    node = make_node()
    node.global_node_messages = {
        1: SimpleNamespace(position=Point(x=1., y=2., z=.3)),
        2: SimpleNamespace(position=Point(x=2., y=2., z=.3))}
    node.global_edge_messages = {
        3: SimpleNamespace(source_id=1, target_id=2,
                           polyline=[v.position for v in node.global_node_messages.values()]),
        4: SimpleNamespace(source_id=2, target_id=999, polyline=[])}
    node.graph_vis_pub = Publisher()
    node.viewpoint_constraints = {1: {"x": 2., "y": 3.}}
    node.publish_graph_markers(Header(frame_id="world"))
    markers = node.graph_vis_pub.messages[-1].markers
    assert [m.id for m in markers] == [0, 1, 2, 3]
    assert [len(m.points) for m in markers] == [2, 2, 1, 0]
    assert all(m.action == Marker.ADD for m in markers)
    node.viewpoint_constraints.clear()
    node.publish_graph_markers(Header(frame_id="world"))
    assert len(node.graph_vis_pub.messages[-1].markers[2].points) == 0


def test_malformed_patch_does_not_advance_global_revision():
    from scan_planner_msgs.msg import LocalMapPatch
    node = make_node()
    for bad in (float("nan"), float("inf"), 0., -1.):
        patch = LocalMapPatch(map_revision=18, width=1, height=1,
                              resolution=bad, occupancy=[0])
        patch.header.frame_id = "world"
        node.patch_callback(patch)
        assert node.last_map_revision == 17


def test_nonfinite_patch_origin_is_rejected_before_extraction():
    from scan_planner_msgs.msg import LocalMapPatch
    patch = LocalMapPatch(map_revision=18, width=1, height=1,
                          resolution=.2, occupancy=[0])
    patch.header.frame_id = "world"
    patch.origin.x = float("nan")
    node = make_node()
    node.patch_callback(patch)
    assert node.last_map_revision == 17


def make_wavefront_node():
    from explorer_core.wavefront_graph import IncrementalWavefrontGraph
    from explorer_core.topology_persistence import ExplicitBlockageTracker
    node = make_node()
    from explorer_core.terrain_graph import TerrainTileStore
    node.spatial_engine = 'wavefront'
    node.wavefront = IncrementalWavefrontGraph(2.)
    node.wavefront_tile_size = 2.0
    node.query_attachment_radius = 3.0
    node.patch_inflation_radius = 0.0
    node.node_attribute_radius = 0.5
    node.experience_half_life = 300.0
    node.terrain = TerrainTileStore(2.0)
    node.node_attribute_hash = {}
    node.edge_attribute_hash = {}
    node.terrain_reset_count = 0
    node.graph_changes = ({}, {}, set(), {}, {}, set())
    node.wavefront_compact_routes = False
    node.last_map_revision = node.graph_revision = 0
    node.max_oracle_queries = 1
    node.topology_oracle_period_revisions = 10
    node.last_topology_oracle = {}
    node.safe_region_shadow_enabled = False
    node.min_free_cells_for_destructive_update = 16
    node.node_blockage = ExplicitBlockageTracker(3)
    node.edge_blockage = ExplicitBlockageTracker(3)
    node.global_nodes, node.global_edges = {}, {}
    node.global_node_messages, node.global_edge_messages = {}, {}
    node.snapshot_period_revisions = 10
    for name in ('delta_pub', 'snapshot_pub', 'graph_vis_pub', 'metrics_pub'):
        setattr(node, name, Publisher())
    return node


def test_wavefront_delta_replaces_explorer_copy_and_keeps_evidence_on_split_merge():
    import numpy as np
    from explorer_core.sparse_routing import SparseRouteGraph, SparseNode, SparseEdge
    from scan_planner_msgs.msg import LocalMapPatch
    node = make_wavefront_node()
    mirror = SparseRouteGraph()
    grid = np.zeros((10, 30), dtype=np.int8)
    node.failure_evidence_callback(evidence())
    constraints = dict(node.viewpoint_constraints)

    def publish_patch():
        patch = LocalMapPatch(map_revision=node.last_map_revision+1,
                              width=30, height=10, resolution=.2,
                              occupancy=grid.ravel().tolist())
        patch.header.frame_id = 'world'
        node.patch_callback(patch)
        delta = node.delta_pub.messages[-1]
        def nodes(messages):
            return [SparseNode(n.id, (n.position.x, n.position.y), n.clearance)
                    for n in messages]
        def edges(messages):
            return [SparseEdge(e.id, e.source_id, e.target_id, e.length,
                               e.minimum_clearance,
                               tuple((p.x, p.y) for p in e.polyline)) for e in messages]
        mirror.apply_delta(delta.graph_revision, delta.source_map_revision,
                           nodes(delta.added_nodes), nodes(delta.updated_nodes),
                           delta.removed_node_ids, edges(delta.added_edges),
                           edges(delta.updated_edges), delta.removed_edge_ids)
        assert set(mirror.nodes) == set(node.wavefront.nodes)
        assert set(mirror.edges) == set(node.wavefront.edges)
        for key, message in node.global_edge_messages.items():
            assert mirror.edges[key].polyline == tuple((p.x, p.y) for p in message.polyline)
        assert node.viewpoint_constraints == constraints
        assert node.route_constraint_revision == 1
        return delta

    publish_patch()
    seed = next(n for n in node.wavefront.nodes.values() if 10 <= n.cell[0] < 20)
    grid[seed.cell[1], seed.cell[0]] = 100
    moved = publish_patch()
    assert seed.node_id in {n.id for n in moved.updated_nodes}
    assert moved.updated_edges
    grid[:, 14] = 100
    closed = publish_patch()
    assert closed.removed_edge_ids and len(mirror.nodes) == 4
    grid[4, 14] = 0
    reopened = publish_patch()
    assert reopened.removed_node_ids and len(mirror.nodes) == 3
    unchanged = publish_patch()
    assert not unchanged.added_nodes and not unchanged.updated_edges
    metrics = json.loads(node.metrics_pub.messages[-1].data)
    assert metrics['wavefront_dirty_tiles'] == 0
    assert metrics['global_nodes'] == 3 and metrics['global_edges'] == 2
    assert all(p.z == .1 for m in node.graph_vis_pub.messages[-1].markers for p in m.points)


def test_compacted_public_delta_survives_closure_and_reopening_in_explorer_copy():
    import numpy as np
    from explorer_core.sparse_routing import SparseRouteGraph, SparseNode, SparseEdge
    from scan_planner_msgs.msg import LocalMapPatch
    node=make_wavefront_node(); node.wavefront_compact_routes=True
    mirror=SparseRouteGraph()
    grid=np.zeros((10,70),dtype=np.int8)
    for phase in range(4):
        if phase==1: grid[:,34]=100
        if phase==2: grid[4,34]=0
        patch=LocalMapPatch(map_revision=phase+1,width=70,height=10,resolution=.2,
                            occupancy=grid.ravel().tolist())
        patch.header.frame_id='world'
        node.patch_callback(patch)
        delta=node.delta_pub.messages[-1]
        ns=lambda messages:[SparseNode(n.id,(n.position.x,n.position.y),n.clearance) for n in messages]
        es=lambda messages:[SparseEdge(e.id,e.source_id,e.target_id,e.length,e.minimum_clearance,
            tuple((p.x,p.y) for p in e.polyline)) for e in messages]
        mirror.apply_delta(delta.graph_revision,delta.source_map_revision,
            ns(delta.added_nodes),ns(delta.updated_nodes),delta.removed_node_ids,
            es(delta.added_edges),es(delta.updated_edges),delta.removed_edge_ids)
        assert set(mirror.nodes)==set(node.global_nodes)
        assert set(mirror.edges)==set(node.global_edges)
        assert all(e.source_id in mirror.nodes and e.target_id in mirror.nodes for e in mirror.edges.values())
        route=mirror.batch_estimates((1.1,1.1),[(12.1,1.1)],.5)[0]
        assert (route is None)==(phase==1)
        if phase==0: assert len(node.global_nodes)<len(node.wavefront.nodes)
        if phase==1: assert delta.removed_edge_ids
        if phase==3: assert not delta.added_nodes and not delta.updated_edges


def test_measured_slow_travel_updates_global_but_jitter_pose_jump_and_stale_data_do_not():
    import numpy as np
    from nav_msgs.msg import Odometry
    node=make_wavefront_node()
    node.wavefront.update(np.zeros((10,30),dtype=np.int8),.2,(0.,0.))
    node.traversal_frame='world'
    now=SimpleNamespace(nanoseconds=1_000_000_000)
    node.get_clock=lambda:SimpleNamespace(now=lambda:now)
    def sample(x, y=1.1, age=0, frame='world'):
        message=Odometry(); message.header.frame_id=frame
        stamp=now.nanoseconds-age
        message.header.stamp.sec=stamp//1000000000
        message.header.stamp.nanosec=stamp%1000000000
        message.pose.pose.position.x=x; message.pose.pose.position.y=y
        node.traversal_callback(message)
    sample(.5)
    for i in range(1,8):
        now.nanoseconds+=100_000_000
        sample(.5+i*.017)  # Effective velocity 0.17 m/s, not the command.
    assert node.wavefront.experience_revision==1
    node.wavefront.update(np.zeros((10,30),dtype=np.int8),.2,(0.,0.))
    revision=node.wavefront.experience_revision
    for i in range(20):
        now.nanoseconds+=100_000_000; sample(.62+(i%2)*.002)
    assert node.wavefront.experience_revision==revision and not node.wavefront.experience_dirty
    now.nanoseconds+=100_000_000; sample(4.)  # Teleport is not travel.
    assert node.wavefront.experience_revision==revision
    now.nanoseconds+=100_000_000; sample(4.2,age=800_000_000)
    assert node.traversal_anchor is None
    now.nanoseconds+=100_000_000; sample(4.2,frame='odom')
    assert node.traversal_anchor is None


def test_odom_travel_uses_sample_time_and_translated_rotated_world_tf():
    import math
    import numpy as np
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    node = make_wavefront_node()
    node.wavefront.update(np.zeros((20,30),dtype=np.int8),.2,(0.,0.))
    node.traversal_frame = 'world'
    now = SimpleNamespace(nanoseconds=1_000_000_000)
    node.get_clock = lambda: SimpleNamespace(now=lambda:now)
    calls = []
    transform = TransformStamped()
    transform.transform.translation.x = 2.
    transform.transform.translation.y = 1.
    transform.transform.rotation.z = math.sin(math.pi/4)
    transform.transform.rotation.w = math.cos(math.pi/4)
    def lookup(target, source, stamp):
        calls.append((target,source,stamp.nanoseconds))
        return transform
    node.traversal_tf_buffer = SimpleNamespace(lookup_transform=lookup)
    for x in (0., .12):
        message = Odometry()
        message.header.frame_id = 'go2/odom'
        message.header.stamp.sec = now.nanoseconds//1_000_000_000
        message.header.stamp.nanosec = now.nanoseconds%1_000_000_000
        message.pose.pose.position.x = x
        node.traversal_callback(message)
        now.nanoseconds += 100_000_000
    assert calls == [('world','go2/odom',1_000_000_000),('world','go2/odom',1_100_000_000)]
    assert node.wavefront.experience_revision == 1
    assert node.traversal_anchor[1] == pytest.approx((2.,1.12))
    assert node.traversal_transformed_samples == 2
    def missing(*args):
        raise MODULE.TransformException('sample-time transform unavailable')
    node.traversal_tf_buffer.lookup_transform = missing
    message.header.stamp.nanosec = 200_000_000
    node.traversal_callback(message)
    assert node.traversal_anchor is None
    assert node.wavefront.experience_revision == 1
    assert node.traversal_transform_failures == 1


def test_unknown_only_patch_reports_no_oracle_cohort():
    from scan_planner_msgs.msg import LocalMapPatch
    node = make_wavefront_node()
    patch = LocalMapPatch(map_revision=1,width=4,height=4,resolution=.2,
                          occupancy=[-1]*16)
    patch.header.frame_id = 'world'
    node.patch_callback(patch)
    metrics = json.loads(node.metrics_pub.messages[-1].data)
    assert metrics['queries'] == 0
    assert metrics['connectivity_recall'] is None


def test_published_room_graph_and_markers_use_the_same_compacted_geometry():
    import numpy as np
    from scan_planner_msgs.msg import LocalMapPatch
    node=make_wavefront_node();node.wavefront_compact_routes=True
    patch=LocalMapPatch(map_revision=1,resolution=.2,width=20,height=20,occupancy=[0]*400)
    patch.header.frame_id='world'
    update=node.wavefront.update(np.zeros((20,20),dtype=np.int8),.2,(0.,0.))
    node.commit_wavefront_update(patch,update)
    assert len(node.wavefront.nodes)==4
    assert len(node.global_nodes)==1 and not node.global_edges
    node.publish_graph_markers(patch.header)
    markers=node.graph_vis_pub.messages[-1].markers
    assert len(markers[0].points)==len(node.global_node_messages)
    assert len(markers[1].points)==sum(2*(len(e.polyline)-1) for e in node.global_edge_messages.values())
    assert all(e.source_id in node.global_nodes and e.target_id in node.global_nodes for e in node.global_edges.values())


@pytest.mark.parametrize("mode", ["shadow", "off"])
def test_observation_modes_cannot_apply_global_planning_inputs(mode):
    from frontier_explorer import FrontierExplorer
    # These entrypoints must return before reading a graph or changing state.
    fake = SimpleNamespace(global_usage_mode=mode, sparse_routing_enabled=True)
    FrontierExplorer.failure_evidence_callback(fake, object())
    FrontierExplorer.safe_region_snapshot_callback(fake, object())
    FrontierExplorer.topology_delta_callback(fake, object())
    FrontierExplorer.topology_snapshot_callback(fake, object())
    candidate = SimpleNamespace(commitment_scope=("branch_option", 4))
    FrontierExplorer.attach_safe_region_commitment_scopes(fake, [candidate], set())
    assert candidate.commitment_scope == ("branch_option", 4)
    assert not FrontierExplorer.apply_global_viewpoint_constraint(fake, 1, (0., 0.), .5, "test")
    routes, _ = FrontierExplorer.sparse_route_estimates(fake, (0, 0), [(1, 0)], set(), set())
    assert routes == [None]
    assert vars(fake) == {"global_usage_mode": mode, "sparse_routing_enabled": True}


def local_patch(revision, grid=None, x=0., stamp=100):
    import numpy as np
    from scan_planner_msgs.msg import LocalMapPatch
    values = np.zeros((10, 30), dtype=np.int8) if grid is None else grid
    patch = LocalMapPatch(map_revision=revision, resolution=.2,
                          width=values.shape[1], height=values.shape[0],
                          occupancy=values.ravel().tolist())
    patch.header.frame_id = "world"
    patch.header.stamp.sec = stamp
    patch.origin.x = x
    return patch


def test_global_raw_connectivity_retains_narrow_opening_for_local_certification():
    import numpy as np
    from explorer_core.sparse_routing import SparseRouteGraph, SparseNode, SparseEdge
    node = make_wavefront_node()
    # The obsolete radius must no longer rewrite raw FREE as OCCUPIED.
    node.patch_inflation_radius = .3
    grid = np.zeros((10, 30), dtype=np.int8)
    grid[:, 14] = 100
    grid[4, 14] = 0
    node.patch_callback(local_patch(1, grid))
    assert node.wavefront.occupancy[(14, 4)] == 0
    mirror = SparseRouteGraph()
    mirror.apply_delta(node.graph_revision, 1,
        [SparseNode(m.id, (m.position.x, m.position.y), m.clearance)
         for m in node.global_node_messages.values()], [], [],
        [SparseEdge(m.id, m.source_id, m.target_id, m.length, m.minimum_clearance,
                    tuple((p.x, p.y) for p in m.polyline))
         for m in node.global_edge_messages.values()], [], [])
    assert mirror.batch_estimates((1., 1.), [(5., 1.)], .5)[0] is not None
    # An actually observed closed wall still removes the connection.
    grid[4, 14] = 100
    node.patch_callback(local_patch(2, grid))
    assert node.wavefront.occupancy[(14, 4)] == 100
    closed = SparseRouteGraph()
    closed.apply_delta(node.graph_revision, 2,
        [SparseNode(m.id, (m.position.x, m.position.y), m.clearance)
         for m in node.global_node_messages.values()], [], [],
        [SparseEdge(m.id, m.source_id, m.target_id, m.length, m.minimum_clearance,
                    tuple((p.x, p.y) for p in m.polyline))
         for m in node.global_edge_messages.values()], [], [])
    assert closed.batch_estimates((1., 1.), [(5., 1.)], .5)[0] is None


def test_unchanged_patch_skips_graph_and_attributes_but_keeps_input_liveness(monkeypatch):
    node = make_wavefront_node()
    node.skip_unchanged_patches = True
    node.topology_oracle_enabled = False
    node.patch_callback(local_patch(1))
    before = (dict(node.global_nodes), dict(node.global_edges))
    def forbidden(*args, **kwargs):
        raise AssertionError("unchanged input performed full graph work")
    monkeypatch.setattr(node.wavefront, "update", forbidden)
    monkeypatch.setattr(node, "apply_terrain_attributes", forbidden)
    patch = local_patch(2)
    patch.content_revision = 99  # input counter alone is not a geometry change
    patch.header.stamp.nanosec = 500_000_000
    node.patch_callback(patch)
    metrics = json.loads(node.metrics_pub.messages[-1].data)
    assert metrics["unchanged_patch"] and metrics["input_stamp"] == 100.5
    assert metrics["map_revision"] == 2 and metrics["graph_revision"] == 2
    assert not metrics["topology_oracle_sampled"]
    assert (node.global_nodes, node.global_edges) == before
    delta = node.delta_pub.messages[-1]
    assert not delta.added_nodes and not delta.updated_edges


def test_changed_origin_geometry_and_age_bucket_are_not_deduplicated():
    import numpy as np
    node = make_wavefront_node()
    node.skip_unchanged_patches = True
    node.topology_oracle_enabled = False
    node.patch_callback(local_patch(1))
    key = node._last_patch_key
    node.patch_callback(local_patch(2, x=2.))
    assert node._last_patch_key != key
    assert not json.loads(node.metrics_pub.messages[-1].data)["unchanged_patch"]
    node.patch_callback(local_patch(3, x=2., stamp=102))
    assert not json.loads(node.metrics_pub.messages[-1].data)["unchanged_patch"]
    grid = np.zeros((10, 30), dtype=np.int8)
    grid[:, 14] = 100
    node.patch_callback(local_patch(4, grid, x=2., stamp=102))
    assert not json.loads(node.metrics_pub.messages[-1].data)["unchanged_patch"]


def test_nan_terrain_facts_have_stable_semantics_but_changed_facts_do_not():
    from scan_planner_msgs.msg import TerrainTileSummary
    node = make_wavefront_node()
    node.skip_unchanged_patches = True
    node.topology_oracle_enabled = False
    def patch(revision, slope=float("nan")):
        message = local_patch(revision)
        message.scan = revision
        message.terrain_tiles = [TerrainTileSummary(tile_size=2., ground_min=float("nan"),
            ground_median=float("nan"), ground_max=float("nan"), slope=slope,
            roughness=float("nan"), last_scan=revision, content_revision=revision)]
        return message
    node.patch_callback(patch(1))
    node.patch_callback(patch(2))
    assert json.loads(node.metrics_pub.messages[-1].data)["unchanged_patch"]
    node.patch_callback(patch(3, slope=.2))
    assert not json.loads(node.metrics_pub.messages[-1].data)["unchanged_patch"]


def test_global_patch_admission_keeps_only_latest_pending_input():
    node = make_wavefront_node()
    node._pending_patch = None
    node.received_patches = node.replaced_pending_patches = 0
    for revision in (1, 2, 3):
        node.queue_patch(local_patch(revision))
    assert node.received_patches == 3 and node.replaced_pending_patches == 2
    node.process_pending_patch()
    assert node.last_map_revision == 3 and node._pending_patch is None
    assert len(node.delta_pub.messages) == 1


@pytest.mark.parametrize("mode,enabled", [("off", False), ("shadow", True), ("route_cost", True)])
def test_global_launch_mode_controls_only_global_process(mode, enabled):
    from launch import LaunchContext
    from launch_ros.actions import Node
    spec = importlib.util.spec_from_file_location(
        "newmine_launch_test", ROOT / "launch" / "newmine_exploration.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations["global_usage_mode"] = mode
    global_node = next(action for action in description.entities
                       if isinstance(action, Node)
                       and action.node_executable == "global_representation_node.py")
    assert global_node.condition.evaluate(context) is enabled


def test_spatial_floor_uses_one_stable_identity_and_cached_rectangle_geometry():
    import numpy as np
    from visualization_msgs.msg import Marker
    node = make_wavefront_node(); node.wavefront_compact_routes = True
    node.topology_oracle_enabled = False
    node.patch_callback(local_patch(1, np.zeros((40, 60), dtype=np.int8)))
    fill = node.graph_vis_pub.messages[-1].markers[3]
    assert fill.ns == 'spatial_domains' and fill.type == Marker.TRIANGLE_LIST
    assert len(fill.points) == 6  # whole rectangle, not 2400 floor cubes
    assert len({(c.r, c.g, c.b) for c in fill.colors}) == 1
    # Unrelated Explorer information IDs, goals and active state have no input.
    original_colors = [(c.r, c.g, c.b) for c in fill.colors]
    node.active_region_id = 92
    node.active_goal = (30., 8.)
    node.wavefront.record_traversal((1., 1.), (2., 1.), .5, stamp=100.)
    node.patch_callback(local_patch(2, np.zeros((40, 60), dtype=np.int8)))
    refreshed = node.graph_vis_pub.messages[-1].markers[3]
    assert refreshed is fill
    assert [(c.r, c.g, c.b) for c in refreshed.colors] == original_colors
    # The existing JSON channel carries query support and the map epoch.
    payload = json.loads(node.safe_region_pub.messages[-1].data)['spatial_supports']
    assert len(payload['domains']) == 1 and payload['world_epoch'] == 0


def test_spatial_support_sync_survives_message_order_and_world_epoch_reset():
    import numpy as np
    from frontier_explorer import FrontierExplorer
    from explorer_core.sparse_routing import SparseRouteGraph
    node = make_wavefront_node(); node.wavefront_compact_routes = True
    node.topology_oracle_enabled = False
    node.patch_callback(local_patch(30, np.zeros((20, 20), dtype=np.int8)))
    mirror = SparseRouteGraph()
    support = json.loads(node.safe_region_pub.messages[-1].data)['spatial_supports']
    assert mirror.update_supports(support)  # support arrives before graph
    snapshot = node.snapshot_pub.messages[-1]
    mirror.apply_delta(snapshot.graph_revision, snapshot.source_map_revision,
        [FrontierExplorer.sparse_node_from_message(n) for n in snapshot.added_nodes], [], [],
        [FrontierExplorer.sparse_edge_from_message(e) for e in snapshot.added_edges], [], [])
    assert mirror.ready and mirror.support_nodes((.1, .1))
    node.patch_callback(local_patch(1, np.zeros((20, 20), dtype=np.int8), x=10.))
    replacement = node.snapshot_pub.messages[-1]
    assert replacement.source_map_revision == 1  # forced full reset snapshot
    assert node.terrain_reset_count == 1
    payload = json.loads(node.safe_region_pub.messages[-1].data)['spatial_supports']
    assert mirror.update_supports(payload)  # lower map revision, new epoch
    assert not mirror.update_supports(support)  # delayed old-world support
    assert payload['world_epoch'] == 1
