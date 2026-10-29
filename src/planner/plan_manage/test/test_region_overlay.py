import pathlib
import sys
from types import SimpleNamespace

import pytest

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from explorer_core.region_overlay import (  # noqa: E402
    ACTIVE_COLOR, REASON_COLORS, build_region_overlay)


def cell_to_world(cell):
    return (cell[0] * 0.1, cell[1] * 0.1)


def region(region_id, cells, centroid, clusters=1):
    return SimpleNamespace(
        region_id=region_id, cells=cells, centroid=centroid,
        clusters=[list(cells[:2]) for _ in range(clusters)],
        topology_key=None, lineage_id=None)


def overlay(**overrides):
    values = dict(
        frame_id="map", stamp=Time(), regions=[], outcomes=[], accepted=(),
        robot=None, active_goal=None, prepared=None, active_region=None,
        region_sequence=(), cell_to_world=cell_to_world)
    values.update(overrides)
    return build_region_overlay(**values)


def markers_by_namespace(message):
    grouped = {}
    for marker in message.markers:
        grouped.setdefault(marker.ns, []).append(marker)
    return grouped


def test_overlay_clears_previous_markers_first():
    message = overlay()
    assert message.markers[0].action == Marker.DELETEALL


def test_information_groups_show_boundary_points_without_repainting_floor():
    message = overlay(
        regions=[region(3, [(10, 10), (11, 10), (12, 10)], (1.1, 1.0))],
        region_colors={3: (0.9, 0.4, 0.1)},
        active_region=3,
        accepted=[SimpleNamespace(region_id=3, cell=(11, 10),
                                  goal=(1.1, 1.0), unknown_gain=42)],
        robot=(1.0, 1.0), region_sequence=[3])
    grouped = markers_by_namespace(message)
    fill = grouped["information_regions"][0]
    assert fill.type == Marker.POINTS
    assert len(fill.points) == 3
    assert not fill.colors
    assert fill.color.a == pytest.approx(0.85)
    assert (fill.color.r, fill.color.g, fill.color.b) == pytest.approx(
        (0.9, 0.4, 0.1))
    assert "region_labels" not in grouped
    assert "decision" not in grouped
    assert all(marker.type != Marker.TEXT_VIEW_FACING
               for marker in message.markers)
    assert len(grouped["region_sequence"][0].points) == 2


def test_rejected_candidates_are_grouped_by_reason():
    message = overlay(
        regions=[region(1, [(0, 0)], (0.0, 0.0))],
        outcomes=[
            {"region": 1, "viewpoint": (5, 5), "frontier": (6, 6),
             "reason": "astar_unreachable"},
            {"region": 1, "viewpoint": (7, 5), "frontier": (6, 6),
             "reason": "final_validation_failed"},
            {"region": 1, "viewpoint": None, "frontier": (6, 6),
             "reason": "no_safe_viewpoint"},
        ],
        robot=(0.0, 0.0))
    grouped = markers_by_namespace(message)
    assert len(grouped["rejected"]) == 3
    by_color = {marker.color.r: marker for marker in grouped["rejected"]}
    assert REASON_COLORS["astar_unreachable"][0] in by_color
    astar = by_color[REASON_COLORS["astar_unreachable"][0]]
    assert astar.points[0].x == 0.5 and astar.points[0].y == 0.5


def test_prepared_and_active_goal_are_distinct_markers():
    prepared = SimpleNamespace(region_id=7, cell=(100, 50), goal=(10.0, 5.0))
    message = overlay(
        regions=[region(7, [(100, 50)], (10.0, 5.0))],
        prepared=prepared, active_goal=(2.0, 3.0), robot=(1.0, 1.0))
    grouped = markers_by_namespace(message)
    assert grouped["prepared"][0].points[0].x == 10.0
    assert grouped["active_goal"][0].points[0].x == 2.0


def test_region_cell_budget_and_empty_inputs():
    cells = [(x, y) for x in range(100) for y in range(60)]
    message = overlay(
        regions=[region(0, cells, (0.0, 0.0))],
        cell_to_world=cell_to_world, max_cells_per_region=1000)
    grouped = markers_by_namespace(message)
    assert len(grouped["information_regions"][0].points) <= 1000
    empty = overlay()
    assert len(empty.markers) == 1  # only DELETEALL


def test_information_overlay_contains_only_current_information_boundary():
    message = overlay(regions=[region(3, [(10, 10)], (1.0, 1.0))])
    grouped = markers_by_namespace(message)
    assert len(grouped["information_regions"][0].points) == 1
    assert all(m.type != Marker.CUBE_LIST for m in message.markers)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_explorer_publishes_overlay_without_ros_node_context():
    from types import SimpleNamespace as NS

    from frontier_explorer import FrontierExplorer

    publisher = _Publisher()
    fake = NS(
        frame_id="map", position=(1.0, 1.0),
        grid=NS(cell_to_world=cell_to_world, resolution=0.1),
        active_goal=(2.0, 2.0), prepared_candidate=None,
        active_region_id=1, active_commitment_scope=("branch_option", 1),
        region_sequence=[1], last_candidate_outcomes=[],
        region_overlay_pub=publisher, _region_overlay_last_publish=0.0,
        get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Time())),
        get_logger=lambda: NS(warning=lambda *args, **kwargs: None))
    FrontierExplorer.publish_region_overlay(
        fake, [region(1, [(0, 0), (1, 0)], (0.05, 0.0))], [])
    assert len(publisher.messages) == 1
    message = publisher.messages[0]
    namespaces = {marker.ns for marker in message.markers}
    assert "information_regions" in namespaces
    assert "region_labels" not in namespaces and "decision" not in namespaces
    assert message.markers[0].action == Marker.DELETEALL
    assert all(marker.type != Marker.TEXT_VIEW_FACING
               for marker in message.markers)
    # The 0.4 s throttle must not republish in the same update.
    FrontierExplorer.publish_region_overlay(
        fake, [region(1, [(0, 0), (1, 0)], (0.05, 0.0))], [])
    assert len(publisher.messages) == 1


def test_region_colors_persist_while_region_expands():
    from frontier_explorer import FrontierExplorer

    fake = SimpleNamespace(_region_color_history={})
    first = FrontierExplorer.region_color_map(
        fake, [region(1, [(0, 0)], (0.5, 0.5))])
    grown = FrontierExplorer.region_color_map(
        fake, [region(1, [(0, 0), (1, 0)], (0.9, 0.5))])
    assert grown[1] == first[1]
    added = FrontierExplorer.region_color_map(
        fake, [region(1, [(0, 0)], (0.5, 0.5)),
               region(2, [(50, 50)], (3.0, 3.0))])
    assert added[1] == first[1]
    assert added[2] != first[1]
    churn = FrontierExplorer.region_color_map(
        fake, [region(1, [(0, 0)], (0.5, 0.5)),
               region(9, [(50, 50)], (3.05, 3.0))])
    # A different persistent ID is a new region, even at the old centroid.
    assert churn[9] != added[2]
    assert churn[1] == first[1]


def test_frontier_seed_motion_and_new_ids_do_not_allocate_floor_territory():
    first = overlay(regions=[region(1, [(2, 5)], (.2, .5))])
    second = overlay(regions=[region(2, [(20, 5)], (2., .5))])
    for message in (first, second):
        points = [m for m in message.markers if m.ns == "information_regions"]
        assert len(points) == 1 and len(points[0].points) == 1
        assert not any(m.type in (Marker.CUBE_LIST, Marker.TRIANGLE_LIST) for m in message.markers)


def test_region_colors_do_not_collapse_after_twelve_or_depend_on_refresh_order():
    from frontier_explorer import FrontierExplorer

    fake = SimpleNamespace()
    regions = [region(i, [(i, 0)], (float(i), 0.0)) for i in range(47)]
    first = FrontierExplorer.region_color_map(fake, regions)
    assert len(set(first.values())) == 47
    # Filtering, reordered candidates and lower IDs must not steal colours.
    subset = [regions[46], regions[12], regions[0]]
    refreshed = FrontierExplorer.region_color_map(fake, subset)
    assert refreshed == {item.region_id: first[item.region_id] for item in subset}
    assert FrontierExplorer.region_color_map(fake, list(reversed(regions))) == first


def test_region_color_survives_absence_and_centroid_motion(monkeypatch):
    from frontier_explorer import FrontierExplorer
    import frontier_explorer

    fake = SimpleNamespace()
    monkeypatch.setattr(frontier_explorer.time, "monotonic", lambda: 10.0)
    first = FrontierExplorer.region_color_map(fake, [region(18, [(0, 0)], (0., 0.))])
    monkeypatch.setattr(frontier_explorer.time, "monotonic", lambda: 1000.0)
    FrontierExplorer.region_color_map(fake, [])
    returned = FrontierExplorer.region_color_map(
        fake, [region(18, [(80, 0)], (8., 0.)), region(2, [(0, 0)], (0., 0.))])
    assert returned[18] == first[18]
    assert returned[2] != first[18]


def test_split_retains_parent_color_and_assigns_new_child_color():
    from frontier_explorer import FrontierExplorer
    from explorer_core.frontier_regions import FrontierRegion, PersistentRegionTracker

    def observation(cells):
        cells = set(cells)
        centroid = (sum(x for x, _ in cells) / len(cells), 0.)
        return FrontierRegion(-1, [sorted(cells)], centroid, cells)

    tracker = PersistentRegionTracker(match_distance_cells=20.)
    parent = tracker.update([observation((x, 0) for x in range(8))])[0]
    fake = SimpleNamespace()
    original = FrontierExplorer.region_color_map(fake, [parent])[parent.region_id]
    children = tracker.update(
        [observation([(0, 0), (1, 0)]), observation((x, 0) for x in range(3, 8))],
        preferred_region_id=parent.region_id)
    colors = FrontierExplorer.region_color_map(fake, children)
    assert colors[parent.region_id] == original
    assert len(set(colors.values())) == 2
    assert {child.lineage_id for child in children} == {parent.lineage_id}
    survivor = next(child for child in children if child.region_id == parent.region_id)
    assert (7, 0) in survivor.cells


def test_refresh_preserves_marker_slots_and_deletes_only_retired_items():
    first = overlay(
        regions=[region(1, [(0, 0)], (0., 0.)), region(2, [(1, 0)], (.1, 0.))],
        accepted=[SimpleNamespace(cell=(0, 0))])
    keys = {(m.ns, m.id) for m in first.markers if m.action == Marker.ADD}
    refreshed = overlay(
        regions=[region(2, [(1, 0), (2, 0)], (.15, 0.)),
                 region(3, [(3, 0)], (.3, 0.))],
        previous_marker_keys=keys)
    assert not any(m.action == Marker.DELETEALL for m in refreshed.markers)
    assert {(m.ns, m.id) for m in refreshed.markers if m.action == Marker.DELETE} == {
        ("information_regions", 1), ("accepted", 0)}
    # A late subscriber gets every current region in the latest snapshot.
    adds = [m for m in refreshed.markers if m.action == Marker.ADD]
    assert {(m.ns, m.id) for m in adds} == {("information_regions", 2), ("information_regions", 3)}
    assert len(next(m for m in adds if m.id == 2).points) == 2
    emptied = overlay(previous_marker_keys={(m.ns, m.id) for m in adds})
    assert {(m.ns, m.id) for m in emptied.markers} == {("information_regions", 2), ("information_regions", 3)}
    assert all(m.action == Marker.DELETE for m in emptied.markers)


def test_explorer_refresh_does_not_clear_the_whole_layer(monkeypatch):
    from frontier_explorer import FrontierExplorer
    import frontier_explorer

    publisher = _Publisher()
    fake = SimpleNamespace(
        frame_id="map", position=(0., 0.),
        grid=SimpleNamespace(cell_to_world=cell_to_world, resolution=.1),
        region_overlay_pub=publisher,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time())),
        get_logger=lambda: SimpleNamespace(warning=lambda *a, **k: None))
    monkeypatch.setattr(frontier_explorer.time, "monotonic", lambda: 100.)
    FrontierExplorer.publish_region_overlay(fake, [region(1, [(0, 0)], (0., 0.))], [])
    assert any(m.action == Marker.DELETEALL for m in publisher.messages[0].markers)
    monkeypatch.setattr(frontier_explorer.time, "monotonic", lambda: 101.)
    FrontierExplorer.publish_region_overlay(fake, [region(1, [(1, 0)], (.1, 0.))], [])
    assert len(publisher.messages) == 2
    assert all(m.action == Marker.ADD for m in publisher.messages[1].markers)
    assert fake._region_overlay_marker_keys == {("information_regions", 1)}
