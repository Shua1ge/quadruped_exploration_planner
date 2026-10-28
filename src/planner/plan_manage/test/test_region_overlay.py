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


def test_regions_fill_the_floor_with_distinct_colors():
    message = overlay(
        regions=[region(3, [(10, 10), (11, 10), (12, 10)], (1.1, 1.0))],
        region_colors={3: (0.9, 0.4, 0.1)},
        active_region=3,
        accepted=[SimpleNamespace(region_id=3, cell=(11, 10),
                                  goal=(1.1, 1.0), unknown_gain=42)],
        robot=(1.0, 1.0), region_sequence=[3])
    grouped = markers_by_namespace(message)
    fill = grouped["regions"][0]
    assert fill.type == Marker.CUBE_LIST
    assert len(fill.points) == 3
    assert len(fill.colors) == 3
    assert fill.colors[0].a == pytest.approx(0.60)
    assert (fill.colors[0].r, fill.colors[0].g, fill.colors[0].b) == pytest.approx(
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
    assert len(grouped["regions"][0].points) <= 1000
    empty = overlay()
    assert len(empty.markers) == 1  # only DELETEALL


def test_region_area_override_covers_more_than_frontier_cells():
    message = overlay(
        regions=[region(3, [(10, 10)], (1.0, 1.0))],
        region_areas={3: [(10, 10), (11, 10), (11, 11), (10, 11)]},
        cell_to_world=cell_to_world)
    grouped = markers_by_namespace(message)
    assert len(grouped["regions"][0].points) == 4


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
    assert "regions" in namespaces
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
    assert churn[9] == added[2]


def test_region_area_bfs_respects_walls_and_partitions_free_space():
    import numpy as np
    from types import SimpleNamespace as NS

    from frontier_explorer import FrontierExplorer

    data = np.full((12, 30), -1, dtype=np.int8)
    data[4:8, :] = 0
    data[4:8, 15] = 100  # solid wall splits the corridor
    grid = NS(data=data, resolution=0.1,
              in_bounds=lambda cell: 0 <= cell[0] < 30 and 0 <= cell[1] < 12)
    fake = NS(grid=grid, map_content_revision=5)
    left = region(1, [(2, 5)], (0.2, 0.5))
    right = region(2, [(27, 5)], (2.7, 0.5))

    areas = FrontierExplorer.region_area_cells(fake, [left, right])

    assert (2, 5) in areas[1] and (27, 5) in areas[2]
    assert all(cell[0] < 15 for cell in areas[1])
    assert all(cell[0] > 15 for cell in areas[2])
    assert all(4 <= cell[1] <= 7 for cell in areas[1])
    assert (5, 15) not in areas[1] and (5, 15) not in areas[2]
    assert FrontierExplorer.region_area_cells(fake, [left, right]) is areas
