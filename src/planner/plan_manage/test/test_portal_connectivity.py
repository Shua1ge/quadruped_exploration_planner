import json
import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from explorer_core.portal_connectivity import SafeRegionConnectivity  # noqa: E402


def snapshot(with_direct_portal=False):
    portals = [
        {"source": 1, "target": 2, "length": 10.0},
        {"source": 2, "target": 3, "length": 10.0},
    ]
    if with_direct_portal:
        portals.append({"source": 1, "target": 3, "length": 5.0})
    return json.dumps({
        "map_revision": 7,
        "regions": [
            {"id": 1, "x": 0.0, "y": 0.0},
            {"id": 2, "x": 0.0, "y": 5.0},
            {"id": 3, "x": 5.0, "y": 0.0},
        ],
        "portals": portals,
    })


def test_frontier_between_regions_proposes_only_missing_shortcut():
    graph = SafeRegionConnectivity()
    assert graph.update_json(snapshot())

    hypothesis = graph.hypothesis((2.5, 0.0), minimum_savings=4.0)

    assert hypothesis is not None
    assert {hypothesis.source_id, hypothesis.target_id} == {1, 3}
    assert hypothesis.known_distance == 20.0
    assert hypothesis.hypothesized_distance == 5.0
    assert hypothesis.route_savings == 15.0


def test_confirmed_portal_is_never_proposed_again():
    graph = SafeRegionConnectivity()
    assert graph.update_json(snapshot(with_direct_portal=True))

    assert graph.hypothesis((2.5, 0.0), minimum_savings=4.0) is None


def test_malformed_or_older_snapshot_does_not_replace_state():
    graph = SafeRegionConnectivity()
    assert graph.update_json(snapshot())
    assert not graph.update_json("not-json")
    assert not graph.update_json(json.dumps({
        "map_revision": 6, "regions": [], "portals": []}))
    assert set(graph.regions) == {1, 2, 3}
