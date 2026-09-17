import hashlib
import importlib.util
import json
import math
import os
import pathlib
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from collections import Counter, deque

import pytest

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
SCENE_DIR = PACKAGE_ROOT / "maps" / "cropped_scenes" / "tunnel_grid"
GENERATOR = SCENE_DIR / "generate_scene.py"


def canonical_edge(first, second):
    return tuple(sorted((tuple(first), tuple(second))))


def resolve_local_uri(uri, root=SCENE_DIR):
    assert "://" not in uri, uri
    return root / uri


def read_pcd(path):
    lines = path.read_text().splitlines()
    data = lines.index("DATA ascii")
    header = {line.split()[0]: line.split()[1:] for line in lines[:data]
              if line and not line.startswith("#")}
    points = [tuple(map(float, line.split())) for line in lines[data + 1:] if line]
    return header, points


def read_obj(path):
    vertices, normals, faces = [], [], []
    for line in path.read_text().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == "v": vertices.append(tuple(map(float, fields[1:4])))
        elif fields[0] == "vn": normals.append(tuple(map(float, fields[1:4])))
        elif fields[0] == "f": faces.append(tuple(tuple(map(int, item.split("/")[::2])) for item in fields[1:]))
    return vertices, normals, faces


@pytest.fixture(scope="module")
def scene():
    return ET.parse(SCENE_DIR / "scene.world").getroot(), json.loads((SCENE_DIR / "scene_meta.json").read_text())


def test_world_is_self_contained_and_resources_are_resolvable(scene):
    root, meta = scene
    text = (SCENE_DIR / "scene.world").read_text().lower()
    assert not root.findall(".//include")
    assert "fuel.ignitionrobotics.org" not in text
    assert "tunnel tile" not in text
    assert "blocker" not in text
    assert "blockers" not in meta
    for uri in root.findall(".//uri"):
        assert resolve_local_uri(uri.text).is_file()


def test_graph_preserves_expected_topology_and_spawn(scene):
    _, meta = scene
    nodes = {tuple(item["grid"]) for item in meta["nodes"]}
    edges = {canonical_edge(*edge) for edge in meta["edges"]}
    adjacency = {node: set() for node in nodes}
    for first, second in edges:
        adjacency[first].add(second); adjacency[second].add(first)
        assert abs(first[0] - second[0]) + abs(first[1] - second[1]) == 1
    reached, queue = set(), deque([next(iter(nodes))])
    while queue:
        node = queue.popleft()
        if node not in reached:
            reached.add(node); queue.extend(adjacency[node] - reached)
    assert (len(nodes), len(edges), len(edges) - len(nodes) + 1) == (21, 22, 2)
    assert reached == nodes
    assert sum(len(neighbors) == 1 for neighbors in adjacency.values()) == 2
    spawn_xy = tuple(meta["spawn"][:2])
    assert spawn_xy == (0.0, 0.0)
    spawn_node = next(tuple(item["grid"]) for item in meta["nodes"] if tuple(item["position_m"][:2]) == spawn_xy)
    assert len(adjacency[spawn_node]) >= 2


def test_geometry_contract_and_single_floor_support(scene):
    root, meta = scene
    contract = meta["geometry_contract"]
    assert contract["corridor_width_m"] == pytest.approx(6.0)
    assert 0.4 <= contract["wall_thickness_m"] <= 0.6
    assert contract["wall_height_m"] >= 2.2
    assert contract["junction_shape"] == "square"
    assert contract["junction_width_m"] == pytest.approx(6.0)
    assert contract["inflation_radius_m"] == pytest.approx(0.65)
    assert contract["floor_support_surfaces_per_xy"] == 1
    assert contract["planar_obstacle_z_range_m"] == [0.08, 0.85]
    floors = root.findall(".//world/model[@name='procedural_floor']")
    assert len(floors) == 1
    floor = floors[0]
    assert len(floor.findall("./link/collision")) == len(floor.findall("./link/visual")) == 1
    collision_uri = floor.findtext("./link/collision/geometry/mesh/uri")
    visual_uri = floor.findtext("./link/visual/geometry/mesh/uri")
    assert collision_uri == visual_uri == meta["floor_mesh"]["uri"]


def test_floor_obj_has_material_valid_normals_and_three_bounded_bumps(scene):
    _, meta = scene
    path = resolve_local_uri(meta["floor_mesh"]["uri"])
    text = path.read_text().splitlines()
    assert "mtllib tunnel_floor.mtl" in text and "usemtl tunnel_floor" in text
    assert (path.parent / "tunnel_floor.mtl").is_file()
    vertices, normals, faces = read_obj(path)
    assert vertices and len(normals) == len(vertices) and faces
    assert all(len(face) == 3 for face in faces)
    assert all(vertex == normal for face in faces for vertex, normal in face)
    for normal in normals:
        assert all(math.isfinite(value) for value in normal)
        assert math.sqrt(sum(value * value for value in normal)) == pytest.approx(1.0, abs=1e-6)
    assert [item["target_slope_deg"] for item in meta["terrain_features"]] == [6.0, 8.0, 9.0]
    assert all(5.0 <= item["actual_max_slope_deg"] <= 10.0 for item in meta["terrain_features"])
    assert all(item["boundary_height_m"] == 0.0 for item in meta["terrain_features"])


def model_pose(model):
    return tuple(map(float, (model.findtext("pose") or "0 0 0 0 0 0").split()))


def collision_boxes(root):
    boxes = []
    for model in root.findall(".//world/model"):
        mx, my, mz, _, _, _ = model_pose(model)
        for collision in model.findall("./link/collision"):
            geometry = list(collision.find("geometry"))
            assert len(geometry) == 1, model.get("name")
            kind = geometry[0].tag
            assert kind in {"box", "mesh"}, (model.get("name"), kind)
            if kind == "mesh":
                assert model.get("name") == "procedural_floor"
                continue
            cx, cy, cz, _, _, _ = tuple(map(float, (collision.findtext("pose") or "0 0 0 0 0 0").split()))
            size = tuple(map(float, geometry[0].findtext("size").split()))
            boxes.append((model.get("name"), (mx + cx, my + cy, mz + cz), size))
    return boxes


def nearest_distance(point, cloud):
    return min(math.dist(point, candidate) for candidate in cloud)


def test_every_collision_geometry_has_surface_and_planar_pcd_oracle(scene):
    root, meta = scene
    _, surface = read_pcd(SCENE_DIR / "scene_surface.pcd")
    _, planar = read_pcd(SCENE_DIR / "scene_planar.pcd")
    assert collision_boxes(root)
    for name, center, size in collision_boxes(root):
        x_face = (center[0] + size[0] / 2, center[1], center[2])
        planar_face = (center[0] + size[0] / 2, center[1], min(0.85, max(0.08, center[2])))
        assert nearest_distance(x_face, surface) <= 0.15, name
        if center[2] + size[2] / 2 >= 0.08:
            assert nearest_distance(planar_face, planar) <= 0.15, name
    # Floor is represented only by surface PCD; traversable slope is intentionally absent from planar PCD.
    assert nearest_distance((0.0, 0.0, 0.0), surface) <= 0.01
    assert nearest_distance((0.0, 0.0, 0.3), planar) > 2.0
    assert meta["point_counts"] == {"scene_surface.pcd": len(surface), "scene_planar.pcd": len(planar)}


def test_known_wall_points_are_planar_and_all_edge_centers_are_free(scene):
    _, meta = scene
    _, planar = read_pcd(SCENE_DIR / "scene_planar.pcd")
    assert nearest_distance((-3.5, 0.0, 0.465), planar) <= 0.01
    for first, second in meta["edges"]:
        center = ((first[0] + second[0]) * meta["spacing"] / 2,
                  (first[1] + second[1]) * meta["spacing"] / 2, 0.4)
        assert nearest_distance(center, planar) > 0.5, (first, second)


def independent_clearance(rock, corridor_width, inflation):
    first, second = rock["edge"]
    horizontal = first[1] == second[1]
    center_lateral = first[1] * 20.0 if horizontal else first[0] * 20.0
    longitudinal_center = (first[0] + second[0]) * 10.0 if horizontal else (first[1] + second[1]) * 10.0
    widths = []
    for index in range(41):
        longitudinal = longitudinal_center - 10.0 + index * 0.5
        obstacles = []
        for primitive in rock["collision_primitives"]:
            center, size = primitive["center_m"], primitive["size_m"]
            long_axis, lateral_axis = (0, 1) if horizontal else (1, 0)
            if abs(longitudinal - center[long_axis]) <= size[long_axis] / 2:
                obstacles.append((center[lateral_axis] - size[lateral_axis] / 2,
                                  center[lateral_axis] + size[lateral_axis] / 2))
        intervals = [(center_lateral - corridor_width / 2, center_lateral + corridor_width / 2)]
        for obstacle in obstacles:
            split = []
            for low, high in intervals:
                if obstacle[1] <= low or obstacle[0] >= high: split.append((low, high))
                else:
                    if low < obstacle[0]: split.append((low, obstacle[0]))
                    if obstacle[1] < high: split.append((obstacle[1], high))
            intervals = split
        geometric = max(high - low for low, high in intervals)
        widths.append((geometric, max(0.0, geometric - 2 * inflation)))
    return min(value[0] for value in widths), min(value[1] for value in widths)


def test_rock_visuals_use_inline_primitive_collisions_and_independent_clearance(scene):
    root, meta = scene
    models = {model.get("name"): model for model in root.findall(".//world/model")}
    contract = meta["geometry_contract"]
    assert len(meta["rocks"]) == 3
    for rock in meta["rocks"]:
        model = models[rock["name"]]
        assert resolve_local_uri(rock["visual_uri"]).is_file()
        assert model.findtext("./link/visual/geometry/mesh/uri") == rock["visual_uri"]
        assert len(model.findall("./link/collision/geometry/box")) in (2, 3)
        for primitive in rock["collision_primitives"]:
            bbox = [[center - size / 2, center + size / 2]
                    for center, size in zip(primitive["center_m"], primitive["size_m"])]
            assert all(outer[0] - 1e-9 <= inner[0] <= inner[1] <= outer[1] + 1e-9
                       for outer, inner in zip(rock["visual_world_bbox_m"], bbox))
        geometric, inflated = independent_clearance(rock, contract["corridor_width_m"], contract["inflation_radius_m"])
        assert geometric == pytest.approx(rock["geometric_clearance_m"])
        assert inflated == pytest.approx(rock["inflated_center_clearance_m"])
        assert inflated >= 1.5
    assert meta["minimum_inflated_center_clearance_m"] >= 1.5


def test_output_manifest_and_generator_are_deterministic_and_portable(scene):
    _, meta = scene
    assert all((SCENE_DIR / relative).is_file() for relative in meta["outputs"])
    assert any(relative.endswith("mine_rock_a/meshes/rock.dae") for relative in meta["outputs"])
    with tempfile.TemporaryDirectory() as directory:
        output = pathlib.Path(directory)
        subprocess.run([sys.executable, str(GENERATOR), "--output-dir", str(output)], check=True)
        first_meta = json.loads((output / "scene_meta.json").read_text())
        first = {name: hashlib.sha256((output / name).read_bytes()).hexdigest() for name in first_meta["outputs"]}
        subprocess.run([sys.executable, str(GENERATOR), "--output-dir", str(output)], check=True)
        second = {name: hashlib.sha256((output / name).read_bytes()).hexdigest() for name in first_meta["outputs"]}
        assert first == second
        assert first == {name: hashlib.sha256((SCENE_DIR / name).read_bytes()).hexdigest() for name in first}
        generated_root = ET.parse(output / "scene.world").getroot()
        assert all(resolve_local_uri(uri.text, output).is_file() for uri in generated_root.findall(".//uri"))
