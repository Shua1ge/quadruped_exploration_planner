"""Deterministic collision-geometry loader for SDF worlds.

The loader intentionally supports only the geometry needed by the benchmark
worlds: COLLADA triangle meshes and SDF boxes.  It applies COLLADA units, scene
node transforms, mesh scale, and SDF collision/link/model poses before sampling
the result.  No Gazebo process is required.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple
import xml.etree.ElementTree as ET

import numpy as np


Point3 = Tuple[float, float, float]
Triangle = Tuple[Point3, Point3, Point3]
COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


@dataclass(frozen=True)
class WorldPointCloud:
    points: List[Point3]
    source_bounds: Tuple[Point3, Point3]
    translation: Point3
    triangle_count: int
    box_count: int


def _pose_matrix(text: str = "") -> np.ndarray:
    values = [float(value) for value in (text or "").split()]
    values.extend([0.0] * (6 - len(values)))
    x, y, z, roll, pitch, yaw = values[:6]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)
    return matrix


def _transform(matrix: np.ndarray, point: Sequence[float]) -> Point3:
    result = matrix @ np.array((point[0], point[1], point[2], 1.0))
    return float(result[0]), float(result[1]), float(result[2])


def _collada_node_matrix(node: ET.Element) -> np.ndarray:
    matrix = np.eye(4)
    for element in node:
        name = element.tag.rsplit("}", 1)[-1]
        values = [float(value) for value in (element.text or "").split()]
        if name == "matrix" and len(values) == 16:
            local = np.asarray(values, dtype=float).reshape((4, 4))
        elif name == "translate" and len(values) >= 3:
            local = np.eye(4)
            local[:3, 3] = values[:3]
        elif name == "scale" and len(values) >= 3:
            local = np.diag((values[0], values[1], values[2], 1.0))
        elif name == "rotate" and len(values) >= 4:
            axis = np.asarray(values[:3], dtype=float)
            norm = np.linalg.norm(axis)
            if norm <= 1e-12:
                continue
            axis /= norm
            angle = math.radians(values[3])
            skew = np.array([[0.0, -axis[2], axis[1]],
                             [axis[2], 0.0, -axis[0]],
                             [-axis[1], axis[0], 0.0]])
            rotation = (np.eye(3) * math.cos(angle)
                        + (1.0 - math.cos(angle)) * np.outer(axis, axis)
                        + math.sin(angle) * skew)
            local = np.eye(4)
            local[:3, :3] = rotation
        else:
            continue
        matrix = matrix @ local
    return matrix


def _collada_geometries(root: ET.Element):
    geometries = {}
    for geometry in root.findall(".//c:library_geometries/c:geometry", COLLADA_NS):
        mesh = geometry.find("c:mesh", COLLADA_NS)
        if mesh is None:
            continue
        sources = {}
        for source in mesh.findall("c:source", COLLADA_NS):
            array = source.find("c:float_array", COLLADA_NS)
            accessor = source.find(".//c:accessor", COLLADA_NS)
            if array is None or accessor is None:
                continue
            stride = int(accessor.get("stride", "1"))
            names = [item.get("name", "")
                     for item in accessor.findall("c:param", COLLADA_NS)]
            if stride < 3 or names[:3] != ["X", "Y", "Z"]:
                continue
            values = [float(value) for value in (array.text or "").split()]
            sources["#" + source.get("id")] = [
                tuple(values[index:index + 3])
                for index in range(0, len(values) - stride + 1, stride)]
        vertices = {}
        for vertex_set in mesh.findall("c:vertices", COLLADA_NS):
            position = next((item for item in vertex_set.findall(
                "c:input", COLLADA_NS) if item.get("semantic") == "POSITION"), None)
            if position is not None:
                vertices["#" + vertex_set.get("id")] = position.get("source")
        triangles = []
        for primitive in mesh.findall("c:triangles", COLLADA_NS):
            inputs = primitive.findall("c:input", COLLADA_NS)
            if not inputs:
                continue
            stride = max(int(item.get("offset", "0")) for item in inputs) + 1
            vertex_input = next((item for item in inputs
                                 if item.get("semantic") == "VERTEX"), None)
            if vertex_input is None:
                continue
            positions = sources[vertices[vertex_input.get("source")]]
            offset = int(vertex_input.get("offset", "0"))
            indices = [int(value) for value in (
                primitive.findtext("c:p", "", COLLADA_NS)).split()]
            selected = indices[offset::stride]
            for index in range(0, len(selected) - 2, 3):
                triangles.append(tuple(positions[item]
                                       for item in selected[index:index + 3]))
        geometries["#" + geometry.get("id")] = triangles
    return geometries


def _walk_collada_nodes(node: ET.Element, parent: np.ndarray,
                         geometries, unit: float) -> Iterable[Triangle]:
    matrix = parent @ _collada_node_matrix(node)
    for instance in node.findall("c:instance_geometry", COLLADA_NS):
        for triangle in geometries.get(instance.get("url"), ()):
            # COLLADA's asset unit applies to the complete authored scene,
            # including translations stored in node matrices, not only to the
            # geometry source arrays.
            yield tuple(tuple(unit * value for value in _transform(matrix, point))
                        for point in triangle)
    for child in node.findall("c:node", COLLADA_NS):
        yield from _walk_collada_nodes(child, matrix, geometries, unit)


def load_collada_triangles(path: Path) -> List[Triangle]:
    root = ET.parse(path).getroot()
    unit_node = root.find("c:asset/c:unit", COLLADA_NS)
    unit = float(unit_node.get("meter", "1.0")) if unit_node is not None else 1.0
    geometries = _collada_geometries(root)
    triangles = []
    for scene in root.findall(".//c:library_visual_scenes/c:visual_scene", COLLADA_NS):
        for node in scene.findall("c:node", COLLADA_NS):
            triangles.extend(_walk_collada_nodes(node, np.eye(4), geometries, unit))
    if not triangles:
        raise RuntimeError(f"No instantiated COLLADA triangles in {path}")
    return triangles


def _normal_z(triangle: Triangle) -> float:
    points = np.asarray(triangle, dtype=float)
    normal = np.cross(points[1] - points[0], points[2] - points[0])
    length = np.linalg.norm(normal)
    return abs(float(normal[2])) / length if length > 1e-12 else 1.0


def _sample_triangle(triangle: Triangle, spacing: float) -> Iterable[Point3]:
    points = np.asarray(triangle, dtype=float)
    longest = max(np.linalg.norm(points[1] - points[0]),
                  np.linalg.norm(points[2] - points[0]),
                  np.linalg.norm(points[2] - points[1]))
    divisions = max(1, int(math.ceil(longest / spacing)))
    for row in range(divisions + 1):
        for column in range(divisions - row + 1):
            u, v = row / divisions, column / divisions
            point = ((1.0 - u - v) * points[0]
                     + u * points[1] + v * points[2])
            yield float(point[0]), float(point[1]), float(point[2])


def _sample_box(size: Sequence[float], matrix: np.ndarray,
                spacing: float, planar: bool) -> Iterable[Point3]:
    sx, sy, sz = (float(value) for value in size)
    nx, ny, nz = (max(1, int(math.ceil(value / spacing)))
                  for value in (sx, sy, sz))
    faces = []
    for ix in range(nx + 1):
        x = -sx / 2.0 + sx * ix / nx
        for iz in range(nz + 1):
            z = -sz / 2.0 + sz * iz / nz
            faces.extend(((x, -sy / 2.0, z), (x, sy / 2.0, z)))
    for iy in range(ny + 1):
        y = -sy / 2.0 + sy * iy / ny
        for iz in range(nz + 1):
            z = -sz / 2.0 + sz * iz / nz
            faces.extend(((-sx / 2.0, y, z), (sx / 2.0, y, z)))
    if not planar:
        for ix in range(nx + 1):
            x = -sx / 2.0 + sx * ix / nx
            for iy in range(ny + 1):
                y = -sy / 2.0 + sy * iy / ny
                faces.extend(((x, y, -sz / 2.0), (x, y, sz / 2.0)))
    return (_transform(matrix, point) for point in faces)


def load_sdf_world(path: Path, spacing: float = 0.2, mode: str = "planar",
                   max_slope_degrees: float = 15.0,
                   obstacle_height: float = 2.5,
                   vertical_spacing: float = 0.2,
                   recenter: bool = True) -> WorldPointCloud:
    if spacing <= 0.0 or vertical_spacing <= 0.0 or obstacle_height <= 0.0:
        raise ValueError("sampling dimensions must be positive")
    if mode not in ("planar", "surface"):
        raise ValueError("mode must be planar or surface")
    world = ET.parse(path).getroot().find("world")
    if world is None:
        raise ValueError(f"No <world> in {path}")
    triangles: List[Triangle] = []
    boxes = []
    for model in world.findall("model"):
        model_matrix = _pose_matrix(model.findtext("pose", ""))
        for link in model.findall("link"):
            link_matrix = model_matrix @ _pose_matrix(link.findtext("pose", ""))
            for collision in link.findall("collision"):
                matrix = link_matrix @ _pose_matrix(collision.findtext("pose", ""))
                mesh = collision.find("geometry/mesh")
                box = collision.find("geometry/box")
                if mesh is not None:
                    uri = (mesh.findtext("uri") or "").strip()
                    mesh_path = Path(uri[7:] if uri.startswith("file://") else uri)
                    if not mesh_path.is_absolute():
                        mesh_path = (path.parent / mesh_path).resolve()
                    scale = [float(value) for value in (
                        mesh.findtext("scale", "1 1 1")).split()]
                    scale.extend([1.0] * (3 - len(scale)))
                    scale_matrix = np.diag((scale[0], scale[1], scale[2], 1.0))
                    for triangle in load_collada_triangles(mesh_path):
                        triangles.append(tuple(_transform(matrix @ scale_matrix, point)
                                               for point in triangle))
                elif box is not None:
                    size = [float(value) for value in (
                        box.findtext("size", "")).split()]
                    if len(size) == 3:
                        boxes.append((size, matrix))
    vertices = [point for triangle in triangles for point in triangle]
    for size, matrix in boxes:
        for sx in (-size[0] / 2.0, size[0] / 2.0):
            for sy in (-size[1] / 2.0, size[1] / 2.0):
                for sz in (-size[2] / 2.0, size[2] / 2.0):
                    vertices.append(_transform(matrix, (sx, sy, sz)))
    if not vertices:
        raise RuntimeError(f"No supported collision geometry in {path}")
    minimum = tuple(float(value) for value in np.min(vertices, axis=0))
    maximum = tuple(float(value) for value in np.max(vertices, axis=0))
    translation = (-0.5 * (minimum[0] + maximum[0]),
                   -0.5 * (minimum[1] + maximum[1]), 0.0) if recenter else (0.0, 0.0, 0.0)
    precision = max(3, int(math.ceil(-math.log10(spacing))) + 1)
    sampled_voxels = set()
    occupied_xy = set()
    slope_limit = math.cos(math.radians(max_slope_degrees))
    planar = mode == "planar"
    for triangle in triangles:
        if planar and _normal_z(triangle) >= slope_limit:
            continue
        for point in _sample_triangle(triangle, spacing):
            x, y, z = point
            if planar:
                occupied_xy.add((round((x + translation[0]) / spacing),
                                 round((y + translation[1]) / spacing)))
            else:
                sampled_voxels.add((
                    round((x + translation[0]) / spacing),
                    round((y + translation[1]) / spacing),
                    round((z + translation[2]) / spacing)))
    for size, matrix in boxes:
        for x, y, z in _sample_box(size, matrix, spacing, planar):
            if planar:
                occupied_xy.add((round((x + translation[0]) / spacing),
                                 round((y + translation[1]) / spacing)))
            else:
                sampled_voxels.add((
                    round((x + translation[0]) / spacing),
                    round((y + translation[1]) / spacing),
                    round((z + translation[2]) / spacing)))
    if planar:
        levels = max(1, int(math.ceil(obstacle_height / vertical_spacing)))
        sampled = {
            (round(ix * spacing, precision), round(iy * spacing, precision),
             round(obstacle_height * level / levels, precision))
            for ix, iy in occupied_xy for level in range(levels + 1)}
    else:
        # COLLADA meshes repeat vertices across neighbouring triangles.
        # Quantize the sampled surface onto the requested 3-D resolution so
        # cave roofs, ramps, and overhangs keep their real height without
        # retaining millions of almost-identical truth-map points.
        sampled = {
            (round(ix * spacing, precision),
             round(iy * spacing, precision),
             round(iz * spacing, precision))
            for ix, iy, iz in sampled_voxels}
    return WorldPointCloud(sorted(sampled), (minimum, maximum), translation,
                           len(triangles), len(boxes))
