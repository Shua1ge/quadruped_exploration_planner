#!/usr/bin/env python3
"""Convert COLLADA collision geometry into planar or surface PCD maps.

Planar mode supports the constant-height deterministic exploration simulator.
Surface mode preserves XYZ terrain geometry for the Gazebo physics benchmark.
"""

import argparse
import math
import pathlib
import xml.etree.ElementTree as ET


COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def _floats(text):
    return [float(value) for value in (text or "").split()]


def _matmul(matrix, point):
    x, y, z = point
    vector = (x, y, z, 1.0)
    return tuple(sum(matrix[row][column] * vector[column]
                     for column in range(4)) for row in range(3))


def _identity():
    return [[1.0 if row == column else 0.0 for column in range(4)]
            for row in range(4)]


def _node_matrix(node):
    element = node.find("c:matrix", COLLADA_NS)
    if element is None:
        return _identity()
    values = _floats(element.text)
    if len(values) != 16:
        raise ValueError("COLLADA node matrix must contain 16 values")
    return [values[row * 4:(row + 1) * 4] for row in range(4)]


def _position_sources(mesh):
    sources = {}
    for source in mesh.findall("c:source", COLLADA_NS):
        array = source.find("c:float_array", COLLADA_NS)
        accessor = source.find(".//c:accessor", COLLADA_NS)
        if array is None or accessor is None:
            continue
        stride = int(accessor.get("stride", "1"))
        parameters = [item.get("name", "")
                      for item in accessor.findall("c:param", COLLADA_NS)]
        if stride < 3 or parameters[:3] != ["X", "Y", "Z"]:
            continue
        values = _floats(array.text)
        sources["#" + source.get("id")] = [
            tuple(values[index:index + 3])
            for index in range(0, len(values) - stride + 1, stride)
        ]
    return sources


def _geometry_triangles(root):
    geometries = {}
    for geometry in root.findall(".//c:library_geometries/c:geometry", COLLADA_NS):
        mesh = geometry.find("c:mesh", COLLADA_NS)
        if mesh is None:
            continue
        sources = _position_sources(mesh)
        vertices = {}
        for vertex_set in mesh.findall("c:vertices", COLLADA_NS):
            position = next((item for item in vertex_set.findall("c:input", COLLADA_NS)
                             if item.get("semantic") == "POSITION"), None)
            if position is not None:
                vertices["#" + vertex_set.get("id")] = position.get("source")

        result = []
        for primitive in mesh.findall("c:triangles", COLLADA_NS):
            inputs = primitive.findall("c:input", COLLADA_NS)
            stride = max(int(item.get("offset", "0")) for item in inputs) + 1
            vertex_input = next(item for item in inputs
                                if item.get("semantic") == "VERTEX")
            vertex_offset = int(vertex_input.get("offset", "0"))
            source_key = vertices[vertex_input.get("source")]
            positions = sources[source_key]
            indices = [int(value) for value in primitive.findtext("c:p", "", COLLADA_NS).split()]
            vertex_indices = indices[vertex_offset::stride]
            for index in range(0, len(vertex_indices), 3):
                if index + 2 < len(vertex_indices):
                    result.append(tuple(positions[item]
                                        for item in vertex_indices[index:index + 3]))
        geometries["#" + geometry.get("id")] = result
    return geometries


def load_triangles(path, excluded_geometries):
    root = ET.parse(path).getroot()
    geometries = _geometry_triangles(root)
    triangles = []
    for node in root.findall(".//c:library_visual_scenes//c:node", COLLADA_NS):
        instance = node.find("c:instance_geometry", COLLADA_NS)
        if instance is None or instance.get("url") not in geometries:
            continue
        geometry_name = instance.get("url").lstrip("#")
        if geometry_name in excluded_geometries:
            continue
        matrix = _node_matrix(node)
        triangles.extend(tuple(_matmul(matrix, point) for point in triangle)
                         for triangle in geometries[instance.get("url")])
    if not triangles:
        raise RuntimeError("No instantiated COLLADA triangles were found")
    return triangles


def _normal_z(triangle):
    first, second, third = triangle
    ab = tuple(second[index] - first[index] for index in range(3))
    ac = tuple(third[index] - first[index] for index in range(3))
    cross = (ab[1] * ac[2] - ab[2] * ac[1],
             ab[2] * ac[0] - ab[0] * ac[2],
             ab[0] * ac[1] - ab[1] * ac[0])
    length = math.sqrt(sum(value * value for value in cross))
    return abs(cross[2]) / length if length > 1e-9 else 1.0


def _inside_triangle(px, py, triangle):
    (ax, ay, _), (bx, by, _), (cx, cy, _) = triangle
    denominator = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
    if abs(denominator) < 1e-12:
        return False
    u = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / denominator
    v = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / denominator
    return u >= -1e-9 and v >= -1e-9 and u + v <= 1.0 + 1e-9


def rasterise_obstacles(triangles, resolution, max_slope_degrees,
                        centre_x=None, centre_y=None, close_boundary=True):
    all_x = [point[0] for triangle in triangles for point in triangle]
    all_y = [point[1] for triangle in triangles for point in triangle]
    if centre_x is None:
        centre_x = 0.5 * (min(all_x) + max(all_x))
    if centre_y is None:
        centre_y = 0.5 * (min(all_y) + max(all_y))
    normal_z_limit = math.cos(math.radians(max_slope_degrees))
    occupied = set()
    steep_count = 0
    for triangle in triangles:
        if _normal_z(triangle) >= normal_z_limit:
            continue
        steep_count += 1
        shifted = tuple((point[0] - centre_x, point[1] - centre_y, point[2])
                        for point in triangle)
        min_x = math.floor(min(point[0] for point in shifted) / resolution)
        max_x = math.ceil(max(point[0] for point in shifted) / resolution)
        min_y = math.floor(min(point[1] for point in shifted) / resolution)
        max_y = math.ceil(max(point[1] for point in shifted) / resolution)
        for ix in range(min_x, max_x + 1):
            for iy in range(min_y, max_y + 1):
                x = ix * resolution
                y = iy * resolution
                if _inside_triangle(x, y, shifted):
                    occupied.add((ix, iy))
        for point in shifted:
            occupied.add((round(point[0] / resolution),
                          round(point[1] / resolution)))
    if close_boundary:
        min_ix = math.floor((min(all_x) - centre_x) / resolution)
        max_ix = math.ceil((max(all_x) - centre_x) / resolution)
        min_iy = math.floor((min(all_y) - centre_y) / resolution)
        max_iy = math.ceil((max(all_y) - centre_y) / resolution)
        for ix in range(min_ix, max_ix + 1):
            occupied.add((ix, min_iy))
            occupied.add((ix, max_iy))
        for iy in range(min_iy, max_iy + 1):
            occupied.add((min_ix, iy))
            occupied.add((max_ix, iy))
    return occupied, (centre_x, centre_y), steep_count


def planar_points(occupied, resolution, height, vertical_spacing):
    levels = max(2, int(math.ceil(height / vertical_spacing)))
    return [(ix * resolution, iy * resolution, height * level / levels)
            for ix, iy in sorted(occupied)
            for level in range(levels + 1)]


def sample_surface(triangles, spacing, centre_x=None, centre_y=None):
    all_x = [point[0] for triangle in triangles for point in triangle]
    all_y = [point[1] for triangle in triangles for point in triangle]
    if centre_x is None:
        centre_x = 0.5 * (min(all_x) + max(all_x))
    if centre_y is None:
        centre_y = 0.5 * (min(all_y) + max(all_y))
    precision = max(4, int(math.ceil(-math.log10(spacing))) + 2)
    points = set()
    for first, second, third in triangles:
        edges = (
            math.dist(first, second),
            math.dist(first, third),
            math.dist(second, third),
        )
        subdivisions = max(1, int(math.ceil(max(edges) / spacing)))
        for row in range(subdivisions + 1):
            for column in range(subdivisions - row + 1):
                u = row / subdivisions
                v = column / subdivisions
                w = 1.0 - u - v
                point = tuple(w * first[index] + u * second[index] + v * third[index]
                              for index in range(3))
                points.add((round(point[0] - centre_x, precision),
                            round(point[1] - centre_y, precision),
                            round(point[2], precision)))
    return sorted(points), (centre_x, centre_y)


def write_pcd(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n")
        stream.write("TYPE F F F\nCOUNT 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\nHEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\nDATA ascii\n")
        for x, y, z in points:
            stream.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
    return len(points)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("planar", "surface"), default="planar")
    parser.add_argument("--dae", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--resolution", type=float, default=0.15)
    parser.add_argument("--height", type=float, default=2.5)
    parser.add_argument("--vertical-spacing", type=float, default=0.15)
    parser.add_argument("--max-slope-degrees", type=float, default=15.0)
    parser.add_argument("--centre-x", type=float)
    parser.add_argument("--centre-y", type=float)
    parser.add_argument(
        "--open-boundary", action="store_true",
        help="Do not add a rectangular guard boundary around the mesh extent")
    parser.add_argument(
        "--exclude-geometry", action="append", default=[],
        help="COLLADA geometry id to omit. May be repeated.")
    args = parser.parse_args()
    if min(args.resolution, args.height, args.vertical_spacing) <= 0.0:
        parser.error("resolution, height and vertical-spacing must be positive")
    if not 0.0 < args.max_slope_degrees < 90.0:
        parser.error("max-slope-degrees must be between 0 and 90")
    triangles = load_triangles(args.dae, set(args.exclude_geometry))
    if args.mode == "surface":
        points, centre = sample_surface(
            triangles, args.resolution, args.centre_x, args.centre_y)
        occupied = None
        steep_count = None
    else:
        occupied, centre, steep_count = rasterise_obstacles(
            triangles, args.resolution, args.max_slope_degrees,
            args.centre_x, args.centre_y, not args.open_boundary)
        points = planar_points(
            occupied, args.resolution, args.height, args.vertical_spacing)
    point_count = write_pcd(args.output, points)
    if args.mode == "surface":
        print(f"Loaded {len(triangles)} triangles; sampled the complete XYZ surface")
    else:
        print(f"Loaded {len(triangles)} triangles; projected {steep_count} steep faces")
    print(f"Centred source by translation ({-centre[0]:.4f}, {-centre[1]:.4f}, 0.0)")
    detail = (f" in {len(occupied)} occupied XY cells" if occupied is not None else "")
    print(f"Wrote {point_count} points{detail} to {args.output}")


if __name__ == "__main__":
    main()
