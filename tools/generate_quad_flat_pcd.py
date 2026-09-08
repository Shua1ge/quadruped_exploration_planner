#!/usr/bin/env python3
"""Convert the gazebo_maps quad maze collision layout into an ASCII PCD map."""

import argparse
import math
import pathlib
import xml.etree.ElementTree as ET


WALL_LENGTHS = {
    "unit_wall": 4.0,
    "half_wall": 2.0,
    "long_wall": 32.0,
}

# quad_map/model.sdf places quad_maze at (-12, 0) with +90 degree yaw.
MAZE_TX = -12.0
MAZE_TY = 0.0
MAZE_YAW = math.pi / 2.0

# Tree centres from quad_map/model.sdf.  A conservative cylinder is sufficient
# for the geometry-only planner benchmark; Gazebo visuals are not collision truth.
TREES = (
    (21.10, 9.36, 0.8),
    (-25.92, -13.81, 0.8),
    (-3.14, -10.62, 0.8),
    (22.49, 5.58, 0.8),
    (11.97, 4.94, 1.0),
    (26.59, -6.45, 1.0),
    (-25.32, -3.00, 1.0),
)


def transform(x, y):
    c = math.cos(MAZE_YAW)
    s = math.sin(MAZE_YAW)
    return MAZE_TX + c * x - s * y, MAZE_TY + s * x + c * y


def sample_wall(points, x, y, yaw, length, spacing, height):
    half_width = 0.075
    samples = max(2, int(math.ceil(length / spacing)))
    z_samples = max(2, int(math.ceil(height / spacing)))
    c = math.cos(yaw)
    s = math.sin(yaw)

    def add(local_x, local_y, z):
        wx = x + c * local_x - s * local_y
        wy = y + s * local_x + c * local_y
        points.add((round(wx, 4), round(wy, 4), round(z, 4)))

    for i in range(samples + 1):
        along = -length / 2.0 + length * i / samples
        for j in range(z_samples + 1):
            z = height * j / z_samples
            add(along, -half_width, z)
            add(along, half_width, z)

    for side in (-length / 2.0, length / 2.0):
        for j in range(z_samples + 1):
            z = height * j / z_samples
            add(side, 0.0, z)


def sample_tree(points, x, y, radius, spacing, height):
    circumference_samples = max(16, int(math.ceil(2.0 * math.pi * radius / spacing)))
    z_samples = max(2, int(math.ceil(height / spacing)))
    for i in range(circumference_samples):
        angle = 2.0 * math.pi * i / circumference_samples
        px = x + radius * math.cos(angle)
        py = y + radius * math.sin(angle)
        for j in range(z_samples + 1):
            pz = height * j / z_samples
            points.add((round(px, 4), round(py, 4), round(pz, 4)))


def load_points(sdf_path, spacing, height):
    root = ET.parse(sdf_path).getroot()
    points = set()
    wall_count = 0
    for include in root.findall(".//include"):
        uri = (include.findtext("uri") or "").strip()
        model_name = uri.rsplit("/", 1)[-1]
        if model_name not in WALL_LENGTHS:
            continue
        pose = [float(value) for value in (include.findtext("pose") or "0 0 0 0 0 0").split()]
        # A few poses in the original 2021 asset omit yaw.  Gazebo Classic
        # treated the missing value as zero; preserve that behaviour explicitly.
        pose.extend([0.0] * (6 - len(pose)))
        local_x, local_y = pose[0], pose[1]
        world_x, world_y = transform(local_x, local_y)
        world_yaw = pose[5] + MAZE_YAW
        sample_wall(points, world_x, world_y, world_yaw, WALL_LENGTHS[model_name], spacing, height)
        wall_count += 1

    for x, y, radius in TREES:
        sample_tree(points, x, y, radius, spacing, height)

    return sorted(points), wall_count


def write_pcd(path, points):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\n")
        stream.write("FIELDS x y z\n")
        stream.write("SIZE 4 4 4\n")
        stream.write("TYPE F F F\n")
        stream.write("COUNT 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\n")
        stream.write("HEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\n")
        stream.write("DATA ascii\n")
        for x, y, z in points:
            stream.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", required=True, type=pathlib.Path,
                        help="quad_maze/model.sdf from engcang/gazebo_maps")
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--spacing", type=float, default=0.10)
    parser.add_argument("--height", type=float, default=2.50)
    args = parser.parse_args()
    if args.spacing <= 0.0 or args.height <= 0.0:
        parser.error("spacing and height must be positive")

    points, wall_count = load_points(args.sdf, args.spacing, args.height)
    if not points or wall_count < 1:
        raise RuntimeError("No supported wall includes found in the supplied SDF")
    write_pcd(args.output, points)
    print(f"Wrote {len(points)} points from {wall_count} walls and {len(TREES)} trees to {args.output}")


if __name__ == "__main__":
    main()
