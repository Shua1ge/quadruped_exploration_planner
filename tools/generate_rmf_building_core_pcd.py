#!/usr/bin/env python3
"""Convert one Open-RMF building level into a lightweight wall-only PCD map."""

import argparse
import math
import pathlib

import yaml


def typed_value(value):
    """Return the payload from Traffic Editor's [type, value] encoding."""
    if isinstance(value, list) and len(value) == 2:
        return value[1]
    return value


def measurement_scale(level):
    measurements = level.get("measurements", [])
    vertices = level["vertices"]
    if not measurements:
        raise ValueError("The selected level has no scale measurement")
    start, end, parameters = measurements[0]
    distance = float(typed_value(parameters["distance"]))
    pixel_distance = math.hypot(
        vertices[start][0] - vertices[end][0],
        vertices[start][1] - vertices[end][1],
    )
    if pixel_distance <= 0.0:
        raise ValueError("The scale measurement has zero pixel length")
    return distance / pixel_distance


def sample_wall(points, start, end, spacing, height, half_width):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return
    nx = -dy / length
    ny = dx / length
    along_samples = max(1, math.ceil(length / spacing))
    z_samples = max(1, math.ceil(height / spacing))
    for side in (-half_width, half_width):
        for i in range(along_samples + 1):
            ratio = i / along_samples
            x = start[0] + ratio * dx + side * nx
            y = start[1] + ratio * dy + side * ny
            for j in range(z_samples + 1):
                z = height * j / z_samples
                points.add((round(x, 4), round(y, 4), round(z, 4)))


def convert(source, level_name, spacing, height, wall_width):
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    levels = document.get("levels", {})
    if level_name not in levels:
        raise ValueError(f"Unknown level {level_name!r}; available: {sorted(levels)}")
    level = levels[level_name]
    vertices = level["vertices"]
    scale = measurement_scale(level)
    min_x = min(vertex[0] for vertex in vertices)
    max_x = max(vertex[0] for vertex in vertices)
    min_y = min(vertex[1] for vertex in vertices)
    max_y = max(vertex[1] for vertex in vertices)
    center_x = 0.5 * (min_x + max_x)
    center_y = 0.5 * (min_y + max_y)

    def world(vertex):
        return ((vertex[0] - center_x) * scale, (center_y - vertex[1]) * scale)

    points = set()
    for start_index, end_index, _parameters in level.get("walls", []):
        sample_wall(
            points,
            world(vertices[start_index]),
            world(vertices[end_index]),
            spacing,
            height,
            0.5 * wall_width,
        )

    bounds = {
        "min_x": (min_x - center_x) * scale,
        "max_x": (max_x - center_x) * scale,
        "min_y": (center_y - max_y) * scale,
        "max_y": (center_y - min_y) * scale,
    }
    return sorted(points), scale, bounds, len(level.get("walls", []))


def write_pcd(output, points):
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="ascii", newline="\n") as stream:
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
    parser.add_argument("--building", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--level", default="L1")
    parser.add_argument("--spacing", type=float, default=0.15)
    parser.add_argument("--height", type=float, default=2.50)
    parser.add_argument("--wall-width", type=float, default=0.15)
    args = parser.parse_args()
    if args.spacing <= 0.0 or args.height <= 0.0 or args.wall_width <= 0.0:
        parser.error("spacing, height, and wall-width must be positive")

    points, scale, bounds, wall_count = convert(
        args.building, args.level, args.spacing, args.height, args.wall_width)
    if not points or wall_count == 0:
        raise RuntimeError("The selected level contains no wall geometry")
    write_pcd(args.output, points)
    print(
        f"Wrote {len(points)} points from {wall_count} walls; "
        f"scale={scale:.8f} m/pixel; bounds="
        f"[{bounds['min_x']:.2f}, {bounds['max_x']:.2f}] x "
        f"[{bounds['min_y']:.2f}, {bounds['max_y']:.2f}] m"
    )


if __name__ == "__main__":
    main()
