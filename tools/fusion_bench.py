#!/usr/bin/env python3
"""Offline fusion benchmark: reference vs batched fusion and old vs new
map serialization, at NewMine scale (280x280 m @ 0.2 m).

Run from anywhere:  python3 tools/fusion_bench.py [--scans 20]
No ROS required; explorer_core is imported straight from the source tree.
"""

import argparse
import array
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "planner" / "plan_manage" / "scripts"))

from explorer_core.grid import ExplorationGrid, OCCUPIED  # noqa: E402


def build_scene(seed: int = 1) -> ExplorationGrid:
    rng = np.random.default_rng(seed)
    grid = ExplorationGrid(280.0, 280.0, 0.2)
    # a tunnel-like structure: walls along two corridors plus scattered rubble
    w, h = grid.width, grid.height
    yy, xx = np.mgrid[0:h, 0:w]
    corridor = ((np.abs(yy - h // 2) < 7) | (np.abs(xx - w // 2) < 7))
    walls = (~corridor) & (rng.random(grid.data.shape) < 0.25)
    grid.data[walls] = OCCUPIED
    grid.evidence[walls] = 5
    return grid


def synthetic_ranges(rng, hit_fraction: float = 0.7, count: int = 720,
                     max_range: float = 5.0) -> list:
    ranges = []
    for _ in range(count):
        kind = rng.random()
        if kind < 0.10:
            ranges.append(float("nan"))
        elif kind > hit_fraction:
            ranges.append(float("inf"))
        else:
            ranges.append(float(rng.uniform(0.4, max_range - 0.1)))
    return ranges


def old_serialization_bytes(data: np.ndarray, planning: bool,
                            inflated) -> array.array:
    payload = data.copy()
    if planning:
        for x, y in inflated:
            payload[y, x] = OCCUPIED
    return array.array('b', payload.reshape(-1).astype(int).tolist())


def new_serialization_bytes(data: np.ndarray, planning: bool,
                            inflated_mask: np.ndarray) -> array.array:
    payload = data.copy()
    if planning:
        payload[inflated_mask] = OCCUPIED
    return array.array('b', payload.reshape(-1).tobytes())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scans", type=int, default=20)
    args = parser.parse_args()

    grid = build_scene()
    rng = np.random.default_rng(3)
    position = (140.0, 140.0)
    print(f"grid {grid.width}x{grid.height} cells "
          f"({grid.width * grid.height / 1e6:.2f} M), {args.scans} scans")

    t_ref = t_vec = t_pub_old = t_pub_new = 0.0
    ref_grid = grid
    vec_grid = grid
    for i in range(args.scans):
        ranges = synthetic_ranges(rng)
        # alternate robot position so the sliding footprint varies
        position = (140.0 + 2.0 * math.cos(i), 140.0 + 2.0 * math.sin(i))

        started = time.perf_counter()
        ref_grid._integrate_ranges_reference(position, ranges, 5.0)
        t_ref += time.perf_counter() - started

        started = time.perf_counter()
        vec_grid.integrate_ranges(position, ranges, 5.0)
        t_vec += time.perf_counter() - started

        inflated = vec_grid.inflated_obstacles(0.65)
        inflated_mask = vec_grid.inflated_mask(0.65)
        started = time.perf_counter()
        old_serialization_bytes(vec_grid.data, True, inflated)
        t_pub_old += time.perf_counter() - started
        started = time.perf_counter()
        new_serialization_bytes(vec_grid.data, True, inflated_mask)
        t_pub_new += time.perf_counter() - started

    # sanity: after identical scan streams both grids must agree
    assert np.array_equal(ref_grid.data, vec_grid.data), "fusion divergence!"
    assert np.array_equal(ref_grid.evidence, vec_grid.evidence), "evidence divergence!"

    n = args.scans
    print(f"fusion reference : {t_ref / n * 1000:8.2f} ms/scan")
    print(f"fusion batched   : {t_vec / n * 1000:8.2f} ms/scan"
          f"  ({t_ref / max(t_vec, 1e-9):.1f}x)")
    print(f"publish tolist   : {t_pub_old / n * 1000:8.2f} ms/scan")
    print(f"publish tobytes  : {t_pub_new / n * 1000:8.2f} ms/scan"
          f"  ({t_pub_old / max(t_pub_new, 1e-9):.1f}x)")
    print(f"total per 0.5 s tick, old vs new: "
          f"{(t_ref + t_pub_old) / n * 1000:.1f} ms -> "
          f"{(t_vec + t_pub_new) / n * 1000:.1f} ms")


if __name__ == "__main__":
    main()
