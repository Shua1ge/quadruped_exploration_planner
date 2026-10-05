"""Property test: the batched fusion is cell-for-cell equivalent to the
reference implementation, across randomized scenes and ray patterns.

Also locks the vectorised frontier_cells against a straightforward reference
loop, and the inflated-mask helper against the frozenset path.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from explorer_core.grid import (  # noqa: E402
    FREE, OCCUPIED, UNKNOWN, ExplorationGrid)


def _clone(grid: ExplorationGrid) -> ExplorationGrid:
    clone = ExplorationGrid(
        grid.width * grid.resolution, grid.height * grid.resolution,
        grid.resolution, grid.origin_x, grid.origin_y)
    clone.data = grid.data.copy()
    clone.evidence = grid.evidence.copy()
    clone.sensor_observed = grid.sensor_observed.copy()
    clone.sensor_revision = grid.sensor_revision
    return clone


def _random_scene(rng: np.random.Generator) -> ExplorationGrid:
    grid = ExplorationGrid(30.0, 24.0, 0.2)
    # scattered walls so first-occluder truncation actually triggers
    walls = rng.random(grid.data.shape) < 0.06
    grid.data[walls] = OCCUPIED
    # some already-free space
    free = rng.random(grid.data.shape) < 0.25
    grid.data[free] = FREE
    grid.evidence[free] = -rng.integers(1, 8, size=int(free.sum()))
    occupied = grid.data == OCCUPIED
    grid.evidence[occupied] = rng.integers(1, 8, size=int(occupied.sum()))
    return grid


def _random_ranges(rng: np.random.Generator, count: int = 720) -> list:
    ranges = []
    for _ in range(count):
        kind = rng.random()
        if kind < 0.12:
            ranges.append(float("nan"))       # height-filtered return
        elif kind < 0.30:
            ranges.append(float("inf"))       # genuine no-return
        elif kind < 0.90:
            ranges.append(float(rng.uniform(0.1, 4.9)))   # hit
        else:
            ranges.append(float(rng.uniform(5.0, 8.0)))   # over max_range
    return ranges


@pytest.mark.parametrize("seed", range(30))
def test_vectorized_fusion_matches_reference(seed):
    rng = np.random.default_rng(seed)
    grid = _random_scene(rng)
    position = (float(rng.uniform(-10.0, 20.0)), float(rng.uniform(-8.0, 16.0)))
    for _ in range(3):  # carry state across scans, like a run does
        ranges = _random_ranges(rng)
        reference = _clone(grid)
        batched = _clone(grid)
        reference._integrate_ranges_reference(position, ranges, 5.0)
        batched.integrate_ranges(position, ranges, 5.0)
        assert np.array_equal(reference.data, batched.data)
        assert np.array_equal(reference.evidence, batched.evidence)
        assert np.array_equal(reference.sensor_observed, batched.sensor_observed)
        assert reference.sensor_revision == batched.sensor_revision
        grid = batched


def test_vectorized_fusion_out_of_bounds_origin_is_noop():
    grid = ExplorationGrid(10.0, 10.0, 0.2)
    before = grid.data.copy()
    grid.integrate_ranges((-50.0, -50.0), [1.0] * 720, 5.0)
    assert np.array_equal(before, grid.data)


@pytest.mark.parametrize("ranges", [[], [float("nan")] * 8,
    [0.1] * 8, [float("-inf")] * 8])
def test_degenerate_scan_preserves_reference_seed_and_sensor_epoch(ranges):
    grid = ExplorationGrid(10.0, 10.0, 0.2)
    reference = _clone(grid)
    grid.integrate_ranges((0., 0.), ranges, 5.)
    reference._integrate_ranges_reference((0., 0.), ranges, 5.)
    assert np.array_equal(reference.data, grid.data)
    assert np.array_equal(reference.evidence, grid.evidence)
    assert np.array_equal(reference.sensor_observed, grid.sensor_observed)
    assert reference.sensor_revision == grid.sensor_revision


def test_vectorized_frontier_cells_match_reference():
    rng = np.random.default_rng(7)
    grid = _random_scene(rng)
    inflated = grid.inflated_obstacles(0.65)

    reference = set()
    for y, x in np.argwhere(grid.data == FREE):
        cell = (int(x), int(y))
        if cell in inflated:
            continue
        neighbours = ((cell[0] - 1, cell[1]), (cell[0] + 1, cell[1]),
                      (cell[0], cell[1] - 1), (cell[0], cell[1] + 1))
        if any(grid.in_bounds(n) and grid.value(n) == UNKNOWN for n in neighbours):
            reference.add(cell)

    assert grid.frontier_cells(inflated) == reference


def test_inflated_mask_matches_frozenset():
    rng = np.random.default_rng(11)
    grid = _random_scene(rng)
    mask = grid.inflated_mask(0.65)
    cells = grid.inflated_obstacles(0.65)
    ys, xs = np.nonzero(mask)
    assert {(int(x), int(y)) for x, y in zip(xs, ys)} == set(cells)
