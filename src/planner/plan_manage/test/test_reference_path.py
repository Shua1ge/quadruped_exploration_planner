import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from explorer_core.grid import FREE, OCCUPIED, ExplorationGrid  # noqa: E402
from explorer_core.path_planning import densified_reference_path  # noqa: E402


def corridor_grid():
    grid = ExplorationGrid(4.0, 4.0, 0.5)
    grid.data[:, :] = OCCUPIED
    grid.data[3, :] = FREE
    return grid


def test_sparse_route_is_densified_into_adjacent_free_cells():
    grid = corridor_grid()
    dense = densified_reference_path(
        grid, [(0, 3), (7, 3)], set(), None, max_segment_cells=1)
    assert dense is not None
    assert dense[0] == (0, 3)
    assert dense[-1] == (7, 3)
    assert len(dense) == 8
    assert all(abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
               for a, b in zip(dense, dense[1:]))


def test_sparse_route_with_a_blocked_chord_is_rejected():
    grid = corridor_grid()
    grid.data[3, 4] = OCCUPIED
    assert densified_reference_path(
        grid, [(0, 3), (7, 3)], set(), None, max_segment_cells=1) is None


def test_short_segment_keeps_endpoints():
    grid = corridor_grid()
    dense = densified_reference_path(
        grid, [(3, 3), (4, 3)], set(), None, max_segment_cells=4)
    assert dense == [(3, 3), (4, 3)]


def test_densification_preserves_checked_raster_on_repeated_validation():
    from explorer_core.grid import bresenham
    grid = ExplorationGrid(10., 10., 1., 0., 0.)
    grid.data.fill(FREE)
    grid.data[2, 2] = OCCUPIED
    original = [(1, 1), (2, 6)]
    first = densified_reference_path(grid, original, set(), max_segment_cells=4)
    assert first is not None
    second = densified_reference_path(grid, first, set(), max_segment_cells=4)
    assert second == first
    raster = []
    for a, b in zip(first, first[1:]):
        segment = bresenham(a, b)
        raster.extend(segment if not raster else segment[1:])
    assert raster == bresenham(*original)


def test_raster_preservation_and_idempotence_in_all_directions():
    from explorer_core.grid import bresenham
    grid = ExplorationGrid(30., 30., 1., 0., 0.)
    grid.data.fill(FREE)
    start = (15, 15)
    for dx in range(-12, 13):
        for dy in range(-12, 13):
            goal = (start[0]+dx, start[1]+dy)
            for step in (1, 3, 4):
                dense = densified_reference_path(grid, [start, goal], set(), max_segment_cells=step)
                assert dense is not None
                assert densified_reference_path(grid, dense, set(), max_segment_cells=step) == dense
                raster = [start]
                for a, b in zip(dense, dense[1:]):
                    raster.extend(bresenham(a, b)[1:])
                assert raster == bresenham(start, goal)


def test_new_occupied_cell_still_invalidates_a_densified_route():
    grid = ExplorationGrid(10., 10., 1., 0., 0.)
    grid.data.fill(FREE)
    dense = densified_reference_path(grid, [(1, 1), (2, 6)], set(), max_segment_cells=4)
    grid.data[3, 1] = OCCUPIED
    assert densified_reference_path(grid, dense, set(), max_segment_cells=4) is None
