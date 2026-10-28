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
    FREE, OCCUPIED, UNKNOWN, ExplorationGrid, classify_planar_scan, rpy_rotation_matrix)


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


def _identified_projection(values, rotation=None, origin_z=.12, dilation=0):
    return classify_planar_scan(values, -math.pi, 2*math.pi/len(values), .2, 7.5,
        np.array([0.,0.,origin_z]), np.eye(3) if rotation is None else rotation,
        .08,.85,7.5,3.,720,dilation,.1)


def test_identified_no_return_extends_known_space_without_fictitious_endpoint():
    ranges, prefix=_identified_projection(np.full(720,np.inf))
    grid=ExplorationGrid(12.,12.,.1)
    grid.integrate_ranges((0.,0.),ranges,7.5,clearing_ranges=prefix)
    assert grid.value(grid.world_to_cell(2.,0.))==FREE
    assert grid.value(grid.world_to_cell(4.,0.))==UNKNOWN
    assert not np.any(grid.data==OCCUPIED)


@pytest.mark.parametrize('invalid',[np.nan,-np.inf,.1,8.])
def test_invalid_identified_beams_do_not_clear_missing_space(invalid):
    ranges,prefix=_identified_projection(np.full(720,invalid))
    assert np.isnan(ranges).all() and not prefix.any()
    grid=ExplorationGrid(12.,12.,.1)
    grid.integrate_ranges((0.,0.),ranges,7.5,clearing_ranges=prefix)
    assert grid.value(grid.world_to_cell(1.,0.))==UNKNOWN


def test_ground_hit_only_clears_its_height_valid_prefix():
    values=np.full(720,np.nan);values[360]=5.
    ranges,prefix=_identified_projection(values,rpy_rotation_matrix(0.,.15,0.),.4)
    grid=ExplorationGrid(12.,12.,.1)
    grid.integrate_ranges((0.,0.),ranges,7.5,clearing_ranges=prefix)
    assert grid.value(grid.world_to_cell(1.,0.))==FREE
    assert grid.value(grid.world_to_cell(2.5,0.))==UNKNOWN
    assert grid.value(grid.world_to_cell(5.,0.))==UNKNOWN
    assert not np.any(grid.data==OCCUPIED)


def test_ray_starting_outside_height_band_cannot_clear_unobserved_prefix():
    values=np.full(720,np.nan);values[360]=3.
    ranges,prefix=_identified_projection(values,rpy_rotation_matrix(0.,-.2,0.),-.1)
    assert not prefix.any()
    grid=ExplorationGrid(12.,12.,.1)
    grid.integrate_ranges((0.,0.),ranges,7.5,clearing_ranges=prefix)
    assert grid.value(grid.world_to_cell(1.,0.))==UNKNOWN
    assert np.any(grid.data==OCCUPIED)  # actual endpoint enters the obstacle band


def test_measured_obstacle_occludes_a_neighbouring_no_return():
    values=np.full(720,np.inf);values[360]=1.5
    ranges,prefix=_identified_projection(values,dilation=3)
    grid=ExplorationGrid(12.,12.,.1)
    grid.integrate_ranges((0.,0.),ranges,7.5,clearing_ranges=prefix)
    assert np.any(grid.data==OCCUPIED)
    assert grid.value(grid.world_to_cell(2.5,0.))==UNKNOWN


@pytest.mark.parametrize('seed',range(20))
def test_bounded_ray_prefixes_match_scalar_fusion(seed):
    rng=np.random.default_rng(seed);grid=_random_scene(rng)
    position=(float(rng.uniform(-10.,10.)),float(rng.uniform(-8.,8.)))
    ranges=_random_ranges(rng);prefix=rng.uniform(0.,5.,len(ranges))
    prefix[::9]=0.;prefix[::13]=np.nan
    reference=_clone(grid);batched=_clone(grid)
    reference._integrate_ranges_reference(position,ranges,5.,clearing_ranges=prefix)
    batched.integrate_ranges(position,ranges,5.,clearing_ranges=prefix)
    assert np.array_equal(reference.data,batched.data)
    assert np.array_equal(reference.evidence,batched.evidence)
    assert np.array_equal(reference.sensor_observed,batched.sensor_observed)


def _spatial_projection(points,kinds,rotation=None,origin_z=.4):
    from explorer_core.grid import classify_spatial_rays
    return classify_spatial_rays(np.asarray(points),np.asarray(kinds),
        np.array([0.,0.,origin_z]),np.eye(3) if rotation is None else rotation,
        .08,.85,7.5,3.,720,0,.1)


def test_spatial_ground_ray_does_not_erase_independent_horizontal_evidence():
    # One downward ray exits the height band; another actual ray certifies a
    # horizontal prefix in the same azimuth bin. Neither creates a fake hit.
    nearest,clear=_spatial_projection([[2.,0.,-1.],[3.,0.,0.]],[1,2])
    assert not np.isfinite(nearest).any()
    assert clear[360]==pytest.approx(3.)
    grid=ExplorationGrid(12.,12.,.1)
    grid.integrate_ranges((0.,0.),nearest,7.5,clearing_ranges=clear)
    assert grid.value(grid.world_to_cell(2.5,0.))==FREE
    assert grid.value(grid.world_to_cell(3.5,0.))==UNKNOWN


def test_spatial_obstacle_hit_caps_all_elevations_and_misses():
    nearest,clear=_spatial_projection([[1.5,0.,0.],[3.,0.,0.],[4.,0.,3.]],[1,2,1])
    assert nearest[360]==pytest.approx(1.5)
    assert clear[360]<=1.5


def test_spatial_miss_cannot_clear_beyond_its_supplied_endpoint():
    nearest,clear=_spatial_projection([[1.2,0.,0.]],[2])
    assert not np.isfinite(nearest).any()
    assert clear[360]==pytest.approx(1.2)


def test_spatial_invalid_rays_never_clear_or_create_obstacles():
    nearest,clear=_spatial_projection([[3.,0.,0.],[np.nan,0.,0.]],[0,1])
    assert np.isnan(nearest).all() and not clear.any()


def test_unitree_pitch_transforms_a_lower_return_to_ground_without_fake_hit():
    # Sensor-local -7deg plus the official +13deg mounting pitch gives a
    # downward -20deg ray in front of a level body.
    angle=math.radians(-7.)
    point=[2.*math.cos(angle),0.,2.*math.sin(angle)]
    rotation=rpy_rotation_matrix(0.,math.radians(13.),0.)
    world=rotation @ point
    assert math.degrees(math.atan2(world[2],world[0]))==pytest.approx(-20.)
    nearest,clear=_spatial_projection([point],[1],rotation,origin_z=.5)
    assert not np.isfinite(nearest).any()
    assert 0.<clear[360]<2.*math.cos(math.radians(20.))


def test_ground_support_revokes_old_floor_hit_but_not_a_same_cell_obstacle():
    grid=ExplorationGrid(12.,12.,.2)
    surface=grid.world_to_cell(2.,0.)
    grid.data[surface[1],surface[0]]=OCCUPIED
    grid.evidence[surface[1],surface[0]]=1
    for _ in range(3):
        grid.integrate_ranges((0.,0.),[np.nan]*720,7.5,ground_cells={surface})
    assert grid.value(surface)==FREE
    # A positive obstacle observed in the same XY cell always wins this scan.
    grid.integrate_ranges((0.,0.),[np.nan]*720,7.5,
        ground_cells={surface},obstacle_cells={surface})
    assert grid.sensor_observed[surface[1],surface[0]]==grid.sensor_revision
    assert grid.evidence[surface[1],surface[0]]==-1
    for _ in range(2):
        grid.integrate_ranges((0.,0.),[np.nan]*720,7.5,
            ground_cells={surface},obstacle_cells={surface})
    assert grid.value(surface)==OCCUPIED


def test_typed_heights_use_local_clearance_in_both_projection_channels():
    # Known heights use Local's ground-relative clearance interval, even when
    # their physical Z is outside the current body band. Both fusion channels
    # must retain the same real step/drop and exclude the same low terrain.
    from explorer_core.grid import classify_spatial_rays

    def h(m):
        return 3 + int(round((max(-0.5, min(1.99, m)) + 0.5) / 0.01))

    floor = np.array([[2., 0., -.30]])
    nearest, clear = classify_spatial_rays(floor, np.array([h(0.0)]),
        np.array([0., 0., .4]), np.eye(3), .15, .65, 7.5, 3., 720, 0, .2)
    assert not np.isfinite(nearest).any()
    assert clear[360] > 1.9   # support still clears to its own measured endpoint
    # A measured 30 cm rise blocks at its own XY regardless of current body Z.
    below = np.array([[2., 0., -.70]])
    nearest, clear = classify_spatial_rays(below, np.array([h(0.30)]),
        np.array([0., 0., .4]), np.eye(3), .15, .65, 7.5, 3., 720, 0, .2)
    assert np.isfinite(nearest).sum() == 1
    inside = np.array([[2., 0., -.10], [2., 1., -.05]])
    nearest, clear = classify_spatial_rays(inside, np.array([h(0.30), h(0.0)]),
        np.array([0., 0., .4]), np.eye(3), .15, .65, 7.5, 3., 720, 0, .2)
    assert np.isfinite(nearest).sum() == 1   # only the in-band rise
    # A miss alone never becomes an obstacle hit.
    miss = np.array([[3., 0., -.4]])
    nearest, clear = classify_spatial_rays(miss, np.array([2]),
        np.array([0., 0., .4]), np.eye(3), .15, .65, 7.5, 3., 720, 0, .2)
    assert not np.isfinite(nearest).any()


@pytest.mark.parametrize('seed',range(5))
def test_typed_surface_endpoint_fusion_matches_reference(seed):
    rng=np.random.default_rng(seed);grid=_random_scene(rng)
    reference=_clone(grid);batched=_clone(grid)
    ground={(int(x),int(y)) for x,y in rng.integers(5,70,size=(20,2))}
    occupied={(int(x),int(y)) for x,y in rng.integers(5,70,size=(8,2))}
    ranges=_random_ranges(rng);clearing=rng.uniform(0,5,len(ranges))
    reference._integrate_ranges_reference((0.,0.),ranges,5.,clearing_ranges=clearing,
        ground_cells=ground,obstacle_cells=occupied)
    batched.integrate_ranges((0.,0.),ranges,5.,clearing_ranges=clearing,
        ground_cells=ground,obstacle_cells=occupied)
    assert np.array_equal(reference.data,batched.data)
    assert np.array_equal(reference.evidence,batched.evidence)
