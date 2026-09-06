import importlib.util
import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "global_astar_planner", ROOT / "scripts" / "global_astar_planner.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def load_pcd():
    points = []
    in_data = False
    with (ROOT / "maps" / "quad_flat.pcd").open(encoding="ascii") as stream:
        for line in stream:
            if in_data:
                points.append(tuple(float(value) for value in line.split()))
            elif line.startswith("DATA"):
                in_data = True
    return points


def make_grid():
    return MODULE.OccupancyGrid2D(load_pcd(), 0.2, 0.65, 0.08, 0.85, 1.0)


def test_fixed_long_routes_are_connected_and_safe():
    grid = make_grid()
    start = grid.world_to_cell(-24.0, 10.0)
    for goal_xy in ((24.0, 10.0), (24.0, -10.0), (-4.0, -12.0)):
        goal = grid.world_to_cell(*goal_xy)
        path = MODULE.astar(grid, start, goal)
        assert path
        simplified = MODULE.simplify_path(grid, path)
        assert simplified[0] == start
        assert simplified[-1] == goal
        assert all(grid.segment_is_free(a, b) for a, b in zip(simplified[:-1], simplified[1:]))


def test_occupied_goal_is_rejected():
    grid = make_grid()
    start = grid.world_to_cell(-24.0, 10.0)
    occupied_goal = next(iter(grid.occupied))
    assert MODULE.astar(grid, start, occupied_goal) is None


def test_diagonal_corner_cutting_is_not_used():
    points = [
        (0.0, 1.0, 0.5),
        (1.0, 0.0, 0.5),
        (0.0, 0.0, 0.0),
        (2.0, 2.0, 0.0),
    ]
    grid = MODULE.OccupancyGrid2D(points, 1.0, 0.0, 0.1, 0.9, 1.0)
    path = MODULE.astar(grid, grid.world_to_cell(0.0, 0.0), grid.world_to_cell(1.0, 1.0))
    assert path is not None
    assert len(path) > 2
