"""Exercise the actual NewMine launch band with a pitched MID-360 body.

No ROS nodes or actuators are started. Projection must keep observed free
prefixes while excluding low returns and retaining above-boundary obstacles.
"""
import importlib.util
import math
import os
import tempfile
import unittest
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np
import yaml
from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters, normalize_parameters

MANAGE = Path(__file__).resolve().parents[1]
ROOT = MANAGE.parents[2]
DESCRIPTION = ROOT / "src/simulator/Utils/go2_description"
sys.path.insert(0, str(MANAGE / "scripts"))
from explorer_core.grid import (FREE, OCCUPIED, ExplorationGrid,
    classify_spatial_rays, rpy_rotation_matrix)


def launch_parameters(gazebo):
    spec = importlib.util.spec_from_file_location(
        "newmine_mid360_band_test", MANAGE / "launch/newmine_exploration.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []
    real_node = module.Node

    def capture(**kwargs):
        if kwargs.get("executable") == "frontier_explorer.py":
            captured.extend(kwargs["parameters"])
        return real_node(**kwargs)

    shares = {"scan_planner": str(MANAGE), "go2_description": str(DESCRIPTION)}
    with patch.object(module, "get_package_share_directory", side_effect=shares.__getitem__), \
            patch.object(module, "Node", side_effect=capture):
        description = module.generate_launch_description()
    context = LaunchContext()
    # Resolve declared defaults without executing preflight or any live nodes.
    for entity in description.entities:
        if hasattr(entity, "default_value") and entity.default_value is not None:
            context.launch_configurations[entity.name] = "".join(
                item.perform(context) for item in entity.default_value)
    context.launch_configurations["use_gazebo_physics"] = str(gazebo).lower()
    return evaluate_parameters(context, normalize_parameters(captured))[0]


def check_gazebo_band_with_body_tilt(pitch):
    parameters = launch_parameters(True)
    planner = yaml.safe_load((MANAGE / "config/planner.yaml").read_text())[
        "scan_planner_node"]["ros__parameters"]
    assert parameters["obstacle_min_z"] == -planner["grid_map.low_return_z_below_body"]
    assert parameters["obstacle_max_z"] == planner["grid_map.high_return_z_above_body"]
    mount = yaml.safe_load((DESCRIPTION / "config/mid360.yaml").read_text())["mount"]
    body = np.array([0., 0., .288])
    body_rotation = rpy_rotation_matrix(0., pitch, 0.)
    origin = body + body_rotation @ np.array([mount[k] for k in ("x", "y", "z")])
    rotation = body_rotation @ rpy_rotation_matrix(
        mount["roll"], mount["pitch"], mount["yaw"])
    # True world-frame endpoints: ordinary floor, lower trench floor, a low
    # obstacle retained by Local, a horizontal measured no-return, and both
    # sides of the upper band boundary. Tunnel beams and arches arrive from
    # above it and must stay clearing evidence.
    beam_above = body[2] + parameters["obstacle_max_z"] + .25
    beam_inside = body[2] + parameters["obstacle_max_z"] - .03
    world = np.array([[2., -1., 0.], [2., 1., -.3], [2., 0., .08],
                      [origin[0], 3., origin[2]],
                      [2., 1.2, beam_above], [2., 1.4, beam_inside]])
    local = (world - origin) @ rotation
    nearest, clearing = classify_spatial_rays(
        local, np.array([1, 1, 1, 2, 1, 1]), origin, rotation,
        body[2] + parameters["obstacle_min_z"],
        body[2] + parameters["obstacle_max_z"], 7.5, 3., 720, 0, .2)
    # The .08 m obstacle and the in-band beam; never the overhead beam.
    assert np.count_nonzero(np.isfinite(nearest)) == 2
    assert clearing.max() >= 2.9
    grid = ExplorationGrid(12., 12., .2)
    grid.integrate_ranges(origin[:2], nearest, 7.5, clearing_ranges=clearing)
    assert grid.value(grid.world_to_cell(origin[0], 2.5)) == FREE
    # The bearing-bin reconstruction quantises the endpoint, so compare in
    # metres: the in-band beam stays the planner's obstacle evidence, the
    # overhead beam is clearing evidence only.
    occupied_world = [grid.cell_to_world((int(x_index), int(y_index)))
                      for y_index, x_index in np.argwhere(grid.data == OCCUPIED)]
    near_occupied = lambda point: any(
        math.hypot(world_xy[0] - point[0], world_xy[1] - point[1]) < .3
        for world_xy in occupied_world)
    assert near_occupied((2., 1.4))
    assert not near_occupied((2., 1.2))
    assert np.count_nonzero(grid.data == OCCUPIED) == 2
    if pitch > 0.:
        assert origin[2] < body[2] + .08  # old launch rejected every prefix


class TestMid360ExplorerProjection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.log_directory = tempfile.TemporaryDirectory(prefix="mid360_band_test_")
        cls.previous_log_directory = os.environ.get("ROS_LOG_DIR")
        os.environ["ROS_LOG_DIR"] = cls.log_directory.name

    @classmethod
    def tearDownClass(cls):
        if cls.previous_log_directory is None:
            os.environ.pop("ROS_LOG_DIR", None)
        else:
            os.environ["ROS_LOG_DIR"] = cls.previous_log_directory
        cls.log_directory.cleanup()

    def test_gazebo_band_keeps_clearing_with_body_tilt(self):
        for pitch in (0., math.radians(1.), math.radians(-1.)):
            with self.subTest(pitch=pitch):
                check_gazebo_band_with_body_tilt(pitch)

    def test_clearing_window_is_wider_than_the_occupancy_band(self):
        """A ray above the body band clears its prefix and never occupies.

        The occupancy band answers "can this block the body"; the clearing
        window answers "how far may a ray travel and still prove the cells
        under it are free". Collapsing them into one narrow band truncates
        every clearing prefix on the pitched MID-360 and freezes the map.
        """
        mount = yaml.safe_load((DESCRIPTION / "config/mid360.yaml").read_text())["mount"]
        body = np.array([0., 0., .288])
        rotation = rpy_rotation_matrix(0., 0., 0.) @ rpy_rotation_matrix(
            mount["roll"], mount["pitch"], mount["yaw"])
        origin = body + np.array([mount[k] for k in ("x", "y", "z")])
        # One hit at body height (inside the occupancy band -> obstacle) and one
        # five degrees above it, four metres out (outside the band -> clearing
        # only). Different azimuths keep them in separate bearing bins.
        in_band = np.array([[6., 0., origin[2]]])
        above = np.array([[4. * math.cos(.4), 4. * math.sin(.4),
                           origin[2] + 4. * math.sin(math.radians(5.))]])
        world = np.vstack([in_band, above])
        local = (world - origin) @ rotation
        nearest, clearing = classify_spatial_rays(
            local, np.array([1, 1]), origin, rotation,
            body[2] - .25, body[2] + .25, 7.5, 3., 720, 0, .2,
            clear_min_z=body[2] - .25, clear_max_z=body[2] + .85)
        # Only the body-height hit is an obstacle.
        assert np.count_nonzero(np.isfinite(nearest)) == 1
        # Address the above-band ray's own bearing bin: the in-band ray clears
        # to its 6 m hit under either window, so a global maximum would hide
        # the effect being tested.
        direction = (local[1] / np.linalg.norm(local[1])) @ rotation.T
        above_bin = int(np.floor(
            (math.atan2(direction[1], direction[0]) + math.pi) * 720 / (2 * math.pi)))
        floor_bin = above_bin - (720 // 32)   # guard: neighbouring bin only
        assert clearing[above_bin] >= 3.5, (clearing[above_bin], above_bin)
        # Reusing the occupancy band as the clearing window truncates it: the
        # ray leaves the band after (0.25 - 0.08)/sin(5 deg) ~= 1.95 m.
        _, narrow = classify_spatial_rays(
            local, np.array([1, 1]), origin, rotation,
            body[2] - .25, body[2] + .25, 7.5, 3., 720, 0, .2)
        assert narrow[above_bin] < 2.5, (narrow[above_bin], above_bin)
        del floor_bin

    def test_unknown_endpoint_slice_uses_local_upward_inflation(self):
        parameters = launch_parameters(True)
        local = yaml.safe_load((MANAGE / "config/planner.yaml").read_text())[
            "scan_planner_node"]["ros__parameters"]
        expected = max(parameters["obstacle_min_z"],
            -local["grid_map.obstacles_inflation_z_up"]
            - .5 * local["grid_map.resolution"])
        self.assertAlmostEqual(parameters["body_slice_min_z"], expected)
        self.assertLess(parameters["obstacle_min_z"], parameters["body_slice_min_z"])
        body_z = .33
        world = np.array([[2., 0., .10], [2., 1., .40]])
        origin = np.array([0., 0., body_z + .08])
        nearest, _ = classify_spatial_rays(world-origin, np.array([1, 1]),
            origin, np.eye(3), body_z+parameters["obstacle_min_z"],
            body_z+parameters["obstacle_max_z"], 7.5, 3., 720, 0, .2,
            unknown_min_z=body_z+parameters["body_slice_min_z"])
        self.assertEqual(np.isfinite(nearest).sum(), 1)  # real body-height wall

    def test_lightweight_world_keeps_existing_height_band(self):
        self.assertEqual(launch_parameters(False)["obstacle_min_z"], .08)


if __name__ == "__main__":
    unittest.main()
