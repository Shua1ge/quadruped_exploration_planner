"""Exercise the ordinary supervisor loader, without moving a robot."""

from collections import deque
import importlib.util
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import rclpy

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from go2_locomotion_supervisor import Go2LocomotionSupervisor
from go2_stateful_policy import LEGACY_INPUTS, StatefulPolicyAdapter
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters, normalize_parameters


def inputs():
    return {name: np.zeros(shape, dtype=np.float32)
            for name, shape in LEGACY_INPUTS.items()}


class SupervisorLoaderTests(unittest.TestCase):
    def setUp(self):
        self.node = None

    def tearDown(self):
        if self.node is not None:
            self.node.shutdown()
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def load(self, *overrides):
        args = ["--ros-args", "-p", "locomotion_mode:=rl_effort"]
        for override in overrides:
            args.extend(["-p", override])
        rclpy.init(args=args)
        self.node = Go2LocomotionSupervisor()
        return self.node

    def test_default_loads_packaged_stateful_model_and_matching_actor(self):
        node = self.load()
        self.assertEqual(node.locomotion_policy, "parkour_moe_stateful")
        self.assertIsInstance(node.onnx_session, StatefulPolicyAdapter)
        self.assertEqual(node.onnx_session.refresh_every, 5)
        self.assertFalse(node.navigation_enabled)
        feeds = inputs()
        for _ in range(11):
            action = node.onnx_session.run(["action"], feeds)[0]
            self.assertEqual(action.shape, (1, 12))
            self.assertTrue(np.isfinite(action).all())
        self.assertEqual(node.onnx_session.statistics()[:2], (11, 3))
        node.begin_policy_phase()
        self.assertEqual(node.onnx_session.statistics(), (0, 0, 0.0))

    def test_launch_empty_paths_select_packaged_defaults(self):
        node = self.load("model_path:=''", "actor_model_path:=''")
        self.assertIsInstance(node.onnx_session, StatefulPolicyAdapter)
        self.assertTrue(np.isfinite(node.onnx_session.run(["action"], inputs())[0]).all())

    def test_legacy_switch_still_loads_old_model(self):
        node = self.load("locomotion_policy:=legacy", "model_path:=''")
        self.assertEqual(node.locomotion_policy, "legacy")
        self.assertNotIsInstance(node.onnx_session, StatefulPolicyAdapter)
        self.assertEqual({item.name: item.shape for item in node.onnx_session.get_inputs()},
                         LEGACY_INPUTS)
        self.assertTrue(np.isfinite(node.onnx_session.run(["action"], inputs())[0]).all())
        # Beginning a legacy policy episode has no recurrent reset method.
        node.begin_policy_phase()


class EpisodeBoundaryTests(unittest.TestCase):
    def test_begin_policy_phase_resets_exactly_once(self):
        reset = Mock()
        observation = np.zeros(45, dtype=np.float32)
        node = SimpleNamespace(
            onnx_session=SimpleNamespace(reset=reset), history=deque(maxlen=10),
            compute_observation=Mock(return_value=observation))
        Go2LocomotionSupervisor.begin_policy_phase(node)
        reset.assert_called_once_with()
        self.assertEqual(len(node.history), 10)
        self.assertEqual(node.policy_handoff_elapsed, 0.0)

    def test_motion_state_change_does_not_reset_policy(self):
        reset = Mock()
        node = SimpleNamespace(
            onnx_session=SimpleNamespace(reset=reset), state=0, reason="old",
            publish_state=Mock())
        Go2LocomotionSupervisor.set_state(node, 1, "RUN")
        Go2LocomotionSupervisor.set_state(node, 1, "HOLD")
        Go2LocomotionSupervisor.set_state(node, 1, "NEW_GOAL")
        reset.assert_not_called()


class LaunchPolicyTests(unittest.TestCase):
    def setup_actions(self, policy):
        filename = Path(__file__).parents[1] / "launch/run.launch.py"
        spec = importlib.util.spec_from_file_location("go2_launch_under_test", filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                context.launch_configurations[action.name] = perform_substitutions(
                    context, action.default_value)
        context.launch_configurations.update(
            locomotion_mode="rl_effort", locomotion_policy=policy,
            use_sim_time="true", use_gazebo_physics="true",
            gazebo_world=str(Path(__file__).parents[4] / "tools/min_speed_probe/flat_ground.sdf"))
        with patch.object(module, "Node", side_effect=lambda **kwargs: SimpleNamespace(**kwargs)):
            actions = module._setup(context)
        return context, {item.executable: item for item in actions
                         if isinstance(item, SimpleNamespace)}

    def test_new_policy_launch_passes_empty_string_paths_and_new_speed_calibration(self):
        context, nodes = self.setup_actions("parkour_moe_stateful")
        params = evaluate_parameters(context, normalize_parameters(
            nodes["go2_locomotion_supervisor.py"].parameters))[1]
        self.assertEqual(params["locomotion_policy"], "parkour_moe_stateful")
        self.assertEqual(params["model_path"], "")
        self.assertEqual(params["actor_model_path"], "")
        self.assertEqual(params["estimator_refresh_every"], 5)
        self.assertEqual(nodes["scan_planner_node"].parameters[-1][
            "fsm.min_forward_effective_speed"], 0.25)

    def test_legacy_launch_keeps_yaml_speed_calibration(self):
        _, nodes = self.setup_actions("legacy")
        self.assertNotIn("fsm.min_forward_effective_speed",
                         nodes["scan_planner_node"].parameters[-1])


if __name__ == "__main__":
    unittest.main(verbosity=2)
