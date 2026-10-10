"""Startup failures terminate the launch rather than leave a paused server."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from launch.actions import Shutdown, ExecuteProcess, TimerAction

spec = importlib.util.spec_from_file_location(
    "go2_sim_launch", Path(__file__).parents[1] / "launch/go2_sim.launch.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class StartupLaunchTest(unittest.TestCase):
    def test_activation_failure_shuts_down(self):
        result = module._resume_after_controller(SimpleNamespace(returncode=1), None)
        self.assertTrue(any(isinstance(a, Shutdown) for a in result))

    def test_broadcaster_failure_does_not_start_controller(self):
        sentinel = object()
        result = module._start_trajectory_after_broadcaster(
            SimpleNamespace(returncode=1), None, sentinel)
        self.assertNotIn(sentinel, result)
        self.assertTrue(any(isinstance(a, Shutdown) for a in result))

    def test_command_controller_failure_does_not_activate(self):
        sentinel = object()
        result = module._activate_controllers_after_load(
            SimpleNamespace(returncode=1), None, sentinel)
        self.assertNotIn(sentinel, result)
        self.assertTrue(any(isinstance(a, Shutdown) for a in result))

    def test_spawn_failure_shuts_down(self):
        result = module._resume_after_spawn(SimpleNamespace(returncode=1), None)
        self.assertTrue(any(isinstance(a, Shutdown) for a in result))

    def test_success_starts_next_controller(self):
        sentinel = object()
        self.assertEqual(module._start_trajectory_after_broadcaster(
            SimpleNamespace(returncode=0), None, sentinel), [sentinel])

    def test_launch_has_bounded_startup_and_gated_spawn(self):
        description = module.generate_launch_description()
        # Only publisher, bridge and GUI start at top level. Creation is
        # returned by the readiness gate, not launched in parallel with it.
        self.assertEqual(sum(isinstance(a, ExecuteProcess)
                             for a in description.entities), 3)
        self.assertEqual(sum(isinstance(a, TimerAction)
                             for a in description.entities), 2)


if __name__ == "__main__":
    unittest.main()
