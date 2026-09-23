"""Regression tests for the Go2 locomotion supervisor logic.

Implemented with the standard library ``unittest`` rather than pytest: this
workspace has a broken ``anyio`` pytest plugin in ``~/.local`` that prevents
``python3 -m pytest`` from starting, so a pytest-based test would be silently
skipped by ``colcon test``.  The logic module is imported directly and has no
ROS dependency, so the test also runs without sourcing the ROS environment.
"""

import importlib.util
import math
import unittest
from pathlib import Path

import numpy as np

MODULE = Path(__file__).parents[1] / "scripts" / "go2_locomotion_logic.py"
spec = importlib.util.spec_from_file_location("go2_locomotion_logic", MODULE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SupervisorSyntaxTest(unittest.TestCase):
    def test_supervisor_script_compiles(self):
        supervisor = MODULE.with_name("go2_locomotion_supervisor.py")
        compile(supervisor.read_text(), str(supervisor), "exec")


class ProjectedGravityTest(unittest.TestCase):
    def test_identity_points_down(self):
        gravity = module.projected_gravity_from_quaternion(0.0, 0.0, 0.0, 1.0)
        np.testing.assert_allclose(gravity, [0.0, 0.0, -1.0], atol=1e-7)

    def test_rotates_into_body_frame(self):
        half = math.sqrt(0.5)
        gravity = module.projected_gravity_from_quaternion(half, 0.0, 0.0, half)
        np.testing.assert_allclose(gravity, [0.0, -1.0, 0.0], atol=1e-7)

    def test_rejects_zero_norm(self):
        with self.assertRaises(ValueError):
            module.projected_gravity_from_quaternion(0.0, 0.0, 0.0, 0.0)


class PdEffortTest(unittest.TestCase):
    def test_go2_torque_limits_match_leg_major_actuator_types(self):
        expected_leg = np.array([23.7, 23.7, 35.55])
        np.testing.assert_allclose(
            module.GO2_TORQUE_LIMITS.reshape(4, 3),
            np.tile(expected_leg, (4, 1)))

    def test_clips_torque(self):
        effort = module.pd_effort(
            np.ones(12), np.zeros(12), np.zeros(12), 40.0, 1.0, 33.5)
        np.testing.assert_allclose(effort, np.full(12, 33.5))

    def test_damping_opposes_velocity(self):
        effort = module.pd_effort(
            np.zeros(12), np.zeros(12), np.ones(12), 40.0, 2.0, 33.5)
        np.testing.assert_allclose(effort, np.full(12, -2.0))

    def test_hold_effort_includes_support_feedforward(self):
        effort = module.hold_effort(
            np.zeros(12), np.full(12, 3.0), np.zeros(12),
            np.zeros(12), 40.0, 4.0, 33.5)
        np.testing.assert_allclose(effort, np.full(12, 3.0))


class GetUpInterpolationTest(unittest.TestCase):
    def test_clamps_before_and_after_trajectory(self):
        start = np.zeros(12)
        target = np.ones(12)
        before, before_progress = module.interpolate_joint_target(
            start, target, -1.0, 2.0)
        after, after_progress = module.interpolate_joint_target(
            start, target, 3.0, 2.0)
        np.testing.assert_allclose(before, start)
        np.testing.assert_allclose(after, target)
        self.assertEqual(before_progress, 0.0)
        self.assertEqual(after_progress, 1.0)

    def test_matches_reference_getup_midpoint(self):
        start = np.zeros(12)
        target = np.full(12, 2.0)
        result, progress = module.interpolate_joint_target(
            start, target, 1.0, 2.0)
        np.testing.assert_allclose(result, np.ones(12))
        self.assertEqual(progress, 0.5)

    def test_rejects_invalid_shape_or_duration(self):
        with self.assertRaises(ValueError):
            module.interpolate_joint_target(np.zeros(11), np.zeros(12), 0.0, 1.0)
        with self.assertRaises(ValueError):
            module.interpolate_joint_target(np.zeros(12), np.zeros(12), 0.0, 0.0)


class HoldTransitionTest(unittest.TestCase):
    def test_estimate_hold_uses_per_joint_medians(self):
        positions = np.vstack([
            np.zeros(12), np.ones(12), np.full(12, 100.0)])
        efforts = np.vstack([
            np.full(12, -2.0), np.full(12, 4.0), np.full(12, 50.0)])
        velocities = np.zeros((3, 12))
        target, feedforward = module.estimate_hold(
            positions, velocities, efforts)
        np.testing.assert_allclose(target, np.ones(12))
        np.testing.assert_allclose(feedforward, np.full(12, 4.0))

    def test_estimate_hold_rejects_empty_or_mismatched_samples(self):
        with self.assertRaises(ValueError):
            module.estimate_hold([], [], [])
        with self.assertRaises(ValueError):
            module.estimate_hold(
                np.zeros((2, 12)), np.zeros((2, 12)), np.zeros((3, 12)))

    def test_blend_effort_has_bumpless_endpoints_and_smooth_midpoint(self):
        policy = np.zeros(12)
        hold = np.full(12, 10.0)
        np.testing.assert_allclose(module.blend_effort(policy, hold, 0.0), policy)
        np.testing.assert_allclose(module.blend_effort(policy, hold, 0.5), 5.0)
        np.testing.assert_allclose(module.blend_effort(policy, hold, 1.0), hold)


class PostureErrorTest(unittest.TestCase):
    """A folded leg must fail the stance check even when other groups look fine."""

    def test_exact_stance_is_zero_error(self):
        self.assertLess(
            module.posture_error(module.DEFAULT_DOF_POS.copy()), 1e-9)

    def test_calf_fold_is_detected(self):
        position = module.DEFAULT_DOF_POS.copy()
        position[2::3] = -2.7
        self.assertGreater(module.posture_error(position), 0.45)

    def test_thigh_fold_is_detected(self):
        position = module.DEFAULT_DOF_POS.copy()
        position[1::3] = 3.4
        self.assertGreater(module.posture_error(position), 0.45)

    def test_reported_failure_case_exceeds_default_limit(self):
        """The observed folded pose (thigh ~3.49 rad) must trip the limit."""
        position = module.DEFAULT_DOF_POS.copy()
        position[1::3] = 3.49
        self.assertGreater(module.posture_error(position), 0.45)

    def test_verified_policy_equilibrium_fits_stand_limit(self):
        """The measured motionless ONNX stance must not be rejected."""
        position = np.array([
            -0.071101, 0.922588, -1.736599,
            -0.059209, 0.933344, -1.838856,
            0.084372, 0.950726, -1.686631,
            -0.137115, 0.950058, -1.868009,
        ])
        self.assertLess(module.posture_error(position), 0.45)


class TiltTest(unittest.TestCase):
    def test_level_robot_is_upright(self):
        self.assertAlmostEqual(
            module.tilt_cos(module.projected_gravity_from_quaternion(
                0.0, 0.0, 0.0, 1.0)), 1.0, places=6)

    def test_rolled_robot_is_not_upright(self):
        half = math.sqrt(0.5)
        gravity = module.projected_gravity_from_quaternion(
            half, 0.0, 0.0, half)
        self.assertLess(module.tilt_cos(gravity), 0.9)


class PlanarPoseValidityTest(unittest.TestCase):
    def test_level_pose_above_minimum_is_valid(self):
        self.assertTrue(module.planar_pose_valid(
            0.0, 0.0, 0.0, 1.0, 0.30,
            math.cos(math.radians(55.0)), 0.15))

    def test_sideways_pose_is_invalid(self):
        half = math.sqrt(0.5)
        self.assertFalse(module.planar_pose_valid(
            half, 0.0, 0.0, half, 0.30,
            math.cos(math.radians(55.0)), 0.15))

    def test_low_pose_is_invalid(self):
        self.assertFalse(module.planar_pose_valid(
            0.0, 0.0, 0.0, 1.0, 0.10,
            math.cos(math.radians(55.0)), 0.15))


class LimitedCommandTest(unittest.TestCase):
    def test_go2_limits_admit_original_deployment_speed(self):
        np.testing.assert_allclose(
            module.GO2_COMMAND_LIMITS, [1.2, 0.5, 1.0])

    def test_clamps_request_and_applies_slew_limit(self):
        result = module.advance_limited_command(
            [0.0, 0.0, 0.0], [2.0, -2.0, 2.0],
            [0.25, 0.15, 0.50], [0.50, 0.40, 1.00], 0.02)
        np.testing.assert_allclose(result, [0.01, -0.008, 0.02])

    def test_repeated_updates_stop_at_configured_limits(self):
        command = np.zeros(3)
        for _ in range(200):
            command = module.advance_limited_command(
                command, [1.0, 1.0, 1.0],
                [0.25, 0.15, 0.50], [0.50, 0.40, 1.00], 0.02)
        np.testing.assert_allclose(command, [0.25, 0.15, 0.50])

    def test_timeout_target_ramps_toward_zero(self):
        result = module.advance_limited_command(
            [0.20, -0.10, 0.30], [0.0, 0.0, 0.0],
            [0.25, 0.15, 0.50], [0.50, 0.40, 1.00], 0.02)
        np.testing.assert_allclose(result, [0.19, -0.092, 0.28])


class BodyMotionStableTest(unittest.TestCase):
    def test_settled_base_is_stable(self):
        self.assertTrue(module.body_motion_stable(
            [0.03, 0.02, 0.0], [0.05, 0.02, 0.01], 0.08, 0.25))

    def test_translation_rejects_stand_verification(self):
        self.assertFalse(module.body_motion_stable(
            [0.09, 0.0, 0.0], [0.0, 0.0, 0.0], 0.08, 0.25))

    def test_rotation_rejects_stand_verification(self):
        self.assertFalse(module.body_motion_stable(
            [0.0, 0.0, 0.0], [0.0, 0.0, 0.26], 0.08, 0.25))

    def test_joint_motion_requires_every_joint_to_be_slow(self):
        self.assertTrue(module.joint_motion_stable(np.ones(12), 2.0))
        velocity = np.ones(12)
        velocity[7] = 2.1
        self.assertFalse(module.joint_motion_stable(velocity, 2.0))


class UprightForHoldTest(unittest.TestCase):
    def test_captures_an_upright_body_at_required_height(self):
        self.assertTrue(module.upright_for_hold(
            0.25, [0.0, 0.0, -1.0], [0.10, 0.0, 0.0],
            [0.0, 0.0, 0.5], 0.24, math.cos(math.radians(30.0)),
            0.20, 1.0))

    def test_rejects_a_low_body(self):
        self.assertFalse(module.upright_for_hold(
            0.20, [0.0, 0.0, -1.0], [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0], 0.24, math.cos(math.radians(30.0)),
            0.20, 1.0))

    def test_rejects_a_tipped_body(self):
        self.assertFalse(module.upright_for_hold(
            0.30, [0.0, -1.0, 0.0], [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0], 0.24, math.cos(math.radians(30.0)),
            0.20, 1.0))

    def test_rejects_a_fast_upright_transition(self):
        self.assertFalse(module.upright_for_hold(
            0.30, [0.0, 0.0, -1.0], [0.30, 0.0, 0.0],
            [0.0, 0.0, 2.0], 0.24, math.cos(math.radians(30.0)),
            0.20, 1.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
