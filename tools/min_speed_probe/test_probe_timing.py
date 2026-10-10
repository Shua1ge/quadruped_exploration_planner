"""ROS imports required; test clock/readiness logic without starting a ROS node."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from probe_run import MinSpeedProbe


class ProbeTimingTests(unittest.TestCase):
    def fake_probe(self, phase="wait_odom"):
        logger = SimpleNamespace(info=Mock(), error=Mock())
        probe = SimpleNamespace(
            DONE="done", SETTLE="settle", DRIVE="drive", REST="rest",
            phase=phase, phase_start=0.0, require_ready=True,
            actuation_ready=False, ready_wall=0.0, started_wall=0.0,
            wait_timeout=60.0, abort_reason=None,
            sample_time=lambda: 3.5, send_cmd=Mock(), get_logger=lambda: logger,
            pose=(0.0, 0.0, 0.30, 0.0, 0.0, 0.0, 1.0),
            rows=[], levels=[0.3], level_idx=0, stand_z_min=0.20,
            imu_w=(0.0, 0.0, 0.0), fallen=False,
        )

        def begin(name):
            probe.phase = name
            probe.phase_start = probe.sample_time()

        probe.begin_phase = begin
        probe.abort = lambda reason: MinSpeedProbe.abort(probe, reason)
        return probe

    def test_sim_clock_is_used_instead_of_wall_clock(self):
        probe = SimpleNamespace(
            use_sim_time=True,
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(nanoseconds=3_500_000_000)),
        )
        with patch("probe_run.time.monotonic", return_value=100):
            self.assertAlmostEqual(MinSpeedProbe.sample_time(probe), 3.5)

    def test_legacy_wall_clock_remains_available(self):
        with patch("probe_run.time.monotonic", return_value=100):
            self.assertEqual(MinSpeedProbe.sample_time(SimpleNamespace(use_sim_time=False)), 100)

    def test_standing_height_is_not_enough_without_controller_ready(self):
        probe = self.fake_probe()
        with patch("probe_run.time.monotonic", return_value=5):
            MinSpeedProbe.tick(probe)
        self.assertEqual(probe.phase, "wait_odom")
        self.assertEqual(probe.rows, [])
        probe.send_cmd.assert_called_once_with(0.0)

    def test_readiness_timeout_ends_without_fake_speed_samples(self):
        probe = self.fake_probe()
        with patch("probe_run.time.monotonic", return_value=61):
            MinSpeedProbe.tick(probe)
        self.assertEqual(probe.phase, "done")
        self.assertIn("timeout", probe.abort_reason)
        self.assertEqual(probe.rows, [])

    def test_lost_readiness_aborts_current_drive(self):
        probe = self.fake_probe("drive")
        with patch("probe_run.time.monotonic", return_value=5):
            MinSpeedProbe.tick(probe)
        self.assertEqual(probe.phase, "done")
        self.assertIn("lost actuation", probe.abort_reason)
        self.assertEqual(probe.rows, [])

    def test_stale_readiness_is_not_accepted(self):
        probe = self.fake_probe("drive")
        probe.actuation_ready = True
        probe.ready_wall = 1.0
        with patch("probe_run.time.monotonic", return_value=5):
            MinSpeedProbe.tick(probe)
        self.assertEqual(probe.phase, "done")


if __name__ == "__main__":
    unittest.main(verbosity=2)
