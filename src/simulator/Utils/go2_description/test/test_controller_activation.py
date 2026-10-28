"""Native step completion, not control ACK, owns the startup handover."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location(
    'controller_activation', Path(__file__).parents[1] /
    'scripts/activate_controllers_while_stepping.py')
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class Clock:
    now = 100.0

    def monotonic(self):
        return self.now


class Stream:
    def __init__(self, clock, samples, on_sample=lambda s: None):
        self.clock, self.samples = clock, iter(samples)
        self.on_sample = on_sample
        self.closed = False

    def next(self, timeout):
        self.clock.now += max(timeout, .1)
        item = next(self.samples, None)
        if item is None:
            return None
        iterations, sim_ns, paused = item
        sample = module.WorldStats(iterations, sim_ns, paused, self.clock.now)
        self.on_sample(sample)
        return sample

    def close(self):
        self.closed = True


class StartupCompletionTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.time_patch = patch.object(module.time, 'monotonic', self.clock.monotonic)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)
        self.paused = module.WorldStats(100, 200_000_000, True, self.clock.now)

    def test_native_json_handles_omitted_defaults_and_uint64_string(self):
        sample = module.WorldStats.from_json('{"simTime":{},"paused":true}')
        self.assertEqual((sample.iterations, sample.sim_ns, sample.paused), (0, 0, True))
        sample = module.WorldStats.from_json(
            '{"iterations":"125","simTime":{"sec":"1","nsec":250000000}}')
        self.assertEqual((sample.iterations, sample.sim_ns, sample.paused),
                         (125, 1_250_000_000, False))

    def test_step_ack_does_not_complete_batch(self):
        stream = Stream(self.clock, [(100, 200_000_000, True),
                                    (115, 230_000_000, False),
                                    (124, 248_000_000, True),
                                    (125, 250_000_000, True)])
        control = Mock(return_value=True)
        drained = module._advance_paused_batch(
            stream, self.paused, 25, control, lambda: None, 1.)
        self.assertEqual(drained.iterations, 125)
        self.assertTrue(drained.paused)
        control.assert_called_once_with('pause: true multi_step: 25')

    def test_switch_done_mid_batch_still_drains_before_single_resume(self):
        future = Mock()
        future.done.return_value = False
        future.result.return_value = SimpleNamespace(ok=True)
        drained = False

        def observed(sample):
            nonlocal drained
            if sample.iterations >= 12:
                future.done.return_value = True
            if sample.iterations == 25 and sample.paused:
                drained = True

        stream = Stream(self.clock, [(0, 0, True), (12, 24_000_000, False),
                                    (25, 50_000_000, True),
                                    (100, 200_000_000, False),
                                    (200, 400_000_000, False),
                                    (300, 600_000_000, False),
                                    (400, 800_000_000, False),
                                    (500, 1_000_000_000, False)], observed)
        requests = []

        def control(world, request):
            if request == 'pause: false':
                self.assertTrue(drained, 'unpause raced unfinished finite steps')
            requests.append(request)
            return True

        node = Mock()
        node.create_client.return_value.call_async.return_value = future
        node.create_client.return_value.wait_for_service.return_value = True
        with patch.object(sys, 'argv', ['activate', '--world', 'fixture.sdf',
                                       '--controller', 'effort']), \
                patch.object(module, '_world_name', return_value='fixture'), \
                patch.object(module, 'WorldStatsStream', return_value=stream), \
                patch.object(module, '_control_world', side_effect=control), \
                patch.object(module.rclpy, 'init'), \
                patch.object(module.rclpy, 'create_node', return_value=node), \
                patch.object(module.rclpy, 'ok', return_value=True), \
                patch.object(module.rclpy, 'spin_once'), \
                patch.object(module.rclpy, 'shutdown'):
            self.assertEqual(module.main(), 0)
        self.assertEqual(requests, ['pause: true multi_step: 25', 'pause: false'])
        self.assertTrue(stream.closed)

    def test_missing_step_completion_is_bounded_and_never_resumes(self):
        control = Mock(return_value=True)
        with self.assertRaisesRegex(module.StartupError, 'STEP_DRAIN_TIMEOUT'):
            module._advance_paused_batch(
                Stream(self.clock, [(115, 230_000_000, False)]), self.paused,
                25, control, lambda: None, .5)
        control.assert_called_once_with('pause: true multi_step: 25')

    def test_resume_ack_without_clock_progress_cannot_report_success(self):
        control = Mock(return_value=True)
        with self.assertRaisesRegex(module.StartupError, 'RUNNING_VERIFICATION_TIMEOUT'):
            module._resume_and_verify(
                Stream(self.clock, [(100, 200_000_000, False)] * 10),
                self.paused, control, lambda: None, .5)
        control.assert_called_once_with('pause: false')

    def test_repause_fails_instead_of_overriding_pause_repeatedly(self):
        control = Mock(return_value=True)
        with self.assertRaisesRegex(module.StartupError, 'REPAUSED_AFTER_RESUME'):
            module._resume_and_verify(
                Stream(self.clock, [(110, 220_000_000, False),
                                    (125, 250_000_000, True)]),
                self.paused, control, lambda: None, 1.)
        control.assert_called_once_with('pause: false')

    def test_ack_rejection_is_explicit(self):
        with self.assertRaisesRegex(module.StartupError, 'RESUME_REQUEST_REJECTED'):
            module._resume_and_verify(Stream(self.clock, []), self.paused,
                                      lambda request: False, lambda: None, 1.)

    def test_missing_native_stats_is_bounded(self):
        with self.assertRaisesRegex(module.StartupError, 'INITIAL_PAUSE_TIMEOUT'):
            module._wait_stats(Stream(self.clock, []), lambda s: s.paused,
                               .5, lambda: None, 'STARTUP_INITIAL_PAUSE_TIMEOUT')


if __name__ == '__main__':
    unittest.main()
