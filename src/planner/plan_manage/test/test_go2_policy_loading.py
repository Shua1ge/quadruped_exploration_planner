"""Exercise the ordinary supervisor loader, without moving a robot."""

from collections import deque
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import rclpy

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from go2_locomotion_supervisor import Go2LocomotionSupervisor
from go2_locomotion_logic import DEFAULT_DOF_POS, GO2_TORQUE_LIMITS, JOINT_NAMES
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from scan_planner_msgs.msg import LocomotionState
from std_msgs.msg import String
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

    def test_default_loads_old_model_with_original_input_signature(self):
        node = self.load()
        self.assertEqual(node.locomotion_policy, "legacy")
        self.assertNotIsInstance(node.onnx_session, StatefulPolicyAdapter)
        self.assertEqual({item.name: item.shape for item in node.onnx_session.get_inputs()},
                         LEGACY_INPUTS)
        self.assertFalse(node.navigation_enabled)
        self.assertTrue(np.isfinite(node.onnx_session.run(["action"], inputs())[0]).all())

    def test_explicit_new_policy_loads_packaged_model_and_matching_actor(self):
        node = self.load("locomotion_policy:=parkour_moe_stateful")
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
        node = self.load("locomotion_policy:=parkour_moe_stateful",
                         "model_path:=''", "actor_model_path:=''")
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

    def test_background_trace_writer_flushes_risk_and_fault_on_shutdown(self):
        node = self.load()
        node.timer.cancel()
        with tempfile.TemporaryDirectory() as directory:
            node.trace_directory = directory
            node.motion_trace.append(dict(q=[1.]*12, action=[2.]*12, effort=[3.]*12))
            node.record_posture_event('LOW_EXTENSION_RISK', 'LOW_LEG_EXTENSION_ONLY')
            node.set_state(LocomotionState.STATE_FAULT, 'IMU_FEEDBACK_INVALID')
            node.shutdown()
            paths = list(Path(directory).glob('*.json'))
            self.assertEqual(len(paths), 2)
            records = [json.loads(path.read_text()) for path in paths]
            self.assertEqual({row['event'] for row in records}, {'FAULT', 'LOW_EXTENSION_RISK'})
            self.assertTrue(all(row['samples'][0]['effort'] == [3.]*12 for row in records))

    def test_actual_worker_correlates_and_installs_packaged_model_results(self):
        self.check_worker_results("parkour_moe_stateful", 3)

    def test_actual_worker_correlates_and_installs_default_legacy_results(self):
        self.check_worker_results(None, None)

    def check_worker_results(self, policy, estimator_refresh_count):
        overrides = () if policy is None else (f"locomotion_policy:={policy}",)
        node = self.load(*overrides)
        node.timer.cancel()
        node.begin_policy_phase()
        # No executor/control loop, no navigation service or motor commands.
        # Only run the real background ONNX worker against the default inputs.
        next_observation = time.monotonic()
        for _ in range(11):
            time.sleep(max(0., next_observation - time.monotonic()))
            next_observation += .02
            self.assertTrue(node.evaluate_policy(np.zeros(3)))
            deadline = time.monotonic() + 1.
            while time.monotonic() < deadline:
                with node.policy_lock:
                    ready = node.policy_result is not None
                if ready:
                    break
                time.sleep(.001)
            self.assertTrue(ready, 'ONNX worker did not return within one second')
            self.assertTrue(node.consume_policy_result())
            self.assertTrue(np.isfinite(node.desired_target).all())
        received = []
        node.create_subscription(String, '/robot/locomotion_policy_timing', received.append, 10)
        node.publish_policy_timing()
        deadline = time.monotonic() + 1.
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        self.assertTrue(received, 'timing topic did not deliver its JSON report')
        report = json.loads(received[0].data)
        self.assertTrue(report['blind'])
        self.assertEqual(report['window_counts']['requested'], 11)
        self.assertEqual(report['window_counts']['inferred'], 11)
        self.assertEqual(report['window_counts']['installed'], 11)
        self.assertEqual(report['latest']['sequence'], 11)
        self.assertEqual(report['latest']['previous_action_sequence'], 10)
        self.assertEqual(report['latest']['estimator_refresh_count'], estimator_refresh_count)
        self.assertIsNone(report['latest']['joint_sample_age_ms'])
        self.assertIsNone(report['latest']['joint_receipt_age_ms'])
        self.assertEqual(report['summary']['inference_wall_ms']['count'], 11)


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


class FaultOutputRuntimeTests(unittest.TestCase):
    def fixture(self):
        return SimpleNamespace(
            mode_name='rl_effort', have_joint_state=True, have_imu=True,
            sensor_source_ns=dict(joint=None, imu=None, odom=None),
            odom_fresh=Mock(return_value=True), last_joint_ns=1_000_000_000,
            last_imu_ns=1_000_000_000, state_timeout=.5,
            q=DEFAULT_DOF_POS.copy(), dq=np.zeros(12), gravity_body=np.array([0., 0., -1.]),
            base_lin_vel=np.zeros(3), base_ang_vel=np.zeros(3),
            reason='BODY_POSE_INVALID_MAP_RESTART_REQUIRED', fault_output_mode=None,
            fault_hold_target=None, fault_hold_feedforward=None, last_effort=np.full(12, 3.),
            torque_limits=GO2_TORQUE_LIMITS, get_logger=Mock(), publish_effort=Mock(),
            zero_output=Mock())

    def test_upright_hold_keeps_support_torque_instead_of_turning_off_motors(self):
        node = self.fixture()
        Go2LocomotionSupervisor.fault_output(node, 1_010_000_000)
        self.assertEqual(node.fault_output_mode, 'SUPPORT_HOLD')
        node.zero_output.assert_not_called()
        np.testing.assert_allclose(node.publish_effort.call_args.args[0], 3.)

    def test_stale_feedback_disables_output_even_after_a_hold_was_captured(self):
        node = self.fixture()
        Go2LocomotionSupervisor.fault_output(node, 1_010_000_000)
        Go2LocomotionSupervisor.fault_output(node, 2_000_000_000)
        self.assertEqual(node.fault_output_mode, 'DISABLED')
        node.zero_output.assert_called_once()

    def test_dynamic_fault_uses_bounded_damping_and_cannot_recapture_a_pose(self):
        node = self.fixture()
        node.base_lin_vel[0] = .7
        node.dq[:] = 20.
        Go2LocomotionSupervisor.fault_output(node, 1_010_000_000)
        self.assertEqual(node.fault_output_mode, 'DAMPING_ONLY')
        self.assertTrue(np.all(np.abs(node.publish_effort.call_args.args[0]) <= .25*GO2_TORQUE_LIMITS))
        node.base_lin_vel.fill(0.)
        node.dq.fill(0.)
        Go2LocomotionSupervisor.fault_output(node, 1_020_000_000)
        self.assertEqual(node.fault_output_mode, 'DAMPING_ONLY')

    def test_low_world_elevation_alone_does_not_create_an_rl_pose_fault(self):
        node = self.fixture()
        node.get_clock = Mock(return_value=SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_010_000_000)))
        node.max_tilt_cos = .57
        node.min_body_height = .15
        msg = Odometry()
        msg.pose.pose.position.z = -1.0
        msg.pose.pose.orientation.w = 1.
        Go2LocomotionSupervisor.odom_callback(node, msg)
        self.assertTrue(node.odom_pose_valid)
        self.assertEqual(node.body_height, -1.)


class RunningPolicySupportTests(unittest.TestCase):
    """Drive the real control branch with fake outputs, never real motors."""

    def fixture(self):
        now = 1_000_000_000
        node = SimpleNamespace(
            mode_name='rl_effort', state=LocomotionState.STATE_RUN, reason='STAND_VERIFIED',
            have_joint_state=True, have_imu=True, navigation_enabled=True,
            last_joint_ns=now, last_imu_ns=now, last_cmd_ns=now, state_timeout=.5,
            cmd_timeout=.5, odom_fresh=Mock(return_value=True), odom_pose_valid=True,
            q=np.tile([0., 2.6, -2.7], 4), dq=np.zeros(12),
            gravity_body=np.array([0., 0., -1.]), support_height=0., body_height=1.617,
            raw_cmd=np.zeros(3), filtered_cmd=np.zeros(3), getup_phase='POLICY',
            policy_cycle=0, policy_decimation=10, policy_dt=.02,
            rl_command_limits=np.array([1.2, .5, 1.]),
            rl_command_slew_rates=np.array([.5, .4, 1.]),
            min_body_height=.15, max_tilt_cos=.57,
            fault_cycles=0, fault_cycles_required=40,
            low_extension_cycles=0, low_extension_active=False,
            desired_target=DEFAULT_DOF_POS.copy(), kp=40., kd=1.,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now)),
            get_logger=Mock(), publish_state=Mock(), zero_output=Mock(),
            evaluate_policy=Mock(return_value=True), consume_policy_result=Mock(return_value=True),
            publish_target_effort=Mock(), fault_output=Mock(), record_posture_event=Mock())
        node.set_state = lambda state, reason: Go2LocomotionSupervisor.set_state(node, state, reason)
        node.observe_leg_extension_risk = lambda: Go2LocomotionSupervisor.observe_leg_extension_risk(node)
        return node

    def test_zero_command_with_sustained_low_reach_keeps_policy_and_pd_support(self):
        node = self.fixture()
        for _ in range(100):
            Go2LocomotionSupervisor.control_loop(node)
        self.assertLess(node.support_height, .15)
        self.assertEqual(node.state, LocomotionState.STATE_RUN)
        self.assertEqual(node.reason, 'STAND_VERIFIED')
        self.assertEqual(node.fault_cycles, 0)
        self.assertEqual(node.evaluate_policy.call_count, 10)
        self.assertEqual(node.publish_target_effort.call_count, 100)
        np.testing.assert_array_equal(node.evaluate_policy.call_args.args[0], np.zeros(3))
        node.zero_output.assert_not_called()
        node.fault_output.assert_not_called()
        node.record_posture_event.assert_called_once_with('LOW_EXTENSION_RISK', 'LOW_LEG_EXTENSION_ONLY')

    def test_low_reach_does_not_revoke_actuation_or_mapping_capability(self):
        node = self.fixture()
        node.support_height = .133
        self.assertEqual(Go2LocomotionSupervisor.capabilities(node, 1_010_000_000), (True, True))

    def test_command_timeout_still_runs_zero_command_policy(self):
        node = self.fixture()
        node.raw_cmd[0] = .6
        node.last_cmd_ns = 1
        Go2LocomotionSupervisor.control_loop(node)
        np.testing.assert_array_equal(node.evaluate_policy.call_args.args[0], np.zeros(3))
        node.publish_target_effort.assert_called_once()
        node.zero_output.assert_not_called()

    def test_tilt_or_invalid_odometry_still_latches_a_fault_and_stops_policy(self):
        for cause in ('tilt', 'odom'):
            with self.subTest(cause=cause):
                node = self.fixture()
                if cause == 'tilt':
                    node.gravity_body = np.array([0., .866, -.5])
                else:
                    node.odom_pose_valid = False
                for _ in range(40):
                    Go2LocomotionSupervisor.control_loop(node)
                self.assertEqual(node.state, LocomotionState.STATE_FAULT)
                count = node.evaluate_policy.call_count
                Go2LocomotionSupervisor.control_loop(node)
                self.assertEqual(node.evaluate_policy.call_count, count)
                self.assertEqual(node.fault_output.call_count, 2)
                self.assertEqual(Go2LocomotionSupervisor.capabilities(node, 1_010_000_000), (False, False))

    def test_invalid_joint_feedback_is_still_a_fault_not_a_reach_warning(self):
        node = self.fixture()
        msg = JointState()
        msg.name = list(JOINT_NAMES)
        msg.position = [float('nan')] * 12
        msg.velocity = [0.] * 12
        Go2LocomotionSupervisor.joint_callback(node, msg)
        self.assertEqual(node.state, LocomotionState.STATE_FAULT)
        self.assertEqual(node.reason, 'JOINT_FEEDBACK_INVALID')
        Go2LocomotionSupervisor.control_loop(node)
        node.evaluate_policy.assert_not_called()
        node.fault_output.assert_called_once()

    def test_reach_recovery_clears_risk_without_resetting_policy(self):
        node = self.fixture()
        for _ in range(40):
            Go2LocomotionSupervisor.control_loop(node)
        self.assertTrue(node.low_extension_active)
        node.q = DEFAULT_DOF_POS.copy()
        Go2LocomotionSupervisor.control_loop(node)
        self.assertFalse(node.low_extension_active)
        self.assertEqual(node.low_extension_cycles, 0)
        self.assertEqual(node.state, LocomotionState.STATE_RUN)


class PostureTracePersistenceTests(unittest.TestCase):
    def fixture(self, directory):
        node = RunningPolicySupportTests().fixture()
        node.epoch = 123
        node.locomotion_policy = 'legacy'
        node.trace_directory = directory
        node.trace_writer = Mock()
        node.fault_trace_pub = Mock()
        node.fault_trace_recorded = False
        node.low_extension_trace_recorded = False
        node.sensor_source_ns = dict(joint=1, imu=2, odom=3)
        node.base_lin_vel = np.zeros(3)
        node.base_ang_vel = np.zeros(3)
        node.last_action = np.zeros(12)
        node.last_effort = np.full(12, 3.)
        node.motion_trace = deque([dict(q=[1.]*12, action=[2.]*12, effort=[3.]*12)], maxlen=100)
        node.persist_posture_trace = lambda trace: Go2LocomotionSupervisor.persist_posture_trace(node, trace)
        node.record_posture_event = lambda event, reason: Go2LocomotionSupervisor.record_posture_event(node, event, reason)
        return node

    def test_fault_is_latched_before_export_and_written_once_with_prefault_samples(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.fixture(directory)
            node.body_height = float('nan')
            node.set_state(LocomotionState.STATE_FAULT, 'ODOM_FEEDBACK_INVALID')
            self.assertEqual(node.state, LocomotionState.STATE_FAULT)
            self.assertFalse(list(Path(directory).glob('*.json')))  # no disk I/O in control callback
            writer, trace = node.trace_writer.submit.call_args.args
            self.assertIsNone(trace['body_world_z'])
            node.motion_trace.clear()
            node.q.fill(0.)
            writer(trace)
            saved = json.loads((Path(directory) / 'go2_123_fault.json').read_text())
            self.assertEqual(saved['samples'][0]['effort'], [3.]*12)
            self.assertEqual(saved['trigger']['q'], np.tile([0., 2.6, -2.7], 4).tolist())
            self.assertEqual(saved['state'], LocomotionState.STATE_FAULT)
            self.assertEqual(saved['sensor_source_ns']['joint'], 1)
            node.set_state(LocomotionState.STATE_FAULT, 'IMU_STALE')
            node.trace_writer.submit.assert_called_once()

    def test_first_low_reach_trace_is_bounded_and_does_not_change_run_state(self):
        node = self.fixture('/tmp/unused_trace_test')
        for _ in range(10):
            node.record_posture_event('LOW_EXTENSION_RISK', 'LOW_LEG_EXTENSION_ONLY')
        node.trace_writer.submit.assert_called_once()
        self.assertEqual(node.state, LocomotionState.STATE_RUN)

    def test_disk_error_keeps_topic_evidence_and_does_not_mutate_control(self):
        node = self.fixture('/tmp/unused_trace_test')
        node.record_posture_event('FAULT', 'IMU_STALE')
        writer, trace = node.trace_writer.submit.call_args.args
        with patch('go2_locomotion_supervisor.os.makedirs', side_effect=OSError('disk unavailable')):
            writer(trace)
        node.fault_trace_pub.publish.assert_called_once()
        self.assertEqual(node.state, LocomotionState.STATE_RUN)
        node.publish_target_effort.assert_not_called()
        node.zero_output.assert_not_called()


class PolicyTimingRuntimeTests(unittest.TestCase):
    """Exercise production callbacks/worker with deterministic clocks, no robot."""

    def fixture(self):
        now = dict(ros=1_000_000_000, wall=1_000_000_000)
        counts = dict(requested=0, inferred=0, installed=0,
                      request_overwrites=0, result_overwrites=0, errors=0)
        node = SimpleNamespace(
            q=DEFAULT_DOF_POS.copy(), dq=np.zeros(12),
            gravity_body=np.array([0., 0., -1.]), base_ang_vel=np.zeros(3),
            filtered_cmd=np.zeros(3), last_action=np.zeros(12),
            policy_command_scale=np.array([2., 2., .25], dtype=np.float32),
            history=deque((np.zeros(45, np.float32) for _ in range(10)), maxlen=10),
            have_joint_state=True, have_imu=True,
            last_joint_ns=now['ros'], last_imu_ns=now['ros'], last_odom_ns=now['ros'],
            sensor_source_ns=dict(joint=900_000_000, imu=950_000_000, odom=800_000_000),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now['ros'])),
            get_logger=Mock(), policy_lock=threading.Lock(),
            policy_event=threading.Event(), policy_stop=threading.Event(),
            policy_request=None, policy_result=None, policy_error=None, policy_inflight=None,
            policy_sequence=0, policy_episode=1, policy_last_installed_sequence=0,
            policy_last_observation_ros_ns=None, policy_latest_timing=None,
            policy_timing_records=deque(maxlen=512), policy_inference_records=deque(maxlen=512),
            policy_timing_counts=counts, policy_timing_report_counts=counts.copy(),
            policy_timing_report_wall_ns=now['wall'], policy_timing_pub=Mock(),
            action_scale=.25, policy_dt=.02, policy_handoff_elapsed=1.,
            policy_handoff_duration=.5, desired_target=DEFAULT_DOF_POS.copy(),
            epoch=1, state=1, reason='RUN', set_state=Mock(), zero_output=Mock())
        node.compute_observation = lambda command: Go2LocomotionSupervisor.compute_observation(node, command)
        return node, now

    def queue(self, node, forward=.6):
        return Go2LocomotionSupervisor.evaluate_policy(node, np.array([forward, 0., 0.]))

    def policy(self, node, hook, refresh_every=5):
        class FakePolicy:
            steps = 0
            refreshes = 0

            def statistics(self):
                return self.steps, self.refreshes, 0.

            def run(self, names, feeds):
                self.refreshes += int(self.steps % refresh_every == 0)
                self.steps += 1
                hook(self.steps)
                return [np.full((1, 12), feeds['obs_now'][0, 6], dtype=np.float32)]
        node.onnx_session = FakePolicy()

    def test_delayed_result_keeps_request_identity_and_measures_source_not_receipt(self):
        node, now = self.fixture()

        def infer(_step):
            now['wall'] += 80_000_000
            now['ros'] += 80_000_000
            self.queue(node, .4)
            self.queue(node, 0.)
            node.policy_stop.set()

        self.policy(node, infer)
        with patch('go2_locomotion_supervisor.time.monotonic_ns', side_effect=lambda: now['wall']):
            self.queue(node)
            Go2LocomotionSupervisor.policy_worker(node)
            now['wall'] += 2_000_000
            now['ros'] += 2_000_000
            self.assertTrue(Go2LocomotionSupervisor.consume_policy_result(node))
            row = node.policy_latest_timing
            self.assertEqual(row['sequence'], 1)
            self.assertEqual(row['previous_action_sequence'], 0)
            self.assertEqual(row['newer_requests'], 2)
            self.assertAlmostEqual(row['inference_wall_ms'], 80.)
            self.assertAlmostEqual(row['result_wait_wall_ms'], 2.)
            self.assertAlmostEqual(row['observation_to_install_wall_ms'], 82.)
            self.assertAlmostEqual(row['observation_to_install_ros_ms'], 82.)
            self.assertAlmostEqual(row['joint_sample_age_ms'], 182.)
            self.assertAlmostEqual(row['joint_receipt_age_ms'], 82.)
            self.assertEqual(node.policy_timing_counts['request_overwrites'], 1)
            self.assertEqual(node.policy_request['timing']['sequence'], 3)
            # This is instrumentation, not a new action-dropping policy.
            np.testing.assert_allclose(node.last_action, 1.2)
            np.testing.assert_allclose(node.desired_target, DEFAULT_DOF_POS + .3)
            Go2LocomotionSupervisor.publish_policy_timing(node)
        report = json.loads(node.policy_timing_pub.publish.call_args.args[0].data)
        self.assertEqual(report['latest']['sequence'], 1)
        self.assertEqual(report['window_counts']['requested'], 3)
        self.assertEqual(report['window_counts']['installed'], 1)
        self.assertEqual(report['pending_sequence'], 3)

    def test_estimator_interval_counts_processed_requests_not_nominal_five_ticks(self):
        node, now = self.fixture()

        def infer(step):
            if step == 6:
                node.policy_stop.set()
            else:
                # Only the newest of three produced requests survives.
                for _ in range(3):
                    now['ros'] += 20_000_000
                    now['wall'] += 20_000_000
                    self.queue(node)

        self.policy(node, infer)
        with patch('go2_locomotion_supervisor.time.monotonic_ns', side_effect=lambda: now['wall']):
            self.queue(node)
            Go2LocomotionSupervisor.policy_worker(node)
            self.assertEqual(node.policy_timing_counts['inferred'], 6)
            self.assertEqual(node.policy_timing_counts['request_overwrites'], 10)
            self.assertEqual(node.policy_timing_counts['result_overwrites'], 5)
            Go2LocomotionSupervisor.consume_policy_result(node)
            row = node.policy_latest_timing
            self.assertTrue(row['estimator_refreshed'])
            self.assertEqual(row['estimator_refresh_count'], 2)
            self.assertEqual(row['estimator_sequence_gap'], 15)
            self.assertAlmostEqual(row['estimator_observation_interval_ros_ms'], 300.)
            self.assertAlmostEqual(row['estimator_execution_interval_ros_ms'], 300.)
            Go2LocomotionSupervisor.publish_policy_timing(node)
        report = json.loads(node.policy_timing_pub.publish.call_args.args[0].data)
        # Overwritten results still contribute to actual inference statistics.
        self.assertEqual(report['summary']['inference_wall_ms']['count'], 6)
        self.assertEqual(report['summary']['estimator_observation_interval_ros_ms']['max'], 300.)

    def test_paused_simulation_does_not_hide_wall_time_latency(self):
        node, now = self.fixture()

        def infer(_step):
            now['wall'] += 80_000_000
            node.policy_stop.set()

        self.policy(node, infer)
        with patch('go2_locomotion_supervisor.time.monotonic_ns', side_effect=lambda: now['wall']):
            self.queue(node)
            Go2LocomotionSupervisor.policy_worker(node)
            now['wall'] += 2_000_000
            Go2LocomotionSupervisor.consume_policy_result(node)
        self.assertEqual(node.policy_latest_timing['observation_to_install_ros_ms'], 0.)
        self.assertAlmostEqual(node.policy_latest_timing['observation_to_install_wall_ms'], 82.)

    def test_callbacks_retain_old_headers_even_when_received_now(self):
        node, now = self.fixture()
        joint = JointState()
        joint.name = list(JOINT_NAMES)
        joint.position = DEFAULT_DOF_POS.tolist()
        joint.velocity = [0.] * 12
        joint.header.stamp.nanosec = 900_000_000
        imu = Imu()
        imu.orientation.w = 1.
        imu.header.stamp.nanosec = 950_000_000
        Go2LocomotionSupervisor.joint_callback(node, joint)
        Go2LocomotionSupervisor.imu_callback(node, imu)
        self.assertEqual(node.last_joint_ns, now['ros'])
        self.assertEqual(node.last_imu_ns, now['ros'])
        self.assertEqual(node.sensor_source_ns['joint'], 900_000_000)
        self.assertEqual(node.sensor_source_ns['imu'], 950_000_000)

    def test_legacy_estimator_frequency_is_unknown_and_episode_mismatch_is_visible(self):
        node, now = self.fixture()

        def run(_names, _feeds):
            node.policy_stop.set()
            return [np.zeros((1, 12), np.float32)]

        node.onnx_session = SimpleNamespace(run=run)
        with patch('go2_locomotion_supervisor.time.monotonic_ns', side_effect=lambda: now['wall']):
            self.queue(node)
            Go2LocomotionSupervisor.policy_worker(node)
            node.policy_episode += 1
            Go2LocomotionSupervisor.consume_policy_result(node)
        self.assertIsNone(node.policy_latest_timing['estimator_refreshed'])
        self.assertFalse(node.policy_latest_timing['episode_matches'])

    def test_model_error_is_counted_without_installing_an_action(self):
        node, now = self.fixture()

        def run(_names, _feeds):
            node.policy_stop.set()
            raise RuntimeError('controlled model failure')

        node.onnx_session = SimpleNamespace(run=run)
        with patch('go2_locomotion_supervisor.time.monotonic_ns', side_effect=lambda: now['wall']):
            self.queue(node)
            Go2LocomotionSupervisor.policy_worker(node)
            self.assertFalse(Go2LocomotionSupervisor.consume_policy_result(node))
        self.assertEqual(node.policy_timing_counts['errors'], 1)
        self.assertEqual(node.policy_timing_counts['installed'], 0)
        self.assertIsNone(node.policy_inflight)
        node.zero_output.assert_called_once()


class LaunchPolicyTests(unittest.TestCase):
    def setup_actions(self, policy=None):
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
            locomotion_mode="rl_effort",
            use_sim_time="true", use_gazebo_physics="true",
            gazebo_world=str(Path(__file__).parents[4] / "tools/min_speed_probe/flat_ground.sdf"))
        if policy is not None:
            context.launch_configurations["locomotion_policy"] = policy
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

    def test_default_launch_uses_legacy_and_keeps_yaml_speed_calibration(self):
        context, nodes = self.setup_actions()
        params = evaluate_parameters(context, normalize_parameters(
            nodes["go2_locomotion_supervisor.py"].parameters))[1]
        self.assertEqual(params["locomotion_policy"], "legacy")
        self.assertEqual(params["model_path"], "")
        self.assertNotIn("fsm.min_forward_effective_speed",
                         nodes["scan_planner_node"].parameters[-1])

    def test_newmine_entry_defaults_to_legacy(self):
        filename = Path(__file__).parents[1] / "launch/newmine_exploration.launch.py"
        spec = importlib.util.spec_from_file_location("go2_newmine_launch_under_test", filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        defaults = {action.name: perform_substitutions(context, action.default_value)
                    for action in module.generate_launch_description().entities
                    if isinstance(action, DeclareLaunchArgument)}
        self.assertEqual(defaults["locomotion_policy"], "legacy")


if __name__ == "__main__":
    unittest.main(verbosity=2)
