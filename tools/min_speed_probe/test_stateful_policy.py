"""Adapter contract tests; optional supplied-model sequence equivalence test."""

import os
import unittest
import zipfile
from types import SimpleNamespace

import numpy as np

from stateful_policy import (
    ACTOR_INPUTS, LEGACY_INPUTS, STATEFUL_INPUTS,
    ProbePolicyAdapter, session_for,
)


def inputs():
    return {name: np.zeros(shape, dtype=np.float32)
            for name, shape in LEGACY_INPUTS.items()}


class FakeActor:
    def __init__(self):
        self.calls = []

    def get_inputs(self):
        return [SimpleNamespace(name=name, shape=shape)
                for name, shape in ACTOR_INPUTS.items()]

    def get_outputs(self):
        return [SimpleNamespace(name="action")]

    def run(self, names, feeds):
        self.calls.append({name: value.copy() for name, value in feeds.items()})
        value = feeds["mcp_code"][0, 0] + feeds["obs"][0, 0]
        return [np.full((1, 12), value, dtype=np.float32)]


class FakeFull:
    def __init__(self):
        self.calls = []
        self.bad_state = False

    def get_inputs(self):
        return [SimpleNamespace(name=name, shape=shape)
                for name, shape in STATEFUL_INPUTS.items()]

    def get_outputs(self):
        return [SimpleNamespace(name=name)
                for name in ("action", "shared_state_next", "mcp_code")]

    def run(self, names, feeds):
        self.calls.append({name: value.copy() for name, value in feeds.items()})
        state = feeds["shared_state"] + np.float32(1)
        mcp = np.full((1, 69), feeds["obs_now"][0, 0] + state[0, 0], dtype=np.float32)
        action = np.full((1, 12), mcp[0, 0] + feeds["obs_now"][0, 0], dtype=np.float32)
        if self.bad_state:
            state[0, 0] = np.nan
        return [action, state, mcp]


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.full = FakeFull()
        self.actor = FakeActor()
        self.adapter = ProbePolicyAdapter(
            full_session=self.full, actor_session=self.actor, refresh_every=5)

    def test_accepts_existing_supervisor_signature(self):
        self.assertEqual(
            {item.name: item.shape for item in self.adapter.get_inputs()}, LEGACY_INPUTS)

    def test_refreshes_on_first_sixth_eleventh_processed_actor_step(self):
        feeds = inputs()
        for step in range(11):
            feeds["obs_now"][0, 0] = step
            self.adapter.run(["action"], feeds)
        self.assertEqual(len(self.full.calls), 3)
        self.assertEqual(len(self.actor.calls), 8)
        self.assertEqual([call["obs_now"][0, 0] for call in self.full.calls], [0, 5, 10])
        np.testing.assert_array_equal(self.full.calls[0]["shared_state"], 0)
        np.testing.assert_array_equal(self.full.calls[1]["shared_state"], 1)
        np.testing.assert_array_equal(self.full.calls[2]["shared_state"], 2)

    def test_non_refresh_actor_uses_latest_obs_and_cached_mcp(self):
        feeds = inputs()
        feeds["obs_now"][0, 0] = 2
        self.adapter.run(["action"], feeds)
        feeds["obs_now"][0, 0] = 7
        action = self.adapter.run(["action"], feeds)[0]
        np.testing.assert_array_equal(self.actor.calls[0]["obs"][0, 0], 7)
        np.testing.assert_array_equal(self.actor.calls[0]["mcp_code"], 3)
        np.testing.assert_array_equal(action, 10)

    def test_uses_last_depth_frame_and_correct_flag_shape(self):
        feeds = inputs()
        feeds["depth_camera"][:, 0] = 1
        feeds["depth_camera"][:, 1] = 2
        self.adapter.run(["action"], feeds)
        self.assertEqual(self.full.calls[0]["depth"].shape, (1, 1, 58, 87))
        np.testing.assert_array_equal(self.full.calls[0]["depth"], 2)
        self.assertEqual(self.full.calls[0]["vision_flag"].shape, (1, 1))

    def test_zero_velocity_does_not_reset_recurrent_state(self):
        feeds = inputs()
        for _ in range(6):
            self.adapter.run(["action"], feeds)
        np.testing.assert_array_equal(self.adapter.shared_state, 2)

    def test_explicit_new_episode_resets_state_and_cache(self):
        self.adapter.run(["action"], inputs())
        self.adapter.reset()
        self.assertEqual(self.adapter.statistics(), (0, 0, 0.0))
        self.assertIsNone(self.adapter.mcp_code)
        self.adapter.run(["action"], inputs())
        np.testing.assert_array_equal(self.full.calls[-1]["shared_state"], 0)

    def test_refresh_every_one_updates_state_each_step(self):
        adapter = ProbePolicyAdapter(
            full_session=self.full, actor_session=self.actor, refresh_every=1)
        for _ in range(3):
            adapter.run(["action"], inputs())
        self.assertEqual(len(self.full.calls), 3)
        self.assertEqual(len(self.actor.calls), 0)

    def test_invalid_interval_rejected(self):
        for interval in (0, -1, 1.5, True):
            with self.subTest(interval=interval), self.assertRaises(ValueError):
                ProbePolicyAdapter(full_session=self.full, actor_session=self.actor,
                                   refresh_every=interval)

    def test_invalid_input_rejected_before_running_model(self):
        for kind in ("nan", "shape", "dtype"):
            feeds = inputs()
            if kind == "nan":
                feeds["obs_now"][0, 0] = np.nan
            elif kind == "shape":
                feeds["obs_now"] = np.zeros((1, 44), dtype=np.float32)
            else:
                feeds["obs_now"] = feeds["obs_now"].astype(np.float64)
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.adapter.run(["action"], feeds)
        self.assertEqual(len(self.full.calls), 0)

    def test_bad_state_is_not_committed(self):
        self.full.bad_state = True
        with self.assertRaises(ValueError):
            self.adapter.run(["action"], inputs())
        np.testing.assert_array_equal(self.adapter.shared_state, 0)
        self.assertEqual(self.adapter.actor_steps, 0)
        self.assertIsNone(self.adapter.mcp_code)


class SuppliedModelTests(unittest.TestCase):
    @unittest.skipUnless(os.environ.get("PARKOUR_PROBE_MODEL")
                         and os.environ.get("PARKOUR_PROBE_ARCHIVE"),
                         "set PARKOUR_PROBE_MODEL and PARKOUR_PROBE_ARCHIVE for actual-model test")
    def test_split_rate_sequence_matches_original_assets(self):
        archive_path = os.environ["PARKOUR_PROBE_ARCHIVE"]
        adapter = ProbePolicyAdapter(os.environ["PARKOUR_PROBE_MODEL"], archive_path)
        with zipfile.ZipFile(archive_path) as archive:
            estimator = session_for(archive.read("policies/parkour_moe_trt_assets/estimator.onnx"))
            actor = session_for(archive.read("policies/parkour_moe_trt_assets/actor.onnx"))
        state = np.zeros((1, 64), dtype=np.float32)
        mcp = None
        rng = np.random.default_rng(20261010)
        for step in range(100):
            feeds = inputs()
            feeds["proprio_history"] = rng.normal(0, 0.2, (1, 450)).astype(np.float32)
            feeds["obs_now"] = rng.normal(0, 0.2, (1, 45)).astype(np.float32)
            if step % 5 == 0:
                mcp, state = estimator.run(["mcp_code", "shared_state_next"], {
                    "proprio_history": feeds["proprio_history"],
                    "depth": np.zeros((1, 1, 58, 87), dtype=np.float32),
                    "vision_flag": np.zeros((1, 1), dtype=np.float32),
                    "obs_now": feeds["obs_now"], "shared_state": state,
                })
            expected = actor.run(["action"], {"mcp_code": mcp, "obs": feeds["obs_now"]})[0]
            actual = adapter.run(["action"], feeds)[0]
            np.testing.assert_allclose(actual, expected, atol=1e-5, rtol=1e-4)
            np.testing.assert_allclose(adapter.shared_state, state, atol=1e-5, rtol=1e-4)
        self.assertEqual(adapter.statistics()[:2], (100, 20))


if __name__ == "__main__":
    unittest.main(verbosity=2)
