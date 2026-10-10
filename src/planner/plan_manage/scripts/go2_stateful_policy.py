"""Stateful Parkour MoE runtime with a cached estimator and current actor inputs.

Accepts the supervisor's observation view; only the model boundary changes.
One recurrent-state update runs every ``refresh_every`` processed actor steps.
Stopping or replacing a navigation goal is not a new policy episode.
"""

import threading
from types import SimpleNamespace

import numpy as np


LEGACY_INPUTS = {
    "proprio_history": [1, 450], "depth_camera": [1, 2, 58, 87],
    "obs_now": [1, 45], "use_vision": [1],
}
STATEFUL_INPUTS = {
    "proprio_history": [1, 450], "depth": [1, 1, 58, 87],
    "vision_flag": [1, 1], "obs_now": [1, 45], "shared_state": [1, 64],
}
ACTOR_INPUTS = {"mcp_code": [1, 69], "obs": [1, 45]}


def session_for(model):
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    return ort.InferenceSession(
        model, sess_options=options, providers=["CPUExecutionProvider"])


def checked_array(name, value, shape):
    value = np.asarray(value)
    if value.dtype != np.float32 or list(value.shape) != shape:
        raise ValueError(f"{name}: expected float32 {shape}, got {value.dtype} {value.shape}")
    if not np.isfinite(value).all():
        raise ValueError(f"{name}: non-finite input/output")
    return value


class StatefulPolicyAdapter:
    def __init__(self, model_path=None, actor_model_path=None, refresh_every=5,
                 *, full_session=None, actor_session=None):
        if isinstance(refresh_every, bool) or int(refresh_every) != refresh_every or refresh_every < 1:
            raise ValueError("refresh_every must be a positive integer")
        if full_session is None:
            full_session = session_for(str(model_path))
        if actor_session is None:
            actor_session = session_for(str(actor_model_path))
        for label, session, expected in (
                ("full stateful model", full_session, STATEFUL_INPUTS),
                ("actor", actor_session, ACTOR_INPUTS)):
            actual = {item.name: item.shape for item in session.get_inputs()}
            if actual != expected:
                raise ValueError(f"unexpected {label} signature: {actual}")
        required = {"action", "shared_state_next", "mcp_code"}
        if not required.issubset({item.name for item in full_session.get_outputs()}):
            raise ValueError("stateful model must expose action, shared_state_next and mcp_code")
        if "action" not in {item.name for item in actor_session.get_outputs()}:
            raise ValueError("actor must expose action")
        self.full_session = full_session
        self.actor_session = actor_session
        self.refresh_every = int(refresh_every)
        self.lock = threading.Lock()
        self.reset()

    def get_inputs(self):
        # The adapter's accepted interface, not the raw model's interface.
        return [SimpleNamespace(name=name, shape=shape.copy())
                for name, shape in LEGACY_INPUTS.items()]

    def reset(self):
        with self.lock:
            self.shared_state = np.zeros((1, 64), dtype=np.float32)
            self.mcp_code = None
            self.actor_steps = 0
            self.estimator_refreshes = 0

    def statistics(self):
        with self.lock:
            return (self.actor_steps, self.estimator_refreshes,
                    float(np.linalg.norm(self.shared_state)))

    def run(self, output_names, feeds):
        if output_names != ["action"] or set(feeds) != set(LEGACY_INPUTS):
            raise ValueError("stateful adapter accepts action with the supervisor's four inputs")
        values = {name: checked_array(name, feeds[name], shape)
                  for name, shape in LEGACY_INPUTS.items()}
        with self.lock:
            refresh = self.mcp_code is None or self.actor_steps % self.refresh_every == 0
            if refresh:
                action, state, mcp = self.full_session.run(
                    ["action", "shared_state_next", "mcp_code"], {
                        "proprio_history": values["proprio_history"],
                        "depth": values["depth_camera"][:, -1:, :, :].copy(),
                        "vision_flag": values["use_vision"].reshape(1, 1),
                        "obs_now": values["obs_now"],
                        "shared_state": self.shared_state,
                    })
                action = checked_array("action", action, [1, 12])
                state = checked_array("shared_state_next", state, [1, 64])
                mcp = checked_array("mcp_code", mcp, [1, 69])
                # No partial state/cache commit when a model output is invalid.
                self.shared_state = state.copy()
                self.mcp_code = mcp.copy()
                self.estimator_refreshes += 1
            else:
                action = checked_array("action", self.actor_session.run(
                    ["action"], {"mcp_code": self.mcp_code,
                                 "obs": values["obs_now"]})[0], [1, 12])
            self.actor_steps += 1
            return [action]
