"""TorchScript adapter for wty-yy/go2_rl_lab velocity policies.

The exported ``policy.pt`` accepts one 45-dimensional proprioceptive frame and
maintains its own ten-frame history.  It is deliberately kept separate from the
production ONNX adapters: this backend is for the isolated experiment
workspace only.
"""

from types import SimpleNamespace
import threading

import numpy as np


class Go2RlLabPolicyAdapter:
    """Run a go2_rl_lab TorchScript policy with bounded CPU parallelism."""

    def __init__(self, model_path, threads=1):
        import torch

        thread_count = max(1, int(threads))
        torch.set_num_threads(thread_count)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            # The process already started parallel work; intra-op is still
            # bounded and the policy worker remains the only inference caller.
            pass
        self.torch = torch
        self.thread_count = thread_count
        self.policy = torch.jit.load(str(model_path), map_location="cpu")
        self.policy.eval()
        self.lock = threading.Lock()
        self.steps = 0
        self.reset()

    def get_inputs(self):
        return [SimpleNamespace(name="obs_now", shape=[1, 45])]

    def reset(self):
        with self.lock:
            self.policy.reset()
            self.steps = 0

    def statistics(self):
        with self.lock:
            return self.steps, self.steps, 0.0

    def run(self, output_names, feeds):
        if output_names != ["action"]:
            raise ValueError("go2_rl_lab adapter accepts action output only")
        if "obs_now" not in feeds:
            raise ValueError("go2_rl_lab adapter requires obs_now")
        obs = np.asarray(feeds["obs_now"], dtype=np.float32).reshape(1, 45)
        if not np.isfinite(obs).all():
            raise ValueError("go2_rl_lab observation is not finite")
        with self.lock:
            with self.torch.inference_mode():
                action = self.policy(self.torch.from_numpy(obs)).detach().cpu().numpy().reshape(-1)
            self.steps += 1
        if action.shape != (12,) or not np.isfinite(action).all():
            raise ValueError(f"invalid go2_rl_lab action {action.shape}")
        return [action]
