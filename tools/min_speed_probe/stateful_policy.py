"""Speed-probe entry for the same stateful runtime used by normal exploration."""

import sys
import zipfile
from pathlib import Path

scripts = Path(__file__).resolve().parents[2] / "src/planner/plan_manage/scripts"
sys.path.insert(0, str(scripts))
from go2_stateful_policy import (  # noqa: E402
    ACTOR_INPUTS, LEGACY_INPUTS, STATEFUL_INPUTS,
    StatefulPolicyAdapter, session_for,
)


class ProbePolicyAdapter(StatefulPolicyAdapter):
    def __init__(self, model_path=None, archive_path=None, refresh_every=5,
                 *, full_session=None, actor_session=None):
        if actor_session is None:
            with zipfile.ZipFile(archive_path) as archive:
                actor_session = session_for(archive.read(
                    "policies/parkour_moe_trt_assets/actor.onnx"))
        super().__init__(model_path, refresh_every=refresh_every,
                         full_session=full_session, actor_session=actor_session)
