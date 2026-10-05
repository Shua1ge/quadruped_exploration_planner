import importlib.util
import json
import pathlib
import sys
from types import SimpleNamespace


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
SPEC = importlib.util.spec_from_file_location(
    "global_representation_node",
    ROOT / "scripts" / "global_representation_node.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_node():
    node = object.__new__(MODULE.GlobalRepresentationNode)
    node.last_map_revision = 17
    node.route_constraint_revision = 0
    node.viewpoint_constraint_radius = 0.6
    node.max_viewpoint_constraints = 4
    node.viewpoint_constraints = {}
    node.failure_evidence_received = 0
    node.failure_evidence_forwarded = 0
    node.global_safe_regions = {
        9: {"id": 9, "x": 4.0, "y": 2.0, "area": 3.0}}
    node.global_safe_portals = {}
    node.safe_region_pub = Publisher()
    node.failure_evidence_pub = Publisher()
    node.get_logger = lambda: SimpleNamespace(
        warning=lambda *args, **kwargs: None)
    return node


def evidence(request_id=42, scope=None, x=4.1, y=2.0):
    return SimpleNamespace(
        request_id=request_id,
        task_id=0, task_version=0, failed_object_id=0,
        trajectory_id=8,
        failure_scope=(MODULE.FailureEvidence.SCOPE_VIEWPOINT
                       if scope is None else scope),
        position=SimpleNamespace(x=x, y=y, z=0.4),
        required_clearance=0.1,
        stage="STRUCTURED_LOCAL_REPAIR",
        reason="FINITE_CANDIDATE_FAMILY_EXHAUSTED")


def test_same_task_viewpoint_is_idempotent_across_path_requests():
    node = make_node()
    first = evidence(42)
    first.task_id, first.task_version, first.failed_object_id = 100, 1, 7
    node.failure_evidence_callback(first)
    second = evidence(43)
    second.task_id, second.task_version, second.failed_object_id = 100, 1, 7
    node.failure_evidence_callback(second)
    assert node.route_constraint_revision == 1
    assert len(node.viewpoint_constraints) == 1


def test_viewpoint_failure_becomes_graph_constraint_before_forwarding():
    node = make_node()
    message = evidence()

    node.failure_evidence_callback(message)

    assert node.route_constraint_revision == 1
    assert node.viewpoint_constraints[42]["region_id"] == 9
    assert node.failure_evidence_pub.messages == [message]
    snapshot = json.loads(node.safe_region_pub.messages[-1].data)
    assert snapshot["route_constraint_revision"] == 1
    assert snapshot["viewpoint_constraints"][0]["request_id"] == 42


def test_duplicate_viewpoint_failure_is_idempotent_but_forwarded():
    node = make_node()
    message = evidence()

    node.failure_evidence_callback(message)
    node.failure_evidence_callback(message)

    assert node.route_constraint_revision == 1
    assert len(node.safe_region_pub.messages) == 1
    assert node.failure_evidence_pub.messages == [message, message]


def test_path_scoped_failure_is_forwarded_without_graph_constraint():
    node = make_node()
    message = evidence(scope=MODULE.FailureEvidence.SCOPE_PATH)
    message.task_id, message.task_version = 100, 2
    message.has_path_segment = True
    message.path_start = SimpleNamespace(x=1.0, y=2.0, z=0.0)
    message.path_end = SimpleNamespace(x=2.0, y=2.0, z=0.0)

    node.failure_evidence_callback(message)

    assert node.route_constraint_revision == 0
    assert node.viewpoint_constraints == {}
    assert node.safe_region_pub.messages == []
    assert node.failure_evidence_pub.messages == [message]


def test_path_scoped_failure_without_identity_or_segment_is_rejected():
    node = make_node()
    message = evidence(scope=MODULE.FailureEvidence.SCOPE_PATH)

    node.failure_evidence_callback(message)

    assert node.failure_evidence_pub.messages == []
    assert node.failure_evidence_forwarded == 0
