import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from explorer_core.grid import ExplorationGrid, FREE
from explorer_core.observation_lifecycle import (
    ObservationTask, evaluate_observation, observation_is_complete,
)
from explorer_core.observation_contract import representative_cells, bind_task_identity


def test_fsm_callsite_does_not_gate_geometry_on_walking_status():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / "src" / "scan_replan_fsm.cpp").read_text()
    begin = source.index("const TrackingDeviationAction tracking_action =")
    call = source[begin:source.index("if (tracking_action ==", begin)]
    assert "= classifyTrackingDeviation(" in call
    assert "execution_progress_eligible ? execution_age : 0.0" in call
    assert ": TrackingDeviationAction::KEEP_EXECUTING" not in call


def test_path_exhaustion_is_wired_to_path_feedback_not_execution_blocked():
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root / "src" / "observation_task_adapter.cpp").read_text()
    handler = source[source.index("void SCANReplanFSM::observationTaskRejected"):]
    assert 'reason == "LOCAL_PATH_FAMILY_EXHAUSTED"' in handler
    assert "path_failed ? scan_planner_msgs::msg::ObservationFeedback::PATH_REJECTED" in handler


def test_disappearing_boundary_without_sensor_evidence_does_not_complete():
    grid = ExplorationGrid(8, 8, 1, 0, 0)
    grid.data.fill(FREE)  # Initialization is not a lidar observation.
    task = ObservationTask(1, (2, 2), {(4, 4)},
                           require_sensor_evidence=True)
    for revision in range(5):
        evaluate_observation(task, grid, None, revision, .8, 2, 2, 1)
    assert not observation_is_complete(task, 2, 2)
    assert task.completion_reason == "none"
    grid._apply_evidence({(4, 4)}, set())
    evaluate_observation(task, grid, None, 5, .8, 2, 2, 1)
    assert observation_is_complete(task, 2, 2)


def test_snapshot_copies_observation_provenance():
    grid = ExplorationGrid(8, 8, 1, 0, 0)
    frozen = grid.snapshot()
    grid._apply_evidence({(4, 4)}, set())
    assert grid.sensor_observed[4, 4]
    assert not frozen.sensor_observed[4, 4]


def test_satisfied_information_ledger_cannot_be_reopened_by_later_maps():
    grid = ExplorationGrid(8, 8, 1, 0, 0)
    task = ObservationTask(1, (2, 2), {(4, 4)}, information_satisfied=True,
        progress=1., completion_streak=2, completion_reason="visible_information_resolved",
        satisfied_map_revision=3, satisfied_sensor_revision=4)
    evaluate_observation(task, grid, None, 9, .8, 2, 2, 1)
    assert task.progress==1. and task.completion_streak==2
    assert task.completion_reason=="visible_information_resolved"
    assert (task.satisfied_map_revision,task.satisfied_sensor_revision)==(3,4)


def test_representatives_are_bounded_deterministic_and_not_denominator():
    cells = {(x, y) for x in range(100) for y in range(4)}
    representatives = representative_cells(cells, 16)
    assert len(representatives) == 16
    assert set(representatives) <= cells
    assert representatives == representative_cells(reversed(sorted(cells)), 16)
    assert max(x for x, _ in representatives) > 90
    assert min(x for x, _ in representatives) < 10
    assert len(cells) == 400


def test_local_goal_change_does_not_reset_identity_or_station_count():
    grid = ExplorationGrid(8, 8, 1, 0, 0)
    task = ObservationTask(1, (2, 2), {(4, 4)})
    bind_task_identity(task, 100, grid)
    task.station_adjustments = 2
    task.goal = (2.5, 2)
    bind_task_identity(task, 101, grid)
    assert (task.task_id, task.task_version, task.station_adjustments) == (100, 1, 2)


def test_historical_sensor_evidence_cannot_close_new_task():
    grid = ExplorationGrid(8, 8, 1, 0, 0)
    grid.data.fill(FREE)
    grid._apply_evidence({(4, 4)}, set())
    task = bind_task_identity(ObservationTask(1, (2, 2), {(4, 4)}), 100, grid)
    for revision in range(5):
        evaluate_observation(task, grid, None, revision, .8, 2, 2, 1)
    assert not observation_is_complete(task, 2, 2)
    grid._apply_evidence({(4, 4)}, set())
    evaluate_observation(task, grid, None, 6, .8, 2, 2, 1)
    assert observation_is_complete(task, 2, 2)
