# Observation task migration — checkpoint 1

This branch is **not yet the enabled task-mode navigation pipeline**.
The legacy `initial_path` flow remains active. No launch default has changed.

## Baseline

Baseline commit: `8edf6ed` (`Save calibrated gait and clearance recovery baseline`).
Migration branch: `codex/observation-task-local-planner-20261003`.

## Implemented and tested primitives

`ObservationTask.msg` mirrors the Explorer objective instead of creating a
second completion owner. The full target set stays in Explorer. The wire message
carries bounded spatial representatives, their full count and resolution.
Representative selection version 1 uses spatial strata with deterministic ties;
it does not claim a per-cell predicted information gain.

`task_id` is a positive monotonically increasing Explorer generation.
`task_version` changes when the objective or lineage changes, not when Local
chooses another station. `request_id` identifies a planning attempt, and
`trajectory_id` identifies a Controller handoff. Zero task identity is legacy.

The existing Python objective now supports sensor-only completion. Each map
cell retains its last accepted sensor revision; footprint seeding does not
change it. Binding a task captures the initial revision. Boundary disappearance
alone cannot close a sensor-only task. This strict mode is not enabled for
legacy objectives, so existing run behavior is preserved.

`local_task_planner.h` is a library, not another ROS owner. Its bounded connected
search selects off-polyline approach points within the coarse-route corridor
union observation disk. The caller supplies complete edge safety and station
visibility checks. Its output is **not** an executable trajectory or a safety
certificate: spline, dynamics, braking sweep and Controller ACK remain required.
Terminals and directional portal/lineage constraints still need integration.

Controller echoes task identity in ACK/state messages. `COMMAND_RETIRE_TASK`
establishes a monotonic task/version fence, stops matching old execution, and
rejects subsequently delivered old task candidates. The FSM does not yet emit
this command; pending-ACK reconciliation must be integrated before enabling
task mode. Global Graph constraints now use task/version/object identity when
present, remaining idempotent across multiple path requests for that object.

## Remaining integration, in order

1. Explorer task publication, lifecycle cancellation and task-level metrics;
   launch `task_mode` switch with both legacy and task mode regression tests.
2. FSM task acceptance and deferred replacement, pending ACK tombstones and
   Controller retirement reconciliation. Feed task identity through execution
   snapshots, pending handoffs, status, failure and parser filters.
3. Invoke the LocalTaskPlanner on an immutable local map snapshot, with the
   existing clearance contract applied to every new approach and trajectory.
   Enforce directed portal/lineage constraints and bound distinct station retries.
4. Typed arrival/failure feedback, same-region sibling station selection,
   Global Graph scope overlays and Explorer-owned sensor-only completion.

Do not enable a half-connected task mode. In particular, arrival is not task
completion, publishing a spline is not Controller acceptance, and clearing a
pending handoff is not sufficient cancellation of a potentially installed spline.

## Checkpoint validation

ROS packages `scan_planner_msgs` and `scan_planner` build successfully.
Colcon reports 82 tests, zero failures. Direct Python regression run reports
132 passed (Explorer, Global Graph and the observation-task contract).
No Gazebo motion validation or enabled task-mode end-to-end test has been run.
