# Observation task migration — integrated opt-in pipeline

The task pipeline is connected. `task_mode:=false` retains the legacy
`initial_path` flow. No launch default has changed. Enable the new pipeline:

```bash
SCAN_ROS_DOMAIN_ID=43 ~/ros2_ws/scan_planner_ws/launch_exploration.sh \
  headless:=false locomotion_mode:=rl_effort auto_start:=true \
  task_mode:=true metrics_file:=/tmp/observation_task_baseline.csv
```

Stop the previous launch completely before starting another simulation.
Task mode requires reference-path navigation and the closed-loop Controller.

## Baseline

Baseline commit: `8edf6ed` (`Save calibrated gait and clearance recovery baseline`).
Migration branch: `codex/observation-task-local-planner-20261003`.

## Ownership and connected implementation

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
alone cannot close a sensor-only task. This strict mode is enabled for task
objectives only; legacy objectives retain their previous completion semantics.

`local_task_planner.h` is a library, not another ROS owner. Its bounded connected
search selects off-polyline approach points within the coarse-route corridor
union observation disk. The caller supplies complete edge safety and station
visibility checks. Its output is **not** an executable trajectory or a safety
certificate: spline, dynamics, braking sweep and Controller ACK remain required.
The inherited directional commitment gate constrains the complete task domain.
Terminal cells are hints, not a requirement to reach the old exact goal.
Each approach edge passes footprint/clearance checks; geometric BFS vertices
are uniformly resampled with an explicit time interval before spline fitting.
The entire candidate must remain inside the task domain and pass the existing
clearance, dynamics and predictive checks. Arbitrary fixed-yaw lateral escape
is disabled in task mode. FSM integration lives in
`observation_task_adapter.cpp`, without a new independent planning node.

Controller echoes task identity in ACK/state messages. `COMMAND_RETIRE_TASK`
establishes a monotonic task/version fence, stops matching old execution, and
rejects subsequently delivered old task candidates. The FSM now emits this
command and waits for explicit `STATUS_TASK_RETIRED` confirmation before
activating the successor task. Delayed ACK cannot revive old execution; the
ambiguous pending trajectory ID is reserved before clearing its handoff.
Global Graph constraints use task/version/object identity when
present, remaining idempotent across multiple path requests for that object.

## Explorer feedback and completion

Explorer's `observation_task_bridge.py` publishes objectives and cancellation.
The FSM queues valid tasks until a fresh local map exists, then calls the Local
library on the immutable planning snapshot. Task identity flows through
execution snapshots, pending handoffs, status, failure evidence and filters.

At an acknowledged station, Local reports measured position, timestamp and
candidate/trajectory IDs through `planning/observation_feedback`. Arrival is
not completion. Explorer awaits accepted observation updates and, if incomplete,
requests another station under the same task. Local retains reached/rejected
stations and the finite adjustment budget (default 4) across requests.
Exhaustion emits `TASK_EXHAUSTED`; candidate failure emits `VIEWPOINT_REJECTED`
plus `FailureEvidence`, rather than indefinite generic `BLOCKED` retries.
Global Graph receives task-scoped evidence; Explorer advances route constraints
and uses same-committed-region sibling arbitration before releasing the region.

Completion or objective replacement cancels the old task. Metrics include task
identity, creation, requests, station arrival, rejection and stale feedback;
`GLOBAL_PLAN_BASELINE` timing output includes task identity.

In particular, arrival is not task
completion, publishing a spline is not Controller acceptance, and clearing a
pending handoff is not sufficient cancellation of a potentially installed spline.

## Validation boundary

ROS packages `scan_planner_msgs` and `scan_planner` build successfully.
Final direct Python regression: 180 passed. Package-scoped Colcon result:
85 tests, 0 errors, 0 failures, 0 skipped.
The regression suite covers Explorer, Global Graph and the task contract.
C++ tests cover connected off-polyline selection, directional gates and rejected
station exclusion. `test_observation_task_runtime.py` starts real FSM/Controller
processes in an isolated ROS domain with synthetic odometry/clouds. It verifies
actual candidate generation/acceptance, delayed retirement ACK, stale ACK,
retired candidates, measured station arrival and station-budget exhaustion.
It does not use Gazebo or command the user's robot.

Gazebo locomotion, mapping efficiency and RL tracking quality are **not yet
validated for this new mode**. Protocol tests do not prove all physical stalls
are eliminated. Use the explicit flag above for the next simulation comparison.
