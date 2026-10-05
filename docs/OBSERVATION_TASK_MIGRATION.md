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
are sampled at uniform time intervals using quintic arc-length timing with
matching initial/final velocity, rather than imposing constant speed at a
zero-speed observation endpoint.
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

### Completion handoff and endpoint repair (2026-10-04)

Information completion is checked before waiting on G1's in-flight Local
request or repairing its obsolete coarse route. A completed task may validate
and activate a prepared cross-scope successor without waiting for station
arrival. An unfinished task still cannot transfer its commitment. Failed route
validation does not commit the prepared successor or change the old scope.
Controller retirement/ACK fences remain mandatory; earlier G2 publication
does not by itself promise uninterrupted physical motion.

Task replanning starts from filtered measured world velocity. A suffix with
0.30 s or less remaining is not reused. Shape rejection reports length, turn,
guide deviation and progress regression and emits PATH-scoped evidence, not a
Viewpoint constraint. It excludes the failed local candidate and tries up to
two additional siblings per rolling attempt, without randomizing the same
seed. The existing FSM retry budget still bounds family exhaustion; only
exhaustion is escalated to Viewpoint rejection. Terminal HOLD is published
against the valid execution version before invalidating its snapshot.

### Physical progress and unresolved observation (2026-10-04)

Controller checks a sliding physical-displacement window whenever translation
is commanded and the endpoint remains outside tolerance. Defaults are 3 s and
0.05 m (`motion_stall_timeout`, `motion_progress_epsilon`). This works before
the spline endpoint and with zero tracking error. Heading alignment, HOLD,
passive locomotion and time resets restart the window rather than counting
intentional stops. `EXECUTION_NO_PROGRESS` latches zero command until a newer
candidate is accepted. Matching FSM feedback requests local repair; repeated
state messages do not consume another attempt. Three stalled executions per
task/version exhaust that bounded recovery; same-task route requests cannot
reset the budget.

Local visibility checks no longer discard an unresolved representative because
it was theoretically visible from a previously visited station. Explorer's
sensor evidence is authoritative; reached stations remain spatially excluded
and the adjustment budget remains enforced. Arrival feedback checks completion
immediately before beginning the bounded post-arrival scan wait. Genuine
unfinished tasks still wait for evidence; this is not continuous-motion task
retirement.

The isolated ROS runtime test includes a ten-second spatial-gait curve and
stationary odometry on that curve: Controller reports mid-segment STALLED
without reaching its terminal time. It also verifies a new station candidate
for a still-unresolved representative after a previous visit.

In particular, arrival is not task
completion, publishing a spline is not Controller acceptance, and clearing a
pending handoff is not sufficient cancellation of a potentially installed spline.

## Execution failure is not Viewpoint evidence (2026-10-04)

Local candidate-family exhaustion now emits `EXECUTION_BLOCKED` with PATH-scoped
evidence at measured odometry. A rejected departure steering sweep is classified
as `DEPARTURE_UNSAFE`. Neither proves the observation station or global branch
is invalid. Explorer retains the task and does not write a Global Viewpoint
constraint for these failures or for a station-adjustment budget exhaustion.
Candidate exclusions last only for the current reference request; physically
visited stations remain separate and persist for the task.

Blocked execution retries only when measured pose/heading or occupancy within
one metre changes. Distant map revisions and the ordinary goal timeout do not
repeatedly dispatch the same failed request. A genuine unchanged blocked state
may remain stopped; this does not manufacture an executable recovery primitive.

Inside the observation disk, a task may carry zero or one coarse-route anchor.
Outside that disk an approach route is still required. Task mode does not invoke
the unused legacy global polynomial merely to activate a local observation task.
The local search retains incoming direction, penalizes turns, and prefers wider
clearance and proximity to the coarse guide while preserving hard collision and
candidate certification checks. FSM heading ownership now comes from matching
versioned ExecutionState, not independent unversioned heading topics.

The scoped regression passes 143 Python tests and 97 C++ tests. The isolated ROS
runtime test also accepts a one-anchor task inside its observation region. No
Gazebo locomotion run has validated these changes. Mandatory station arrival HOLD,
cross-task retirement fences, and the conservative steering-sweep certification
remain; this change is not a complete MPC or continuous-motion handoff redesign.

## Validation boundary

Task-mode heading alignment now owns the acknowledged local trajectory: ordinary
rolling/predictive replanning and station arrival do not interrupt it. A safety
stop, a replacement reference, or Controller heading-stall feedback bypasses
the alignment gate. Frozen state is taken only from matching execution lineage
and is reset on handoff. The independent physical-safety callback remains active.

Malformed task contracts publish request-scoped `REFERENCE_PATH_REJECTED` with
`reason=INVALID_TASK_CONTRACT` and the rejected field. Explorer ends the pending
request without converting that protocol error into a Viewpoint/region constraint.
The isolated ROS test verifies incoming task/request lineage for this rejection.

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
