# Observation task migration — integrated opt-in pipeline

## Bounded turn execution and stopped recertification (2026-10-14)

`handoff_turn_position_allowance` now names Local's certified turn execution
envelope independently from the 0.02 m installation-state allowance. Local
selects the largest 0.02–0.10 m envelope fully supported by the footprint,
connector and task domain; Controller accepts only 0.02–0.10 m, retains 0.02 m for zero/legacy grants and
fixed-yaw actions, and never widens an ungranted turn. Ordinary moving handoffs
retain their original bound. A turn failure uses the existing replacement
budget, confirms HOLD and fresh linear/angular rest, and retains its endpoint
before recertification. No spline siblings are generated while waiting.
`HANDOFF_REST_STATE_CHANGED` and `HANDOFF_REST_UNCONFIRMED` now reach that same
bounded decision instead of leaving Controller latched with Local waiting.
Rebuild the message package and all consumers together before restarting.


## Mapping, terminal continuation and safe turn conversion (2026-10-13)

Gazebo launches now deliver the original LaserScan to Explorer and Local while
retaining the point cloud for visualization. Finite returns terminate rays;
valid no returns clear only a bounded 3 m prefix. Invalid ranges do not clear,
and a beam rejected by the 2D height band cannot clear behind its measured
endpoint. Ray-derived free space is obstacle evidence, not ground support.
The legacy XYZ input remains the default outside the Gazebo laser launch.

Local compares preferred 0.20 m clearance arrivals and approach shapes before
falling back to the 0.10 m hard belt. The endpoint families and optimizer
alternatives share one 1024-pop geometry budget. A rolling segment may retain
certified exit speed when the full braking distance fits. A stopped observation
station requires a return-turn and braking certificate across the arrival
capture envelope, checked again at measured rest. When a turn cannot start at
the old endpoint, the existing bounded forward stage moves to an authenticated
turn position before reconnecting to the same task.

Controller reuses only continuous, fresh odometry proof of rest. A certified
fixed-yaw departure waits for both linear and angular rest when that proof is
absent. Initial alignment survives candidate replacement and releases
translation after a measured settled heading; ordinary rolling translation
continues. Recovery memory is rechecked at each same-task recovery entrance
against measured movement or a newly certified exit.

These changes have synthetic ROS and unit coverage. Live Gazebo LaserScan
bridging, terrain slip, ONNX gait response and the full physical transition
remain field-validation work; the protocol tests do not assert those outcomes.

## Contract-stack V1 implementation status (2026-10-10)

The existing opt-in pipeline is not the completed contract-stack blueprint.
This first implementation batch replaces compute-tick observation confirmation,
ambiguous task-mode acceptance, and implicit cancellation waiting. Explorer now
counts fresh task-relevant sensor revisions, rejects repeated scan timestamps,
and requires independently observed coverage of the tracked boundary for each
`frontier_closed` confirmation. Seeded FREE and unrelated map updates do not
confirm information; an invalidated boundary proof is withdrawn even on an idle
compute revision. Legacy non-task observation semantics are unchanged.

Controller emits `STATUS_ACTIVATED` only after installing a moving version;
`STATUS_HOLD_INSTALLED` names a stop/held installation. `STATUS_ACCEPTED` remains
legacy-only and cannot authorize a task-mode FSM transition or G2 commit. Task
motion without measured odometry is rejected before installation. HOLD ACKs do
not release safety latches or emit `EXECUTION_READY`.

Cancellation resends do not renew the fixed deadline. Explorer's
`task_cancel_timeout` and FSM's `fsm.task_cancel_timeout` default to 10 seconds.
Missing confirmation produces `CANCEL_UNCONFIRMED`, never a fabricated
`CANCEL_ACK`. Explorer terminates the pending wait but retains unresolved remote
ownership, blocking new motion authorization. FSM remains fenced and resends
retirement for recovery. A later exact cancellation ACK reconciles that state;
repeating an already-confirmed cancellation returns the same terminal fact.
Local authorization fences and Controller-confirmed retirement are now separate
watermarks. Sending a retire command never grants `CANCEL_ACK`. An unconfirmed
repeat resends the fence without renewing the cancellation deadline. Retirement
ACKs are drained as individual events, so a late predecessor ACK cannot erase a
successor confirmation or prematurely finish its pending cancellation.
The task-scoped no-progress budget also latches exhaustion: same-task request
refreshes and ACKs cannot reopen it, and repeated exhaustion is idempotent.

The remaining V1 work is substantive: explicit session/Attempt identity and
certification lifetime, one full motion arbiter replacing all old writers,
scope-specific cancellation preserving a certified predecessor, and the
pending/confirmed/revoked shared spatial-evidence lifecycle. In particular,
the current monotonic retirement upper bound still retires a rolling predecessor
when its pending successor is cancelled. It is a known conservative legacy
behavior, not the scoped-cancellation design. Do not call V1 complete until these
responsibilities are retired and the full acceptance scenarios pass.

Both message packages and planner consumers must be rebuilt and sourced
together after this change. This batch does not change launch defaults, policy
assets, gait calibration, map resolution, or clearance margins. Verification
uses an isolated build/install tree and synthetic odometry, not a live Gazebo
performance or physical-safety claim. The sections below record earlier pipeline
milestones and do not supersede this implementation status.

### Cancellation-confirmation follow-up (2026-10-11)

The predecessor/successor cancellation interleaving now has a real-process
regression: hold G2's candidate and retirement commands, cancel G2 then G1,
and verify there is no fabricated G1 `CANCEL_ACK`. A real G1 retirement ACK
confirms G1 only; it does not close G2's pending cancellation. A later G1 ACK
also cannot overwrite G2's confirmation. Confirmed repeats remain idempotent.
The existing conservative upper-bound cancellation scope is unchanged.

The normal ROS workspace (not the earlier staged source copy) was rebuilt:
`scan_planner_msgs` and `scan_planner`, both successful. Targeted regressions
passed: 202 Explorer/observation tests, 112 C++ utility/local-task tests,
5 real-process synthetic-odometry scenarios, and 24 policy/probe tests.
The policy tests include 100 steps against the supplied original estimator and
actor assets. Normal exploration still defaults to packaged
`parkour_moe_stateful`; legacy rollback is retained. No Gazebo motion run was
started for this follow-up, and these results are not a terrain-safety claim.

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

### Rolling sensor-completion handoff (2026-10-06)

Sensor-completed G1 -> G2 carries `predecessor_task_id/version`; Explorer does
not publish an early SUPERSEDED for this transition. Local prepares G2 while
retaining G1's execution snapshot and measured progress for safety. The first
G2 spline sets `rolling_task_handoff`: Controller checks position, heading
(within its heading-release threshold) and a bounded velocity change before
installation. A discontinuity is rejected without replacing execution; SCAN
requests `TASK_TRANSITION_REQUIRES_STOP`, then retries a stop-first connection.
It does not turn this discontinuity into a rejected station or obstacle edge.

Controller fences G1 atomically when installing G2; SCAN retires G1 only after
the matching G2 ACK. Explicit cancellations still stop execution, including a
queued or in-flight successor and its predecessor. Station arrival without
confirmed observation completion still enters the existing observation wait;
this change does not remove safety HOLD or guarantee every task can be joined
without stopping. Message definitions changed: rebuild both `scan_planner_msgs`
and `scan_planner`, then restart the complete launch (do not mix old/new nodes).

### Information satisfaction versus task commit (2026-10-06)

In task mode, sensor completion now latches G1 as `SATISFIED`, preserving its
information evidence, active task identity and region commitment. Explorer
publishes G2 as a pending handoff, not as its active objective. SCAN generates
the candidate using the existing safety and rolling-continuity checks.
After the exact Controller candidate ACK, SCAN publishes typed
`ObservationFeedback.EXECUTION_READY` with task/version, request, candidate and
trajectory identities. Only this feedback commits G2 and its region, records
G1 completion and clears the pending handoff. A plain `PATH_TRAJECTORY_READY`
status string is not a commit event; duplicate or stale feedback cannot commit
twice or reopen G1's information objective.

Pending failure or timeout enters cancellation. Explorer retains G1's satisfied
ledger and region until G2 cancellation is confirmed, then selects a replacement
without claiming a retired predecessor execution. This cancellation also fences
the old execution; it does not promise uninterrupted motion after failure.
An invalid/rejected successor still obtains a Controller retirement ACK and a
correctly identified `CANCEL_ACK`, even if it never became Local's active task.
Handoff timeout/cancellation runs before health and map-revision gates. Physical
safety HOLD remains independent and unchanged. Legacy non-task mode is retained.

Validation: both ROS packages build; 298 direct Python regression tests pass,
83 FSM utility and 17 LocalTaskPlanner C++ tests pass, and 2 isolated real ROS
process tests pass (synthetic odometry/clouds, no Gazebo actuation). The latter
cover withheld candidate ACK, typed execution confirmation, successor cancellation,
late ACK fencing and cancellation of a successor rejected before activation.
Physical RL tracking and live G1-to-G2 smoothness still require a simulation run.

### Local geometric autonomy and ordered connection contract (2026-10-06)

With `task_mode:=true`, `task_geometry_autonomy:=true` is now the default.
Explorer owns the information objective, remaining sensor-evidenced targets,
scope and next-task arbitration. The coarse polyline defines an allowed domain
and an approach hint, not the executing path. Local owns connected seed search,
station selection and finite motion replacement. Controller ACK and hard
footprint/dynamics/braking-sweep validation remain mandatory.

The autonomous path retires Global's per-map polyline repair/rejoin, Local's
global-polyline monotonicity guards, world-space commitment half-plane and
visited-station visibility-as-coverage proxy. Shape checks still compare the
candidate against its **locally generated** seed. Temporary failed-candidate
exclusions and a station-attempt budget are motion state, not observed coverage.
After station arrival, Explorer sends its actual unresolved representatives
without synthesizing another Local route. Only Explorer's sensor ledger can
declare information satisfaction.

`LOCAL_MOTION_EXHAUSTED` terminates one bounded Local motion attempt. Explorer
cancels/fences that task, then selects another task in the same scope first.
If no sibling exists, commitment may be released without marking the region
blocked. No Local candidate failure is promoted to a Global blocked edge or
viewpoint constraint. Operational pose cooldown depends on the actual local
departure context, not arbitrary distant map updates, and does not advance
`route_constraint_revision`. Explicit viewpoint/portal evidence remains a
separate mechanism; motion failure alone cannot establish it.

`PlanarClearanceField` is a separable O(N) planar distance transform cached by
immutable inflated-map snapshot and vertical slice (at most three slices).
The connected Local search, cost-preserving seed simplification and both
B-spline rebound/refinement use a continuous preferred-clearance cost. Its
bounded tail remains active beyond the preferred belt, so this is not merely
changing the 0.10 m repulsion threshold. It is a path-quality preference only;
hard footprint/unknown-space/sweep checks are not replaced by interpolated EDT.

Sparse routes now carry the directed edges actually selected by graph search.
When validated sparse geometry is available, the task freezes `RoutePortal[]`
and `route_graph_revision`; Local advances only the next checkpoint on a
continuous, forward odometry crossing inside its aperture. Reverse crossings,
later-portal crossings and >0.5 m discontinuities cannot advance it. The cursor
is execution topology state, NOT task observation progress. Contract changes
require a new task version. Dense fallback, spliced routes, direct same-edge
attachments without a complete transition sequence, stale graph estimates and
already-passed first checkpoints explicitly publish an empty portal contract.

This is **not yet a complete persistent branch-sequence representation**.
The current skeleton IDs do not have split/merge lineage migration. Checkpoint
IDs are frozen within a task's graph revision; they are not silently remapped
to a newer graph. The allowed domain still uses a coarse-route corridor union
observation disk, not a branch-neighborhood mask. Completing E1 and replacing
that fallback domain are prerequisites for removing geometric domain hints
entirely. No claim of halved protocol traffic or smoother RL motion is made
without a live A/B run. Legacy behavior is available with
`task_geometry_autonomy:=false`; non-task mode is unchanged.

Validation for this revision: `colcon build --packages-up-to scan_planner`
succeeds (12 packages), followed by a successful final incremental planner
build. Direct Python regression selection: 303 passed; LocalTaskPlanner C++:
22 passed; FSM utility C++: 83 passed; isolated real ROS process scenarios:
3 passed. Scenarios cover autonomous candidate ACK gating, frozen topology
contract rejection, delayed ACK retirement fences, directed/ordered crossings,
close checkpoints, brute-force distance-field equivalence, clearance-preserving
simplification and nonspatial motion-terminal deduplication. The direct Python
selection excludes ROS launch tests and the pre-existing tunnel-third-slice
suite; it is not a claim that every repository test or Gazebo/RL scenario ran.
No live simulation was stopped/restarted and no Git commit/push was performed.

### 2026-10-06 — End station motion instead of replaying the observation route

In autonomous task mode a measured, acknowledged station arrival ends the motion
attempt even while heading alignment is active. The task specifies a position,
not a final yaw. Hard footprint/braking checks and successor Controller ACKs are
unchanged. Local retains the exact selected subgoal during rolling replanning;
a changed search lattice is not permission to drift the goal. If the fixed-goal
approach has no valid candidate, bounded sibling search still validates safety.

Explorer collects `observation_done_updates` accepted post-arrival scans (default
3), with the existing 2-second capture bound. Information completion can hand off
immediately. An incomplete capture instead sets `motion_released`, prepares an
ACK-gated successor, and never republishes the old task for another station.
It does not set `information_satisfied`, increment `observations_satisfied`, or
add the partial goal to `completed_goals`. The full target set/observation progress
remains unchanged while waiting; unresolved map/frontier information remains
eligible for future tasks after leaving the attempted departure area.

Motion-attempt exclusions survive heading-bin changes, ordinary map revisions,
and successor cancellation. A new departure must be at least
`max(1m, 2*task_corridor_radius)` away. Only `DEPARTURE_UNSAFE` may reopen locally
when a previously occupied departure cell is actually confirmed FREE, not
UNKNOWN. These are operational exclusions, not graph connectivity evidence.
Successor selection includes a soft initial-route-heading access cost; it uses
the route departure rather than the straight vector to the remote goal and is
not an execution certificate. Controller validation remains authoritative.

Empty unresolved representative sets stay empty during completion confirmation;
they no longer repopulate with already-observed targets. Prepared commitment
transfer logging explicitly says PREPARED, rather than implying an ACKed commit.

Validation: planner build/install succeeds; direct Python regression selection
(Explorer, bridge, contract and sparse routing) passes 202 tests. LocalTaskPlanner
passes 25 C++ tests, FSM utilities 83, and isolated ROS process scenarios 3.
The station-capture scheduling matrix covers 0/1/2/3 post-arrival scans and times
on either side of the 2-second bound. Other regressions cover incomplete-task
ACK commit without fabricated completion, cancelled successor/late ACK fencing,
heading/map jitter exclusions, departure-obstacle removal and fixed-goal safety.
These are protocol/synthetic-motion checks, not a live Gazebo/RL smoothness claim.

### 2026-10-06 — Failure classification and field diagnostics only

This change does not alter arrival, recovery, handoff ownership, safety margins,
candidate acceptance or persistent Global Graph failure evidence.

Local emits `[LOCAL_VALIDATION_FAILURE]` independently of the suppressed
autonomous candidate-level `FailureEvidence`. `stage` identifies the actual
validator (final clearance, task domain, forward-gait turn/steering/braking,
handoff rotation/connector, or optimizer). `location` identifies the rejected
part: CURRENT_FOOTPRINT_INVALID, DEPARTURE_CONNECTION_INVALID, PATH_INVALID,
TERMINAL_INVALID, or UNASSESSED. These labels describe checks, not proof that a
viewpoint or whole corridor is impossible.

The diagnostic departure window is the first 0.5 m of sampled planar arc length;
it is not an additional safety threshold. The terminal window is the last
existing validation sampling interval (not a newly added endpoint check).
CURRENT_FOOTPRINT_INVALID requires a separately measured-pose footprint failure
and a rejected departure sample. A later path/terminal failure remains labeled
by its sample location even if the measured departure pose was also invalid.
Current footprint flags remain independently visible. `current_margin_blocked`
and `current_physical_blocked` are -1 if the measured pose is unavailable.
An optimizer failure without a validated candidate is UNASSESSED. Handoff
geometry has no spline sample time and reports sample_t=-1. `hit_valid=0` means
the printed point is not certified obstacle evidence (e.g. a task-domain reject).
Existing clearance reason numbers and all existing safety decisions are retained.

Explorer retains the boolean `double_cylinder_path_free` API and exposes the
same predicate through `PathValidationResult`. `[GLOBAL_PATH_VALIDATION_FAILURE]`
reports the first failed sample index, cell/world position, failed disc cell,
path distance, tangent yaw, measured pose/yaw, task identity and snapshot content
revision. It distinguishes OCCUPIED_CELL, UNKNOWN_CELL, OUTSIDE_MAP and
INFLATION_OR_MARGIN. Current-footprint status is unknown without both measured
position and finite body yaw; a repaired path start is not substituted for the
robot position. The global predicate has no timing model, so it does not invent
a trajectory time or call an inflated-cell failure a certified physical collision.
The measured position/yaw/stamp are taken together from one normalized odometry
sample, never from a planning-bound old position paired with a newer live yaw.

Global detail logs are throttled to one per 2 seconds. CSV adds
`last_global_path_validation_failure` (JSON) and cumulative
`global_path_validation_<location>` counts. Counts measure rejected validation
attempts, not distinct obstacles, goals or failed requests. Diagnostics do not
change cooldowns, exclusions, motion budgets, task completion or route constraints.

### Global path correction after a rejected footprint (2026-10-06)

A dense or materialized sparse route rejection now triggers bounded
footprint-aware A* for the same terminal, with edge endpoint tangents using
the existing final predicate. Sibling terminals are still considered before a
shorter observation approach. A shorter approach must pass the unchanged
footprint predicate, goal exclusions, minimum travel distance, and a visibility
hypothesis for remaining UNKNOWN target cells within observation radius.
It preserves the frontier and information target set; predicted visibility
never increments observed coverage. Local remains responsible for spline,
rotation, braking, terrain and Controller handoff certification.

`global_repair_search_limit` defaults to 4 searches per candidate pass;
`global_repair_max_expansions` defaults to 400 per search;
`global_approach_min_distance` defaults to 1 m. Correction results are memoized
for the map content revision, route constraint revision, robot start cell,
footprint/inflation settings and blocked edges. Failed geometry does not add a
viewpoint cooldown or mark an autonomous task region densely invalid.
Point-only polyline shortcuts are re-rasterized and footprint checked; otherwise
the validated original path is retained.

`GLOBAL_PATH_CORRECTION` distinguishes FOOTPRINT_AWARE (attempt_id, cached,
failed cell/yaw, geometry_changed), SAFE_OBSERVATION_APPROACH (old/new terminal),
and PRESERVE_FOOTPRINT_ROUTE. A finite search failure is logged as
NO_PATH_WITHIN_BUDGET, not as proof of corridor blockage. Search-budget deferral
does not memoize an untried goal as failed.

Released-station handoff now checks the existing goal_timeout before its early
return. HANDOFF_WAIT_TIMEOUT cancels the held task and clears its execution
identity for re-arbitration, without blacklisting its goal, releasing its
region or claiming information completion. Frontier demand is rediscovered
from real occupancy; unchanged geometry may reuse correction results.


## 2026-10-14 execution completion and rolling update

MODE_NORMAL terminal capture now relinquishes its old motion output just as spatial/recovery capture does: Controller latches zero command, reports TERMINAL_SETTLING, and reports FINISHED after at least 0.30 s and three fresh measured linear/angular rest samples. TERMINAL_CAPTURE_DRIFT and TERMINAL_STOP_UNCONFIRMED remain zero-command outcomes and enter bounded Local arbitration. A certified successor may still install before capture or replace this stopped version; ordinary rolling does not acquire a mandatory stop at every segment boundary. Local preserves an exact observation station's settling execution until rest or a terminal failure.

Moving transit segments prepare successors with the 1.50 s fsm.rolling_replan_prepare_time window; station/zero-speed conversion contracts retain their rest gate. The candidate ACK remains the commit boundary.

Bspline.handoff_alignment_tolerance carries Local's certified residual yaw band, 0.05--0.12 rad (zero retains 0.05). Local checks the full turn, match connector and first 0.50 m departure prefix with the corresponding footprint buffer, in addition to the hard 0.10 m margin; execution snapshot and real-time remaining-turn checks retain that same grant. Controller cannot widen the band or waive measured rest.

A stopped pending side-step with pose_occupied_latched terminates as RECOVERY_START_FOOTPRINT_OCCUPIED. Explorer treats it as departure-state failure and waits for actual state/obstacle change, rather than replenishing recovery with a remote Goal. This does not authorize movement through physical collision.

Global retains fine anchors on accepted measured route turns >=25 degrees or material body-center height changes (>=0.04 m and >=15% of travel distance). It preserves occupancy and existing query coverage rules. These are travel-complexity hints, not ground-height truth or terrain-friction estimates.

Detailed message/state/timing walkthrough: [Explorer-to-Local execution chain](/home/t1an/文档/Codex/2026-10-14/new-chat/EXPLORER_LOCAL_EXECUTION_CHAIN_2026-10-14_ZH.md).


## 2026-10-14 forward-stage overrun fix

MODE_FORWARD_TURN_STAGE=3 separates the rest-to-rest fixed-yaw conversion from generic clearance-limited gait. Bspline carries forward_stage_stop_latency and forward_stage_deceleration from Local. Controller begins zero command before the endpoint using measured speed, reaction and braking distance; there is no minimum-forward override during braking.

Local certifies the incoming lateral/yaw envelope and an entire 0.15 m stopped capture region plus the future turn/braking. Controller rejects >0.08 m cross-track, >0.10 m overrun, >0.10 rad yaw drift, >0.05 m regression, stale odometry and bounded lack of progress. After at least 0.30 s and three fresh rest samples it finishes only inside the certified capture region. Stop-short or 5 s unconfirmed rest is explicit failure, never fake arrival or a reverse/forward capture loop. Actual stopped departure is checked again before Local releases the conversion stage.

Local consumes FORWARD_STAGE_* failures and tracking-repair requests even while the conversion transaction owns execution. It leaves stage waiting, holds, confirms measured rest and uses the existing two execution replacements from that actual pose, preserving the information task. Normal braking/capture owns zero command without generic nearest-curve replanning; independent actual braking, pose and temporal safety remain active.

Regression fixtures distinguish command 0.30 from measured effective velocity 0.165 and provide a chamber that fits the complete capture envelope. The captured start is tested in real isolated Planner/Controller processes; no field actuator is controlled. Existing messages/nodes must be restarted together after this interface extension.


### 2026-10-14: ineffective initial yaw and failure delivery

Within a certified initial alignment only, Controller ramps an additive yaw bias at 0.25 rad/s² (maximum 0.35 rad/s) after 0.30 s without sufficient progress when measured angular speed is below 0.08 rad/s. The candidate yaw limit still caps the total command. Target crossing resets the bias; entering the certified residual band clears it before zero-command measured settling. Installation, pose drift, remaining rotation checks and the three-second stall window are unchanged. This addresses small-command loss of yaw response without widening the certified residual band.

In task mode, an ineffective alignment now terminates through ObservationFeedback with reason HEADING_ALIGNMENT_INEFFECTIVE (LOCAL_MOTION_EXHAUSTED for geometry autonomy). It no longer spends two attempts rebuilding the same heading spline. Explorer arbitrates the next objective without claiming corridor obstruction or completed information. Legacy matched BLOCKED status also passes a reason explicitly to the same handler; absent typed feedback must not cause a None.reason exception.


### 2026-10-15: departure failure scope and causal feedback

ObservationFeedback now carries failure_scope (FAILURE_ACTION / FAILURE_CURRENT_POSE), trigger_reason, recovery_reason and planning_reason. A failed side-step/rejoin is action-scoped unless the current physical footprint is occupied. Local keeps the first trigger, actual recovery execution failure (including generic EXECUTION_NO_PROGRESS), and final planning cause when mapping a failed rejoin to LATERAL_RECOVERY_REJOIN_FAILED. CANCEL/ACK and identity matching remain required.

Explorer retains pose-wide waiting only for explicit current-pose occupancy. For exhausted recovery actions, it records the initial remaining route direction at the measured failure state. Nearby Goals with the same entry cannot replenish that action. Another candidate must differ by at least 45 degrees from every failed route entry; dense materialization and the final simplified/published route are checked too. This is a bounded arbitration key, not a kinematic motion certificate. Every candidate still needs Local certification and Controller ACK; Local's physical side-step budget survives new task identities. No timer refills this memory. If no eligible direction remains, status is LOCAL_DEPARTURE_ACTIONS_EXHAUSTED, not a claim of completed exploration or impassable global space.

Lateral recovery STALLED terminal_error is distance to the curve endpoint on every update, including a latched failure. It must not be replaced by displacement from the installation anchor.
