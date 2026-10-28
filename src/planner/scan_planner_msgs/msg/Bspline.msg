int32 order
int64 traj_id
uint64 request_id
uint64 task_id
uint64 task_version
uint64 candidate_id
# First cross-task candidate must connect motion, not merely position.
bool rolling_task_handoff
# Local certifies this exact initial match and align-then-connect motion.
# Controller must not independently choose a different trajectory prefix.
# Moving task candidates require the certificate; stationary HOLD is exempt.
bool handoff_certified
float64 handoff_matched_time
geometry_msgs/Point handoff_position
float64 handoff_body_yaw
float64 handoff_target_yaw
# Execution envelope certified by Local in addition to the installation error.
# Zero retains the legacy 0.02 m bound. Positive values are limited to 0.10 m.
float64 handoff_turn_position_allowance
# Local certifies the residual yaw band for turn AND departure.
# Zero retains 0.05 rad; positive grants must lie in [0.05, 0.12] rad.
float64 handoff_alignment_tolerance
builtin_interfaces/Time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt

# The planner and controller share one execution contract. Zero numeric limits
# select controller defaults. Schema changes require rebuilding all consumers.
uint8 MODE_NORMAL=0
uint8 MODE_CLEARANCE_LIMITED=1
uint8 MODE_RECOVERY_PRIMITIVE=2
# Rest-to-rest straight conversion; fixed body yaw, bounded capture and braking.
uint8 MODE_FORWARD_TURN_STAGE=3
uint8 execution_mode
# Match progress to the measured body independently of speed limits/calibration.
# False retains reference-time tracking for explicitly time-indexed callers.
# Fixed-yaw lateral recovery owns measured capture through its own primitive.
bool use_measured_progress
float64 max_forward_speed
float64 max_lateral_speed
float64 max_yaw_rate
float64 max_tracking_correction
# Explicit planner-authorized forward gait calibration; zero disables it.
float64 min_forward_command
float64 min_forward_effective_speed

# Required for MODE_FORWARD_TURN_STAGE. Local and Controller use the same
# conservative braking contract; these values do not widen the capture region.
float64 forward_stage_stop_latency
float64 forward_stage_deceleration

# A low-speed transit endpoint constrains only its braking approach, not the
# entire otherwise wide trajectory. Zero disables this additional bound.
float64 terminal_speed_limit
float64 terminal_braking_deceleration
float64 terminal_command_latency
