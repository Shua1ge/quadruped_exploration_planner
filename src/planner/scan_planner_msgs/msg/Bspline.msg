int32 order
int64 traj_id
uint64 request_id
builtin_interfaces/Time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt

# The planner and controller share one execution contract. Zero limits keep
# old bags/messages backward compatible and select the controller defaults.
uint8 MODE_NORMAL=0
uint8 MODE_CLEARANCE_LIMITED=1
uint8 MODE_RECOVERY_PRIMITIVE=2
uint8 execution_mode
float64 max_forward_speed
float64 max_lateral_speed
float64 max_yaw_rate
float64 max_tracking_correction
