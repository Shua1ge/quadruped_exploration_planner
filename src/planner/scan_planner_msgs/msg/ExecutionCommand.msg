uint8 COMMAND_HOLD=0
uint8 COMMAND_RESUME=1
uint8 COMMAND_RETIRE_TASK=2
# Refresh an executing initial turn only after Local rechecks the complete
# remaining rotation, reconnection and physical braking sweep. Never resumes HOLD.
uint8 COMMAND_REFRESH_TURN=3

uint64 request_id
uint64 task_id
uint64 task_version
int64 trajectory_id
uint8 command
string reason

# Used only by COMMAND_REFRESH_TURN; the request/task/trajectory must match.
builtin_interfaces/Time certificate_stamp
uint64 certificate_map_revision
geometry_msgs/Point certificate_position
float64 certificate_target_yaw
float64 certificate_position_allowance
