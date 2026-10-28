uint8 STATION_REACHED=1
uint8 VIEWPOINT_REJECTED=2
uint8 TASK_EXHAUSTED=3
uint8 CANCEL_ACK=4
# Current execution/path failure; not spatial evidence against a viewpoint.
uint8 EXECUTION_BLOCKED=5
# Local path candidates exhausted; retry the route before rejecting the station.
uint8 PATH_REJECTED=6
# Exact task/request/candidate was installed and ACKed by Controller.
uint8 EXECUTION_READY=7
# Local bounded motion search/budget exhausted. NOT proof of an impassable
# viewpoint, portal or region; Explorer alone decides another task.
uint8 LOCAL_MOTION_EXHAUSTED=8
uint8 CANCEL_UNCONFIRMED=9
# Same successor identity. Phase events grant neither motion nor completion.
uint8 TRANSITION_WAIT_STOP=10
uint8 TRANSITION_RESTARTING=11
std_msgs/Header header
uint64 task_id
uint64 task_version
uint64 request_id
int64 trajectory_id
uint64 candidate_id
uint8 state
geometry_msgs/Point station
uint32 station_adjustments
# Execution topology cursor only; must NEVER count as observed information.
uint64 route_graph_revision
uint32 route_connection_cursor
string reason
# Failure scope never grants motion or proves a remote route impassable.
uint8 FAILURE_ACTION=0
uint8 FAILURE_CURRENT_POSE=1
uint8 failure_scope
# Preserve the causal chain when a failed escape cannot rejoin its task.
string trigger_reason
string recovery_reason
string planning_reason
