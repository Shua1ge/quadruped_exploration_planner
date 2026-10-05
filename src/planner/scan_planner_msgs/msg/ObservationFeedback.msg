uint8 STATION_REACHED=1
uint8 VIEWPOINT_REJECTED=2
uint8 TASK_EXHAUSTED=3
uint8 CANCEL_ACK=4
# Current execution/path failure; not spatial evidence against a viewpoint.
uint8 EXECUTION_BLOCKED=5
std_msgs/Header header
uint64 task_id
uint64 task_version
uint64 request_id
int64 trajectory_id
uint64 candidate_id
uint8 state
geometry_msgs/Point station
uint32 station_adjustments
string reason
