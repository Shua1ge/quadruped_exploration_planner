uint8 CLEARANCE_NONE=0
uint8 CLEARANCE_PHYSICAL_COLLISION=1
uint8 CLEARANCE_PLANNING_MARGIN=2
uint8 CLEARANCE_ESCAPE_WORSENING=3
uint8 CLEARANCE_ESCAPE_TIMEOUT=4

uint8 SCOPE_PATH=1
uint8 SCOPE_VIEWPOINT=2
uint8 SCOPE_PORTAL=3
uint8 SCOPE_REGION=4

builtin_interfaces/Time stamp
uint64 request_id
int64 trajectory_id
string stage
string reason
geometry_msgs/Point position
uint32 attempted_candidates
float64 required_clearance
uint8 clearance_failure
uint8 failure_scope
