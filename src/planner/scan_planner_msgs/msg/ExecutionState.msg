uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_HEADING_FROZEN=2
uint8 STATE_SOFT_HOLD=3
uint8 STATE_HARD_STOP=4
uint8 STATE_FINISHED=5
uint8 STATE_STALLED=6

uint64 request_id
int64 trajectory_id
uint8 state
float64 execution_time
float64 duration
float64 terminal_error
string reason
