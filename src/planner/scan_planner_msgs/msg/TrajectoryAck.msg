uint8 STATUS_ACCEPTED=1
uint8 STATUS_REJECTED=2
uint8 STATUS_TASK_RETIRED=3
# Task-mode installation results. ACCEPTED remains a legacy-only adapter.
uint8 STATUS_ACTIVATED=4
uint8 STATUS_HOLD_INSTALLED=5

builtin_interfaces/Time stamp
uint64 request_id
uint64 task_id
uint64 task_version
uint64 candidate_id
int64 trajectory_id
uint8 status
string reason
float64 matched_time
float64 position_error
float64 yaw_error
