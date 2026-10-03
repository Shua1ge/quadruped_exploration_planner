# Explorer owns completion. Local goal changes do not change task_version.
# Objective / lineage changes require a newer version or a new task_id.
uint8 ACTIVE=1
uint8 SUPERSEDED=2
uint8 CANCELLED=3
std_msgs/Header header
uint64 task_id
uint64 task_version
uint64 request_id
uint8 lifecycle
bool has_region
uint64 region_id
geometry_msgs/Point goal
# Allowed domain: corridor around coarse_route UNION observation disk.
geometry_msgs/Point[] coarse_route
float64 corridor_radius
float64 observation_radius
# Spatially distributed representatives; NOT the completion denominator.
geometry_msgs/Point[] target_cells
uint32 target_cell_count
float64 resolution
uint32 representative_selection_version
geometry_msgs/Point[] terminal_cells
uint32 max_station_adjustments
