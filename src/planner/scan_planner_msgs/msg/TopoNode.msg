uint64 id
geometry_msgs/Point position
uint8 degree
float32 clearance
# Terrain and experience aggregated by Global over this node's footprint.
# traversability is a route-cost multiplier in [0.8, 2.0], never a hard block.
float32 ground_min
float32 ground_median
float32 ground_max
float32 terrain_slope
float32 terrain_roughness
uint32 rise_cells
uint32 drop_cells
uint32 overhead_cells
uint32 measured_columns
uint32 last_scan
uint8 terrain_state
float32 traversability
uint32 traversals
float32 experience_score
