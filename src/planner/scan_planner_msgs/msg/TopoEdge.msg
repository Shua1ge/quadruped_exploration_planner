uint64 id
uint64 source_id
uint64 target_id
float32 length
float32 minimum_clearance
geometry_msgs/Point[] polyline
# Terrain and experience aggregated by Global along this edge's polyline.
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
