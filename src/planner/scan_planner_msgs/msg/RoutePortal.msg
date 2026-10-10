# A directed checkpoint on an actually selected sparse graph connection.
# Identity is valid only within ObservationTask.route_graph_revision.
uint64 connection_id
uint64 source_node_id
uint64 target_node_id
geometry_msgs/Point center
geometry_msgs/Vector3 direction
float64 radius
