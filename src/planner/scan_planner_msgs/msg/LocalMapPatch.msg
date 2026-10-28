std_msgs/Header header
uint64 map_revision
uint64 pose_revision
float32 resolution
geometry_msgs/Point origin
uint32 width
uint32 height
# Raw measured slice at origin.z: 100 occupied, 0 observed free, -1 unknown.
# No inflation and no hazard projection: clearance is Global's contract.
int8[] occupancy
float32 generation_ms
# Local terrain scan counter this patch was built from.
uint32 scan
# Changes only when raw occupancy or terrain tiles change materially.
uint64 content_revision
# Terrain summaries for the 2 m tiles overlapping this patch window.
TerrainTileSummary[] terrain_tiles
