# One world-aligned square tile of Local terrain evidence.
# Ground statistics are derived from measured ground-height facts only:
# a cell without a measured height is not assumed flat, it is simply not
# counted.  Counts are per measured column and non-exclusive (a column can
# carry both drop and rise evidence).
int32 tile_x                # world tile index, tile origin = (tile_x*tile_size, tile_y*tile_size)
int32 tile_y
float32 tile_size           # metres
float32 ground_min          # world z (m); NaN when no measured ground column
float32 ground_median
float32 ground_max
float32 slope               # rad, least-squares plane over per-column ground points; NaN if < 3
float32 roughness           # m, RMS residual of that plane; NaN if < 3
uint32 support_cells        # columns whose lowest and highest facts are within floor tolerance
uint32 rise_cells           # columns with a body-height rise fact
uint32 drop_cells           # columns with a drop fact
uint32 overhead_cells       # columns with an overhead fact above the body top
uint32 measured_columns     # columns holding at least one measured height fact
uint32 last_scan            # Local terrain scan counter of the freshest fact in the tile
uint64 content_revision     # hash of this summary, changes only when content changes
