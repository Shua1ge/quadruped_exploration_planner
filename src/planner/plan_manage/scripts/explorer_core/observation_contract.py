"""Bounded wire representation of the existing Explorer-owned objective.

No ROS imports: serialization and lifecycle rules are independently testable.
"""

import math


def representative_cells(cells, limit):
    """Spatial strata, deterministic ties, bounded output.

    Completion always uses the full frozen target_cells, never this subset.
    This version deliberately has no unknown 'gain' estimate on individual
    boundary cells: spatial coverage is preferable to invented gain ordering.
    """
    limit = int(limit)
    if limit <= 0:
        raise ValueError("representative limit must be positive")
    cells = set(cells)
    if len(cells) <= limit:
        return tuple(sorted(cells))
    xmin = min(x for x, _ in cells)
    xmax = max(x for x, _ in cells)
    ymin = min(y for _, y in cells)
    ymax = max(y for _, y in cells)
    width, height = xmax-xmin+1, ymax-ymin+1
    nx = max(1, min(limit, int(math.sqrt(limit*width/height))))
    ny = max(1, limit//nx)
    bins = {}
    for x, y in cells:
        bx = min(nx-1, (x-xmin)*nx//width)
        by = min(ny-1, (y-ymin)*ny//height)
        center = (xmin+(bx+.5)*width/nx, ymin+(by+.5)*height/ny)
        rank = ((x-center[0])**2+(y-center[1])**2, x, y)
        key = (bx, by)
        if key not in bins or rank < bins[key]:
            bins[key] = rank
    selected = {(rank[1], rank[2]) for rank in bins.values()}
    # Thin/bent boundaries can leave strata empty. Fill deterministically,
    # retaining all spatially selected representatives.
    for cell in sorted(cells):
        if len(selected) >= limit:
            break
        selected.add(cell)
    return tuple(sorted(selected))


def bind_task_identity(task, task_id, grid):
    """Assign once; local path / terminal changes must not reset the task."""
    if task.task_id:
        return task
    if int(task_id) <= 0:
        raise ValueError("task_id must be a positive monotonic generation")
    task.task_id = int(task_id)
    task.task_version = 1
    task.require_sensor_evidence = True
    task.initial_sensor_revision = grid.sensor_revision
    task.last_evaluated_sensor_revision = grid.sensor_revision
    task.last_closure_sensor_revision = grid.sensor_revision
    return task
