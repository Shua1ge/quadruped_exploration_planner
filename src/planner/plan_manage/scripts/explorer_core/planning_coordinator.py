"""Thread-local immutable map snapshots for long-running global planning."""

import functools
import threading
from contextlib import contextmanager
from typing import Any, Iterator, Optional

from .grid import ExplorationGrid, Point2


class PlanningSnapshotCoordinator:
    """Own the live-grid lock and bind one snapshot per planning thread."""

    def __init__(self):
        self.lock = threading.RLock()
        self.context = threading.local()

    def value(self, name: str, live_value: Any) -> Any:
        if hasattr(self.context, name):
            return getattr(self.context, name)
        return live_value

    @contextmanager
    def bind(self, live_grid: ExplorationGrid,
             position: Optional[Point2], map_update_count: int,
             map_content_revision: int) -> Iterator[None]:
        """Bind a stable grid and revision to the current planning callback."""
        if getattr(self.context, "grid", None) is not None:
            yield
            return
        with self.lock:
            self.context.grid = live_grid.snapshot()
            self.context.map_update_count = map_update_count
            self.context.map_content_revision = map_content_revision
        self.context.position = position
        try:
            yield
        finally:
            del self.context.grid
            del self.context.position
            del self.context.map_update_count
            del self.context.map_content_revision


def with_planning_grid_snapshot(callback):
    """Run one planning callback against one immutable occupancy revision."""
    @functools.wraps(callback)
    def wrapped(self, *args, **kwargs):
        with self.planning_grid_snapshot():
            return callback(self, *args, **kwargs)
    return wrapped
