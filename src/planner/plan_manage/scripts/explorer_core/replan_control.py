"""Bound repeated global replans while the planning inputs are unchanged."""

from dataclasses import dataclass
from typing import Optional

from .grid import Cell


def retry_epoch(now_ns: int, period_seconds: float) -> int:
    """Return a time deadline bucket independent of sensor frame count."""
    period_ns = max(1, int(max(0.1, float(period_seconds)) * 1e9))
    return max(0, int(now_ns)) // period_ns


@dataclass(frozen=True)
class ReplanContext:
    robot_cell: Cell
    map_content_revision: int
    route_constraint_revision: int
    retry_epoch: int = 0


class ReplanGate:
    """Allow a small number of attempts for one unchanged planning context."""

    def __init__(self, max_attempts: int = 2):
        self.max_attempts = max(1, int(max_attempts))
        self.context: Optional[ReplanContext] = None
        self.attempts = 0

    def allow(self, context: ReplanContext) -> bool:
        if context != self.context:
            self.context = context
            self.attempts = 0
        if self.attempts >= self.max_attempts:
            return False
        self.attempts += 1
        return True

    def reset(self):
        self.context = None
        self.attempts = 0
