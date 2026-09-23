"""Temporal guards for merging rolling 2D topology patches."""

import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

import numpy as np

from .grid import OCCUPIED


def node_world_xy(node_id: int, resolution: float) -> Tuple[float, float]:
    """Recover the centre of a globally quantized topology-node cell."""
    gx = (int(node_id) >> 32) & 0xFFFFFFFF
    gy = int(node_id) & 0xFFFFFFFF
    gx = gx - (1 << 32) if gx & (1 << 31) else gx
    gy = gy - (1 << 32) if gy & (1 << 31) else gy
    return ((gx + 0.5) * resolution, (gy + 0.5) * resolution)


def occupancy_at_world(occupancy: np.ndarray,
                       origin: Tuple[float, float], resolution: float,
                       point: Tuple[float, float]) -> Optional[int]:
    """Return the patch value at a world point, or None outside the patch."""
    if resolution <= 0.0:
        return None
    x = int(math.floor((point[0] - origin[0]) / resolution))
    y = int(math.floor((point[1] - origin[1]) / resolution))
    if not (0 <= y < occupancy.shape[0] and 0 <= x < occupancy.shape[1]):
        return None
    return int(occupancy[y, x])


class ExplicitBlockageTracker:
    """Remove topology only after consecutive, explicit occupied evidence.

    Missing skeleton elements are not obstacle evidence. UNKNOWN, FREE, a
    reappearing element, or an invalid patch breaks the occupied streak so a
    transient rolling-map frame cannot erase the persistent global graph.
    """

    def __init__(self, confirmations: int):
        if confirmations < 1:
            raise ValueError("confirmations must be positive")
        self.confirmations = int(confirmations)
        self._streaks: Dict[int, int] = {}

    def observe(self, key: int, present: bool,
                sampled_states: Iterable[Optional[int]],
                destructive_updates_allowed: bool) -> bool:
        states = tuple(sampled_states)
        explicitly_blocked = any(state == OCCUPIED for state in states)
        if present or not destructive_updates_allowed or not explicitly_blocked:
            self._streaks.pop(int(key), None)
            return False
        streak = self._streaks.get(int(key), 0) + 1
        self._streaks[int(key)] = streak
        return streak >= self.confirmations

    def forget(self, keys: Sequence[int]) -> None:
        for key in keys:
            self._streaks.pop(int(key), None)

    @property
    def pending(self) -> int:
        return len(self._streaks)
