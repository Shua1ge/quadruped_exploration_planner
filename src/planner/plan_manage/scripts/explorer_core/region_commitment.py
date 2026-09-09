"""Small, side-effect-free policy for residual region commitment.

The policy never publishes a path or changes a planner state.  It only
decides whether the coordinator may widen the current candidate set.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ResidualCommitmentDecision:
    release_reason: Optional[str]
    remaining_cells: int
    exhausted_streak: int
    stagnant_revisions: int

    @property
    def committed(self) -> bool:
        return self.release_reason is None


class ResidualCommitmentGate:
    """Release a region only after residual value is exhausted or stagnant.

    Map-content revisions, rather than timer calls, provide the clock.  A
    repeated evaluation of the same revision is therefore idempotent.
    """

    def __init__(self, minimum_remaining_cells: int,
                 exhausted_revisions: int, stagnation_revisions: int,
                 minimum_progress_cells: int = 1):
        if minimum_remaining_cells < 0:
            raise ValueError("minimum_remaining_cells must be non-negative")
        if exhausted_revisions < 1:
            raise ValueError("exhausted_revisions must be positive")
        if stagnation_revisions < 1:
            raise ValueError("stagnation_revisions must be positive")
        if minimum_progress_cells < 1:
            raise ValueError("minimum_progress_cells must be positive")
        self.minimum_remaining_cells = minimum_remaining_cells
        self.exhausted_revisions = exhausted_revisions
        self.stagnation_revisions = stagnation_revisions
        self.minimum_progress_cells = minimum_progress_cells
        self.reset()

    def reset(self, region_id: Optional[int] = None,
              map_revision: int = -1,
              remaining_cells: Optional[int] = None) -> None:
        self.region_id = region_id
        self.last_revision = map_revision
        self.progress_revision = map_revision
        self.progress_reference = remaining_cells
        self.exhausted_streak = 0
        self.cached_decision: Optional[ResidualCommitmentDecision] = None

    def evaluate(self, region_id: int, map_revision: int,
                 remaining_cells: int) -> ResidualCommitmentDecision:
        remaining_cells = max(0, int(remaining_cells))
        if region_id != self.region_id:
            self.reset(region_id, map_revision, remaining_cells)
        elif map_revision == self.last_revision and self.cached_decision is not None:
            return self.cached_decision

        if self.progress_reference is None:
            self.progress_reference = remaining_cells
            self.progress_revision = map_revision
        elif self.progress_reference - remaining_cells >= self.minimum_progress_cells:
            self.progress_reference = remaining_cells
            self.progress_revision = map_revision
        elif remaining_cells > self.progress_reference:
            # Frontier growth is new work, not evidence that exploration stalled.
            self.progress_reference = remaining_cells
            self.progress_revision = map_revision

        if remaining_cells <= self.minimum_remaining_cells:
            self.exhausted_streak += 1
        else:
            self.exhausted_streak = 0

        stagnant_revisions = max(0, map_revision - self.progress_revision)
        release_reason = None
        if self.exhausted_streak >= self.exhausted_revisions:
            release_reason = "residual_exhausted"
        elif (remaining_cells > self.minimum_remaining_cells
              and stagnant_revisions >= self.stagnation_revisions):
            release_reason = "residual_stagnant"

        decision = ResidualCommitmentDecision(
            release_reason, remaining_cells, self.exhausted_streak,
            stagnant_revisions)
        self.last_revision = map_revision
        self.cached_decision = decision
        return decision
