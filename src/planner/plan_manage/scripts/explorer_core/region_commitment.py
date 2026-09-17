"""Side-effect-free semi-MDP option for regional exploration.

The policy never publishes a path or changes a planner state.  It only
decides whether the coordinator may widen the current candidate set.
"""

from dataclasses import dataclass
from typing import Hashable, Optional


@dataclass(frozen=True)
class ResidualCommitmentDecision:
    release_reason: Optional[str]
    remaining_cells: int
    exhausted_streak: int
    stagnant_revisions: int

    @property
    def committed(self) -> bool:
        return self.release_reason is None


@dataclass(frozen=True)
class RegionOptionState:
    """Observable state of one temporally extended regional action."""

    option_id: Optional[Hashable]
    initiation_revision: int
    elapsed_revisions: int
    remaining_cells: int
    collected_cells: int
    opportunity_streak: int
    phase: str


class ResidualCommitmentGate:
    """Semi-MDP option with an analytic, idempotent termination policy.

    Initiation binds the option to a stable connectivity scope.  Map-content
    revisions are the decision epochs.  The option terminates when its reward
    source is exhausted/stagnant or when a persistently better alternative
    exceeds the existing switching hysteresis.  Re-evaluating one revision is
    idempotent.
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

    def reset(self, region_id: Optional[Hashable] = None,
              map_revision: int = -1,
              remaining_cells: Optional[int] = None) -> None:
        self.region_id = region_id
        self.initiation_revision = map_revision
        self.initial_remaining_cells = remaining_cells
        self.current_remaining_cells = remaining_cells
        self.last_revision = map_revision
        self.progress_revision = map_revision
        self.progress_reference = remaining_cells
        self.exhausted_streak = 0
        self.opportunity_streak = 0
        self.opportunity_revision = -1
        self.cached_decision: Optional[ResidualCommitmentDecision] = None

    @property
    def state(self) -> RegionOptionState:
        remaining = max(0, int(self.current_remaining_cells or 0))
        initial = max(remaining, int(self.initial_remaining_cells or 0))
        return RegionOptionState(
            self.region_id, self.initiation_revision,
            max(0, self.last_revision - self.initiation_revision),
            remaining, max(0, initial - remaining),
            self.opportunity_streak,
            "UNCOMMITTED" if self.region_id is None else "ACTIVE")

    def evaluate(self, region_id: Hashable, map_revision: int,
                 remaining_cells: int) -> ResidualCommitmentDecision:
        remaining_cells = max(0, int(remaining_cells))
        self.current_remaining_cells = remaining_cells
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

    def evaluate_opportunity(self, region_id: Hashable, map_revision: int,
                             current_value: float, alternative_value: float,
                             switch_ratio: float) -> Optional[str]:
        """Return ``opportunity_dominated`` after persistent Bellman advantage.

        For an option o, continue while Q(s,o) is at least the best competing
        one-step value after switching cost.  ``switch_ratio`` is the existing
        hysteresis and therefore introduces no additional tuning parameter.
        """
        if region_id != self.region_id or map_revision == self.opportunity_revision:
            return None
        self.opportunity_revision = map_revision
        dominated = (alternative_value > 0.0 and
                     alternative_value > max(0.0, current_value) * switch_ratio)
        self.opportunity_streak = self.opportunity_streak + 1 if dominated else 0
        if self.opportunity_streak >= self.exhausted_revisions:
            return "opportunity_dominated"
        return None


# Scientific name used by new code; the old name remains a compatibility alias
# for launch files and tests written before the option formalisation.
RegionExplorationOption = ResidualCommitmentGate
