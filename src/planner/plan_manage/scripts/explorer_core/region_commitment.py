"""Side-effect-free semi-MDP option for regional exploration.

The policy never publishes a path or changes a planner state.  It only
decides whether the coordinator may widen the current candidate set.
"""

from dataclasses import dataclass
from typing import Hashable, Optional

import math
from scipy.ndimage import label

from .grid import FREE, bresenham


@dataclass(frozen=True)
class BranchCommitment:
    """Immutable world witnesses for a chosen exploration entrance.

    Neither Frontier IDs nor graph IDs define membership. A cross-section of
    an observed corridor separates its forward side from the departure side.
    Curves on the forward side are allowed. If the cut is bypassed, or its
    witness leaves the map, association is unresolved, never 'disconnected'.
    In open space without such a cut, only the original frontier witnesses
    are associated; proximity alone cannot expand the option to another arm.
    Information accounting remains in Explorer's existing target ledger.
    """
    seed: tuple
    departure: tuple
    entrance: tuple
    frontier_witnesses: tuple

    @classmethod
    def initiate(cls, grid, path, frontier_cells, seed, maximum_width=4.0):
        # The certified stand-off pose is a more persistent witness than the
        # FREE/UNKNOWN boundary itself. Moving Frontier cells are associations.
        seed = path[-1] if path else seed
        dense = []
        for a, b in zip(path, path[1:]):
            dense.extend(bresenham(a, b)[:-1])
        if path:
            dense.append(path[-1])
        departure = path[0] if path else seed
        cut = ()
        # Prefer the last verifiable corridor entrance on this route. This
        # excludes sibling arms behind it, rather than inheriting a lineage.
        for index in range(len(dense)-2, 0, -1):
            center = dense[index]
            if math.dist(center, dense[-1])*grid.resolution < .5:
                continue
            previous, following = dense[index-1], dense[index+1]
            dx, dy = following[0]-previous[0], following[1]-previous[1]
            norm = math.hypot(dx, dy)
            if norm == 0:
                continue
            section, closed = {center}, True
            for sign in (-1, 1):
                boundary = False
                for step in range(1, int(maximum_width/grid.resolution)+1):
                    point = (round(center[0]-sign*dy/norm*step),
                             round(center[1]+sign*dx/norm*step))
                    if grid.value(point) != FREE:
                        boundary = True
                        break
                    section.update(bresenham(center, point))
                closed = closed and boundary
            if not closed or len(section)*grid.resolution > maximum_width:
                continue
            trial = cls(grid.cell_to_world(seed), grid.cell_to_world(departure),
                        tuple(grid.cell_to_world(p) for p in sorted(section)),
                        tuple(grid.cell_to_world(p) for p in frontier_cells))
            if trial.domain(grid) is not None:
                cut = trial.entrance
                break
        return cls(grid.cell_to_world(seed), grid.cell_to_world(departure),
                   cut, tuple(grid.cell_to_world(p) for p in frontier_cells))

    def domain(self, grid):
        if not self.entrance:
            return None
        free = grid.data == FREE
        for point in self.entrance:
            cell = grid.world_to_cell(*point)
            if grid.in_bounds(cell):
                free[cell[1], cell[0]] = False
        seed = grid.world_to_cell(*self.seed)
        departure = grid.world_to_cell(*self.departure)
        if not grid.in_bounds(seed) or not free[seed[1], seed[0]]:
            return None
        components, _ = label(free)
        selected = components[seed[1], seed[0]]
        if (grid.in_bounds(departure)
                and components[departure[1], departure[0]] == selected):
            return None  # New bypass makes this entrance ambiguous.
        return components == selected

    def contains(self, grid, frontier_cells, domain=None):
        if self.entrance:
            if domain is None:
                return False
            # A merged cluster straddling the entrance is ambiguous. Do not
            # grant its other arm the active commitment automatically.
            return bool(frontier_cells) and all(
                grid.in_bounds(c) and domain[c[1], c[0]] for c in frontier_cells)
        witnesses = {grid.world_to_cell(*p) for p in self.frontier_witnesses}
        return bool(set(frontier_cells) & witnesses)


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
    phase: str


class ResidualCommitmentGate:
    """Semi-MDP option with an analytic, idempotent termination policy.

    Initiation binds the option to a stable connectivity scope. The caller
    supplies decision epochs (branch options use fresh scan-fusion updates,
    legacy options use map-content revisions). The option terminates when its
    frozen reward source is exhausted or stagnant.  Alternative regions remain
    useful for preparation, but their noisy one-step scores cannot terminate an
    active option.  Re-evaluating one revision is idempotent.
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
        self.cached_decision: Optional[ResidualCommitmentDecision] = None

    @property
    def state(self) -> RegionOptionState:
        remaining = max(0, int(self.current_remaining_cells or 0))
        initial = max(remaining, int(self.initial_remaining_cells or 0))
        return RegionOptionState(
            self.region_id, self.initiation_revision,
            max(0, self.last_revision - self.initiation_revision),
            remaining, max(0, initial - remaining),
            "UNCOMMITTED" if self.region_id is None else "ACTIVE")

    def evaluate(self, region_id: Hashable, map_revision: int,
                 remaining_cells: int,
                 continuation_present: bool = False) -> ResidualCommitmentDecision:
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

        if remaining_cells <= self.minimum_remaining_cells and not continuation_present:
            self.exhausted_streak += 1
        else:
            self.exhausted_streak = 0

        stagnant_revisions = max(0, map_revision - self.progress_revision)
        release_reason = None
        if self.exhausted_streak >= self.exhausted_revisions:
            release_reason = "residual_exhausted"
        elif ((remaining_cells > self.minimum_remaining_cells or continuation_present)
              and stagnant_revisions >= self.stagnation_revisions):
            release_reason = "residual_stagnant"

        decision = ResidualCommitmentDecision(
            release_reason, remaining_cells, self.exhausted_streak,
            stagnant_revisions)
        self.last_revision = map_revision
        self.cached_decision = decision
        return decision

# Scientific name used by new code; the old name remains a compatibility alias
# for launch files and tests written before the option formalisation.
RegionExplorationOption = ResidualCommitmentGate
