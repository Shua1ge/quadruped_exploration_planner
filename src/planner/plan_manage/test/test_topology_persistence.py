import pathlib
import sys

import numpy as np
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from explorer_core.grid import FREE, OCCUPIED, UNKNOWN  # noqa: E402
from explorer_core.topology import stable_node_id  # noqa: E402
from explorer_core.topology_persistence import (  # noqa: E402
    ExplicitBlockageTracker, node_world_xy, occupancy_at_world)


def test_node_world_position_round_trips_stable_quantization():
    node_id = stable_node_id((3, 4), (-1.0, 2.0), 0.2)
    world = node_world_xy(node_id, 0.2)

    assert world == pytest.approx((-0.3, 2.9))


def test_world_lookup_distinguishes_free_unknown_and_occupied():
    occupancy = np.asarray([[FREE, UNKNOWN, OCCUPIED]], dtype=np.int8)

    assert occupancy_at_world(occupancy, (1.0, 2.0), 0.2,
                              (1.1, 2.1)) == FREE
    assert occupancy_at_world(occupancy, (1.0, 2.0), 0.2,
                              (1.3, 2.1)) == UNKNOWN
    assert occupancy_at_world(occupancy, (1.0, 2.0), 0.2,
                              (1.5, 2.1)) == OCCUPIED
    assert occupancy_at_world(occupancy, (1.0, 2.0), 0.2,
                              (0.9, 2.1)) is None


def test_missing_element_requires_consecutive_explicit_occupancy():
    tracker = ExplicitBlockageTracker(confirmations=3)

    assert not tracker.observe(7, False, [OCCUPIED], True)
    assert not tracker.observe(7, False, [OCCUPIED], True)
    assert tracker.observe(7, False, [OCCUPIED], True)


def test_unknown_free_and_reappearance_break_occupied_streak():
    tracker = ExplicitBlockageTracker(confirmations=2)

    assert not tracker.observe(9, False, [OCCUPIED], True)
    assert not tracker.observe(9, False, [UNKNOWN], True)
    assert not tracker.observe(9, False, [OCCUPIED], True)
    assert not tracker.observe(9, True, [OCCUPIED], True)
    assert not tracker.observe(9, False, [FREE], True)
    assert tracker.pending == 0


def test_invalid_patch_cannot_accumulate_destructive_evidence():
    tracker = ExplicitBlockageTracker(confirmations=2)

    assert not tracker.observe(11, False, [OCCUPIED], False)
    assert not tracker.observe(11, False, [OCCUPIED], True)
    assert tracker.pending == 1
