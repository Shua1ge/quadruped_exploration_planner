import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from explorer_core.region_commitment import ResidualCommitmentGate  # noqa: E402


def test_gate_retains_region_while_residual_value_is_useful():
    gate = ResidualCommitmentGate(10, 3, 20)

    decision = gate.evaluate(4, 100, 50)

    assert decision.committed
    assert decision.remaining_cells == 50


def test_gate_releases_only_after_consecutive_exhausted_revisions():
    gate = ResidualCommitmentGate(10, 3, 20)

    assert gate.evaluate(4, 100, 8).committed
    assert gate.evaluate(4, 101, 7).committed
    decision = gate.evaluate(4, 102, 6)

    assert decision.release_reason == "residual_exhausted"


def test_repeated_evaluation_of_same_revision_is_idempotent():
    gate = ResidualCommitmentGate(10, 2, 20)

    first = gate.evaluate(4, 100, 8)
    repeated = gate.evaluate(4, 100, 8)

    assert first == repeated
    assert repeated.exhausted_streak == 1


def test_progress_resets_stagnation_revision():
    gate = ResidualCommitmentGate(10, 3, 5, minimum_progress_cells=2)
    gate.evaluate(4, 100, 50)
    gate.evaluate(4, 104, 47)

    decision = gate.evaluate(4, 108, 47)

    assert decision.committed
    assert decision.stagnant_revisions == 4


def test_gate_releases_persistently_stagnant_residual():
    gate = ResidualCommitmentGate(10, 3, 5)
    gate.evaluate(4, 100, 50)

    decision = gate.evaluate(4, 105, 50)

    assert decision.release_reason == "residual_stagnant"


def test_new_active_region_resets_previous_history():
    gate = ResidualCommitmentGate(10, 2, 5)
    gate.evaluate(4, 100, 8)

    decision = gate.evaluate(9, 101, 8)

    assert decision.committed
    assert decision.exhausted_streak == 1
