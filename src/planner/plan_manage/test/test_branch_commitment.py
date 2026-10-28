import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / 'scripts'))
from explorer_core.grid import ExplorationGrid, FREE, OCCUPIED, UNKNOWN
from explorer_core.region_commitment import BranchCommitment


def fork():
    grid = ExplorationGrid(20., 20., 1., 0., 0.)
    grid.data[:] = OCCUPIED
    grid.data[8:11, 1:18] = FREE
    grid.data[1:11, 7:10] = FREE
    grid.data[8:11, 18:] = UNKNOWN
    intent = BranchCommitment.initiate(grid, [(2,9),(15,9)], {(17,9)}, (17,9))
    assert intent.entrance
    return grid, intent


def test_entry_separates_sibling_arm_without_frontier_or_graph_ids():
    grid, intent = fork()
    domain = intent.domain(grid)
    assert intent.contains(grid, {(17,9)}, domain)
    assert not intent.contains(grid, {(8,1)}, domain)
    assert not intent.contains(grid, {(17,9),(8,1)}, domain)
    grid.data[8:11, 18:20] = FREE  # Forward frontier moves and splits.
    assert intent.contains(grid, {(19,8),(19,10)}, intent.domain(grid))


def test_curved_branch_can_turn_back_in_world_x_without_crossing_entry():
    grid, intent = fork()
    grid.data[8:18, 15:18] = FREE
    grid.data[15:18, 10:18] = FREE
    assert intent.contains(grid, {(10,16)}, intent.domain(grid))
    assert not intent.contains(grid, {(8,1)}, intent.domain(grid))


def test_temporary_unknown_witness_does_not_mutate_or_rebind_entrance():
    grid, intent = fork()
    frozen = intent.entrance
    grid.data[9,15] = UNKNOWN
    assert intent.domain(grid) is None
    assert not intent.contains(grid, {(17,9)}, intent.domain(grid))
    assert intent.entrance == frozen
    grid.data[9,15] = FREE
    assert intent.contains(grid, {(17,9)}, intent.domain(grid))
    assert intent.entrance == frozen


def test_new_bypass_is_ambiguous_rather_than_expanding_into_other_arm():
    grid, intent = fork()
    grid.data[9:14, 2:5] = FREE
    grid.data[11:14, 2:18] = FREE
    grid.data[9:14, 15:18] = FREE
    assert intent.domain(grid) is None
    assert not intent.contains(grid, {(8,1)}, intent.domain(grid))


def test_world_witness_survives_grid_origin_change_and_missing_window():
    grid, intent = fork()
    shifted = ExplorationGrid(20.,20.,1.,1.,0.)
    shifted.data[:,:-1] = grid.data[:,1:]
    shifted.data[:,-1] = UNKNOWN
    assert intent.contains(shifted, {(16,9)}, intent.domain(shifted))
    absent = ExplorationGrid(5.,5.,1.,0.,0.)
    absent.data[:] = FREE
    assert intent.domain(absent) is None
    assert intent.contains(grid, {(17,9)}, intent.domain(grid))


def test_open_space_proximity_alone_never_adds_another_frontier():
    grid = ExplorationGrid(20.,20.,1.,0.,0.)
    grid.data[:] = FREE
    intent = BranchCommitment.initiate(grid, [(2,9),(15,9)], {(17,9)}, (17,9))
    assert not intent.entrance
    assert intent.contains(grid, {(17,9)})
    assert not intent.contains(grid, {(17,10)})
