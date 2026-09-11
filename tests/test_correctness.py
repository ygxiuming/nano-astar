"""Randomized differential tests: nano-astar vs networkx on 200 random grids.

Covers sizes 5..50, obstacle densities 0.0/0.3/0.5/1.0, 4- and 8-connectivity,
and edge cases (start == goal, start/goal on obstacle, unreachable pairs).
"""

from __future__ import annotations

import numpy as np
import pytest

from gridutil import make_grid, grid_to_nx, nx_solve

from nano_astar import astar

N_CASES = 200


def _cases():
    rng = np.random.default_rng(20260911)
    for i in range(N_CASES):
        size = int(rng.integers(5, 51))
        density = [0.0, 0.3, 0.5, 1.0][i % 4]
        grid = make_grid(rng, size, density)
        diagonal = bool(i % 2)
        start = (int(rng.integers(0, size)), int(rng.integers(0, size)))
        goal = (int(rng.integers(0, size)), int(rng.integers(0, size)))
        if i % 25 == 0:
            goal = start  # start == goal case
        yield i, grid, start, goal, diagonal


@pytest.mark.parametrize("i,grid,start,goal,diagonal", list(_cases()))
def test_matches_networkx(i, grid, start, goal, diagonal):
    heuristic = "octile" if diagonal else "manhattan"
    G = grid_to_nx(grid, diagonal)
    expected = nx_solve(G, start, goal, diagonal)

    try:
        result = astar(grid, start, goal, heuristic=heuristic,
                       diagonal=diagonal)
    except ValueError:
        result = None  # start/goal on obstacle: invalid for us, absent for nx

    if expected is None:
        assert result is None, (
            f"case {i}: networkx says unreachable, nano-astar found a path"
        )
        return

    assert result is not None, (
        f"case {i}: networkx found cost {expected}, nano-astar returned None"
    )
    path, cost = result
    assert path.dtype == np.int32 and path.ndim == 2 and path.shape[1] == 2
    assert tuple(path[0]) == start and tuple(path[-1]) == goal

    if diagonal:
        assert abs(cost - expected) < 1e-6, (
            f"case {i}: cost {cost} != networkx {expected}"
        )
    else:
        assert cost == expected, (  # integer grid: exact equality
            f"case {i}: cost {cost} != networkx {expected}"
        )


def test_all_obstacle_grid():
    grid = np.ones((20, 20), dtype=np.uint8)
    with pytest.raises(ValueError):
        astar(grid, (0, 0), (19, 19))


def test_unreachable_region():
    grid = np.zeros((30, 30), dtype=np.uint8)
    grid[:, 15] = 1  # full vertical wall
    for diagonal in (False, True):
        assert astar(grid, (5, 5), (5, 25), diagonal=diagonal) is None


def test_open_grid_exact_costs():
    grid = np.zeros((10, 10), dtype=np.uint8)
    _, c4 = astar(grid, (0, 0), (9, 9), diagonal=False)
    assert c4 == 18.0
    _, c8 = astar(grid, (0, 0), (9, 9), diagonal=True)
    assert abs(c8 - 9 * 2**0.5) < 1e-12
