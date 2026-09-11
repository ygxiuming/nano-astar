"""API semantics, GIL release, heuristic optimality, and history consistency."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from gridutil import make_grid, grid_to_nx, nx_dijkstra_cost

import nano_astar
from nano_astar import astar
from nano_astar import _core


# ---------------------------------------------------------------------------
# API surface
# ---------------------------------------------------------------------------

def test_exports():
    assert set(nano_astar.__all__) == {"astar", "__version__"}
    assert isinstance(nano_astar.__version__, str)


def test_return_types():
    grid = np.zeros((15, 15), dtype=np.uint8)
    path, cost = astar(grid, (0, 0), (14, 14))
    assert path.dtype == np.int32 and path.shape == (15, 2)
    assert isinstance(cost, float)
    # path must be walkable step-by-step
    steps = np.abs(np.diff(path, axis=0)).sum(axis=1)
    assert (steps <= 2).all()


def test_input_dtypes_and_layouts():
    base = np.zeros((20, 20))
    base[10, 3:18] = 1  # wall with a gap at col 2 / 18
    expected = astar(base.astype(np.uint8), (0, 0), (19, 19))
    for g in (
        base,                                   # float64
        base.astype(np.int64),                  # int64
        base.astype(bool),                      # bool
        np.asfortranarray(base.astype(np.uint8)),  # Fortran order
        base.astype(np.uint8).T.T,              # non-standard strides path
        base.tolist(),                          # plain lists
    ):
        result = astar(g, (0, 0), (19, 19))
        assert result is not None
        assert abs(result[1] - expected[1]) < 1e-9


def test_value_errors():
    grid = np.zeros((10, 10), dtype=np.uint8)
    grid[4, 4] = 1
    with pytest.raises(ValueError):
        astar(grid, (0, 0), (10, 0))            # goal out of bounds
    with pytest.raises(ValueError):
        astar(grid, (-1, 0), (9, 9))            # start out of bounds
    with pytest.raises(ValueError):
        astar(grid, (4, 4), (9, 9))             # start on obstacle
    with pytest.raises(ValueError):
        astar(grid, (0, 0), (4, 4))             # goal on obstacle
    with pytest.raises(ValueError):
        astar(np.zeros((4, 4, 4), np.uint8), (0, 0), (3, 3))  # 3-D grid
    with pytest.raises(ValueError):
        astar(np.zeros((0, 0), np.uint8), (0, 0), (0, 0))     # empty grid
    with pytest.raises(ValueError):
        astar(grid, (0, 0), (9, 9), heuristic="chebyshev")    # unknown


def test_unreachable_returns_none_not_raise():
    grid = np.ones((8, 8), dtype=np.uint8)
    grid[0, 0] = grid[7, 7] = 0
    assert astar(grid, (0, 0), (7, 7)) is None


def test_start_equals_goal():
    grid = np.zeros((5, 5), dtype=np.uint8)
    path, cost = astar(grid, (2, 3), (2, 3))
    assert cost == 0.0
    assert path.tolist() == [[2, 3]]


# ---------------------------------------------------------------------------
# Heuristic optimality (vs Dijkstra ground truth)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("heuristic",
                         ["octile", "manhattan", "euclidean", "diagonal"])
@pytest.mark.parametrize("diagonal", [False, True])
def test_heuristics_are_optimal(heuristic, diagonal):
    # manhattan overestimates on 8-connected grids (dx+dy > octile), so it is
    # only admissible — and therefore only guaranteed optimal — for 4-conn.
    if heuristic == "manhattan" and diagonal:
        pytest.skip("manhattan is not admissible for 8-connected grids")
    rng = np.random.default_rng(7)
    for _ in range(10):
        grid = make_grid(rng, int(rng.integers(10, 31)), 0.25)
        free = np.argwhere(grid == 0)
        if len(free) < 2:
            continue
        a, b = free[rng.choice(len(free), 2, replace=False)]
        start, goal = tuple(a), tuple(b)
        result = astar(grid, start, goal, heuristic=heuristic,
                       diagonal=diagonal)
        G = grid_to_nx(grid, diagonal)
        expected = nx_dijkstra_cost(G, start, goal)
        if expected is None:
            assert result is None
        else:
            assert result is not None
            assert abs(result[1] - expected) < 1e-9, heuristic


# ---------------------------------------------------------------------------
# return_history consistency
# ---------------------------------------------------------------------------

def test_history_consistency():
    rng = np.random.default_rng(42)
    grid = make_grid(rng, 40, 0.3)
    free = np.argwhere(grid == 0)
    a, b = free[rng.choice(len(free), 2, replace=False)]
    start, goal = tuple(a), tuple(b)

    plain = astar(grid, start, goal)
    with_hist = astar(grid, start, goal, return_history=True)
    assert (plain is None) == (with_hist is None)
    if plain is None:
        return
    path, cost = plain
    path2, cost2, history = with_hist
    assert cost == cost2                        # same optimal cost
    assert np.array_equal(path, path2)          # same path
    assert history.dtype == np.int32 and history.ndim == 2
    assert history.shape[1] == 2
    assert tuple(history[-1]) == goal           # goal is closed last
    # every closed cell is free and inside the grid
    assert (grid[history[:, 0], history[:, 1]] == 0).all()


# ---------------------------------------------------------------------------
# Engine equivalence: bucket queue vs 4-ary heap on integer grids
# ---------------------------------------------------------------------------

def test_bucket_queue_matches_heap():
    rng = np.random.default_rng(1234)
    for _ in range(20):
        grid = make_grid(rng, int(rng.integers(10, 41)), 0.3)
        free = np.argwhere(grid == 0)
        if len(free) < 2:
            continue
        a, b = free[rng.choice(len(free), 2, replace=False)]
        start, goal = tuple(a), tuple(b)
        for h in ("manhattan", "octile", "euclidean", "diagonal"):
            bucket = _core.astar(grid, start, goal, h, False, False, False)
            heap = _core.astar(grid, start, goal, h, False, False, True)
            assert (bucket is None) == (heap is None)
            if bucket is not None:
                assert bucket[1] == heap[1]


# ---------------------------------------------------------------------------
# GIL release: 4 threads, no deadlock, correct results
# ---------------------------------------------------------------------------

def test_gil_released_under_threads():
    rng = np.random.default_rng(99)
    grids, queries, expected = [], [], []
    for _ in range(4):
        while True:
            g = make_grid(rng, 1000, 0.28)
            free = np.argwhere(g == 0)
            a, b = free[rng.choice(len(free), 2, replace=False)]
            res = astar(g, tuple(a), tuple(b))
            if res is not None and res[1] > 500:  # want long searches
                grids.append(g)
                queries.append((tuple(a), tuple(b)))
                expected.append(res[1])
                break

    results = [None] * 4
    errors = []

    def worker(i):
        try:
            results[i] = astar(grids[i], *queries[i])
        except Exception as exc:  # pragma: no cover
            errors.append(exc)

    t0 = time.perf_counter()
    for i in range(4):
        astar(grids[i], *queries[i])
    serial = time.perf_counter() - t0

    t0 = time.perf_counter()
    threads = [threading.Thread(target=worker, args=(i,)) for i in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=120)
    parallel = time.perf_counter() - t0

    assert not any(t.is_alive() for t in threads), "deadlock: thread hung"
    assert not errors
    for i in range(4):
        assert results[i] is not None
        assert results[i][1] == expected[i]
    speedup = serial / parallel
    print(f"\n4-thread speedup: {speedup:.2f}x "
          f"(serial {serial:.3f}s, parallel {parallel:.3f}s)")
    assert speedup > 1.0, "threads should not be slower than serial"
