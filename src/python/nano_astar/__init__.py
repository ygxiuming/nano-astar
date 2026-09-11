"""nano-astar — tiny, fast A* pathfinding on occupancy grids.

C++ core (bucket queue for integer costs, 4-ary heap for float costs),
nanobind bindings, GIL released during search.
"""

from __future__ import annotations

import warnings
from typing import Optional, Sequence, Tuple, Union

import numpy as np

from ._core import astar as _astar_cpp

__version__ = "0.1.2"
__all__ = ["astar", "__version__"]

_HEURISTICS = ("octile", "manhattan", "euclidean", "diagonal")

# numpy 2 renamed np.bool_; .view(np.uint8) on the comparison result works on
# both numpy 1.x and 2.x.
Path = np.ndarray  # (n, 2) int32, row/col pairs
Result = Union[
    Tuple[Path, float],
    Tuple[Path, float, np.ndarray],
    None,
]


def astar(
    grid: np.ndarray,
    start: Sequence[int],
    goal: Sequence[int],
    heuristic: str = "octile",
    diagonal: bool = True,
    return_history: bool = False,
) -> Result:
    """Run A* on a 2-D occupancy grid.

    Parameters
    ----------
    grid:
        2-D array-like; 0 = free cell, nonzero = obstacle.  A contiguous
        ``uint8`` array is passed to C++ with zero copies; any other dtype is
        converted once with ``grid != 0``.
    start, goal:
        ``(row, col)`` cell indices.
    heuristic:
        One of ``"octile"`` (default), ``"manhattan"``, ``"euclidean"``,
        ``"diagonal"``.  All heuristics run inline in C++; there is no Python
        callback overhead.
    diagonal:
        If True (default) allow 8-connected moves with cost ``sqrt(2)`` per
        diagonal step (no corner cutting).  If False, 4-connected moves with
        unit cost — this selects the integer bucket-queue engine.
    return_history:
        If True, also return the exploration order as an ``(m, 2)`` int32
        array of closed cells (useful for animations).

    Returns
    -------
    ``(path, cost)`` or ``(path, cost, history)`` where ``path`` is an
    ``(n, 2)`` int32 array from ``start`` to ``goal``; ``None`` if no path
    exists (never raised).

    Raises
    ------
    ValueError
        If ``start``/``goal`` are out of bounds or on an obstacle, the grid is
        not 2-D, or ``heuristic`` is unknown.
    """
    arr = np.asarray(grid)
    if arr.ndim != 2:
        raise ValueError(f"grid must be 2-D, got {arr.ndim}-D")
    rows, cols = arr.shape
    if rows == 0 or cols == 0:
        raise ValueError("grid must be non-empty")
    if heuristic not in _HEURISTICS:
        raise ValueError(
            f"unknown heuristic {heuristic!r}; expected one of {_HEURISTICS}"
        )
    if diagonal and heuristic == "manhattan":
        warnings.warn(
            "manhattan is inadmissible on 8-connected grids; the returned "
            "path may be suboptimal. Use 'octile' (the default) instead.",
            RuntimeWarning,
            stacklevel=2,
        )

    # Zero-copy fast path: uint8 + C-contiguous goes straight to C++.
    if arr.dtype != np.uint8 or not arr.flags.c_contiguous:
        arr = np.ascontiguousarray((arr != 0).view(np.uint8))

    s = (int(start[0]), int(start[1]))
    g = (int(goal[0]), int(goal[1]))
    for name, (r, c) in (("start", s), ("goal", g)):
        if not (0 <= r < rows and 0 <= c < cols):
            raise ValueError(
                f"{name} {r, c} is out of bounds for a {rows}x{cols} grid"
            )
        if arr[r, c]:
            raise ValueError(f"{name} {r, c} is on an obstacle")

    return _astar_cpp(arr, s, g, heuristic, bool(diagonal), bool(return_history))
