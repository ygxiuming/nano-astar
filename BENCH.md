# BENCH.md — nano-astar vs networkx

Environment: Python 3.11.15, networkx 3.6.1, numpy 2.4.6, nano-astar 0.1.0, MSVC 19.51 (cl /O2), Windows-10-10.0.26200-SP0

Methodology: random grids, 30% obstacles, 8-connected moves (sqrt(2)
diagonals, no corner cutting), octile heuristic on both sides. All
numbers are medians of repeated runs — networkx search: 100x100: 5, 500x500: 3, 1000x1000: 2 reps; nano-astar: 100x100: 50, 500x500: 30, 1000x1000: 20 reps — after warmup on
both sides. Different sizes use different seeds (first seed yielding a
solvable grid); within one size, both engines see the exact same grid.
The networkx graph build uses the fast vectorized construction in
`tests/bench_vs_networkx.py` (same adjacency rules), timed separately
and also included in the total column.

## 8-connected grids (octile heuristic)

| grid | seed | nano-astar (ms) | networkx search-only (ms) | networkx build+search (ms) | speedup (search only) | speedup (incl. build) | cost match |
|---|---|---|---|---|---|---|---|
| 100x100 | 0 | 0.66 | 9.3 | 40.2 | 14x | 61x | yes |
| 500x500 | 1 | 24.37 | 360.0 | 1273.4 | 15x | 52x | yes |
| 1000x1000 | 1 | 115.91 | 2030.3 | 7304.9 | 18x | 63x | yes |

## 4-connected integer grids (manhattan heuristic, bucket queue)

| grid | nano-astar (ms) | networkx search-only (ms) | networkx build+search (ms) | speedup (search only) | speedup (incl. build) | cost match |
|---|---|---|---|---|---|---|
| 100x100 | 0.07 | 3.1 | 289.2 | 43x | 4037x | yes |
| 500x500 | 4.02 | 109.3 | 718.4 | 27x | 179x | yes |
| 1000x1000 | 7.65 | 124.3 | 3546.6 | 16x | 464x | yes |

## Bucket queue vs 4-ary heap (integer 4-connected grids, manhattan)

| grid | bucket queue (ms) | 4-ary heap (ms) | heap/bucket |
|---|---|---|---|
| 100x100 | 0.07 | 0.17 | 2.36x |
| 500x500 | 4.25 | 5.48 | 1.29x |
| 1000x1000 | 7.56 | 10.60 | 1.40x |

Reproduce: `python tests/bench_vs_networkx.py`
