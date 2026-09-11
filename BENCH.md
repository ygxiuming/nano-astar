# BENCH.md — nano-astar vs networkx

Environment: Python 3.11.15, networkx 3.6.1, numpy 2.4.6, nano-astar 0.1.0, MSVC 19.51 (cl /O2), Windows-10-10.0.26200-SP0

Random grids, 30% obstacles, 8-connected moves (sqrt(2) diagonals, no
corner cutting), octile heuristic on both sides. Median wall times;
both sides warmed up. networkx build uses the fast vectorized
construction in `tests/bench_vs_networkx.py` (same adjacency rules).

| grid | seed | nano-astar (ms) | networkx search-only (ms) | networkx build+search (ms) | speedup (search only) | speedup (incl. build) | cost match |
|---|---|---|---|---|---|---|---|
| 100x100 | 0 | 0.65 | 9.4 | 40.6 | 14x | 62x | yes |
| 500x500 | 1 | 26.15 | 392.6 | 1317.2 | 15x | 50x | yes |
| 1000x1000 | 1 | 135.88 | 2379.2 | 7870.6 | 18x | 58x | yes |

## Bucket queue vs 4-ary heap (integer 4-connected grids, manhattan)

| grid | bucket queue (ms) | 4-ary heap (ms) | heap/bucket |
|---|---|---|---|
| 100x100 | 0.07 | 0.17 | 2.42x |
| 500x500 | 4.27 | 5.70 | 1.34x |
| 1000x1000 | 8.12 | 11.82 | 1.46x |

Reproduce: `python tests/bench_vs_networkx.py`
