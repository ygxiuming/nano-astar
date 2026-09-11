"""Generate the comparison charts shipped in the README:

  docs/benchmark.png   — nano-astar vs networkx timings (from
                         docs/bench_results.json, written by
                         tests/bench_vs_networkx.py) plus the internal
                         bucket-queue vs 4-ary-heap comparison.
  docs/heuristics.png  — explored-region comparison of the four built-in
                         heuristics on one maze (4-connected).

Run after tests/bench_vs_networkx.py:  python tools/make_plots.py
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from make_gif import make_maze
from nano_astar import astar

REPO_ROOT = Path(__file__).resolve().parent.parent
DOCS = REPO_ROOT / "docs"

C_OURS = "#d62728"
C_NX = "#7f7f7f"
C_NX_TOTAL = "#c7c7c7"
C_BUCKET = "#1f77b4"
C_HEAP = "#ff7f0e"


def bench_chart() -> None:
    data = json.loads((DOCS / "bench_results.json").read_text())
    main = data["main"]
    sizes = [f"{r['size']}²" for r in main]
    ours = [r["ours"] for r in main]
    nx_search = [r["nx_search"] for r in main]
    nx_total = [r["nx_total"] for r in main]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2),
                                   gridspec_kw={"width_ratios": [3, 2]})

    x = np.arange(len(sizes))
    w = 0.27
    b1 = ax1.bar(x - w, nx_total, w, label="networkx (build + search)",
                 color=C_NX_TOTAL, edgecolor="white")
    b2 = ax1.bar(x, nx_search, w, label="networkx (search only)",
                 color=C_NX, edgecolor="white")
    b3 = ax1.bar(x + w, ours, w, label="nano-astar", color=C_OURS,
                 edgecolor="white")
    ax1.set_yscale("log")
    ax1.set_xticks(x, sizes)
    ax1.set_ylabel("median wall time (ms, log scale)")
    ax1.set_xlabel("grid size, 30% obstacles, 8-connected")
    ax1.set_title("A* on random occupancy grids — lower is better")
    ax1.legend(frameon=False, loc="upper left")
    for bars in (b1, b2, b3):
        for b in bars:
            ax1.annotate(f"{b.get_height():.1f}",
                         (b.get_x() + b.get_width() / 2, b.get_height()),
                         textcoords="offset points", xytext=(0, 2),
                         ha="center", fontsize=8)
    for r, xi in zip(main, x):
        ax1.annotate(f"{r['speedup_total']:.0f}x",
                     (xi + w, r["ours"]), textcoords="offset points",
                     xytext=(0, 14), ha="center", fontsize=9,
                     fontweight="bold", color=C_OURS)

    bq = data["bucket_vs_heap"]
    sizes2 = [f"{r['size']}²" for r in bq]
    bucket = [r["bucket"] for r in bq]
    heap = [r["heap"] for r in bq]
    x2 = np.arange(len(sizes2))
    b4 = ax2.bar(x2 - 0.18, bucket, 0.36, label="bucket queue (integer)",
                 color=C_BUCKET, edgecolor="white")
    b5 = ax2.bar(x2 + 0.18, heap, 0.36, label="4-ary heap (general)",
                 color=C_HEAP, edgecolor="white")
    ax2.set_xticks(x2, sizes2)
    ax2.set_ylabel("median wall time (ms)")
    ax2.set_xlabel("4-connected grid, integer costs")
    ax2.set_title("Bucket queue vs 4-ary heap (same engine)")
    ax2.legend(frameon=False, loc="upper left")
    for bars in (b4, b5):
        for b in bars:
            ax2.annotate(f"{b.get_height():.2f}",
                         (b.get_x() + b.get_width() / 2, b.get_height()),
                         textcoords="offset points", xytext=(0, 2),
                         ha="center", fontsize=8)

    fig.tight_layout()
    fig.savefig(DOCS / "benchmark.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("wrote docs/benchmark.png")


def heuristics_chart() -> None:
    # Open field with scattered obstacles: heuristic strength actually
    # changes the explored region (a single-corridor maze would not).
    rng = np.random.default_rng(11)
    while True:
        grid = (rng.random((60, 60)) < 0.2).astype(np.uint8)
        start, goal = (0, 0), (59, 59)
        grid[start] = 0
        grid[goal] = 0
        probe = astar(grid, start, goal, diagonal=False, return_history=True)
        if probe is not None and probe[1] > 115:  # want a long detour
            break

    fig, axes = plt.subplots(1, 4, figsize=(14, 3.9))
    for ax, h in zip(axes, ("manhattan", "octile", "euclidean", "diagonal")):
        path, cost, history = astar(grid, start, goal, heuristic=h,
                                    diagonal=False, return_history=True)
        rgb = np.ones((*grid.shape, 3))
        rgb[grid == 1] = 0.1
        rgb[history[:, 0], history[:, 1]] = (0.55, 0.75, 0.9)
        rgb[path[:, 0], path[:, 1]] = (0.85, 0.15, 0.15)
        ax.imshow(rgb, interpolation="nearest")
        ax.set_title(f"{h}: explored {len(history)} cells", fontsize=11)
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(DOCS / "heuristics.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("wrote docs/heuristics.png")


if __name__ == "__main__":
    bench_chart()
    heuristics_chart()
