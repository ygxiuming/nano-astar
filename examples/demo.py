"""Minimal terminal demo: ASCII rendering of a nano-astar path.

Run:  python examples/demo.py
"""

from __future__ import annotations

import numpy as np

from nano_astar import astar


def main() -> None:
    rng = np.random.default_rng(3)
    grid = (rng.random((25, 60)) < 0.25).astype(np.uint8)
    start, goal = (0, 0), (24, 59)
    grid[start] = 0
    grid[goal] = 0

    result = astar(grid, start, goal, heuristic="octile", diagonal=True)
    if result is None:
        print("no path; try another seed")
        return
    path, cost = result

    canvas = np.full(grid.shape, " ", dtype="<U1")
    canvas[grid == 1] = "#"
    for r, c in path[1:-1]:
        canvas[r, c] = "*"
    canvas[start] = "S"
    canvas[goal] = "G"

    print(f"25x60 grid, 25% obstacles, 8-connected — cost {cost:.3f}, "
          f"{len(path)} cells\n")
    for row in canvas:
        print("".join(row))


if __name__ == "__main__":
    main()
