"""Render the A* demo GIF: 60x60 maze, exploration front, final path.

Frames: obstacles black, closed cells as a light-blue gradient (by close
order), the current frontier orange, the final path red.  Output:
docs/demo.gif (<= 3 MB, <= 400 frames, final frame held for 20 frames).

Run:  python tools/make_gif.py
"""

from __future__ import annotations

import io
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from nano_astar import astar

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT = REPO_ROOT / "docs" / "demo.gif"

SIZE = 60
EXPLORE_FRAMES = 355      # + ~15 path-growth + 20 hold frames <= 400 total
PATH_FRAMES = 15
HOLD_FRAMES = 20
DPI = 100
FIGSIZE = 3.6             # 360x360 px output

# palette (RGB)
FREE = np.array([255, 255, 255], np.uint8)
WALL = np.array([17, 17, 17], np.uint8)
FRONTIER = np.array([255, 127, 14], np.uint8)
PATH = np.array([214, 39, 40], np.uint8)
START = np.array([44, 160, 44], np.uint8)
GOAL = np.array([148, 103, 189], np.uint8)
CLOSED_LO = np.array([222, 235, 247], float)   # lightest blue
CLOSED_HI = np.array([107, 174, 214], float)   # deepest blue


def make_maze(size: int, seed: int = 7) -> np.ndarray:
    """Recursive-backtracker maze on the odd lattice; 1 = wall."""
    rng = np.random.default_rng(seed)
    grid = np.ones((size, size), np.uint8)
    cells_r = list(range(1, size - 1, 2))
    cells_c = list(range(1, size - 1, 2))
    start = (cells_r[0], cells_c[0])
    grid[start] = 0
    stack = [start]
    seen = {start}
    while stack:
        r, c = stack[-1]
        nbrs = []
        for dr, dc in ((-2, 0), (2, 0), (0, -2), (0, 2)):
            r2, c2 = r + dr, c + dc
            if (r2 in cells_r and c2 in cells_c) and (r2, c2) not in seen:
                nbrs.append((r2, c2))
        if not nbrs:
            stack.pop()
            continue
        r2, c2 = nbrs[rng.integers(0, len(nbrs))]
        grid[(r + r2) // 2, (c + c2) // 2] = 0
        grid[r2, c2] = 0
        seen.add((r2, c2))
        stack.append((r2, c2))
    # braid: knock down ~8% of interior walls so the maze has loops
    interior = grid[1:-1, 1:-1]
    walls = np.argwhere(interior == 1)
    drop = walls[rng.choice(len(walls), size=len(walls) // 12, replace=False)]
    interior[drop[:, 0], drop[:, 1]] = 0
    return grid


def render(rgb: np.ndarray) -> Image.Image:
    fig = plt.figure(figsize=(FIGSIZE, FIGSIZE), dpi=DPI)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(rgb, interpolation="nearest")
    ax.axis("off")
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=DPI)
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).convert("RGB")


def shared_palette() -> Image.Image:
    """Fixed palette: our exact colors + a 64-step closed-cell blue gradient,
    so every GIF frame quantizes to identical colors."""
    colors = [tuple(FREE), tuple(WALL), tuple(FRONTIER), tuple(PATH),
              tuple(START), tuple(GOAL)]
    for i in range(64):
        t = i / 63.0
        c = CLOSED_LO + (CLOSED_HI - CLOSED_LO) * t
        colors.append(tuple(int(round(v)) for v in c))
    colors += [(0, 0, 0)] * (256 - len(colors))
    pal = Image.new("P", (1, 1))
    pal.putpalette([v for c in colors for v in c])
    return pal


def main() -> None:
    grid = make_maze(SIZE)
    start, goal = (1, 1), (57, 57)
    result = astar(grid, start, goal, heuristic="octile", diagonal=True,
                   return_history=True)
    if result is None:
        raise RuntimeError("maze has no path; pick another seed")
    path, cost, history = result
    print(f"path length {len(path)}, cost {cost:.3f}, "
          f"closed {len(history)} cells")

    n_closed = len(history)
    step = max(1, -(-n_closed // EXPLORE_FRAMES))  # ceil
    checkpoints = list(range(step, n_closed + 1, step))
    if checkpoints[-1] != n_closed:
        checkpoints.append(n_closed)

    closed_mask = np.zeros((SIZE, SIZE), bool)
    closed_order = np.zeros((SIZE, SIZE), float)
    base = np.empty((SIZE, SIZE, 3), np.uint8)
    base[grid == 0] = FREE
    base[grid == 1] = WALL

    frames = []
    hist_r = history[:, 0]
    hist_c = history[:, 1]
    prev = 0
    for k in checkpoints:
        # accumulate newly closed cells with a gradient by close order
        closed_mask[hist_r[prev:k], hist_c[prev:k]] = True
        closed_order[hist_r[prev:k], hist_c[prev:k]] = np.arange(prev, k)
        prev = k

        rgb = base.copy()
        t = (closed_order[closed_mask] / max(1, n_closed - 1))[:, None]
        rgb[closed_mask] = (CLOSED_LO + (CLOSED_HI - CLOSED_LO) * t).astype(
            np.uint8)
        # frontier: free cells adjacent (8-conn) to the closed region
        if k < n_closed:
            frontier = np.zeros((SIZE, SIZE), bool)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    shifted = np.roll(closed_mask, (dr, dc), axis=(0, 1))
                    frontier |= shifted
            frontier &= ~closed_mask & (grid == 0)
            rgb[frontier] = FRONTIER
        rgb[start] = START
        rgb[goal] = GOAL
        frames.append(render(rgb))
        if k == n_closed:
            final_cells = rgb  # keep the 60x60 cell array for path overlay

    # path growth animation, then hold the last frame
    n = len(path)
    for f in range(PATH_FRAMES):
        upto = (f + 1) * n // PATH_FRAMES
        arr = final_cells.copy()
        arr[path[:upto, 0], path[:upto, 1]] = PATH
        arr[start] = START
        arr[goal] = GOAL
        frames.append(render(arr))
    frames.extend([frames[-1]] * HOLD_FRAMES)

    pal = shared_palette()
    qframes = [f.quantize(palette=pal, dither=Image.Dither.NONE)
               for f in frames]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    qframes[0].save(OUT, save_all=True, append_images=qframes[1:],
                    duration=40, loop=0, optimize=False, disposal=0)
    size_mb = OUT.stat().st_size / 1e6
    print(f"{len(frames)} frames -> {OUT} ({size_mb:.2f} MB)")
    if len(frames) > 400:
        raise RuntimeError("frame budget exceeded")
    if size_mb > 3.0:
        raise RuntimeError("GIF exceeds 3 MB")


if __name__ == "__main__":
    main()
