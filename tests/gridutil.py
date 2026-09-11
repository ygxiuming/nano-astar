"""Shared helpers for nano-astar tests and benchmarks."""

from __future__ import annotations

import math

import networkx as nx
import numpy as np

SQRT2 = math.sqrt(2.0)


def make_grid(rng: np.random.Generator, size: int, density: float) -> np.ndarray:
    """Random 0/1 occupancy grid, uint8, C-contiguous."""
    return (rng.random((size, size)) < density).astype(np.uint8)


def grid_to_nx(grid: np.ndarray, diagonal: bool) -> nx.Graph:
    """Build a networkx graph with the exact same adjacency rules as
    nano-astar: 4- or 8-connected, unit/sqrt(2) weights, no corner cutting."""
    rows, cols = grid.shape
    G = nx.Graph()
    free = zip(*np.nonzero(grid == 0))
    G.add_nodes_from((int(r), int(c)) for r, c in free)
    for r in range(rows):
        for c in range(cols):
            if grid[r, c]:
                continue
            # right and down orthogonal neighbours
            for dr, dc in ((0, 1), (1, 0)):
                r2, c2 = r + dr, c + dc
                if r2 < rows and c2 < cols and not grid[r2, c2]:
                    G.add_edge((r, c), (r2, c2), weight=1.0)
            if diagonal:
                for dr, dc in ((1, 1), (1, -1)):
                    r2, c2 = r + dr, c + dc
                    if (
                        r2 < rows
                        and 0 <= c2 < cols
                        and not grid[r2, c2]
                        and not grid[r, c2]  # no corner cutting
                        and not grid[r2, c]
                    ):
                        G.add_edge((r, c), (r2, c2), weight=SQRT2)
    return G


def nx_heuristic(diagonal: bool):
    """Admissible heuristic matching nano-astar's defaults."""

    def h4(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def h8(a, b):
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return max(dx, dy) + (SQRT2 - 1.0) * min(dx, dy)

    return h8 if diagonal else h4


def nx_solve(G: nx.Graph, start, goal, diagonal: bool):
    """A* via networkx. Returns cost or None if unreachable/invalid."""
    if start not in G or goal not in G:
        return None
    try:
        path = nx.astar_path(G, start, goal, heuristic=nx_heuristic(diagonal),
                             weight="weight")
    except nx.NetworkXNoPath:
        return None
    cost = 0.0
    for u, v in zip(path[:-1], path[1:]):
        cost += G[u][v]["weight"]
    return cost


def nx_dijkstra_cost(G: nx.Graph, start, goal):
    """Ground-truth optimal cost via Dijkstra (no heuristic)."""
    if start not in G or goal not in G:
        return None
    try:
        return nx.dijkstra_path_length(G, start, goal, weight="weight")
    except nx.NetworkXNoPath:
        return None
