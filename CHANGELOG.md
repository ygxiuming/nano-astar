# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `tools/release.py`: one-command release script — stamps the Unreleased
  changelog section, bumps `pyproject.toml` and `__init__.py`, commits, tags,
  and pushes (`python tools/release.py X.Y.Z`, supports `--dry-run`).

## [0.1.1] - 2026-09-12

### Fixed

- README images (demo GIF, benchmark and heuristic charts) now use absolute
  URLs, so they render on the PyPI project page as well as on GitHub.

### Changed

- CI and release pipelines build x86_64 Linux wheels only (the 32-bit i686
  build had no installable nanobind/numpy dependencies); musllinux is skipped.
- Release pipeline publishes to PyPI via the `PYPI_API_TOKEN` repository
  secret with `skip-existing`, and GitHub Release notes are assembled from
  this changelog plus auto-categorized PR notes.
- Repository topics added for discoverability (a-star, pathfinding, astar,
  nanobind, numpy, cpp, motion-planning, networkx, python-bindings).

## [0.1.0] - 2026-09-11

### Added

- Initial release.
- C++17 A* engine over flat, cache-friendly arrays; closed set is a
  1-byte-per-cell bitmap, no hash maps.
- Dual open list: ring bucket queue with O(1) amortized push/pop and true
  decrease-key for 4-connected integer grids; 4-ary heap for float costs
  (8-connected `sqrt(2)` diagonals) — selected automatically.
- Four inline C++ heuristics: `octile`, `manhattan`, `euclidean`, `diagonal`.
- nanobind bindings; GIL released for the entire search
  (`nb::call_guard<nb::gil_scoped_release>`), no Python objects touched
  while released.
- Zero-copy read of contiguous `uint8` occupancy grids; other dtypes are
  converted once.
- `astar(grid, start, goal, heuristic, diagonal, return_history)` returning
  `(path, cost)` or `(path, cost, history)`; `None` when unreachable,
  `ValueError` for invalid endpoints.
- 200-case randomized differential test suite against networkx, heuristic
  optimality tests vs Dijkstra, 4-thread GIL-release test.
- Reproducible benchmark (`tests/bench_vs_networkx.py` → `BENCH.md`),
  comparison charts (`tools/make_plots.py`), demo GIF
  (`tools/make_gif.py`).
- CI: Linux/macOS/Windows × CPython 3.9–3.12, plus cibuildwheel wheels.
