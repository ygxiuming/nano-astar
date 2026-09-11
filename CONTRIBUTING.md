# Contributing

Thanks for your interest! nano-astar is intentionally small; contributions
that keep it small, fast and honest are welcome.

## Setup

```bash
git clone <repo-url> && cd nano-astar
uv venv --python 3.11 .venv        # or: python -m venv .venv
uv pip install --python .venv -e ".[dev]"
```

A C++17 compiler is required (MSVC on Windows, clang/gcc elsewhere). CMake
and ninja are pulled in automatically as build requirements.

## Ground rules

1. **Tests before benchmarks before README claims.** Never write a
   performance number into the README that does not come from
   `BENCH.md` on the same commit.
2. **Benchmarks must be fair.** Same grids, same adjacency, warmup and
   repetitions for both sides; report graph-construction time separately.
3. **No Python-object access while the GIL is released.** The binding layer
   extracts plain C++ data first and re-acquires the GIL before building
   return values.
4. **No new runtime dependencies** beyond numpy. Dev-only dependencies go in
   the `dev` extra.
5. **The closed set stays a flat bitmap.** No `unordered_set`/hash maps in
   the hot path.

## Checks before sending a PR

```bash
# C++ smoke test — Windows (MSVC):
cl /std:c++17 /O2 /EHsc /I src/cpp tools/smoke.cpp /Fe:build\smoke.exe
build\smoke.exe
# Linux / macOS:
g++ -std=c++17 -O2 -Isrc/cpp tools/smoke.cpp -o /tmp/smoke && /tmp/smoke

pytest tests/ -v                  # must be fully green
python tests/bench_vs_networkx.py # if you touched the engine hot path
```

If your change affects performance, regenerate `BENCH.md` and the charts
(`python tools/make_plots.py`) and include the diff in your PR.

## Releasing

Releases are automated (`.github/workflows/release.yml`):

1. Bump `version` in `pyproject.toml`, update `CHANGELOG.md`, merge to main.
2. `git tag vX.Y.Z && git push origin vX.Y.Z`
3. CI builds the sdist + wheels (Linux/macOS/Windows via cibuildwheel),
   publishes to PyPI (trusted publishing, `pypi` environment), and creates a
   GitHub Release whose notes are auto-generated from PR labels
   (`.github/release.yml`).

Maintainer one-time setup: on PyPI, add a *pending publisher* for this repo
(workflow `release.yml`, environment `pypi`). No API token needed.

## Reporting issues

Please include: OS, Python version, compiler, a minimal reproducer, and the
output of `pip show nano-astar`.
