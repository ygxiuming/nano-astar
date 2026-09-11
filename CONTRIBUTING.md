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
cl /std:c++17 /O2 /EHsc /I src/cpp tools/smoke.cpp   # C++ smoke test
build\smoke.exe
pytest tests/ -v                  # must be fully green
python tests/bench_vs_networkx.py # if you touched the engine hot path
```

If your change affects performance, regenerate `BENCH.md` and the charts
(`python tools/make_plots.py`) and include the diff in your PR.

## Reporting issues

Please include: OS, Python version, compiler, a minimal reproducer, and the
output of `pip show nano-astar`.
