// smoke.cpp — minimal standalone check of the C++ engine (no Python).
// Verifies a 10x10 empty grid, (0,0) -> (9,9):
//   4-connected manhattan path: cost exactly 18 (integer engine)
//   8-connected octile path:    cost exactly 9*sqrt(2) (float engine)
// Build (MSVC):  cl /std:c++17 /O2 /EHsc /I src/cpp tools/smoke.cpp
#include <cmath>
#include <cstdio>
#include <vector>

#define NANO_ASTAR_STANDALONE
#include "../src/cpp/astar.cpp"

int main() {
  const int32_t R = 10, C = 10;
  std::vector<uint8_t> occ(R * C, 0);
  nanoastar::Grid g{occ.data(), R, C};

  const auto r4 = nanoastar::astar(g, 0, 0, 9, 9,
                                   nanoastar::Heuristic::Manhattan,
                                   /*diagonal=*/false, /*record_history=*/true,
                                   /*force_heap=*/false);
  const auto r8 = nanoastar::astar(g, 0, 0, 9, 9, nanoastar::Heuristic::Octile,
                                   /*diagonal=*/true, /*record_history=*/true,
                                   /*force_heap=*/false);
  // Asymmetric case: rectangle + asymmetric endpoints catch transposed
  // row/col handling that a square grid with (0,0)->(9,9) would hide.
  const int32_t R2 = 4, C2 = 6;
  std::vector<uint8_t> occ2(R2 * C2, 0);
  nanoastar::Grid g2{occ2.data(), R2, C2};
  const auto ra = nanoastar::astar(g2, 3, 0, 0, 5,
                                   nanoastar::Heuristic::Manhattan,
                                   /*diagonal=*/false, false, false);

  const double expect8 = 9.0 * 1.4142135623730950488;
  int fails = 0;
  auto check = [&](bool ok, const char* what) {
    std::printf("%-44s %s\n", what, ok ? "OK" : "FAIL");
    if (!ok) ++fails;
  };

  check(r4.found, "4-conn path found");
  check(r4.cost == 18.0, "4-conn cost == 18 (exact integer)");
  check(r4.path.size() / 2 == 19, "4-conn path length == 19 cells");
  check(!r4.history.empty(), "4-conn history recorded");

  check(r8.found, "8-conn path found");
  check(std::fabs(r8.cost - expect8) < 1e-12, "8-conn cost == 9*sqrt(2)");
  check(r8.path.size() / 2 == 10, "8-conn path length == 10 cells");
  check(!r8.history.empty(), "8-conn history recorded");

  check(ra.found && ra.cost == 8.0, "asym path found, cost == 8");
  check(ra.path.front() == 3 && ra.path[1] == 0, "asym path starts at (3,0)");
  check(ra.path[ra.path.size() - 2] == 0 && ra.path.back() == 5,
        "asym path ends at (0,5)");

  // Blocked grid must report "not found".
  for (int32_t r = 0; r < R; ++r) occ[r * C + 5] = 1;  // wall at col 5
  const auto rb = nanoastar::astar(g, 0, 0, 9, 9, nanoastar::Heuristic::Octile,
                                   true, false, false);
  check(!rb.found, "walled grid reports unreachable");

  std::printf(fails ? "SMOKE FAILED (%d)\n" : "SMOKE PASSED\n", fails);
  return fails ? 1 : 0;
}
