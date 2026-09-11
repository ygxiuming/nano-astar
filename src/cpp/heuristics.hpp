// heuristics.hpp — inline heuristic functions for grid A*.
//
// Two flavours of every heuristic are provided:
//   *_i  — integer, scaled by kScale, used by the bucket-queue engine on
//          4-connected grids where every edge costs exactly kScale.
//   *_d  — exact double, used by the 4-ary-heap engine (8-connected grids
//          have sqrt(2) diagonal edges and need exact float priorities).
//
// All four heuristics are admissible for their respective connectivity:
//   manhattan — optimal for 4-connected unit grids
//   octile    — optimal for 8-connected grids with sqrt(2) diagonals
//   euclidean — admissible for both, weaker than octile on 8-connected grids
//   diagonal  — max(|dx|, |dy|); admissible but weak (use octile instead)
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

namespace nanoastar {

// Integer scaling factor. 4-connected edges cost exactly kScale, so scaled
// g-values stay integral and the bucket queue ordering is exact.
inline constexpr int32_t kScale = 1024;
// floor(kScale * (sqrt(2) - 1)) — keeps scaled octile admissible.
inline constexpr int32_t kOctileFrac = 424;

enum class Heuristic : int { Manhattan = 0, Octile, Euclidean, Diagonal };

// Parse a heuristic name. Returns false on unknown names.
inline bool parse_heuristic(const char* name, Heuristic& out) {
  if (std::strcmp(name, "manhattan") == 0) { out = Heuristic::Manhattan; return true; }
  if (std::strcmp(name, "octile") == 0)    { out = Heuristic::Octile;    return true; }
  if (std::strcmp(name, "euclidean") == 0) { out = Heuristic::Euclidean; return true; }
  if (std::strcmp(name, "diagonal") == 0)  { out = Heuristic::Diagonal;  return true; }
  return false;
}

inline int32_t h_manhattan_i(int32_t dx, int32_t dy) {
  return (dx + dy) * kScale;
}

inline int32_t h_octile_i(int32_t dx, int32_t dy) {
  const int32_t hi = dx > dy ? dx : dy;
  const int32_t lo = dx > dy ? dy : dx;
  return hi * kScale + lo * kOctileFrac;
}

inline int32_t h_euclidean_i(int32_t dx, int32_t dy) {
  // floor of the scaled euclidean distance; stays admissible.
  return static_cast<int32_t>(
      static_cast<double>(kScale) *
      std::sqrt(static_cast<double>(dx) * dx + static_cast<double>(dy) * dy));
}

inline int32_t h_diagonal_i(int32_t dx, int32_t dy) {
  return (dx > dy ? dx : dy) * kScale;
}

inline int32_t h_int(Heuristic h, int32_t dx, int32_t dy) {
  switch (h) {
    case Heuristic::Manhattan: return h_manhattan_i(dx, dy);
    case Heuristic::Octile:    return h_octile_i(dx, dy);
    case Heuristic::Euclidean: return h_euclidean_i(dx, dy);
    case Heuristic::Diagonal:  return h_diagonal_i(dx, dy);
  }
  return 0;  // unreachable
}

inline double h_double(Heuristic h, double dx, double dy) {
  const double ax = dx < 0 ? -dx : dx;
  const double ay = dy < 0 ? -dy : dy;
  switch (h) {
    case Heuristic::Manhattan: return ax + ay;
    case Heuristic::Octile: {
      const double hi = ax > ay ? ax : ay;
      const double lo = ax > ay ? ay : ax;
      return hi + 0.4142135623730950488 * lo;  // sqrt(2) - 1
    }
    case Heuristic::Euclidean: return std::sqrt(ax * ax + ay * ay);
    case Heuristic::Diagonal:  return ax > ay ? ax : ay;
  }
  return 0.0;  // unreachable
}

}  // namespace nanoastar
