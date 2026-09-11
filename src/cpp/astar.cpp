// astar.cpp — A* engine and nanobind bindings for nano-astar.
//
// The engine is a single header-style template over the cost type and the
// open-list implementation, instantiated exactly twice:
//   * int32_t  + BucketQueue — 4-connected grids: every edge costs kScale,
//     priorities stay integral, bucket queue pops in O(1) amortized.
//   * double   + Heap4       — 8-connected grids (sqrt(2) diagonal edges) or
//     an explicit request for the general path.
//
// Memory layout is flat and cache-friendly: one byte per cell for the closed
// bitmap, one int32 per cell for parent links, one cost per cell for g.  No
// hash maps anywhere.
//
// Compile with -DNANO_ASTAR_STANDALONE to skip the nanobind section (used by
// tools/smoke.cpp, which #includes this file directly).

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

#include "bucket_queue.hpp"
#include "heap4.hpp"
#include "heuristics.hpp"

namespace nanoastar {

// Ring size must exceed the maximum live-key spread (2 * kScale + slack).
static constexpr int32_t kRingSize = 4096;

struct Grid {
  const uint8_t* occ;  // row-major, 0 = free, nonzero = obstacle
  int32_t rows;
  int32_t cols;
};

struct Result {
  bool found = false;
  double cost = 0.0;
  std::vector<int32_t> path;     // (row, col) pairs, start -> goal
  std::vector<int32_t> history;  // closed cells in pop order, (row, col) pairs
};

// Cost-type adapters so one template serves both engines.
inline int32_t step_orth(int32_t*) { return kScale; }
inline double step_orth(double*) { return 1.0; }
inline int32_t step_diag(int32_t*) { return kScale; }  // never used on int path
inline double step_diag(double*) { return 1.4142135623730950488; }

inline int32_t heuristic_of(int32_t*, Heuristic h, int32_t dx, int32_t dy) {
  return h_int(h, dx, dy);
}
inline double heuristic_of(double*, Heuristic h, int32_t dx, int32_t dy) {
  return h_double(h, static_cast<double>(dx), static_cast<double>(dy));
}

inline double unscale(int32_t*, int32_t g) {
  return static_cast<double>(g) / static_cast<double>(kScale);
}
inline double unscale(double*, double g) { return g; }

// ---------------------------------------------------------------------------
// Engine
// ---------------------------------------------------------------------------

template <typename Cost, class Open>
Result astar_run(const Grid& grid, int32_t sr, int32_t sc, int32_t gr,
                 int32_t gc, Heuristic h, bool diagonal, bool record_history) {
  const int32_t rows = grid.rows;
  const int32_t cols = grid.cols;
  const int32_t n = rows * cols;
  const int32_t start = sr * cols + sc;
  const int32_t goal = gr * cols + gc;

  Result res;
  if (start == goal) {
    res.found = true;
    res.cost = 0.0;
    res.path = {sr, sc};
    if (record_history) res.history = {sr, sc};
    return res;
  }

  std::vector<uint8_t> closed(n, 0);
  std::vector<int32_t> parent(n, -1);
  std::vector<Cost> g_score(n, Cost(0));
  std::vector<uint8_t> touched(n, 0);  // g_score validity bitmap
  Open open(n);

  const Cost c_orth = step_orth(static_cast<Cost*>(nullptr));
  const Cost c_diag = step_diag(static_cast<Cost*>(nullptr));

  touched[start] = 1;
  g_score[start] = Cost(0);
  open.push(start, Cost(heuristic_of(static_cast<Cost*>(nullptr), h,
                                     sr > gr ? sr - gr : gr - sr,
                                     sc > gc ? sc - gc : gc - sc)));

  // Neighbour offsets: 4-connected first, diagonals appended when allowed.
  static const int32_t kDir4[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  static const int32_t kDir8[4][2] = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
  const int32_t ndiag = diagonal ? 4 : 0;

  while (!open.empty()) {
    const int32_t u = open.pop();
    if (closed[u]) continue;  // defensive; unreachable with true decrease-key
    closed[u] = 1;
    if (record_history) {
      res.history.push_back(u / cols);
      res.history.push_back(u % cols);
    }
    if (u == goal) break;

    const int32_t ur = u / cols;
    const int32_t uc = u % cols;
    const Cost gu = g_score[u];

    for (int32_t d = 0; d < 4 + ndiag; ++d) {
      const int32_t dr = d < 4 ? kDir4[d][0] : kDir8[d - 4][0];
      const int32_t dc = d < 4 ? kDir4[d][1] : kDir8[d - 4][1];
      const int32_t vr = ur + dr;
      const int32_t vc = uc + dc;
      if (static_cast<uint32_t>(vr) >= static_cast<uint32_t>(rows) ||
          static_cast<uint32_t>(vc) >= static_cast<uint32_t>(cols))
        continue;
      const int32_t v = vr * cols + vc;
      if (grid.occ[v] || closed[v]) continue;
      // No corner cutting: a diagonal step requires both orthogonal
      // neighbours to be free.
      if (d >= 4 && (grid.occ[ur * cols + vc] || grid.occ[vr * cols + uc]))
        continue;

      const Cost cand = gu + (d < 4 ? c_orth : c_diag);
      if (touched[v] && cand >= g_score[v]) continue;
      touched[v] = 1;
      g_score[v] = cand;
      parent[v] = u;
      const int32_t dx = vr > gr ? vr - gr : gr - vr;
      const int32_t dy = vc > gc ? vc - gc : gc - vc;
      open.push(v, cand + Cost(heuristic_of(static_cast<Cost*>(nullptr), h,
                                            dx, dy)));
    }
  }

  if (!closed[goal]) return res;  // unreachable

  res.found = true;
  res.cost = unscale(static_cast<Cost*>(nullptr), g_score[goal]);
  // Walk parent links backwards (goal -> start), then reverse PAIR-WISE:
  // the buffer is interleaved (row, col) scalars, so a plain reverse would
  // also flip every coordinate pair.
  std::vector<int32_t> rev;
  for (int32_t v = goal; v != -1; v = parent[v]) {
    rev.push_back(v / cols);
    rev.push_back(v % cols);
  }
  res.path.reserve(rev.size());
  for (auto it = rev.end(); it != rev.begin();) {
    it -= 2;
    res.path.push_back(it[0]);
    res.path.push_back(it[1]);
  }
  return res;
}

// Public dispatch: integer/bucket-queue path for 4-connected grids,
// float/heap path otherwise. force_heap selects the general engine on
// 4-connected grids (used by the bucket-vs-heap benchmark).
Result astar(const Grid& grid, int32_t sr, int32_t sc, int32_t gr, int32_t gc,
             Heuristic h, bool diagonal, bool record_history, bool force_heap) {
  if (!diagonal && !force_heap)
    return astar_run<int32_t, BucketQueue>(grid, sr, sc, gr, gc, h, false,
                                           record_history);
  return astar_run<double, Heap4>(grid, sr, sc, gr, gc, h, diagonal,
                                  record_history);
}

}  // namespace nanoastar

// ---------------------------------------------------------------------------
// nanobind bindings (skipped in standalone/smoke builds)
// ---------------------------------------------------------------------------
#ifndef NANO_ASTAR_STANDALONE

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace nanoastar {

using GridArray =
    nb::ndarray<const uint8_t, nb::ndim<2>, nb::c_contig, nb::device::cpu>;

// Wrap an interleaved (row, col) buffer as an (n, 2) int32 numpy array,
// transferring ownership to Python.
static nb::ndarray<nb::numpy, int32_t> pairs_to_numpy(
    const std::vector<int32_t>& v) {
  const size_t n = v.size() / 2;
  int32_t* data = new int32_t[v.size()];
  std::memcpy(data, v.data(), v.size() * sizeof(int32_t));
  nb::capsule owner(data, [](void* p) noexcept {
    delete[] static_cast<int32_t*>(p);
  });
  return nb::ndarray<nb::numpy, int32_t>(data, {n, 2}, owner);
}

static nb::object astar_py(GridArray grid, std::pair<int32_t, int32_t> start,
                           std::pair<int32_t, int32_t> goal,
                           const std::string& heuristic, bool diagonal,
                           bool return_history, bool force_heap) {
  // The call_guard has already released the GIL.  Until the re-acquire below
  // this function touches only plain C++ data: the ndarray accessors read the
  // DLPack struct in place (no Python C-API), and the grid buffer is borrowed
  // memory kept alive by the caller's argument tuple.
  const int32_t rows = static_cast<int32_t>(grid.shape(0));
  const int32_t cols = static_cast<int32_t>(grid.shape(1));
  const uint8_t* occ = grid.data();

  Heuristic h;
  if (!parse_heuristic(heuristic.c_str(), h))
    throw std::invalid_argument(
        "unknown heuristic (expected octile|manhattan|euclidean|diagonal)");

  const auto check_cell = [&](std::pair<int32_t, int32_t> rc,
                              const char* name) {
    if (rc.first < 0 || rc.first >= rows || rc.second < 0 || rc.second >= cols)
      throw std::invalid_argument(std::string(name) + " is out of bounds");
    if (occ[rc.first * cols + rc.second])
      throw std::invalid_argument(std::string(name) + " is on an obstacle");
  };
  check_cell(start, "start");
  check_cell(goal, "goal");

  Grid g{occ, rows, cols};
  Result res = astar(g, start.first, start.second, goal.first, goal.second, h,
                     diagonal, return_history, force_heap);

  nb::object out;
  {
    // Output objects are built with the GIL held; the heavy search above ran
    // entirely GIL-free.
    nb::gil_scoped_acquire gil;
    if (!res.found) {
      out = nb::none();
    } else {
      nb::object path = nb::cast(pairs_to_numpy(res.path));
      nb::object cost = nb::float_(res.cost);
      if (return_history)
        out = nb::make_tuple(path, cost, nb::cast(pairs_to_numpy(res.history)));
      else
        out = nb::make_tuple(path, cost);
    }
  }
  return out;
}

}  // namespace nanoastar

NB_MODULE(_core, m) {
  m.doc() = "nano-astar C++ core (nanobind)";
  m.def("astar", &nanoastar::astar_py, nb::arg("grid"), nb::arg("start"),
        nb::arg("goal"), nb::arg("heuristic") = "octile",
        nb::arg("diagonal") = true, nb::arg("return_history") = false,
        nb::arg("force_heap") = false,
        nb::call_guard<nb::gil_scoped_release>());
}

#endif  // NANO_ASTAR_STANDALONE
