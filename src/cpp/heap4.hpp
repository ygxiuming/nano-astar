// heap4.hpp — 4-ary min-heap with O(log n) decrease-key.
//
// Used for float/general priorities (8-connected grids have sqrt(2) edges).
// A 4-ary heap has shallower trees than a binary heap, so sift-up touches
// fewer cache lines; this is the standard choice for Dijkstra/A* engines.
// Each node occupies at most one heap slot: pos_ maps node -> slot so a
// repeated push with a smaller key is a true decrease-key (no lazy
// duplicates, no stale-entry pops).
#pragma once

#include <cstdint>
#include <vector>

namespace nanoastar {

class Heap4 {
 public:
  explicit Heap4(int32_t capacity)
      : key_(capacity, 0.0), pos_(capacity, kAbsent) {
    heap_.reserve(1024);
  }

  bool empty() const { return heap_.empty(); }

  // Insert node with priority `prio`, or decrease its key if already queued.
  void push(int32_t node, double prio) {
    int32_t slot = pos_[node];
    if (slot != kAbsent) {
      if (prio >= key_[node]) return;  // not an improvement
      key_[node] = prio;
      sift_up(slot);
      return;
    }
    key_[node] = prio;
    slot = static_cast<int32_t>(heap_.size());
    heap_.push_back(node);
    pos_[node] = slot;
    sift_up(slot);
  }

  // Remove and return the node with the smallest priority.
  int32_t pop() {
    const int32_t top = heap_[0];
    const int32_t last = heap_.back();
    heap_.pop_back();
    pos_[top] = kAbsent;
    if (!heap_.empty()) {
      heap_[0] = last;
      pos_[last] = 0;
      sift_down(0);
    }
    return top;
  }

 private:
  static constexpr int32_t kAbsent = -1;

  void sift_up(int32_t slot) {
    const int32_t node = heap_[slot];
    const double k = key_[node];
    while (slot > 0) {
      const int32_t parent = (slot - 1) >> 2;  // 4-ary
      if (key_[heap_[parent]] <= k) break;
      heap_[slot] = heap_[parent];
      pos_[heap_[slot]] = slot;
      slot = parent;
    }
    heap_[slot] = node;
    pos_[node] = slot;
  }

  void sift_down(int32_t slot) {
    const int32_t node = heap_[slot];
    const double k = key_[node];
    const int32_t n = static_cast<int32_t>(heap_.size());
    for (;;) {
      int32_t child = 4 * slot + 1;
      if (child >= n) break;
      // pick smallest of up to 4 children
      double best = key_[heap_[child]];
      const int32_t last_child = child + 4 < n ? child + 4 : n;
      for (int32_t c = child + 1; c < last_child; ++c) {
        if (key_[heap_[c]] < best) {
          best = key_[heap_[c]];
          child = c;
        }
      }
      if (k <= best) break;
      heap_[slot] = heap_[child];
      pos_[heap_[slot]] = slot;
      slot = child;
    }
    heap_[slot] = node;
    pos_[node] = slot;
  }

  std::vector<double> key_;
  std::vector<int32_t> pos_;
  std::vector<int32_t> heap_;
};

}  // namespace nanoastar
