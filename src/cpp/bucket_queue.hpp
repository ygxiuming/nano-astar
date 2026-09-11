// bucket_queue.hpp — ring bucket queue for small-integer priorities.
//
// Design notes
// ------------
// A* with a (near-)consistent heuristic pops nodes in non-decreasing f order,
// and every node pushed while the minimum key is `cur` satisfies
//     cur - 1 <= f <= cur + 2 * kMaxStep + 1
// (the -1 covers flooring in scaled-euclidean; the upper bound is the classic
//  consistency argument |h(u) - h(v)| <= c(u, v)).  Hence all live keys fit in
// a small window and a ring of kRingSize buckets suffices — no giant
// vector<deque<int>> allocation is ever needed.
//
// Each node lives in at most one bucket at a time (true decrease-key):
// links are intrusive (per-node next/prev arrays), so push() is O(1) and
// there is no stale-entry cleanup like in lazy binary-heap A*.
#pragma once

#include <cstdint>
#include <vector>

namespace nanoastar {

class BucketQueue {
 public:
  // capacity: number of nodes (grid cells). ring_size: buckets, power of two,
  // must exceed the maximum live key spread (see header comment); the default
  // covers 2 * kScale with ample slack.
  explicit BucketQueue(int32_t capacity, int32_t ring_size = 4096)
      : mask_(ring_size - 1),
        head_(ring_size, kNil),
        next_(capacity, kNil),
        prev_(capacity, kNil),
        key_(capacity, 0),
        inq_(capacity, 0) {}

  bool empty() const { return size_ == 0; }
  int32_t size() const { return size_; }
  int32_t min_key() const { return cur_; }

  // Insert node with priority `prio`, or decrease its key if already queued.
  void push(int32_t node, int32_t prio) {
    if (inq_[node]) {
      if (prio >= key_[node]) return;  // not an improvement
      unlink(node);
    } else {
      inq_[node] = 1;
      ++size_;
    }
    key_[node] = prio;
    const int32_t b = prio & mask_;
    const int32_t old_head = head_[b];
    next_[node] = old_head;
    prev_[node] = kNil;
    if (old_head != kNil) prev_[old_head] = node;
    head_[b] = node;
    if (prio < cur_) cur_ = prio;  // keys may dip by 1 (floored heuristics)
  }

  // Remove and return the node with the smallest priority.
  int32_t pop() {
    while (head_[cur_ & mask_] == kNil) ++cur_;
    const int32_t node = head_[cur_ & mask_];
    unlink(node);
    inq_[node] = 0;
    --size_;
    return node;
  }

 private:
  static constexpr int32_t kNil = -1;

  void unlink(int32_t node) {
    const int32_t p = prev_[node];
    const int32_t n = next_[node];
    if (p != kNil) next_[p] = n;
    else           head_[key_[node] & mask_] = n;
    if (n != kNil) prev_[n] = p;
  }

  const int32_t mask_;
  std::vector<int32_t> head_;
  std::vector<int32_t> next_;
  std::vector<int32_t> prev_;
  std::vector<int32_t> key_;
  std::vector<uint8_t> inq_;
  int32_t cur_ = 0;  // smallest priority currently present
  int32_t size_ = 0;
};

}  // namespace nanoastar
