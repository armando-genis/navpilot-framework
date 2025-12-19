#pragma once
#include <deque>
#include <optional>
#include <vector>
#include <cstdint>
#include <mutex>
#include <algorithm>

namespace v4l2_camera
{

template<typename T>
struct Stamped
{
  uint64_t stamp_ns;
  T data;
};

template<typename T>
struct SyncedSet
{
  uint64_t stamp_ns_ref;
  std::vector<Stamped<T>> frames; // size == N
};

template<typename T>
class MultiCamSynchronizer
{
public:
  MultiCamSynchronizer(size_t num_cams, uint64_t tolerance_ns, size_t max_queue = 10)
  : bufs_(num_cams), tol_(tolerance_ns), maxq_(max_queue) {}

  void push(size_t cam_id, uint64_t stamp_ns, T frame)
  {
    std::lock_guard<std::mutex> lk(m_);
    auto & q = bufs_.at(cam_id);
    q.push_back({stamp_ns, std::move(frame)});
    while (q.size() > maxq_) q.pop_front();
  }

  std::optional<SyncedSet<T>> try_pop()
  {
    std::lock_guard<std::mutex> lk(m_);
    for (auto &q : bufs_) if (q.empty()) return std::nullopt;

    // reference timestamp = latest among the oldest frames
    uint64_t ref = 0;
    for (auto &q : bufs_) ref = std::max(ref, q.front().stamp_ns);

    std::vector<size_t> idx(bufs_.size(), 0);
    uint64_t min_ts = UINT64_MAX, max_ts = 0;

    for (size_t cam = 0; cam < bufs_.size(); ++cam) {
      auto &q = bufs_[cam];

      size_t k = 0;
      while (k + 1 < q.size() && q[k].stamp_ns < ref) ++k;

      size_t best = k;
      if (k > 0) {
        auto d0 = (ref > q[k-1].stamp_ns) ? (ref - q[k-1].stamp_ns) : (q[k-1].stamp_ns - ref);
        auto d1 = (ref > q[k].stamp_ns)   ? (ref - q[k].stamp_ns)   : (q[k].stamp_ns   - ref);
        best = (d0 <= d1) ? (k - 1) : k;
      }

      idx[cam] = best;
      min_ts = std::min(min_ts, q[best].stamp_ns);
      max_ts = std::max(max_ts, q[best].stamp_ns);
    }

    if (max_ts - min_ts > tol_) {
      // drop oldest front frame to avoid deadlock
      size_t oldest_cam = 0;
      for (size_t cam = 1; cam < bufs_.size(); ++cam) {
        if (bufs_[cam].front().stamp_ns < bufs_[oldest_cam].front().stamp_ns) {
          oldest_cam = cam;
        }
      }
      bufs_[oldest_cam].pop_front();
      return std::nullopt;
    }

    SyncedSet<T> out;
    out.stamp_ns_ref = ref;
    out.frames.reserve(bufs_.size());

    for (size_t cam = 0; cam < bufs_.size(); ++cam) {
      auto &q = bufs_[cam];
      for (size_t drop = 0; drop < idx[cam]; ++drop) q.pop_front();
      out.frames.push_back(std::move(q.front()));
      q.pop_front();
    }
    return out;
  }

private:
  std::vector<std::deque<Stamped<T>>> bufs_;
  uint64_t tol_;
  size_t maxq_;
  std::mutex m_;
};

} // namespace v4l2_camera
