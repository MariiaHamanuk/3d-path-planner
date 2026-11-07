#pragma once

#include "core/communication.hpp"
#include "core/config.hpp"
#include "mapping/map_view.hpp"
#include "core/types.hpp"

#include <Eigen/Core>

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

namespace path3d {

/**
 * @brief A private copy of the map for one reader (planner), kept current from the mapper's
 * updates. The copy changes only inside sync(), which the owner calls from its own thread,
 * so between two sync() calls the map can be queried with no locks and never changes under
 * the reader. Updates must reach the copy in order. A diff applies only on top of the
 * version it was made from; otherwise (an update was lost, or the client joined late) the
 * copy is out of date and waits for the next keyframe, which holds the whole map. Create the
 * client before the mapper starts publishing so it does not have to wait for one. Not
 * thread-safe: use one client per thread; do not share it between planners. Debug builds
 * check this: sync() asserts that it is always called from the thread that called it first.
 */
template <ReplicatedMap M>
class MapClient {
 public:
  using Update = typename M::Update;

  /**
   * @param updates Topic the mapper publishes its updates on.
   * @param config Client settings (see MapClientConfig).
   */
  explicit MapClient(Topic<Update>& updates, const MapClientConfig& config = {})
      : m_queue(updates.subscribe_queue(config.queue_capacity)) {}

  /// Applies every update received since the last call. Returns ready().
  bool sync() {
    assert(called_from_owner() && "MapClient::sync() called from two threads: use one client each");
    std::vector<Msg<Update>> pending;
    while (Msg<Update> update = m_queue->pop(std::chrono::milliseconds::zero())) pending.push_back(std::move(update));
    /// Everything before the newest keyframe is superseded by it.
    auto first = pending.begin();
    for (auto it = pending.begin(); it != pending.end(); ++it) {
      if ((*it)->keyframe) first = it;
    }
    for (auto it = first; it != pending.end(); ++it) apply(**it);
    return m_synced;
  }

  /// True once the copy holds a full map and has not lost an update since.
  bool ready() const { return m_synced; }

  /// Meaningful once ready(); changes only in sync().
  const M& map() const { return m_map; }

  std::size_t applied_count() const { return m_applied_count; }
  std::size_t gap_count() const { return m_gap_count; }

 private:
  /// Applies one update if it fits the copy; otherwise marks the copy out of date until the next keyframe.
  void apply(const Update& update) {
    if (update.keyframe || (m_synced && update.from_version == m_map.version)) {
      m_map.apply(update);
      /// Set here, not left to each map's apply(): the gap check depends on them.
      m_map.stamp = update.stamp;
      m_map.version = update.to_version;
      m_synced = true;
      ++m_applied_count;
      return;
    }
    /// not a gap while still waiting for the first keyframe
    if (m_synced) ++m_gap_count;
    m_synced = false;
  }

  /// The first caller becomes the owner; true if the caller is the owner.
  bool called_from_owner() {
    std::thread::id expected{};
    const std::thread::id self = std::this_thread::get_id();
    return m_owner.compare_exchange_strong(expected, self) || expected == self;
  }

  std::shared_ptr<BoundedQueue<Update>> m_queue;
  std::atomic<std::thread::id> m_owner{};
  M m_map;
  /// True while the copy holds a full map and has not lost an update since.
  bool m_synced = false;
  std::size_t m_applied_count = 0;
  std::size_t m_gap_count = 0;
};

/**
 * @brief Drone access for planners: the newest pose, from the pose topic (published at the pose
 * rate, independent of the camera). Requests go up through the pipe, not through here.
 */
class UavClient {
 public:
  /**
   * @param poses Topic the drone pose is published on.
   */
  explicit UavClient(Topic<StampedPose>& poses) : m_poses(poses.subscribe_latest()) {}

  /// Newest world-frame pose with its stamp (sim time), or nullopt before the first pose. Compare
  /// the stamp with other stamps to judge its age.
  std::optional<StampedPose> pose() const {
    const Msg<StampedPose> pose = m_poses->latest();
    if (!pose) return std::nullopt;
    return *pose;
  }

  /// World-frame position from pose(), or nullopt before the first pose.
  std::optional<Eigen::Vector3d> position() const {
    const auto stamped = pose();
    if (!stamped) return std::nullopt;
    return stamped->pose.position;
  }

 private:
  std::shared_ptr<LatestSlot<StampedPose>> m_poses;
};

}
