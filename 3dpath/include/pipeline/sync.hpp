#pragma once

#include "core/communication.hpp"
#include "core/config.hpp"
#include "io/reader.hpp"
#include "core/types.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>

namespace path3d {

/**
 * @brief Assembles SensorFrames from the separate sensor streams: every depth image is paired with
 * the drone pose interpolated at its stamp, the newest depth calibration and the newest
 * camera mount. Depth drives the output; poses are kept in a history so the pair can be
 * found even when the pose arrives after the image. Subscribes in the constructor, so create
 * it before the Reader. next() is meant to be called from one consumer thread; close() may
 * be called from any thread.
 */
class FrameSynchronizer {
 public:
  /**
   * @param in Sensor streams to pair up.
   * @param config Queue sizes and the pose wait (see SyncConfig).
   */
  explicit FrameSynchronizer(SensorTopics& in, const SyncConfig& config = {})
      : m_depth(in.depth.subscribe_queue(config.depth_queue)),
        m_poses(in.pose.subscribe_history(config.pose_history)),
        m_calibration(in.depth_calibration.subscribe_latest()),
        m_mount(in.base_to_depth.subscribe_latest()),
        m_pose_wait(config.pose_wait) {}

  /// Next depth image with everything needed to project it. Waits up to `timeout` for an image,
  /// then up to `SyncConfig::pose_wait` for a pose at or after its stamp. Null on timeout, after
  /// close(), or when the image can't be synchronized (no calibration or mount yet, or its stamp is
  /// older than the pose history); such images are dropped and counted in unsynced_count().
  Msg<SensorFrame> next(std::chrono::milliseconds timeout) {
    const Msg<ImageFrame> depth = m_depth->pop(timeout);
    if (!depth || m_closed) return nullptr;
    Msg<SensorFrame> frame = synchronize(*depth);
    if (!frame && !m_closed) ++m_unsynced_count;
    return frame;
  }

  /// Wakes a blocked next(); later calls return null.
  void close() {
    m_closed = true;
    m_depth->close();
    m_poses->close();
  }

  /// Depth images the consumer was too slow for.
  std::size_t dropped_count() const { return m_depth->dropped_count(); }
  std::size_t unsynced_count() const { return m_unsynced_count; }

 private:
  /// Builds the frame for one depth image; null if the pose, calibration or mount is missing.
  Msg<SensorFrame> synchronize(const ImageFrame& depth) const {
    const Msg<CameraCalibration> calibration = m_calibration->latest();
    const Msg<StampedPose> mount = m_mount->latest();
    if (!calibration || !mount) return nullptr;
    if (!m_poses->wait_until_newest(depth.stamp, m_pose_wait)) return nullptr;
    const auto [before, after] = m_poses->bracket(depth.stamp);
    if (!before || !after) return nullptr;
    return std::make_shared<const SensorFrame>(
        SensorFrame{.stamp = depth.stamp,
                    .drone_pose = interpolate(*before, *after, depth.stamp),
                    .depth = depth,
                    .depth_calibration = *calibration,
                    .base_to_depth = *mount});
  }

  std::shared_ptr<BoundedQueue<ImageFrame>> m_depth;
  std::shared_ptr<HistoryBuffer<StampedPose>> m_poses;
  std::shared_ptr<LatestSlot<CameraCalibration>> m_calibration;
  std::shared_ptr<LatestSlot<StampedPose>> m_mount;
  std::chrono::milliseconds m_pose_wait;
  std::atomic<bool> m_closed{false};
  std::atomic<std::size_t> m_unsynced_count{0};
};

}
