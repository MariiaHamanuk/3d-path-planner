#pragma once

#include "core/communication.hpp"
#include "core/types.hpp"

namespace path3d {

/**
 * @brief The sensor streams of the system, one topic per kind of data. A Reader publishes into
 * them; consumers subscribe with the policy they need and synchronize what they combine
 * themselves (see FrameSynchronizer in sync.hpp). Only project types travel here (no ROS).
 */
struct SensorTopics {
  Topic<StampedPose> pose;  ///< drone base link in the world frame, at the pose rate
  Topic<ImageFrame> color;
  Topic<ImageFrame> depth;
  Topic<CameraCalibration> color_calibration;  ///< republished regularly (with every camera_info)
  Topic<CameraCalibration> depth_calibration;
  Topic<StampedPose> base_to_color;  ///< T_base_camera, republished with the calibration
  Topic<StampedPose> base_to_depth;
};

/**
 * @brief Source of sensor data: publishes into the SensorTopics given at construction, from its own
 * thread(s), as data arrives. The rest of the system depends only on the topics, so the
 * simulation can be replaced by a real drone, a rosbag replay, or a test that publishes
 * directly. Implementations convert their native frame names to `FrameId`; no frame strings
 * are published. Create every subscriber before the reader: topics deliver only to existing
 * subscribers.
 */
class Reader {
 public:
  virtual ~Reader() = default;
  Reader(const Reader&) = delete;
  Reader& operator=(const Reader&) = delete;

 protected:
  /**
   * @param out Topics the reader publishes into.
   */
  explicit Reader(SensorTopics& out) : m_out(out) {}

  /// Topics the reader publishes into.
  SensorTopics& m_out;
};

}
