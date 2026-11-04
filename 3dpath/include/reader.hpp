#pragma once

#include "types.hpp"

#include <chrono>
#include <optional>

namespace path3d {

// Source of sensor data for the pipeline. The rest of the system depends only on this interface,
// so the simulation can be replaced by a real drone, a rosbag replay or a test fake.
//
// Contract for implementations: getters are thread-safe, never block, and return std::nullopt
// until the first matching data is available. Only project types appear here (no ROS).
// Implementations convert their native frame names to `FrameId`; no frame strings cross this
// interface.
class Reader {
 public:
  virtual ~Reader() = default;
  Reader(const Reader&) = delete;
  Reader& operator=(const Reader&) = delete;

  // Drone base link in the world frame.
  virtual std::optional<StampedPose> drone_pose() const = 0;

  virtual std::optional<ImageFrame> color_image() const = 0;
  virtual std::optional<ImageFrame> depth_image() const = 0;

  virtual std::optional<CameraCalibration> calibration(CameraId camera) const = 0;

  // Pose of the camera frame in the drone base-link frame (T_base_camera).
  virtual std::optional<StampedPose> base_to_camera(CameraId camera) const = 0;

  // Blocks until every getter above has data, or the timeout expires.
  virtual bool wait_for_data(std::chrono::milliseconds timeout) const = 0;

 protected:
  Reader() = default;
};

// Snapshot of everything the reader currently provides; null if any part is still missing.
inline std::optional<SensorFrame> read_sensor_frame(const Reader& reader) {
  auto drone_pose = reader.drone_pose();
  auto color = reader.color_image();
  auto depth = reader.depth_image();
  auto color_calibration = reader.calibration(CameraId::Color);
  auto depth_calibration = reader.calibration(CameraId::Depth);
  auto base_to_color = reader.base_to_camera(CameraId::Color);
  auto base_to_depth = reader.base_to_camera(CameraId::Depth);
  if (!drone_pose || !color || !depth || !color_calibration || !depth_calibration ||
      !base_to_color || !base_to_depth) {
    return std::nullopt;
  }
  return SensorFrame{.stamp = depth->stamp,
                     .drone_pose = std::move(*drone_pose),
                     .color = std::move(*color),
                     .depth = std::move(*depth),
                     .color_calibration = std::move(*color_calibration),
                     .depth_calibration = std::move(*depth_calibration),
                     .base_to_color = std::move(*base_to_color),
                     .base_to_depth = std::move(*base_to_depth)};
}

}  // namespace path3d
