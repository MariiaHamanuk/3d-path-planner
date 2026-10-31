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

}  // namespace path3d
