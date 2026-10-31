#pragma once

#include "path3d/types.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace path3d {

struct SimReaderConfig {
  std::string node_name = "sim_reader";
  std::string world_frame = "default";  // = Gazebo world name ("default", "city")
  std::string base_frame = "parrot_bebop_2/body";
  std::string color_image_topic = "/camera";
  std::string color_info_topic = "/camera/camera_info";
  std::string depth_image_topic = "/depth_camera";
  // Gazebo's depth camera has no CameraInfo topic; its intrinsics are derived from the
  // image size and this field of view (keep in sync with <horizontal_fov> in model.sdf).
  double depth_hfov_rad = 1.047;
};

// Subscribes to the simulation (through the ros_gz bridge) in a background thread and
// keeps the latest message of each kind. Getters are thread-safe, never block, and
// return std::nullopt until the first matching message has arrived.
class SimReader {
 public:
  explicit SimReader(SimReaderConfig config = {});
  ~SimReader();
  SimReader(const SimReader&) = delete;
  SimReader& operator=(const SimReader&) = delete;

  // Drone base link in the world frame.
  std::optional<StampedPose> drone_pose() const;

  std::optional<ImageFrame> color_image() const;
  std::optional<ImageFrame> depth_image() const;

  std::optional<CameraCalibration> calibration(CameraId camera) const;

  // Pose of the camera frame in the drone base-link frame (T_base_camera).
  std::optional<StampedPose> base_to_camera(CameraId camera) const;

  // Blocks until every getter above has data, or the timeout expires.
  bool wait_for_data(std::chrono::milliseconds timeout) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace path3d
