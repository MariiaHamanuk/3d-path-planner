#pragma once

#include "reader.hpp"

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

// Reader backed by the Gazebo simulation: subscribes (through the ros_gz bridge) in a background
// thread and keeps the latest message of each kind.
class SimReader : public Reader {
 public:
  explicit SimReader(SimReaderConfig config = {});
  ~SimReader() override;

  std::optional<StampedPose> drone_pose() const override;
  std::optional<ImageFrame> color_image() const override;
  std::optional<ImageFrame> depth_image() const override;
  std::optional<CameraCalibration> calibration(CameraId camera) const override;
  std::optional<StampedPose> base_to_camera(CameraId camera) const override;
  bool wait_for_data(std::chrono::milliseconds timeout) const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

}  // namespace path3d
