#pragma once

#include "reader.hpp"

#include <opencv2/core.hpp>

namespace path3d::test {

// Reader with canned data: no ROS, no simulation. `ready = false` simulates missing data.
class FakeReader : public Reader {
 public:
  explicit FakeReader(bool ready = true) : m_ready(ready) {}

  std::optional<StampedPose> drone_pose() const override {
    if (!m_ready) return std::nullopt;
    StampedPose pose;
    pose.stamp = 1.5;
    pose.parent_frame = FrameId::World;
    pose.child_frame = FrameId::Base;
    pose.pose.position = Eigen::Vector3d(1, 2, 3);
    return pose;
  }
  std::optional<ImageFrame> color_image() const override {
    if (!m_ready) return std::nullopt;
    return ImageFrame{.stamp = 1.4, .frame_id = FrameId::ColorCamera,
                      .image = cv::Mat(4, 6, CV_8UC3, cv::Scalar(1, 2, 3))};
  }
  std::optional<ImageFrame> depth_image() const override {
    if (!m_ready) return std::nullopt;
    return ImageFrame{.stamp = 1.45, .frame_id = FrameId::DepthCamera,
                      .image = cv::Mat(2, 3, CV_32FC1, cv::Scalar(2.5))};
  }
  std::optional<CameraCalibration> calibration(CameraId camera) const override {
    if (!m_ready) return std::nullopt;
    CameraCalibration calibration;
    calibration.frame_id = camera_frame(camera);
    return calibration;
  }
  std::optional<StampedPose> base_to_camera(CameraId camera) const override {
    if (!m_ready) return std::nullopt;
    StampedPose pose;
    pose.parent_frame = FrameId::Base;
    pose.child_frame = camera_frame(camera);
    pose.pose.position = Eigen::Vector3d(0.1, 0, 0);
    return pose;
  }
  bool wait_for_data(std::chrono::milliseconds) const override { return m_ready; }

 private:
  bool m_ready;
};

}  // namespace path3d::test
