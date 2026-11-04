#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <cstdint>
#include <string>

// Project-wide data types. No ROS / Gazebo types leak past this header.
namespace path3d {

using TimeSec = double;  // seconds, simulation clock

enum class CameraId { Color, Depth };

// Coordinate frames of the system. Readers convert their native frame names (ROS/TF strings,
// URDF links, ...) to this enum, so no string is stored or copied per message.
enum class FrameId : std::uint8_t {
  World,        // fixed world frame
  Base,         // drone base link
  ColorCamera,  // color camera optical frame
  DepthCamera,  // depth camera optical frame
};

inline const char* to_string(FrameId frame) {
  switch (frame) {
    case FrameId::World: return "world";
    case FrameId::Base: return "base";
    case FrameId::ColorCamera: return "color_camera";
    case FrameId::DepthCamera: return "depth_camera";
  }
  return "unknown";
}

constexpr FrameId camera_frame(CameraId camera) {
  return camera == CameraId::Color ? FrameId::ColorCamera : FrameId::DepthCamera;
}

struct Pose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();

  Eigen::Isometry3d to_isometry() const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = orientation.normalized().toRotationMatrix();
    T.translation() = position;
    return T;
  }
};

// Pose of `child_frame` expressed in `parent_frame` (i.e. T_parent_child).
struct StampedPose {
  TimeSec stamp = 0.0;
  FrameId parent_frame = FrameId::World;
  FrameId child_frame = FrameId::Base;
  Pose pose;
};

// Color: CV_8UC3, BGR.
// Depth: CV_32FC1, meters along the optical axis; non-finite (inf/NaN) = no return.
struct ImageFrame {
  TimeSec stamp = 0.0;
  FrameId frame_id = FrameId::ColorCamera;
  cv::Mat image;
};

// Pinhole camera model (sensor_msgs/CameraInfo layout).
// Camera frame follows the Gazebo convention: x forward, y left, z up;
// K is expressed for the image (u right, v down).
struct CameraCalibration {
  TimeSec stamp = 0.0;
  FrameId frame_id = FrameId::ColorCamera;
  int width = 0;
  int height = 0;
  cv::Matx33d K = cv::Matx33d::eye();  // intrinsics
  cv::Matx33d R = cv::Matx33d::eye();  // rectification
  cv::Matx34d P = cv::Matx34d::zeros();  // projection
  cv::Mat_<double> distortion;           // 1xN, empty if none
  std::string distortion_model;

  double fx() const { return K(0, 0); }
  double fy() const { return K(1, 1); }
  double cx() const { return K(0, 2); }
  double cy() const { return K(1, 2); }
};

// Everything a mapper needs for one depth observation: all data a Reader provides, bundled so a
// consumer never has to query the Reader piecemeal. `stamp` is the depth image's stamp. Poses are
// the newest the reader had at capture time (not interpolated to `stamp`).
struct SensorFrame {
  TimeSec stamp = 0.0;
  StampedPose drone_pose;  // base link in the world frame
  ImageFrame color;
  ImageFrame depth;
  CameraCalibration color_calibration;
  CameraCalibration depth_calibration;
  StampedPose base_to_color;  // T_base_camera
  StampedPose base_to_depth;
};

}  // namespace path3d
