#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <string>

// Project-wide data types. No ROS / Gazebo types leak past this header.
namespace path3d {

using TimeSec = double;  // seconds, simulation clock

enum class CameraId { Color, Depth };

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
  std::string parent_frame;
  std::string child_frame;
  Pose pose;
};

// Color: CV_8UC3, BGR.
// Depth: CV_32FC1, meters along the optical axis; non-finite (inf/NaN) = no return.
struct ImageFrame {
  TimeSec stamp = 0.0;
  std::string frame_id;
  cv::Mat image;
};

// Pinhole camera model (sensor_msgs/CameraInfo layout).
// Camera frame follows the Gazebo convention: x forward, y left, z up;
// K is expressed for the image (u right, v down).
struct CameraCalibration {
  TimeSec stamp = 0.0;
  std::string frame_id;
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

}  // namespace path3d
