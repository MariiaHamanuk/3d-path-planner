#pragma once

#include "reader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace path3d {

struct SimReaderConfig {
  std::string node_name = "sim_reader";
  std::string world_frame = "default";  // = Gazebo world name ("default", "city")
  std::string base_frame = "parrot_bebop_2/body";
  std::string color_image_topic = "/camera";
  std::string color_info_topic = "/camera/camera_info";
  std::string depth_image_topic = "/depth_camera";
  std::string depth_info_topic = "/depth_camera/camera_info";
};

// Reader backed by the Gazebo simulation: subscribes (through the ros_gz bridge) in a background
// thread and keeps the latest message of each kind. This is the ROS-facing I/O layer, so its
// header exposes ROS types; the rest of the system uses it only through `Reader`.
class SimReader : public Reader {
 public:
  explicit SimReader(SimReaderConfig config = {});
  ~SimReader() override;
  SimReader(const SimReader&) = delete;
  SimReader& operator=(const SimReader&) = delete;

  std::optional<StampedPose> drone_pose() const override;
  std::optional<ImageFrame> color_image() const override;
  std::optional<ImageFrame> depth_image() const override;
  std::optional<CameraCalibration> calibration(CameraId camera) const override;
  std::optional<StampedPose> base_to_camera(CameraId camera) const override;
  bool wait_for_data(std::chrono::milliseconds timeout) const override;

 private:
  void on_image(const sensor_msgs::msg::Image& msg, CameraId camera);
  void on_info(const sensor_msgs::msg::CameraInfo& msg, CameraId camera);
  std::optional<std::string> frame_name(FrameId frame) const;
  std::optional<StampedPose> lookup(FrameId target, FrameId source) const;  // `source` in `target`

  SimReaderConfig m_cfg;
  bool m_owns_context = false;
  std::shared_ptr<rclcpp::Node> m_node;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;  // created after rclcpp::init
  std::thread m_spin_thread;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_color_sub, m_depth_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_color_info_sub, m_depth_info_sub;

  mutable std::mutex m_mtx;  // guards everything below
  std::string m_color_name, m_depth_name;  // native TF names of the camera frames
  std::optional<ImageFrame> m_color, m_depth;
  std::optional<CameraCalibration> m_color_cal, m_depth_cal;
};

}  // namespace path3d
