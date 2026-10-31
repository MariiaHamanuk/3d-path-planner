#include "sim_reader.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <mutex>
#include <thread>

namespace path3d {
namespace {

TimeSec to_sec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

StampedPose to_stamped_pose(const geometry_msgs::msg::TransformStamped& ts, FrameId parent,
                            FrameId child) {
  StampedPose out;
  out.stamp = to_sec(ts.header.stamp);
  out.parent_frame = parent;
  out.child_frame = child;
  const auto& t = ts.transform.translation;
  const auto& q = ts.transform.rotation;
  out.pose.position = Eigen::Vector3d(t.x, t.y, t.z);
  out.pose.orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
  return out;
}

// ROS image -> owned cv::Mat in the project's convention (BGR8 / 32FC1 meters).
std::optional<cv::Mat> to_mat(const sensor_msgs::msg::Image& m, CameraId camera) {
  const auto view = [&m](int type) {
    return cv::Mat(static_cast<int>(m.height), static_cast<int>(m.width), type,
                   const_cast<uint8_t*>(m.data.data()), m.step);
  };
  if (camera == CameraId::Depth) {
    if (m.encoding == "32FC1") return view(CV_32FC1).clone();
    return std::nullopt;
  }
  cv::Mat bgr;
  if (m.encoding == "rgb8") {
    cv::cvtColor(view(CV_8UC3), bgr, cv::COLOR_RGB2BGR);
  } else if (m.encoding == "bgr8") {
    bgr = view(CV_8UC3).clone();
  } else if (m.encoding == "mono8") {
    cv::cvtColor(view(CV_8UC1), bgr, cv::COLOR_GRAY2BGR);
  } else {
    return std::nullopt;
  }
  return bgr;
}

CameraCalibration to_calibration(const sensor_msgs::msg::CameraInfo& m, FrameId frame) {
  CameraCalibration c;
  c.stamp = to_sec(m.header.stamp);
  c.frame_id = frame;
  c.width = static_cast<int>(m.width);
  c.height = static_cast<int>(m.height);
  for (int i = 0; i < 9; ++i) {
    c.K(i / 3, i % 3) = m.k[i];
    c.R(i / 3, i % 3) = m.r[i];
  }
  for (int i = 0; i < 12; ++i) c.P(i / 4, i % 4) = m.p[i];
  c.distortion = cv::Mat_<double>(1, static_cast<int>(m.d.size()));
  for (size_t i = 0; i < m.d.size(); ++i) c.distortion(0, static_cast<int>(i)) = m.d[i];
  c.distortion_model = m.distortion_model;
  return c;
}

// Gazebo's depth_camera sensor publishes no CameraInfo, so build the ideal pinhole model
// (no distortion) from the image size and the sensor's horizontal field of view.
CameraCalibration pinhole_from_fov(const ImageFrame& f, double hfov_rad) {
  CameraCalibration c;
  c.stamp = f.stamp;
  c.frame_id = f.frame_id;
  c.width = f.image.cols;
  c.height = f.image.rows;
  const double focal = (c.width / 2.0) / std::tan(hfov_rad / 2.0);
  const double cx = c.width / 2.0;
  const double cy = c.height / 2.0;
  c.K = cv::Matx33d(focal, 0, cx, 0, focal, cy, 0, 0, 1);
  c.P = cv::Matx34d(focal, 0, cx, 0, 0, focal, cy, 0, 0, 0, 1, 0);
  c.distortion = cv::Mat_<double>(1, 0);
  c.distortion_model = "plumb_bob";
  return c;
}

}  // namespace

struct SimReader::Impl {
  explicit Impl(SimReaderConfig config) : m_cfg(std::move(config)) {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      m_owns_context = true;
    }
    m_node = std::make_shared<rclcpp::Node>(m_cfg.node_name);
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer, m_node, false);

    const auto qos = rclcpp::SensorDataQoS();
    m_color_sub = m_node->create_subscription<sensor_msgs::msg::Image>(
        m_cfg.color_image_topic, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_image(*m, CameraId::Color); });
    m_depth_sub = m_node->create_subscription<sensor_msgs::msg::Image>(
        m_cfg.depth_image_topic, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_image(*m, CameraId::Depth); });
    m_color_info_sub = m_node->create_subscription<sensor_msgs::msg::CameraInfo>(
        m_cfg.color_info_topic, qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { on_color_info(*m); });

    m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    m_executor->add_node(m_node);
    m_spin_thread = std::thread([this] { m_executor->spin(); });
  }

  ~Impl() {
    if (m_executor) m_executor->cancel();
    if (m_spin_thread.joinable()) m_spin_thread.join();
    m_tf_listener.reset();
    m_tf_buffer.reset();
    m_color_sub.reset();
    m_depth_sub.reset();
    m_color_info_sub.reset();
    m_node.reset();
    if (m_owns_context) rclcpp::shutdown();
  }

  void on_image(const sensor_msgs::msg::Image& m, CameraId camera) {
    auto mat = to_mat(m, camera);
    if (!mat) {
      RCLCPP_WARN_THROTTLE(m_node->get_logger(), *m_node->get_clock(), 5000,
                           "unsupported image encoding '%s'", m.encoding.c_str());
      return;
    }
    ImageFrame f;
    f.stamp = to_sec(m.header.stamp);
    f.frame_id = camera_frame(camera);
    f.image = std::move(*mat);
    std::lock_guard<std::mutex> lock(m_mtx);
    std::string& name = camera == CameraId::Color ? m_color_name : m_depth_name;
    if (name.empty()) name = m.header.frame_id;
    if (camera == CameraId::Depth && (!m_depth_cal || m_depth_cal->width != f.image.cols ||
                                      m_depth_cal->height != f.image.rows)) {
      m_depth_cal = pinhole_from_fov(f, m_cfg.depth_hfov_rad);
    }
    (camera == CameraId::Color ? m_color : m_depth) = std::move(f);
  }

  void on_color_info(const sensor_msgs::msg::CameraInfo& m) {
    auto c = to_calibration(m, FrameId::ColorCamera);
    std::lock_guard<std::mutex> lock(m_mtx);
    if (!m.header.frame_id.empty()) m_color_name = m.header.frame_id;
    m_color_cal = std::move(c);
  }

  // Native (TF) name of a frame; null while a camera frame has not been seen yet.
  std::optional<std::string> frame_name(FrameId frame) const {
    switch (frame) {
      case FrameId::World: return m_cfg.world_frame;
      case FrameId::Base: return m_cfg.base_frame;
      case FrameId::ColorCamera:
      case FrameId::DepthCamera: {
        std::lock_guard<std::mutex> lock(m_mtx);
        const std::string& name = frame == FrameId::ColorCamera ? m_color_name : m_depth_name;
        if (name.empty()) return std::nullopt;
        return name;
      }
    }
    return std::nullopt;
  }

  // pose of `source` expressed in `target`
  std::optional<StampedPose> lookup(FrameId target, FrameId source) const {
    const auto target_name = frame_name(target);
    const auto source_name = frame_name(source);
    if (!target_name || !source_name) return std::nullopt;
    try {
      return to_stamped_pose(m_tf_buffer->lookupTransform(*target_name, *source_name,
                                                          tf2::TimePointZero),
                             target, source);
    } catch (const tf2::TransformException&) {
      return std::nullopt;
    }
  }

  SimReaderConfig m_cfg;
  bool m_owns_context = false;
  std::shared_ptr<rclcpp::Node> m_node;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;  // after rclcpp::init
  std::thread m_spin_thread;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_color_sub, m_depth_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_color_info_sub;

  mutable std::mutex m_mtx;
  std::string m_color_name, m_depth_name;  // native TF names of the camera frames
  std::optional<ImageFrame> m_color, m_depth;
  std::optional<CameraCalibration> m_color_cal, m_depth_cal;
};

SimReader::SimReader(SimReaderConfig config) : m_impl(std::make_unique<Impl>(std::move(config))) {}
SimReader::~SimReader() = default;

std::optional<StampedPose> SimReader::drone_pose() const {
  return m_impl->lookup(FrameId::World, FrameId::Base);
}

std::optional<ImageFrame> SimReader::color_image() const {
  std::lock_guard<std::mutex> lock(m_impl->m_mtx);
  return m_impl->m_color;
}

std::optional<ImageFrame> SimReader::depth_image() const {
  std::lock_guard<std::mutex> lock(m_impl->m_mtx);
  return m_impl->m_depth;
}

std::optional<CameraCalibration> SimReader::calibration(CameraId camera) const {
  std::lock_guard<std::mutex> lock(m_impl->m_mtx);
  return camera == CameraId::Color ? m_impl->m_color_cal : m_impl->m_depth_cal;
}

std::optional<StampedPose> SimReader::base_to_camera(CameraId camera) const {
  return m_impl->lookup(FrameId::Base, camera_frame(camera));
}

bool SimReader::wait_for_data(std::chrono::milliseconds timeout) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const auto ready = [this] {
    return drone_pose() && color_image() && depth_image() && calibration(CameraId::Color) &&
           calibration(CameraId::Depth) && base_to_camera(CameraId::Color) &&
           base_to_camera(CameraId::Depth);
  };
  while (!ready()) {
    if (std::chrono::steady_clock::now() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return true;
}

}  // namespace path3d
