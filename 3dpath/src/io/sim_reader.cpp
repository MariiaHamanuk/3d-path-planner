#include "io/sim_reader.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/exceptions.h>


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

/// ROS image -> owned cv::Mat in the project's convention (BGR8 / 32FC1 meters).
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

}

SimReader::SimReader(SimReaderConfig config, SensorTopics& out)
    : Reader(out), m_cfg(std::move(config)) {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
    m_owns_context = true;
  }
  m_node = std::make_shared<rclcpp::Node>(m_cfg.node_name);
  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());

  m_tf_sub = m_node->create_subscription<tf2_msgs::msg::TFMessage>(
      m_cfg.tf_topic, rclcpp::QoS(100),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) { on_tf(*msg); });
  const auto qos = rclcpp::SensorDataQoS();
  m_color_sub = m_node->create_subscription<sensor_msgs::msg::Image>(
      m_cfg.color_image_topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(*msg, CameraId::Color); });
  m_depth_sub = m_node->create_subscription<sensor_msgs::msg::Image>(
      m_cfg.depth_image_topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(*msg, CameraId::Depth); });
  m_color_info_sub = m_node->create_subscription<sensor_msgs::msg::CameraInfo>(
      m_cfg.color_info_topic, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        on_info(*msg, CameraId::Color);
      });
  m_depth_info_sub = m_node->create_subscription<sensor_msgs::msg::CameraInfo>(
      m_cfg.depth_info_topic, qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        on_info(*msg, CameraId::Depth);
      });

  m_executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  m_executor->add_node(m_node);
  m_spin_thread = std::thread([this] { m_executor->spin(); });
}

SimReader::~SimReader() {
  if (m_executor) m_executor->cancel();
  if (m_spin_thread.joinable()) m_spin_thread.join();
  m_tf_sub.reset();
  m_color_sub.reset();
  m_depth_sub.reset();
  m_color_info_sub.reset();
  m_depth_info_sub.reset();
  m_tf_buffer.reset();
  m_node.reset();
  if (m_owns_context) rclcpp::shutdown();
}

/// Feeds the TF buffer ourselves (instead of a TransformListener), so the pose published here
/// always includes the message that triggered it.
void SimReader::on_tf(const tf2_msgs::msg::TFMessage& msg) {
  for (const auto& transform : msg.transforms) {
    m_tf_buffer->setTransform(transform, m_cfg.node_name, /*is_static=*/false);
  }
  auto pose = lookup(m_cfg.world_frame, FrameId::World, m_cfg.base_frame, FrameId::Base);
  if (pose && pose->stamp > m_last_pose_stamp) {
    m_last_pose_stamp = pose->stamp;
    m_out.pose.publish(std::move(*pose));
  }
}

void SimReader::on_image(const sensor_msgs::msg::Image& msg, CameraId camera) {
  auto mat = to_mat(msg, camera);
  if (!mat) {
    RCLCPP_WARN_THROTTLE(m_node->get_logger(), *m_node->get_clock(), 5000,
                         "unsupported image encoding '%s'", msg.encoding.c_str());
    return;
  }
  std::string& name = camera == CameraId::Color ? m_color_name : m_depth_name;
  if (name.empty()) name = msg.header.frame_id;
  ImageFrame frame{.stamp = to_sec(msg.header.stamp),
                   .frame_id = camera_frame(camera),
                   .image = std::move(*mat)};
  (camera == CameraId::Color ? m_out.color : m_out.depth).publish(std::move(frame));
}

/// Publishes the calibration and, with it, the camera mount (base -> camera), so both are
/// republished regularly and late subscribers get them too.
void SimReader::on_info(const sensor_msgs::msg::CameraInfo& msg, CameraId camera) {
  std::string& name = camera == CameraId::Color ? m_color_name : m_depth_name;
  if (!msg.header.frame_id.empty()) name = msg.header.frame_id;
  const bool color = camera == CameraId::Color;
  (color ? m_out.color_calibration : m_out.depth_calibration)
      .publish(to_calibration(msg, camera_frame(camera)));
  if (name.empty()) return;
  if (auto mount = lookup(m_cfg.base_frame, FrameId::Base, name, camera_frame(camera))) {
    (color ? m_out.base_to_color : m_out.base_to_depth).publish(std::move(*mount));
  }
}

std::optional<StampedPose> SimReader::lookup(const std::string& target, FrameId target_id,
                                             const std::string& source, FrameId source_id) const {
  try {
    return to_stamped_pose(m_tf_buffer->lookupTransform(target, source, tf2::TimePointZero),
                           target_id, source_id);
  } catch (const tf2::TransformException&) {
    return std::nullopt;
  }
}

}
