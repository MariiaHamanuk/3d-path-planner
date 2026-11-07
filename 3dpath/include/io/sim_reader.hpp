#pragma once

#include "core/config.hpp"
#include "io/reader.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>

#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace path3d {

/**
 * @brief Reader backed by the Gazebo simulation (through the ros_gz bridge). Every ROS callback
 * converts its message and publishes it into the SensorTopics right away; nothing is cached
 * for readers. Callbacks run on one executor thread, started by the constructor. This is the
 * ROS-facing I/O layer, so its header exposes ROS types; the rest of the system only sees
 * the topics.
 */
class SimReader : public Reader {
public:
	/**
	 * @param config Topic and frame names.
	 * @param out Topics the reader publishes into.
	 */
	SimReader(SimReaderConfig config, SensorTopics& out);
	/// Stops the executor thread and releases the ROS node.
	~SimReader() override;

private:
	void on_tf(const tf2_msgs::msg::TFMessage& msg);
	void on_image(const sensor_msgs::msg::Image& msg, CameraId camera);
	void on_info(const sensor_msgs::msg::CameraInfo& msg, CameraId camera);
	std::optional<StampedPose> lookup(const std::string& target, FrameId target_id,
	                                  const std::string& source, FrameId source_id) const;

	SimReaderConfig m_cfg;
	/// True if this reader called rclcpp::init and must shut ROS down.
	bool m_owns_context = false;
	std::shared_ptr<rclcpp::Node> m_node;
	std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;  ///< fed by on_tf, read only on the executor thread
	std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
	std::thread m_spin_thread;
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_sub;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_color_sub, m_depth_sub;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_color_info_sub, m_depth_info_sub;

	/// Touched only by callbacks, i.e. only on the executor thread: no lock needed.
	std::string m_color_name, m_depth_name;
	/// Stamp of the newest published pose; older or equal transforms are skipped.
	TimeSec m_last_pose_stamp = -1.0;
};

}
