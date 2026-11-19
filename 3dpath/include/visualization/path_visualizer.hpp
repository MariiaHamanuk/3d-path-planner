#pragma once

#include "core/communication.hpp"
#include "core/config.hpp"
#include "core/threading.hpp"
#include "core/types.hpp"

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <memory>
#include <vector>

namespace path3d {

/**
 * @brief Publishes the current global planned path and the drone's flown path as nav_msgs/Path
 * topics for Foxglove.
 *
 *   planned_path_topic  nav_msgs/Path of the latest valid GlobalPath waypoints
 *   flown_path_topic    nav_msgs/Path of the drone's recorded poses (sampled every min_flown_step m)
 *
 * Both are in `frame_id`, stamped with sim time, and latched (transient local) so a viewer that
 * connects late receives the latest paths. Create after SimReader (which brings up ROS) and
 * before the planners/threads start publishing.
 */
class PathVisualizer : public Thread {
public:
	/**
	 * @param global_paths Topic the global planner publishes plans on (e.g. pipe.down()).
	 * @param poses Topic the reader publishes drone poses on (e.g. sensors.pose).
	 * @param config Topic names, frame, rate and sampling step.
	 */
	PathVisualizer(Topic<GlobalPath>& global_paths, Topic<StampedPose>& poses,
	               const PathVisualizerConfig& config = {});
	~PathVisualizer() override;

private:
	void run() override;
	void publish_planned(const GlobalPath& path);
	void publish_flown(const StampedPose& pose);

	PathVisualizerConfig m_config;
	bool m_owns_context = false;
	std::shared_ptr<LatestSlot<GlobalPath>> m_path_sub;
	std::shared_ptr<LatestSlot<StampedPose>> m_pose_sub;
	std::shared_ptr<rclcpp::Node> m_node;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_planned_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_flown_pub;
	std::vector<Eigen::Vector3d> m_planned_waypoints;
	std::size_t m_planned_prefix_start = 0;
	std::vector<geometry_msgs::msg::PoseStamped> m_flown_poses;
	Eigen::Vector3d m_last_flown_position = Eigen::Vector3d::Zero();
	bool m_has_flown_pose = false;
};

}
