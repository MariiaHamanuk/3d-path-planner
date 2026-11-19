#include "visualization/path_visualizer.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace path3d {

namespace {

builtin_interfaces::msg::Time to_ros_time(TimeSec t) {
	builtin_interfaces::msg::Time out;
	const double seconds = std::floor(t);
	out.sec = static_cast<std::int32_t>(seconds);
	out.nanosec = static_cast<std::uint32_t>((t - seconds) * 1e9);
	return out;
}

geometry_msgs::msg::PoseStamped make_pose_stamped(const std_msgs::msg::Header& header,
                                                  const Eigen::Vector3d& position,
                                                  const Eigen::Quaterniond& orientation) {
	geometry_msgs::msg::PoseStamped msg;
	msg.header = header;
	msg.pose.position.x = position.x();
	msg.pose.position.y = position.y();
	msg.pose.position.z = position.z();
	const Eigen::Quaterniond q = orientation.normalized();
	msg.pose.orientation.x = q.x();
	msg.pose.orientation.y = q.y();
	msg.pose.orientation.z = q.z();
	msg.pose.orientation.w = q.w();
	return msg;
}

/// Index of the segment [i, i + 1] (from `start_index` onward) closest to `point`.
std::size_t nearest_segment(const std::vector<Eigen::Vector3d>& waypoints,
                            std::size_t start_index, const Eigen::Vector3d& point) {
	std::size_t best_seg = start_index;
	double best_dist_sq = std::numeric_limits<double>::infinity();
	for (std::size_t i = start_index; i + 1 < waypoints.size(); ++i) {
		const Eigen::Vector3d ab = waypoints[i + 1] - waypoints[i];
		const double len_sq = ab.squaredNorm();
		const double t = len_sq > 1e-12
		                     ? std::clamp((point - waypoints[i]).dot(ab) / len_sq, 0.0, 1.0)
		                     : 0.0;
		const double dist_sq = (point - (waypoints[i] + t * ab)).squaredNorm();
		if (dist_sq < best_dist_sq) {
			best_dist_sq = dist_sq;
			best_seg = i;
		}
	}
	return best_seg;
}

}

PathVisualizer::PathVisualizer(Topic<GlobalPath>& global_paths, Topic<StampedPose>& poses,
                               const PathVisualizerConfig& config)
    : m_config(config),
      m_path_sub(global_paths.subscribe_latest()),
      m_pose_sub(poses.subscribe_latest()) {
	if (!rclcpp::ok()) {
		rclcpp::init(0, nullptr);
		m_owns_context = true;
	}
	m_node = std::make_shared<rclcpp::Node>(m_config.node_name);
	const auto qos = rclcpp::QoS(1).reliable().transient_local();
	m_planned_pub = m_node->create_publisher<nav_msgs::msg::Path>(
	    m_config.planned_path_topic, qos);
	m_flown_pub = m_node->create_publisher<nav_msgs::msg::Path>(
	    m_config.flown_path_topic, qos);
	start();
}

PathVisualizer::~PathVisualizer() {
	stop();
	m_planned_pub.reset();
	m_flown_pub.reset();
	m_node.reset();
	if (m_owns_context) rclcpp::shutdown();
}

void PathVisualizer::run() {
	Rate rate(m_config.rate_hz);
	Msg<GlobalPath> last_published_path;
	TimeSec last_pose_stamp = -1.0;

	while (running()) {
		if (const Msg<GlobalPath> path = m_path_sub->latest();
		    path && path != last_published_path) {
			last_published_path = path;
			if (path->status == PlanStatus::Ok && !path->waypoints.empty()) {
				publish_planned(*path);
			}
		}

		if (const Msg<StampedPose> pose = m_pose_sub->latest();
		    pose && pose->stamp > last_pose_stamp) {
			last_pose_stamp = pose->stamp;
			const Eigen::Vector3d& pos = pose->pose.position;
			if (!m_has_flown_pose ||
			    (pos - m_last_flown_position).norm() >= m_config.min_flown_step) {
				m_last_flown_position = pos;
				m_has_flown_pose = true;
				publish_flown(*pose);
			}
		}

		rate.sleep();
	}
}

void PathVisualizer::publish_planned(const GlobalPath& path) {
	if (m_planned_waypoints.size() >= 2) {
		const std::size_t seg =
		    nearest_segment(m_planned_waypoints, m_planned_prefix_start, path.waypoints.front());
		m_planned_waypoints.resize(seg + 1);
		m_planned_prefix_start = seg;
	} else {
		m_planned_waypoints.clear();
		m_planned_prefix_start = 0;
	}
	m_planned_waypoints.insert(
	    m_planned_waypoints.end(), path.waypoints.begin(), path.waypoints.end());

	nav_msgs::msg::Path msg;
	msg.header.frame_id = m_config.frame_id;
	msg.header.stamp = to_ros_time(path.stamp);
	msg.poses.reserve(m_planned_waypoints.size());
	for (const Eigen::Vector3d& wp : m_planned_waypoints) {
		msg.poses.push_back(make_pose_stamped(msg.header, wp, Eigen::Quaterniond::Identity()));
	}
	m_planned_pub->publish(msg);
}

void PathVisualizer::publish_flown(const StampedPose& pose) {
	std_msgs::msg::Header header;
	header.frame_id = m_config.frame_id;
	header.stamp = to_ros_time(pose.stamp);
	m_flown_poses.push_back(
	    make_pose_stamped(header, pose.pose.position, pose.pose.orientation));

	nav_msgs::msg::Path msg;
	msg.header = header;
	msg.poses = m_flown_poses;
	m_flown_pub->publish(msg);
}

}
