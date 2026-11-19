#include "visualization/plan_visualizer.hpp"

#include <cmath>
#include <cstdint>

namespace path3d {

namespace {

builtin_interfaces::msg::Time to_ros_time(TimeSec t) {
	builtin_interfaces::msg::Time out;
	const double seconds = std::floor(t);
	out.sec = static_cast<std::int32_t>(seconds);
	out.nanosec = static_cast<std::uint32_t>((t - seconds) * 1e9);
	return out;
}

geometry_msgs::msg::Point to_point(const Eigen::Vector3d& p) {
	geometry_msgs::msg::Point point;
	point.x = p.x();
	point.y = p.y();
	point.z = p.z();
	return point;
}

geometry_msgs::msg::PoseStamped to_pose(const std_msgs::msg::Header& header, const Eigen::Vector3d& position,
                                        const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity()) {
	geometry_msgs::msg::PoseStamped pose;
	pose.header = header;
	pose.pose.position = to_point(position);
	const Eigen::Quaterniond q = orientation.normalized();
	pose.pose.orientation.x = q.x();
	pose.pose.orientation.y = q.y();
	pose.pose.orientation.z = q.z();
	pose.pose.orientation.w = q.w();
	return pose;
}

}

PlanVisualizer::PlanVisualizer(Topic<GlobalPath>& global_paths, Topic<StampedPose>& poses, Topic<Trajectory>& trajectories,
                               Topic<TrajectoryPlannerReport>& reports, const PlanVisualizerConfig& config)
    : m_config(config),
      m_global_paths(global_paths.subscribe_latest()),
      m_poses(poses.subscribe_latest()),
      m_trajectories(trajectories.subscribe_latest()),
      m_reports(reports.subscribe_latest()) {
	if (!rclcpp::ok()) {
		rclcpp::init(0, nullptr);
		m_owns_context = true;
	}
	m_node = std::make_shared<rclcpp::Node>(m_config.node_name);
	const auto latched = rclcpp::QoS(1).reliable().transient_local();
	m_planned_pub = m_node->create_publisher<nav_msgs::msg::Path>(m_config.planned_path_topic, latched);
	m_flown_pub = m_node->create_publisher<nav_msgs::msg::Path>(m_config.flown_path_topic, latched);
	m_local_pub = m_node->create_publisher<nav_msgs::msg::Path>(m_config.local_path_topic, latched);
	m_trajectory_pub = m_node->create_publisher<nav_msgs::msg::Path>(m_config.trajectory_topic, rclcpp::QoS(1));
	m_stop_pub = m_node->create_publisher<visualization_msgs::msg::Marker>(m_config.stop_point_topic, latched);
	m_flown.header.frame_id = m_config.frame_id;
	start();
}

PlanVisualizer::~PlanVisualizer() {
	stop();
	m_planned_pub.reset();
	m_flown_pub.reset();
	m_local_pub.reset();
	m_trajectory_pub.reset();
	m_stop_pub.reset();
	m_node.reset();
	if (m_owns_context) rclcpp::shutdown();
}

void PlanVisualizer::run() {
	Rate rate(m_config.rate_hz);
	Msg<GlobalPath> last_global;
	Msg<StampedPose> last_pose;
	Msg<Trajectory> last_trajectory;
	Msg<TrajectoryPlannerReport> last_report;
	std::shared_ptr<const std::vector<Eigen::Vector3d>> last_local;
	while (running()) {
		if (const Msg<GlobalPath> path = m_global_paths->latest(); path && path != last_global) {
			last_global = path;
			if (path->status == PlanStatus::Ok && !path->waypoints.empty()) publish_path(*m_planned_pub, path->waypoints, path->stamp);
		}
		if (const Msg<StampedPose> pose = m_poses->latest(); pose && pose != last_pose) {
			last_pose = pose;
			record_flown(*pose);
		}
		if (const Msg<Trajectory> trajectory = m_trajectories->latest(); trajectory && trajectory != last_trajectory) {
			last_trajectory = trajectory;
			publish_trajectory(*trajectory);
		}
		if (const Msg<TrajectoryPlannerReport> report = m_reports->latest(); report && report != last_report) {
			last_report = report;
			if (report->path && report->path != last_local) {
				last_local = report->path;
				publish_path(*m_local_pub, *report->path, report->stamp);
			}
			publish_stop_point(*report);
		}
		rate.sleep();
	}
}

void PlanVisualizer::publish_path(rclcpp::Publisher<nav_msgs::msg::Path>& publisher, const std::vector<Eigen::Vector3d>& points, TimeSec stamp) {
	nav_msgs::msg::Path msg;
	msg.header.frame_id = m_config.frame_id;
	msg.header.stamp = to_ros_time(stamp);
	msg.poses.reserve(points.size());
	for (const Eigen::Vector3d& p : points) msg.poses.push_back(to_pose(msg.header, p));
	publisher.publish(msg);
}

void PlanVisualizer::record_flown(const StampedPose& pose) {
	if (!m_flown.poses.empty()) {
		const auto& last = m_flown.poses.back().pose.position;
		if ((pose.pose.position - Eigen::Vector3d(last.x, last.y, last.z)).norm() < m_config.min_flown_step) return;
	}
	m_flown.header.stamp = to_ros_time(pose.stamp);
	m_flown.poses.push_back(to_pose(m_flown.header, pose.pose.position, pose.pose.orientation));
	m_flown_pub->publish(m_flown);
}

void PlanVisualizer::publish_trajectory(const Trajectory& trajectory) {
	nav_msgs::msg::Path msg;
	msg.header.frame_id = m_config.frame_id;
	msg.header.stamp = to_ros_time(trajectory.stamp);
	msg.poses.reserve(trajectory.points.size());
	for (const Setpoint& point : trajectory.points) {
		std_msgs::msg::Header header = msg.header;
		header.stamp = to_ros_time(point.stamp);
		const Eigen::Quaterniond yaw(Eigen::AngleAxisd(point.yaw, Eigen::Vector3d::UnitZ()));
		msg.poses.push_back(to_pose(header, point.position, yaw));
	}
	m_trajectory_pub->publish(msg);
}

void PlanVisualizer::publish_stop_point(const TrajectoryPlannerReport& report) {
	visualization_msgs::msg::Marker marker;
	marker.header.frame_id = m_config.frame_id;
	marker.header.stamp = to_ros_time(report.stamp);
	marker.ns = "stop_point";
	marker.id = 0;
	if (!report.stop_point) {
		marker.action = visualization_msgs::msg::Marker::DELETE;
		m_stop_pub->publish(marker);
		return;
	}
	marker.type = visualization_msgs::msg::Marker::SPHERE;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.position = to_point(*report.stop_point);
	marker.pose.orientation.w = 1.0;
	marker.scale.x = marker.scale.y = marker.scale.z = 0.6;
	marker.color.r = 1.0f;
	marker.color.g = report.emergency ? 0.0f : 0.55f;
	marker.color.b = 0.0f;
	marker.color.a = 0.9f;
	m_stop_pub->publish(marker);
}

}
