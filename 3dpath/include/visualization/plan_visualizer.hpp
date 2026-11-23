#pragma once

#include "core/communication.hpp"
#include "core/config.hpp"
#include "core/threading.hpp"
#include "core/types.hpp"
#include "planning/local/trajectory_planner_report.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <vector>

namespace path3d {

/**
 * @brief Publishes the planning of a flight for Foxglove:
 *
 *   planned_path_topic  nav_msgs/Path of the newest global path that was found
 *   flown_path_topic    nav_msgs/Path of where the drone has flown (a pose every min_flown_step m)
 *   local_path_topic    nav_msgs/Path of the smoothed path the local planner flies
 *   trajectory_topic    nav_msgs/Path of the newest trajectory sent to the drone
 *   stop_point_topic    visualization_msgs/Marker: a sphere where the drone stops for an obstacle,
 *                       orange for a normal stop, red for an emergency stop; deleted while the
 *                       path is free
 *
 * All in `frame_id`, stamped with sim time; the paths are latched (transient local), so a viewer
 * that connects late gets them. Create after SimReader (which brings up ROS).
 */
class PlanVisualizer : public Thread {
public:
	/**
	 * @param global_paths Topic the global planner publishes its paths on (pipe.down()).
	 * @param poses Topic the reader publishes the drone pose on.
	 * @param trajectories Topic the local planner's trajectories are published on.
	 * @param reports Topic the local planner publishes its reports on.
	 * @param config Topic names, frame and rate.
	 */
	PlanVisualizer(Topic<GlobalPath>& global_paths, Topic<StampedPose>& poses, Topic<Trajectory>& trajectories,
	               Topic<TrajectoryPlannerReport>& reports, const PlanVisualizerConfig& config = {});
	~PlanVisualizer() override;

private:
	void run() override;
	void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>& publisher, const std::vector<Eigen::Vector3d>& points, TimeSec stamp);
	void publish_trajectory(const Trajectory& trajectory);
	void publish_stop_point(const TrajectoryPlannerReport& report);
	void record_flown(const StampedPose& pose);

	PlanVisualizerConfig m_config;
	bool m_owns_context = false;
	std::shared_ptr<LatestSlot<GlobalPath>> m_global_paths;
	std::shared_ptr<LatestSlot<StampedPose>> m_poses;
	std::shared_ptr<LatestSlot<Trajectory>> m_trajectories;
	std::shared_ptr<LatestSlot<TrajectoryPlannerReport>> m_reports;
	std::shared_ptr<rclcpp::Node> m_node;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_planned_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_flown_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_local_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_trajectory_pub;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_stop_pub;
	nav_msgs::msg::Path m_flown;  ///< every recorded pose of the flight
};

}
