#pragma once

#include "core/config.hpp"
#include "core/threading.hpp"
#include "mapping/log_odds_grid_map.hpp"
#include "pipeline/clients.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>

namespace path3d {

/**
 * @brief Publishes a LogOddsGridMap to ROS topics for Foxglove, from its own copy of the map (a
 * MapClient on the mapper's update topic), a few times per second and only when the map changed.
 *
 *   occupied_cloud_topic  PointCloud2 of the centers of the occupied voxels (intensity = log-odds)
 *   voxels_topic          Marker CUBE_LIST of the same voxels as cubes, colored by height
 *   free_cloud_topic      PointCloud2 of the free voxels, only if publish_free is set
 *
 * All are in `frame_id`, stamped with the map's sim time, and latched (transient local) so a
 * viewer that connects late gets the last map. This is ROS-facing I/O, so its header exposes ROS
 * types; the rest of the system never sees them. Create it after the SimReader, which brings up
 * ROS (this class would otherwise do it and shut it down too early). Create it before the
 * mapper starts publishing, or it waits for the next keyframe.
 */
class LogOddsMapVisualizer : public Thread {
public:
	/**
	 * @param updates Topic the mapper publishes its updates on.
	 * @param config Topic names, frame and rate.
	 */
	LogOddsMapVisualizer(Topic<LogOddsGridMap::Update>& updates,
	                     const LogOddsMapVisualizerConfig& config = {});
	~LogOddsMapVisualizer() override;

private:
	void run() override;
	void publish(const LogOddsGridMap& map);

	LogOddsMapVisualizerConfig m_config;
	bool m_owns_context = false;
	MapClient<LogOddsGridMap> m_client;
	std::shared_ptr<rclcpp::Node> m_node;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_occupied_pub;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_free_pub;
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_voxels_pub;
};

}
