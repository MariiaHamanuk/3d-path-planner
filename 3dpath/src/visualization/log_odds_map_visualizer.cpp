#include "visualization/log_odds_map_visualizer.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace path3d {

namespace {

builtin_interfaces::msg::Time to_ros_time(TimeSec t) {
	builtin_interfaces::msg::Time out;
	const double seconds = std::floor(t);
	out.sec = static_cast<std::int32_t>(seconds);
	out.nanosec = static_cast<std::uint32_t>((t - seconds) * 1e9);
	return out;
}

struct Voxel {
	Eigen::Vector3d center;
	float log_odds;
};

sensor_msgs::msg::PointCloud2 make_cloud(const std::vector<Voxel>& voxels,
                                         const std_msgs::msg::Header& header) {
	sensor_msgs::msg::PointCloud2 cloud;
	cloud.header = header;
	sensor_msgs::PointCloud2Modifier modifier(cloud);
	modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
	                              sensor_msgs::msg::PointField::FLOAT32, "z", 1,
	                              sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
	                              sensor_msgs::msg::PointField::FLOAT32);
	modifier.resize(voxels.size());
	sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"),
	    intensity(cloud, "intensity");
	for (const Voxel& voxel : voxels) {
		*x = static_cast<float>(voxel.center.x());
		*y = static_cast<float>(voxel.center.y());
		*z = static_cast<float>(voxel.center.z());
		*intensity = voxel.log_odds;
		++x;
		++y;
		++z;
		++intensity;
	}
	return cloud;
}

/// Blue (lowest) -> red (highest).
std_msgs::msg::ColorRGBA height_color(double t) {
	std_msgs::msg::ColorRGBA color;
	color.r = static_cast<float>(t);
	color.g = static_cast<float>(1.0 - std::abs(2.0 * t - 1.0));
	color.b = static_cast<float>(1.0 - t);
	color.a = 1.0f;
	return color;
}

}

LogOddsMapVisualizer::LogOddsMapVisualizer(Topic<LogOddsGridMap::Update>& updates,
                                           const LogOddsMapVisualizerConfig& config)
    : m_config(config), m_client(updates) {
	if (!rclcpp::ok()) {
		rclcpp::init(0, nullptr);
		m_owns_context = true;
	}
	m_node = std::make_shared<rclcpp::Node>(m_config.node_name);
	const auto qos = rclcpp::QoS(1).reliable().transient_local();
	m_occupied_pub = m_node->create_publisher<sensor_msgs::msg::PointCloud2>(
	    m_config.occupied_cloud_topic, qos);
	m_voxels_pub = m_node->create_publisher<visualization_msgs::msg::Marker>(
	    m_config.voxels_topic, qos);
	if (m_config.publish_free) {
		m_free_pub = m_node->create_publisher<sensor_msgs::msg::PointCloud2>(
		    m_config.free_cloud_topic, qos);
	}
	start();
}

LogOddsMapVisualizer::~LogOddsMapVisualizer() {
	stop();
	m_occupied_pub.reset();
	m_free_pub.reset();
	m_voxels_pub.reset();
	m_node.reset();
	if (m_owns_context) rclcpp::shutdown();
}

void LogOddsMapVisualizer::run() {
	Rate rate(m_config.rate_hz);
	std::uint64_t published_version = 0;
	while (running()) {
		if (m_client.sync() && m_client.map().version != published_version) {
			published_version = m_client.map().version;
			publish(m_client.map());
		}
		rate.sleep();
	}
}

void LogOddsMapVisualizer::publish(const LogOddsGridMap& map) {
	std_msgs::msg::Header header;
	header.frame_id = m_config.frame_id;
	header.stamp = to_ros_time(map.stamp);

	std::vector<Voxel> occupied, free;
	for (const auto& [key, value] : map.voxels()) {
		const Voxel voxel{map.center_of(LogOddsGridMap::index_of_key(key)), value};
		if (value > LogOddsGridMap::kOccupiedThreshold) {
			occupied.push_back(voxel);
		} else if (m_free_pub) {
			free.push_back(voxel);
		}
	}

	m_occupied_pub->publish(make_cloud(occupied, header));
	if (m_free_pub) m_free_pub->publish(make_cloud(free, header));

	visualization_msgs::msg::Marker marker;
	marker.header = header;
	marker.ns = "log_odds_map";
	marker.id = 0;
	marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
	marker.action = visualization_msgs::msg::Marker::ADD;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = marker.scale.y = marker.scale.z = map.resolution();
	marker.color.a = 1.0f;
	double z_min = 0.0, z_max = 0.0;
	if (!occupied.empty()) {
		const auto [low, high] = std::minmax_element(
		    occupied.begin(), occupied.end(),
		    [](const Voxel& a, const Voxel& b) { return a.center.z() < b.center.z(); });
		z_min = low->center.z();
		z_max = high->center.z();
	}
	const double z_span = std::max(z_max - z_min, 1e-6);
	marker.points.reserve(occupied.size());
	marker.colors.reserve(occupied.size());
	for (const Voxel& voxel : occupied) {
		geometry_msgs::msg::Point point;
		point.x = voxel.center.x();
		point.y = voxel.center.y();
		point.z = voxel.center.z();
		marker.points.push_back(point);
		marker.colors.push_back(height_color((voxel.center.z() - z_min) / z_span));
	}
	m_voxels_pub->publish(marker);
}

}
