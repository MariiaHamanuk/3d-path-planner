#include "mapping/log_odds_mapper.hpp"
#include "visualization/log_odds_map_visualizer.hpp"

#include <gtest/gtest.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <chrono>

using namespace std::chrono_literals;
using namespace path3d;

namespace {

SensorFrame wall_frame() {
	SensorFrame frame;
	frame.stamp = 1.5;
	frame.depth.image = cv::Mat(12, 16, CV_32FC1, cv::Scalar(2.0f));
	frame.depth_calibration.K = cv::Matx33d(10, 0, 8, 0, 10, 6, 0, 0, 1);
	return frame;
}

class VisualizerTest : public ::testing::Test {
protected:
	void SetUp() override {
		if (!rclcpp::ok()) rclcpp::init(0, nullptr);
	}
	void TearDown() override { rclcpp::shutdown(); }
};

}

TEST_F(VisualizerTest, PublishesTheOccupiedVoxelsOfTheMap) {
	LogOddsMapVisualizerConfig config;
	config.rate_hz = 50.0;
	config.frame_id = "city";
	Topic<LogOddsGridMap::Update> updates;

	auto node = std::make_shared<rclcpp::Node>("visualizer_test_listener");
	sensor_msgs::msg::PointCloud2::SharedPtr cloud;
	visualization_msgs::msg::Marker::SharedPtr marker;
	const auto qos = rclcpp::QoS(1).reliable().transient_local();
	auto cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
	    config.occupied_cloud_topic, qos,
	    [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloud = msg; });
	auto marker_sub = node->create_subscription<visualization_msgs::msg::Marker>(
	    config.voxels_topic, qos, [&](visualization_msgs::msg::Marker::SharedPtr msg) { marker = msg; });

	LogOddsMapVisualizer visualizer(updates, config);
	LogOddsMapperConfig mapper_config;
	mapper_config.pixel_stride = 1;
	LogOddsMapper mapper(mapper_config);
	mapper.integrate(wall_frame());
	updates.publish(*mapper.make_update(true));

	const auto deadline = std::chrono::steady_clock::now() + 10s;
	while ((!cloud || !marker) && std::chrono::steady_clock::now() < deadline) {
		rclcpp::spin_some(node);
		std::this_thread::sleep_for(10ms);
	}
	ASSERT_TRUE(cloud && marker);

	const std::size_t occupied = mapper.map().occupied_count();
	ASSERT_GT(occupied, 0u);
	EXPECT_EQ(cloud->width * cloud->height, occupied);
	EXPECT_EQ(cloud->header.frame_id, "city");
	EXPECT_EQ(cloud->header.stamp.sec, 1);
	EXPECT_EQ(cloud->header.stamp.nanosec, 500000000u);
	sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
	EXPECT_NEAR(*x, 2.125f, 0.5f);

	EXPECT_EQ(marker->points.size(), occupied);
	EXPECT_EQ(marker->colors.size(), occupied);
	EXPECT_DOUBLE_EQ(marker->scale.x, mapper.map().resolution());
	EXPECT_EQ(marker->type, visualization_msgs::msg::Marker::CUBE_LIST);
}
