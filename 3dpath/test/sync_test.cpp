#include "pipeline/sync.hpp"

#include <gtest/gtest.h>

#include <thread>

using namespace std::chrono_literals;
using namespace path3d;

namespace {

StampedPose pose_at(TimeSec stamp, double x) {
	StampedPose pose;
	pose.stamp = stamp;
	pose.pose.position = Eigen::Vector3d(x, 0, 0);
	return pose;
}

ImageFrame depth_at(TimeSec stamp) {
	return ImageFrame{.stamp = stamp, .frame_id = FrameId::DepthCamera, .image = cv::Mat(2, 3, CV_32FC1, cv::Scalar(1.0))};
}

void publish_calibration_and_mount(SensorTopics& sensors) {
	CameraCalibration calibration;
	calibration.frame_id = FrameId::DepthCamera;
	sensors.depth_calibration.publish(calibration);
	StampedPose mount;
	mount.parent_frame = FrameId::Base;
	mount.child_frame = FrameId::DepthCamera;
	mount.pose.position = Eigen::Vector3d(0.1, 0, 0);
	sensors.base_to_depth.publish(mount);
}

}

TEST(FrameSynchronizer, PoseIsInterpolatedAtTheDepthStamp) {
	SensorTopics sensors;
	FrameSynchronizer sync(sensors);
	publish_calibration_and_mount(sensors);
	sensors.pose.publish(pose_at(1.0, 0.0));
	sensors.pose.publish(pose_at(2.0, 4.0));
	sensors.depth.publish(depth_at(1.25));

	const Msg<SensorFrame> frame = sync.next(100ms);
	ASSERT_NE(frame, nullptr);
	EXPECT_DOUBLE_EQ(frame->stamp, 1.25);
	EXPECT_DOUBLE_EQ(frame->drone_pose.stamp, 1.25);
	EXPECT_TRUE(frame->drone_pose.pose.position.isApprox(Eigen::Vector3d(1, 0, 0)));
	EXPECT_EQ(frame->depth.image.at<float>(0, 0), 1.0f);
	EXPECT_EQ(frame->depth_calibration.frame_id, FrameId::DepthCamera);
	EXPECT_EQ(frame->base_to_depth.child_frame, FrameId::DepthCamera);
	EXPECT_EQ(sync.unsynced_count(), 0u);
}

TEST(FrameSynchronizer, WaitsForAPoseThatArrivesAfterTheImage) {
	SensorTopics sensors;
	SyncConfig config;
	config.pose_wait = 1000ms;
	FrameSynchronizer sync(sensors, config);
	publish_calibration_and_mount(sensors);
	sensors.pose.publish(pose_at(1.0, 0.0));
	/// newer than every pose so far
	sensors.depth.publish(depth_at(1.5));
	std::thread late_pose([&] {
		std::this_thread::sleep_for(50ms);
		sensors.pose.publish(pose_at(2.0, 2.0));
	});
	const Msg<SensorFrame> frame = sync.next(100ms);
	late_pose.join();
	ASSERT_NE(frame, nullptr);
	EXPECT_TRUE(frame->drone_pose.pose.position.isApprox(Eigen::Vector3d(1, 0, 0)));
}

TEST(FrameSynchronizer, ImageOlderThanThePoseHistoryIsDropped) {
	SensorTopics sensors;
	FrameSynchronizer sync(sensors);
	publish_calibration_and_mount(sensors);
	sensors.pose.publish(pose_at(5.0, 0.0));
	sensors.depth.publish(depth_at(1.0));
	EXPECT_EQ(sync.next(100ms), nullptr);
	EXPECT_EQ(sync.unsynced_count(), 1u);
}

TEST(FrameSynchronizer, ImageWithoutCalibrationIsDropped) {
	SensorTopics sensors;
	FrameSynchronizer sync(sensors);
	sensors.pose.publish(pose_at(1.0, 0.0));
	sensors.pose.publish(pose_at(2.0, 0.0));
	sensors.depth.publish(depth_at(1.5));
	EXPECT_EQ(sync.next(100ms), nullptr);
	EXPECT_EQ(sync.unsynced_count(), 1u);
}

TEST(FrameSynchronizer, CloseWakesABlockedNext) {
	SensorTopics sensors;
	FrameSynchronizer sync(sensors);
	std::thread closer([&] {
		std::this_thread::sleep_for(50ms);
		sync.close();
	});
	const auto begin = std::chrono::steady_clock::now();
	EXPECT_EQ(sync.next(5s), nullptr);
	closer.join();
	EXPECT_LT(std::chrono::steady_clock::now() - begin, 1s);
	EXPECT_EQ(sync.unsynced_count(), 0u);
}
