#include "pipeline/clients.hpp"
#include "fake_map_view.hpp"
#include "pipeline/stages.hpp"

#include <gtest/gtest.h>

#include <functional>
#include <mutex>
#include <vector>

using namespace std::chrono_literals;
using namespace path3d;
using path3d::test::FakeMapper;
using path3d::test::FakeView;

namespace {

/**
 * @brief A FakeMapper that also records the stamps and drone x positions it integrates.
 */
class CollectingMapper : public FakeMapper {
 public:
  void integrate(const SensorFrame& frame) override {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_stamps.push_back(frame.stamp);
      m_xs.push_back(frame.drone_pose.pose.position.x());
    }
    FakeMapper::integrate(frame);
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stamps.size();
  }
  std::vector<double> stamps() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stamps;
  }
  std::vector<double> xs() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_xs;
  }

 private:
  mutable std::mutex m_mutex;
  std::vector<double> m_stamps;
  std::vector<double> m_xs;
};

MappingConfig mapping_config(std::size_t depth_queue,
                             TimeSec publish_period = MappingConfig{}.publish_period,
                             std::chrono::milliseconds keyframe_period =
                                 MappingConfig{}.keyframe_period) {
  MappingConfig config;
  config.sync.depth_queue = depth_queue;
  config.publish_period = publish_period;
  config.keyframe_period = keyframe_period;
  return config;
}

bool wait_for(const std::function<bool()>& done, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done()) {
    if (std::chrono::steady_clock::now() > deadline) return false;
    std::this_thread::sleep_for(5ms);
  }
  return true;
}

/// Plays the role of a Reader: the drone flies along x at 1 m/s, with poses at 100 Hz and depth
/// images at `depth_hz`, both over [0, duration] seconds of sim time.
void publish_flight(SensorTopics& sensors, double duration, double depth_hz) {
  sensors.depth_calibration.publish(CameraCalibration{});
  sensors.base_to_depth.publish(StampedPose{});
  for (int i = 0; i <= static_cast<int>(duration * 100); ++i) {
    StampedPose pose;
    pose.stamp = 0.01 * i;
    pose.pose.position = Eigen::Vector3d(pose.stamp, 0, 0);
    sensors.pose.publish(pose);
  }
  for (int i = 0; i * (1.0 / depth_hz) < duration; ++i) {
    sensors.depth.publish(ImageFrame{.stamp = 0.005 + i / depth_hz, .frame_id = FrameId::DepthCamera,
                                     .image = cv::Mat(2, 3, CV_32FC1, cv::Scalar(1.0))});
  }
}

}

TEST(Pipeline, DepthFlowsToMapperInOrderWithInterpolatedPoses) {
  SensorTopics sensors;
  Topic<FakeView::Update> maps;
  CollectingMapper mapper;
  /// a reader of the map, created before the mapper publishes
  MapClient<FakeView> client(maps);
  MappingStage<FakeView> mapping(sensors, mapper, maps, mapping_config(64));
  mapping.start();
  /// poses and images at different rates and times
  publish_flight(sensors, 1.0, 15.0);

  ASSERT_TRUE(wait_for([&] { return mapper.size() == 15; }, 3s));
  /// The reader's copy catches up with the mapper through the updates alone.
  ASSERT_TRUE(wait_for([&] { client.sync(); return client.map().version == 15; }, 3s));
  mapping.stop();
  EXPECT_TRUE(client.ready());
  EXPECT_EQ(client.map().cells(), mapper.cells());
  EXPECT_EQ(client.gap_count(), 0u);

  const auto stamps = mapper.stamps();
  const auto xs = mapper.xs();
  for (std::size_t i = 1; i < stamps.size(); ++i) EXPECT_GT(stamps[i], stamps[i - 1]);
  for (std::size_t i = 0; i < stamps.size(); ++i) EXPECT_NEAR(xs[i], stamps[i], 1e-9);
  EXPECT_EQ(mapping.integrated_count(), mapper.size());
  EXPECT_EQ(mapping.unsynced_count(), 0u);
}

TEST(Pipeline, NothingIsIntegratedWithoutPoses) {
  SensorTopics sensors;
  Topic<FakeView::Update> maps;
  CollectingMapper mapper;
  MappingStage<FakeView> mapping(sensors, mapper, maps);
  mapping.start();
  sensors.depth_calibration.publish(CameraCalibration{});
  sensors.base_to_depth.publish(StampedPose{});
  ImageFrame depth;
  depth.stamp = 1.0;
  sensors.depth.publish(depth);
  ASSERT_TRUE(wait_for([&] { return mapping.unsynced_count() == 1; }, 2s));
  mapping.stop();
  EXPECT_EQ(mapping.integrated_count(), 0u);
}

TEST(Pipeline, SlowMapperDropsOldestInsteadOfFallingBehind) {
  class SlowMapper : public CollectingMapper {
   public:
    void integrate(const SensorFrame& frame) override {
      std::this_thread::sleep_for(30ms);
      CollectingMapper::integrate(frame);
    }
  };
  SensorTopics sensors;
  Topic<FakeView::Update> maps;
  SlowMapper mapper;
  MappingStage<FakeView> mapping(sensors, mapper, maps, mapping_config(2));
  mapping.start();
  /// 30 images at once
  publish_flight(sensors, 2.0, 15.0);
  std::this_thread::sleep_for(300ms);
  mapping.stop();
  EXPECT_GT(mapping.dropped_count(), 0u);
  EXPECT_LT(mapper.size(), 30u);
  const auto stamps = mapper.stamps();
  ASSERT_FALSE(stamps.empty());
  /// kept up with the newest images
  EXPECT_GT(stamps.back(), 1.9);
}

TEST(Pipeline, UpdatesAreThrottledBySimTimeAndFlushedWhenIdle) {
  SensorTopics sensors;
  Topic<FakeView::Update> maps;
  auto updates = maps.subscribe_queue(1000);
  CollectingMapper mapper;
  MappingStage<FakeView> mapping(sensors, mapper, maps, mapping_config(100, 0.2));
  mapping.start();
  /// 50 images over 0.5 s of sim time
  publish_flight(sensors, 0.5, 100.0);

  ASSERT_TRUE(wait_for([&] { return mapper.size() == 50; }, 2s));
  /// idle: the last changes get flushed
  std::this_thread::sleep_for(300ms);
  mapping.stop();
  /// keyframe, ~0.2, ~0.4, then the idle flush
  EXPECT_GE(mapping.published_update_count(), 4u);
  /// not one per image
  EXPECT_LT(mapping.published_update_count(), 10u);
  EXPECT_EQ(updates->size(), mapping.published_update_count());

  /// The first update is a keyframe and every diff builds on the one before it.
  Msg<FakeView::Update> previous = updates->pop(0ms);
  ASSERT_NE(previous, nullptr);
  EXPECT_TRUE(previous->keyframe);
  while (Msg<FakeView::Update> update = updates->pop(0ms)) {
    EXPECT_FALSE(update->keyframe);
    EXPECT_EQ(update->from_version, previous->to_version);
    previous = update;
  }
  /// nothing was left unpublished
  EXPECT_EQ(previous->to_version, 50u);
}

TEST(Pipeline, KeyframesAreRepeatedWhileTheMapIsIdle) {
  SensorTopics sensors;
  Topic<FakeView::Update> maps;
  auto updates = maps.subscribe_queue(1000);
  CollectingMapper mapper;
  MappingStage<FakeView> mapping(sensors, mapper, maps, mapping_config(100, 0.2, 50ms));
  mapping.start();
  /// 10 images, then nothing
  publish_flight(sensors, 0.1, 100.0);
  ASSERT_TRUE(wait_for([&] { return mapper.size() == 10; }, 2s));
  /// the map stays unchanged
  std::this_thread::sleep_for(500ms);
  mapping.stop();

  /// late readers can always catch up
  EXPECT_GE(mapping.published_keyframe_count(), 3u);
  std::size_t keyframes = 0;
  while (Msg<FakeView::Update> update = updates->pop(0ms)) {
    if (update->keyframe) {
      ++keyframes;
      /// a keyframe holds the whole map
      EXPECT_EQ(update->cells.size(), update->to_version);
    }
  }
  EXPECT_EQ(keyframes, mapping.published_keyframe_count());
}

TEST(Pipeline, NothingIsPublishedWhileTheMapIsEmpty) {
  SensorTopics sensors;
  Topic<FakeView::Update> maps;
  auto updates = maps.subscribe_queue(10);
  CollectingMapper mapper;
  MappingStage<FakeView> mapping(sensors, mapper, maps, mapping_config(2, 0.2, 10ms));
  mapping.start();
  /// many keyframe periods, but no map yet
  std::this_thread::sleep_for(300ms);
  mapping.stop();
  EXPECT_EQ(mapping.published_update_count(), 0u);
  EXPECT_EQ(updates->size(), 0u);
}
