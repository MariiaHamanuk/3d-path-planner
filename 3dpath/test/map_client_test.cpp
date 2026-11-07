#include "pipeline/clients.hpp"
#include "fake_map_view.hpp"

#include <gtest/gtest.h>

using namespace path3d;
using path3d::test::diff_update;
using path3d::test::FakeView;
using path3d::test::keyframe_update;

using Updates = Topic<FakeView::Update>;
using Cells = std::vector<int>;

TEST(MapClient, NotReadyUntilTheFirstKeyframe) {
  Updates updates;
  MapClient<FakeView> client(updates);
  EXPECT_FALSE(client.sync());
  /// a diff cannot start a copy
  updates.publish(diff_update(0, 1, {1}));
  EXPECT_FALSE(client.sync());
  /// waiting for the first keyframe is not a gap
  EXPECT_EQ(client.gap_count(), 0u);
  updates.publish(keyframe_update(3, {1, 2, 3}));
  EXPECT_TRUE(client.sync());
  EXPECT_EQ(client.map().cells(), (Cells{1, 2, 3}));
  EXPECT_EQ(client.map().version, 3u);
}

TEST(MapClient, AppliesDiffsInOrder) {
  Updates updates;
  MapClient<FakeView> client(updates);
  updates.publish(keyframe_update(2, {1, 2}));
  updates.publish(diff_update(2, 3, {3}));
  updates.publish(diff_update(3, 5, {4, 5}));
  ASSERT_TRUE(client.sync());
  EXPECT_EQ(client.map().cells(), (Cells{1, 2, 3, 4, 5}));
  EXPECT_EQ(client.map().version, 5u);
  EXPECT_EQ(client.applied_count(), 3u);
}

TEST(MapClient, MapIsStableBetweenSyncCalls) {
  Updates updates;
  MapClient<FakeView> client(updates);
  updates.publish(keyframe_update(1, {1}));
  ASSERT_TRUE(client.sync());
  /// arrives, but is applied only by the next sync()
  updates.publish(diff_update(1, 2, {2}));
  EXPECT_EQ(client.map().cells(), (Cells{1}));
  client.sync();
  EXPECT_EQ(client.map().cells(), (Cells{1, 2}));
}

TEST(MapClient, GapDesynchronizesUntilTheNextKeyframe) {
  Updates updates;
  MapClient<FakeView> client(updates);
  updates.publish(keyframe_update(2, {1, 2}));
  ASSERT_TRUE(client.sync());
  /// the diff 2 -> 4 never arrived
  updates.publish(diff_update(4, 5, {5}));
  EXPECT_FALSE(client.sync());
  EXPECT_EQ(client.gap_count(), 1u);
  /// must not be applied on a stale copy
  updates.publish(diff_update(5, 6, {6}));
  EXPECT_FALSE(client.sync());
  EXPECT_EQ(client.gap_count(), 1u);
  updates.publish(keyframe_update(8, {1, 2, 3, 4, 5, 6, 7, 8}));
  EXPECT_TRUE(client.sync());
  EXPECT_EQ(client.map().cells().size(), 8u);
  updates.publish(diff_update(8, 9, {9}));
  EXPECT_TRUE(client.sync());
  EXPECT_EQ(client.map().cells().size(), 9u);
}

TEST(MapClient, SkipsStraightToTheNewestKeyframe) {
  Updates updates;
  MapClient<FakeView> client(updates);
  updates.publish(keyframe_update(1, {1}));
  updates.publish(diff_update(1, 2, {2}));
  updates.publish(keyframe_update(3, {7, 8, 9}));
  updates.publish(diff_update(3, 4, {10}));
  ASSERT_TRUE(client.sync());
  EXPECT_EQ(client.map().cells(), (Cells{7, 8, 9, 10}));
  /// the superseded updates were not applied
  EXPECT_EQ(client.applied_count(), 2u);
}

TEST(MapClient, ClientsAreIndependent) {
  Updates updates;
  MapClient<FakeView> early(updates);
  updates.publish(keyframe_update(2, {1, 2}));
  /// joins after the keyframe
  MapClient<FakeView> late(updates);
  updates.publish(diff_update(2, 3, {3}));
  EXPECT_TRUE(early.sync());
  /// got only a diff: waits for the next keyframe
  EXPECT_FALSE(late.sync());
  updates.publish(keyframe_update(3, {1, 2, 3}));
  EXPECT_TRUE(late.sync());
  EXPECT_EQ(early.sync() ? early.map().cells() : Cells{}, (Cells{1, 2, 3}));
  EXPECT_EQ(late.map().cells(), (Cells{1, 2, 3}));
}

TEST(MapClient, QueueOverflowCountsAsAGap) {
  Updates updates;
  MapClientConfig config;
  /// holds at most two updates between syncs
  config.queue_capacity = 2;
  MapClient<FakeView> client(updates, config);
  updates.publish(keyframe_update(1, {1}));
  ASSERT_TRUE(client.sync());
  updates.publish(diff_update(1, 2, {2}));
  updates.publish(diff_update(2, 3, {3}));
  updates.publish(diff_update(3, 4, {4}));
  /// the first two are dropped
  updates.publish(diff_update(4, 5, {5}));
  EXPECT_FALSE(client.sync());
  EXPECT_EQ(client.gap_count(), 1u);
}

namespace {
/**
 * @brief A map whose apply() leaves stamp and version alone: the client must set them itself.
 */
class ForgetfulMap : public MapView {
 public:
  struct Update {
    TimeSec stamp = 0.0;
    std::uint64_t from_version = 0;
    std::uint64_t to_version = 0;
    bool keyframe = false;
  };
  ForgetfulMap() = default;
  void apply(const Update&) { ++applied_count; }
  int applied_count = 0;
};
}

TEST(MapClient, SetsStampAndVersionItselfAfterApply) {
  Topic<ForgetfulMap::Update> updates;
  MapClient<ForgetfulMap> client(updates);
  updates.publish(ForgetfulMap::Update{.stamp = 1.5, .from_version = 0, .to_version = 4,
                                       .keyframe = true});
  updates.publish(ForgetfulMap::Update{.stamp = 2.0, .from_version = 4, .to_version = 5,
                                       .keyframe = false});
  ASSERT_TRUE(client.sync());
  /// the diff was accepted: its from_version matched
  EXPECT_EQ(client.map().applied_count, 2);
  EXPECT_EQ(client.map().version, 5u);
  EXPECT_DOUBLE_EQ(client.map().stamp, 2.0);
  EXPECT_EQ(client.gap_count(), 0u);
}
