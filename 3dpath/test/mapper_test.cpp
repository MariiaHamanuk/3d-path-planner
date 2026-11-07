#include "fake_map_view.hpp"
#include "mapping/mapper.hpp"

#include <gtest/gtest.h>

#include <type_traits>

using namespace path3d;
using path3d::test::FakeMapper;
using path3d::test::FakeView;

static_assert(std::is_abstract_v<Mapper<FakeView>>);

namespace {
/// Pipeline code that only knows the interface.
void feed(Mapper<FakeView>& mapper, double stamp) {
  SensorFrame frame;
  frame.stamp = stamp;
  mapper.integrate(frame);
}
}

TEST(Mapper, DiffCarriesTheChangesSinceTheLastUpdate) {
  FakeMapper mapper;
  EXPECT_EQ(mapper.version(), 0u);
  EXPECT_FALSE(mapper.make_update(false).has_value());
  feed(mapper, 1.0);
  feed(mapper, 2.0);
  const auto first = mapper.make_update(false);
  ASSERT_TRUE(first.has_value());
  EXPECT_FALSE(first->keyframe);
  EXPECT_EQ(first->from_version, 0u);
  EXPECT_EQ(first->to_version, 2u);
  EXPECT_EQ(first->cells, (std::vector<int>{1, 2}));
  EXPECT_DOUBLE_EQ(first->stamp, 2.0);
  EXPECT_FALSE(mapper.make_update(false).has_value());
  feed(mapper, 3.0);
  const auto second = mapper.make_update(false);
  ASSERT_TRUE(second.has_value());
  /// builds on the previous update
  EXPECT_EQ(second->from_version, 2u);
  EXPECT_EQ(second->to_version, 3u);
  EXPECT_EQ(second->cells, (std::vector<int>{3}));
}

TEST(Mapper, KeyframeCarriesTheWholeMapEvenWithoutChanges) {
  FakeMapper mapper;
  feed(mapper, 1.0);
  feed(mapper, 2.0);
  ASSERT_TRUE(mapper.make_update(false).has_value());
  const auto keyframe = mapper.make_update(true);
  ASSERT_TRUE(keyframe.has_value());
  EXPECT_TRUE(keyframe->keyframe);
  EXPECT_EQ(keyframe->to_version, 2u);
  EXPECT_EQ(keyframe->cells, (std::vector<int>{1, 2}));
  feed(mapper, 3.0);
  /// a diff after a keyframe builds on it
  EXPECT_EQ(mapper.make_update(false)->from_version, 2u);
}

TEST(Mapper, ACopyBuiltFromUpdatesMatchesTheMapper) {
  FakeMapper mapper;
  FakeView copy;
  feed(mapper, 1.0);
  copy.apply(*mapper.make_update(true));
  feed(mapper, 2.0);
  feed(mapper, 3.0);
  copy.apply(*mapper.make_update(false));
  EXPECT_EQ(copy.cells(), mapper.cells());
  EXPECT_EQ(copy.version, mapper.version());
  EXPECT_DOUBLE_EQ(copy.stamp, 3.0);
}
