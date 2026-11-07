#include "core/communication.hpp"
#include "fake_map_view.hpp"
#include "mapping/map_view.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace path3d;
using path3d::test::FakeView;

static_assert(MapViewType<FakeView>);
/// views can be published on a Topic
static_assert(Stamped<FakeView>);
static_assert(!MapViewType<int>);
/// no occupancy query, so no planner that needs one
static_assert(!OccupancyMap<FakeView>);
static_assert(!GridMap<FakeView>);
static_assert(!DistanceMap<FakeView>);
static_assert(!GradientMap<FakeView>);

/// Update concepts: the transport needs only these fields, and a map with an update type and apply().
static_assert(MapUpdate<FakeView::Update>);
static_assert(ReplicatedMap<FakeView>);
static_assert(!MapUpdate<int>);
static_assert(!ReplicatedMap<int>);
namespace {
struct NoVersions {
	double stamp;
	bool keyframe;
};
static_assert(!MapUpdate<NoVersions>);

class NoUpdateType : public MapView {};  ///< a view without an Update type
static_assert(!ReplicatedMap<NoUpdateType>);

/**
 * @brief apply() must return void
 */
class WrongApply : public MapView {
public:
	struct Update {
		double stamp;
		std::uint64_t from_version, to_version;
		bool keyframe;
	};
	bool apply(const Update&) { return true; }
};
static_assert(!ReplicatedMap<WrongApply>);

/**
 * @brief Copy policy: every update is a keyframe holding a full copy, so any copyable map
 *   qualifies.
 */
class CopyMap : public MapView {
public:
	struct Update {
		TimeSec stamp = 0.0;
		std::uint64_t from_version = 0;
		std::uint64_t to_version = 0;
		bool keyframe = true;
		std::shared_ptr<const CopyMap> snapshot;
	};
	CopyMap() = default;
	void apply(const Update& update) { *this = *update.snapshot; }
	int payload = 0;
};
static_assert(ReplicatedMap<CopyMap>);
}

TEST(MapView, BaseCarriesStampVersionAndFrame) {
	FakeView view;
	view.stamp = 2.5;
	view.version = 7;
	EXPECT_EQ(view.frame, FrameId::World);
	EXPECT_DOUBLE_EQ(view.stamp, 2.5);
	EXPECT_EQ(view.version, 7u);
}

TEST(MapView, CopyPolicyReplacesTheWholeMap) {
	auto snapshot = std::make_shared<CopyMap>();
	snapshot->payload = 42;
	snapshot->stamp = 3.0;
	snapshot->version = 9;
	CopyMap copy;
	copy.apply({.stamp = 3.0, .to_version = 9, .snapshot = snapshot});
	EXPECT_EQ(copy.payload, 42);
	EXPECT_EQ(copy.version, 9u);
}
