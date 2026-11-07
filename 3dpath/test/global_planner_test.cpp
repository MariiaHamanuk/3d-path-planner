#include "pipeline/clients.hpp"
#include "fake_map_view.hpp"
#include "planning/global_planner.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>

using namespace path3d;
using path3d::test::FakeView;
using path3d::test::keyframe_update;
using Planner = GlobalPlanner<FakeView, ReplanRequest, GlobalPath>;

static_assert(std::is_abstract_v<Planner>);
static_assert(PlanRequest<ReplanRequest>);
static_assert(PlanAnswer<GlobalPath>);
static_assert(!PlanAnswer<ReplanRequest>);

namespace {

/**
 * @brief Two-point plan from the request position to a fixed goal; Failed if the goal is below
 *   ground.
 */
class StraightLinePlanner : public Planner {
 public:
  StraightLinePlanner(MapClient<FakeView>& map, UavClient& uav, Eigen::Vector3d goal)
      : Planner(map, uav), m_goal(goal) {}
  GlobalPath plan(const ReplanRequest& request) override {
    /// stable for the whole plan
    const FakeView& map = m_map.map();
    GlobalPath path;
    /// the plan belongs to the map it was made from
    path.stamp = map.stamp;
    if (m_goal.z() < 0.0) {
      path.status = PlanStatus::Failed;
      return path;
    }
    path.waypoints = {request.position, m_goal};
    return path;
  }

  std::optional<Eigen::Vector3d> uav_position() const { return m_uav.position(); }

 private:
  Eigen::Vector3d m_goal;
};

struct Fixture {
  Topic<FakeView::Update> maps;
  Topic<StampedPose> poses;
  MapClient<FakeView> map_client{maps};
  UavClient uav{poses};
};

}

TEST(GlobalPlanner, PlanStartsAtTheRequestPosition) {
  Fixture f;
  f.maps.publish(keyframe_update(1, {}, 4.0));
  ReplanRequest request;
  request.position = Eigen::Vector3d(2, 0, 1);
  std::unique_ptr<Planner> planner =
      std::make_unique<StraightLinePlanner>(f.map_client, f.uav, Eigen::Vector3d(5, 0, 1));
  ASSERT_TRUE(planner->prepare());
  const GlobalPath plan = planner->plan(request);
  EXPECT_EQ(plan.status, PlanStatus::Ok);
  EXPECT_TRUE(plan.waypoints.front().isApprox(Eigen::Vector3d(2, 0, 1)));
  EXPECT_EQ(plan.stamp, 4.0);
  EXPECT_EQ(plan.frame, FrameId::World);
}

TEST(GlobalPlanner, ReadsTheCurrentDronePositionThroughUavClient) {
  Fixture f;
  StraightLinePlanner planner(f.map_client, f.uav, Eigen::Vector3d(5, 0, 1));
  EXPECT_FALSE(planner.uav_position().has_value());
  StampedPose pose;
  pose.pose.position = Eigen::Vector3d(3, 1, 2);
  f.poses.publish(pose);
  ASSERT_TRUE(planner.uav_position().has_value());
  EXPECT_TRUE(planner.uav_position()->isApprox(Eigen::Vector3d(3, 1, 2)));
}

TEST(GlobalPlanner, NotReadyWithoutAMap) {
  Fixture f;
  StraightLinePlanner planner(f.map_client, f.uav, Eigen::Vector3d(5, 0, 1));
  EXPECT_FALSE(planner.prepare());
}

TEST(GlobalPlanner, NoPathIsAnsweredWithFailedStatus) {
  Fixture f;
  f.maps.publish(keyframe_update(1));
  StraightLinePlanner planner(f.map_client, f.uav, Eigen::Vector3d(1, 0, -1));
  ASSERT_TRUE(planner.prepare());
  const GlobalPath plan = planner.plan(ReplanRequest{});
  EXPECT_EQ(plan.status, PlanStatus::Failed);
  EXPECT_TRUE(plan.waypoints.empty());
}
