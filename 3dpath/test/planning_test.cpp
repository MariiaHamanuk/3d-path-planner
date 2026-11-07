#include "fake_map_view.hpp"
#include "planning/global_planner.hpp"
#include "planning/local_planner.hpp"
#include "pipeline/stages.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>

using namespace std::chrono_literals;
using namespace path3d;
using path3d::test::FakeView;
using path3d::test::keyframe_update;

namespace {

bool wait_for(const std::function<bool()>& done, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done()) {
    if (std::chrono::steady_clock::now() > deadline) return false;
    std::this_thread::sleep_for(5ms);
  }
  return true;
}

using Global = GlobalPlanner<FakeView, ReplanRequest, GlobalPath>;
using Local = LocalPlanner<FakeView, GlobalPath, ReplanRequest>;

/**
 * @brief Makes a two-point plan from the request position, and records the start of each call.
 */
class StubGlobalPlanner : public Global {
 public:
  StubGlobalPlanner(MapClient<FakeView>& map, UavClient& uav) : Global(map, uav) {}
  GlobalPath plan(const ReplanRequest& request) override {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_starts.push_back(request.position);
    }
    GlobalPath path;
    path.stamp = m_map.map().stamp;
    path.waypoints = {request.position, Eigen::Vector3d(5, 0, 1)};
    return path;
  }
  std::vector<Eigen::Vector3d> starts() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_starts;
  }

 private:
  mutable std::mutex m_mutex;
  std::vector<Eigen::Vector3d> m_starts;
};

/**
 * @brief Never finds a path.
 */
class FailingGlobalPlanner : public Global {
 public:
  FailingGlobalPlanner(MapClient<FakeView>& map, UavClient& uav) : Global(map, uav) {}
  GlobalPath plan(const ReplanRequest&) override {
    ++m_call_count;
    GlobalPath path;
    path.status = PlanStatus::Failed;
    return path;
  }
  int call_count() const { return m_call_count; }

 private:
  std::atomic<int> m_call_count{0};
};

/**
 * @brief Takes plans; once it has one and knows its position, asks for a single replan from there.
 */
class StubLocalPlanner : public Local {
 public:
  StubLocalPlanner(MapClient<FakeView>& map, UavClient& uav) : Local(map, uav) {}
  LocalStep<ReplanRequest> plan(const GlobalPath* new_route) override {
    if (new_route) ++m_path_count;
    const auto position = m_uav.position();
    if (m_path_count == 0 || m_requested || !position) return {};
    m_requested = true;
    ReplanRequest request;
    request.reason = ReplanReason::Blocked;
    request.position = *position;
    return {request, std::nullopt};
  }
  int path_count() const { return m_path_count; }
  bool requested() const { return m_requested; }

 private:
  std::atomic<int> m_path_count{0};
  std::atomic<bool> m_requested{false};
};

/**
 * @brief Asks for a replan on every cycle and records the request ids of the plans it is given.
 */
class AlwaysRequestingPlanner : public Local {
 public:
  AlwaysRequestingPlanner(MapClient<FakeView>& map, UavClient& uav) : Local(map, uav) {}
  LocalStep<ReplanRequest> plan(const GlobalPath* new_route) override {
    if (new_route) {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_seen.push_back(new_route->request_id);
    }
    return {ReplanRequest{}, std::nullopt};
  }
  std::vector<std::uint64_t> seen() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_seen;
  }

 private:
  mutable std::mutex m_mutex;
  std::vector<std::uint64_t> m_seen;
};

GlobalPlanningConfig global_config(std::chrono::milliseconds request_wait) {
  GlobalPlanningConfig config;
  config.request_wait = request_wait;
  return config;
}

LocalPlanningConfig local_config(double rate_hz, std::chrono::milliseconds request_timeout =
                                                     LocalPlanningConfig{}.request_timeout) {
  LocalPlanningConfig config;
  config.rate_hz = rate_hz;
  config.request_timeout = request_timeout;
  return config;
}

ReplanRequest request_at(const Eigen::Vector3d& position, std::uint64_t id = 0) {
  ReplanRequest request;
  request.id = id;
  request.reason = ReplanReason::NoPath;
  request.position = position;
  return request;
}

StampedPose pose_at(const Eigen::Vector3d& position) {
  StampedPose pose;
  pose.pose.position = position;
  return pose;
}

GlobalPath answer_to(std::uint64_t request_id) {
  GlobalPath path;
  path.request_id = request_id;
  return path;
}

}

TEST(GlobalPlanningStage, AnswersTheInitialRequestOnceWithItsId) {
  Topic<FakeView::Update> maps;
  Topic<StampedPose> poses;
  MapClient<FakeView> map_client(maps);
  UavClient uav(poses);
  Pipe<GlobalPath, ReplanRequest> pipe;
  auto routes = pipe.down().subscribe_queue(8);
  FailingGlobalPlanner planner(map_client, uav);
  GlobalPlanningStage<FakeView, GlobalPath, ReplanRequest> stage(
      planner, pipe, request_at(Eigen::Vector3d(0, 0, 1), 3), global_config(20ms));
  maps.publish(keyframe_update(1));
  stage.start();
  ASSERT_TRUE(wait_for([&] { return planner.call_count() >= 1; }, 2s));
  /// many more loop iterations
  std::this_thread::sleep_for(100ms);
  stage.stop();
  /// planned once, no requests arrived
  EXPECT_EQ(planner.call_count(), 1);
  const Msg<GlobalPath> answer = routes->pop(10ms);
  /// a failure is answered too
  ASSERT_NE(answer, nullptr);
  EXPECT_EQ(answer->request_id, 3u);
  EXPECT_EQ(answer->status, PlanStatus::Failed);
  /// and nothing else is published
  EXPECT_EQ(routes->pop(10ms), nullptr);
}

TEST(Planning, LocalRequestTriggersReplanFromItsPosition) {
  Topic<FakeView::Update> maps;
  Topic<StampedPose> poses;
  Pipe<GlobalPath, ReplanRequest> pipe;
  /// a client belongs to one thread
  MapClient<FakeView> global_map(maps);
  MapClient<FakeView> local_map(maps);
  UavClient uav(poses);
  StubGlobalPlanner global(global_map, uav);
  StubLocalPlanner local(local_map, uav);
  GlobalPlanningStage<FakeView, GlobalPath, ReplanRequest> global_stage(
      global, pipe, request_at(Eigen::Vector3d(0, 0, 1)), global_config(20ms));
  Topic<Trajectory> trajectories;
  LocalPlanningStage<FakeView, GlobalPath, ReplanRequest> local_stage(local, pipe, trajectories,
                                                                      local_config(50.0));

  maps.publish(keyframe_update(1));
  poses.publish(pose_at(Eigen::Vector3d(2, 0, 1)));
  global_stage.start();
  local_stage.start();

  const auto replanned_from = [&] {
    const auto starts = global.starts();
    return std::any_of(starts.begin(), starts.end(),
                       [](const Eigen::Vector3d& s) { return s.isApprox(Eigen::Vector3d(2, 0, 1)); });
  };
  ASSERT_TRUE(wait_for(replanned_from, 2s));
  /// the replan reached the local
  ASSERT_TRUE(wait_for([&] { return local.path_count() >= 2; }, 2s));
  global_stage.stop();
  local_stage.stop();

  EXPECT_TRUE(local.requested());
  EXPECT_EQ(local_stage.sent_request_count(), 1u);
  EXPECT_EQ(local_stage.stale_route_count(), 0u);
  /// initial plan from the start, then the replan
  ASSERT_EQ(global.starts().size(), 2u);
  EXPECT_TRUE(global.starts().front().isApprox(Eigen::Vector3d(0, 0, 1)));
}

TEST(LocalPlanningStage, SendsOneRequestAtATimeAndDropsStaleAnswers) {
  Topic<FakeView::Update> maps;
  Topic<StampedPose> poses;
  Pipe<GlobalPath, ReplanRequest> pipe;
  auto requests = pipe.up().subscribe_queue(16);
  MapClient<FakeView> map_client(maps);
  UavClient uav(poses);
  AlwaysRequestingPlanner local(map_client, uav);
  Topic<Trajectory> trajectories;
  LocalPlanningStage<FakeView, GlobalPath, ReplanRequest> stage(local, pipe, trajectories,
                                                                local_config(100.0));
  maps.publish(keyframe_update(1));
  poses.publish(pose_at(Eigen::Vector3d(0, 0, 1)));
  stage.start();

  const Msg<ReplanRequest> first = requests->pop(2s);
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first->id, 1u);
  /// ~10 more cycles, each one asking again
  std::this_thread::sleep_for(100ms);
  /// still waiting for the answer to request 1
  EXPECT_EQ(stage.sent_request_count(), 1u);
  EXPECT_EQ(requests->pop(0ms), nullptr);

  /// answers an older request
  pipe.down().publish(answer_to(0));
  ASSERT_TRUE(wait_for([&] { return stage.stale_route_count() == 1; }, 2s));
  EXPECT_TRUE(local.seen().empty());

  pipe.down().publish(answer_to(1));
  const Msg<ReplanRequest> second = requests->pop(2s);
  stage.stop();
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(second->id, 2u);
  EXPECT_EQ(local.seen(), std::vector<std::uint64_t>{1});
}

TEST(LocalPlanningStage, UnansweredRequestTimesOutAndIsSentAgain) {
  Topic<FakeView::Update> maps;
  Topic<StampedPose> poses;
  Pipe<GlobalPath, ReplanRequest> pipe;
  auto requests = pipe.up().subscribe_queue(16);
  MapClient<FakeView> map_client(maps);
  UavClient uav(poses);
  AlwaysRequestingPlanner local(map_client, uav);
  Topic<Trajectory> trajectories;
  LocalPlanningStage<FakeView, GlobalPath, ReplanRequest> stage(local, pipe, trajectories,
                                                                local_config(100.0, 100ms));
  maps.publish(keyframe_update(1));
  poses.publish(pose_at(Eigen::Vector3d(0, 0, 1)));
  stage.start();

  const Msg<ReplanRequest> first = requests->pop(2s);
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first->id, 1u);
  const Msg<ReplanRequest> second = requests->pop(2s);  ///< no answer ever comes
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(second->id, 2u);
  EXPECT_GE(stage.request_timeout_count(), 1u);

  /// the late answer to the lost request is stale
  pipe.down().publish(answer_to(1));
  ASSERT_TRUE(wait_for([&] { return stage.stale_route_count() == 1; }, 2s));
  stage.stop();
  EXPECT_TRUE(local.seen().empty());
}

TEST(LocalPlanningStage, PublishesThePlannersTrajectories) {
  class StraightAheadPlanner : public Local {
   public:
    StraightAheadPlanner(MapClient<FakeView>& map, UavClient& uav) : Local(map, uav) {}
    LocalStep<ReplanRequest> plan(const GlobalPath*) override {
      Trajectory trajectory;
      trajectory.stamp = static_cast<double>(++m_plan_count);
      trajectory.points = {Setpoint{}, Setpoint{}};
      return {std::nullopt, trajectory};
    }

   private:
    int m_plan_count = 0;
  };
  Topic<FakeView::Update> maps;
  Topic<StampedPose> poses;
  Pipe<GlobalPath, ReplanRequest> pipe;
  Topic<Trajectory> trajectories;
  auto received = trajectories.subscribe_queue(1000);
  MapClient<FakeView> map_client(maps);
  UavClient uav(poses);
  StraightAheadPlanner local(map_client, uav);
  LocalPlanningStage<FakeView, GlobalPath, ReplanRequest> stage(local, pipe, trajectories,
                                                                local_config(100.0));
  maps.publish(keyframe_update(1));
  poses.publish(pose_at(Eigen::Vector3d(0, 0, 1)));
  stage.start();
  ASSERT_TRUE(wait_for([&] { return received->size() >= 3; }, 2s));
  stage.stop();
  EXPECT_EQ(stage.published_trajectory_count(), received->size());
  const Msg<Trajectory> first = received->pop(0ms);
  ASSERT_NE(first, nullptr);
  EXPECT_DOUBLE_EQ(first->stamp, 1.0);
  EXPECT_EQ(first->points.size(), 2u);
}
