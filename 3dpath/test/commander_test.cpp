#include "fake_vehicle.hpp"
#include "pipeline/stages.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <functional>
#include <thread>

using namespace std::chrono_literals;
using namespace path3d;
using path3d::test::FakeVehicle;

namespace {

bool wait_for(const std::function<bool()>& done, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done()) {
    if (std::chrono::steady_clock::now() > deadline) return false;
    std::this_thread::sleep_for(5ms);
  }
  return true;
}

/**
 * @brief Publishes the drone pose every 5 ms with a stamp that advances like the sim clock.
 */
class PoseFeed {
 public:
  PoseFeed(Topic<StampedPose>& poses, Eigen::Vector3d position)
      : m_poses(poses), m_position(position), m_start(std::chrono::steady_clock::now()) {
    m_thread = std::thread([this] {
      while (!m_stop) {
        StampedPose pose;
        pose.stamp = now();
        pose.pose.position = m_position;
        m_poses.publish(pose);
        std::this_thread::sleep_for(5ms);
      }
    });
  }
  ~PoseFeed() { pause(); }
  /// the pose stops arriving
  void pause() {
    m_stop = true;
    if (m_thread.joinable()) m_thread.join();
  }
  TimeSec now() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - m_start).count();
  }

 private:
  Topic<StampedPose>& m_poses;
  Eigen::Vector3d m_position;
  std::chrono::steady_clock::time_point m_start;
  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

VehicleState controllable() {
  return {.stamp = 0.0, .connected = true, .armed = true, .frames_ready = true,
          .mode = FlightMode::External};
}

/// Straight line along x with x = t (so a sample's x equals its stamp), from `begin` to `end`.
Trajectory line(TimeSec planned_at, TimeSec begin, TimeSec end) {
  Trajectory trajectory;
  trajectory.stamp = planned_at;
  for (TimeSec t = begin; t <= end + 1e-9; t += 0.1) {
    Setpoint point;
    point.stamp = t;
    point.position = Eigen::Vector3d(t, 0, 2);
    point.velocity = Eigen::Vector3d(1, 0, 0);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

struct Fixture {
  Topic<Trajectory> trajectories;
  Topic<VehicleState> state;
  Topic<StampedPose> poses;
  FakeVehicle vehicle{state};
  PoseFeed feed{poses, Eigen::Vector3d(3, 4, 5)};
  CommanderStage commander;

  explicit Fixture(CommanderConfig config = {.rate_hz = 100.0})
      : commander(vehicle, trajectories, state, poses, config) {}
};

}

TEST(CommanderStage, SendsNothingUnlessTheVehicleIsControllable) {
  Fixture f;
  f.trajectories.publish(line(f.feed.now(), f.feed.now(), f.feed.now() + 10.0));
  VehicleState disconnected = controllable();
  disconnected.connected = false;
  VehicleState disarmed = controllable();
  disarmed.armed = false;
  VehicleState frames_unknown = controllable();
  frames_unknown.frames_ready = false;
  VehicleState holding = controllable();
  holding.mode = FlightMode::Hold;
  f.commander.start();
  for (const VehicleState& state : {VehicleState{}, disconnected, disarmed, frames_unknown, holding}) {
    f.vehicle.report(state);
    std::this_thread::sleep_for(60ms);
  }
  f.commander.stop();
  EXPECT_TRUE(f.vehicle.sent().empty());
}

TEST(CommanderStage, SendsNothingBeforeTheFirstTrajectory) {
  Fixture f;
  f.vehicle.report(controllable());
  f.commander.start();
  std::this_thread::sleep_for(100ms);
  f.commander.stop();
  /// manual control keeps working until a planner engages
  EXPECT_TRUE(f.vehicle.sent().empty());
}

TEST(CommanderStage, FollowsTheTrajectoryAtThePoseTime) {
  Fixture f;
  f.vehicle.report(controllable());
  f.commander.start();
  /// Replanned every 100 ms, like a local planner would.
  for (int i = 0; i < 4; ++i) {
    const TimeSec now = f.feed.now();
    f.trajectories.publish(line(now, now - 0.1, now + 3.0));
    std::this_thread::sleep_for(100ms);
  }
  f.commander.stop();
  const auto sent = f.vehicle.sent();
  ASSERT_GE(sent.size(), 10u);
  EXPECT_EQ(f.commander.hold_count(), 0u);
  for (const Setpoint& setpoint : sent) {
    /// sampled where the trajectory says
    EXPECT_NEAR(setpoint.position.x(), setpoint.stamp, 1e-6);
    EXPECT_TRUE(setpoint.velocity.isApprox(Eigen::Vector3d(1, 0, 0)));
  }
}

TEST(CommanderStage, HoldsAtTheLatchedPoseWhenTheTrajectoryRunsOut) {
  Fixture f;
  f.vehicle.report(controllable());
  const TimeSec now = f.feed.now();
  /// over in 100 ms
  f.trajectories.publish(line(now, now, now + 0.1));
  f.commander.start();
  ASSERT_TRUE(wait_for([&] { return f.commander.hold_count() >= 10; }, 2s));
  f.commander.stop();
  const auto sent = f.vehicle.sent();
  const Setpoint& last = sent.back();
  /// where the drone was
  EXPECT_TRUE(last.position.isApprox(Eigen::Vector3d(3, 4, 5)));
  EXPECT_TRUE(last.velocity.isZero());
  EXPECT_TRUE(last.acceleration.isZero());
}

TEST(CommanderStage, HoldsWhenTheTrajectoryIsTooOld) {
  Fixture f;
  f.vehicle.report(controllable());
  const TimeSec now = f.feed.now();
  /// planner stopped a second ago
  f.trajectories.publish(line(now - 1.0, now - 1.0, now + 10.0));
  f.commander.start();
  ASSERT_TRUE(wait_for([&] { return f.commander.hold_count() >= 5; }, 2s));
  f.commander.stop();
  EXPECT_EQ(f.commander.sent_count(), 0u);
}

TEST(CommanderStage, HoldsWhenThePoseStopsArriving) {
  Fixture f({.rate_hz = 100.0, .max_trajectory_age = 100.0, .max_pose_silence = 50ms});
  f.vehicle.report(controllable());
  const TimeSec now = f.feed.now();
  f.trajectories.publish(line(now, now, now + 100.0));
  f.commander.start();
  ASSERT_TRUE(wait_for([&] { return f.commander.sent_count() >= 5; }, 2s));
  f.feed.pause();
  const std::size_t sent_before = f.commander.sent_count();
  ASSERT_TRUE(wait_for([&] { return f.commander.hold_count() >= 5; }, 2s));
  f.commander.stop();
  /// at most until the silence is noticed
  EXPECT_LE(f.commander.sent_count(), sent_before + 10);
  EXPECT_TRUE(f.vehicle.sent().back().velocity.isZero());
}

TEST(CommanderStage, StopsSendingWhenTheModeLeavesExternal) {
  Fixture f;
  f.vehicle.report(controllable());
  const TimeSec now = f.feed.now();
  f.trajectories.publish(line(now, now, now + 100.0));
  f.commander.start();
  ASSERT_TRUE(wait_for([&] { return f.vehicle.sent().size() >= 5; }, 2s));
  VehicleState pilot_took_over = controllable();
  pilot_took_over.mode = FlightMode::Manual;
  f.vehicle.report(pilot_took_over);
  /// let the commander see it
  std::this_thread::sleep_for(50ms);
  f.vehicle.clear();
  std::this_thread::sleep_for(100ms);
  f.commander.stop();
  EXPECT_TRUE(f.vehicle.sent().empty());
}
