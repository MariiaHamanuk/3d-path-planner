#include "core/threading.hpp"

#include "core/communication.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace std::chrono_literals;
using path3d::BoundedQueue;
using path3d::Rate;
using path3d::Stage;
using path3d::Topic;

namespace {
struct Tick {
  double stamp;
};
}

namespace {
class CountingStage : public Stage {
 public:
  explicit CountingStage(Topic<Tick>& topic) : m_queue(topic.subscribe_queue(4)) {}
  ~CountingStage() override { stop(); }
  int count() const { return m_count; }

 private:
  void run() override {
    while (running()) {
      if (m_queue->pop(1s)) ++m_count;
    }
  }
  void on_stop() override { m_queue->close(); }
  std::shared_ptr<BoundedQueue<Tick>> m_queue;
  std::atomic<int> m_count{0};
};
}

TEST(Stage, StopsPromptlyEvenWhileBlockedOnAQueue) {
  Topic<Tick> topic;
  CountingStage stage(topic);
  stage.start();
  topic.publish(Tick{1.0});
  std::this_thread::sleep_for(50ms);
  const auto begin = std::chrono::steady_clock::now();
  stage.stop();
  /// not the 1 s pop timeout
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 500ms);
  EXPECT_EQ(stage.count(), 1);
  stage.stop();
}

TEST(Stage, ExceptionInRunIsReportedInsteadOfTerminating) {
  class ThrowingStage : public Stage {
   public:
    ~ThrowingStage() override { stop(); }

   private:
    void run() override { throw std::runtime_error("mapper exploded"); }
  };
  ThrowingStage stage;
  EXPECT_FALSE(stage.failed());
  stage.start();
  /// joins the finished thread
  stage.stop();
  EXPECT_TRUE(stage.failed());
  EXPECT_EQ(stage.error(), "mapper exploded");
}

TEST(Rate, KeepsAbsolutePeriod) {
  Rate rate(100.0);
  const auto begin = std::chrono::steady_clock::now();
  for (int i = 0; i < 10; ++i) {
    /// work shorter than the period must not stretch it
    std::this_thread::sleep_for(3ms);
    rate.sleep();
  }
  const auto elapsed = std::chrono::steady_clock::now() - begin;
  EXPECT_GE(elapsed, 95ms);
  EXPECT_LT(elapsed, 150ms);
}
