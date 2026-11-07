#include "core/communication.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using path3d::BoundedQueue;
using path3d::HistoryBuffer;
using path3d::LatestSlot;
using path3d::Topic;

namespace {
struct Sample {
  double stamp;
  int id;
};
struct NoStamp {
  double time;
};
struct WrongStampType {
  std::string stamp;
};

Sample sample(int id) { return Sample{static_cast<double>(id), id}; }
path3d::Msg<Sample> msg(int id) { return std::make_shared<const Sample>(sample(id)); }
}

static_assert(path3d::Stamped<Sample>);
static_assert(!path3d::Stamped<NoStamp>);
static_assert(!path3d::Stamped<WrongStampType>);
static_assert(!path3d::Stamped<int>);

TEST(LatestSlot, EmptyThenNewestWins) {
  LatestSlot<Sample> slot;
  EXPECT_EQ(slot.latest(), nullptr);
  slot.publish(msg(1));
  slot.publish(msg(2));
  EXPECT_EQ(slot.latest()->id, 2);
}

TEST(BoundedQueue, DropsOldestWhenFull) {
  BoundedQueue<Sample> queue(2);
  for (int i = 1; i <= 4; ++i) queue.push(msg(i));
  EXPECT_EQ(queue.dropped_count(), 2u);
  EXPECT_EQ(queue.size(), 2u);
  EXPECT_EQ(queue.pop(10ms)->id, 3);
  EXPECT_EQ(queue.pop(10ms)->id, 4);
  EXPECT_EQ(queue.pop(10ms), nullptr);
}

TEST(BoundedQueue, ZeroCapacityIsClampedToOne) {
  BoundedQueue<Sample> queue(0);
  queue.push(msg(1));
  queue.push(msg(2));
  EXPECT_EQ(queue.pop(10ms)->id, 2);
}

TEST(BoundedQueue, CloseWakesBlockedPop) {
  BoundedQueue<Sample> queue(1);
  std::atomic<bool> woke{false};
  std::thread consumer([&] {
    EXPECT_EQ(queue.pop(5s), nullptr);
    woke = true;
  });
  std::this_thread::sleep_for(50ms);
  queue.close();
  consumer.join();
  EXPECT_TRUE(woke);
}

TEST(Topic, FansOutOneMessageToAllSubscribers) {
  Topic<Sample> topic;
  auto slot = topic.subscribe_latest();
  auto queue = topic.subscribe_queue(4);
  topic.publish(sample(7));
  ASSERT_NE(slot->latest(), nullptr);
  auto from_queue = queue->pop(10ms);
  ASSERT_NE(from_queue, nullptr);
  /// same shared object, not a copy
  EXPECT_EQ(slot->latest(), from_queue);
  EXPECT_EQ(from_queue->id, 7);
}

TEST(Topic, DroppingTheHandleUnsubscribes) {
  Topic<Sample> topic;
  auto keep = topic.subscribe_latest();
  {
    auto temp = topic.subscribe_queue(1);
    EXPECT_EQ(topic.subscriber_count(), 2u);
  }
  topic.publish(sample(1));
  EXPECT_EQ(topic.subscriber_count(), 1u);
  EXPECT_EQ(keep->latest()->id, 1);
}

TEST(Topic, ConcurrentPublishAndConsume) {
  Topic<Sample> topic;
  auto queue = topic.subscribe_queue(8);
  auto slot = topic.subscribe_latest();
  std::atomic<int> received{0};
  std::thread consumer([&] {
    while (auto msg = queue->pop(200ms)) ++received;
  });
  std::thread producer([&] {
    for (int i = 0; i < 2000; ++i) topic.publish(sample(i));
  });
  producer.join();
  consumer.join();
  EXPECT_EQ(slot->latest()->id, 1999);
  EXPECT_EQ(static_cast<std::size_t>(received) + queue->dropped_count(), 2000u);
}


TEST(HistoryBuffer, ClosestPicksNearestAndEarlierOnTie) {
  Topic<Sample> topic;
  auto history = topic.subscribe_history(10);
  EXPECT_EQ(history->closest(1.0), nullptr);
  topic.publish(Sample{1.0, 1});
  topic.publish(Sample{2.0, 2});
  topic.publish(Sample{4.0, 4});
  EXPECT_EQ(history->closest(0.0)->id, 1);
  EXPECT_EQ(history->closest(1.4)->id, 1);
  EXPECT_EQ(history->closest(1.6)->id, 2);
  /// tie -> earlier
  EXPECT_EQ(history->closest(1.5)->id, 1);
  EXPECT_EQ(history->closest(3.2)->id, 4);
  EXPECT_EQ(history->closest(99.0)->id, 4);
}

TEST(HistoryBuffer, ClosestHonoursTolerance) {
  Topic<Sample> topic;
  auto history = topic.subscribe_history(10);
  topic.publish(Sample{1.0, 1});
  EXPECT_NE(history->closest(1.05, 0.1), nullptr);
  EXPECT_EQ(history->closest(1.5, 0.1), nullptr);
}

TEST(HistoryBuffer, BracketGivesNeighboursForInterpolation) {
  Topic<Sample> topic;
  auto history = topic.subscribe_history(10);
  topic.publish(Sample{1.0, 1});
  topic.publish(Sample{2.0, 2});

  auto mid = history->bracket(1.25);
  EXPECT_EQ(mid.first->id, 1);
  EXPECT_EQ(mid.second->id, 2);

  auto exact = history->bracket(2.0);
  EXPECT_EQ(exact.first->id, 2);
  EXPECT_EQ(exact.second->id, 2);

  /// outside the history
  EXPECT_EQ(history->bracket(0.5).first, nullptr);
  EXPECT_EQ(history->bracket(0.5).second->id, 1);
  EXPECT_EQ(history->bracket(3.0).second, nullptr);
  EXPECT_EQ(history->bracket(3.0).first->id, 2);
}

TEST(HistoryBuffer, EvictsOldestAndSortsOutOfOrderInserts) {
  Topic<Sample> topic;
  auto history = topic.subscribe_history(3);
  topic.publish(Sample{3.0, 3});
  /// late arrival is placed by stamp
  topic.publish(Sample{1.0, 1});
  topic.publish(Sample{2.0, 2});
  EXPECT_EQ(*history->oldest_stamp(), 1.0);
  EXPECT_EQ(*history->newest_stamp(), 3.0);
  /// evicts stamp 1.0
  topic.publish(Sample{4.0, 4});
  EXPECT_EQ(history->size(), 3u);
  EXPECT_EQ(*history->oldest_stamp(), 2.0);
  EXPECT_EQ(history->closest(1.0)->id, 2);
}

TEST(HistoryBuffer, WaitUntilNewestBlocksUntilData) {
  Topic<Sample> topic;
  auto history = topic.subscribe_history(4);
  topic.publish(Sample{1.0, 1});
  EXPECT_TRUE(history->wait_until_newest(1.0, 10ms));
  EXPECT_FALSE(history->wait_until_newest(2.0, 20ms));

  std::thread producer([&] {
    std::this_thread::sleep_for(50ms);
    topic.publish(Sample{2.5, 2});
  });
  EXPECT_TRUE(history->wait_until_newest(2.0, 2s));
  producer.join();
}

TEST(HistoryBuffer, CloseWakesWaiter) {
  HistoryBuffer<Sample> history(4);
  std::thread closer([&] {
    std::this_thread::sleep_for(50ms);
    history.close();
  });
  EXPECT_FALSE(history.wait_until_newest(10.0, 5s));
  closer.join();
}

TEST(HistoryBuffer, ConcurrentPushAndQuery) {
  Topic<Sample> topic;
  auto history = topic.subscribe_history(64);
  std::thread producer([&] {
    for (int i = 0; i < 3000; ++i) topic.publish(Sample{i * 0.001, i});
  });
  std::thread reader([&] {
    for (int i = 0; i < 3000; ++i) {
      if (auto sample = history->closest(i * 0.001)) {
        EXPECT_GE(sample->id, 0);
      }
      history->bracket(i * 0.001);
    }
  });
  producer.join();
  reader.join();
  EXPECT_EQ(history->size(), 64u);
}

TEST(Pipe, EachDirectionIsAnIndependentTopic) {
  path3d::Pipe<Sample, Sample> pipe;
  auto down = pipe.down().subscribe_queue(4);
  auto up = pipe.up().subscribe_queue(4);
  pipe.down().publish(sample(1));
  EXPECT_EQ(down->pop(10ms)->id, 1);
  /// nothing went upwards
  EXPECT_EQ(up->pop(10ms), nullptr);
  pipe.up().publish(sample(2));
  EXPECT_EQ(up->pop(10ms)->id, 2);
  EXPECT_EQ(down->pop(10ms), nullptr);
}
