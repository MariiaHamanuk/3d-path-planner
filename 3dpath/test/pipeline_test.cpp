#include "fake_reader.hpp"
#include "stages.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <mutex>
#include <vector>

using namespace std::chrono_literals;
using namespace path3d;

namespace {
// A reader whose depth camera produces a new frame on every poll.
class AdvancingReader : public test::FakeReader {
 public:
  std::optional<ImageFrame> depth_image() const override {
    auto frame = FakeReader::depth_image();
    frame->stamp = 10.0 + static_cast<double>(m_calls++);
    return frame;
  }

 private:
  mutable std::atomic<int> m_calls{0};
};

class CollectingMapper : public Mapper {
 public:
  void integrate(const SensorFrame& frame) override {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stamps.push_back(frame.stamp);
  }
  std::uint64_t version() const override { return size(); }
  std::size_t size() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stamps.size();
  }
  std::vector<double> stamps() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stamps;
  }

 private:
  mutable std::mutex m_mutex;
  std::vector<double> m_stamps;
};

bool wait_for(const std::function<bool()>& done, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done()) {
    if (std::chrono::steady_clock::now() > deadline) return false;
    std::this_thread::sleep_for(5ms);
  }
  return true;
}
}  // namespace

TEST(Pipeline, FramesFlowFromReaderToMapperInOrder) {
  AdvancingReader reader;
  Topic<SensorFrame> frames;
  CollectingMapper mapper;
  MapperStage mapper_stage(frames, mapper, 64);
  ReaderStage reader_stage(reader, frames, 500.0);
  mapper_stage.start();
  reader_stage.start();

  ASSERT_TRUE(wait_for([&] { return mapper.size() >= 10; }, 3s));
  reader_stage.stop();
  mapper_stage.stop();

  const auto stamps = mapper.stamps();
  for (std::size_t i = 1; i < stamps.size(); ++i) EXPECT_GT(stamps[i], stamps[i - 1]);
  EXPECT_EQ(mapper_stage.integrated(), mapper.size());
}

TEST(Pipeline, UnchangedDepthStampIsPublishedOnlyOnce) {
  test::FakeReader reader;  // always the same depth stamp
  Topic<SensorFrame> frames;
  CollectingMapper mapper;
  MapperStage mapper_stage(frames, mapper);
  ReaderStage reader_stage(reader, frames, 500.0);
  mapper_stage.start();
  reader_stage.start();

  ASSERT_TRUE(wait_for([&] { return mapper.size() >= 1; }, 2s));
  std::this_thread::sleep_for(100ms);  // many more polls
  reader_stage.stop();
  mapper_stage.stop();
  EXPECT_EQ(reader_stage.published(), 1u);
  EXPECT_EQ(mapper.size(), 1u);
}

TEST(Pipeline, NothingFlowsWhileReaderHasNoData) {
  test::FakeReader reader(false);
  Topic<SensorFrame> frames;
  CollectingMapper mapper;
  MapperStage mapper_stage(frames, mapper);
  ReaderStage reader_stage(reader, frames, 500.0);
  mapper_stage.start();
  reader_stage.start();
  std::this_thread::sleep_for(100ms);
  reader_stage.stop();
  mapper_stage.stop();
  EXPECT_EQ(reader_stage.published(), 0u);
}

TEST(Pipeline, SlowMapperDropsOldestInsteadOfFallingBehind) {
  class SlowMapper : public CollectingMapper {
   public:
    void integrate(const SensorFrame& frame) override {
      std::this_thread::sleep_for(30ms);
      CollectingMapper::integrate(frame);
    }
  };
  AdvancingReader reader;
  Topic<SensorFrame> frames;
  SlowMapper mapper;
  MapperStage mapper_stage(frames, mapper, 2);
  ReaderStage reader_stage(reader, frames, 500.0);
  mapper_stage.start();
  reader_stage.start();
  std::this_thread::sleep_for(400ms);
  reader_stage.stop();
  mapper_stage.stop();
  EXPECT_GT(mapper_stage.dropped(), 0u);
  EXPECT_GT(reader_stage.published(), mapper.size());
}
