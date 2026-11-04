// Entry point of the planner: wires the pipeline stages together.
//
//   SimReader ─ReaderStage─► Topic<SensorFrame> ─MapperStage─► Mapper
//
// Usage: ros2 run path3d path3d_main [world]      (world = default | city)
//
// Only this file knows the concrete implementations (SimReader, StatsMapper); the stages see
// the Reader and Mapper interfaces. Swapping the simulation for a real drone or the placeholder
// mapper for a real map builder means changing the two marked lines.
#include "sim_reader.hpp"
#include "stages.hpp"

#include <atomic>
#include <csignal>
#include <iostream>

namespace {

std::atomic<bool> g_interrupted{false};
void on_signal(int) { g_interrupted = true; }

// Placeholder Mapper: records what it receives. Replace with a real map builder.
class StatsMapper : public path3d::Mapper {
 public:
  void integrate(const path3d::SensorFrame& frame) override {
    m_last_stamp = frame.stamp;
    ++m_version;
  }
  std::uint64_t version() const override { return m_version; }
  double last_stamp() const { return m_last_stamp; }

 private:
  std::atomic<std::uint64_t> m_version{0};
  std::atomic<double> m_last_stamp{0.0};
};

}  // namespace

int main(int argc, char** argv) {
  path3d::SimReaderConfig config;
  if (argc > 1) config.world_frame = argv[1];

  // Declared in dependency order: destroyed in reverse, so producers stop before consumers.
  path3d::SimReader reader(config);                      // <- the Reader implementation
  std::signal(SIGINT, on_signal);  // after the reader: rclcpp::init installs its own handler
  path3d::Topic<path3d::SensorFrame> frames;
  StatsMapper mapper;                                    // <- the Mapper implementation
  path3d::MapperStage mapper_stage(frames, mapper);      // subscribes: create before the reader stage
  path3d::ReaderStage reader_stage(reader, frames);

  mapper_stage.start();
  reader_stage.start();

  while (!g_interrupted) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "frames: read " << reader_stage.published() << ", mapped "
              << mapper_stage.integrated() << ", dropped " << mapper_stage.dropped()
              << ", map version " << mapper.version() << ", last stamp " << mapper.last_stamp()
              << "\n";
  }
}
