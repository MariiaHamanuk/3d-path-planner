/** @file sim_reader_demo.cpp
 * @brief Prints what SimReader publishes once per second:  ros2 run path3d sim_reader_demo [world]
 */
#include "core/config.hpp"
#include "io/reader.hpp"
#include "io/sim_reader.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
  path3d::SimReaderConfig cfg;
  if (argc > 1) cfg.world_frame = argv[1];

  /// Subscribers first: topics deliver only to subscribers that already exist.
  path3d::SensorTopics sensors;
  const auto pose = sensors.pose.subscribe_latest();
  const auto color = sensors.color.subscribe_latest();
  const auto depth = sensors.depth.subscribe_latest();
  const auto color_calibration = sensors.color_calibration.subscribe_latest();
  const auto depth_calibration = sensors.depth_calibration.subscribe_latest();
  const auto base_to_color = sensors.base_to_color.subscribe_latest();
  const auto base_to_depth = sensors.base_to_depth.subscribe_latest();
  path3d::SimReader reader(cfg, sensors);

  for (;;) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (auto p = pose->latest())
      std::cout << "drone in " << path3d::to_string(p->parent_frame) << " at t=" << p->stamp << ": "
                << p->pose.position.transpose() << "\n";
    if (auto c = color->latest())
      std::cout << "color " << c->image.cols << "x" << c->image.rows << " type " << c->image.type()
                << " frame " << path3d::to_string(c->frame_id) << "\n";
    if (auto d = depth->latest())
      std::cout << "depth " << d->image.cols << "x" << d->image.rows << " t=" << d->stamp
                << " centre " << d->image.at<float>(d->image.rows / 2, d->image.cols / 2) << " m\n";
    for (const auto& k : {color_calibration->latest(), depth_calibration->latest()}) {
      if (k)
        std::cout << to_string(k->frame_id) << " K fx " << k->fx() << " fy " << k->fy() << " cx "
                  << k->cx() << " cy " << k->cy() << "\n";
    }
    for (const auto& t : {base_to_color->latest(), base_to_depth->latest()}) {
      if (t)
        std::cout << "base->" << path3d::to_string(t->child_frame) << ": "
                  << t->pose.position.transpose() << "\n";
    }
    std::cout << "---\n";
  }
}
