// Prints what SimReader sees once per second:  ros2 run path3d sim_reader_demo [world]
#include "reader.hpp"
#include "sim_reader.hpp"

#include <chrono>
#include <iostream>
#include <thread>

// Works with any Reader implementation.
void print_forever(const path3d::Reader& reader) {
  if (!reader.wait_for_data(std::chrono::seconds(15))) std::cerr << "timeout: partial data only\n";

  for (;;) {
    if (auto p = reader.drone_pose())
      std::cout << "drone in " << path3d::to_string(p->parent_frame) << ": " << p->pose.position.transpose() << "\n";
    if (auto c = reader.color_image())
      std::cout << "color " << c->image.cols << "x" << c->image.rows << " type " << c->image.type()
                << " frame " << path3d::to_string(c->frame_id) << "\n";
    if (auto d = reader.depth_image())
      std::cout << "depth " << d->image.cols << "x" << d->image.rows << " centre "
                << d->image.at<float>(d->image.rows / 2, d->image.cols / 2) << " m\n";
    for (auto id : {path3d::CameraId::Color, path3d::CameraId::Depth}) {
      if (auto k = reader.calibration(id))
        std::cout << "K fx " << k->fx() << " fy " << k->fy() << " cx " << k->cx() << " cy " << k->cy()
                  << "\n";
      if (auto t = reader.base_to_camera(id))
        std::cout << "base->" << path3d::to_string(t->child_frame) << ": " << t->pose.position.transpose() << "\n";
    }
    std::cout << "---\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

int main(int argc, char** argv) {
  path3d::SimReaderConfig cfg;
  if (argc > 1) cfg.world_frame = argv[1];
  path3d::SimReader reader(cfg);
  print_forever(reader);
}
