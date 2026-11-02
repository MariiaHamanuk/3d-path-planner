# Code conventions

Apply these to all new and changed code (C++ unless noted). Match the surrounding code for
anything not covered here.

1. **Class names** are `CamelCase` (`SimReader`, `CameraCalibration`). This also applies to
   structs, enums and type aliases.
2. **Methods, functions and variables** are `snake_case` (`wait_for_data`, `to_stamped_pose`,
   `depth_image`). This includes parameters, locals and the data fields of plain aggregates.
3. **Class data members** have an `m_` prefix (`m_node`, `m_color_cal`). It applies to any
   type that has private or protected state. Plain all-public data structs (for example
   `Pose`, `ImageFrame`, `SimReaderConfig`) are aggregates and keep unprefixed `snake_case`
   fields, so they can be used with designated initializers and read naturally
   (`pose.position`).

Enumerators are `CamelCase` (`CameraId::Color`). Constants and macros are not covered yet.

## Project rules

- ROS types are confined to the I/O layer (`sim_reader.hpp/.cpp`). Every other header exposes only
  project types (Eigen, OpenCV, std) - see `3dpath/include/types.hpp` - and the rest of the system
  talks to the I/O layer through interfaces such as `Reader`.
- The code is C++20. Constrain templates with concepts instead of relying on
  implicit requirements (see `Stamped` in `communication.hpp`: every message type must have a `stamp`).
