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

- ROS types stay inside `.cpp` files of the I/O layer. Public headers expose only project
  types (Eigen, OpenCV, std) - see `3dpath/include/types.hpp`.
- The code is C++20. Constrain templates with concepts instead of relying on
  implicit requirements (see `Stamped` in `communication.hpp`: every message type must have a `stamp`).
