cmake_minimum_required(VERSION 3.16)
project(path3d)

if(NOT CMAKE_BUILD_TYPE)
  # colcon passes no build type, which means no optimization: the mapper runs ~15x slower.
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# MAVSDK: the C++ MAVLink library behind MavsdkVehicle (installed in the docker image).
find_package(MAVSDK REQUIRED)

# sim_reader.hpp is the ROS-facing I/O layer and exposes ROS types; everything else in the public
# headers uses only Eigen + OpenCV + std types (consumers use SimReader through Reader).
add_library(path3d src/core/threading.cpp src/io/sim_reader.cpp src/io/mavsdk_vehicle.cpp
  src/mapping/log_odds_mapper.cpp src/visualization/log_odds_map_visualizer.cpp)
target_include_directories(path3d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(path3d
  PUBLIC Eigen3::Eigen ${OpenCV_LIBS} rclcpp::rclcpp ${sensor_msgs_TARGETS} tf2::tf2
         tf2_ros::tf2_ros ${tf2_msgs_TARGETS} ${visualization_msgs_TARGETS})
target_include_directories(path3d PUBLIC ${OpenCV_INCLUDE_DIRS})
# visualization/ is ROS-facing too: it publishes the map to ROS topics.
# mavsdk_vehicle.hpp is the MAVSDK-facing I/O layer and exposes MAVSDK types.
target_link_libraries(path3d PUBLIC MAVSDK::mavsdk)

find_package(Threads REQUIRED)
target_link_libraries(path3d PUBLIC Threads::Threads)

add_executable(path3d_main apps/main.cpp)
target_link_libraries(path3d_main path3d)
add_executable(line_planner_demo apps/line_planner_demo.cpp)
target_link_libraries(line_planner_demo path3d)
add_executable(planner_benchmark apps/planner_benchmark.cpp)
target_link_libraries(planner_benchmark path3d)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS path3d EXPORT export_path3d
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS path3d_main line_planner_demo planner_benchmark DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(communication_test test/communication_test.cpp)
  target_link_libraries(communication_test path3d)
  ament_add_gtest(types_test test/types_test.cpp)
  target_link_libraries(types_test path3d)
  ament_add_gtest(reader_test test/reader_test.cpp)
  target_link_libraries(reader_test path3d)
  ament_add_gtest(map_view_test test/map_view_test.cpp)
  target_link_libraries(map_view_test path3d)
  ament_add_gtest(map_client_test test/map_client_test.cpp)
  target_link_libraries(map_client_test path3d)
  ament_add_gtest(mapper_test test/mapper_test.cpp)
  target_link_libraries(mapper_test path3d)
  ament_add_gtest(log_odds_map_test test/log_odds_map_test.cpp)
  target_link_libraries(log_odds_map_test path3d)
  ament_add_gtest(log_odds_map_visualizer_test test/log_odds_map_visualizer_test.cpp)
  target_link_libraries(log_odds_map_visualizer_test path3d)
  ament_add_gtest(line_planner_test test/line_planner_test.cpp)
  target_link_libraries(line_planner_test path3d)
  ament_add_gtest(pipeline_test test/pipeline_test.cpp)
  target_link_libraries(pipeline_test path3d)
  ament_add_gtest(global_planner_test test/global_planner_test.cpp)
  target_link_libraries(global_planner_test path3d)
  ament_add_gtest(planning_test test/planning_test.cpp)
  target_link_libraries(planning_test path3d)
  ament_add_gtest(sync_test test/sync_test.cpp)
  target_link_libraries(sync_test path3d)
  ament_add_gtest(trajectory_test test/trajectory_test.cpp)
  target_link_libraries(trajectory_test path3d)
  ament_add_gtest(ned_test test/ned_test.cpp)
  target_link_libraries(ned_test path3d)
  ament_add_gtest(commander_test test/commander_test.cpp)
  target_link_libraries(commander_test path3d)
  ament_add_gtest(mavsdk_vehicle_test test/mavsdk_vehicle_test.cpp)
  target_link_libraries(mavsdk_vehicle_test path3d)
  ament_add_gtest(threading_test test/threading_test.cpp)
  target_link_libraries(threading_test path3d)
  ament_add_gtest(search_algorithms_test test/search_algorithms_test.cpp)
  target_link_libraries(search_algorithms_test path3d)
  ament_add_gtest(search_planners_test test/search_planners_test.cpp)
  target_link_libraries(search_planners_test path3d)
endif()

ament_export_targets(export_path3d HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV rclcpp sensor_msgs tf2 tf2_ros tf2_msgs visualization_msgs MAVSDK)
ament_package()
