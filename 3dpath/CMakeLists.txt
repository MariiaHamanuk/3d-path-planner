cmake_minimum_required(VERSION 3.16)
project(path3d)

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
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# Public headers expose only Eigen + OpenCV + std types; ROS stays private to the library.
add_library(path3d src/threading.cpp src/sim_reader.cpp)
target_include_directories(path3d
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(path3d
  PUBLIC Eigen3::Eigen ${OpenCV_LIBS}
  PRIVATE rclcpp::rclcpp ${sensor_msgs_TARGETS} tf2::tf2 tf2_ros::tf2_ros)
target_include_directories(path3d PUBLIC ${OpenCV_INCLUDE_DIRS})

find_package(Threads REQUIRED)
target_link_libraries(path3d PUBLIC Threads::Threads)

add_executable(sim_reader_demo apps/sim_reader_demo.cpp)
target_link_libraries(sim_reader_demo path3d)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS path3d EXPORT export_path3d
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS sim_reader_demo DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(communication_test test/communication_test.cpp)
  target_link_libraries(communication_test path3d)
  ament_add_gtest(types_test test/types_test.cpp)
  target_link_libraries(types_test path3d)
  ament_add_gtest(reader_test test/reader_test.cpp)
  target_link_libraries(reader_test path3d)
  ament_add_gtest(threading_test test/threading_test.cpp)
  target_link_libraries(threading_test path3d)
endif()

ament_export_targets(export_path3d HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV)
ament_package()
