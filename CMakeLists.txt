cmake_minimum_required(VERSION 3.5)

project(tf2_delay)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(time_tf2_frames src/time_tf2_frames.cpp)
ament_target_dependencies(time_tf2_frames
  "builtin_interfaces"
  "geometry_msgs"
  "rclcpp"
  "tf2"
  "tf2_ros")

add_executable(pub_tf2_frames src/pub_tf2_frames.cpp)
ament_target_dependencies(pub_tf2_frames
  "geometry_msgs"
  "rclcpp"
  "tf2_ros")

install(TARGETS time_tf2_frames pub_tf2_frames
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
