cmake_minimum_required(VERSION 3.5)
project(deepstream_ros2_bridge_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
  tf2
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

add_executable(object_map_coordinates src/object_map_coordinates.cpp)
ament_target_dependencies(object_map_coordinates ${dependencies})

add_executable(delay_topic src/delay_topic.cpp)
ament_target_dependencies(delay_topic rclcpp sensor_msgs std_msgs)

install(TARGETS
  object_map_coordinates
  delay_topic
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/ObjectDetection.msg"
)

ament_package()
