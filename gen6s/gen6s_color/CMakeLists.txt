cmake_minimum_required(VERSION 3.8)
project(gen6s_color)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(gen6s_color_node src/gen6s_color_node.cpp)
ament_target_dependencies(gen6s_color_node rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS
    gen6s_color_node
    DESTINATION lib/${PROJECT_NAME})

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
