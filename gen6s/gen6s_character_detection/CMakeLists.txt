cmake_minimum_required(VERSION 3.8)
project(gen6s_character_detection)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(
  include
  ${ament_index_cpp_INCLUDE_DIRS}
)

# Define the executable
add_executable(char_recognition_node src/char_recognition_node.cpp)

# Link libraries
target_link_libraries(char_recognition_node ${OpenCV_LIBRARIES})

# Add dependencies
ament_target_dependencies(
  char_recognition_node
  rclcpp
  sensor_msgs
  std_msgs
  OpenCV
  cv_bridge
  ament_index_cpp
)

# Install targets
install(TARGETS
  char_recognition_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install resources (e.g., char_templates folder)
install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources/
)

# Export package
ament_package()
