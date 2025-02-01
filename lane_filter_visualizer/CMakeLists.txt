cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)

project(lane_filter_visualizer)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)




# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree segmentation)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include 
  ${PCL_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# Add executables
add_executable(lane_filter_visualizer src/lane_filter_visualizer.cpp)
ament_target_dependencies(lane_filter_visualizer
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs 
  PCL
)

# Target include directories
target_include_directories(lane_filter_visualizer PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # 헤더 경로를 추가
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# Link libraries
target_link_libraries(lane_filter_visualizer
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# Install targets
install(TARGETS 
  lane_filter_visualizer
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
)

ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
