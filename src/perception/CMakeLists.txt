cmake_minimum_required(VERSION 3.5)
project(perception)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)


INCLUDE_DIRECTORIES(
  include
)

add_executable(feature_detector
  src/FeatureDetectionMiddleware.cpp
  src/FeatureDetection.cpp
)

target_link_libraries(feature_detector fmt::fmt)
ament_target_dependencies(feature_detector rclcpp sensor_msgs laser_geometry 
                                           visualization_msgs pcl_conversions PCL)
  
add_executable(voxelization
  src/VoxelizationMiddleware.cpp
  src/Voxelization.cpp
)

target_link_libraries(voxelization fmt::fmt)
ament_target_dependencies(voxelization rclcpp sensor_msgs laser_geometry 
                                           visualization_msgs pcl_conversions PCL)

install(TARGETS
  feature_detector
  voxelization
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()