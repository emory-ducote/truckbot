cmake_minimum_required(VERSION 3.5)
project(localization)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(fmt REQUIRED)


include_directories(
  include
)
  
add_executable(ekf
  src/EKFMiddleware.cpp
  src/EKF.cpp
  src/LowPassFilter.cpp
)

target_link_libraries(ekf Eigen3::Eigen fmt::fmt)
ament_target_dependencies(ekf rclcpp sensor_msgs std_msgs nav_msgs)

add_executable(particle_filter
  src/ParticleFilterMiddleware.cpp
  src/ParticleFilter.cpp
)

target_link_libraries(particle_filter Eigen3::Eigen fmt::fmt)
ament_target_dependencies(particle_filter rclcpp)

add_executable(particle_filter_simulator
  src/ParticleFilterSimulator.cpp
)

target_link_libraries(particle_filter_simulator Eigen3::Eigen)

install(TARGETS
  ekf
  particle_filter
  particle_filter_simulator
  DESTINATION lib/${PROJECT_NAME})

ament_package()