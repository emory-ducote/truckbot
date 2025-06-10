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
find_package(Eigen3 3.3 REQUIRED)
find_package(fmt REQUIRED)


INCLUDE_DIRECTORIES(
  include
)
  
add_executable(ekf
  src/EKFMiddleware.cpp
  src/EKF.cpp
)

target_link_libraries(ekf Eigen3::Eigen fmt::fmt)
ament_target_dependencies(ekf rclcpp sensor_msgs nav_msgs)


install(TARGETS
  ekf
  DESTINATION lib/${PROJECT_NAME})

ament_package()