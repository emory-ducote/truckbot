cmake_minimum_required(VERSION 3.5)
project(icm20948_driver)

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
find_package(Eigen3 3.3 REQUIRED)


INCLUDE_DIRECTORIES(
  include
)

add_executable(imu
  src/IMUMiddleware.cpp
  src/rpi5ICM20948.cpp
)

target_link_libraries(imu lgpio Eigen3::Eigen)
ament_target_dependencies(imu rclcpp sensor_msgs)


install(TARGETS
  imu
  DESTINATION lib/${PROJECT_NAME})

ament_package()