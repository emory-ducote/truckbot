cmake_minimum_required(VERSION 3.5)
project(truckbot)

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
find_package(nav_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)


INCLUDE_DIRECTORIES(
  include
)
  
add_executable(motor_controller 
  src/MotorControllerMiddleware.cpp
  src/MotorController.cpp
)

add_executable(imu
  src/IMUMiddleware.cpp
  src/rpi5ICM20948.cpp
)

add_executable(ekf
  src/EKFMiddleware.cpp
  src/EKF.cpp
)

target_link_libraries(motor_controller lgpio)
target_link_libraries(imu lgpio)
target_link_libraries(ekf Eigen3::Eigen)
ament_target_dependencies(motor_controller rclcpp std_msgs sensor_msgs)
ament_target_dependencies(imu rclcpp sensor_msgs)
ament_target_dependencies(ekf rclcpp sensor_msgs nav_msgs)


install(TARGETS
  motor_controller
  imu
  ekf
  DESTINATION lib/${PROJECT_NAME})

ament_package()