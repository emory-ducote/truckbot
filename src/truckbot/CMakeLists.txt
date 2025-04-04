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


INCLUDE_DIRECTORIES(
  include
)
  
add_executable(motor_controller 
  src/MotorControllerMiddleware.cpp
  src/MotorController.cpp
)

add_executable(imu
  src/IMUMiddleware.cpp
  src/rpi5_ICM20948.cpp
)

target_link_libraries(motor_controller lgpio)
target_link_libraries(imu lgpio)
ament_target_dependencies(motor_controller rclcpp std_msgs sensor_msgs)
ament_target_dependencies(imu rclcpp sensor_msgs)

install(TARGETS
  motor_controller
  imu
  DESTINATION lib/${PROJECT_NAME})

ament_package()