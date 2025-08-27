cmake_minimum_required(VERSION 3.5)
project(teleop)

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
find_package(geometry_msgs REQUIRED)

INCLUDE_DIRECTORIES(
  include
)
  
add_executable(motor_controller 
  src/MotorControllerMiddleware.cpp
  src/MotorController.cpp
)

target_link_libraries(motor_controller lgpio)
ament_target_dependencies(motor_controller rclcpp std_msgs sensor_msgs)

add_executable(teleop
  src/TeleOp.cpp
)

ament_target_dependencies(teleop
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  motor_controller
  teleop
  DESTINATION lib/${PROJECT_NAME})

ament_package()