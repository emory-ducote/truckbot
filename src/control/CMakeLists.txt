cmake_minimum_required(VERSION 3.5)
project(control)

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
  
add_executable(teleop
  src/TeleOpMiddleware.cpp
)

ament_target_dependencies(teleop
  rclcpp
  geometry_msgs
  sensor_msgs
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_executable(motor_controller 
    src/MotorControllerMiddleware.cpp
    src/MotorController.cpp
  )

  target_link_libraries(motor_controller lgpio)
  ament_target_dependencies(motor_controller rclcpp std_msgs sensor_msgs)

  add_executable(encoder_driver
    src/EncoderDriverMiddleware.cpp
    src/EncoderDriver.cpp
  )

  target_link_libraries(encoder_driver lgpio)
  ament_target_dependencies(encoder_driver
    rclcpp
    std_msgs
    geometry_msgs
  )

  install(TARGETS
    motor_controller
    encoder_driver
    DESTINATION lib/${PROJECT_NAME})
endif()

install(TARGETS
  teleop
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()