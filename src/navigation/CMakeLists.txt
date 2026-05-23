cmake_minimum_required(VERSION 3.5)
project(navigation)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


INCLUDE_DIRECTORIES(
  include
)
  
add_executable(path_tracker
  src/PathTrackerMiddleware.cpp
  src/PathTracker.cpp
)

target_link_libraries(path_tracker)
ament_target_dependencies(path_tracker rclcpp nav_msgs std_msgs)

add_executable(spline_generator
  src/SplineGeneratorMiddleware.cpp
  src/SplineGenerator.cpp
)

target_link_libraries(spline_generator)
ament_target_dependencies(spline_generator rclcpp nav_msgs std_msgs)

add_executable(simple_path_server
  src/SimplePathServerMiddleware.cpp
)

target_link_libraries(simple_path_server)
ament_target_dependencies(simple_path_server rclcpp nav_msgs geometry_msgs)

add_executable(pure_pursuit_controller
  src/PurePursuitControllerMiddleware.cpp
  src/PurePursuitController.cpp
)

target_link_libraries(pure_pursuit_controller)
ament_target_dependencies(pure_pursuit_controller rclcpp nav_msgs geometry_msgs std_msgs)

install(TARGETS
  path_tracker
  spline_generator
  simple_path_server
  pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/navigation/config
)

ament_package()