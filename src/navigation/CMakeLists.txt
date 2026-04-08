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

install(TARGETS
  path_tracker
  spline_generator
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/navigation/config
)

ament_package()