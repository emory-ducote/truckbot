cmake_minimum_required(VERSION 3.5)
project(missions)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(truckbot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

INCLUDE_DIRECTORIES(
  include
)

add_executable(mission_runner
  src/MissionRunnerMiddleware.cpp
  src/MissionRunner.cpp
)
target_link_libraries(mission_runner ${YAML_CPP_LIBRARIES})
ament_target_dependencies(mission_runner rclcpp truckbot_msgs geometry_msgs std_msgs std_srvs ament_index_cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mission_runner
    test/test_mission_runner.cpp
    src/MissionRunner.cpp
    )
  target_link_libraries(test_mission_runner ${YAML_CPP_LIBRARIES})
endif()

install(TARGETS
  mission_runner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
