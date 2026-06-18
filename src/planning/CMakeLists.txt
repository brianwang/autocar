cmake_minimum_required(VERSION 3.8)
project(autocar_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Algorithm library (no main)
add_library(autocar_planning_algo
  src/astar.cpp
  src/dwa_planner.cpp
  src/behavior_nodes.cpp
)
target_include_directories(autocar_planning_algo PUBLIC include)
ament_target_dependencies(autocar_planning_algo
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
)

set(EXECUTABLES behavior_tree global_planner local_planner)

foreach(EXEC IN LISTS EXECUTABLES)
  add_executable(${EXEC} src/${EXEC}.cpp)
  target_link_libraries(${EXEC} autocar_planning_algo)
  ament_target_dependencies(${EXEC}
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
  )
  install(TARGETS ${EXEC}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
