cmake_minimum_required(VERSION 3.8)
project(autocar_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# Kalman filter library (no main)
add_library(autocar_perception_algo src/kalman_filter.cpp)
target_include_directories(autocar_perception_algo PUBLIC include)
ament_target_dependencies(autocar_perception_algo rclcpp Eigen3)

set(EXECUTABLES
  gps_driver
  imu_driver
  lidar_driver
  camera_driver
  localization_fusion
  slam_node
)

foreach(EXEC IN LISTS EXECUTABLES)
  add_executable(${EXEC} src/${EXEC}.cpp)
  target_link_libraries(${EXEC} autocar_perception_algo)
  ament_target_dependencies(${EXEC}
    rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs Eigen3
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
