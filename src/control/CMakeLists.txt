cmake_minimum_required(VERSION 3.8)
project(autocar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autocar_interfaces REQUIRED)

include_directories(include)
include_directories(${rclcpp_INCLUDE_DIRS})

set(EXECUTABLES
  motor_driver
  velocity_controller
  steering_controller
  safety_watchdog
)

foreach(EXEC IN LISTS EXECUTABLES)
  add_executable(${EXEC} src/${EXEC}.cpp)
  ament_target_dependencies(${EXEC}
    rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros autocar_interfaces
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
