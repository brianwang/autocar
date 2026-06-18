find_package(GTest REQUIRED)

# PID controller test
add_executable(test_pid test_pid.cpp ../src/pid_controller.cpp)
target_include_directories(test_pid PRIVATE ../include)
ament_target_dependencies(test_pid rclcpp)
target_link_libraries(test_pid GTest::gtest GTest::gtest_main pthread)
ament_add_gtest(test_pid test_pid.cpp ../src/pid_controller.cpp)

# Motor driver test
add_executable(test_motor_driver test_motor_driver.cpp ../src/motor_driver.cpp)
target_include_directories(test_motor_driver PRIVATE ../include)
ament_target_dependencies(test_motor_driver rclcpp geometry_msgs nav_msgs std_msgs tf2_ros)
target_link_libraries(test_motor_driver GTest::gtest GTest::gtest_main pthread)
ament_add_gtest(test_motor_driver test_motor_driver.cpp ../src/motor_driver.cpp)

# Velocity controller test
add_executable(test_velocity_controller test_velocity_controller.cpp ../src/velocity_controller.cpp ../src/pid_controller.cpp)
target_include_directories(test_velocity_controller PRIVATE ../include)
ament_target_dependencies(test_velocity_controller rclcpp geometry_msgs nav_msgs std_msgs)
target_link_libraries(test_velocity_controller GTest::gtest GTest::gtest_main pthread)
ament_add_gtest(test_velocity_controller test_velocity_controller.cpp ../src/velocity_controller.cpp ../src/pid_controller.cpp)

# Safety watchdog test
add_executable(test_safety_watchdog test_safety_watchdog.cpp)
target_include_directories(test_safety_watchdog PRIVATE ../include)
ament_target_dependencies(test_safety_watchdog rclcpp)
target_link_libraries(test_safety_watchdog GTest::gtest GTest::gtest_main pthread)
ament_add_gtest(test_safety_watchdog test_safety_watchdog.cpp)
