find_package(GTest REQUIRED)

# PID controller test
ament_add_gtest(test_pid test_pid.cpp ../src/pid_controller.cpp)
target_include_directories(test_pid PRIVATE ../include)
target_link_libraries(test_pid GTest::gtest GTest::gtest_main pthread)

# Motor driver test (pure math, no node needed)
ament_add_gtest(test_motor_driver test_motor_driver.cpp)
target_link_libraries(test_motor_driver GTest::gtest GTest::gtest_main pthread)

# Velocity controller test (only tests PID, no main() conflict)
ament_add_gtest(test_velocity_controller test_velocity_controller.cpp ../src/pid_controller.cpp)
target_include_directories(test_velocity_controller PRIVATE ../include)
ament_target_dependencies(test_velocity_controller rclcpp geometry_msgs nav_msgs std_msgs)
target_link_libraries(test_velocity_controller GTest::gtest GTest::gtest_main pthread)

# Safety watchdog test
ament_add_gtest(test_safety_watchdog test_safety_watchdog.cpp)
target_include_directories(test_safety_watchdog PRIVATE ../include)
target_link_libraries(test_safety_watchdog GTest::gtest GTest::gtest_main pthread)
