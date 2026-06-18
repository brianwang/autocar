find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(test_kalman test_kalman.cpp ../src/kalman_filter.cpp)
target_include_directories(test_kalman PRIVATE ../include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(test_kalman rclcpp)
target_link_libraries(test_kalman GTest::gtest GTest::gtest_main pthread)
ament_add_gtest(test_kalman test_kalman.cpp ../src/kalman_filter.cpp)
