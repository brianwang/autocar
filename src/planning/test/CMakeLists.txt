find_package(GTest REQUIRED)

ament_add_gtest(test_astar test_astar.cpp ../src/astar.cpp)
target_include_directories(test_astar PRIVATE ../include)
target_link_libraries(test_astar GTest::gtest GTest::gtest_main pthread)
