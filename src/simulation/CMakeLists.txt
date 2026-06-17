cmake_minimum_required(VERSION 3.8)
project(autocar_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

ament_package()
