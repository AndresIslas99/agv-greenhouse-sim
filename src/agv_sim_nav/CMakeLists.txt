cmake_minimum_required(VERSION 3.8)
project(agv_sim_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
