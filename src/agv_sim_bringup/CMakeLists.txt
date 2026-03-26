cmake_minimum_required(VERSION 3.8)
project(agv_sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY maps/ DESTINATION share/${PROJECT_NAME}/maps)
install(PROGRAMS scripts/sim_global_odom.py DESTINATION lib/${PROJECT_NAME})

ament_package()
