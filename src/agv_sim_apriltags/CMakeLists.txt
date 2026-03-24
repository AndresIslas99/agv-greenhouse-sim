cmake_minimum_required(VERSION 3.8)
project(agv_sim_apriltags)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS scripts/fake_marker_detector.py DESTINATION lib/${PROJECT_NAME})

ament_package()
