cmake_minimum_required(VERSION 3.8)
project(agv_sim_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)

ament_package()
