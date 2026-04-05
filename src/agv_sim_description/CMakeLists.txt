cmake_minimum_required(VERSION 3.8)
project(agv_sim_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py
    TIMEOUT 30)
endif()

ament_package()
