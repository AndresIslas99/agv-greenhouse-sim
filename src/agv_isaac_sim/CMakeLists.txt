cmake_minimum_required(VERSION 3.8)
project(agv_isaac_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/   DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY urdf/     DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY assets/   DESTINATION share/${PROJECT_NAME}/assets)
install(DIRECTORY worlds/   DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY robot/    DESTINATION share/${PROJECT_NAME}/robot)
install(DIRECTORY extensions/ DESTINATION share/${PROJECT_NAME}/extensions)

install(PROGRAMS
  scripts/build_greenhouse_usd.py
  scripts/import_robot_usd.py
  scripts/isaac_ros_bridge_node.py
  scripts/sim_depth_noise.py
  scripts/sim_domain_randomizer.py
  scripts/sim_global_odom.py
  scripts/sim_motor_gate.py
  scripts/sim_odom_tf_broadcaster.py
  scripts/sim_pointcloud_downsample.py
  scripts/sim_wheel_odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/world/ DESTINATION lib/${PROJECT_NAME}/world)
install(DIRECTORY scripts/robot/ DESTINATION lib/${PROJECT_NAME}/robot)

ament_package()
