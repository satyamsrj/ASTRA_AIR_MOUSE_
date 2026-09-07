cmake_minimum_required(VERSION 3.0.2)
project(nidar_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  mavros_msgs
  quadrotor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs mavros_msgs quadrotor_msgs
)

catkin_install_python(PROGRAMS
  scripts/entry_detection_module.py
  scripts/apply_mission_config.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
