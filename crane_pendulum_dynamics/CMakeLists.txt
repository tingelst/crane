cmake_minimum_required(VERSION 2.8.3)
project(crane_pendulum_dynamics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
   scripts/spherical_pendulum_visualization
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)