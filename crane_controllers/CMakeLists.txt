cmake_minimum_required(VERSION 2.8.3)
project(crane_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  kdl_conversions
  geometry_msgs
  hardware_interface
  sensor_msgs
  message_generation
  pluginlib
  realtime_tools
  roscpp
)

find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES crane_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    kdl_conversions
    geometry_msgs
    hardware_interface
    sensor_msgs
    message_runtime
    pluginlib
    realtime_tools
    roscpp
)

add_library(crane_controllers
  src/crane_controller.cpp
  src/joint_state_controller.cpp
)

target_link_libraries(crane_controllers PUBLIC
  ${catkin_LIBRARIES}
)

target_include_directories(crane_controllers SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(crane_controllers PUBLIC
  include
)

## Installation
install(TARGETS crane_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES crane_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
