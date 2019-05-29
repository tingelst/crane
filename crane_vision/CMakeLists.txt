cmake_minimum_required(VERSION 2.8.3)
project(crane_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  roscpp
  cv_bridge
  image_transport
)

find_package( OpenCV REQUIRED )

catkin_python_setup()

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs cv_bridge sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(example_node src/opencv_example_node.cpp)

target_link_libraries(example_node 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES})


catkin_install_python(PROGRAMS
   nodes/crane_vision_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)