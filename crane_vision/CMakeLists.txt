cmake_minimum_required(VERSION 2.8.3)
project(crane_vision)

find_package(catkin REQUIRED COMPONENTS
  # rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  roscpp
  cv_bridge
  image_transport
  nodelet
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# catkin_python_setup()

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs cv_bridge sensor_msgs nodelet
   DEPENDS OpenCV
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ekf src/crane_vision_ekf.cpp)
target_link_libraries(ekf
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES})


add_executable(example_node src/opencv_example_node.cpp)
target_link_libraries(example_node 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES})

add_library(crane_vision_nodelet src/crane_vision_nodelet.cpp)
target_link_libraries(crane_vision_nodelet 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES})

# catkin_install_python(PROGRAMS
#    nodes/crane_vision_node
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS crane_vision_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
install(FILES crane_vision_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )