cmake_minimum_required(VERSION 2.8.3)
project(crane_hw_interface)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  angles
  cmake_modules
  controller_manager
  hardware_interface
  joint_limits_interface
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    crane_hw_interface
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    joint_limits_interface
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mlpi SHARED IMPORTED)
set_target_properties(mlpi PROPERTIES
  IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/external/mlpiCoreClient/bin/libmlpi.so"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/external/mlpiCoreClient/include"
)

add_library(crane_hw_interface
  src/crane_hw_interface.cpp
)

target_link_libraries(crane_hw_interface
  ${catkin_LIBRARIES}
  mlpi
)

add_executable(crane_hw_interface_node
  src/crane_hw_interface_node.cpp
)

target_link_libraries(crane_hw_interface_node
  crane_hw_interface
)

install(
  TARGETS crane_hw_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  TARGETS crane_hw_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})