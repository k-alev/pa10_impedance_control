cmake_minimum_required(VERSION 2.8.3)
project(pa10_impedance_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rospy
  controller_interface
  hardware_interface
  urdf
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

# set(rct_lib_DIR "/usr/local/lib/ros_control_toolbox")
# set(rct_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}}/lib/ros_control_toolbox)
# find_package(rct_lib REQUIRED HINTS ${rct_INSTALL_DIR})

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rct_impedance_control
#  CATKIN_DEPENDS pluginlib roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${rct_lib_INCLUDE_DIRS}
  /usr/local/include/ros_control_toolbox
  /usr/include/fcl
  /usr/local/include/fcl2rct
)

add_library(${PROJECT_NAME}
src/impedance_controller.cpp
src/admittance_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  /usr/local/lib/ros_control_toolbox/Release/librct_lib.a
  /usr/local/lib/fcl2rct/Debug/libfcl2rct_lib.a
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  /usr/lib/x86_64-linux-gnu/libfcl.so.0.5
)