#---------------------------------------------------------------------------
# Dependencies on ROS and IHMC controller_msgs
#---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.1)
project(ihmc_msg_interface)

set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)

#------------------------------------------------------------------------
#     Build Setup
#------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  moveit_msgs
  controller_msgs
  val_dynacore_utils
)

#------------------------------------------------------------------------
#     catkin Setup
#------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS roscpp tf geometry_msgs sensor_msgs std_msgs trajectory_msgs moveit_msgs controller_msgs val_dynacore_utils
)
include_directories(${catkin_INCLUDE_DIRS})

#---------------------------------------------------------------------------
# Add subdirectories
#---------------------------------------------------------------------------
include_directories ("${PROJECT_SOURCE_DIR}/..")
include_directories ("${PROJECT_SOURCE_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/ihmc_utils")
include_directories ("${PROJECT_SOURCE_DIR}/ihmc_nodes")
include_directories (${catkin_INCLUDE_DIRS})
add_subdirectory (ihmc_utils)
add_subdirectory (ihmc_nodes)
add_subdirectory (ihmc_tests)

