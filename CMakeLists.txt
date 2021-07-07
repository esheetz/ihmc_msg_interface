#---------------------------------------------------------------------------
# Dependencies on ROS and IHMC controller_msgs
#---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.1)
project(IHMCMsgInterface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  sensor_msgs
  std_msgs
  controller_msgs
  val_dynacore
)
catkin_package(
  CATKIN_DEPENDS roscpp tf geometry_msgs sensor_msgs std_msgs controller_msgs val_dynacore
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

