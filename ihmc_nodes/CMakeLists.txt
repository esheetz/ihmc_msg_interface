#----------------------------------------------------------------------------
# IHMC Interface Node:
# for reading joint state messages and publishing appropriate IHMC message
#----------------------------------------------------------------------------
add_executable(ihmc_interface_node ihmc_interface_node.cpp)
target_link_libraries(ihmc_interface_node ihmc_msg_utils ${catkin_LIBRARIES})
