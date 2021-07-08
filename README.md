# IHMCMsgInterface

Interface for using IHMC messages to send commands to NASA's Valkyrie robot.



## Prerequisites
- cmake
- eigen3 (in ubuntu: sudo apt instal libeigen3-dev)
- `val_dynacore` package for robot model and some utilities (https://github.com/esheetz/val_dynacore)



## Compile (in Linux)
To compile the `IHMCMsgInterface` package as part of a catkin workspace, clone this package, clone the `val_dynacore` package, and build the workspace:
```
$ catkin build
$ source devel/setup.bash
```



## IHMC Kinematics Toolbox
NASA's Johnson Space Center (JSC) uses IHMC messages to communicate with the Valkyrie robot.  These controller messages are part of the IHMC Kinematics Toolbox, which includes functionality for safety checks, etc.  The available IHMC messages: https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/val-develop/ihmc-interfaces/src/main/messages/ros1/controller_msgs/msg



## Package Structure
The `IHMCMsgInterface` is designed to be a largely stand-alone package.  This interface alone should be modified to address any changes to the IHMC messages.  Constructing the package to be independent of where the commands are coming from ensures that the details of populating and sending the messages are abstracted away from the controllers sending the commands.

### Utilities
The `ihmc_utils` directory contains utility functions for constructing IHMC messages.

### Nodes
The `ihmc_nodes` directory contains the IHMC Interface Node, which listens for joint commands and pelvis transforms, constructs the appropriate IHMC whole-body message, and publishes the message to the robot.  This node is designed to be a stand-alone node that will take joint commands from any other node; simply adjust the connections by changing the subscribed topics to the appropriate names.



## Running in Simulation
To send commands to the robot, use the SCS simulator.  Directions for running in simulation can be found in `val_dynacore` package documentation (https://github.com/esheetz/val_dynacore#running-in-simulation).  For more information about running in the SCS sim (link will only work with NASA ID verification): https://bender.jsc.nasa.gov/confluence/pages/viewpage.action?spaceKey=VAL2&title=Running+with+SCS+instead+of+Gazebo
