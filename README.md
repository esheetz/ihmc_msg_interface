# IHMCMsgInterface

Interface for using IHMC messages to send commands to NASA's Valkyrie robot.



## Prerequisites
- cmake
- eigen3 (in ubuntu: sudo apt install libeigen3-dev)
- [`val_dynacore` package](https://github.com/esheetz/val_dynacore) for robot model and some utilities



## Compile (in Linux)
To compile the `IHMCMsgInterface` package as part of a catkin workspace, clone this package, clone the `val_dynacore` package, and build the workspace:
```
$ catkin build
$ source devel/setup.bash
```



## IHMC Kinematics Toolbox
NASA's Johnson Space Center (JSC) uses IHMC messages to communicate with the Valkyrie robot.  These controller messages are part of the IHMC Kinematics Toolbox, which includes functionality for safety checks, etc.  The available IHMC messages can be seen in the [IHMC `controller_msgs` repo](https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/val-develop/ihmc-interfaces/src/main/messages/ros1/controller_msgs/msg).



## Package Structure
The `IHMCMsgInterface` is designed to be a largely stand-alone package that will create IHMC whole-body messages and send them to the robot (real or in sim).  This interface alone should be modified to address any changes to the IHMC messages.  Constructing the package to be independent of where the commands are coming from ensures that the details of populating and sending the messages are abstracted away from the controllers sending the commands.

Note that the `IHMCMsgInterface` has some dependencies on [`val_dynacore`](https://github.com/esheetz/val_dynacore) for the robot model and some utility functions.

### Utilities
The `ihmc_utils` directory contains utility functions for constructing IHMC messages.

### Nodes
The `ihmc_nodes` directory contains the IHMC Interface Node, which listens for joint commands and pelvis transforms, constructs the appropriate IHMC whole-body message, and publishes the message to the robot.  This node is designed to be a stand-alone node that will take joint commands from any other node; simply adjust the connections by changing the subscribed topics to the appropriate names.

If the controllers are stopped, they will send a stop status to the IHMC Message Interface, which will tell the node to stop accepting joint commands.  If the IHMC Message Interface receives a start status, it will begin listening for joint commands again and send the appropriate whole-body messages to the robot.  This makes it so the IHMC Message Interface does not need to be restarted every time controllers are stopped or started.

### Launch
The `ihmc_launch` directory contains a launch file for starting the IHMC Message Interface.  The default parameters will initialized the IHMC Interface Node to listen for joint commands from controllers.  For more information about how the `IHMCMsgInterface` is used to communicate with the robot, see the `val_dynacore` package documentation on [running the SCS simulation](https://github.com/esheetz/val_dynacore/blob/master/docs/SCS_sim.md#running-scs-sim) and [running the Valkyrie robot](https://github.com/esheetz/val_dynacore/blob/master/docs/robot_ops.md#communicating-with-the-robot).



## Running in Simulation
To send commands to the robot, use the SCS simulator.  See the `val_dynacore` package documentation for instructions on [running in SCS](https://github.com/esheetz/val_dynacore/blob/master/docs/SCS_sim.md).
