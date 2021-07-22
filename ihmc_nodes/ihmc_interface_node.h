/**
 * IHMC Interface Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _IHMC_INTERFACE_NODE_H_
#define _IHMC_INTERFACE_NODE_H_

#include <sstream> // for testing purposes only // TODO
#include <std_msgs/String.h> // for testing purposes only // TODO

#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <ihmc_utils/ihmc_msg_utilities.h>

class IHMCInterfaceNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    IHMCInterfaceNode(const ros::NodeHandle& nh);
    ~IHMCInterfaceNode();

    // CONNECTIONS
    bool initializeConnections();

    // CALLBACKS
    void transformCallback(const geometry_msgs::TransformStamped& tf_msg);
    void jointCommandCallback(const sensor_msgs::JointState& js_msg);

    // TESTING FUNCTIONS
    void publishTestMessage(); // TODO

    // PUBLISH MESSAGE
    void publishWholeBodyMessage();

    // HELPER FUNCTIONS
    bool getStopNodeFlag();
    void updateStopNodeFlag();
    void prepareConfigurationVector();

private:
    ros::NodeHandle nh_; // node handler

    // TESTING VARIABLES
    ros::Publisher test_publisher_; // publisher // TODO
    int test_counter_ = 0; // TODO

    std::string pelvis_tf_topic_; // topic to subscribe to for listening to pelvis transforms
    ros::Subscriber pelvis_transform_sub_; // subscriber for listening for pelvis transforms in world frame
    std::string joint_command_topic_; // topic to subscribe to for listening to joint commands
    ros::Subscriber joint_command_sub_; // subscriber for listening for joint commands

    ros::Publisher wholebody_pub_; // publisher for wholebody messages

    bool commands_from_controllers_; // flag indicating whether joint commands are coming from controllers (affects queueing properties of messages)
    bool receive_pelvis_transform_; // flag indicating whether to accept new transforms
    bool receive_joint_commands_; // flag indicating whether to accept new joint commands

    bool stop_node_; // flag indicating when to publish whole body messages

    dynacore::Vector q_joint_; // vector of commanded joint positions
    tf::Transform tf_pelvis_wrt_world_; // transform of pelvis in world frame
    dynacore::Vector q_; // full configuration vector, including virtual joints
};

#endif
