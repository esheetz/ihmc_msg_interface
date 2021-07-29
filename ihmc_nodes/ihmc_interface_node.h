/**
 * IHMC Interface Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#ifndef _IHMC_INTERFACE_NODE_H_
#define _IHMC_INTERFACE_NODE_H_

#include <vector>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
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
    void controlledLinkIdsCallback(const std_msgs::Int32MultiArray& arr_msg);
    void jointCommandCallback(const sensor_msgs::JointState& js_msg);
    void statusCallback(const std_msgs::String& status_msg);

    // PUBLISH MESSAGE
    void publishWholeBodyMessage();

    // HELPER FUNCTIONS
    std::string getStatus();
    bool getCommandsFromControllersFlag();
    bool getPublishCommandsFlag();
    void updatePublishCommandsFlag();
    bool getStopNodeFlag();
    void updateStopNodeFlag();
    void prepareConfigurationVector();

private:
    ros::NodeHandle nh_; // node handler

    std::string pelvis_tf_topic_; // topic to subscribe to for listening to pelvis transforms
    ros::Subscriber pelvis_transform_sub_; // subscriber for listening for pelvis transforms in world frame
    std::string controlled_link_topic_; // topic to subscribe to for listening to controlled links
    ros::Subscriber controlled_link_sub_; // subscriber for listening for controlled link ids
    std::string joint_command_topic_; // topic to subscribe to for listening to joint commands
    ros::Subscriber joint_command_sub_; // subscriber for listening for joint commands
    std::string status_topic_; // topic to subscribe to for listening to statuses
    ros::Subscriber status_sub_; // subscriber for listening to statuses
    std::string status_; // string indicating current status

    ros::Publisher wholebody_pub_; // publisher for wholebody messages

    bool commands_from_controllers_; // flag indicating whether joint commands are coming from controllers (affects queueing properties of messages)
    bool receive_pelvis_transform_; // flag indicating whether to accept pelvis transforms
    bool received_pelvis_transform_; // flag indicating whether pelvis transform has been received
    bool receive_link_ids_; // flag indicating whether to accept link ids
    bool received_link_ids_; // flag indicating whether link ids have been received
    bool receive_joint_command_; // flag indicating whether to accept new joint commands
    bool received_joint_command_; // flag indicating whether joint command has been received
    bool publish_commands_; // flag indicating if joint and pelvis information has been received and whole body message can be published
    bool stop_node_; // flag indicating when to publish whole body messages

    dynacore::Vector q_joint_; // vector of commanded joint positions
    tf::Transform tf_pelvis_wrt_world_; // transform of pelvis in world frame
    dynacore::Vector q_; // full configuration vector, including virtual joints
    std::vector<int> controlled_links_; // vector of controlled links
};

#endif
