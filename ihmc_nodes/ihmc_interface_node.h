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
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
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
    void handPoseCommandCallback(const geometry_msgs::TransformStamped& tf_msg);
    void receiveCartesianGoalsCallback(const std_msgs::Bool& bool_msg);
    void plannedMoveItRobotTrajectoryCallback(const moveit_msgs::RobotTrajectory& moveit_msg);
    void receiveMoveItTrajCallback(const std_msgs::Bool& bool_msg);
    void trajectoryPointTimeCallback(const std_msgs::Float32& float_msg);

    // PUBLISH MESSAGE
    void publishWholeBodyMessage();
    void publishWholeBodyMessageCartesianHandGoals();
    void publishWholeBodyMessageMoveItTrajectory();
    void publishGoHomeMessage();
    void publishHandFingerMessage();
    void publishFingerOpenLeftMessage();
    void publishFingerCloseLeftMessage();
    void publishFingerOpenRightMessage();
    void publishFingerCloseRightMessage();
    void publishAbortWalkingMessage();
    void publishPauseWalkingMessage();
    void publishResumeWalkingMessage();
    void publishStopAllTrajectoryMessage();

    // HELPER FUNCTIONS
    std::string getStatus();
    bool getCommandsFromControllersFlag();
    bool getPublishCommandsFlag();
    void updatePublishCommandsFlag();
    bool getStopNodeFlag();
    void updateStopNodeFlag();
    bool getPublishGoHomeCommandFlag();
    void updatePublishGoHomeCommandFlag();
    bool getPublishFingerCommandFlag();
    void updatePublishFingerCommandFlag();
    bool getPublishHandCommandFlag();
    void updatePublishHandCommandFlag();
    bool getPublishAbortWalkingCommandFlag();
    bool getPublishPauseWalkingCommandFlag();
    bool getPublishResumeWalkingCommandFlag();
    bool getPublishStopAllTrajectoryCommandFlag();
    bool getPublishMoveItTrajectoryFlag();
    void updatePublishMoveItTrajectoryFlag();
    void prepareEmptyPose(dynacore::Vect3& pos, dynacore::Quaternion& quat);
    void preparePoseFromTransform(dynacore::Vect3& pos, dynacore::Quaternion& quat,
                                  geometry_msgs::TransformStamped tf_msg);
    bool prepareCartesianHandGoals(dynacore::Vect3& left_pos, dynacore::Quaternion& left_quat,
                                   dynacore::Vect3& right_pos, dynacore::Quaternion& right_quat,
                                   std::string& frame_id, std::vector<int>& controlled_links);
    void prepareControlledLinksFromMoveItTraj(std::vector<int>& controlled_links);
    void prepareConfigurationVector();

private:
    ros::NodeHandle nh_; // node handler

    std::string pelvis_tf_topic_; // topic to subscribe to for listening to pelvis transforms
    ros::Subscriber pelvis_transform_sub_; // subscriber for listening for pelvis transforms in world frame
    std::string controlled_link_topic_; // topic to subscribe to for listening to controlled links
    ros::Subscriber controlled_link_sub_; // subscriber for listening for controlled link ids
    std::string joint_command_topic_; // topic to subscribe to for listening to joint commands
    ros::Subscriber joint_command_sub_; // subscriber for listening for joint commands
    std::string hand_pose_command_topic_; // topic to subscribe to for listening to Cartesian hand goals
    ros::Subscriber hand_pose_command_sub_; // subscriber for listening for Cartesian hand goals
    std::string moveit_traj_topic_; // topic to subscribe to for listening to MoveIt trajectories
    ros::Subscriber moveit_traj_sub_; // subscriber for listening for MoveIt trajectories
    std::string status_topic_; // topic to subscribe to for listening to statuses
    ros::Subscriber status_sub_; // subscriber for listening for statuses
    std::string receive_cartesian_goals_topic_; // topic to subscribe to for listening to Cartesian goal updates
    ros::Subscriber receive_cartesian_goals_sub_; // subscriber for listening for Cartesian goal updates
    std::string receive_moveit_traj_topic_; // topic to subscribe to for listening to MoveIt trajectories
    ros::Subscriber receive_moveit_traj_sub_; // subscriber for listening for MoveIt trajectories
    std::string trajectory_point_time_topic_; // topic to subscribe to for setting trajectory point time
    ros::Subscriber trajectory_point_time_sub_; // subscriber for listening for trajectory point times
    std::string status_; // string indicating current status

    ros::Publisher wholebody_pub_; // publisher for wholebody messages
    ros::Publisher go_home_pub_; // publisher for go home messages
    ros::Publisher finger_pub_; // publisher for finger messages
    ros::Publisher abort_walking_pub_; // publisher for abort walking messages
    ros::Publisher pause_walking_pub_; // publisher for pause/resume walking messages
    ros::Publisher stop_all_traj_pub_; // publisher for stop all trajectory messages

    bool commands_from_controllers_; // flag indicating whether joint commands are coming from controllers (affects queueing properties of messages)
    bool cartesian_hand_goals_; // flag indicating whether arm commands are in Cartesian space or joint space (affects which fields of messages get set)
    bool receive_pelvis_transform_; // flag indicating whether to accept pelvis transforms
    bool received_pelvis_transform_; // flag indicating whether pelvis transform has been received
    bool receive_link_ids_; // flag indicating whether to accept link ids
    bool received_link_ids_; // flag indicating whether link ids have been received
    bool receive_joint_command_; // flag indicating whether to accept new joint commands
    bool received_joint_command_; // flag indicating whether joint command has been received
    bool received_left_hand_goal_; // flag indicating whether Cartesian left hand goal has been received
    bool received_right_hand_goal_; // flag indicating whether Cartesian right hand goal has been received
    bool receive_moveit_traj_; // flag indicating whether to accept MoveIt trajectories
    bool received_moveit_traj_; // flag indicating whether MoveIt trajectory has been received
    bool publish_commands_; // flag indicating if joint and pelvis information has been received and whole body message can be published
    bool stop_node_; // flag indicating when to publish whole body messages

    bool home_left_arm_; // flag indicating if homing message for left arm should be published
    bool home_right_arm_; // flag indicating if homing message for right arm should be published
    bool home_chest_; // flag indicating if homing message for ches should be published
    bool home_pelvis_; // flag indicating if homing message for pelvis should be published
    bool publish_go_home_command_; // flag indicating if any homing messages need to be published

    bool open_left_hand_; // flag indicating if open left hand message should be published
    bool close_left_hand_; // flag indicating if close left hand message should be published
    bool open_right_hand_; // flag indicating if open right hand message should be published
    bool close_right_hand_; // flag indicating if close right hand message should be published
    bool publish_finger_command_; // flag indicating if any finger messages need to be published
    bool publish_hand_command_; // flag indicating if any hand messages need to be published

    bool publish_abort_walking_command_; // flag indicating if abort walking message needs to be published
    bool publish_pause_walking_command_; // flag indicating if pause walking message needs to be published
    bool publish_resume_walking_command_; // flag indicating if resume walking message needs to be published
    bool publish_stop_all_traj_command_; // flag indicating if stop all trajectory message needs to be published

    bool publish_moveit_traj_; // flag indicating if MoveIt trajectory needs to be published

    dynacore::Vector q_joint_; // vector of commanded joint positions
    tf::Transform tf_pelvis_wrt_world_; // transform of pelvis in world frame
    dynacore::Vector q_; // full configuration vector, including virtual joints
    std::vector<int> controlled_links_; // vector of controlled links
    geometry_msgs::TransformStamped left_hand_target_; // target pose for left hand
    geometry_msgs::TransformStamped right_hand_target_; // target pose for right hand
    moveit_msgs::RobotTrajectory moveit_robot_traj_; // planned MoveIt robot trajectory
    double trajectory_point_time_; // time for trajectory points

    tf::TransformListener tf_;
};

#endif
