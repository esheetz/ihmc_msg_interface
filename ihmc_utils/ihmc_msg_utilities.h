/**
 * Utilities for Using IHMC Messages
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <iostream>
#include <memory>
#include <chrono>
#include <algorithm>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <ihmc_utils/ihmc_msg_params.h>

#include <Utils/wrap_eigen.hpp>
#include <Utils/rosmsg_utils.hpp>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>

#include <controller_msgs/AbortWalkingMessage.h>
#include <controller_msgs/ArmTrajectoryMessage.h>
#include <controller_msgs/ChestTrajectoryMessage.h>
#include <controller_msgs/FootTrajectoryMessage.h>
#include <controller_msgs/FrameInformation.h>
#include <controller_msgs/HandTrajectoryMessage.h>
#include <controller_msgs/JointspaceTrajectoryMessage.h>
#include <controller_msgs/NeckTrajectoryMessage.h>
#include <controller_msgs/OneDoFJointTrajectoryMessage.h>
#include <controller_msgs/PauseWalkingMessage.h>
#include <controller_msgs/PelvisTrajectoryMessage.h>
#include <controller_msgs/QueueableMessage.h>
#include <controller_msgs/SE3TrajectoryMessage.h>
#include <controller_msgs/SE3TrajectoryPointMessage.h>
#include <controller_msgs/SelectionMatrix3DMessage.h>
#include <controller_msgs/SO3TrajectoryMessage.h>
#include <controller_msgs/SO3TrajectoryPointMessage.h>
#include <controller_msgs/SpineTrajectoryMessage.h>
#include <controller_msgs/StopAllTrajectoryMessage.h>
#include <controller_msgs/TrajectoryPoint1DMessage.h>
#include <controller_msgs/WeightMatrix3DMessage.h>
#include <controller_msgs/WholeBodyTrajectoryMessage.h>
#include <controller_msgs/GoHomeMessage.h>
#include <controller_msgs/ValkyrieHandFingerTrajectoryMessage.h>

namespace IHMCMsgUtils {

    void testFunction();

    // FUNCTIONS FOR MAKING IHMC MESSAGES
    /*
     * makes an AbortWalkingMessage
     * @param abort_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post abort_msg populated
     */
    void makeIHMCAbortWalkingMessage(controller_msgs::AbortWalkingMessage& abort_msg,
                                     IHMCMessageParameters msg_params);

    /*
     * makes an ArmTrajectoryMessage from the given configuration vector
     * @param q_joints, the vector containing the desired configuration for the relevant joints
     * @param joint_traj, the vector of vectors containing the desired joint trajectory for the relevant joints
     * @param joint_vels, the vector of vectors containing the desired joint velocities for the relevant joints
     * @param joint_traj_times, the vector containing the desired joint trajectory waypoint times
     * @param arm_msg, the message to be populated
     * @param robot_side, an integer representing which arm is being controlled
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @pre robot_side is either 0 (left arm) or 1 (right arm)
     * @post arm_msg populated based on the given configuration
     */
    void makeIHMCArmTrajectoryMessage(dynacore::Vector q_joints,
                                      controller_msgs::ArmTrajectoryMessage& arm_msg,
                                      int robot_side,
                                      IHMCMessageParameters msg_params);
    void makeIHMCArmTrajectoryMessage(std::vector<dynacore::Vector> joint_traj,
                                      std::vector<dynacore::Vector> joint_vels,
                                      std::vector<double> joint_traj_times,
                                      controller_msgs::ArmTrajectoryMessage& arm_msg,
                                      int robot_side,
                                      IHMCMessageParameters msg_params);

    /*
     * makes a ChestTrajectoryMessage from the given quaternion
     * @param quat, the quaternion containing the desired chest orientation
     * @param chest_traj, the vector of quaternions containing the desired chest orientation trajectory
     * @param chest_traj_times, the vector containing the desired chest orientation trajectory waypoint times
     * @param chest_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post chest_msg populated based on the given quaternion
     */
    void makeIHMCChestTrajectoryMessage(dynacore::Quaternion quat,
                                        controller_msgs::ChestTrajectoryMessage& chest_msg,
                                        IHMCMessageParameters msg_params);
    void makeIHMCChestTrajectoryMessage(std::vector<dynacore::Quaternion> chest_traj,
                                        std::vector<double> chest_traj_times,
                                        controller_msgs::ChestTrajectoryMessage& chest_msg,
                                        IHMCMessageParameters msg_params);

    /*
     * makes a FootTrajectoryMessage from the given pose
     * @param pos, the vector containing the desired position
     * @param quat, the quaternion containing the desired orientation
     * @param foot_msg, the message to be populated
     * @param robot_side, an integer representing which foot is being controlled
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @pre robot_side is either 0 (left arm) or 1 (right arm)
     * @post foot_msg populated based on the given configuration
     */
    void makeIHMCFootTrajectoryMessage(dynacore::Vect3 pos,
                                       dynacore::Quaternion quat,
                                       controller_msgs::FootTrajectoryMessage& foot_msg,
                                       int robot_side,
                                       IHMCMessageParameters msg_params);

    /*
     * makes a FrameInformation message
     * @param frame_msg, the message to be populated
     * @param trajectory_reference_frame_id, the reference frame id (provided explicitly for differences in the same whole-body message)
     * @param data_reference_frame_id, the reference frame id for data in a packet (provided explicitly for differences in the same whole-body message)
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post frame_msg populated
     */
    void makeIHMCFrameInformationMessage(controller_msgs::FrameInformation& frame_msg,
                                         int trajectory_reference_frame_id,
                                         int data_reference_frame_id,
                                         IHMCMessageParameters msg_params);

    /*
     * makes a HandTrajectoryMessage from the given pose
     * @param pos, the vector containing the desired position
     * @param quat, the quaternion containing the desired position
     * @param hand_msg, the message to be populated
     * @param robot_side, an integer representing which arm is being controlled
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @pre robot_side is either 0 (left arm) or 1 (right arm)
     * @post hand_msg populated based on the given configuration
     */
    void makeIHMCHandTrajectoryMessage(dynacore::Vect3 pos,
                                       dynacore::Quaternion quat,
                                       controller_msgs::HandTrajectoryMessage& hand_msg,
                                       int robot_side,
                                       IHMCMessageParameters msg_params);

    /*
     * makes a JointspaceTrajectoryMessage from the given configuration vector
     * @param q_joints, the vector containing the desired configuration for the relevant joints
     * @param q_joints_vector, the vector containing the desired configuration for the relevant joints
     * @param joint_traj, the vector of vectors containing the desired joint trajectory for the relevant joints
     * @param joint_vels, the vector of vectors containing the desired joint velocities for the relevant joints
     * @param joint_traj_times, the vector containing the desired joint trajectory waypoint times
     * @param js_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post js_msg populated based on the given configuration
     */
    void makeIHMCJointspaceTrajectoryMessage(dynacore::Vector q_joints,
                                             controller_msgs::JointspaceTrajectoryMessage& js_msg,
                                             IHMCMessageParameters msg_params);
    void makeIHMCJointspaceTrajectoryMessage(std::vector<double> q_joints_vector,
                                             controller_msgs::JointspaceTrajectoryMessage& js_msg,
                                             IHMCMessageParameters msg_params);
    void makeIHMCJointspaceTrajectoryMessage(std::vector<dynacore::Vector> joint_traj,
                                             std::vector<dynacore::Vector> joint_vels,
                                             std::vector<double> joint_traj_times,
                                             controller_msgs::JointspaceTrajectoryMessage& js_msg,
                                             IHMCMessageParameters msg_params);

    /*
     * makes a NeckTrajectoryMessage from the given configuration vector
     * @param q_joints, the vector containing the desired configuration for the relevant joints
     * @param neck_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post neck_msg populated based on the given configuration
     */
    void makeIHMCNeckTrajectoryMessage(dynacore::Vector q_joints,
                                       controller_msgs::NeckTrajectoryMessage& neck_msg,
                                       IHMCMessageParameters msg_params);

    /*
     * makes a OneDoFJointTrajectoryMessage from the given joint position value
     * @param q_joint, the desired position of the joint
     * @param dof_idx, the index of the degree of freedom in the joint trajectory
     * @param joint_traj, the vector of vectors containing the desired joint trajectory for the relevant joints
     * @param joint_vels, the vector of vectors containing the desired joint velocities for the relevant joints
     * @param joint_traj_times, the vector containing the desired joint trajectory waypoint times
     * @param j_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post j_msg populated based on the given joint position
     */
    void makeIHMCOneDoFJointTrajectoryMessage(double q_joint,
                                              controller_msgs::OneDoFJointTrajectoryMessage& j_msg,
                                              IHMCMessageParameters msg_params);
    void makeIHMCOneDoFJointTrajectoryMessage(int dof_idx,
                                              std::vector<dynacore::Vector> joint_traj,
                                              std::vector<dynacore::Vector> joint_vels,
                                              std::vector<double> joint_traj_times,
                                              controller_msgs::OneDoFJointTrajectoryMessage& j_msg,
                                              IHMCMessageParameters msg_params);

    /*
     * makes a PauseWalkingMessage
     * @param pause_msg, the message to be populated
     * @param pause, a boolean indicating whether to pause or unpause/resume footsteps
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post pause_msg populated
     */
    void makeIHMCPauseWalkingMessage(controller_msgs::PauseWalkingMessage& pause_msg,
                                     bool pause,
                                     IHMCMessageParameters msg_params);

    /*
     * makes a PauseWalkingMessage to either pause or resume walking
     * @param pause_msg, the message to be populated
     * @param resume_msg, the message to be populated
     * @return none
     * @post pause_msg populated
     * @post resume_msg populated
     */
    void makeIHMCPauseWalkingMessage(controller_msgs::PauseWalkingMessage& pause_msg,
                                     IHMCMessageParameters msg_params);
    void makeIHMCResumeWalkingMessage(controller_msgs::PauseWalkingMessage& resume_msg,
                                      IHMCMessageParameters msg_params);

    /*
     * makes a PelvisTrajectoryMessage from the given configuration vector
     * @param q_joints, the vector containing the desired configuration for the relevant joints
     * @param pelvis_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post pelvis_msg populated based on the given configuration
     */
    void makeIHMCPelvisTrajectoryMessage(dynacore::Vector q_joints,
                                         controller_msgs::PelvisTrajectoryMessage& pelvis_msg,
                                         IHMCMessageParameters msg_params);

    /*
     * makes a QueueableMessage // TODO parameters will likely change
     * @param q_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post q_msg populated based on the given parameters
     */
    void makeIHMCQueueableMessage(controller_msgs::QueueableMessage& q_msg,
                                  IHMCMessageParameters msg_params);

    /*
     * makes an SE3TrajectoryMessage from the given configuration vector
     * @param pos, the vector containing the desired position
     * @param quat, the quaternion containing the desired orientation
     * @param se3_msg, the message to be populated
     * @param trajectory_reference_frame_id, the reference frame id (provided explicitly for differences in the same whole-body message)
     * @param data_reference_frame_id, the reference frame id for data in a packet (provided explicitly for differences in the same whole-body message)
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post se3_msg populated based on the given configuration
     */
    void makeIHMCSE3TrajectoryMessage(dynacore::Vect3 pos, dynacore::Quaternion quat,
                                      controller_msgs::SE3TrajectoryMessage& se3_msg,
                                      int trajectory_reference_frame_id,
                                      int data_reference_frame_id,
                                      IHMCMessageParameters msg_params);

    /*
     * makes an SE3TrajectoryPointMessage from the given configuration vector
     * @param pos, the vector containing the desired position
     * @param quat, the quaternion containing the desired orientation
     * @param se3_point_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post se3_point_msg populated based on the given configuration
     */
    void makeIHMCSE3TrajectoryPointMessage(dynacore::Vect3 pos, dynacore::Quaternion quat,
                                           controller_msgs::SE3TrajectoryPointMessage& se3_point_msg,
                                           IHMCMessageParameters msg_params);

    /*
     * makes a SelectionMatrix3DMessage
     * @param selmat_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post selmat_msg populated
     */
    void makeIHMCSelectionMatrix3DMessage(controller_msgs::SelectionMatrix3DMessage& selmat_msg,
                                          IHMCMessageParameters msg_params);

    /*
     * makes an SO3TrajectoryMessage from the given quaternion
     * @param quat, the quaternion containing the desired orientation
     * @param chest_traj, the vector of quaternions containing the desired chest orientation trajectory
     * @param chest_traj_times, the vector containing the desired chest orientation trajectory waypoint times
     * @param so3_msg, the message to be populated
     * @param trajectory_reference_frame_id, the reference frame id (provided explicitly for differences in the same whole-body message)
     * @param data_reference_frame_id, the reference frame id for data in a packet (provided explicitly for differences in the same whole-body message)
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post so3_msg populated based on the given orientation
     */
    void makeIHMCSO3TrajectoryMessage(dynacore::Quaternion quat,
                                      controller_msgs::SO3TrajectoryMessage& so3_msg,
                                      int trajectory_reference_frame_id,
                                      int data_reference_frame_id,
                                      IHMCMessageParameters msg_params);
    void makeIHMCSO3TrajectoryMessage(std::vector<dynacore::Quaternion> chest_traj,
                                      std::vector<double> chest_traj_times,
                                      controller_msgs::SO3TrajectoryMessage& so3_msg,
                                      int trajectory_reference_frame_id,
                                      int data_reference_frame_id,
                                      IHMCMessageParameters msg_params);

    /*
     * makes an SO3TrajectoryPointMessage from the given quaternion
     * @param quat, the quaternion containing the desired orientation
     * @param so3_point_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post so3_point_msg populated based on the given orientation
     */
    void makeIHMCSO3TrajectoryPointMessage(dynacore::Quaternion quat,
                                           controller_msgs::SO3TrajectoryPointMessage& so3_point_msg,
                                           IHMCMessageParameters msg_params);

    /*
     * makes a SpineTrajectoryMessage from the given configuration vector
     * @param q_joints, the vector containing the desired configuration for the relevant joints
     * @param joint_traj, the vector of vectors containing the desired joint trajectory for the relevant joints
     * @param joint_vels, the vector of vectors containing the desired joint velocities for the relevant joints
     * @param joint_traj_times, the vector containing the desired joint trajectory waypoint times
     * @param spine_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post spine_msg populated based on the given configuration
     */
    void makeIHMCSpineTrajectoryMessage(dynacore::Vector q_joints,
                                        controller_msgs::SpineTrajectoryMessage& spine_msg,
                                        IHMCMessageParameters msg_params);
    void makeIHMCSpineTrajectoryMessage(std::vector<dynacore::Vector> joint_traj,
                                        std::vector<dynacore::Vector> joint_vels,
                                        std::vector<double> joint_traj_times,
                                        controller_msgs::SpineTrajectoryMessage& spine_msg,
                                        IHMCMessageParameters msg_params);

    /*
     * makes a StopAllTrajectoryMessage
     * @param stop_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post stop_msg populated
     */
    void makeIHMCStopAllTrajectoryMessage(controller_msgs::StopAllTrajectoryMessage& stop_msg,
                                          IHMCMessageParameters msg_params);

    /*
     * makes a TrajectoryPoint1DMessage from the given joint position value
     * @param q_joint, the desired position of the joint
     * @param q_vel, the desired velocity of the joint
     * @param point_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post point_msg populated based on the given joint position
     */
    void makeIHMCTrajectoryPoint1DMessage(double q_joint,
                                          controller_msgs::TrajectoryPoint1DMessage& point_msg,
                                          IHMCMessageParameters msg_params);
    void makeIHMCTrajectoryPoint1DMessage(double q_joint, double q_vel,
                                          controller_msgs::TrajectoryPoint1DMessage& point_msg,
                                          IHMCMessageParameters msg_params);

    /*
     * makes a WeightMatrix3DMessage
     * @param wmat_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post wmat_msg populated
     */
    void makeIHMCWeightMatrix3DMessage(controller_msgs::WeightMatrix3DMessage& wmat_msg,
                                       IHMCMessageParameters msg_params);

    /*
     * makes a WholeBodyTrajecoryMessage from the given configuration vector
     * @param q, the vector containing the desired robot configuration
     * @param left_hand_pos, the vector containing the desired left hand position
     * @param left_hand_quat, the quaternion containing the desired left hand orientation
     * @param right_hand_pos, the vector containing the desired right hand position
     * @param right_hand_quat, the quaternion containing the desired right hand orientation
     * @param moveit_robot_traj_msg, the planned MoveIt RobotTrajecory message
     * @param wholebody_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @param tf_hand_goal_frame_wrt_world, the transform of the hand goal frame to world
     * @return none
     * @post wholebody_msg populated based on the given configuration
     */
    void makeIHMCWholeBodyTrajectoryMessage(dynacore::Vector q,
                                            controller_msgs::WholeBodyTrajectoryMessage& wholebody_msg,
                                            IHMCMessageParameters msg_params);
    void makeIHMCWholeBodyTrajectoryMessage(dynacore::Vector q,
                                            dynacore::Vect3 left_hand_pos, dynacore::Quaternion left_hand_quat,
                                            dynacore::Vect3 right_hand_pos, dynacore::Quaternion right_hand_quat,
                                            controller_msgs::WholeBodyTrajectoryMessage& wholebody_msg,
                                            IHMCMessageParameters msg_params, tf::Transform tf_hand_goal_frame_wrt_world);
    void makeIHMCWholeBodyTrajectoryMessage(moveit_msgs::RobotTrajectory moveit_robot_traj_msg,
                                            controller_msgs::WholeBodyTrajectoryMessage& wholebody_msg,
                                            IHMCMessageParameters msg_params);

    /*
     * makes a GoHomeMessage for the corresponding humanoid body part
     * @param go_home_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post go_home_msg populated for the respective body part
     */
    void makeIHMCHomeLeftArmMessage(controller_msgs::GoHomeMessage& go_home_msg,
                                    IHMCMessageParameters msg_params);
    void makeIHMCHomeRightArmMessage(controller_msgs::GoHomeMessage& go_home_msg,
                                     IHMCMessageParameters msg_params);
    void makeIHMCHomeChestMessage(controller_msgs::GoHomeMessage& go_home_msg,
                                  IHMCMessageParameters msg_params);
    void makeIHMCHomePelvisMessage(controller_msgs::GoHomeMessage& go_home_msg,
                                   IHMCMessageParameters msg_params);

    /*
     * makes a ValkyrieHandFingerTrajectoryMessage from given finger selections and positions
     * @param finger_msg, the message to be populated
     * @param robot_side, an integer representing which arm is being controlled
     * @param open, a boolean indicating whether hand should be open(true) or close(false)
     * @param finger_selection, a vector of integers indicating which fingers are being controlled
     * @param finger_positions, a vector of doubles indicating the desired position for each finger
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     */
    void makeIHMCValkyrieHandFingerTrajectoryMessage(controller_msgs::ValkyrieHandFingerTrajectoryMessage& finger_msg,
                                                     int robot_side,
                                                     bool open,
                                                     IHMCMessageParameters msg_params);

    /*
     * makes a ValkyrieHandFingerTrajectoryMessage from given finger selections and positions
     * @param finger_msg, the message to be populated
     * @param robot_side, an integer representing which arm is being controlled
     * @param finger_selection, a vector of integers indicating which fingers are being controlled
     * @param finger_positions, a vector of doubles indicating the desired position for each finger
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     */
    void makeIHMCValkyrieHandFingerTrajectoryMessage(controller_msgs::ValkyrieHandFingerTrajectoryMessage& finger_msg,
                                                     int robot_side,
                                                     std::vector<int> finger_selection,
                                                     std::vector<double> finger_positions,
                                                     IHMCMessageParameters msg_params);

    // HELPER FUNCTIONS
    /*
     * select the joint positions for the relevant joints
     * @param q, the vector containing the desired robot configuration
     * @param joint_indices, the vector containing the relevant joint indices
     * @param q_joints, a reference to the vector that will be updated
     * @return none
     * @post q_joints updated to contain the desired joint positions of the relevant joints
     */
    void selectRelevantJointsConfiguration(dynacore::Vector q,
                                           std::vector<int> joint_indices,
                                           dynacore::Vector& q_joints);

    /*
     * select the joint trajectory waypoint and waypoint time for the relevant joints
     * @param joint_point_msg, the joint trajectory point message containing the desired joint positions
     * @param joint_indices, the vector containing the relevant joint indices
     * @param joint_waypoint, a reference to the vector (joint trajectory waypoint) that will be updated
     * @param joint_velocity, a reference to the vector (joint trajectory velocity) that will be updated
     * @param waypoint_time, a reference to the double (waypoint time) that will be updated
     * @return none
     * @post joint_waypoint updated to contain the desired joint positions of the relevant joints
     * @post joint_velocity updated to contain the desired joint velocities of the relevant joints
     * @post waypoint_time updated to contain the time the waypoint should be achieved
     */
    void selectRelevantJointTrajectoryWaypoint(trajectory_msgs::JointTrajectoryPoint joint_point_msg,
                                               std::vector<int> joint_indices,
                                               dynacore::Vector& joint_waypoint,
                                               dynacore::Vector& joint_velocity,
                                               double& waypoint_time);

    /*
     * select the joint trajectory and waypoint times for the relevant joints
     * @param joint_traj_msg, the joint trajectory message containing the desired joint trajectory
     * @param joint_indices, the vector containing the relevant joint indices
     * @param joint_traj, a reference to the vector of vectors (joint trajectory waypoints) that will be updated
     * @param joint_vels, a reference to the vector of vectors (joint trajectory velocities) that will be updated
     * @param joint_traj_times, a reference to the vector (waypoint times) that will be updated
     * @return none
     * @post joint_traj updated to contain the desired joint trajectory of the relevant joints
     *       NOTE: each element of joint_traj is a waypoint in the trajectory represented as a dynacore::Vector;
     *             each element represents the positions for the relevant joints at that waypoint
     * @post joint_vels updated to contain the desired joint velocities of the relevant joints
     *       NOTE: each element of joint_vels is a waypoint in the trajectory represented as a dynacore::Vector;
     *             each element represents the velocities for the relevant joints at that waypoint
     * @post joint_traj_times updated to contain the times each trajectory waypoint should be achieved
     */
    void selectRelevantJointsTrajectory(trajectory_msgs::JointTrajectory joint_traj_msg,
                                        std::vector<int> joint_indices,
                                        std::vector<dynacore::Vector>& joint_traj,
                                        std::vector<dynacore::Vector>& joint_vels,
                                        std::vector<double>& joint_traj_times);

    /*
     * get the relevant joint indices for different joint groups
     * @param joint_indices, a reference to the vector of indices that will be updated
     * @return none
     * @post joint_indices updated to contain the indices of the joints for the joint group according to Valkyrie_Definition.h
     */
    void getRelevantValDefJointIndicesPelvis(std::vector<int>& joint_indices);
    void getRelevantValDefJointIndicesLeftLeg(std::vector<int>& joint_indices);
    void getRelevantValDefJointIndicesRightLeg(std::vector<int>& joint_indices);
    void getRelevantValDefJointIndicesTorso(std::vector<int>& joint_indices);
    void getRelevantValDefJointIndicesLeftArm(std::vector<int>& joint_indices);
    void getRelevantValDefJointIndicesNeck(std::vector<int>& joint_indices);
    void getRelevantValDefJointIndicesRightArm(std::vector<int>& joint_indices);

    /*
     * get the relevant joint indices for different joint groups
     * NOTE: based on how joint groups are defined in val_moveit_config, every joint along kinematic chain is included;
     *       this means we do not have to worry about missing joints along kinematic chain
     * @param moveit_msg_joint_names, the vector of joint names from the MoveIt RobotTrajectory message
     * @param val_def_joint_indices, the indices of the joints for the joint group according to Valkyrie_Definition.h
     * @param joint_indices, a reference to the vector of indices that will be updated
     * @return none
     * @post joint_indices updated to contain the indices of the joints for the joint group based on the MoveIt RobotTrajectory message
     */
    void getRelevantMoveItMsgJointIndicesPelvis(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices);
    void getRelevantMoveItMsgJointIndicesLeftLeg(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices);
    void getRelevantMoveItMsgJointIndicesRightLeg(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices);
    void getRelevantMoveItMsgJointIndicesTorso(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices);
    void getRelevantMoveItMsgJointIndicesLeftArm(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices);
    void getRelevantMoveItMsgJointIndicesNeck(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices);
    void getRelevantMoveItMsgJointIndicesRightArm(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices);
    void getRelevantMoveItMsgJointIndices(std::vector<std::string> moveit_msg_joint_names,
                                          std::vector<int> val_def_joint_indices,
                                          std::vector<int>& joint_indices);

    /*
     * get the {orientation/poses} of the {chest/pelvis/feet} induced by the given configuration
     * @param q_joints, the vector containing the desired configuration for the relevant joints
     * @param q, the vector containing the robot configuration
     * @param joint_point_msg, the joint trajectory point message containing the desired joint positions
     * @param pelvis_{pos/quat}, a reference to the {position/quaternion} of the pelvis that will be updated
     * @param {l/r}foot_{pos/quat}, a reference to the {position/quaternion} of the {left/right} foot that will be updated
     * @param chest_quat, a reference to the quaternion of the chest that will be updated
     * @param waypoint_time, a reference to the double that will be updated
     * @return none
     * @post given {orientation/pose} information updated based on given configuration
     * @post waypoint_time update to contain the time the waypoint should be achieved
     */
    void getPelvisPose(dynacore::Vector q_joints,
                       dynacore::Vect3& pelvis_pos, dynacore::Quaternion& pelvis_quat);
    void getFeetPoses(dynacore::Vector q,
                      dynacore::Vect3& lfoot_pos, dynacore::Quaternion& lfoot_quat,
                      dynacore::Vect3& rfoot_pos, dynacore::Quaternion& rfoot_quat);
    void getChestOrientation(dynacore::Vector q, dynacore::Quaternion& chest_quat);
    void getChestOrientation(std::vector<std::string> moveit_msg_joint_names,
                             trajectory_msgs::JointTrajectoryPoint joint_point_msg,
                             dynacore::Quaternion& chest_quat,
                             double& waypoint_time);

    /*
     * get the chest orientation trajectory from the planned joint trajectory
     * @param joint_traj_msg, the joint trajectory message containing the desired joint trajectory
     * @param chest_traj, a reference to the vector of quaternions (orientation waypoints) that will be updated
     * @param chest_traj_times, a reference to the vector (waypoint times) that will be updated
     * @return none
     * @post chest_traj updated to contain the desired chest trajectory
     * @post chest_traj_times updated to contain the times each trajectory waypoint should be achieved
     */
    void getChestOrientationTrajectory(trajectory_msgs::JointTrajectory joint_traj_msg,
                                       std::vector<dynacore::Quaternion>& chest_traj,
                                       std::vector<double>& chest_traj_times);

    /*
     * checks if a particular link is in the vector of controlled links
     * if true, then whole-body message will populate fields based on joint group
     * if false, then whole-body message will not populate fields and will use defaults
     * @param controlled_links, the vector of controlled links to check
     * @param link_id, the link id to check
     * @return bool indicating if link_id is in controlled_links
     */
    bool checkControlledLink(std::vector<int> controlled_links, int link_id);

    /*
     * gets the controlled links from a MoveIt RobotTrajectory message
     * @param moveit_robot_traj_msg, the planned MoveIt RobotTrajecory message
     * @param controlled_links, a reference to the vector that will be updated
     * @return none
     * @post controlled_links updated to contain links controlled by MoveIt message
     *       NOTE: only possibilities for links controlled by MoveIt RobotTrajectory messages are
     *             torso and arms; this is based on joint groups defined in SRDF in val_moveit_config
     */
    void getControlledLinksFromMoveItMsg(moveit_msgs::RobotTrajectory moveit_robot_traj_msg,
                                         std::vector<int>& controlled_links);

    /*
     * check if a particular link is controlled by the given joints
     * @param joint_indices, the vector containing the relevant joint indices
     * @return bool indicating if link is controlled by joints
     */
    bool checkPelvisLinkControlledByJoints(std::vector<int> joint_indices);
    bool checkLeftFootLinkControlledByJoints(std::vector<int> joint_indices);
    bool checkRightFootLinkControlledByJoints(std::vector<int> joint_indices);
    bool checkTorsoLinkControlledByJoints(std::vector<int> joint_indices);
    bool checkLeftPalmLinkControlledByJoints(std::vector<int> joint_indices);
    bool checkHeadLinkControlledByJoints(std::vector<int> joint_indices);
    bool checkRightPalmLinkControlledByJoints(std::vector<int> joint_indices);

    /*
     * check if given joints match any of the joints for a particular joint group
     * @param joint_indices, the vector containing the relevant joint indices
     * @param joint_group_indices, the vector containing the joint indices of some joint group
     * @return bool indicating if any of the given joints are joints for the given joint group
     */
    bool checkJointsAgainstJointGroup(std::vector<int> joint_indices,
                                      std::vector<int> joint_group_indices);

    /*
     * get links controlled by the given joints
     * @param joint_indices, the vector containing the relevant joint indices
     * @param controlled_links, a reference to the vector that will be updated
     * @param links_of_interest, a vector of the subset of links to check
     * @return none
     * @post controlled_links updated to contain links controlled by joints
     */
    void getLinksControlledByJoints(std::vector<int> joint_indices, std::vector<int>& controlled_links);
    void getMoveItLinksControlledByJoints(std::vector<int> joint_indices, std::vector<int>& controlled_links);
    void getLinksControlledByJoints(std::vector<int> joint_indices, std::vector<int>& controlled_links,
                                    std::vector<int> links_of_interest);

    /*
     * applies the fixed hand offset to given hand goals
     * @param left_hand_pos, the vector containing the desired left hand position
     * @param left_hand_quat, the quaternion containing the desired left hand orientation
     * @param right_hand_pos, the vector containing the desired right hand position
     * @param right_hand_quat, the quaternion containing the desired right hand orientation
     * @param frame_id, a string representing the frame id for the given hand goals
     * @param tf_frameid_wrt_world, the transform from the given frame id to world
     * @return none
     * @post poses updated to reflect hand offset
     */
    void applyHandOffset(dynacore::Vect3& left_hand_pos, dynacore::Quaternion& left_hand_quat,
                         dynacore::Vect3& right_hand_pos, dynacore::Quaternion& right_hand_quat,
                         std::string frame_id, tf::Transform tf_frameid_wrt_world);

    /*
     * transforms a pose from one frame to another
     * @param pos_in, the position in the current frame
     * @param quat_in, the quaternion in the current frame
     * @param pos_out, the position in the target frame
     * @param quat_out, the quaternion in the target frame
     * @param tf_input_wrt_output, the transform from the input(current) frame to the output(target) frame
     * @return none
     * @post pos_out and quat_out updated to reflect transformed pose
     */
    void transformDynacorePose(dynacore::Vect3 pos_in, dynacore::Quaternion quat_in,
                               dynacore::Vect3& pos_out, dynacore::Quaternion& quat_out,
                               tf::Transform tf_input_wrt_output);

} // end namespace IHMCMsgUtils
