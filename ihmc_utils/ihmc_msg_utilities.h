/**
 * Utilities for Using IHMC Messages
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <iostream>
#include <memory>
#include <chrono>
#include <algorithm>
#include <vector>

#include <ihmc_utils/ihmc_msg_params.h>

#include <Utils/wrap_eigen.hpp>
#include <Utils/rosmsg_utils.hpp>
#include <Valkyrie/Valkyrie_Definition.h>
#include <Valkyrie/Valkyrie_Model.hpp>

#include <controller_msgs/ArmTrajectoryMessage.h>
#include <controller_msgs/ChestTrajectoryMessage.h>
#include <controller_msgs/FootTrajectoryMessage.h>
#include <controller_msgs/FrameInformation.h>
#include <controller_msgs/JointspaceTrajectoryMessage.h>
#include <controller_msgs/NeckTrajectoryMessage.h>
#include <controller_msgs/OneDoFJointTrajectoryMessage.h>
#include <controller_msgs/PelvisTrajectoryMessage.h>
#include <controller_msgs/QueueableMessage.h>
#include <controller_msgs/SE3TrajectoryMessage.h>
#include <controller_msgs/SE3TrajectoryPointMessage.h>
#include <controller_msgs/SelectionMatrix3DMessage.h>
#include <controller_msgs/SO3TrajectoryMessage.h>
#include <controller_msgs/SO3TrajectoryPointMessage.h>
#include <controller_msgs/SpineTrajectoryMessage.h>
#include <controller_msgs/TrajectoryPoint1DMessage.h>
#include <controller_msgs/WeightMatrix3DMessage.h>
#include <controller_msgs/WholeBodyTrajectoryMessage.h>

namespace IHMCMsgUtils {

    void testFunction();

    // FUNCTIONS FOR MAKING IHMC MESSAGES
    /*
     * makes an ArmTrajectoryMessage from the given configuration vector
     * @param q_joints, the vector containing the desired configuration for the relevant joints
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

    /*
     * makes a ChestTrajectoryMessage from the given quaternion
     * @param quat, the quaternion containing the desired chest orientation
     * @param chest_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post chest_msg populated based on the given quaternion
     */
    void makeIHMCChestTrajectoryMessage(dynacore::Quaternion quat,
                                        controller_msgs::ChestTrajectoryMessage& chest_msg,
                                        IHMCMessageParameters msg_params);

    /*
     * makes an FootTrajectoryMessage from the given pose
     * @param pos, the vector containing the desired position
     * @param quat, the quaternion containing the desired orientation
     * @param foot_msg, the message to be populated
     * @param robot_side, an integer representing which arm is being controlled
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
     * makes a JointspaceTrajectoryMessage from the given configuration vector
     * @param q_joints, the vector containing the desired configuration for the relevant joints
     * @param js_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post js_msg populated based on the given configuration
     */
    void makeIHMCJointspaceTrajectoryMessage(dynacore::Vector q_joints,
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
     * @param j_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post j_msg populated based on the given joint position
     */
    void makeIHMCOneDoFJointTrajectoryMessage(double q_joint,
                                              controller_msgs::OneDoFJointTrajectoryMessage& j_msg,
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
     * @param spine_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post spine_msg populated based on the given configuration
     */
    void makeIHMCSpineTrajectoryMessage(dynacore::Vector q_joints,
                                        controller_msgs::SpineTrajectoryMessage& spine_msg,
                                        IHMCMessageParameters msg_params);

    /*
     * makes a TrajectoryPoint1DMessage from the given joint position value
     * @param q_joint, the desired position of the joint
     * @param point_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post point_msg populated based on the given joint position
     */
    void makeIHMCTrajectoryPoint1DMessage(double q_joint,
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
     * @param wholebody_msg, the message to be populated
     * @param msg_params, the IHMCMessageParameters struct containing parameters for populating the message
     * @return none
     * @post wholebody_msg populated based on the given configuration
     */
    void makeIHMCWholeBodyTrajectoryMessage(dynacore::Vector q,
                                            controller_msgs::WholeBodyTrajectoryMessage& wholebody_msg,
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
     * get the relevant joint indices for different joint groups
     * @param joint_indices, a reference to the vector of indices that will be updated
     * @return none
     * @post joint_indices updated to contain the indices of the joints for the joint group
     */
    void getRelevantJointIndicesPelvis(std::vector<int>& joint_indices);
    void getRelevantJointIndicesLeftLeg(std::vector<int>& joint_indices);
    void getRelevantJointIndicesRightLeg(std::vector<int>& joint_indices);
    void getRelevantJointIndicesTorso(std::vector<int>& joint_indices);
    void getRelevantJointIndicesLeftArm(std::vector<int>& joint_indices);
    void getRelevantJointIndicesNeck(std::vector<int>& joint_indices);
    void getRelevantJointIndicesRightArm(std::vector<int>& joint_indices);

    /*
     * get the {orientation/poses} of the {chest/pelvis/feet} induced by the given configuration
     * @param q, the vector containing the robot configuration
     * @param q_joints, the vector containing the desired configuration for the relevant joints
     * @param chest_quat, the quaternion of the chest that will be updated
     * @param pelvis_{pos/quat}, the {position/quaternion} of the pelvis that will be updated
     * @param {l/r}foot_{pos/quat}, the {position/quaternion} of the {left/right} foot that will be updated
     * @return none
     * @post given {orientation/pose} information updated based on given configuration
     */
    void getChestOrientation(dynacore::Vector q, dynacore::Quaternion& chest_quat);
    void getPelvisPose(dynacore::Vector q_joints,
                       dynacore::Vect3& pelvis_pos, dynacore::Quaternion& pelvis_quat);
    void getFeetPoses(dynacore::Vector q,
                      dynacore::Vect3& lfoot_pos, dynacore::Quaternion& lfoot_quat,
                      dynacore::Vect3& rfoot_pos, dynacore::Quaternion& rfoot_quat);

    /*
     * checks if a particular link is in the vector of controlled links;
     * if true, then whole-body message will populate fields based on joint group;
     * if false, then whole-body message will not populate fields and will use defaults
     * @param controlled_links, the vector of controlled links to check
     * @param link_id, the link id to check
     * @return bool indicating if link_id is in controlled_links
     */
    bool checkControlledLink(std::vector<int> controlled_links, int link_id);

} // end namespace IHMCMsgUtils
