/**
 * Utilities for Using IHMC Messages
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <ihmc_utils/ihmc_msg_utilities.h>

namespace IHMCMsgUtils {

    void testFunction() {
        std::cout << "IHMCMsgUtilities HELLO WORLD!" << std::endl;
        return;
    }

    // FUNCTIONS FOR MAKING IHMC MESSAGES
    void makeIHMCAbortWalkingMessage(controller_msgs::AbortWalkingMessage& abort_msg,
                                     IHMCMessageParameters msg_params) {
        // set sequence id
        abort_msg.sequence_id = msg_params.sequence_id;

        return;
    }

    void makeIHMCArmTrajectoryMessage(dynacore::Vector q_joints,
                                      controller_msgs::ArmTrajectoryMessage& arm_msg,
                                      int robot_side,
                                      IHMCMessageParameters msg_params) {
        // set sequence id, robot side, and force execution
        arm_msg.sequence_id = msg_params.sequence_id;
        arm_msg.robot_side = robot_side;
        arm_msg.force_execution = msg_params.arm_params.force_execution;

        // construct and set JointspaceTrajectoryMessage for arm
        makeIHMCJointspaceTrajectoryMessage(q_joints, arm_msg.jointspace_trajectory, msg_params);

        return;
    }

    void makeIHMCArmTrajectoryMessage(std::vector<dynacore::Vector> joint_traj,
                                      std::vector<dynacore::Vector> joint_vels,
                                      std::vector<double> joint_traj_times,
                                      controller_msgs::ArmTrajectoryMessage& arm_msg,
                                      int robot_side,
                                      IHMCMessageParameters msg_params) {
        // set sequence id, robot side, and force execution
        arm_msg.sequence_id = msg_params.sequence_id;
        arm_msg.robot_side = robot_side;
        arm_msg.force_execution = msg_params.arm_params.force_execution;

        // construct and set JointspaceTrajectoryMessage for arm
        makeIHMCJointspaceTrajectoryMessage(joint_traj, joint_vels, joint_traj_times, arm_msg.jointspace_trajectory, msg_params);

        return;
    }

    void makeIHMCChestTrajectoryMessage(dynacore::Quaternion quat,
                                        controller_msgs::ChestTrajectoryMessage& chest_msg,
                                        IHMCMessageParameters msg_params) {
        // set sequence id
        chest_msg.sequence_id = msg_params.sequence_id;

        // construct and set SO3TrajectoryMessage for chest
        makeIHMCSO3TrajectoryMessage(quat,
                                     chest_msg.so3_trajectory,
                                     msg_params.frame_params.trajectory_reference_frame_id_pelviszup,
                                     msg_params.frame_params.data_reference_frame_id_world,
                                     msg_params);

        return;
    }

    void makeIHMCChestTrajectoryMessage(std::vector<dynacore::Quaternion> chest_traj,
                                        std::vector<double> chest_traj_times,
                                        controller_msgs::ChestTrajectoryMessage& chest_msg,
                                        IHMCMessageParameters msg_params) {
        // set sequence id
        chest_msg.sequence_id = msg_params.sequence_id;

        // construct and set SO3TrajectoryMessage for chest
        makeIHMCSO3TrajectoryMessage(chest_traj, chest_traj_times,
                                     chest_msg.so3_trajectory,
                                     msg_params.frame_params.trajectory_reference_frame_id_pelviszup,
                                     msg_params.frame_params.data_reference_frame_id_world,
                                     msg_params);

        return;
    }

    void makeIHMCFootTrajectoryMessage(dynacore::Vect3 pos,
                                       dynacore::Quaternion quat,
                                       controller_msgs::FootTrajectoryMessage& foot_msg,
                                       int robot_side,
                                       IHMCMessageParameters msg_params) {
        // set sequence id and robot side
        foot_msg.sequence_id = msg_params.sequence_id;
        foot_msg.robot_side = robot_side;

        // construct and set SE3TrajectoryMessage for foot
        makeIHMCSE3TrajectoryMessage(pos, quat,
                                     foot_msg.se3_trajectory,
                                     msg_params.frame_params.trajectory_reference_frame_id_world,
                                     msg_params.frame_params.data_reference_frame_id_world,
                                     msg_params);

        return;
    }

    void makeIHMCFrameInformationMessage(controller_msgs::FrameInformation& frame_msg,
                                         int trajectory_reference_frame_id,
                                         int data_reference_frame_id,
                                         IHMCMessageParameters msg_params) {
        // set sequence id, trajectory reference frame, and data reference frame
        frame_msg.sequence_id = msg_params.sequence_id;
        frame_msg.trajectory_reference_frame_id = trajectory_reference_frame_id;
        frame_msg.data_reference_frame_id = data_reference_frame_id;

        return;
    }

    void makeIHMCHandTrajectoryMessage(dynacore::Vect3 pos,
                                       dynacore::Quaternion quat,
                                       controller_msgs::HandTrajectoryMessage& hand_msg,
                                       int robot_side,
                                       IHMCMessageParameters msg_params) {
        // set sequence id and robot side
        hand_msg.sequence_id = msg_params.sequence_id;
        hand_msg.robot_side = robot_side;

        // set frame information based on reference frame
        int trajectory_reference_frame;
        int data_reference_frame;
        if( msg_params.frame_params.cartesian_goal_reference_frame_name == std::string("pelvis") ) {
            // pelvis frame
            trajectory_reference_frame = msg_params.frame_params.trajectory_reference_frame_id_pelviszup;
            data_reference_frame = msg_params.frame_params.data_reference_frame_id_pelviszup;
        }
        else {
            // world frame
            trajectory_reference_frame = msg_params.frame_params.trajectory_reference_frame_id_world;
            data_reference_frame = msg_params.frame_params.data_reference_frame_id_world;
        }

        // construct and set SE3TrajectoryMessage for hand
        makeIHMCSE3TrajectoryMessage(pos, quat,
                                     hand_msg.se3_trajectory,
                                     trajectory_reference_frame,
                                     data_reference_frame,
                                     msg_params);

        return;
    }

    void makeIHMCJointspaceTrajectoryMessage(dynacore::Vector q_joints,
                                             controller_msgs::JointspaceTrajectoryMessage& js_msg,
                                             IHMCMessageParameters msg_params) {
        // set sequence id
        js_msg.sequence_id = msg_params.sequence_id;

        // construct and set queueing properties message
        makeIHMCQueueableMessage(js_msg.queueing_properties, msg_params);

        // clear vector of joint trajectory messages
        js_msg.joint_trajectory_messages.clear();

        // set trajectory for each joint
        for( int i = 0 ; i < q_joints.size() ; i++ ) {
            // construct OneDoFJointTrajectoryMessage for joint
            controller_msgs::OneDoFJointTrajectoryMessage j_msg;
            makeIHMCOneDoFJointTrajectoryMessage(q_joints[i], j_msg, msg_params);

            // add OneDoFJointTrajectoryMessage to vector
            js_msg.joint_trajectory_messages.push_back(j_msg);
        }

        return;
    }

    void makeIHMCJointspaceTrajectoryMessage(std::vector<double> q_joints_vector,
                                             controller_msgs::JointspaceTrajectoryMessage& js_msg,
                                             IHMCMessageParameters msg_params) {
        // create dynacore::Vector for joints
        dynacore::Vector q_joints;

        // resize and clear vector
        q_joints.resize(q_joints_vector.size());
        q_joints.setZero();

        // convert from std::vector to dynacore::Vector
        for( int i = 0 ; i < q_joints_vector.size() ; i++ ) {
            // set corresponding entry in dynacore::Vector
            q_joints[i] = q_joints_vector[i];
        }

        // construct and set JointspaceTrajectoryMessage
        makeIHMCJointspaceTrajectoryMessage(q_joints, js_msg, msg_params);

        return;
    }

    void makeIHMCJointspaceTrajectoryMessage(std::vector<dynacore::Vector> joint_traj,
                                             std::vector<dynacore::Vector> joint_vels,
                                             std::vector<double> joint_traj_times,
                                             controller_msgs::JointspaceTrajectoryMessage& js_msg,
                                             IHMCMessageParameters msg_params) {
        // set sequence id
        js_msg.sequence_id = msg_params.sequence_id;

        // construct and set queueing properties message
        makeIHMCQueueableMessage(js_msg.queueing_properties, msg_params);

        // clear vector of joint trajectory messages
        js_msg.joint_trajectory_messages.clear();

        // get number of joints
        int num_joints = joint_traj[0].size();

        // set trajectory for each joint
        for( int i = 0 ; i < num_joints ; i++ ) {
            // construct OneDoFJointTrajectoryMessage for joint
            controller_msgs::OneDoFJointTrajectoryMessage j_msg;
            makeIHMCOneDoFJointTrajectoryMessage(i, joint_traj, joint_vels, joint_traj_times, j_msg, msg_params);

            // add OneDoFJointTrajectoryMessage to vector
            js_msg.joint_trajectory_messages.push_back(j_msg);
        }

        return;
    }

    void makeIHMCNeckTrajectoryMessage(dynacore::Vector q_joints,
                                       controller_msgs::NeckTrajectoryMessage& neck_msg,
                                       IHMCMessageParameters msg_params) {
        // set sequence id
        neck_msg.sequence_id = msg_params.sequence_id;

        // construct and set JointspaceTrajectoryMessage for neck
        makeIHMCJointspaceTrajectoryMessage(q_joints, neck_msg.jointspace_trajectory, msg_params);

        return;
    }

    void makeIHMCOneDoFJointTrajectoryMessage(double q_joint,
                                              controller_msgs::OneDoFJointTrajectoryMessage& j_msg,
                                              IHMCMessageParameters msg_params) {
        // set sequence id and weight
        j_msg.sequence_id = msg_params.sequence_id;
        j_msg.weight = msg_params.onedof_joint_params.weight;

        // clear vector of trajectory points
        j_msg.trajectory_points.clear();

        // construct TrajectoryPoint1DMessage
        controller_msgs::TrajectoryPoint1DMessage point_msg;
        makeIHMCTrajectoryPoint1DMessage(q_joint, point_msg, msg_params);

        // add TrajectoryPoint1DMessage to vector
        j_msg.trajectory_points.push_back(point_msg);

        return;
    }

    void makeIHMCOneDoFJointTrajectoryMessage(int dof_idx,
                                              std::vector<dynacore::Vector> joint_traj,
                                              std::vector<dynacore::Vector> joint_vels,
                                              std::vector<double> joint_traj_times,
                                              controller_msgs::OneDoFJointTrajectoryMessage& j_msg,
                                              IHMCMessageParameters msg_params) {
        // set sequence id and weight
        j_msg.sequence_id = msg_params.sequence_id;
        j_msg.weight = msg_params.onedof_joint_params.weight;

        // clear vector of trajectory points
        j_msg.trajectory_points.clear();

        // construct position for each waypoint
        for( int i = 0 ; i < joint_traj.size() ; i++ ) {
            // set trajectory point time based on waypoint time
            msg_params.traj_point_params.time = joint_traj_times[i];
            // set sequence id to be consecutively increasing
            if( i != 0 ) {
                // only update when not the first trajectory point in the sequence
                // first trajectory point will have same sequence id as message
                msg_params.sequence_id++;
            }
            // NOTE: these are only local changes to the msg_params struct

            // get waypoint
            dynacore::Vector traj_waypoint = joint_traj[i];
            // get velocity
            dynacore::Vector traj_velocity = joint_vels[i];

            // construct TrajectoryPoint1DMessage
            controller_msgs::TrajectoryPoint1DMessage point_msg;
            makeIHMCTrajectoryPoint1DMessage(traj_waypoint[dof_idx], traj_velocity[dof_idx], point_msg, msg_params);

            // add TrajectoryPoint1DMessage to vector
            j_msg.trajectory_points.push_back(point_msg);
        }

        return;
    }

    void makeIHMCPauseWalkingMessage(controller_msgs::PauseWalkingMessage& pause_msg,
                                     bool pause,
                                     IHMCMessageParameters msg_params) {
        // set sequence id
        pause_msg.sequence_id = msg_params.sequence_id;

        // set pause/unpause flag
        pause_msg.pause = pause;

        return;
    }

    void makeIHMCPauseWalkingMessage(controller_msgs::PauseWalkingMessage& pause_msg,
                                     IHMCMessageParameters msg_params) {
        // make pause message to pause walking
        makeIHMCPauseWalkingMessage(pause_msg, true, msg_params);

        return;
    }

    void makeIHMCResumeWalkingMessage(controller_msgs::PauseWalkingMessage& resume_msg,
                                      IHMCMessageParameters msg_params) {
        // make pause message to unpause/resume walking
        makeIHMCPauseWalkingMessage(resume_msg, false, msg_params);

        return;
    }

    void makeIHMCPelvisTrajectoryMessage(dynacore::Vector q_joints,
                                         controller_msgs::PelvisTrajectoryMessage& pelvis_msg,
                                         IHMCMessageParameters msg_params) {
        // set sequence id, force execution, user mode, user mode during walking
        pelvis_msg.sequence_id = msg_params.sequence_id;
        pelvis_msg.force_execution = msg_params.pelvis_params.force_execution;
        pelvis_msg.enable_user_pelvis_control = msg_params.pelvis_params.enable_user_pelvis_control;
        pelvis_msg.enable_user_pelvis_control_during_walking = msg_params.pelvis_params.enable_user_pelvis_control_during_walking;

        // get pose from given configuration
        dynacore::Vect3 pelvis_pos;
        dynacore::Quaternion pelvis_quat;
        getPelvisPose(q_joints, pelvis_pos, pelvis_quat);

        // construct and set SE3TrajectoryMessage for pelvis
        makeIHMCSE3TrajectoryMessage(pelvis_pos, pelvis_quat,
                                     pelvis_msg.se3_trajectory,
                                     msg_params.frame_params.trajectory_reference_frame_id_world,
                                     msg_params.frame_params.data_reference_frame_id_world,
                                     msg_params);

        return;
    }

    void makeIHMCQueueableMessage(controller_msgs::QueueableMessage& q_msg,
                                  IHMCMessageParameters msg_params) {
        // set sequence id, execution mode, and message id
        q_msg.sequence_id = msg_params.sequence_id;
        q_msg.execution_mode = msg_params.queueable_params.execution_mode;
        q_msg.message_id = msg_params.queueable_params.message_id;

        // if queueing messages, set previous message id
        if( msg_params.queueable_params.execution_mode == 1 ) {
            q_msg.previous_message_id = msg_params.queueable_params.previous_message_id;
        }

        // if streaming messages, set integration duration
        if( msg_params.queueable_params.execution_mode == 2 ) {
            q_msg.stream_integration_duration = msg_params.queueable_params.stream_integration_duration;
        }

        // get current time for timestamp
        auto t = std::chrono::system_clock::now();
        // set timestamp in nanoseconds when the message was created
        q_msg.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();

        return;
    }

    void makeIHMCSE3TrajectoryMessage(dynacore::Vect3 pos, dynacore::Quaternion quat,
                                      controller_msgs::SE3TrajectoryMessage& se3_msg,
                                      int trajectory_reference_frame_id,
                                      int data_reference_frame_id,
                                      IHMCMessageParameters msg_params) {
        // set sequence id and custom control frame flag
        se3_msg.sequence_id = msg_params.sequence_id;
        se3_msg.use_custom_control_frame = msg_params.se3so3_params.use_custom_control_frame;

        // construct and set custom control frame pose (setting pose to all zeros)
        ROSMsgUtils::makeZeroPoseMessage(se3_msg.control_frame_pose);

        // construct and set queueing properties message
        makeIHMCQueueableMessage(se3_msg.queueing_properties, msg_params);

        // construct and set frame information
        makeIHMCFrameInformationMessage(se3_msg.frame_information,
                                        trajectory_reference_frame_id,
                                        data_reference_frame_id,
                                        msg_params);

        // construct and set selection matrices
        makeIHMCSelectionMatrix3DMessage(se3_msg.angular_selection_matrix, msg_params);
        makeIHMCSelectionMatrix3DMessage(se3_msg.linear_selection_matrix, msg_params);

        // construct and set weight matrices
        makeIHMCWeightMatrix3DMessage(se3_msg.angular_weight_matrix, msg_params);
        makeIHMCWeightMatrix3DMessage(se3_msg.linear_weight_matrix, msg_params);

        // clear vector of trajectory points
        se3_msg.taskspace_trajectory_points.clear();

        // construct SE3TrajectoryPointMessage
        controller_msgs::SE3TrajectoryPointMessage se3_point_msg;
        makeIHMCSE3TrajectoryPointMessage(pos, quat, se3_point_msg, msg_params);

        // add SE3TrajectoryPointMessage to vector
        se3_msg.taskspace_trajectory_points.push_back(se3_point_msg);

        return;
    }

    void makeIHMCSE3TrajectoryPointMessage(dynacore::Vect3 pos, dynacore::Quaternion quat,
                                           controller_msgs::SE3TrajectoryPointMessage& se3_point_msg,
                                           IHMCMessageParameters msg_params) {
        // set sequence id and time
        se3_point_msg.sequence_id = msg_params.sequence_id;
        se3_point_msg.time = msg_params.traj_point_params.time;

        // set position based on given position
        ROSMsgUtils::makePointMessage(pos, se3_point_msg.position);

        // set orientation based on given orientation
        ROSMsgUtils::makeQuaternionMessage(quat, se3_point_msg.orientation);

        // set linear and angular velocity to zero
        ROSMsgUtils::makeZeroVector3Message(se3_point_msg.linear_velocity);
        ROSMsgUtils::makeZeroVector3Message(se3_point_msg.angular_velocity);

        return;
    }

    void makeIHMCSelectionMatrix3DMessage(controller_msgs::SelectionMatrix3DMessage& selmat_msg,
                                          IHMCMessageParameters msg_params) {
        // set sequence id, selection frame id, and axes to select
        selmat_msg.sequence_id = msg_params.sequence_id;
        selmat_msg.selection_frame_id = msg_params.selection_matrix_params.selection_frame_id;
        selmat_msg.x_selected = msg_params.selection_matrix_params.x_selected;
        selmat_msg.y_selected = msg_params.selection_matrix_params.y_selected;
        selmat_msg.z_selected = msg_params.selection_matrix_params.z_selected;

        return;
    }

    void makeIHMCSO3TrajectoryMessage(dynacore::Quaternion quat,
                                      controller_msgs::SO3TrajectoryMessage& so3_msg,
                                      int trajectory_reference_frame_id,
                                      int data_reference_frame_id,
                                      IHMCMessageParameters msg_params) {
        // set sequence id and custom control frame flag
        so3_msg.sequence_id = msg_params.sequence_id;
        so3_msg.use_custom_control_frame = msg_params.se3so3_params.use_custom_control_frame;

        // construct and set custom control frame pose (setting pose to all zeros)
        ROSMsgUtils::makeZeroPoseMessage(so3_msg.control_frame_pose);

        // construct and set queueing properties message
        makeIHMCQueueableMessage(so3_msg.queueing_properties, msg_params);

        // construct and set frame information
        makeIHMCFrameInformationMessage(so3_msg.frame_information,
                                        trajectory_reference_frame_id,
                                        data_reference_frame_id,
                                        msg_params);

        // construct and set selection matrix
        makeIHMCSelectionMatrix3DMessage(so3_msg.selection_matrix, msg_params);

        // construct and set weight matrix
        makeIHMCWeightMatrix3DMessage(so3_msg.weight_matrix, msg_params);

        // clear vector of trajectory points
        so3_msg.taskspace_trajectory_points.clear();

        // construct SO3TrajectoryPointMessage
        controller_msgs::SO3TrajectoryPointMessage so3_point_msg;
        makeIHMCSO3TrajectoryPointMessage(quat, so3_point_msg, msg_params);

        // add SO3TrajectoryPointMessage to vector
        so3_msg.taskspace_trajectory_points.push_back(so3_point_msg);

        return;
    }

    void makeIHMCSO3TrajectoryMessage(std::vector<dynacore::Quaternion> chest_traj,
                                      std::vector<double> chest_traj_times,
                                      controller_msgs::SO3TrajectoryMessage& so3_msg,
                                      int trajectory_reference_frame_id,
                                      int data_reference_frame_id,
                                      IHMCMessageParameters msg_params) {
        // set sequence id and custom control frame flag
        so3_msg.sequence_id = msg_params.sequence_id;
        so3_msg.use_custom_control_frame = msg_params.se3so3_params.use_custom_control_frame;

        // construct and set custom control frame pose (setting pose to all zeros)
        ROSMsgUtils::makeZeroPoseMessage(so3_msg.control_frame_pose);

        // construct and set queueing properties message
        makeIHMCQueueableMessage(so3_msg.queueing_properties, msg_params);

        // construct and set frame information
        makeIHMCFrameInformationMessage(so3_msg.frame_information,
                                        trajectory_reference_frame_id,
                                        data_reference_frame_id,
                                        msg_params);

        // construct and set selection matrix
        makeIHMCSelectionMatrix3DMessage(so3_msg.selection_matrix, msg_params);

        // construct and set weight matrix
        makeIHMCWeightMatrix3DMessage(so3_msg.weight_matrix, msg_params);

        // clear vector of trajectory points
        so3_msg.taskspace_trajectory_points.clear();

        // construct orientation for each waypoint
        for( int i = 0 ; i < chest_traj.size() ; i++ ) {
            // set trajectory point time based on waypoint time
            msg_params.traj_point_params.time = chest_traj_times[i];
            // NOTE: this is only a locla change to the msg_params struct

            // get waypoint
            dynacore::Quaternion traj_waypoint = chest_traj[i];

            // construct SO3TrajectoryPointMessage
            controller_msgs::SO3TrajectoryPointMessage so3_point_msg;
            makeIHMCSO3TrajectoryPointMessage(traj_waypoint, so3_point_msg, msg_params);

            // add SO3TrajectoryPointMessage to vector
            so3_msg.taskspace_trajectory_points.push_back(so3_point_msg);
        }

        return;
    }

    void makeIHMCSO3TrajectoryPointMessage(dynacore::Quaternion quat,
                                           controller_msgs::SO3TrajectoryPointMessage& so3_point_msg,
                                           IHMCMessageParameters msg_params) {
        // set sequence id and time
        so3_point_msg.sequence_id = msg_params.sequence_id;
        so3_point_msg.time = msg_params.traj_point_params.time;

        // set quaternion based on given orientation
        ROSMsgUtils::makeQuaternionMessage(quat, so3_point_msg.orientation);

        // set angular velocity to zero
        ROSMsgUtils::makeZeroVector3Message(so3_point_msg.angular_velocity);

        return;
    }

    void makeIHMCSpineTrajectoryMessage(dynacore::Vector q_joints,
                                        controller_msgs::SpineTrajectoryMessage& spine_msg,
                                        IHMCMessageParameters msg_params) {
        // set sequence id
        spine_msg.sequence_id = msg_params.sequence_id;

        // construct and set JointspaceTrajectoryMessage for spine
        makeIHMCJointspaceTrajectoryMessage(q_joints, spine_msg.jointspace_trajectory, msg_params);

        return;
    }

    void makeIHMCSpineTrajectoryMessage(std::vector<dynacore::Vector> joint_traj,
                                        std::vector<dynacore::Vector> joint_vels,
                                        std::vector<double> joint_traj_times,
                                        controller_msgs::SpineTrajectoryMessage& spine_msg,
                                        IHMCMessageParameters msg_params) {
        // set sequence id
        spine_msg.sequence_id = msg_params.sequence_id;

        // construct and set JointspaceTrajectoryMessage for spine
        makeIHMCJointspaceTrajectoryMessage(joint_traj, joint_vels, joint_traj_times, spine_msg.jointspace_trajectory, msg_params);

        return;
    }

    void makeIHMCStopAllTrajectoryMessage(controller_msgs::StopAllTrajectoryMessage& stop_msg,
                                          IHMCMessageParameters msg_params) {
        // set sequence id
        stop_msg.sequence_id = msg_params.sequence_id;

        return;
    }

    void makeIHMCTrajectoryPoint1DMessage(double q_joint,
                                          controller_msgs::TrajectoryPoint1DMessage& point_msg,
                                          IHMCMessageParameters msg_params) {
        // set sequence id and time
        point_msg.sequence_id = msg_params.sequence_id;
        point_msg.time = msg_params.traj_point_params.time;

        // set desired position based on input
        point_msg.position = q_joint;

        // set desired velocity to 0
        point_msg.velocity = 0.0;

        return;
    }

    void makeIHMCTrajectoryPoint1DMessage(double q_joint, double q_vel,
                                          controller_msgs::TrajectoryPoint1DMessage& point_msg,
                                          IHMCMessageParameters msg_params) {
        // set sequence id and time
        point_msg.sequence_id = msg_params.sequence_id;
        point_msg.time = msg_params.traj_point_params.time;

        // set desired position based on input
        point_msg.position = q_joint;

        // set desired velocity based on input
        point_msg.velocity = q_vel;

        return;
    }

    void makeIHMCWeightMatrix3DMessage(controller_msgs::WeightMatrix3DMessage& wmat_msg,
                                       IHMCMessageParameters msg_params) {
        // set sequence id, weight frame id, and axis weights
        wmat_msg.sequence_id = msg_params.sequence_id;
        wmat_msg.weight_frame_id = msg_params.weight_matrix_params.weight_frame_id;
        wmat_msg.x_weight = msg_params.weight_matrix_params.x_weight;
        wmat_msg.y_weight = msg_params.weight_matrix_params.y_weight;
        wmat_msg.z_weight = msg_params.weight_matrix_params.z_weight;

        return;
    }

    void makeIHMCWholeBodyTrajectoryMessage(dynacore::Vector q,
                                            controller_msgs::WholeBodyTrajectoryMessage& wholebody_msg,
                                            IHMCMessageParameters msg_params) {
        // set sequence id
        wholebody_msg.sequence_id = msg_params.sequence_id;

        // check what links given configuration is controlling
        // we will not set whole-body message information for not controlled links
        bool control_pelvis = checkControlledLink(msg_params.controlled_links, valkyrie_link::pelvis);
        bool control_chest = checkControlledLink(msg_params.controlled_links, valkyrie_link::torso);
        bool control_rfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::rightCOP_Frame);
        bool control_lfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::leftCOP_Frame);
        bool control_rarm = checkControlledLink(msg_params.controlled_links, valkyrie_link::rightPalm);
        bool control_larm = checkControlledLink(msg_params.controlled_links, valkyrie_link::leftPalm);
        bool control_neck = checkControlledLink(msg_params.controlled_links, valkyrie_link::head);
        // not used:
        // bool control_rfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::rightFoot);
        // bool control_lfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::leftFoot);

        // HAND TRAJECTORIES (not needed)
        // do not set trajectory for left hand: wholebody_msg.left_hand_trajectory_message
        // do not set trajectory for right hand: wholebody_msg.right_hand_trajectory_message

        // ARM TRAJECTORIES
        if( control_larm ) {
            // get relevant joint indices for left arm
            std::vector<int> larm_joint_indices;
            getRelevantValDefJointIndicesLeftArm(larm_joint_indices);
            // get relevant configuration values for left arm
            dynacore::Vector q_larm;
            selectRelevantJointsConfiguration(q, larm_joint_indices, q_larm);
            // construct and set arm message for left arm
            makeIHMCArmTrajectoryMessage(q_larm,
                                         wholebody_msg.left_arm_trajectory_message,
                                         0, msg_params);
        }

        if( control_rarm ) {
            // get relevant joint indices for right arm
            std::vector<int> rarm_joint_indices;
            getRelevantValDefJointIndicesRightArm(rarm_joint_indices);
            // get relevant configuration values for right arm
            dynacore::Vector q_rarm;
            selectRelevantJointsConfiguration(q, rarm_joint_indices, q_rarm);
            // construct and set arm message for right arm
            makeIHMCArmTrajectoryMessage(q_rarm,
                                         wholebody_msg.right_arm_trajectory_message,
                                         1, msg_params);
        }

        // CHEST TRAJECTORY
        if( control_chest ) {
            // get orientation of chest induced by configuration
            dynacore::Quaternion chest_quat;
            getChestOrientation(q, chest_quat);
            // construct and set chest message
            makeIHMCChestTrajectoryMessage(chest_quat, wholebody_msg.chest_trajectory_message, msg_params);
        }

        // SPINE TRAJECTORY
        /*
         * NOTE: spine trajectories work well in sim, but not on real robot;
         * the code below has been tested in sim and works,
         * but is commented out since it is unreliable in practice
         */
        /*
        if( control_chest ) {
            // get relevant joint indices for spine
            std::vector<int> torso_joint_indices;
            getRelevantValDefJointIndicesTorso(torso_joint_indices);
            // get relevant configuration values for spine
            dynacore::Vector q_spine;
            selectRelevantJointsConfiguration(q, torso_joint_indices, q_spine);
            // construct and set spine message
            makeIHMCSpineTrajectoryMessage(q_spine, wholebody_msg.spine_trajectory_message, msg_params);
        }
        */

        // PELVIS TRAJECTORY
        if( control_pelvis ) {
            // get relevant joint indices for pelvis
            std::vector<int> pelvis_joint_indices;
            getRelevantValDefJointIndicesPelvis(pelvis_joint_indices);
            // get relevant configuration values for pelvis
            dynacore::Vector q_pelvis;
            selectRelevantJointsConfiguration(q, pelvis_joint_indices, q_pelvis);
            // construct and set pelvis message
            makeIHMCPelvisTrajectoryMessage(q_pelvis, wholebody_msg.pelvis_trajectory_message, msg_params);
        }

        // FOOT TRAJECTORIES
        /*
         * NOTE: foot trajectories will be complicated to send because
         * IHMC interface has safety features to prevent moving feet when robot is already standing;
         * the code below has been tested in sim and it does seem to move the feet,
         * but not accurately due to balance issues;
         * sending foot trajectories also seems to interfere with arms,
         * so code is commented out since we will trust the robot to balance on its own
         */
        /*
        // get poses of feet induced by configuration
        dynacore::Vect3 lfoot_pos;
        dynacore::Quaternion lfoot_quat;
        dynacore::Vect3 rfoot_pos;
        dynacore::Quaternion rfoot_quat;
        getFeetPoses(q, lfoot_pos, lfoot_quat, rfoot_pos, rfoot_quat);
        if( control_lfoot ) {
            // construct and set left foot message
            makeIHMCFootTrajectoryMessage(lfoot_pos, lfoot_quat,
                                          wholebody_msg.left_foot_trajectory_message,
                                          0, msg_params);
        }
        if( control_rfoot ) {
            // construct and set right foot message
            makeIHMCFootTrajectoryMessage(rfoot_pos, rfoot_quat,
                                          wholebody_msg.left_foot_trajectory_message,
                                          1, msg_params);
        }
        */

        // NECK TRAJECTORY
        if( control_neck ) {
            // get relevant joint indices for neck
            std::vector<int> neck_joint_indices;
            getRelevantValDefJointIndicesNeck(neck_joint_indices);
            // get relevant configuration values for neck
            dynacore::Vector q_neck;
            selectRelevantJointsConfiguration(q, neck_joint_indices, q_neck);
            // construct and set neck message
            makeIHMCNeckTrajectoryMessage(q_neck, wholebody_msg.neck_trajectory_message, msg_params);
        }

        // HEAD TRAJECTORY
        // do not set trajectory for head: wholebody_msg.head_trajectory_message

        return;
    }

    void makeIHMCWholeBodyTrajectoryMessage(dynacore::Vector q,
                                            dynacore::Vect3 left_hand_pos, dynacore::Quaternion left_hand_quat,
                                            dynacore::Vect3 right_hand_pos, dynacore::Quaternion right_hand_quat,
                                            controller_msgs::WholeBodyTrajectoryMessage& wholebody_msg,
                                            IHMCMessageParameters msg_params, tf::Transform tf_hand_goal_frame_wrt_world) {
        // set sequence id
        wholebody_msg.sequence_id = msg_params.sequence_id;

        // check what links given configuration is controlling
        // we will not set whole-body message information for not controlled links
        bool control_pelvis = checkControlledLink(msg_params.controlled_links, valkyrie_link::pelvis);
        bool control_chest = checkControlledLink(msg_params.controlled_links, valkyrie_link::torso);
        bool control_rfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::rightCOP_Frame);
        bool control_lfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::leftCOP_Frame);
        bool control_rarm = checkControlledLink(msg_params.controlled_links, valkyrie_link::rightPalm);
        bool control_larm = checkControlledLink(msg_params.controlled_links, valkyrie_link::leftPalm);
        bool control_neck = checkControlledLink(msg_params.controlled_links, valkyrie_link::head);
        // not used:
        // bool control_rfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::rightFoot);
        // bool control_lfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::leftFoot);

        if( msg_params.cartesian_hand_goals ) { // cartesian goals for arms
            // HAND TRAJECTORIES

            // apply fixed offset to hand poses
            dynacore::Vect3 offset_left_hand_pos(left_hand_pos);
            dynacore::Quaternion offset_left_hand_quat(left_hand_quat);
            dynacore::Vect3 offset_right_hand_pos(right_hand_pos);
            dynacore::Quaternion offset_right_hand_quat(right_hand_quat);
            applyHandOffset(offset_left_hand_pos, offset_left_hand_quat,
                            offset_right_hand_pos, offset_right_hand_quat,
                            msg_params.frame_params.cartesian_goal_reference_frame_name,
                            tf_hand_goal_frame_wrt_world);

            if( control_larm ) {
                // construct and set hand message for left hand
                makeIHMCHandTrajectoryMessage(offset_left_hand_pos, offset_left_hand_quat,
                                              wholebody_msg.left_hand_trajectory_message,
                                              0, msg_params);
            }

            if( control_rarm ) {
                // construct and set hand message for right hand
                makeIHMCHandTrajectoryMessage(offset_right_hand_pos, offset_right_hand_quat,
                                              wholebody_msg.right_hand_trajectory_message,
                                              1, msg_params);
            }
        }
        else { // jointspace goals for arms
            // ARM TRAJECTORIES
            if( control_larm ) {
                // get relevant joint indices for left arm
                std::vector<int> larm_joint_indices;
                getRelevantValDefJointIndicesLeftArm(larm_joint_indices);
                // get relevant configuration values for left arm
                dynacore::Vector q_larm;
                selectRelevantJointsConfiguration(q, larm_joint_indices, q_larm);
                // construct and set arm message for left arm
                makeIHMCArmTrajectoryMessage(q_larm,
                                             wholebody_msg.left_arm_trajectory_message,
                                             0, msg_params);
            }

            if( control_rarm ) {
                // get relevant joint indices for right arm
                std::vector<int> rarm_joint_indices;
                getRelevantValDefJointIndicesRightArm(rarm_joint_indices);
                // get relevant configuration values for right arm
                dynacore::Vector q_rarm;
                selectRelevantJointsConfiguration(q, rarm_joint_indices, q_rarm);
                // construct and set arm message for right arm
                makeIHMCArmTrajectoryMessage(q_rarm,
                                             wholebody_msg.right_arm_trajectory_message,
                                             1, msg_params);
            }
        }

        // CHEST TRAJECTORY
        if( control_chest ) {
            // get orientation of chest induced by configuration
            dynacore::Quaternion chest_quat;
            getChestOrientation(q, chest_quat);
            // construct and set chest message
            makeIHMCChestTrajectoryMessage(chest_quat, wholebody_msg.chest_trajectory_message, msg_params);
        }

        // SPINE TRAJECTORY
        /*
         * NOTE: spine trajectories work well in sim, but not on real robot;
         * the code below has been tested in sim and works,
         * but is commented out since it is unreliable in practice
         */
        /*
        if( control_chest ) {
            // get relevant joint indices for spine
            std::vector<int> torso_joint_indices;
            getRelevantValDefJointIndicesTorso(torso_joint_indices);
            // get relevant configuration values for spine
            dynacore::Vector q_spine;
            selectRelevantJointsConfiguration(q, torso_joint_indices, q_spine);
            // construct and set spine message
            makeIHMCSpineTrajectoryMessage(q_spine, wholebody_msg.spine_trajectory_message, msg_params);
        }
        */

        // PELVIS TRAJECTORY
        if( control_pelvis ) {
            // get relevant joint indices for pelvis
            std::vector<int> pelvis_joint_indices;
            getRelevantValDefJointIndicesPelvis(pelvis_joint_indices);
            // get relevant configuration values for pelvis
            dynacore::Vector q_pelvis;
            selectRelevantJointsConfiguration(q, pelvis_joint_indices, q_pelvis);
            // construct and set pelvis message
            makeIHMCPelvisTrajectoryMessage(q_pelvis, wholebody_msg.pelvis_trajectory_message, msg_params);
        }

        // FOOT TRAJECTORIES
        /*
         * NOTE: foot trajectories will be complicated to send because
         * IHMC interface has safety features to prevent moving feet when robot is already standing;
         * the code below has been tested in sim and it does seem to move the feet,
         * but not accurately due to balance issues;
         * sending foot trajectories also seems to interfere with arms,
         * so code is commented out since we will trust the robot to balance on its own
         */
        /*
        // get poses of feet induced by configuration
        dynacore::Vect3 lfoot_pos;
        dynacore::Quaternion lfoot_quat;
        dynacore::Vect3 rfoot_pos;
        dynacore::Quaternion rfoot_quat;
        getFeetPoses(q, lfoot_pos, lfoot_quat, rfoot_pos, rfoot_quat);
        if( control_lfoot ) {
            // construct and set left foot message
            makeIHMCFootTrajectoryMessage(lfoot_pos, lfoot_quat,
                                          wholebody_msg.left_foot_trajectory_message,
                                          0, msg_params);
        }
        if( control_rfoot ) {
            // construct and set right foot message
            makeIHMCFootTrajectoryMessage(rfoot_pos, rfoot_quat,
                                          wholebody_msg.left_foot_trajectory_message,
                                          1, msg_params);
        }
        */

        // NECK TRAJECTORY
        if( control_neck ) {
            // get relevant joint indices for neck
            std::vector<int> neck_joint_indices;
            getRelevantValDefJointIndicesNeck(neck_joint_indices);
            // get relevant configuration values for neck
            dynacore::Vector q_neck;
            selectRelevantJointsConfiguration(q, neck_joint_indices, q_neck);
            // construct and set neck message
            makeIHMCNeckTrajectoryMessage(q_neck, wholebody_msg.neck_trajectory_message, msg_params);
        }

        // HEAD TRAJECTORY
        // do not set trajectory for head: wholebody_msg.head_trajectory_message

        return;
    }

    void makeIHMCWholeBodyTrajectoryMessage(moveit_msgs::RobotTrajectory moveit_robot_traj_msg,
                                            controller_msgs::WholeBodyTrajectoryMessage& wholebody_msg,
                                            IHMCMessageParameters msg_params) {
        // set sequence id
        wholebody_msg.sequence_id = msg_params.sequence_id;

        // check what links given configuration is controlling
        // we will not set whole-body message information for not controlled links
        bool control_chest = checkControlledLink(msg_params.controlled_links, valkyrie_link::torso);
        bool control_rarm = checkControlledLink(msg_params.controlled_links, valkyrie_link::rightPalm);
        bool control_larm = checkControlledLink(msg_params.controlled_links, valkyrie_link::leftPalm);
        // NOTE: when creating WholeBodyTrajectory messages from MoveIt RobotTrajectory messages, only allow
        //       control of torso and arms; this is based on joint groups defined in SRDF in val_moveit_config
        // not used by MoveIt:
        // bool control_pelvis = checkControlledLink(msg_params.controlled_links, valkyrie_link::pelvis);
        // bool control_rfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::rightCOP_Frame);
        // bool control_lfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::leftCOP_Frame);
        // bool control_neck = checkControlledLink(msg_params.controlled_links, valkyrie_link::head);
        // not used:
        // bool control_rfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::rightFoot);
        // bool control_lfoot = checkControlledLink(msg_params.controlled_links, valkyrie_link::leftFoot);

        // HAND TRAJECTORIES (not needed)
        // do not set trajectory for left hand: wholebody_msg.left_hand_trajectory_message
        // do not set trajectory for right hand: wholebody_msg.right_hand_trajectory_message

        // ARM TRAJECTORIES
        if( control_larm ) {
            // get relevant joint indices for left arm
            std::vector<int> larm_joint_indices;
            getRelevantMoveItMsgJointIndicesLeftArm(moveit_robot_traj_msg.joint_trajectory.joint_names,
                                                    larm_joint_indices);
            // get relevant trajectory for left arm
            std::vector<dynacore::Vector> larm_joint_traj;
            std::vector<dynacore::Vector> larm_joint_vels;
            std::vector<double> larm_joint_traj_times;
            selectRelevantJointsTrajectory(moveit_robot_traj_msg.joint_trajectory,
                                           larm_joint_indices,
                                           larm_joint_traj,
                                           larm_joint_vels,
                                           larm_joint_traj_times);
            // construct and set arm message for left arm
            makeIHMCArmTrajectoryMessage(larm_joint_traj, larm_joint_vels, larm_joint_traj_times,
                                         wholebody_msg.left_arm_trajectory_message,
                                         0, msg_params);
        }

        if( control_rarm ) {
            // get relevant joint indices for right arm
            std::vector<int> rarm_joint_indices;
            getRelevantMoveItMsgJointIndicesRightArm(moveit_robot_traj_msg.joint_trajectory.joint_names,
                                                     rarm_joint_indices);
            // get relevant trajectory for right arm
            std::vector<dynacore::Vector> rarm_joint_traj;
            std::vector<dynacore::Vector> rarm_joint_vels;
            std::vector<double> rarm_joint_traj_times;
            selectRelevantJointsTrajectory(moveit_robot_traj_msg.joint_trajectory,
                                           rarm_joint_indices,
                                           rarm_joint_traj,
                                           rarm_joint_vels,
                                           rarm_joint_traj_times);
            // construct and set arm message for right arm
            makeIHMCArmTrajectoryMessage(rarm_joint_traj, rarm_joint_vels, rarm_joint_traj_times,
                                         wholebody_msg.right_arm_trajectory_message,
                                         1, msg_params);
        }

        // CHEST TRAJECTORY
        if( control_chest ) {
            // get trajectory of chest orientations
            std::vector<dynacore::Quaternion> chest_traj;
            std::vector<double> chest_traj_times;
            getChestOrientationTrajectory(moveit_robot_traj_msg.joint_trajectory,
                                          chest_traj,
                                          chest_traj_times);
            // construct and set chest message
            makeIHMCChestTrajectoryMessage(chest_traj, chest_traj_times,
                                           wholebody_msg.chest_trajectory_message,
                                           msg_params);
        }

        // SPINE TRAJECTORY
        /*
         * NOTE: spine trajectories work well in sim, but not on real robot;
         * the code below has been tested in sim and works,
         * but is commented out since it is unreliable in practice
         */
        /*
        if( control_chest ) {
            // get relevant joint indices for spine
            std::vector<int> torso_joint_indices;
            getRelevantMoveItMsgJointIndicesTorso(moveit_robot_traj_msg.joint_trajectory.joint_names,
                                                  torso_joint_indices);
            // get relevant trajectory for spine
            std::vector<dynacore::Vector> spine_joint_traj;
            std::vector<dynacore::Vector> spine_joint_vels;
            std::vector<double> spine_joint_traj_times;
            selectRelevantJointsTrajectory(moveit_robot_traj_msg.joint_trajectory,
                                           torso_joint_indices,
                                           spine_joint_traj,
                                           spine_joint_vels,
                                           spine_joint_traj_times);
            // construct and set spine message
            makeIHMCSpineTrajectoryMessage(spine_joint_traj, spine_joint_vels, spine_joint_traj_times,
                                           wholebody_msg.spine_trajectory_message,
                                           msg_params);
        }
        */

        return;
    }

    void makeIHMCHomeLeftArmMessage(controller_msgs::GoHomeMessage& go_home_msg,
                                    IHMCMessageParameters msg_params)
    {
        // set body part, robot side, and trajectory time
        go_home_msg.humanoid_body_part = go_home_msg.HUMANOID_BODY_PART_ARM;
        go_home_msg.robot_side = go_home_msg.ROBOT_SIDE_LEFT;
        go_home_msg.trajectory_time = msg_params.go_home_params.trajectory_time;

        return;
    }

    void makeIHMCHomeRightArmMessage(controller_msgs::GoHomeMessage& go_home_msg,
                                     IHMCMessageParameters msg_params)
    {
        // set body part, robot side, and trajectory time
        go_home_msg.humanoid_body_part = go_home_msg.HUMANOID_BODY_PART_ARM;
        go_home_msg.robot_side = go_home_msg.ROBOT_SIDE_RIGHT;
        go_home_msg.trajectory_time = msg_params.go_home_params.trajectory_time;

        return;
    }

    void makeIHMCHomeChestMessage(controller_msgs::GoHomeMessage& go_home_msg,
                                  IHMCMessageParameters msg_params)
    {
        // set body part and trajectory time
        go_home_msg.humanoid_body_part = go_home_msg.HUMANOID_BODY_PART_CHEST;
        go_home_msg.trajectory_time = msg_params.go_home_params.trajectory_time;

        return;
    }

    void makeIHMCHomePelvisMessage(controller_msgs::GoHomeMessage& go_home_msg,
                                   IHMCMessageParameters msg_params)
    {
        // set body part and trajectory time
        go_home_msg.humanoid_body_part = go_home_msg.HUMANOID_BODY_PART_PELVIS;
        go_home_msg.trajectory_time = msg_params.go_home_params.trajectory_time;

        return;
    }

    void makeIHMCValkyrieHandFingerTrajectoryMessage(controller_msgs::ValkyrieHandFingerTrajectoryMessage& finger_msg,
                                                     int robot_side,
                                                     bool open,
                                                     IHMCMessageParameters msg_params)
    {
        // create vector of fingers
        std::vector<int> finger_selection{msg_params.finger_traj_params.thumb_finger_roll,
                                          msg_params.finger_traj_params.thumb_finger_proximal,
                                          msg_params.finger_traj_params.thumb_finger_distal,
                                          msg_params.finger_traj_params.index_finger,
                                          msg_params.finger_traj_params.middle_finger,
                                          msg_params.finger_traj_params.pinky_finger};
        
        // set motor value
        double motor_value;
        if( open ) {
            // open motor position
            motor_value = msg_params.finger_traj_params.open_motor_position;
        }
        else {
            motor_value = msg_params.finger_traj_params.close_motor_position;
        }

        // create vector of motor positions
        std::vector<double> finger_positions;
        for( int i = 0 ; i < finger_selection.size() ; i++ ) {
            finger_positions.push_back(motor_value);
        }

        // make finger trajectory message
        makeIHMCValkyrieHandFingerTrajectoryMessage(finger_msg, robot_side, finger_selection, finger_positions, msg_params);

        return;
    }

    void makeIHMCValkyrieHandFingerTrajectoryMessage(controller_msgs::ValkyrieHandFingerTrajectoryMessage& finger_msg,
                                                     int robot_side,
                                                     std::vector<int> finger_selection,
                                                     std::vector<double> finger_positions,
                                                     IHMCMessageParameters msg_params)
    {
        // set sequence id and robot side
        finger_msg.sequence_id = msg_params.sequence_id;
        finger_msg.robot_side = robot_side;

        // set motor names
        for( int i = 0 ; i < finger_selection.size() ; i++ ) {
            finger_msg.valkyrie_finger_motor_names.push_back(finger_selection[i]);
        }

        // construct and set JointspaceTrajectoryMessage for hand
        makeIHMCJointspaceTrajectoryMessage(finger_positions, finger_msg.jointspace_trajectory, msg_params);

        return;
    }

    // HELPER FUNCTIONS
    void selectRelevantJointsConfiguration(dynacore::Vector q,
                                           std::vector<int> joint_indices,
                                           dynacore::Vector& q_joints) {
        // resize and clear relevant joint configuration vector
        q_joints.resize(joint_indices.size());
        q_joints.setZero();

        // push back relevant joint positions
        for( int i = 0 ; i < joint_indices.size() ; i++ ) {
            // check for special index -1
            if( joint_indices[i] == -1 ) {
                // special index -1 indicates that joint is not included in valkyrie definition,
                // but is needed in the wholebody message
                // set joint position to 0
                q_joints[i] = 0.0;
            }
            else {
                // joint position exists in valkyrie definition
                // set based on given configuration
                q_joints[i] = q[joint_indices[i]];
            }
        }

        return;
    }

    void selectRelevantJointTrajectoryWaypoint(trajectory_msgs::JointTrajectoryPoint joint_point_msg,
                                               std::vector<int> joint_indices,
                                               dynacore::Vector& joint_waypoint,
                                               dynacore::Vector& joint_velocity,
                                               double& waypoint_time) {
        // resize and clear relevant joint configuration vector
        joint_waypoint.resize(joint_indices.size());
        joint_waypoint.setZero();

        // resize and clear relevant joint velocity vector
        joint_velocity.resize(joint_indices.size());
        joint_velocity.setZero();

        // get relevant joint positions and velocities
        for( int i = 0 ; i < joint_indices.size() ; i++ ) {
            // check for special index -1 for left and right wrists
            if( joint_indices[i] == -1 ) {
                // special index -1 indicates that joint is not included in valkyrie definition,
                // but is needed in the wholebody message
                // set joint position and velocity to 0
                joint_waypoint[i] = 0.0;
                joint_velocity[i] = 0.0;
            }
            else {
                // joint exists in valkyrie definiition
                // set joint position based on given trajectory point
                joint_waypoint[i] = joint_point_msg.positions[joint_indices[i]];
                joint_velocity[i] = joint_point_msg.velocities[joint_indices[i]];
            }
            // IMPORTANT NOTE: JointTrajectoryPoint will have accelerations set,
            //                 but IHMC messages only have fields for positions and velocities
        }

        // set joint waypoint time (convert from ros::Duration to double)
        waypoint_time = joint_point_msg.time_from_start.toSec();

        return;
    }

    void selectRelevantJointsTrajectory(trajectory_msgs::JointTrajectory joint_traj_msg,
                                        std::vector<int> joint_indices,
                                        std::vector<dynacore::Vector>& joint_traj,
                                        std::vector<dynacore::Vector>& joint_vels,
                                        std::vector<double>& joint_traj_times) {
        // clear and resize trajectory vector
        joint_traj.clear();
        joint_traj.resize(joint_traj_msg.points.size());

        // clear and resize velocities vector
        joint_vels.clear();
        joint_vels.resize(joint_traj_msg.points.size());

        // clear and resize waypoint times vector
        joint_traj_times.clear();
        joint_traj_times.resize(joint_traj_msg.points.size());

        // get relevant trajectory waypoints and times
        for( int i = 0 ; i < joint_traj_msg.points.size() ; i++ ) {
            // set each trajectory waypoint
            selectRelevantJointTrajectoryWaypoint(joint_traj_msg.points[i],
                                                  joint_indices,
                                                  joint_traj[i],
                                                  joint_vels[i],
                                                  joint_traj_times[i]);
        }

        return;
    }

    void getRelevantValDefJointIndicesPelvis(std::vector<int>& joint_indices) {
        // clear joint index vector
        joint_indices.clear();

        // push back joints for pelvis [x, y, z, rx, ry, rz, rw]
        joint_indices.push_back(valkyrie_joint::virtual_X);
        joint_indices.push_back(valkyrie_joint::virtual_Y);
        joint_indices.push_back(valkyrie_joint::virtual_Z);
        joint_indices.push_back(valkyrie_joint::virtual_Rx);
        joint_indices.push_back(valkyrie_joint::virtual_Ry);
        joint_indices.push_back(valkyrie_joint::virtual_Rz);
        joint_indices.push_back(valkyrie_joint::virtual_Rw);

        return;
    }

    void getRelevantValDefJointIndicesLeftLeg(std::vector<int>& joint_indices) {
        // clear joint index vector
        joint_indices.clear();

        // push back joints for left leg [hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll]
        joint_indices.push_back(valkyrie_joint::leftHipYaw);
        joint_indices.push_back(valkyrie_joint::leftHipRoll);
        joint_indices.push_back(valkyrie_joint::leftHipPitch);
        joint_indices.push_back(valkyrie_joint::leftKneePitch);
        joint_indices.push_back(valkyrie_joint::leftAnklePitch);
        joint_indices.push_back(valkyrie_joint::leftAnkleRoll);

        return;
    }

    void getRelevantValDefJointIndicesRightLeg(std::vector<int>& joint_indices) {
        // clear joint index vector
        joint_indices.clear();

        // push back joints for right leg [hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll]
        joint_indices.push_back(valkyrie_joint::rightHipYaw);
        joint_indices.push_back(valkyrie_joint::rightHipRoll);
        joint_indices.push_back(valkyrie_joint::rightHipPitch);
        joint_indices.push_back(valkyrie_joint::rightKneePitch);
        joint_indices.push_back(valkyrie_joint::rightAnklePitch);
        joint_indices.push_back(valkyrie_joint::rightAnkleRoll);

        return;
    }

    void getRelevantValDefJointIndicesTorso(std::vector<int>& joint_indices) {
        // clear joint index vector
        joint_indices.clear();

        // push back joints for torso [yaw, pitch, roll]
        joint_indices.push_back(valkyrie_joint::torsoYaw);
        joint_indices.push_back(valkyrie_joint::torsoPitch);
        joint_indices.push_back(valkyrie_joint::torsoRoll);

        return;
    }

    void getRelevantValDefJointIndicesLeftArm(std::vector<int>& joint_indices) {
        // clear joint index vector
        joint_indices.clear();

        // push back joints for left arm [shoulderPitch, shoulderRoll, shoulderYaw, elbowPitch, forearmYaw]
        joint_indices.push_back(valkyrie_joint::leftShoulderPitch);
        joint_indices.push_back(valkyrie_joint::leftShoulderRoll);
        joint_indices.push_back(valkyrie_joint::leftShoulderYaw);
        joint_indices.push_back(valkyrie_joint::leftElbowPitch);
        joint_indices.push_back(valkyrie_joint::leftForearmYaw);

        // push back special joint index for left wrist; not included in valkyrie definition
        joint_indices.push_back(-1); // leftWristRoll
        joint_indices.push_back(-1); // leftWristPitch

        return;
    }

    void getRelevantValDefJointIndicesNeck(std::vector<int>& joint_indices) {
        // clear joint index vector
        joint_indices.clear();

        // push back joints for neck [lowerPitch, yaw, upperPitch]
        joint_indices.push_back(valkyrie_joint::lowerNeckPitch);
        joint_indices.push_back(valkyrie_joint::neckYaw);
        joint_indices.push_back(valkyrie_joint::upperNeckPitch);

        return;
    }

    void getRelevantValDefJointIndicesRightArm(std::vector<int>& joint_indices) {
        // clear joint index vector
        joint_indices.clear();

        // push back joints for right arm [shoulderPitch, shoulderRoll, shoulderYaw, elbowPitch, forearmYaw]
        joint_indices.push_back(valkyrie_joint::rightShoulderPitch);
        joint_indices.push_back(valkyrie_joint::rightShoulderRoll);
        joint_indices.push_back(valkyrie_joint::rightShoulderYaw);
        joint_indices.push_back(valkyrie_joint::rightElbowPitch);
        joint_indices.push_back(valkyrie_joint::rightForearmYaw);

        // push back special joint index for right wrist; not included in valkyrie definition
        joint_indices.push_back(-1); // rightWristRoll
        joint_indices.push_back(-1); // rightWristPitch

        return;
    }

    void getRelevantMoveItMsgJointIndicesPelvis(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices) {
        // get relevant joint indices for pelvis
        std::vector<int> val_def_joint_indices;
        getRelevantValDefJointIndicesPelvis(val_def_joint_indices);

        // get MoveIt message joint indices
        getRelevantMoveItMsgJointIndices(moveit_msg_joint_names,
                                         val_def_joint_indices,
                                         joint_indices);

        return;
    }

    void getRelevantMoveItMsgJointIndicesLeftLeg(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices) {
        // get relevant joint indices for left leg
        std::vector<int> val_def_joint_indices;
        getRelevantValDefJointIndicesLeftLeg(val_def_joint_indices);

        // get MoveIt message joint indices
        getRelevantMoveItMsgJointIndices(moveit_msg_joint_names,
                                         val_def_joint_indices,
                                         joint_indices);

        return;
    }

    void getRelevantMoveItMsgJointIndicesRightLeg(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices) {
        // get relevant joint indices for right leg
        std::vector<int> val_def_joint_indices;
        getRelevantValDefJointIndicesRightLeg(val_def_joint_indices);

        // get MoveIt message joint indices
        getRelevantMoveItMsgJointIndices(moveit_msg_joint_names,
                                         val_def_joint_indices,
                                         joint_indices);

        return;
    }

    void getRelevantMoveItMsgJointIndicesTorso(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices) {
        // get relevant joint indices for torso
        std::vector<int> val_def_joint_indices;
        getRelevantValDefJointIndicesTorso(val_def_joint_indices);

        // get MoveIt message joint indices
        getRelevantMoveItMsgJointIndices(moveit_msg_joint_names,
                                         val_def_joint_indices,
                                         joint_indices);

        return;
    }

    void getRelevantMoveItMsgJointIndicesLeftArm(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices) {
        // get relevant joint indices for left arm
        std::vector<int> val_def_joint_indices;
        getRelevantValDefJointIndicesLeftArm(val_def_joint_indices);

        // get MoveIt message joint indices
        getRelevantMoveItMsgJointIndices(moveit_msg_joint_names,
                                         val_def_joint_indices,
                                         joint_indices);

        return;
    }

    void getRelevantMoveItMsgJointIndicesNeck(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices) {
        // get relevant joint indices for neck
        std::vector<int> val_def_joint_indices;
        getRelevantValDefJointIndicesNeck(val_def_joint_indices);

        // get MoveIt message joint indices
        getRelevantMoveItMsgJointIndices(moveit_msg_joint_names,
                                         val_def_joint_indices,
                                         joint_indices);

        return;
    }

    void getRelevantMoveItMsgJointIndicesRightArm(std::vector<std::string> moveit_msg_joint_names, std::vector<int>& joint_indices) {
        // get relevant joint indices for right arm
        std::vector<int> val_def_joint_indices;
        getRelevantValDefJointIndicesRightArm(val_def_joint_indices);

        // get MoveIt message joint indices
        getRelevantMoveItMsgJointIndices(moveit_msg_joint_names,
                                         val_def_joint_indices,
                                         joint_indices);

        return;
    }

    void getRelevantMoveItMsgJointIndices(std::vector<std::string> moveit_msg_joint_names,
                                          std::vector<int> val_def_joint_indices,
                                          std::vector<int>& joint_indices) {
        // clear joint index vector
        joint_indices.clear();

        // look through val_def joints
        for( int i = 0 ; i < val_def_joint_indices.size() ; i++ ) {
            // check for special index -1 for left and right wrists
            if( val_def_joint_indices[i] == -1 ) {
                // add special -1 joint index for MoveIt
                joint_indices.push_back(-1);
            }
            else {
                // get corresponding joint name
                std::string joint_name = val::joint_indices_to_names[val_def_joint_indices[i]];

                // find joint name in MoveIt message
                std::vector<std::string>::iterator it;
                it = std::find(moveit_msg_joint_names.begin(), moveit_msg_joint_names.end(), joint_name);
                if( it != moveit_msg_joint_names.end() ) {
                    // add index of joint name in MoveIt message
                    joint_indices.push_back(it - moveit_msg_joint_names.begin());
                }
                /*
                 * NOTE: based on how joint groups are set up in val_moveit_config, we can safely assume that all
                 * joints will be found in the joint names vector; therefore, we do not check for this condition
                 */
            }
        }

        return;
    }

    void getPelvisPose(dynacore::Vector q_joints,
                       dynacore::Vect3& pelvis_pos, dynacore::Quaternion& pelvis_quat) {
        // get position of pelvis based on given configuration
        pelvis_pos[0] = q_joints[0];
        pelvis_pos[1] = q_joints[1];
        pelvis_pos[2] = q_joints[2];
        // set orientation of pelvis based on given configuration
        pelvis_quat.x() = q_joints[3];
        pelvis_quat.y() = q_joints[4];
        pelvis_quat.z() = q_joints[5];
        pelvis_quat.w() = q_joints[6];

        return;
    }

    void getFeetPoses(dynacore::Vector q,
                      dynacore::Vect3& lfoot_pos, dynacore::Quaternion& lfoot_quat,
                      dynacore::Vect3& rfoot_pos, dynacore::Quaternion& rfoot_quat) {
        // construct robot model
        std::shared_ptr<Valkyrie_Model> robot_model(new Valkyrie_Model);

        // initialize zero velocity vector
        dynacore::Vector qdot;
        qdot.resize(valkyrie::num_qdot);
        qdot.setZero();

        // update system to reflect joint configuration
        robot_model->UpdateSystem(q, qdot);

        // get pose of left foot based on joint configuration
        robot_model->getPos(valkyrie_link::leftCOP_Frame, lfoot_pos);
        robot_model->getOri(valkyrie_link::leftCOP_Frame, lfoot_quat);

        // get pose of right foot based on joint configuration
        robot_model->getPos(valkyrie_link::rightCOP_Frame, rfoot_pos);
        robot_model->getOri(valkyrie_link::rightCOP_Frame, rfoot_quat);

        return;
    }

    void getChestOrientation(dynacore::Vector q, dynacore::Quaternion& chest_quat) {
        // construct robot model
        std::shared_ptr<Valkyrie_Model> robot_model(new Valkyrie_Model);

        // initialize zero velocity vector
        dynacore::Vector qdot;
        qdot.resize(valkyrie::num_qdot);
        qdot.setZero();

        // update system to reflect joint configuration
        robot_model->UpdateSystem(q, qdot);

        // get orientation of chest based on joint configuration
        robot_model->getOri(valkyrie_link::torso, chest_quat);

        return;
    }

    void getChestOrientation(std::vector<std::string> moveit_msg_joint_names,
                             trajectory_msgs::JointTrajectoryPoint joint_point_msg,
                             dynacore::Quaternion& chest_quat,
                             double& waypoint_time) {
        // initialize full configuration vector
        dynacore::Vector q;
        q.resize(valkyrie::num_q);
        q.setZero();
        // note that joints not controlled by joint trajectory message will be 0

        // loop through joints in joint trajectory message
        for( int i = 0 ; i < moveit_msg_joint_names.size() ; i++ ) {
            // get joint name
            std::string joint_name = moveit_msg_joint_names[i];
            // get index of joint name
            int joint_idx = val::joint_names_to_indices[joint_name];
            // set joint position in configuration vector
            q[joint_idx] = joint_point_msg.positions[i];
        }

        // get chest orientation for configuration vector
        getChestOrientation(q, chest_quat);

        // set chest waypoint time (convert from ros::Duration to double)
        waypoint_time = joint_point_msg.time_from_start.toSec();

        return;
    }

    void getChestOrientationTrajectory(trajectory_msgs::JointTrajectory joint_traj_msg,
                                       std::vector<dynacore::Quaternion>& chest_traj,
                                       std::vector<double>& chest_traj_times) {
        // clear and resize trajectory vector
        chest_traj.clear();
        chest_traj.resize(joint_traj_msg.points.size());

        // clear and resize waypoint times vector
        chest_traj_times.clear();
        chest_traj_times.resize(joint_traj_msg.points.size());

        // get each waypoint and time
        for( int i = 0 ; i < joint_traj_msg.points.size() ; i++ ) {
            // set each trajectory waypoint
            getChestOrientation(joint_traj_msg.joint_names,
                                joint_traj_msg.points[i],
                                chest_traj[i],
                                chest_traj_times[i]);
        }

        return;
    }

    bool checkControlledLink(std::vector<int> controlled_links, int link_id) {
        // check if link id is in vector
        std::vector<int>::iterator it;
        it = std::find(controlled_links.begin(), controlled_links.end(), link_id);

        return (it != controlled_links.end());
    }

    void getControlledLinksFromMoveItMsg(moveit_msgs::RobotTrajectory moveit_robot_traj_msg,
                                         std::vector<int>& controlled_links) {
        // get name of controlled joints
        std::vector<std::string> joint_names = moveit_robot_traj_msg.joint_trajectory.joint_names;

        // get corresponding joint indices
        std::vector<int> joint_indices;
        joint_indices.resize(joint_names.size());
        for( int i = 0 ; i < joint_names.size() ; i++ ) {
            joint_indices[i] = val::joint_names_to_indices[joint_names[i]];
        }

        // get links controlled by joint indices; only consider MoveIt links (torso, left hand, right hand)
        getMoveItLinksControlledByJoints(joint_indices, controlled_links);

        return;
    }

    bool checkPelvisLinkControlledByJoints(std::vector<int> joint_indices) {
        // get pelvis joint group indices
        std::vector<int> pelvis_joint_indices;
        getRelevantValDefJointIndicesPelvis(pelvis_joint_indices);

        // check if any joint indices are in pelvis joint group
        return checkJointsAgainstJointGroup(joint_indices, pelvis_joint_indices);
    }

    bool checkLeftFootLinkControlledByJoints(std::vector<int> joint_indices) {
        // get left leg joint group indices
        std::vector<int> lleg_joint_indices;
        getRelevantValDefJointIndicesLeftLeg(lleg_joint_indices);

        // check if any joint indices are in left leg joint group
        return checkJointsAgainstJointGroup(joint_indices, lleg_joint_indices);
    }

    bool checkRightFootLinkControlledByJoints(std::vector<int> joint_indices) {
        // get right leg joint group indices
        std::vector<int> rleg_joint_indices;
        getRelevantValDefJointIndicesRightLeg(rleg_joint_indices);

        // check if any joint indices are in right leg joint group
        return checkJointsAgainstJointGroup(joint_indices, rleg_joint_indices);
    }

    bool checkTorsoLinkControlledByJoints(std::vector<int> joint_indices) {
        // get torso joint group indices
        std::vector<int> torso_joint_indices;
        getRelevantValDefJointIndicesTorso(torso_joint_indices);

        // check if any joint indices are in torso joint group
        return checkJointsAgainstJointGroup(joint_indices, torso_joint_indices);
    }

    bool checkLeftPalmLinkControlledByJoints(std::vector<int> joint_indices) {
        // get left arm joint group indices
        std::vector<int> larm_joint_indices;
        getRelevantValDefJointIndicesLeftArm(larm_joint_indices);

        // check if any joint indices are in left arm joint group
        return checkJointsAgainstJointGroup(joint_indices, larm_joint_indices);
    }

    bool checkHeadLinkControlledByJoints(std::vector<int> joint_indices) {
        // get neck joint group indices
        std::vector<int> neck_joint_indices;
        getRelevantValDefJointIndicesNeck(neck_joint_indices);

        // check if any joint indices are in neck joint group
        return checkJointsAgainstJointGroup(joint_indices, neck_joint_indices);
    }

    bool checkRightPalmLinkControlledByJoints(std::vector<int> joint_indices) {
        // get right arm joint group indices
        std::vector<int> rarm_joint_indices;
        getRelevantValDefJointIndicesRightArm(rarm_joint_indices);

        // check if any joint indices are in right arm joint group
        return checkJointsAgainstJointGroup(joint_indices, rarm_joint_indices);
    }

    bool checkJointsAgainstJointGroup(std::vector<int> joint_indices,
                                      std::vector<int> joint_group_indices) {
        // loop through given joints
        for( int i = 0 ; i < joint_indices.size() ; i++ ) {
            // find given joint in joint group
            std::vector<int>::iterator it;
            it = std::find(joint_group_indices.begin(), joint_group_indices.end(), joint_indices[i]);
            if( it != joint_group_indices.end() ) {
                // one of the given joints is in the given joint group
                return true;
            }
        }

        // if we reach here, none of the given joints are in the given joint group
        return false;
    }

    void getLinksControlledByJoints(std::vector<int> joint_indices, std::vector<int>& controlled_links) {
        // construct vector of all links
        std::vector<int> all_valkyrie_links{
            valkyrie_link::pelvis,
            valkyrie_link::torso,
            valkyrie_link::rightCOP_Frame,
            valkyrie_link::leftCOP_Frame,
            valkyrie_link::rightPalm,
            valkyrie_link::leftPalm,
            valkyrie_link::head
        };

        // get links from links of interest that are controlled by given joints
        getLinksControlledByJoints(joint_indices, controlled_links, all_valkyrie_links);

        return;
    }

    void getMoveItLinksControlledByJoints(std::vector<int> joint_indices, std::vector<int>& controlled_links) {
        // construct vector of links controlled by MoveIt
        std::vector<int> moveit_links{
            valkyrie_link::torso,
            valkyrie_link::leftPalm,
            valkyrie_link::rightPalm
        };

        // get links from links of interest that are controlled by given joints
        getLinksControlledByJoints(joint_indices, controlled_links, moveit_links);
    }

    void getLinksControlledByJoints(std::vector<int> joint_indices, std::vector<int>& controlled_links,
                                    std::vector<int> links_of_interest) {
        // clear controlled links vector
        controlled_links.clear();

        // initialize iterator for finding links
        std::vector<int>::iterator it;

        // check for pelvis link
        it = std::find(links_of_interest.begin(), links_of_interest.end(), valkyrie_link::pelvis);
        if( it != links_of_interest.end() ) {
            // check if pelvis controlled by given joints
            if ( checkPelvisLinkControlledByJoints(joint_indices) ) {
                // add pelvis to controlled links
                controlled_links.push_back(valkyrie_link::pelvis);
            }
        }

        // check for torso link
        it = std::find(links_of_interest.begin(), links_of_interest.end(), valkyrie_link::torso);
        if( it != links_of_interest.end() ) {
            // check if torso controlled by given joints
            if ( checkTorsoLinkControlledByJoints(joint_indices) ) {
                // add torso to controlled links
                controlled_links.push_back(valkyrie_link::torso);
            }
        }

        // check for right foot link
        it = std::find(links_of_interest.begin(), links_of_interest.end(), valkyrie_link::rightCOP_Frame);
        if( it != links_of_interest.end() ) {
            // check if right foot controlled by given joints
            if ( checkRightFootLinkControlledByJoints(joint_indices) ) {
                // add right foot to controlled links
                controlled_links.push_back(valkyrie_link::rightCOP_Frame);
            }
        }

        // check for left foot link
        it = std::find(links_of_interest.begin(), links_of_interest.end(), valkyrie_link::leftCOP_Frame);
        if( it != links_of_interest.end() ) {
            // check if left foot controlled by given joints
            if ( checkLeftFootLinkControlledByJoints(joint_indices) ) {
                // add left foot to controlled links
                controlled_links.push_back(valkyrie_link::leftCOP_Frame);
            }
        }

        // check for right palm link
        it = std::find(links_of_interest.begin(), links_of_interest.end(), valkyrie_link::rightPalm);
        if( it != links_of_interest.end() ) {
            // check if right palm controlled by given joints
            if ( checkRightPalmLinkControlledByJoints(joint_indices) ) {
                // add right palm to controlled links
                controlled_links.push_back(valkyrie_link::rightPalm);
            }
        }

        // check for left palm link
        it = std::find(links_of_interest.begin(), links_of_interest.end(), valkyrie_link::leftPalm);
        if( it != links_of_interest.end() ) {
            // check if left palm controlled by given joints
            if ( checkLeftPalmLinkControlledByJoints(joint_indices) ) {
                // add left palm to controlled links
                controlled_links.push_back(valkyrie_link::leftPalm);
            }
        }

        // check for head link
        it = std::find(links_of_interest.begin(), links_of_interest.end(), valkyrie_link::head);
        if( it != links_of_interest.end() ) {
            // check if head controlled by given joints
            if ( checkHeadLinkControlledByJoints(joint_indices) ) {
                // add head to controlled links
                controlled_links.push_back(valkyrie_link::head);
            }
        }

        return;
    }

    void applyHandOffset(dynacore::Vect3& left_hand_pos, dynacore::Quaternion& left_hand_quat,
                         dynacore::Vect3& right_hand_pos, dynacore::Quaternion& right_hand_quat,
                         std::string frame_id, tf::Transform tf_frameid_wrt_world) {
        // NOTE: dynacore::Transforms are Eigen affine transforms
        //       in a d-dimensional space, affine transforms are (d+1)x(d+1) matrices
        //       where the last row is [0 ... 0 1]
        //       affine transforms are structured as follows:
        //          [[linear    translation]
        //           [0 ... 0        1     ]]
        //       so in a 4x4 affine transformation,
        //       the linear part of the transform represents the rotation and
        //       the translation part of the transform represents the translation

        // GET INPUT POSES IN WORLD FRAME
        // initialize targets in world frame
        dynacore::Vect3 left_hand_pos_wrt_world;
        dynacore::Quaternion left_hand_quat_wrt_world;
        dynacore::Vect3 right_hand_pos_wrt_world;
        dynacore::Quaternion right_hand_quat_wrt_world;

        // check target hand pose frame ID and transform into world, if necessary
        bool transformed_targets = false;
        if( frame_id.compare(std::string("world")) != 0 ) {
            // transformation needed, set flag
            transformed_targets = true;

            // transform targets into world frame
            transformDynacorePose(left_hand_pos, left_hand_quat,
                                  left_hand_pos_wrt_world, left_hand_quat_wrt_world,
                                  tf_frameid_wrt_world);
            transformDynacorePose(right_hand_pos, right_hand_quat,
                                  right_hand_pos_wrt_world, right_hand_quat_wrt_world,
                                  tf_frameid_wrt_world);
        }
        else {
            // already in world frame, set targets
            left_hand_pos_wrt_world = left_hand_pos;
            left_hand_quat_wrt_world = left_hand_quat;
            right_hand_pos_wrt_world = right_hand_pos;
            right_hand_quat_wrt_world = right_hand_quat;
        }

        // APPLY HAND OFFSET
        // initialize hand transforms
        dynacore::Transform left_pose_wrt_world;
        left_pose_wrt_world.translation() = left_hand_pos_wrt_world;
        left_pose_wrt_world.linear() = left_hand_quat_wrt_world.normalized().toRotationMatrix();
        dynacore::Transform right_pose_wrt_world;
        right_pose_wrt_world.translation() = right_hand_pos_wrt_world;
        right_pose_wrt_world.linear() = right_hand_quat_wrt_world.normalized().toRotationMatrix();

        // initialize hand offset transform for left hand
        dynacore::Vect3 hand_translation_offset_left;
        hand_translation_offset_left << 0.025, 0.07, 0.0; // translate opposite y direction from right
        Eigen::AngleAxisd hand_rotational_offset_left(M_PI/2, dynacore::Vect3(0, 0, 1)); // rotate opposite way from right

        // initialize hand offset transform for left hand
        dynacore::Transform hand_offset_left;
        hand_offset_left.translation() = hand_translation_offset_left;
        hand_offset_left.linear() = hand_rotational_offset_left.toRotationMatrix();

        // initialize hand offset transform for right hand
        dynacore::Vect3 hand_translation_offset_right;
        hand_translation_offset_right << 0.025, -0.07, 0.0;
        Eigen::AngleAxisd hand_rotational_offset_right(M_PI/2, dynacore::Vect3(0, 0, -1));

        // initialize hand offset transform for right hand
        dynacore::Transform hand_offset_right;
        hand_offset_right.translation() = hand_translation_offset_right;
        hand_offset_right.linear() = hand_rotational_offset_right.toRotationMatrix();

        // compute transformed poses
        dynacore::Transform offset_left_pose_wrt_world = left_pose_wrt_world * hand_offset_left;
        dynacore::Transform offset_right_pose_wrt_world = right_pose_wrt_world * hand_offset_right;

        // initialize offsets in world frame
        dynacore::Vect3 offset_left_hand_pos_wrt_world(offset_left_pose_wrt_world.translation());
        dynacore::Quaternion offset_left_hand_quat_wrt_world(offset_left_pose_wrt_world.linear());
        dynacore::Vect3 offset_right_hand_pos_wrt_world(offset_right_pose_wrt_world.translation());
        dynacore::Quaternion offset_right_hand_quat_wrt_world(offset_right_pose_wrt_world.linear());

        // GET POSES IN GIVEN FRAME
        // initialize targets in given frame
        dynacore::Vect3 left_hand_pos_offset;
        dynacore::Quaternion left_hand_quat_offset;
        dynacore::Vect3 right_hand_pos_offset;
        dynacore::Quaternion right_hand_quat_offset;

        // transform back to original frame, if necessary
        if( transformed_targets ) {
            tf::Transform tf_world_wrt_frameid = tf_frameid_wrt_world.inverse();
            // transform targets into given frame
            transformDynacorePose(offset_left_hand_pos_wrt_world, offset_left_hand_quat_wrt_world,
                                  left_hand_pos_offset, left_hand_quat_offset,
                                  tf_world_wrt_frameid);
            transformDynacorePose(offset_right_hand_pos_wrt_world, offset_right_hand_quat_wrt_world,
                                  right_hand_pos_offset, right_hand_quat_offset,
                                  tf_world_wrt_frameid);
        }
        else {
            // already in world frame, set targets
            left_hand_pos_offset = offset_left_hand_pos_wrt_world;
            left_hand_quat_offset = offset_left_hand_quat_wrt_world;
            right_hand_pos_offset = offset_right_hand_pos_wrt_world;
            right_hand_quat_offset = offset_right_hand_quat_wrt_world;
        }

        // UPDATE GIVEN POSES WITH OFFSET
        // update given hand poses to be offset poses
        left_hand_pos = left_hand_pos_offset;
        left_hand_quat = left_hand_quat_offset;
        left_hand_quat.normalize();
        right_hand_pos = right_hand_pos_offset;
        right_hand_quat = right_hand_quat_offset;
        right_hand_quat.normalize();

        return;
    }

    void transformDynacorePose(dynacore::Vect3 pos_in, dynacore::Quaternion quat_in,
                               dynacore::Vect3& pos_out, dynacore::Quaternion& quat_out,
                               tf::Transform tf_input_wrt_output) {
        // convert from dynacore pose to tf pose
        tf::Transform tf_wrt_input;
        dynacore::convert(pos_in, quat_in, tf_wrt_input);

        // transform pose
        tf::Transform tf_wrt_output = tf_input_wrt_output * tf_wrt_input;

        // convert from tf pose to dynacore pose
        dynacore::convert(tf_wrt_output, pos_out, quat_out);

        return;
    }

} // end namespace IHMCMsgUtilities
