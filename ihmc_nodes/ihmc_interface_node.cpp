/**
 * IHMC Interface Node
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <ihmc_nodes/ihmc_interface_node.h>

// CONSTRUCTORS/DESTRUCTORS
IHMCInterfaceNode::IHMCInterfaceNode(const ros::NodeHandle& nh) {
    nh_ = nh;

    // set up parameters
    nh_.param("commands_from_controllers", commands_from_controllers_, true);
    std::string managing_node;
    nh_.param("managing_node", managing_node, std::string("IHMCInterfaceNode"));
    managing_node = std::string("/") + managing_node + std::string("/");
    nh_.param("pelvis_tf_topic", pelvis_tf_topic_,
              std::string("controllers/output/ihmc/pelvis_transform"));
    nh_.param("controlled_link_topic", controlled_link_topic_,
              std::string("controllers/output/ihmc/controlled_link_ids"));
    nh_.param("joint_command_topic", joint_command_topic_,
              std::string("controllers/output/ihmc/joint_commands"));
    nh_.param("status_topic", status_topic_,
              std::string("controllers/output/ihmc/controller_status"));
    nh_.param("hand_pose_command_topic", hand_pose_command_topic_,
              std::string("controllers/output/ihmc/cartesian_hand_targets"));
    nh_.param("receive_cartesian_goals_topic", receive_cartesian_goals_topic_,
              std::string("controllers/output/ihmc/receive_cartesian_goals"));
    nh_.param("moveit_traj_topic", moveit_traj_topic_,
              std::string("moveit_planned_robot_trajectory"));
    nh_.param("receive_moveit_traj_topic", receive_moveit_traj_topic_,
              std::string("receive_moveit_trajectories"));

    // if coming from controllers, update topic names to come from managing node
    if( commands_from_controllers_ ) {
        pelvis_tf_topic_ = managing_node + pelvis_tf_topic_;
        controlled_link_topic_ = managing_node + controlled_link_topic_;
        joint_command_topic_ = managing_node + joint_command_topic_;
        status_topic_ = managing_node + status_topic_;
        hand_pose_command_topic_ = managing_node + hand_pose_command_topic_;
        receive_cartesian_goals_topic_ = managing_node + receive_cartesian_goals_topic_;
        // we know MoveIt trajectory information will always come from the MoveIt Planner Executor Server
        moveit_traj_topic_ = managing_node + moveit_traj_topic_;
        receive_moveit_traj_topic_ = managing_node + receive_moveit_traj_topic_;
        trajectory_point_time_topic_ = managing_node + "/cartesian_hand_trajectory_time";
    }

    initializeConnections();

    // initialize flags for receiving and publishing messages
    if( commands_from_controllers_ ) {
        receive_pelvis_transform_ = false;
        receive_joint_command_ = false;
        receive_link_ids_ = false;
        received_link_ids_ = false;
    }
    else {
        receive_pelvis_transform_ = true;
        receive_joint_command_ = true;
        receive_link_ids_ = false;
        received_link_ids_ = true;
        // will not wait for link ids, assume all links controlled
        controlled_links_.clear();
        controlled_links_.push_back(valkyrie_link::pelvis);
        controlled_links_.push_back(valkyrie_link::torso);
        controlled_links_.push_back(valkyrie_link::rightCOP_Frame);
        controlled_links_.push_back(valkyrie_link::leftCOP_Frame);
        controlled_links_.push_back(valkyrie_link::rightPalm);
        controlled_links_.push_back(valkyrie_link::leftPalm);
        controlled_links_.push_back(valkyrie_link::head);
    }
    cartesian_hand_goals_ = true;
    received_pelvis_transform_ = false;
    received_joint_command_ = false;
    received_left_hand_goal_ = false;
    received_right_hand_goal_ = false;
    receive_moveit_traj_ = true;
    received_moveit_traj_ = false;
    publish_commands_ = false;
    stop_node_ = false;

    home_left_arm_ = false;
    home_right_arm_ = false;
    home_chest_ = false;
    home_pelvis_ = false;
    publish_go_home_command_ = false;

    open_left_hand_ = false;
    close_left_hand_ = false;
    open_right_hand_ = false;
    close_right_hand_ = false;
    publish_finger_command_ = false;
    publish_hand_command_ = false;

    publish_abort_walking_command_ = false;
    publish_pause_walking_command_ = false;
    publish_resume_walking_command_ = false;
    publish_stop_all_traj_command_ = false;

    publish_moveit_traj_ = false;

    trajectory_point_time_ = 0.0;

    // set initial empty status
    status_ = std::string("");

    std::cout << "[IHMC Interface Node] Constructed" << std::endl;
}

IHMCInterfaceNode::~IHMCInterfaceNode() {
    std::cout << "[IHMC Interface Node] Destroyed" << std::endl;
}

// CONNECTIONS
bool IHMCInterfaceNode::initializeConnections() {
    // subscribers for receiving whole-body information
    pelvis_transform_sub_ = nh_.subscribe(pelvis_tf_topic_, 1, &IHMCInterfaceNode::transformCallback, this);
    joint_command_sub_ = nh_.subscribe(joint_command_topic_, 1, &IHMCInterfaceNode::jointCommandCallback, this);
    if( commands_from_controllers_ ) {
        controlled_link_sub_ = nh_.subscribe(controlled_link_topic_, 1, &IHMCInterfaceNode::controlledLinkIdsCallback, this);
        status_sub_ = nh_.subscribe(status_topic_, 100, &IHMCInterfaceNode::statusCallback, this);
        hand_pose_command_sub_ = nh_.subscribe(hand_pose_command_topic_, 1, &IHMCInterfaceNode::handPoseCommandCallback, this);
        receive_cartesian_goals_sub_ = nh_.subscribe(receive_cartesian_goals_topic_, 1, &IHMCInterfaceNode::receiveCartesianGoalsCallback, this);
        moveit_traj_sub_ = nh_.subscribe(moveit_traj_topic_, 1, &IHMCInterfaceNode::plannedMoveItRobotTrajectoryCallback, this);
        receive_moveit_traj_sub_ = nh_.subscribe(receive_moveit_traj_topic_, 1, &IHMCInterfaceNode::receiveMoveItTrajCallback, this);
        trajectory_point_time_sub_ = nh_.subscribe(trajectory_point_time_topic_, 1, &IHMCInterfaceNode::trajectoryPointTimeCallback, this);
    }

    // publishers for sending whole-body messages
    wholebody_pub_ = nh_.advertise<controller_msgs::WholeBodyTrajectoryMessage>("/ihmc/valkyrie/humanoid_control/input/whole_body_trajectory", 1);
    go_home_pub_ = nh_.advertise<controller_msgs::GoHomeMessage>("/ihmc/valkyrie/humanoid_control/input/go_home", 20);
    finger_pub_ = nh_.advertise<controller_msgs::ValkyrieHandFingerTrajectoryMessage>("/ihmc/valkyrie/humanoid_control/input/valkyrie_hand_finger_trajectory", 10);

    // publishers for sending high-level abort/pause/resume/stop messages
    abort_walking_pub_ = nh_.advertise<controller_msgs::AbortWalkingMessage>("/ihmc/valkyrie/humanoid_control/input/abort_walking", 1);
    pause_walking_pub_ = nh_.advertise<controller_msgs::PauseWalkingMessage>("/ihmc/valkyrie/humanoid_control/input/pause_walking", 1);
    stop_all_traj_pub_ = nh_.advertise<controller_msgs::StopAllTrajectoryMessage>("/ihmc/valkyrie/humanoid_control/input/stop_all_trajectory", 1);

    return true;
}

// CALLBACKS
void IHMCInterfaceNode::transformCallback(const geometry_msgs::TransformStamped& tf_msg) {
    if( receive_pelvis_transform_ ) {
        // set pelvis translation based on message
        tf_pelvis_wrt_world_.setOrigin(tf::Vector3(tf_msg.transform.translation.x,
                                                   tf_msg.transform.translation.y,
                                                   tf_msg.transform.translation.z));
        // set pelvis orientation based on message
        tf::Quaternion quat_pelvis_wrt_world(tf_msg.transform.rotation.x,
                                             tf_msg.transform.rotation.y,
                                             tf_msg.transform.rotation.z,
                                             tf_msg.transform.rotation.w);
        tf_pelvis_wrt_world_.setRotation(quat_pelvis_wrt_world);

        // set flag indicating pelvis transform has been received
        received_pelvis_transform_ = true;

        // set flag to no longer receive transform messages
        if( !commands_from_controllers_ ) {
            receive_pelvis_transform_ = false;
        }
    }

    // update flag to publish commands
    updatePublishCommandsFlag();

    // update flag to stop node
    updateStopNodeFlag();

    return;
}

void IHMCInterfaceNode::controlledLinkIdsCallback(const std_msgs::Int32MultiArray& arr_msg) {
    if( receive_link_ids_ ) {
        // clear vector of controlled links
        controlled_links_.clear();

        // set controlled links
        for( int i = 0 ; i < arr_msg.data.size() ; i++ ) {
            controlled_links_.push_back(arr_msg.data[i]);
        }

        // set flag indicating link ids have been received
        received_link_ids_ = true;

        // set flag to no longer receive link ids
        if( !commands_from_controllers_ ) {
            receive_link_ids_ = false;
        }
    }

    // update flag to publish commands
    updatePublishCommandsFlag();

    // update flag to stop node
    updateStopNodeFlag();

    return;
}

void IHMCInterfaceNode::jointCommandCallback(const sensor_msgs::JointState& js_msg) {
    if( receive_joint_command_ ) {
        // resize vector for joint positions
        q_joint_.resize(valkyrie::num_act_joint);
        q_joint_.setZero();

        // set positions for each joint
        for( int i = 0 ; i < js_msg.position.size(); i++ ) {
            // joint state message may contain joints we don't care about, especially when coming from IHMC
            // messages coming from ControllerManager will not have this problem, but it's good to be safe
            // check if joint is one of Valkyrie's action joints
            std::map<std::string, int>::iterator it;
            it = val::joint_names_to_indices.find(js_msg.name[i]);
            if( it != val::joint_names_to_indices.end() ) {
                // joint state message may publish joints in an order not expected by configuration vector
                // set index for joint based on joint name; add offset to ignoring virtual joints
                int jidx = val::joint_names_to_indices[js_msg.name[i]] - valkyrie::num_virtual;
                q_joint_[jidx] = js_msg.position[i];
            }
            // if joint name is not one of Valkyrie's action joints, ignore it
        }

        // set flag indicating joint command has been received
        received_joint_command_ = true;

        // set flag to no longer receive joint command messages
        if( !commands_from_controllers_ ) {
            receive_joint_command_ = false;
        }
    }

    // update flag to publish commands
    updatePublishCommandsFlag();

    // update flag to stop node
    updateStopNodeFlag();

    return;
}

void IHMCInterfaceNode::statusCallback(const std_msgs::String& status_msg) {
    if( status_msg.data == std::string("STOP-LISTENING") ) {
        // set status
        status_ = status_msg.data;

        // controllers have converged, do not receive any more messages
        receive_pelvis_transform_ = false;
        received_pelvis_transform_ = false;
        receive_link_ids_ = false;
        received_link_ids_ = false;
        receive_joint_command_ = false;
        received_joint_command_ = false;

        // update flag to publish commands
        updatePublishCommandsFlag();

        // update flag to stop node
        updateStopNodeFlag();

        ROS_INFO("[IHMC Interface Node] Controllers stopped, no longer publishing whole-body messages");
        ROS_INFO("[IHMC Interface Node] Waiting for status change to receive more joint commands...");
        // stream of messages can be ended with message with velocity of 0
        // all messages sent with velocity 0, so ending on any message is fine
    }
    else if( status_msg.data == std::string("START-LISTENING") ) {
        // set status
        status_ = status_msg.data;

        // controllers are started, prepare to receive messages
        receive_pelvis_transform_ = true;
        received_pelvis_transform_ = false;
        receive_link_ids_ = true;
        received_link_ids_ = false;
        receive_joint_command_ = true;
        received_joint_command_ = false;

        // update flag to publish commands
        updatePublishCommandsFlag();

        // update flag to stop node
        updateStopNodeFlag();

        ROS_INFO("[IHMC Interface Node] Controllers started, waiting for joint commands...");
    }
    else if( status_msg.data == std::string("HOME-LEFTARM") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        home_left_arm_ = true;

        // update flag to publish go home message
        updatePublishGoHomeCommandFlag();

        ROS_INFO("[IHMC Interface Node] Homing left arm...");
    }
    else if( status_msg.data == std::string("HOME-RIGHTARM") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        home_right_arm_ = true;

        // update flag to publish go home message
        updatePublishGoHomeCommandFlag();

        ROS_INFO("[IHMC Interface Node] Homing right arm...");
    }
    else if( status_msg.data == std::string("HOME-CHEST") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        home_chest_ = true;

        // update flag to publish go home message
        updatePublishGoHomeCommandFlag();

        ROS_INFO("[IHMC Interface Node] Homing chest...");
    }
    else if( status_msg.data == std::string("HOME-PELVIS") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        home_pelvis_ = true;

        // update flag to publish go home message
        updatePublishGoHomeCommandFlag();

        ROS_INFO("[IHMC Interface Node] Homing pelvis...");
    }
    else if( status_msg.data == std::string("OPEN-LEFT-HAND") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        open_left_hand_ = true;

        // update flag to publish finger message
        updatePublishFingerCommandFlag();

        ROS_INFO("[IHMC Interface Node] Opening left hand...");
    }
    else if( status_msg.data == std::string("CLOSE-LEFT-HAND") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        close_left_hand_ = true;

        // update flag to publish finger message
        updatePublishFingerCommandFlag();

        ROS_INFO("[IHMC Interface Node] Closing left hand...");
    }
    else if( status_msg.data == std::string("OPEN-RIGHT-HAND") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        open_right_hand_ = true;

        // update flag to publish finger message
        updatePublishFingerCommandFlag();

        ROS_INFO("[IHMC Interface Node] Opening right hand...");
    }
    else if( status_msg.data == std::string("CLOSE-RIGHT-HAND") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        close_right_hand_ = true;

        // update flag to publish finger message
        updatePublishFingerCommandFlag();

        ROS_INFO("[IHMC Interface Node] Closing right hand...");
    }
    else if( status_msg.data == std::string("ABORT-WALKING") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        publish_abort_walking_command_ = true;

        ROS_INFO("[IHMC Interface Node] Aborting walking...");
    }
    else if( status_msg.data == std::string("PAUSE-WALKING") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        publish_pause_walking_command_ = true;

        ROS_INFO("[IHMC Interface Node] Pausing walking...");
    }
    else if( status_msg.data == std::string("RESUME-WALKING") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        publish_resume_walking_command_ = true;

        ROS_INFO("[IHMC Interface Node] Resuming walking...");
    }
    else if( status_msg.data == std::string("STOP-ALL-TRAJECTORY") ) {
        // set status
        status_ = status_msg.data;

        // set flag
        publish_stop_all_traj_command_ = true;

        ROS_INFO("[IHMC Interface Node] Stopping all trajectories...");
    }
    else {
        ROS_WARN("[IHMC Interface Node] Unrecognized status %s, ignoring status message", status_msg.data.c_str());
    }
    return;
}

void IHMCInterfaceNode::handPoseCommandCallback(const geometry_msgs::TransformStamped& tf_msg) {
    if( cartesian_hand_goals_ ) {
        // check for left hand goal
        if( tf_msg.child_frame_id.find(std::string("left")) != std::string::npos ) {
            // store left target
            left_hand_target_ = tf_msg;
            // set flag indicating hand goal has been received
            received_left_hand_goal_ = true;
        }
        // check for right hand goal
        else if( tf_msg.child_frame_id.find(std::string("right")) != std::string::npos ) {
            // store right target
            right_hand_target_ = tf_msg;
            // set flag indicating hand goal has been received
            received_right_hand_goal_ = true;
        }
        else {
            ROS_WARN("[IHMC Interface Node] Unrecognized child frame id %s, ignoring hand pose command message", tf_msg.child_frame_id.c_str());
            return;
        }
    }

    // update flag to publish hand commands
    updatePublishHandCommandFlag();

    return;
}

void IHMCInterfaceNode::receiveCartesianGoalsCallback(const std_msgs::Bool& bool_msg) {
    // update Cartesian goals flag based on message
    cartesian_hand_goals_ = bool_msg.data;

    // status of Cartesian goals has changed; reset targets
    geometry_msgs::TransformStamped empty_tf_msg;
    left_hand_target_ = empty_tf_msg;
    right_hand_target_ = empty_tf_msg;

    // update flags
    if( cartesian_hand_goals_ ) {
        // prepare to receive Cartesian hand goals
        received_left_hand_goal_ = false;
        received_right_hand_goal_ = false;

        // update flag to publish hand commands
        updatePublishHandCommandFlag();

        ROS_INFO("[IHMC Interface Node] Accepting Cartesian hand goals");
    }
    else {
        // not receiving Cartesian hand goals
        received_left_hand_goal_ = false;
        received_right_hand_goal_ = false;

        // update flag to publish hand commands
        updatePublishHandCommandFlag();

        // update flag to stop node
        updateStopNodeFlag();

        ROS_INFO("[IHMC Interface Node] Not accepting Cartesian hand goals");
    }

    return;
}

void IHMCInterfaceNode::plannedMoveItRobotTrajectoryCallback(const moveit_msgs::RobotTrajectory& moveit_msg) {
    if( receive_moveit_traj_ ) {
        // store MoveIt trajectory
        moveit_robot_traj_ = moveit_msg;
        // set flag indicating trajectory has been received
        received_moveit_traj_ = true;
    }

    // update flag to publish trajectory
    updatePublishMoveItTrajectoryFlag();

    return;
}

void IHMCInterfaceNode::receiveMoveItTrajCallback(const std_msgs::Bool& bool_msg) {
    // update receive MoveIt trajectories flag based on message
    receive_moveit_traj_ = bool_msg.data;

    // status of MoveIt trajectories has changed; reset targets
    moveit_msgs::RobotTrajectory empty_robot_traj_msg;
    moveit_robot_traj_ = empty_robot_traj_msg;

    // update flags
    if( receive_moveit_traj_ ) {
        // prepare to receive MoveIt trajectory
        received_moveit_traj_ = false;

        // update flag to publish MoveIt trajectory
        updatePublishMoveItTrajectoryFlag();

        ROS_INFO("[IHMC Interface Node] Accepting MoveIt trajectories");
    }
    else {
        // not receiving MoveIt trajectories
        received_moveit_traj_ = false;

        // update flag to publish MoveIt trajectory
        updatePublishMoveItTrajectoryFlag();

        // update flag to stop node (not necessary, but just to be sure)
        updateStopNodeFlag();

        ROS_INFO("[IHMC Interface Node] Not accepting MoveIt trajectories");
    }

    return;
}

void IHMCInterfaceNode::trajectoryPointTimeCallback(const std_msgs::Float32& float_msg) {
    // store time internally
    if( trajectory_point_time_ != float_msg.data ) {
        trajectory_point_time_ = float_msg.data;
        ROS_INFO("[IHMC Interface Node] Set trajectory point time for Cartesian hand goals to %f seconds", trajectory_point_time_);
    }

    return;
}

// PUBLISH MESSAGE
void IHMCInterfaceNode::publishWholeBodyMessage() {
    // prepare configuration vector based on received pelvis transform and joint command
    prepareConfigurationVector();

    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;
    // set controlled links
    msg_params.controlled_links = controlled_links_;

    // if commands are coming from controllers, default message parameters will need to be changed
    if( commands_from_controllers_ ) {
        // set execution mode to streaming (0 override; 1 queue; 2 stream)
        msg_params.queueable_params.execution_mode = 2;
        // set stream integration duration (equal or slightly longer than interval between two consecutive messages, which should be coming in at 10 Hz or 0.1 secs)
        msg_params.queueable_params.stream_integration_duration = 0.13; // TODO?
        // set time to achieve trajectory point messages (1.0 for queueing, 0.0 for streaming)
        msg_params.traj_point_params.time = 0.0;
    }

    // create whole-body message
    controller_msgs::WholeBodyTrajectoryMessage wholebody_msg;
    IHMCMsgUtils::makeIHMCWholeBodyTrajectoryMessage(q_, wholebody_msg, msg_params);

    // publish message
    wholebody_pub_.publish(wholebody_msg);

    return;
}

void IHMCInterfaceNode::publishWholeBodyMessageCartesianHandGoals() {
    // initialize left and right hand goals, frame id, and controlled links
    dynacore::Vect3 left_pos;
    dynacore::Quaternion left_quat;
    dynacore::Vect3 right_pos;
    dynacore::Quaternion right_quat;
    std::string cartesian_frame_id;
    std::vector<int> controlled_links;

    // prepare left and right goals
    bool proceed = prepareCartesianHandGoals(left_pos, left_quat, right_pos, right_quat, cartesian_frame_id, controlled_links);

    if( !proceed ) {
        ROS_WARN("[IHMC Interface Node] Not publishing whole-body message");
        return;
    }

    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;
    // set controlled links
    msg_params.controlled_links = controlled_links;

    // update message parameters for Cartesian goals
    msg_params.cartesian_hand_goals = cartesian_hand_goals_;
    msg_params.frame_params.cartesian_goal_reference_frame_name = cartesian_frame_id;
    if( trajectory_point_time_ != 0.0 ) {
        // received non-default time
        msg_params.traj_point_params.time = trajectory_point_time_;
    }

    // get transform from pelvis to world
    tf::StampedTransform tf_pelvis_wrt_world;
    ROS_INFO("[IHMC Interface Node] Trying to get transform from pelvis to world...");
    // try getting transformation
    try {
        // wait for most recent transform
        tf_.waitForTransform("world", "pelvis", ros::Time(0), ros::Duration(2.0));
        // lookup transform
        tf_.lookupTransform("world", "pelvis", ros::Time(0), tf_pelvis_wrt_world);
    }
    catch (tf2::TransformException ex) {
        ROS_WARN("[IHMC Interface Node] No transform from pelvis to world.");
        return;
    }

    ROS_INFO("[IHMC Interface Node] Got transform from pelvis to world!");

    // create whole-body message
    controller_msgs::WholeBodyTrajectoryMessage wholebody_msg;
    IHMCMsgUtils::makeIHMCWholeBodyTrajectoryMessage(q_, left_pos, left_quat, right_pos, right_quat,
                                                     wholebody_msg, msg_params, tf_pelvis_wrt_world);
    // configuration vector q_ will not be used

    // publish message
    wholebody_pub_.publish(wholebody_msg);

    // reset flags since received targets have been processed
    received_left_hand_goal_ = false;
    received_right_hand_goal_ = false;

    // update flag to publish hand message
    updatePublishHandCommandFlag();

    return;
}

void IHMCInterfaceNode::publishWholeBodyMessageMoveItTrajectory() {
    // initialize controlled links
    std::vector<int> controlled_links;

    // prepare controlled links based on received MoveIt trajectory
    prepareControlledLinksFromMoveItTraj(controlled_links);

    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;
    // set controlled links
    msg_params.controlled_links = controlled_links;

    // create whole-body message
    controller_msgs::WholeBodyTrajectoryMessage wholebody_msg;
    IHMCMsgUtils::makeIHMCWholeBodyTrajectoryMessage(moveit_robot_traj_, wholebody_msg, msg_params);

    // publish message
    wholebody_pub_.publish(wholebody_msg);

    // reset flags since received targets have been processed
    received_moveit_traj_ = false;

    // update flag to publish MoveIt trajectory
    updatePublishMoveItTrajectoryFlag();

    return;
}

void IHMCInterfaceNode::publishGoHomeMessage() {
    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;

    // home left arm
    if( home_left_arm_ ) {
        // create go home message
        controller_msgs::GoHomeMessage go_home_msg;
        IHMCMsgUtils::makeIHMCHomeLeftArmMessage(go_home_msg, msg_params);

        // publish message
        go_home_pub_.publish(go_home_msg);

        // reset flag
        home_left_arm_ = false;
    }

    // home right arm
    if( home_right_arm_ ) {
        // create go home message
        controller_msgs::GoHomeMessage go_home_msg;
        IHMCMsgUtils::makeIHMCHomeRightArmMessage(go_home_msg, msg_params);

        // publish message
        go_home_pub_.publish(go_home_msg);

        // reset flag
        home_right_arm_ = false;
    }

    // home chest
    if( home_chest_ ) {
        // create go home message
        controller_msgs::GoHomeMessage go_home_msg;
        IHMCMsgUtils::makeIHMCHomeChestMessage(go_home_msg, msg_params);

        // publish message
        go_home_pub_.publish(go_home_msg);

        // reset flag
        home_chest_ = false;
    }

    // home pelvis
    if( home_pelvis_ ) {
        // create go home message
        controller_msgs::GoHomeMessage go_home_msg;
        IHMCMsgUtils::makeIHMCHomePelvisMessage(go_home_msg, msg_params);

        // publish message
        go_home_pub_.publish(go_home_msg);

        // reset flag
        home_pelvis_ = false;
    }

    // update flag to publish go home message
    updatePublishGoHomeCommandFlag();

    return;
}

void IHMCInterfaceNode::publishHandFingerMessage() {
    // open left hand
    if( open_left_hand_ ) {
        // publish message
        publishFingerOpenLeftMessage();

        // reset flag
        open_left_hand_ = false;
    }

    // close left hand
    if( close_left_hand_ ) {
        // publish message
        publishFingerCloseLeftMessage();

        // reset flag
        close_left_hand_ = false;
    }

    // open right hand
    if( open_right_hand_ ) {
        // publish message
        publishFingerOpenRightMessage();

        // reset flag
        open_right_hand_ = false;
    }

    // close right hand
    if( close_right_hand_ ) {
        // publish message
        publishFingerCloseRightMessage();

        // reset flag
        close_right_hand_ = false;
    }

    // update flag to publish hand message
    updatePublishFingerCommandFlag();

    return;
}

void IHMCInterfaceNode::publishFingerOpenLeftMessage() {
    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;
    // modify default parameters for finger messages
    msg_params.setParametersForFingerMessages();
    // set time for trajectory
    msg_params.traj_point_params.time = msg_params.finger_traj_params.open_hand_time;

    // create finger message
    controller_msgs::ValkyrieHandFingerTrajectoryMessage finger_msg;
    IHMCMsgUtils::makeIHMCValkyrieHandFingerTrajectoryMessage(finger_msg, finger_msg.ROBOT_SIDE_LEFT, true, msg_params);

    // publish message
    finger_pub_.publish(finger_msg);

    return;
}

void IHMCInterfaceNode::publishFingerCloseLeftMessage() {
    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;
    // modify default parameters for finger messages
    msg_params.setParametersForFingerMessages();
    // set time for trajectory
    msg_params.traj_point_params.time = msg_params.finger_traj_params.close_hand_time;

    // create finger message
    controller_msgs::ValkyrieHandFingerTrajectoryMessage finger_msg;
    IHMCMsgUtils::makeIHMCValkyrieHandFingerTrajectoryMessage(finger_msg, finger_msg.ROBOT_SIDE_LEFT, false, msg_params);

    // publish message
    finger_pub_.publish(finger_msg);

    return;
}

void IHMCInterfaceNode::publishFingerOpenRightMessage() {
    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;
    // modify default parameters for finger messages
    msg_params.setParametersForFingerMessages();
    // set time for trajectory
    msg_params.traj_point_params.time = msg_params.finger_traj_params.open_hand_time;

    // create finger message
    controller_msgs::ValkyrieHandFingerTrajectoryMessage finger_msg;
    IHMCMsgUtils::makeIHMCValkyrieHandFingerTrajectoryMessage(finger_msg, finger_msg.ROBOT_SIDE_RIGHT, true, msg_params);

    // publish message
    finger_pub_.publish(finger_msg);

    return;
}

void IHMCInterfaceNode::publishFingerCloseRightMessage() {
    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;
    // modify default parameters for finger messages
    msg_params.setParametersForFingerMessages();
    // set time for trajectory
    msg_params.traj_point_params.time = msg_params.finger_traj_params.close_hand_time;

    // create finger message
    controller_msgs::ValkyrieHandFingerTrajectoryMessage finger_msg;
    IHMCMsgUtils::makeIHMCValkyrieHandFingerTrajectoryMessage(finger_msg, finger_msg.ROBOT_SIDE_RIGHT, false, msg_params);

    // publish message
    finger_pub_.publish(finger_msg);

    return;
}

void IHMCInterfaceNode::publishAbortWalkingMessage() {
    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;

    // create abort walking message
    controller_msgs::AbortWalkingMessage abort_msg;
    IHMCMsgUtils::makeIHMCAbortWalkingMessage(abort_msg, msg_params);

    // publish message
    abort_walking_pub_.publish(abort_msg);

    // reset flag
    publish_abort_walking_command_ = false;

    return;
}

void IHMCInterfaceNode::publishPauseWalkingMessage() {
    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;

    // create pause walking message
    controller_msgs::PauseWalkingMessage pause_msg;
    IHMCMsgUtils::makeIHMCPauseWalkingMessage(pause_msg, msg_params);

    // publish message
    pause_walking_pub_.publish(pause_msg);

    // reset flag
    publish_pause_walking_command_ = false;

    return;
}

void IHMCInterfaceNode::publishResumeWalkingMessage() {
    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;

    // create resume walking message
    controller_msgs::PauseWalkingMessage resume_msg;
    IHMCMsgUtils::makeIHMCResumeWalkingMessage(resume_msg, msg_params);

    // publish message
    pause_walking_pub_.publish(resume_msg);

    // reset flag
    publish_resume_walking_command_ = false;

    return;
}

void IHMCInterfaceNode::publishStopAllTrajectoryMessage() {
    // initialize struct of default IHMC message parameters
    IHMCMsgUtils::IHMCMessageParameters msg_params;

    // create stop all trajectory message
    controller_msgs::StopAllTrajectoryMessage stop_all_msg;
    IHMCMsgUtils::makeIHMCStopAllTrajectoryMessage(stop_all_msg, msg_params);

    // publish message
    stop_all_traj_pub_.publish(stop_all_msg);

    // reset flag
    publish_stop_all_traj_command_ = false;

    return;
}

// HELPER FUNCTIONS
std::string IHMCInterfaceNode::getStatus() {
    return status_;
}

bool IHMCInterfaceNode::getCommandsFromControllersFlag() {
    return commands_from_controllers_;
}

bool IHMCInterfaceNode::getPublishCommandsFlag() {
    return publish_commands_;
}

void IHMCInterfaceNode::updatePublishCommandsFlag() {
    // if pelvis and joint command both received, then commands can be published
    publish_commands_ = received_pelvis_transform_ && received_link_ids_ && received_joint_command_;

    return;
}

bool IHMCInterfaceNode::getStopNodeFlag() {
    return stop_node_;
}

void IHMCInterfaceNode::updateStopNodeFlag() {
    // if both pelvis and joint commands no longer being received, then prepare to stop node
    stop_node_ = !receive_pelvis_transform_ && !receive_joint_command_;

    return;
}

bool IHMCInterfaceNode::getPublishGoHomeCommandFlag() {
    return publish_go_home_command_;
}

void IHMCInterfaceNode::updatePublishGoHomeCommandFlag() {
    // if any body parts need to be homed, then go home message(s) need to be published
    publish_go_home_command_ = home_left_arm_ || home_right_arm_ || home_chest_ || home_pelvis_;

    return;
}

bool IHMCInterfaceNode::getPublishFingerCommandFlag() {
    return publish_finger_command_;
}

void IHMCInterfaceNode::updatePublishFingerCommandFlag() {
    // if any hand message needs to be opened/closed, then finger message needs to be published
    publish_finger_command_ = open_left_hand_ || close_left_hand_ || open_right_hand_ || close_right_hand_;

    return;
}

bool IHMCInterfaceNode::getPublishHandCommandFlag() {
    return publish_hand_command_;
}

void IHMCInterfaceNode::updatePublishHandCommandFlag() {
    // if Cartesian goals are being accepted and either left or right goal received, then hand message needs to be published
    publish_hand_command_ = cartesian_hand_goals_ && (received_left_hand_goal_ || received_right_hand_goal_);

    return;
}

bool IHMCInterfaceNode::getPublishAbortWalkingCommandFlag() {
    return publish_abort_walking_command_;
}

bool IHMCInterfaceNode::getPublishPauseWalkingCommandFlag() {
    return publish_pause_walking_command_;
}

bool IHMCInterfaceNode::getPublishResumeWalkingCommandFlag() {
    return publish_resume_walking_command_;
}

bool IHMCInterfaceNode::getPublishStopAllTrajectoryCommandFlag() {
    return publish_stop_all_traj_command_;
}

bool IHMCInterfaceNode::getPublishMoveItTrajectoryFlag() {
    return publish_moveit_traj_;
}

void IHMCInterfaceNode::updatePublishMoveItTrajectoryFlag() {
    // if MoveIt trajectories are being accepted and MoveIt trajectory received, then MoveIt trajectory needs to be published
    publish_moveit_traj_ = receive_moveit_traj_ && received_moveit_traj_;

    return;
}

void IHMCInterfaceNode::prepareEmptyPose(dynacore::Vect3& pos, dynacore::Quaternion& quat) {
    // set position to zero
    pos.setZero();
    // set quaternion to identity
    quat.setIdentity();

    return;
}

void IHMCInterfaceNode::preparePoseFromTransform(dynacore::Vect3& pos, dynacore::Quaternion& quat,
                                                 geometry_msgs::TransformStamped tf_msg) {
    // set position from transform
    pos << tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z;
    // set quaternion from transform
    quat.x() = tf_msg.transform.rotation.x;
    quat.y() = tf_msg.transform.rotation.y;
    quat.z() = tf_msg.transform.rotation.z;
    quat.w() = tf_msg.transform.rotation.w;

    return;
}

bool IHMCInterfaceNode::prepareCartesianHandGoals(dynacore::Vect3& left_pos, dynacore::Quaternion& left_quat,
                                                  dynacore::Vect3& right_pos, dynacore::Quaternion& right_quat,
                                                  std::string& frame_id, std::vector<int>& controlled_links) {
    // clear controlled links vector
    controlled_links.clear();

    // check if left target received
    if( !received_left_hand_goal_ ) {
        // no target received
        prepareEmptyPose(left_pos, left_quat);
    }
    else {
        // set target from transform
        preparePoseFromTransform(left_pos, left_quat, left_hand_target_);
    }

    // check if right target received
    if( !received_right_hand_goal_ ) {
        // no target received
        prepareEmptyPose(right_pos, right_quat);
    }
    else {
        // set target from transform
        preparePoseFromTransform(right_pos, right_quat, right_hand_target_);
    }

    // set frame id
    if( !received_left_hand_goal_ && !received_right_hand_goal_ ) {
        // neither pose set
        frame_id = std::string("");
        return false;
    }
    else if( received_left_hand_goal_ && !received_right_hand_goal_ ) {
        // left pose set, right pose not
        frame_id = std::string(left_hand_target_.header.frame_id);
        controlled_links.push_back(valkyrie_link::leftPalm);
        return true;
    }
    else if( !received_left_hand_goal_ && received_right_hand_goal_ ) {
        // right pose set, left pose not
        frame_id = std::string(right_hand_target_.header.frame_id);
        controlled_links.push_back(valkyrie_link::rightPalm);
        return true;
    }
    else { // received_left_hand_goal_ && received_right_hand_goal_
        // make sure frames are the same
        if( left_hand_target_.header.frame_id.compare(right_hand_target_.header.frame_id) == 0 ) {
            // frames are the same
            frame_id = std::string(left_hand_target_.header.frame_id);
            controlled_links.push_back(valkyrie_link::leftPalm);
            controlled_links.push_back(valkyrie_link::rightPalm);
            return true;
        }
        else {
            // frames are not the same; cannot confidently set frame
            ROS_WARN("[IHMC Interface Node] Received left hand target in frame %s and right hand target in frame %s; cannot send Cartesian hand targets due to ambiguity",
                      left_hand_target_.header.frame_id.c_str(), right_hand_target_.header.frame_id.c_str());
            frame_id = std::string("");
            return false;
        }
    }
}

void IHMCInterfaceNode::prepareControlledLinksFromMoveItTraj(std::vector<int>& controlled_links) {
    // get controlled links from the stored MoveIt trajectory
    IHMCMsgUtils::getControlledLinksFromMoveItMsg(moveit_robot_traj_, controlled_links);

    return;
}

void IHMCInterfaceNode::prepareConfigurationVector() {
    // pelvis transform and joint command received, so prepare configuration vector
    // resize configuration vector
    q_.resize(valkyrie::num_q);
    q_.setZero();

    // get pelvis transform
    tf::Vector3 pelvis_origin = tf_pelvis_wrt_world_.getOrigin();
    tf::Quaternion pelvis_tfrotation = tf_pelvis_wrt_world_.getRotation();

    // set pelvis position
    q_[valkyrie_joint::virtual_X] = pelvis_origin.getX();
    q_[valkyrie_joint::virtual_Y] = pelvis_origin.getY();
    q_[valkyrie_joint::virtual_Z] = pelvis_origin.getZ();

    // convert pelvis orientation to dynacore (Eigen) quaternion
    dynacore::Quaternion pelvis_rotation;
    dynacore::convert(pelvis_tfrotation, pelvis_rotation);

    // set pelvis rotation
    q_[valkyrie_joint::virtual_Rx] = pelvis_rotation.x();
    q_[valkyrie_joint::virtual_Ry] = pelvis_rotation.y();
    q_[valkyrie_joint::virtual_Rz] = pelvis_rotation.z();
    q_[valkyrie_joint::virtual_Rw] = pelvis_rotation.w();

    // set joints
    for( int i = 0 ; i < q_joint_.size() ; i++ ) {
        // set index for joint, add offset to account for virtual joints
        int jidx = i + valkyrie::num_virtual;
        q_[jidx] = q_joint_[i];
    }

    return;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "IHMCInterfaceNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    IHMCInterfaceNode ihmc_interface_node(nh);

    if( ihmc_interface_node.getCommandsFromControllersFlag() ) {
        ROS_INFO("[IHMC Interface Node] Node started, waiting for controller status...");
    }
    else {
        ROS_INFO("[IHMC Interface Node] Node started, waiting for joint commands...");
    }

    ros::Rate rate(10);
    while( ros::ok() ) {
        // check if commands coming from controllers
        if( ihmc_interface_node.getCommandsFromControllersFlag() ) {
            // consistently publish messages until controllers converge
            if( ihmc_interface_node.getPublishCommandsFlag() ) {
                // ready to publish commands
                ROS_INFO("[IHMC Interface Node] Preparing and streaming whole-body message...");
                ihmc_interface_node.publishWholeBodyMessage();
            }

            // check if any body parts need to be homed
            if( ihmc_interface_node.getPublishGoHomeCommandFlag() ) {
                // ready to publish homing message
                ROS_INFO("[IHMC Interface Node] Publishing go home message...");
                ihmc_interface_node.publishGoHomeMessage();
            }

            // check if any hands need to be opened/closed
            if( ihmc_interface_node.getPublishFingerCommandFlag() ) {
                // ready to publish finger message
                ROS_INFO("[IHMC Interface Node] Publishing hand finger trajectory message...");
                ihmc_interface_node.publishHandFingerMessage();
            }

            // check if walking needs to be aborted
            if( ihmc_interface_node.getPublishAbortWalkingCommandFlag() ) {
                // ready to publish abort walking message
                ROS_INFO("[IHMC Interface Node] Publishing abort walking message...");
                ihmc_interface_node.publishAbortWalkingMessage();
            }

            // check if walking needs to be paused
            if( ihmc_interface_node.getPublishPauseWalkingCommandFlag() ) {
                // ready to publish pause walking message
                ROS_INFO("[IHMC Interface Node] Publishing pause walking message...");
                ihmc_interface_node.publishPauseWalkingMessage();
            }

            // check if walking needs to be resumed
            if( ihmc_interface_node.getPublishResumeWalkingCommandFlag() ) {
                // ready to publish resume walking message
                ROS_INFO("[IHMC Interface Node] Publishing resume walking message...");
                ihmc_interface_node.publishResumeWalkingMessage();
            }

            // check if trajectories need to be stopped
            if( ihmc_interface_node.getPublishStopAllTrajectoryCommandFlag() ) {
                // ready to publish stop all trajectories message
                ROS_INFO("[IHMC Interface Node] Publishing stop all trajectories message...");
                ihmc_interface_node.publishStopAllTrajectoryMessage();
            }

            // check if any hands need to be moved to target
            if( ihmc_interface_node.getPublishHandCommandFlag() ) {
                // ready to publish hand message
                ROS_INFO("[IHMC Interface Node] Publishing hand trajectory message...");
                ihmc_interface_node.publishWholeBodyMessageCartesianHandGoals();
            }

            // check if MoveIt trajectory needs to be published
            if( ihmc_interface_node.getPublishMoveItTrajectoryFlag() ) {
                // ready to publish MoveIt trajectory
                ROS_INFO("[IHMC Interface Node] Preparing and executing MoveIt trajectory as IHMC whole-body message...");
                ihmc_interface_node.publishWholeBodyMessageMoveItTrajectory();
            }
        }
        else {
            // otherwise, publish single whole-body message and exit node
            if( ihmc_interface_node.getPublishCommandsFlag() && ihmc_interface_node.getStopNodeFlag() ) {
                ROS_INFO("[IHMC Interface Node] Preparing and executing whole-body message...");
                ihmc_interface_node.publishWholeBodyMessage();
                ros::Duration(3.0).sleep();
                break; // only publish one message, then stop
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[IHMC Interface Node] Published whole-body message, all done!");

    return 0;
}
