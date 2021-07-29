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
    nh_.param("pelvis_tf_topic", pelvis_tf_topic_,
              std::string("/ControllerTestNode/controllers/output/ihmc/pelvis_transform"));
    nh_.param("controlled_link_topic", controlled_link_topic_,
              std::string("/ControllerTestNode/controllers/output/ihmc/controlled_link_ids"));
    nh_.param("joint_command_topic", joint_command_topic_,
              std::string("/ControllerTestNode/controllers/output/ihmc/joint_commands"));
    nh_.param("status_topic", status_topic_,
              std::string("/ControllerTestNode/controllers/output/ihmc/controller_status"));

    initializeConnections();

    // initialize flags for receiving and publishing messages
    receive_pelvis_transform_ = true;
    received_pelvis_transform_ = false;
    if( commands_from_controllers_ ) {
        receive_link_ids_ = true;
        received_link_ids_ = false;
    }
    else {
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
    receive_joint_command_ = true;
    received_joint_command_ = false;
    publish_commands_ = false;
    stop_node_ = false;
    
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
    controlled_link_sub_ = nh_.subscribe(controlled_link_topic_, 1, &IHMCInterfaceNode::controlledLinkIdsCallback, this);
    joint_command_sub_ = nh_.subscribe(joint_command_topic_, 1, &IHMCInterfaceNode::jointCommandCallback, this);
    status_sub_ = nh_.subscribe(status_topic_, 1, &IHMCInterfaceNode::statusCallback, this);

    // publishers for sending whole-body messages
    wholebody_pub_ = nh_.advertise<controller_msgs::WholeBodyTrajectoryMessage>("/ihmc/valkyrie/humanoid_control/input/whole_body_trajectory", 1);

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
    // TODO additional statuses? special message to start the stream of messages?
    if( status_msg.data == std::string("STOP") ) {
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
        
        ROS_INFO("Controllers stopped, no longer publishing whole body messages");        
        ROS_INFO("Waiting for status change to receive more joint commands...");  	            
        // stream of messages can be ended with message with velocity of 0
        // all messages sent with velocity 0, so ending on any message is fine
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
        msg_params.queueable_params.execution_mode = 2; // TODO?
        // set stream integration duration (equal or slightly longer than interval between two consecutive messages, which should be coming in at 10 Hz or 0.1 secs)
        msg_params.queueable_params.stream_integration_duration = 0.13; // TODO?
        // set time to achieve trajectory point messages (1.0 for queueing, 0.0 for streaming)
        msg_params.traj_point_params.time = 0.0; // TODO?
    }

    // create whole body message
    controller_msgs::WholeBodyTrajectoryMessage wholebody_msg;
    IHMCMsgUtils::makeIHMCWholeBodyTrajectoryMessage(q_, wholebody_msg, msg_params);

    // publish message
    wholebody_pub_.publish(wholebody_msg);

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
    std::cout << "IHMC Interface Node" << std::endl;

    // initialize node
    ros::init(argc, argv, "IHMCInterfaceNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    IHMCInterfaceNode ihmc_interface_node(nh);

    ROS_INFO("[IHMC Interface Node] node started, waiting for joint commands...");

    ros::Rate rate(10);
    while( ros::ok() ) {
        // if commands coming from controllers, consistently publish messages until controllers converge
    	if( ihmc_interface_node.getCommandsFromControllersFlag() && ihmc_interface_node.getPublishCommandsFlag() ) {
    	    // ready to publish commands
    	    ROS_INFO("Preparing and streaming whole body message..."); // TODO streaming or queueing?
    	    ihmc_interface_node.publishWholeBodyMessage();
    	}
    	else {
    	    // otherwise, publish single wholebody message and exit node
            if( ihmc_interface_node.getPublishCommandsFlag() && ihmc_interface_node.getStopNodeFlag() ) {
                ROS_INFO("Preparing and executing whole body message...");
                ihmc_interface_node.publishWholeBodyMessage();
                ros::Duration(3.0).sleep();
                break; // only publish one message, then stop
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[IHMC Interface Node] published whole body message, all done!");

    return 0;
}
