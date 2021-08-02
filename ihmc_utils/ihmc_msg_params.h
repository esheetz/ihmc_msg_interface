/**
 * Structs for Storing Parameters for IHMC Messages
 * Emily Sheetz, NSTGRO VTE 2021
 **/

#include <vector>

namespace IHMCMsgUtils {

    // STRUCT FOR ARM TRAJECTORY MESSAGE PARAMETERS
    struct IHMCArmTrajectoryParams {
        // flag to set safety check, which may restrict upper-body motion while robot is walking
        bool force_execution;

        // DEFAULT CONSTRUCTOR; sets all parameters to default values
        IHMCArmTrajectoryParams() {
            force_execution = false;
        }
    };

    // STRUCT FOR FRAME INFORMATION PARAMETERS
    struct IHMCFrameParams {
        // id of reference frame for world frame (default)
        int trajectory_reference_frame_id_world;

        // id of world reference frame for data in a packet; default same as trajectory_reference_frame_id
        int data_reference_frame_id_world;

        // id of reference frame for pelvis zup frame (used for chest orientation)
        int trajectory_reference_frame_id_pelviszup;

        // id of pelvis zup reference frame for data in a packet; default same as trajectory_reference_frame_id
        int data_reference_frame_id_pelviszup;

        // DEFAULT CONSTRUCTOR; sets all parameters to default values
        IHMCFrameParams() {
            trajectory_reference_frame_id_world = 83766130; // world frame
            data_reference_frame_id_world = 83766130; // 1 indicates same as trajectory_reference_frame_id, but we set explicitly
            trajectory_reference_frame_id_pelviszup = -101; // pelvis zup
            data_reference_frame_id_pelviszup = -101; // 1 indicates same as trajectory_reference_frame_id, but we set explicitly
        }
    };

    // STRUCT FOR ONE DOF JOINT TRAJECTORY MESSAGE PARAMETERS
    struct IHMCOneDoFJointTrajectoryParams {
        // weight used to encode priority for achieving trajectory
        double weight;

        // DEFAULT CONSTRUCTOR; sets all parameters to default values
        IHMCOneDoFJointTrajectoryParams() {
            weight = -1.0;
        }
    };

    // STRUCT FOR PELVIS TRAJECTORY MESSAGE PARAMETERS
    struct IHMCPelvisTrajectoryParams {
        // flag to set safety check, which may restrict upper-body motion while robot is walking
        bool force_execution;

        // flag to set user mode, which tries to achieve desired configuration regardless of leg kinematics
        bool enable_user_pelvis_control;

        // flag to set user mode during walking
        bool enable_user_pelvis_control_during_walking;

        // DEFAULT CONSTRUCTOR; sets all parameters to default values
        IHMCPelvisTrajectoryParams() {
            force_execution = false;
            enable_user_pelvis_control = false;
            enable_user_pelvis_control_during_walking = false;
        }
    };

    // STRUCT FOR QUEUABLE MESSAGE PARAMETERS
    struct IHMCQueueableParams {
        // execution mode for queueable messages; 0 is override, 1 is queue, 2 is stream
        int execution_mode;

        // message id for queued queueable messages; default -1, only needs to be set if another message is queued
        int message_id;

        // id of previous message for queued queueable messages; default 0?
        int previous_message_id;

        // integration duration (s) for streamed queuable messages; helps smooth out delays between messages
        double stream_integration_duration;

        // DEFAULT CONSTRUCTOR; sets all parameters to default values
        IHMCQueueableParams() {
            execution_mode = 0;
            message_id = -1;
            previous_message_id = -1;
            stream_integration_duration = 0.0;
        }
    };

    // STRUCT FOR SE3 AND SO3 MESSAGE PARAMETERS
    struct IHMCSE3SO3Params {
        // flag to set use of custom control frame
        bool use_custom_control_frame;

        // DEFAULT CONSTRUCTOR; sets all parameters to default values
        IHMCSE3SO3Params() {
            use_custom_control_frame = false;
        }
    };

    // STRUCT FOR SELECTION MATRIX 3D MESSAGE PARAMETERS
    struct IHMCSelectionMatrixParams {
        // id of reference frame for selection matrix
        int selection_frame_id;

        // flag to select x-axis of reference frame
        bool x_selected;

        // flag to select y-axis of reference frame
        bool y_selected;

        // flag to select z-axis of reference frame
        bool z_selected;

        // DEFAULT CONSTRUCTOR; sets all parameters to default values
        IHMCSelectionMatrixParams() {
            selection_frame_id = 0;
            x_selected = true;
            y_selected = true;
            z_selected = true;
        }
    };

    // STRUCT FOR WEIGHT MATRIX 3D MESSAGE PARAMETERS
    struct IHMCWeightMatrixParams {
        // id of reference frame for weight matrix
        int weight_frame_id;

        // weight for x-axis of reference frame
        double x_weight;

        // weight for y-axis of reference frame
        double y_weight;

        // weight for z-axis of reference frame
        double z_weight;

        // DEFAULT CONSTRUCTOR; sets all parameters to default values
        IHMCWeightMatrixParams() {
            weight_frame_id = 0;
            x_weight = -1.0;
            y_weight = -1.0;
            z_weight = -1.0;
        }
    };

    // STRUCT FOR TRAJECTORY POINT MESSAGE PARAMETERS
    struct IHMCTrajectoryPointParams {
        // time at which trajectory point should be reached, relative to trajectory start
        double time;

        // DEFAULT CONSTRUCTOR; sets all parameters to default values
        IHMCTrajectoryPointParams() {
            time = 5.0;
        }
    };

    // STRUCT FOR WHOLE-BODY MESSAGE PARAMETERS
    struct IHMCMessageParameters {
        // id used to identify message, should be consecutively increasing
        int sequence_id;

        // vector of controlled links
        std::vector<int> controlled_links;

        // parameters for message types involved in whole-body messages
        IHMCArmTrajectoryParams arm_params;
        IHMCFrameParams frame_params;
        IHMCOneDoFJointTrajectoryParams onedof_joint_params;
        IHMCPelvisTrajectoryParams pelvis_params;
        IHMCQueueableParams queueable_params;
        IHMCSE3SO3Params se3so3_params;
        IHMCSelectionMatrixParams selection_matrix_params;
        IHMCWeightMatrixParams weight_matrix_params;
        IHMCTrajectoryPointParams traj_point_params;

        // DEFAULT CONSTRUCTOR; sets all parameters to default values
        IHMCMessageParameters() {
            sequence_id = 1;
            // all other params already at defaults
        }
    };

} // end namespace IHMCMsgUtils
