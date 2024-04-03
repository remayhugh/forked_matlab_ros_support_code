function [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name,get_robot_gripper_pose_flag)
%--------------------------------------------------------------------------
% Extract robot and model poses wrt to base_link. 
% Use gazebo service to extract poses wrt to world.
% Transform coordinate frames from world_to_base link.
% Get starting pose of robot and use its orientation to pick up object
%
% Input:
% model_name (string) - name of model available in gazebo simulation
% get_robot_gripper_pose_flag (double) - 1|0 to indicate if we want to compute robot gripper (uses tf tform)
%
% Output: 
% - mat_R_T_G [4x4] double - transformation from robot base_link to tip
% - mart_R_T_M [4x4] double -  transformation from robot base_link to obj
%--------------------------------------------------------------------------

    %% Local variables
    tf_listening_time   = 10;    % Time (secs) to listen for transformation in ros
    frameAdjustmentFlag = 1;     % Indicates matlab's base_link does not match ros. Need adjustment.
    toolAdjustmentFlag  = 1;     % Indicates we have fingers but have not adjusted IKs for it.    

    % 
    if nargin == 1
        get_robot_gripper_pose_flag = 0;
    end

    
    %% 1. Get Poses from matlab wrt to World
    disp('Setting the goal...');

    % Robot's base_link and model pose wrt gazebo world origin
    W_T_R = get_model_pose('robot');
    W_T_M = get_model_pose(model_name);
    
    %% 2. Get Goal|Current Pose wrt to **MATLAB** base link in matlab format
    mat_W_T_R = ros2matlabPose(W_T_R, frameAdjustmentFlag, 0);
    mat_W_T_M = ros2matlabPose(W_T_M, frameAdjustmentFlag, toolAdjustmentFlag); % Frame at junction with table
    
    % Change reference frame from world to robot's base_link
    mat_R_T_M = inv(mat_W_T_R)*mat_W_T_M; 

    z_offset = 0.02; %0.052; % Can height is 5.2cm
    mat_R_T_M(3,4) = mat_R_T_M(3,4) + z_offset; % Offset along +z_base_link to simulate knowing height of top of can.

    %% 3. Modify orientation of robot pose to be a top-down pick (see tool0 vs base_link) w fingers aligned with matlab's y_world -axis
    fing_along_y = eul2tform([-pi/2 -pi 0]); % ZYX axis
    %fing_along_x = eul2tform([0 0 pi]); 

    mat_R_T_M(1:3,1:3) = fing_along_y(1:3,1:3);
            
    %% 4. Current Robot Pose in Cartesian Format
    mat_R_T_G = eye(4,4);

    if get_robot_gripper_pose_flag

        tftree       = rostf('DataFormat','struct');     
        base         = 'base_link';
        end_effector = 'tool0'; % When finger is properly modeled use 'gripper_tip_link'
    
        % Get end-effector pose wrt to base via getTransform(tftree,targetframe,sourceframe), where sourceframe is the reference frame 
        try
            current_pose = getTransform(tftree,end_effector,base,rostime('now'),'Timeout', tf_listening_time);
        catch
            % Try again
            current_pose = getTransform(tftree,end_effector,base,rostime('now'),'Timeout', tf_listening_time);
        end
    
        % Convert gripper pose to matlab format
        mat_R_T_G = ros2matlabPose(current_pose,frameAdjustmentFlag,toolAdjustmentFlag);    
    end
end