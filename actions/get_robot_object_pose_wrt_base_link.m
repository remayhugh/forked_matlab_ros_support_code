function [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name)
%--------------------------------------------------------------------------
% Extract robot and model poses wrt to base_link. 
% Use gazebo service to extract poses wrt to world.
% Transform coordinate frames from world_to_base link.
% Get starting pose of robot and use its orientation to pick up object
%
% Input:
% model_name (string) - name of model available in gazebo simulation
%
% Output: 
% - mat_R_T_G [4x4] double - transformation from robot base_link to tip
% - mart_R_T_M [4x4] double -  transformation from robot base_link to obj
%--------------------------------------------------------------------------

    %% Local variables
    tf_listening_time   = 10;    % Time (secs) to listen for transformation in ros

    %% 1. Get Poses from matlab wrt to World
    disp('Setting the goal...');

    % Robot's base_link and model pose wrt gazebo world origin
    W_T_R = get_model_pose('robot');
    W_T_C = get_model_pose(model_name);
    
    %% 2. Get Goal|Current Pose wrt to base link in matlab format
    mat_W_T_R = ros2matlabPose(W_T_R);
    mat_W_T_M = ros2matlabPose(W_T_C); % Frame at junction with table
    
    % Change reference frame from gazebo world to robot's base_link
    mat_R_T_M = inv(mat_W_T_R)*mat_W_T_M; 

    canHeight = 0.2; % Can height is 20cm
    mat_R_T_M(3,4) = mat_R_T_M(3,4) + canHeight; % Offset along +z to simulate knowing height of top of can.

    %% 3. Modify orientation of robot pose to be  a top-down pick (see rviz gripper_tip_link vs base_link)
    mat_R_T_M(1:3,1:3) = rpy2r(0, pi/2, -pi);
    
    %  From orgnizers
    % T=eul2tform([pi/2 -pi 0]);
    % mat_R_T_M(1:3,1:3) = T(1:3,1:3);
    
    
    
    %% 4. Current Robot Pose in Cartesian Format:
    tftree = rostf('DataFormat','struct');     
    
    % Get gripper_tip_link pose wrt tzo base via getTransform(tftree,targetframe,sourceframe), where sourceframe is the reference frame 
    try
        current_pose = getTransform(tftree,'gripper_tip_link','base', rostime('now'), 'Timeout', tf_listening_time);
    catch
        % Try again
        current_pose = getTransform(tftree,'gripper_tip_link','base', rostime('now'), 'Timeout', tf_listening_time);
    end
    
    % Convert gripper pose to matlab format
    mat_R_T_G = ros2matlabPose(current_pose);
    
    % Set pick orientation equal to starting pose (gripper down).
    % mat_R_T_M(1:3,1:3) = mat_R_T_G(1:3,1:3); 
    
    %% Hacks: set x-axis to y-axis and y-axis to x-axis and use tool0 but
    % lift goal by difference.
    %mat_R_T_M(1:2,4) = mat_R_T_M(2:-1:1,4);
    mat_R_T_M(3,4) = mat_R_T_M(3,4) + 0.1670; % manually account for tool0 to tip if we do not attach rigid tool
end