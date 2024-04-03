function [mat_cur_q,robot_joint_names] = get_current_joint_states
%--------------------------------------------------------------------------
% Returns the current joint angles of the UR5e robot in matlab form: 
% [pan,shoulder, elbow, w1,w2,w3]
%
% Input:
% N/A
%
% Output:
% mat_cur_q [1x6] - the six joint angles of the UR5e arm proper
% robot_joint_names {1,7} - the names of the UR5e joint angles from ROS
% joint_states topic.
%--------------------------------------------------------------------------

    % Create subscriber
    joint_state_sub = rossubscriber("/joint_states", 'DataFormat','struct');

    % Receive message in ROS format
    try
        ros_cur_jnt_state_msg = receive(joint_state_sub,2);
    catch
        ros_cur_jnt_state_msg = receive(joint_state_sub,2);
    end
   
    % Reorder from ROS format to Matlab format, need names.
    [mat_cur_q,robot_joint_names] = ros2matlabJoints(ros_cur_jnt_state_msg);
end