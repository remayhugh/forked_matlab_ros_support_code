function [grip_result_msg,grip_result_state] = doGrip(type)
%-----
% Tell gripper to either pick or place via the ros gripper action client
%
% Input: type (string) - 'pick' or 'place'
% Output: actoin result and state

    % Initialize variables
    grip_result_msg = '';
    grip_result_state = '';

    % Create a gripper action client
    grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
    % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

    % Set Grip Pos by default to pick / close gripper
    gripPos = 0; 

    % Modify it if place (i.e. open)
    if strcmp(type,'place')
        gripPos = 0.8;           
    end

    % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    % Send action goal
    sendGoal(grip_action_client,grip_goal);
end