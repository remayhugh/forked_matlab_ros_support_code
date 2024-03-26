function [resultMsg,state,status] = goHome(config)
    %----------------------------------------------------------------------
    % This functions uses the FollowJointTrajectory service to send desired
    % joint angles that move the robot arm to a home or ready
    % configuration. 
    % 
    % The angles are hard-coded. 
    % 
    % Expansion:
    % Possible to expand to different desirable configurations and have an
    % input string make the selection. 
    %
    % Inputs:
    % config (string): 'qr', 'qz' or other. 
    %                   qz = zeros(1,6)
    %                   qr = [0 0 pi/2 -pi/2 0 0]
    %
    % Outputs
    % resultMsg (struc) - Result message
    % state [char] - Final goal state
    % status [char]- Status text
    %----------------------------------------------------------------------
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);

    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
    % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 
    
    % Convert to trajectory_msgs/FollowJointTrajectory
    if nargin == 0 || strcmp(config,'qr')
        % Ready config
        q = [0 0 pi/2 -pi/2 0 0];
    
    elseif(strcmp(config,'qz'))
        % Zero config
        q = zeros(1,6);
    
    else
        % Default to qr ready config
        q = [0 0 pi/2 -pi/2 0 0];
    
    end

    traj_goal = convert2ROSPointVec(q,ros_cur_jnt_state_msg.Name,1,1,traj_goal);
    
    % Finally send ros trajectory with traj_steps
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [resultMsg,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        resultMsg = -1; state = 'failed'; status = 'could not find server';
    end
end