function res = moveToQ(config)
% moveToQ 
% Move to joint configuration prescribed by config
% 
% Inputs
% config (string):    qr, qz... can expand in future
%
% Outputs:
% res (bool): 0 indicates success, other failure.
%----------------------------------------------------------------------
    
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);

    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
    % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 
    
    % Set to qr
    if nargin == 0 || strcmp(config,'qr')
        % Ready config
        q = [0 0 pi/2 -pi/2 0 0];
    
    % Set to qz
    elseif(strcmp(config,'qz'))
        q = zeros(1,6);
    
    % Default to qr
    else
        % Default to qr ready config
        q = [0 0 pi/2 -pi/2 0 0];
    
    end

    %% Convert to ROS waypoint
    traj_goal = convert2ROSPointVec(q,ros_cur_jnt_state_msg.Name,1,1,traj_goal);
    
    %% Send ros trajectory with traj_steps
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end
    
    % Extract result
    res = res.ErrorCode;
    
end