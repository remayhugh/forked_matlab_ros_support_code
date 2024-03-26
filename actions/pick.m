function ret = pick(mat_R_T_G, mat_R_T_M)
    %----------------------------------------------------------------------
    % pick 
    % Top-level function to executed a complete pick. 
    % 
    % 01 
    % 02 
    %
    % Inputs
    % mat_R_T_G  [4x4]: gripper pose wrt to base_link
    % mat_R_T_M [4x4]: object pose wrt to base_link
    %
    % Outputs:
    % ret (bool): indicates whether pick succeeded or not. 
    %----------------------------------------------------------------------
    
    %% 1. Local variables
    debug               = 1;     % If set to true visualize traj before running  
    traj_steps          = 2;     % Num of traj steps
    traj_duration       = 2;     % Traj duration (secs)
    
    ur5e = loadrobot("universalUR5e",DataFormat="row");   

    %% 2. Call ctraj.
    disp('Computing matlab waypoints via ctraj...');
    mat_traj = ctraj(mat_R_T_G,mat_R_T_M,traj_steps); % Currently unstable due to first ik transformation of joints. Just do one point.
    %mat_traj = mat_R_T_M;
    
    %% 3. Convert to joint angles via IKs
    disp('Converting waypoints to joint angles...');
    [mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj);

    %% Visualize trajectory
    if debug
        r = rateControl(1);
        for i = 1:size(mat_joint_traj,1)
            ur5e.show(mat_joint_traj(i,:),FastUpdate=true,PreservePlot=false);
            r.waitfor;
        end
    end
    
    %% 4. Create action client, message, populate ROS trajectory goal and send
    % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
    % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 
    
    % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');
    traj_goal = convert2ROSPointVec(mat_joint_traj,rob_joint_names,traj_steps,traj_duration,traj_goal);
    
    % Finally send ros trajectory with traj_steps
    disp('Calling arm client...')
    sendGoal(pick_traj_act_client,traj_goal); 
    ret = 0;

    % If you want to cancel the goal, run this command
    %cancelGoal(pick_traj_act_client);
    
    %% 5. Pick if successfull (check structure of resultState). Otherwise...
    % if traj_result_state
    %[grip_result_msg,grip_result_state] = 
    doGrip('pick'); %%TODO
    % end
end