function [mat_joint_traj,robot_joint_names] = convertPoseTraj2JointTraj(robot,mat_traj)
    %-------------------------------------------------------------
    % convertPoseTraj2JointTraj
    % Compute IKs for Cartesian trajectory. Will need to:
    % 1. Create an array of joint angles the same size as trajectory
    % 2. Instantiate a UR5 object
    % 3. Perform adjustments for URDF Fwd Kins
    % 4. Instantiate numerical IK object for Robot
    % 5. Loop through trajectory
    % 6. Extract solution w/ checks-- first one for right/elbow/down. 
    %
    % Input:
    % - mat_traj [] - array of 4 x 4 x n waypoints of homogeneous trasformation trajectories
    %
    % Output:
    % - mat_joint_traj [] - array of n x 6 waypoints of joint angle trajectories
    % - robot_joint_names {} - array of cells of joint names
    %-------------------------------------------------------------
    
    %1. Size in terms 4x4xn
    traj_sz = size(mat_traj);
    if length(traj_sz) == 2
        num_traj_points = 1;
    else
        num_traj_points = traj_sz(1,3);
    end
    
    % 2 Create trajectory of joint angles as a row matrix (m x 6) where, we have m waypoints by 6 joint angles for the UR5e
    mat_joint_traj = zeros(num_traj_points,6);    
   
    % 3. Adjust the forward kinematics to match the URDF model in Gazebo:
    name = 'UR5e';
    robot = urdfAdjustment(robot,name);    
    
    % 4. Create the numerical IK Solver
    ik = inverseKinematics("RigidBodyTree",robot);
    
    % Set solver weights
    ikWeights = [0.25 0.25 0.25 0.1 0.1 .1];
    
    % Set initial guess to current position
    joint_state_sub = rossubscriber("/joint_states");
    ros_cur_jnt_state_msg = receive(joint_state_sub,1);
   
    % Reorder from ROS format to Matlab format, need names.
    [mat_cur_q,robot_joint_names] = ros2matlabJoints(ros_cur_jnt_state_msg);
        
    % 5. Go through trajectory loop
    % Check for time complexity. Can we improve efficiency.
    for i = 1:num_traj_points
        [des_q, ~] = ik('tool0',mat_traj(:,:,i),ikWeights,mat_cur_q); % Should we update initial guess with the latest q_traj value?

        % Print soln for debugging purposes:
        fprintf('IKs: '); fprintf('%.2f,', des_q);fprintf('\n')
        
        % 6. Set 1st row of des_q's to the 1st row of mat_joint_traj
        mat_joint_traj(i,:) = des_q(1,:);         %q_sz = size(des_q);

        % 7. Update joint angles for next round: mat_cur_q
        ros_cur_jnt_state_msg = receive(joint_state_sub,1);
        [mat_cur_q,~] = ros2matlabJoints(ros_cur_jnt_state_msg);
    end
% Debug disp(mat_joint_traj)
end

