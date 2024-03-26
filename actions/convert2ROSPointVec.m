function traj_goal = convert2ROSPointVec(mat_joint_traj, robot_joint_names, traj_steps, traj_duration, traj_goal)
%--------------------------------------------------------------------------
% convert2ROSPointVec
% Converts all of the matlab joint trajectory values into a vector of ROS
% Trajectory Points. 
% 
% Make sure all messages have the same DataFormat (i.e. struct)
%
% Inputs:
% mat_joint_traj (n x q) - matrix of n trajectory points for q joint values
% robot_joint_names {} - cell of robot joint names
% traj_steps (double) - num of steps in trajectory
% traj_duration double - duration of trajectory in seconds
% traj_goal (FollowJointTrajectoryGoal)
%
% Outputs
% vector of TrajectoryPoints (1 x n)
%--------------------------------------------------------------------------
    % Compute timeStep
    timeStep = traj_duration / traj_steps;
    
    % When we set the names, remove finger entry 2:
    traj_goal.Trajectory.JointNames = robot_joint_names([1,3:7]);
    
    %% Set Tolerances for each Joint
    % % Create Tolerance Message
    % traj_goal.GoalTolerance = rosmessage('control_msgs/JointTolerance', 'DataFormat','struct');
    % 
    % for idx = 1:length(traj_goal.Trajectory.JointNames)
    % % Populate
    %     traj_goal.GoalTolerance(idx).Name = traj_goal.Trajectory.JointNames{idx};
    %     traj_goal.GoalTolerance(idx).Position = 0;
    %     traj_goal.GoalTolerance(idx).Velocity = 0;
    %     traj_goal.GoalTolerance(idx).Acceleration = 0;
    % end
    
    %% Set Points

    % Set an array of cells
    points = cell(1,traj_steps);

    % Set time with format as structure
    if traj_steps == 1
        % Create Point message
        point = rosmessage('trajectory_msgs/JointTrajectoryPoint', 'DataFormat','struct');
        
        % Extract each waypoint and set it as a 6x1 (use transpose)
	    point.Positions     = mat2rosJoints( mat_joint_traj(1, :) )'; 
        point.TimeFromStart = rosduration(1,'DataFormat','struct');

        % Set inside points cell
        points{1} = point; 
    
    else

        % Start at 2nd entry such that timeFromStart is not 0 for 1st entry. Does not work well with FollowJointTrajectory type
        for i = 2:traj_steps
    
            % Create Point message
            point = rosmessage('trajectory_msgs/JointTrajectoryPoint', 'DataFormat','struct');
    
            % Extract each waypoint and set it as a 6x1 (use transpose)
	        point.Positions     = mat2rosJoints( mat_joint_traj(i, :) )';    
    
            % Set time with format as structure
            if traj_steps == 1
                point.TimeFromStart = rosduration(1,'DataFormat','struct');
            else
	            point.TimeFromStart = rosduration( (i-1) * timeStep, 'DataFormat','struct');    
            end
            
            % Set inside points cell
            points{i} = point;   
        end
    end

    
    traj_goal.Trajectory.Points = [ points{:} ];
end