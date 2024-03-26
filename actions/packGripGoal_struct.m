function gripGoal=packGripGoal(pos,gripGoal)
    %----------------------------------------------------------------------
    % packGripGoal
    %----------------------------------------------------------------------
    % This function will populate gripGoal with all information necessary
    % in a control_msgs/FollowJointTrajectoryGoal message.
    %
    % This functions assumes a single waypoint for the gripper, i.e. open
    % or close. 
    %
    % The function will accomplish 3 steps:
    % 1) Set Point, name and duration. 
    % 2) Set Goal Tolerance via a control_msgs/JointTolerance message type
    % 3) Set a trajectory_msgs/JointTrajectoryPoint with time/pos/vel/accel/effort
    %
    % Input Arguments:
    % pos (double): a position where 0 is open and 
    % gripGoal (control_msgs/FollowJointTrajectoryGoal)
    %
    % Output
    % A populated gripGoal message
    %----------------------------------------------------------------------
    
    % 1. Set Point, name and Duration
    jointWaypoints = [pos];                % Set position of way points
    jointWaypointTimes = 0.01;             % Time at which action is launched
    numJoints = size(jointWaypoints,1);    % Only 1 joint for gripper

    % Joint Names --> gripGoal.Trajectory
    gripGoal.Trajectory.JointNames = {'robotiq_85_left_knuckle_joint'};

    % 2. Set Goal Tolerance: set type, name, and pos/vel/acc tolerance
    % Note: tolerances are considered as: waypoint +/- tolerance
    % gripGoal.GoalTolerance = rosmessage('control_msgs/JointTolerance','DataFormat', 'struct');
    % 
    % gripGoal.GoalTolerance.Name         = gripGoal.Trajectory.JointNames{1};
    % gripGoal.GoalTolerance.Position     = 0;
    % gripGoal.GoalTolerance.Velocity     = 0.1;
    % gripGoal.GoalTolerance.Acceleration = 0.1;
    
    %3. Create a trajectory_msgs/JointTrajectoryPoint. Copy TimeFromSTart
    % and Positions in here along with vel/acc/effort. Finally copy this
    % variable into grip.Goal.Trajectory.Points
    trajPts = rosmessage('trajectory_msgs/JointTrajectoryPoint','DataFormat', 'struct');
    
    % Time Stamp
    if numJoints == 1
        trajPts.TimeFromStart   = rosduration(1, 'DataFormat','struct');
    else
        trajPts.TimeFromStart   = rosduration(jointWaypointTimes,'DataFormat', 'struct');
    end
    
    % Position
    trajPts.Positions       = jointWaypoints;

    % Vel/Acc/Effort
    trajPts.Velocities      = zeros(size(jointWaypoints));
    trajPts.Accelerations   = zeros(size(jointWaypoints));
    trajPts.Effort          = zeros(size(jointWaypoints));
    
    % Copy trajPts --> gripGoal.Trajectory.Points
    gripGoal.Trajectory.Points = trajPts;
end