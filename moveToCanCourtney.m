% This m-file will move the end-effector such that the tip touches the top of rCan3.
function [x,y,z,r,p,h] = moveToCan

masterhostIP = "192.168.64.129";
rosshutdown;
rosinit(masterhostIP)

disp('Going home...');
goHome('qr');    % moves robot arm to a qr or qz start config

disp('Resetting the world...');
resetWorld      % reset models through a gazebo service

models = getModels; 

model_name = models.ModelNames{26}; % 'rCan3'

[gripper_wrt_base_pose, object_wrt_base_pose] = get_robot_object_pose_wrt_base_link(model_name);
% - mat_R_T_G [4x4] double - transformation from robot base_link to tip
% - mart_R_T_M [4x4] double -  transformation from robot base_link to obj

robot = loadrobot("universalUR5e",DataFormat="row"); 

% compute matlab waypoints via ctraj
traj_steps = 2;
mat_traj = ctraj(gripper_wrt_base_pose, object_wrt_base_pose, traj_steps); % Currently unstable due to first ik transformation of joints. Just do one point.

[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(robot, mat_traj, 0);

rCan3_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
traj_goal = rosmessage(rCan3_traj_act_client); 
traj_steps          = 2;     % Num of traj steps
traj_duration       = 2;     % Traj duration (secs)

traj_goal = convert2ROSPointVec(mat_joint_traj,rob_joint_names,traj_steps,traj_duration,traj_goal);

pause(5);
sendGoal(rCan3_traj_act_client,traj_goal); 

%[x,y,z,r,p,y] = traj_goal.Trajectory.Points.Positions(:);
x = traj_goal.Trajectory.Points.Positions(1);
y = traj_goal.Trajectory.Points.Positions(2);
z = traj_goal.Trajectory.Points.Positions(3);
r = traj_goal.Trajectory.Points.Positions(4);
p = traj_goal.Trajectory.Points.Positions(5);
h = traj_goal.Trajectory.Points.Positions(6);

end