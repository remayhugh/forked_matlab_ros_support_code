%% Overview

% This m-file will move the end-effector such that the tip touches the top
% or rCan3.


%% Inputs

masterhostIP = '192.168.64.129';
port = 11311;
vmIP = '10.51.84.223';


%% Preliminaries

%rosinit(masterhostIP,11311,'NodeHost','10.51.84.223')
rosinit(masterhostIP,port,'NodeHost',vmIP)
% quick comment here: when you run this code, you will need to put in your
% own port, IP, and vm IP that is running your gazebo.


%% Outputs

% Controlling the Robot Arm Pose using the ROS Action Client
% trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory')
% trajGoal = rosmessage(trajAct);
% trajAct.FeedbackFcn = []; 
% jointSub = rossubscriber("/joint_states")
% jointStateMsg = jointSub.LatestMessage






