%% Overview

% This m-file will move the end-effector such that the tip touches the top
% or rCan3.


%% Inputs

masterhostIP = 'add your own machine IP'


%% Preliminaries

rosshurtdown;
rosinit(masterhostIP)
% quick comment here: when you run this code, you will need to put in your
% own port, IP, and vm IP that is running your gazebo.


%% Outputs

Controlling the Robot Arm Pose using the ROS Action Client
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory');
trajGoal = rosmessage(trajAct);
trajAct.FeedbackFcn = []; 
jointSub = rossubscriber("/joint_states");
%jointStateMsg = jointSub.LatestMessage;

%initalize robot
UR5e = loadrobot('universalUR5e', DataFormat="row");

tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e); % Create Inverse kinematics solver
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1]; % configuration weights for IK solver [Translation Orientation] see documentation
jointStateMsg = receive(jointSub,3); % receive current robot configuration
initialIKGuess = homeConfiguration(UR5e);
jointStateMsg.Name
initialIKGuess(1) = jointStateMsg.Position(4); % update configuration in initial guess
initialIKGuess(2) = jointStateMsg.Position(3);
initialIKGuess(3) = jointStateMsg.Position(1);
initialIKGuess(4) = jointStateMsg.Position(5);
initialIKGuess(5) = jointStateMsg.Position(6);
initialIKGuess(6) = jointStateMsg.Position(7);
show(UR5e,initialIKGuess)

gripper1 = -0.54;
gripper2 = 0.15;
gripper3 = 0.22;

gripperTranslation = [gripper1 gripper2 gripper3];
gripperRotation = [-pi/2 -pi 0]; %  [Z Y Z] radians
tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform
[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);
%show(UR5e,configSoln)

UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)];
trajGoal = packTrajGoal(UR5econfig,trajGoal);
sendGoal(trajAct,trajGoal)



