
%% Inputs

masterhostIP = '192.168.64.129';

%% Outputs

rosshutdown;
rosinit(masterhostIP)

trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory')
trajGoal = rosmessage(trajAct)
trajAct.FeedbackFcn = []; 
jointSub = rossubscriber("/joint_states")
%jointStateMsg = jointSub.LatestMessage

UR5e = loadrobot('universalUR5e', DataFormat="row")

% Adjust forward kinematics to match Gazebo:
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

% Create Inverse kinematics solver
ik = inverseKinematics("RigidBodyTree",UR5e); 

% configuration weights for IK solver 
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1]; 

% receive current robot configuration
jointStateMsg = receive(jointSub,3) 

% sets default configuration
initialIKGuess = homeConfiguration(UR5e)

% set configuration of each joint
% first figure out order
jointStateMsg.Name

initialIKGuess(1) = jointStateMsg.Position(4);
initialIKGuess(2) = jointStateMsg.Position(3);
initialIKGuess(3) = jointStateMsg.Position(1);
initialIKGuess(4) = jointStateMsg.Position(5);
initialIKGuess(5) = jointStateMsg.Position(6);
initialIKGuess(6) = jointStateMsg.Position(7);
show(UR5e,initialIKGuess) % shows what that should look like

% set final gripper position
ik_init = UR5e.getTransform(initialIKGuess, "tool0")
gripperX = 0.8;
gripperY = -0.032295;
gripperZ = ik_init(3,4);

gripperTranslation = [gripperY gripperX gripperZ];
gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

% set tform as transform of final position & orientation
tform = eul2tform(gripperRotation) % sets rotation
tform(1:3,4) = gripperTranslation % sets translation

% determine joint angles using inverse kinematics
[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess)

% show to see what robot should look like
show(UR5e,configSoln)

UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)]

trajGoal = packTrajGoal(UR5econfig,trajGoal);

sendGoal(trajAct,trajGoal)

% Now move top down
jointStateMsg.Name

initialIKGuess(1) = jointStateMsg.Position(4);
initialIKGuess(2) = jointStateMsg.Position(3);
initialIKGuess(3) = jointStateMsg.Position(1);
initialIKGuess(4) = jointStateMsg.Position(5);
initialIKGuess(5) = jointStateMsg.Position(6);
initialIKGuess(6) = jointStateMsg.Position(7);
show(UR5e,initialIKGuess) % shows what that should look like

% set final gripper position
gripperX2 = 0.8;
gripperY2 = -0.032295;
gripperZ2 = 0.15;

gripperTranslation = [gripperY2 gripperX2 gripperZ2];
gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians

% set tform as transform of final position & orientation
tform = eul2tform(gripperRotation) % sets rotation
tform(1:3,4) = gripperTranslation % sets translation

% determine joint angles using inverse kinematics
[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess)

%send to gazebo
UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)]

trajGoal = packTrajGoal(UR5econfig,trajGoal)

sendGoal(trajAct,trajGoal)

