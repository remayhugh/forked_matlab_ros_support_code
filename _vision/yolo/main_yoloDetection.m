%% 00 Connect to ROS (use your own masterhost IP address)
clc
clear
rosshutdown;

pause(2);       % Check if more down time helps diminish connection errors
masterhostIP = "192.168.122.128";
rosinit(masterhostIP)

%% 02 Go Home
% disp('Going home...');
% goHome('qr');    % moves robot arm to a qr or qz start config

disp('Resetting the world...');
resetWorld;      % reset models through a gazebo service

%% Create robot object
ur5e = loadrobot("universalUR5e",DataFormat="row");
[res,qr] = moveToQ('qr');

% Create robot_object coordinator
initialRobotJConfig = qr;
robotEndEffector    = ur5e.BodyNames{end};
coordinator         = robot_object(ur5e, initialRobotJConfig, robotEndEffector);

% Arrange poses for placing objects
coordinator.PlacingPose{1} = trvec2tform([[0.2 0.55 0.26]])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi]);
coordinator.PlacingPose{2} = trvec2tform([[0.2 -0.55 0.26]])*axang2tform([0 0 1 pi/2])*axang2tform([0 1 0 pi]);

%% Train Detector

%% Detect Parts Yolo Detection
detect_cans_bottles_pouches(coordinator)