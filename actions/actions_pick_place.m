% Pick and place

%% 00 Connect to ROS (use your own masterhost IP address)
clc
clear
rosshutdown;
masterhostIP = "192.168.122.128";
rosinit(masterhostIP)

%% 02 Go Home
disp('Going home...');
goHome('qr');    % moves robot arm to a qr or qz start config

disp('Resetting the world...');
resetWorld;      % reset models through a gazebo service

%% 03 Get Pose
disp('Getting goal...')
type = 'manual'; % gazebo, ptcloud, cam, manual

% Via Gazebo
if strcmp(type,'gazebo')
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{26};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  

    fprintf('Picking up model: %s \n',model_name);
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);

% Manual (i.e. rCan3)
elseif strcmp(type,'manual')
    goal = [0.8, -0.04, 0.15, -pi/2, -pi 0];     %[px,py,pz, z y z]
    mat_R_T_M = set_manual_goal(goal);
else
    % Manually
    goal = [0.8, -0.04, 0.10, -pi/2, -pi 0];     %[px,py,pz, z y z]
    mat_R_T_M = set_manual_goal(goal);
end

%% 04 Pick Model
% Assign strategy: topdown, direct
strategy = 'topdown';
ret = pick(strategy, mat_R_T_M); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G);

%% 05 Place
if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
end

%% Return to home
if ~ret
    ret = moveToQ('qr');
end

