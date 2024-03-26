% Pick and place

% 01 Connect to ROS (use your own masterhost IP address)
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
type = 'manual'; % gazebo, ptcloud, cam, manual
disp('Getting goal...')
% Via Gazebo
if strcmp(type,'gazebo')
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{26};         % 'rCan3'...%model_name = models.ModelNames{i}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);
elseif strcmp(type,'manual')
    % Manually
    goal = [0.8, -0.04, 0.15, -pi/2, -pi 0];     %[px,py,pz, z y z]
    mat_R_T_M = set_manual_goal(goal);
else
    % Manually
    goal = [0.8, -0.04, 0.10, -pi/2, -pi 0];     %[px,py,pz, z y z]
    mat_R_T_M = set_manual_goal(goal);
end

%% 04 Pick Model
ret = pick(mat_R_T_M);
%ret = pick(mat_R_T_M,mat_R_T_G);

%% 05 Logic to set bin according to object
if ~ret
    disp('Attempting place...')
    % placePose = selectBin(model_name); 
    fprintf('Moving to bin...');
    ret = place(placePose);
end
