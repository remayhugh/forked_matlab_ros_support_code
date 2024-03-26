% Pick and place

% 01 Connect to ROS (use your own masterhost IP address)
masterhostIP = "192.168.122.128";
rosshutdown;
rosinit(masterhostIP)

%% 02 Go Home
disp('Going home...');
goHome('qr');    % moves robot arm to a qr or qz start config

disp('Resetting the world...');
resetWorld      % reset models through a gazebo service

%% 03 Get Pose

% Extract a list of models via a ga
models = getModels; 

% Get Gazebo model pose (note wrt to a world not in the base of the
% arm)
model_name = models.ModelNames{26};         % 'rCan3'...%model_name = models.ModelNames{i}  
[mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);

%% 04 Pick Model
ret = pick(mat_R_T_G, mat_R_T_M);

%% 05 Logic to set bin according to object
% placePose = selectBin(model_name); 
% ret = place(placePose);
