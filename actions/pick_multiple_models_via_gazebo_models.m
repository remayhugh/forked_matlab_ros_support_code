% Pick and place

%% 00 Connect to ROS (use your own masterhost IP address)
clc
clear
rosshutdown;

pause(2);       % Check if more down time helps diminish connection errors
masterhostIP = "192.168.122.128";
rosinit(masterhostIP)

%% 02 Go Home
disp('Going home...');
goHome('qr');    % moves robot arm to a qr or qz start config

disp('Resetting the world...');
resetWorld;      % reset models through a gazebo service

%% 03 Get Pose from Gazebo Models
disp('Getting goal...') 
model_names = {'rCan3','yCan1','yCan3','gCan4','rCan2','gCan3','gCan1','rCan1','gCan2'};
mod_sz = length(model_names);

% Loop through them.
r = rosrate(10);
for i=1:mod_sz
    nm = model_names{i};
    fprintf('Picking up model: %s \n',nm);
    [~,mat_R_T_M] = get_robot_object_pose_wrt_base_link(nm,0);


    %% 04 Pick Model
    % Assign strategy: topdown, direct
    strategy = 'topdown';
    ret = pick(strategy, mat_R_T_M); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G);
    
    %% 05 Place
    if ~ret
        disp('Attempting place...')
        
        % Set bin goals
        greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0]; % left
        blueBin  = [-0.4,  0.45, 0.25, -pi/2, -pi 0]; % right
        
        % Set place pose to blue bin only if it is a bottle
        if contains(nm,'Bottle')
            goal_pose = blueBin; 
        else
            goal_pose = greenBin;
        end

        place_pose = set_manual_goal(goal_pose);

        strategy = 'topdown';
        fprintf('Moving to bin...');
        ret = moveToBin(strategy,mat_R_T_M,place_pose);
    end
    
    %% Return to home
    if ~ret
        ret = moveToQ('qr');
    end

    % Control loop
    waitfor(r);
end

    