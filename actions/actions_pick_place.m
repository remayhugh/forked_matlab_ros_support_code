% Pick and place

%% 00 Connect to ROS (use your own masterhost IP address)
clc
clear
rosshutdown;

pause(2);       % Check if more down time helps diminish connection errors
masterhostIP = "your ip here";
rosinit(masterhostIP)

%% 02 Go Home
disp('Going home...');
goHome('qr');    % moves robot arm to a qr or qz start config

disp('Resetting the world...');
resetWorld;      % reset models through a gazebo service

%% Function loop
type = 'gazebo'; % gazebo, ptcloud, cam, manual
disp('Getting goal...')

% via gazebo
models = getModels;
n = 3;

% via manual
rCan1 = [0.4, -0.5, 0.14, -pi/2, -pi 0];
rCan2 = [0.018, 0.66, 0.25, -pi/2, -pi 0];
rCan3 = [0.8, -0.03, 0.15, -pi/2, -pi, 0];
model_pos = [rCan1;rCan2;rCan3];

% Loop de loop!
r = rosrate(10);
if strcmp(type,'gazebo')
    for i=1:n
        % 03 Get Pose
        model_name = models.ModelNames{23+i};
        fprintf('Picking up model: %s \n',model_name);
        [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);
       
        % 04 Pick Model
        strategy = 'topdown'; % Assign strategy: topdown, direct
        ret = pick(strategy, mat_R_T_M); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G);
        
        % 05 Place
        if ~ret
            disp('Attempting place...')
            greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
            place_pose = set_manual_goal(greenBin);
            strategy = 'topdown';
            fprintf('Moving to bin...');
            ret = moveToBin(strategy,mat_R_T_M,place_pose);
        end

        % Return to home
        if ~ret
            ret = moveToQ('qr');
        end

        % Control loop
        waitfor(r);
    end
elseif strcmp(type,'manual')
    for i=1:n
        % 03 Get Pose
        goal = model_pos(i,1:6);
        mat_R_T_M = set_manual_goal(goal);

        % 04 Pick Model
        strategy = 'topdown'; % Assign strategy: topdown, direct
        ret = pick(strategy, mat_R_T_M); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G);
        pick(strategy,mat_R_T_M,mat_R_T_G);

        % 05 Place
        if ~ret
            disp('Attempting place...')
            greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
            place_pose = set_manual_goal(greenBin);
            strategy = 'topdown';
            fprintf('Moving to bin...');
            ret = moveToBin(strategy,mat_R_T_M,place_pose);
        end

        % Return to home
        if ~ret
            ret = moveToQ('qr');
        end

        % Control loop
        waitfor(r);
    end
else
    disp("Error: unknown input (type)");
end
    
