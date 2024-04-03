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

    %% 03 Get Images

    % Save all the names of objects in world
    obj_names = {'yCan1'};
    % obj_names = {%'rCan1',                
    %              'gCan1',...
    %              'yCan1',...
    %              'rBottle1',...
    %              'bBottle1',...
    %              'yBottle1',...
    %              'pouch1',...
    %              };
    % obj_names = {%'rCan1',
    %              'rCan2','rCan3',...
    %              'gCan1','gCan2','gCan3','gCan4',...
    %              'yCan1','yCan2','yCan3','yCan4',...                 
    %              'rBottle1','rBottle2',...
    %              'bBottle1','bBottle2','bBottle3',...
    %              'yBottle1','yBottle2','yBottle3','yBottle4',...
    %              'pouch1','pouch2','pouch3','pouch4','pouch5','pouch6','pouch7','pouch8'
    %              };

    obj_len = length(obj_names); 
    img_struct = cell(1,obj_len);

    % Save image structures in the cells
    for i=1:length(obj_names)

        % Move arm around object and save imgs as struct
        img_struct{i} = collect_obj_images(obj_names{i});
    end

    %% 04 Label Images

    %% 05 Train Detector