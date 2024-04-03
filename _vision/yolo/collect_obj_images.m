function myImgStruct = collect_obj_images(model_name)
%--------------------------------------------------------------------------
%myImgStruct = collect_obj_images() collects image data from gazebo objects in the simulation world. 
%
% The method expects an environment with a single object. The method
% will identify the object's position while keep the robot arm's current height. 
% Then the robot arm will cycle about the object an 'iteration' number of
% times, visiting rot_offset points around the circle, and restarting the
% cycle at lin_offset distance away from the center at the next cycle.  
%
% Parameters:
% - rot_ffset (double) - number of points to visit around the circle
% - lin_offset (double) - displacement by which arm should move w/ e/ cycle
% - iterations (doulbe) - number of loops to execute, each one move
%                         lin_offset further away.
% Inputs:
% - model (string) - name of specific object, i.e. gCan1, yBottle2, pouch3
% 
% Output:
% - myImgStruct (struct) - struct containing imgs of name img#
%
% Write Files:
% 'myImageFile.mat' - saves the img structure to file
%
% TODO: reload different objects directly via service
%-------------------------------------------------------------------------- 

   
    %% 01 Local Variables
    myImgStruct = struct(); % Keep images here
    
    % Dict
    optns = dictionary();                 % Type of global dictionary with all options to facilitate passing of options
    optns("debug")               = 0;     % If set to true visualize traj before running  
    optns("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    optns("traj_steps")          = 1;     % Num of traj steps
    optns("z_offset")            = 0.09;  % Vertical offset for top-down approach
    optns("traj_duration")       = 2;     % Traj duration (secs)   
    
    optns("frameAdjustmentFlag") = 1;
    optns("toolAdjustmentFlag")  = 1;
    optns("toolAdjustment")      = 0.165; % Distance from tool0 to gripper_tip_link
    
    optns("rot_offset")          = 12;        % 2pi / rot_offset
    optns("lin_offset")          = 0.10;      % Displacement in (m)
    optns("iterations")          = 2;         % How many loops
           
    % Modify rot_offset with random numbers to introduce variance
    rot_offset = optns("rot_offset");
    rand_valence = randi([0, 1]) * 2 - 1;               % randomly shifts between 1 and -1
    rand_addition = randi(3);                           % random offset between 1 and 3
    rand_offset = rand_valence * rand_addition;
    rot_offset = rot_offset + rand_offset; 

    % Calculate total number of operations
    total_operations = rot_offset*optns("iterations");
    
    %% Center Arm around Object
    models = getModels;                         % Extract gazebo model list
    
    % Retrieve desired index
    try
        index = find(strcmp(models.ModelNames, model_name));
    catch
        error('The model you are looking for does not exist');   
    end
    
    % Retrieve the model name
    model_name = models.ModelNames{index};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  
    
    % Extract pose of that model
    get_robot_gripper_pose_flag = 1;
    [rob, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name,get_robot_gripper_pose_flag);
    
    %% Create base pose and move arm there
    base_pose = [mat_R_T_M(1,4), ... % mat_x = -ros_y
                 mat_R_T_M(2,4), ... % mat_y =  ros_x
                 rob(3,4), ...
                 0 0 pi]; % -pi/2, -pi, 0];     % [px,py,pz, z y z]
    
    disp('Moving on top of object...');
    mat_R_T_M = set_manual_goal(base_pose);
    
    disp('Our base pose is: '); 
    disp(mat_R_T_M);
    
    % Center robot arm around object
    traj_result = moveTo(mat_R_T_M,optns);
    
    %% Create img subscriber
    rgbImgSub = rossubscriber("/camera/rgb/image_raw","sensor_msgs/Image","DataFormat","struct");
    pause(1);
    
    %% Cycle around object
    for i = 1:total_operations
       
        %% Read Image
        myImg = rosReadImage(rgbImgSub.LatestMessage); 
        fprintf('Recorded img %d of %d...\n',i,total_operations);

        % Visualize 
        if optns('debug')
            imshow(myImg)
        end
    
        % Save img before transformation to file
        imageName = sprintf('img%d', i);
        myImgStruct.(imageName) = myImg;
        
    
        %% Transform offset
        % Create offsets and insert noise for randomness
        % Linear offset
        lOff_k = ceil( i/rot_offset );             % Ie n=4: (1,2,3,4)=>1, (5,6,7,8)=>2
                        
        % Rotation offset        
        rOff_k = mod( i,optns("rot_offset") );              % Ie n=4: (1,2,3,4), (1,2,3,4),...
           
        % Compute an offset arm from base that rotates with each iteration
        d = (optns("lin_offset") + 0.05*rand )* lOff_k;     % Add to this offset some random difference of upto 5cm
        d = [d,0,0,0]';
        R = trotz(2*pi/rot_offset * rOff_k);
        T = R*d;
    
        % Ajust matlab2ros coord frames
        temp = T(2);
        T(2) = T(1);
        T(1) = -temp; 
        new_R_T_M = mat_R_T_M; % Do not modify base pose, you will use it anew in each iteration
        new_R_T_M(:,4) = new_R_T_M(:,4) + T;
        
        %% Move to new location around circle, will later take a picture
        traj_result = moveTo(new_R_T_M,optns);    
        fprintf('Cycle %d/%d...\n\n', i,total_operations);
    end
    
    %% Save img to structure
    % Name images as img00 and save them to file with random class name
    imageName   = sprintf('img%d', i+1); 
    myImg       = rosReadImage(rgbImgSub.LatestMessage);
    
    myImgStruct.(imageName) = myImg;
    
    %% Save img struct to desired file name path

    % Create file name associated with model
    outputFileName = set_inputObj_FileName(model_name(1:end-1));

    % Save in data folder
    fullPath = fullfile('data', outputFileName); % Creates a full file path   
    if ~exist('data', 'dir')                     % If the folder does not exist, create it 
        mkdir('data');
    end

    % Save struct to file
    save(fullPath, 'myImgStruct')
    fprintf('File saved as %s\n', outputFileName);

end