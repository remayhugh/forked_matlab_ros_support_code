classdef robot_object < handle
%--------------------------------------------------------------------------
% This class used to encapsulate setup ROS communication with Gazebo
% This class is used to control the pick-and-place workflow and:
% 1. It holds information about ROS-MATLAB communication (subscribers,
%    publishers etc.)
% 2. It stores all data about the current pick and place job
%
% Copyright 2020-2023 The MathWorks, Inc.
% Mofified by Juan Rojas, March 2024
%--------------------------------------------------------------------------

    properties         
        %FlowChart
        Robot
        World = {};
        
        % ROS Structures
        ROSinfo

        % Manipulator Information 
        base
        RobotEndEffector
        CurrentRobotJConfig
        CurrentRobotTaskConfig
        NextPart = 0;
        HomeRobotTaskConfig 
        PlacingPose
        GraspPose
        NumJoints
        NumDetectionRuns = 0;
                
        % Vision
        ScanPoses = {}; 
        MergedPointCloud
        PointCloudSegments = {};
        PathHandle
        WorldPatchHandles = {}; 

        % Yolo
        Parts = {};
        DetectedParts = {};
        DetectorModel
    end
    
    methods
        function obj = robot_object(robot, initialRobotJConfig, robotEndEffector)
            obj.Robot = robot;
            
            % Initialize ROS utilities
            obj.ROSinfo.joints_sub           = rossubscriber('/joint_states','DataFormat','struct');                    
            
            obj.ROSinfo.rgb_sub             = rossubscriber('/camera/rgb/image_raw'); % Do not use struct. readImage does not expect it
            obj.ROSinfo.pt_cloud_sub        = rossubscriber('/camera/depth/image_raw');
            
            % Services
            obj.ROSinfo.set_model_svc        = rossvcclient('/gazebo/set_model_state','DataFormat','struct');
            obj.ROSinfo.get_world_svc        = rossvcclient('/gazebo/get_world_properties','DataFormat','struct');
            obj.ROSinfo.get_model_state_svc  = rossvcclient('/gazebo/get_model_state', 'DataFormat','struct');
            
            % Actions
            obj.ROSinfo.gripClient          = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                                              'control_msgs/FollowJointTrajectory',...
                                                              'DataFormat','struct');
    
            obj.ROSinfo.trajClient          = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                                              'control_msgs/FollowJointTrajectory', ...
                                                              'DataFormat', 'struct');
    
            obj.ROSinfo.gazeboJointNames = {'elbow_joint','robotiq_85_left_knuckle_joint','robotiq_85_left_knuckle_joint','shoulder_pan_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}; % joint names of robot model in GAZEBO            

            % Initialize robot configuration in GAZEBO
            disp('Going home...');
            [~,configResp]=goHome('qr');    % moves robot arm to a qr or qz start config
            
            disp('Resetting the world...');
            resetWorld;      % reset models through a gazebo service

            % Unpause GAZEBO physics - 1 Off
            physicsClient = rossvcclient('gazebo/unpause_physics');
            physicsResp = call(physicsClient,'Timeout',3);
            
            % Update robot properties
            obj.base                    = obj.Robot.Base;
            obj.CurrentRobotJConfig     = get_current_joint_states;
            obj.RobotEndEffector        = robotEndEffector;
            obj.CurrentRobotTaskConfig  = getTransform(obj.Robot, obj.CurrentRobotJConfig, obj.RobotEndEffector);
            obj.NumJoints               = numel(obj.CurrentRobotJConfig);
            
            % Specify scanning poses for Build World task
            obj.ScanPoses ={trvec2tform([0.5,  0.25, 0.45])*axang2tform([0 0 1 -pi/2])*axang2tform([0 1 0 pi]),...
                            trvec2tform([0.5, -0.25, 0.45])*axang2tform([0 0 1 -pi/2])*axang2tform([0 1 0 pi]),...
                            trvec2tform([0.4,  0.30, 0.45])*axang2tform([0 0 1  pi/2])*axang2tform([0 1 0 pi]),...
                            trvec2tform([0.4, -0.10, 0.45])*axang2tform([0 0 1  pi/2])*axang2tform([0 1 0 pi]),...
                            trvec2tform([0.4, -0.30, 0.45])*axang2tform([0 0 1  pi/2])*axang2tform([0 1 0 pi])};

            % Load deep learning model for object detection
            temp = load('exampleYolo2DetectorROSGazebo.mat');
            obj.DetectorModel = temp.detectorYolo2; 

            % Initialize visualization
            figure("Visible","on"); 
            show(obj.Robot, obj.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
            hold on
            axis([-0.1 1 -1 1 -0.1 1.2]);
            view(gca,[90.97 50.76]);
            set(gca,...
                'CameraViewAngle',4.25,...
                'DataAspectRatio',[1 1 1],...
                'PlotBoxAspectRatio',[1 1.81 1.18],...
                'Projection','perspective'); 
        end
          


        function JConfig = getCurrentRobotJConfig(obj)
            jMsg = receive(obj.ROSinfo.joints_sub);
            JConfig =  jMsg.Position(1,3:7)';
            % TODO: do we need to do ros2matlabJoints??
            % JConfig = ros2matlabJoints(JConfig);
        end
        
        function configResp = setCurrentRobotJConfig(obj, JConfig)            
            configReq = rosmessage(obj.ROSinfo.set_model_svc);
            configReq.ModelName         = "UR5e";
            configReq.UrdfParamName     = "/UR5e/robot_description";
            configReq.JointNames        = obj.ROSinfo.gazeboJointNames;
            configReq.JointPositions    = JConfig; 
            configResp = call(obj.ROSinfo.configClient, configReq, 'Timeout', 3);
        end
        
        function isMoving = getMovementStatus(obj)
            statusMsg = receive(obj.ROSinfo.controllerStateSub);
            velocities = statusMsg.Actual.Velocities;
            if all(velocities<0.1)
                isMoving = 0;
            else
                isMoving = 1;
            end
        end
        
        % Delete function
        function delete(obj)
            delete(obj.FlowChart)
        end
        
        function visualizePath(obj, positions)
            show(obj.Robot, obj.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
            poses = zeros(size(positions,2),3);
            for i=1:size(positions,2)               
                poseNow = getTransform(obj.Robot, positions(:,i)', obj.RobotEndEffector);
                poses(i,:) = [poseNow(1,4), poseNow(2,4), poseNow(3,4)];
            end
            obj.PathHandle = plot3(poses(:,1), poses(:,2), poses(:,3),'r-','LineWidth',5);            
            drawnow;
        end
            
    end
  
end

