function detect_cans_bottles_pouches(coord)

% Detect parts and identify their poses
% This function detects parts using a pre-trained deep learning model. Each
% part is a struct with 2 elements: centerPoint and type.

% Camera info coming from /camera/rbg/camera_info
  % FrameId :  camera_depth_link
  % Roi                
  %   XOffset   :  0
  %   YOffset   :  0
  %   Height    :  0
  %   Width     :  0
  %   DoRectify :  0
  % Height          :  480
  % Width           :  640
  % DistortionModel :  plumb_bob
  % D               :  [0, 0, 0, 0, 0]
  % K               :  [554.3827128226441, 0, 320.5, 0, 554.3827128226441, 240.5, 0, 0, 1]
  % R               :  [1, 0, 0, 0, 1, 0, 0, 0, 1]
  % P               :  [554.3827128226441, 0, 320.5, -38.80678989758509, 0, 554.3827128226441, 240.5, 0, 0, 0, 1, 0]
  % BinningX        :  0
  % BinningY        :  0

% Copyright 2020 The MathWorks, Inc.

     % Empty cell array of parts to detect new parts
     coord.Parts = {};

     % Camera properties
     hfov = 1.211269;
     imageWidth = 480;
     focalLength = (imageWidth/2)/tan(hfov/2);

     % Read image from simulated Gazebo camera
     rgbImg = readImage(coord.ROSinfo.rgb_sub.LatestMessage);
     centerPixel = [round(size(rgbImg,1)/2), round(size(rgbImg,2)/2)];

     % Detect parts and show labels
     % figure;
     imshow(rgbImg);

     [bboxes,~,labels] = detect(coord.DetectorModel,rgbImg);
     if ~isempty(labels)
        labeledImg = insertObjectAnnotation(rgbImg,'Rectangle',bboxes,cellstr(labels));
        
        imshow(labeledImg);            
        
        numObjects = size(bboxes,1);
        allLabels  = table(labels);
       
        for i=1:numObjects
            
            if allLabels.labels(i)=='can'
                % Height of objects is known according to type
                part.Z = 0.052;
                part.type = 2;
            
            else if allLabels.labels(i)=='bottle'
                part.Z = 0.17;
                part.type = 1;

            % Pouch: 
            else 
                part.Z = 0.01;
                part.type = 3;
            end
            
            % Transform from cam2model to gripper2model: G_T_M <-- C_T_M
            tftree       = rostf('DataFormat','struct');
            cameraTransf = getTransform(tftree,base,end_effector,rostime('now'),'Timeout', tf_listening_time);
            mat_R_T_C = ros2matlabPose(current_pose,frameAdjustmentFlag,toolAdjustmentFlag);

            % cameraTransf = getTransform(coord.Robot, coord.CurrentRobotJConfig, 'EndEffector_Link');
            cameraZ      = mat_R_T_C(3,4);            
            zDistance = cameraZ - part.Z;
            

            %% 
            centerBox = [bboxes(i,2)+ round(bboxes(i,4)/2), bboxes(i,1)+ round(bboxes(i,3)/2)];
            centerBoxwrtCenterPixel = centerBox - centerPixel; % in pixels
            
            worldCenterBoxwrtCenterPixel = (zDistance/focalLength)*centerBoxwrtCenterPixel; % in meters
            actualCameraTransf = cameraTransf * trvec2tform([0, 0.041, 0.0]);
            actualpartXY = actualCameraTransf(1:2,4)' + worldCenterBoxwrtCenterPixel;
            
            part.centerPoint = [actualpartXY(1),actualpartXY(2),part.Z];
            coord.Parts{i} = part;
        end
     end
    
    coord.NextPart = 0;
    
    if ~isempty(coord.Parts) && coord.NextPart<=length(coord.Parts)
        coord.DetectedParts = coord.Parts;
        % Trigger event 'partsDetected' on Stateflow
        % coord.FlowChart.partsDetected;
        return;
    end
    
    coord.NumDetectionRuns = coord.NumDetectionRuns +1;

    % Trigger event 'noPartsDetected' on Stateflow
    %coord.FlowChart.noPartsDetected; 
   
end