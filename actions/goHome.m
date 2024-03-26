function [ret,state,status] = goHome(config)
    %----------------------------------------------------------------------
    % This functions uses the FollowJointTrajectory service to send desired
    % joint angles that move the robot arm to a home or ready
    % configuration. 
    % 
    % The angles are hard-coded. 
    % 
    % Expansion:
    % Possible to expand to different desirable configurations and have an
    % input string make the selection. 
    %
    % Inputs:
    % config (string): 'qr', 'qz' or other. 
    %                   qz = zeros(1,6)
    %                   qr = [0 0 pi/2 -pi/2 0 0]
    %
    % Outputs
    % resultMsg (struc) - Result message
    % state [char] - Final goal state
    % status [char]- Status text
    %----------------------------------------------------------------------

    % Open Fingers
    disp('Opening fingers...')
    doGrip('place');

    % Move arm according to config
    if nargin == 0
        ret = moveToQ('qr');
    else
        ret = moveToQ(config);
    end
end