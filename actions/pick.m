function grip_result = pick(strategy,mat_R_T_M, mat_R_T_G)
    %----------------------------------------------------------------------
    % pick 
    % Top-level function to executed a complete pick. 
    % 
    % 01 Calls moveTo to move to desired pose
    % 02 Calls doGrip to execute a grip
    %
    % Inputs
    % mat_R_T_M [4x4]: object pose wrt to base_link
    % mat_R_T_G  [4x4]: gripper pose wrt to base_link used as starting point in ctraj (optional)    
    %
    % Outputs:
    % ret (bool): 0 indicates success, other failure.
    %----------------------------------------------------------------------
    
    %% 1. Vars / Dictionary of options
    optns = dictionary();                 % Type of global dictionary with all options to facilitate passing of options
    optns("debug")               = 0;     % If set to true visualize traj before running  
    optns("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    optns("traj_steps")          = 1;     % Num of traj steps
    optns("z_offset")            = 0.09;  % Vertical offset for top-down approach
    optns("traj_duration")       = 2;     % Traj duration (secs)   
    
    optns("frameAdjustmentFlag") = 1;
    optns("toolAdjustmentFlag")  = 1;
    optns("toolAdjustment")      = 0.165; % Distance from tool0 to gripper_tip_link

    % optns("qz")                  = {[0 0 0 0 0 0]};        % qz angles
    % optns("qr")                  = {[0 0 pi/2 -pi/2 0 0]}; % qr angles

    grip_result                = -1;           % Init to failure number  
    
    %% 2. Move to desired location
    if strcmp(strategy,'topdown')
        
        % Hover over
        over_R_T_M = lift(mat_R_T_M, optns("z_offset") );
        traj_result = moveTo(over_R_T_M,optns);
        
        % Descend
        if ~traj_result
            traj_result = moveTo(mat_R_T_M,optns);
        else
            error('Trajectory failed with result %d', int2str(traj_result));            
        end

    elseif strcmpi(strategy,'direct')
        traj_result = moveTo(mat_R_T_M,optns);
    end


    %% 3. Pick if successfull (check structure of resultState). Otherwise...
    if ~traj_result
        [grip_result,grip_state] = doGrip('pick'); 
        grip_result = grip_result.ErrorCode;
    else
        error('Grip command failed with result %d', int2str(grip_result));
    end
end