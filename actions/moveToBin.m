function grip_result = moveToBin(strategy,mat_current, mat_bin)
%----------------------------------------------------------------------
% moveToBin 
% Top-level function to execute a complete place. 
% 
% 01 Move according to strategy
% 02 Open fingers
%
% Inputs
% strategy (string):    direct or topdown
% mat_current [4x4]:    object pose wrt to base_link
% mat_bin  [4x4]:       bin pose wrt to base_link
%
% Outputs:
% ret (bool): 0 indicates success, other failure.
%----------------------------------------------------------------------
    
    %% 1. Vars / Dictionary of options
    ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.1;   % Vertical offset for top-down approach
    ops("traj_duration")       = 2;     % Traj duration (secs)   

    grip_result                = -1;           % Init to failure number  
    
    %% 2. Move to desired location
    if strcmp(strategy,'topdown') % Lift and then go to bin       
        
        % Lift
        disp('Lifting object....')
        over_R_T_M = lift(mat_current, ops("z_offset") );
        traj_result = moveTo(over_R_T_M,ops);

        % Displace
        if ~traj_result
            disp('Moving to bin...');
            traj_result = moveTo(mat_bin,ops);
        end
        
    % Move directly
    elseif strcmpi(strategy,'direct')
        traj_result = moveTo(mat_bin,ops);
    end


    %% 3. Place if successfull (check structure of resultState). Otherwise...
    if ~traj_result
        [grip_result,grip_state] = doGrip('place'); 
        grip_result = grip_result.ErrorCode;
    end
end