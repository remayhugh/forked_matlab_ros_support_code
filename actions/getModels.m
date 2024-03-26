function models = getModels
%-------------------------------------------------------------------------- 
% getModels
% This method will create an action client that talks to Gazebo's
% get_world_properties to get all models.
%
% Inputs
% None
%
% Output
% models (gazebo_msgs/GetWorldPropertiesResponse): has cell of ModelNames
%--------------------------------------------------------------------------


% 01 Create get_model_state action client
get_models_client = rossvcclient('/gazebo/get_world_properties',...
                                 'DataFormat','struct');

% 02 Create model_client_msg 
get_models_client_msg = rosmessage(get_models_client);

% 03 Call client 
try
    [models, status,statustext] = call(get_models_client,get_models_client_msg);
catch
    disp('Error - models not listed')
    models = rosmessage('gazebo_msgs/GetWorldPropertiesResponse', 'DataFormat','struct');
end