function resetWorld
%--
% Calls /gazebo

res_client = rossvcclient('/gazebo/reset_world', 'std_srvs/Empty', 'DataFormat', 'struct');
ros_client_msg = rosmessage(res_client);
call(res_client, ros_client_msg);