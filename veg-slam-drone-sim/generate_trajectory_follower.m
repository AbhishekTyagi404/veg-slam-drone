
%% File: generate_trajectory_follower.m
% Author: Abhishek Tyagi
% Version: 2.3
% Builds a Simulink model 'trajectory_follower.slx' for Veg using PD-based trajectory controller

modelName = 'trajectory_follower';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

% Inputs: desired [x_d, y_d, z_d], actual [x, y, z, psi, u, v]
inputs = {'x_d','y_d','z_d','x','y','z','psi','u','v'};
for i = 1:length(inputs)
    add_block('simulink/Sources/In1', [modelName '/' inputs{i}], ...
        'Position', [30 40*i 60 40*i+20]);
end

% Compute errors in x and y in body frame
add_block('simulink/Math Operations/Sum', [modelName '/Ex'], ...
    'Inputs', '+-', 'Position', [100 80 120 100]);
add_block('simulink/Math Operations/Sum', [modelName '/Ey'], ...
    'Inputs', '+-', 'Position', [100 120 120 140]);

add_line(modelName, 'x_d/1', 'Ex/1');
add_line(modelName, 'x/1', 'Ex/2');
add_line(modelName, 'y_d/1', 'Ey/1');
add_line(modelName, 'y/1', 'Ey/2');

% Convert error to body frame
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelName '/BodyFrameTransform'], 'Position', [150 90 220 130]);
set_param([modelName '/BodyFrameTransform'], 'MATLABFcn', ...
['function [ud, vd] = f(ex, ey, psi)
' ...
 'ud =  cos(psi)*ex + sin(psi)*ey;
' ...
 'vd = -sin(psi)*ex + cos(psi)*ey;
' ...
 'end']);

add_line(modelName, 'Ex/1', 'BodyFrameTransform/1');
add_line(modelName, 'Ey/1', 'BodyFrameTransform/2');
add_line(modelName, 'psi/1', 'BodyFrameTransform/3');

% Compute attitude commands
add_block('simulink/Math Operations/Gain', [modelName '/theta_cmd'], ...
    'Gain', 'Kp_theta', 'Position', [260 90 300 110]);
add_block('simulink/Math Operations/Gain', [modelName '/phi_cmd'], ...
    'Gain', '-Kp_phi', 'Position', [260 130 300 150]);

add_line(modelName, 'BodyFrameTransform/1', 'theta_cmd/1'); % ud
add_line(modelName, 'BodyFrameTransform/2', 'phi_cmd/1');   % vd

% Outputs
add_block('simulink/Sinks/Out1', [modelName '/theta_des'], 'Position', [340 90 370 110]);
add_block('simulink/Sinks/Out1', [modelName '/phi_des'], 'Position', [340 130 370 150]);

add_line(modelName, 'theta_cmd/1', 'theta_des/1');
add_line(modelName, 'phi_cmd/1', 'phi_des/1');

save_system(modelName);
fprintf('Simulink model "%s.slx" generated successfully.\n', modelName);
