
%% File: generate_attitude_controller.m
% Author: Abhishek Tyagi
% Version: 2.3
% Builds a Simulink model 'attitude_controller.slx' for Veg with LQR + PD cascade

modelName = 'attitude_controller';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

% Inputs: 6 (phi, theta, psi, p, q, r) + reference angles (phi_des, theta_des, psi_des)
inputs = {'phi','theta','psi','p','q','r','phi_des','theta_des','psi_des'};
for i = 1:length(inputs)
    add_block('simulink/Sources/In1', [modelName '/' inputs{i}], ...
        'Position', [30 50*i 60 50*i+20]);
end

% LQR gain block
add_block('simulink/Math Operations/Gain', [modelName '/LQR_Gain'], ...
    'Gain', 'K_lqr', ...
    'Position', [150 100 200 150]);

% Mux to combine current states
add_block('simulink/Signal Routing/Mux', [modelName '/StateMux'], ...
    'Inputs', '6', 'Position', [90 100 110 250]);
for i = 1:6
    add_line(modelName, [inputs{i} '/1'], ['StateMux/' num2str(i)]);
end

% Mux to combine ref states
add_block('simulink/Signal Routing/Mux', [modelName '/RefMux'], ...
    'Inputs', '3', 'Position', [90 300 110 380]);
for i = 1:3
    add_line(modelName, [inputs{6+i} '/1'], ['RefMux/' num2str(i)]);
end

% Demux LQR output torques
add_block('simulink/Signal Routing/Demux', [modelName '/Demux'], ...
    'Outputs', '3', 'Position', [230 120 250 200]);

% Outputs: tau_phi, tau_theta, tau_psi
add_block('simulink/Sinks/Out1', [modelName '/tau_phi'], 'Position', [300 120 330 140]);
add_block('simulink/Sinks/Out1', [modelName '/tau_theta'], 'Position', [300 150 330 170]);
add_block('simulink/Sinks/Out1', [modelName '/tau_psi'], 'Position', [300 180 330 200]);

add_line(modelName, 'StateMux/1', 'LQR_Gain/1');
add_line(modelName, 'LQR_Gain/1', 'Demux/1');
add_line(modelName, 'Demux/1', 'tau_phi/1');
add_line(modelName, 'Demux/2', 'tau_theta/1');
add_line(modelName, 'Demux/3', 'tau_psi/1');

% Save the model
save_system(modelName);
fprintf('Simulink model "%s.slx" generated successfully.\n', modelName);
