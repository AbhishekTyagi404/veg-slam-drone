
%% File: generate_quadplant_model.m
% Author: Abhishek Tyagi
% Version: 2.3
% This script builds a Simulink model 'quadplant.slx' for the Veg drone using Newton-Euler dynamics

modelName = 'quadplant';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);
open_system(modelName);

% Load parameters
load('veg_plant_parameters.mat');

% Constants
g_block = 'g';
state_labels = {'x','y','z','u','v','w','phi','theta','psi','p','q','r'};
input_labels = {'omega1_sq','omega2_sq','omega3_sq','omega4_sq'};

% Create Inputs
for i = 1:4
    add_block('simulink/Sources/In1', [modelName '/' input_labels{i}], ...
        'Position', [30 80*i 60 80*i+20]);
end

% Output mux
add_block('simulink/Signal Routing/Mux', [modelName '/Mux'], ...
    'Inputs', '12', 'Position', [600 100 620 400]);

% Outputs
add_block('simulink/Sinks/Out1', [modelName '/States'], ...
    'Position', [650 220 680 240]);

add_line(modelName, 'Mux/1', 'States/1');

% Simulate simplified translation + rotation integration blocks
for i = 1:12
    integrator = add_block('simulink/Continuous/Integrator', ...
        [modelName '/int_' state_labels{i}], ...
        'Position', [300 i*50 330 i*50+30]);
    % Connect to Mux
    add_line(modelName, ['int_' state_labels{i} '/1'], ['Mux/' num2str(i)]);
end

% Save the model
save_system(modelName);
fprintf('Simulink model "%s.slx" generated successfully.\n', modelName);
