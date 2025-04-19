% lqr_gain_calc.m
% LQR gain calculation for quadcopter roll dynamics

% Moment of inertia about X-axis (Ix)
Ix = 0.0023;  % in kg.m^2

% State-space model: x = [phi; phi_dot]
A = [0 1; 
     0 0];
B = [0; 
     1/Ix];

% Weighting matrices
Q = diag([5, 0.1]);  % Penalizes phi and phi_dot
R = 0.01;            % Penalizes control effort

% Compute optimal gain
K = lqr(A, B, Q, R);

% Display the result
disp('Computed LQR Gain K:');
disp(K);
