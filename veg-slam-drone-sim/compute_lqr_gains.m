%% File: compute_lqr_gains.m
% Author: Abhishek Tyagi
% Version: 2.4
% Computes the roll/pitch LQR gains used by firmware/veg_flight_controller
% (K_ROLL, K_PITCH in config.h) from veg_plant_parameters.mat.
%
% Firmware law (per axis):  u_us = K(1)*(angle_ref - angle) - K(2)*rate
%   angle in deg, rate in deg/s, u in microseconds of ESC pulse added to one
%   side of the X-quad and subtracted from the other.
%
% Model per axis:  angle_dot = rate
%                  rate_dot  = (180/pi) * b_torque * u / I
%   b_torque = torque per microsecond of differential command
%            = 4 motors * kT_us * (arm_length / sqrt(2))       (X configuration)
%
% >>> kT_us (thrust change per microsecond, N/us) MUST BE MEASURED on a thrust
%     stand around hover for your motor/prop/ESC. The value below is only a
%     placeholder estimate (hover thrust per motor over ~400 us of range).

load('veg_plant_parameters.mat');   % mass, arm_length, Ix, Iy, ...

kT_us = (mass * g / 4) / 400;       % N per us  — PLACEHOLDER, measure it!
b_torque = 4 * kT_us * arm_length / sqrt(2);

Q = diag([1, 0.02]);   % weight on angle error (deg^2) and rate (deg/s)^2
R = 0.05;              % weight on actuator effort (us^2) — keeps poles ~10 rad/s, safe for ESC lag + 50 Hz servo output

axes_names = {'ROLL', 'PITCH'};
inertia = [Ix, Iy];
for i = 1:2
    A = [0 1; 0 0];
    B = [0; (180/pi) * b_torque / inertia(i)];
    K = lqr(A, B, Q, R);
    cl = eig(A - B*K);
    fprintf('const float K_%s[2]  = { %.3ff, %.3ff };   // closed-loop poles: %s\n', ...
            axes_names{i}, K(1), K(2), mat2str(cl', 3));
end
fprintf('Paste these lines into firmware/veg_flight_controller/config.h\n');
