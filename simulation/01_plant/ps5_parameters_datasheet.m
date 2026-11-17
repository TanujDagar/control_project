%% PS5 - Quanser QUBE-Servo 3 + inertia disk datasheet parameters
% MATLAB R2024a. All quantities in SI units unless stated otherwise.
% Source: uploaded Table 2.2 (QUBE-Servo 3 system parameters).
% This file deliberately DOES NOT import assumed hardware controller gains.

V_nom = 24.0;          % V, manufacturer nominal input (NOT the approved command limit)
tau_nom = 20.4e-3;    % N*m, nominal torque
omega_nom_rpm = 5400;  % rpm
I_nom = 0.5;           % A
Rm = 7.5;              % ohm
Kt = 0.0422;           % N*m/A
Km = 0.0422;           % V/(rad/s)
Jm = 1.4e-6;           % kg*m^2
Lm = 1.15e-3;          % H
mh = 0.0106;           % kg
rh = 0.0111;           % m
Jh = 0.6e-6;           % kg*m^2, supplied hub inertia
md = 0.053;            % kg, disc mass
rd = 0.0248;           % m, disc radius
N_enc = 2048;          % quadrature counts per revolution (512 lines x 4)

% Calculated inertia, assuming a uniform SOLID disk rigidly coupled to the shaft.
Jd = 0.5*md*rd^2;
J = Jm + Jh + Jd;

% Friction NOT provided by uploaded datasheet; zero-viscous-friction
% initial ideal model only. Replace after identification if justified.
b = 0;                 % N*m*s/rad, assumed (not a manufacturer value)

% Useful quantities - not controller parameters.
encoder_rad_per_count = 2*pi/N_enc;
