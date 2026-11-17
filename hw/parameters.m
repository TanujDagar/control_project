%% parameters.m
%  QUBE-Servo 3 System Parameters & Controller Gains

clear; clc;

%% 1. Manufacturer & Physical Parameters (Theoretical)
R   = 7.50;         % Armature Resistance (Ohms)
L   = 1.15e-3;      % Armature Inductance (H)
Kt  = 0.0422;       % Motor Torque Constant (N.m/A)
Ke  = 0.0422;       % Back-EMF Constant (V.s/rad)
Jeq = 1.83e-5;      % Equivalent Inertia (kg.m^2)
beq = 1.50e-5;      % Equivalent Viscous Damping (N.m.s/rad)

%% 2. Theoretical Full 2nd-Order Transfer Function (for Simulation Plant)
% G_full(s) = Kt / [ (L*s + R)*(Jeq*s + beq) + Kt*Ke ]
num_full = Kt;
den_full = [L*Jeq,  (L*beq + R*Jeq),  (R*beq + Kt*Ke)];

%% 3. Hardware / System Limits
Ts    = 0.002;      % Sample Time for real-time control (s)
Vmax  = 10.0;       % Maximum safe voltage command (V)
Vmin  = -10.0;      % Minimum safe voltage command (V)
N_enc = 2048;       % Encoder resolution (counts per revolution in 4X mode)

%% 4. Low-Pass Filter for Speed Measurement
% First-order filter cutoff at 100 rad/s (~16 Hz) to smooth derivative noise
tau_f   = 1 / 100;
alpha_f = tau_f / (tau_f + Ts);

%% =========================================================
%  5. HARDWARE-IDENTIFIED PARAMETERS (Averaged from tests)
% =========================================================
K_dc = 23.7468;     % Identified DC Speed Gain ((rad/s) / V)
tau  = 0.0984;      % Identified Time Constant (s)

%% =========================================================
%  6. CALCULATED CONTROLLER GAINS (zeta = 0.80, wn = 40)
% =========================================================
Kp_only = 0.2231;   % Proportional-only control gain
Kp      = 0.2231;   % PI Proportional gain
Ki      = 6.6306;   % PI Integral gain

%% ===== Print Summary to Command Window =====
fprintf('\n===== QUBE-Servo 3 Parameters Loaded =====\n');
fprintf('Theoretical DC Gain K  = %.4f (rad/s)/V\n', num_full / den_full(3));
fprintf('Theoretical Time Const = %.4f s\n', (R*Jeq) / den_full(3));
fprintf('------------------------------------------\n');
fprintf('HARDWARE Identified K  = %.4f (rad/s)/V\n', K_dc);
fprintf('HARDWARE Identified tau= %.4f s\n', tau);
fprintf('------------------------------------------\n');
fprintf('Control Limits: +/- %.1f V\n', Vmax);
fprintf('Active Gains: Kp_only = %.4f | Kp = %.4f | Ki = %.4f\n', Kp_only, Kp, Ki);
fprintf('==========================================\n\n');