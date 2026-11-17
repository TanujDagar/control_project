%% parameters_sim_gains_for_hardware.m
% QUBE-Servo 3 inertia-disk hardware tests with SIMULATION-IDENTIFIED gains
% MATLAB R2024a-compatible; use this instead of hardware-identified gains.
% To use unmodified P/PI model builders that call run('parameters.m'),
% copy this file to their working folder as parameters.m (keep original backed up).
% No hardware data were used to choose Kp_only, Kp, or Ki.

%% 1. Manufacturer motor and inertia-disk parameters
R   = 7.50;       % Terminal resistance (Ohm)
L   = 1.15e-3;    % Rotor inductance (H)
Kt  = 0.0422;     % Torque constant (N*m/A)
Ke  = 0.0422;     % Back-EMF constant (V*s/rad)
Jm  = 1.4e-6;     % Motor rotor inertia (kg*m^2)
Jh  = 0.6e-6;     % Attachment hub inertia (kg*m^2)
md  = 0.053;      % Disk mass (kg)
rd  = 0.0248;     % Disk radius (m)
Jdisk = 0.5*md*rd^2;
Jeq = Jm + Jh + Jdisk; % 1.829856e-5 kg*m^2
beq = 0;         % Datasheet does not specify viscous damping: assumed zero

%% 2. Datasheet-derived full motor speed plant (reference only)
% G_full(s) = Kt / ((L*s + R)*(Jeq*s + beq) + Kt*Ke)
num_full = Kt;
den_full = [L*Jeq, L*beq + R*Jeq, R*beq + Kt*Ke];
K_theoretical = Kt/den_full(3);
tau_theoretical = R*Jeq/den_full(3);

%% 3. Hardware interface settings (same as original experiments)
Ts    = 0.002;    % Discrete controller sample time (s)
Vmax  = 10.0;     % Command upper limit (V)
Vmin  = -10.0;    % Command lower limit (V)
N_enc = 2048;     % Quadrature encoder counts/revolution
N_avg = 4;        % 4-sample count-difference speed estimate

%% 4. Encoder-speed low-pass filter (same as original experiments)
tau_f = 1/100;    % Filter nominal time constant (s)
alpha_f = tau_f/(tau_f + Ts);  % 0.833333333333333

%% 5. SIMULATED-IDENTIFICATION parameters from completed Stage 2
% These are NOT values identified from the physical QUBE hardware.
K_dc = 23.69817022; % (rad/s)/V
tau  = 0.07707900;   % s

%% 6. EXACT controller gains used in completed Stages 4 and 5
% Original design targets: zeta=0.80, wn=40 rad/s.
% Stored at the precision reported by the simulation model builder.
zeta = 0.80;
wn = 40;
Kp_only = 0.165964552;   % P-only controller
Kp      = 0.165964552;   % PI proportional gain
Ki      = 5.204047578;   % PI integral gain, 1/s within controller

%% 7. Confirmation (do not clear variables: model builders use this workspace)
fprintf('\n===== QUBE-SERVO 3: SIMULATION GAINS ON HARDWARE =====\n');
fprintf('Datasheet model: K = %.8f (rad/s)/V; tau = %.8f s\n', K_theoretical, tau_theoretical);
fprintf('Sim-identified:  K = %.8f (rad/s)/V; tau = %.8f s\n', K_dc, tau);
fprintf('Ts = %.4f s; N_enc = %d; N_avg = %d; alpha_f = %.8f\n', Ts, N_enc, N_avg, alpha_f);
fprintf('Limits = [%.1f, %.1f] V\n', Vmin, Vmax);
fprintf('ACTIVE GAINS: Kp_only = %.9f; Kp = %.9f; Ki = %.9f\n', Kp_only, Kp, Ki);
fprintf('======================================================\n\n');
