%% PS5 - Controller parity inputs: TEMPLATE (NOT yet used by Stage 01)
% This file is a checklist, NOT a runnable control configuration.
% The uploaded hardware builders call run('parameters.m'), which is MISSING.
% Never silently assume these values match hardware.
% Later fill the numeric fields from the EXACT run's hardware parameters.m
% and use one locked parameter set for that simulation-vs-hardware trial.
Ts = NaN;          % seconds: use measured hardware sample time
Vmax = NaN;        % V: actual approved controller command upper bound
Vmin = NaN;        % V: actual approved controller command lower bound
Kp_only = NaN;     % V/(rad/s), hardware P gain
Kp = NaN;          % V/(rad/s), hardware PI proportional gain
Ki = NaN;          % V/rad, hardware PI integral gain
alpha_f = NaN;     % one-pole filter coefficient, 0<=alpha_f<1
N_avg = 4;         % hard-coded in the two uploaded controller scripts
N_enc = 2048;      % QUBE-Servo 3 quadrature counts/rev in supplied table
speed_ref_radps = 50;
step_time_s = 1;
disturbance_V = 0.5;
disturbance_time_s = 5;
stop_time_s = 10;
