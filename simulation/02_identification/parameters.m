%% PS5 - SIMULATION-ONLY controller parameters (DATASHEET BASELINE)
% MATLAB/Simulink R2024a. All quantities in SI units.
% This is NOT the hardware-identified parameters.m supplied by the user.
% Stage 2 separately generates parameters_sim_identified.m from simulated data.
% No clear/clc: safe to run inside other MATLAB scripts and model builders.

%% Manufacturer data: QUBE-Servo 3 with hub + inertia disk
R = 7.5;               % ohm, armature resistance
L = 1.15e-3;           % H, armature inductance
Kt = 0.0422;           % N*m/A, motor torque constant
Ke = 0.0422;           % V*s/rad, motor back-EMF constant
Jm = 1.4e-6;           % kg*m^2, motor rotor inertia
Jh = 0.6e-6;           % kg*m^2, attachment-hub inertia
md = 0.053;            % kg, inertia disk mass
rd = 0.0248;           % m, inertia disk radius
Jdisk = 0.5*md*rd^2;  % kg*m^2, uniform solid disk assumption
Jeq = Jm + Jh + Jdisk;
beq = 0;               % N*m*s/rad: ASSUMPTION, NOT in supplied datasheet
N_enc = 2048;          % counts/rev, quadrature mode

%% Full electrical-mechanical plant, and reduced first-order model
% G_full(s) = Kt / ((L*s+R)*(Jeq*s+beq) + Kt*Ke)
num_full = Kt;
den_full = [L*Jeq, (L*beq + R*Jeq), (R*beq + Kt*Ke)];
K_dc = Kt/(R*beq + Kt*Ke);  % rad/s per V: theoretical DC gain
tau = R*Jeq/(R*beq + Kt*Ke); % s: reduced speed time constant

%% SIMULATION DESIGN choices (not all are manufacturer specifications)
Ts = 0.002;            % s, matches supplied hardware controller sample period
Vmax = 10.0;           % V, datasheet-recommended amplifier output limit
Vmin = -10.0;          % V
N_avg = 4;             % samples, matches supplied controller architecture
tau_f = 1/100;         % s, speed LPF time constant (controller design choice)
alpha_f = tau_f/(tau_f+Ts);  % 0.833333..., matches supplied filter formula

%% Controller pole-placement design: reduced plant K/(tau*s+1)
% Choose zeta=0.8 and wn=40 rad/s to match the hardware design TARGETS,
% not hardware-identification results. PI closed-loop characteristic:
% tau*s^2 + (1+K_dc*Kp)*s + K_dc*Ki = 0.
zeta = 0.80;           % design choice, NOT datasheet
wn = 40;               % rad/s, design choice, NOT datasheet
Kp = (2*zeta*wn*tau - 1)/K_dc; % V/(rad/s), PI proportional gain
Ki = (wn^2*tau)/K_dc;         % V/rad, PI integral gain
Kp_only = Kp;          % fair P baseline, same choice as uploaded hardware file

%% Source-traceable checks
assert(abs(Jeq-1.829856e-5)<1e-12,'Unexpected inertia');
assert(Ts>0 && Vmax>0 && Vmin<0 && alpha_f>=0 && alpha_f<1);
assert(Kp_only>0 && Kp>0 && Ki>0,'Chosen design targets led to invalid gains');
