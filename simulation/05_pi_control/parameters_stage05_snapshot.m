%% Stage 05 fallback: frozen simulation-only parameter snapshot
% These identified K/tau values were copied from USER-SUPPLIED STAGE 02
% MATLAB command-window output. They are NOT hardware measurements.
% Preferred source: ../02_identification/parameters_sim_identified.m
R=7.5; L=1.15e-3; Kt=0.0422; Ke=0.0422;
Jm=1.4e-6; Jh=0.6e-6; md=0.053; rd=0.0248;
Jdisk=0.5*md*rd^2; Jeq=Jm+Jh+Jdisk; beq=0;
Ts=0.002; Vmax=10; Vmin=-10; N_enc=2048; N_avg=4;
tau_f=1/100; alpha_f=tau_f/(tau_f+Ts);
zeta=0.8; wn=40;
K_dc=23.69817022; tau=0.07707900;
Kp=(2*zeta*wn*tau-1)/K_dc;
Ki=wn^2*tau/K_dc;
Kp_only=Kp;
num_full=Kt; den_full=[L*Jeq,L*beq+R*Jeq,R*beq+Kt*Ke];
