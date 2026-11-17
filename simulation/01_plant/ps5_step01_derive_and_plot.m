%% PS5 - Step 01: derive, analyse and archive datasheet plant models
% MATLAB R2024a + Control System Toolbox. No QUARC or hardware required.
% Execute from any folder: the script uses its own location for I/O.
% Creates CSV, MAT, PNG, FIG files in ../results/01_plant.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(thisDir);
outDir = fullfile(projectDir,'results','01_plant');
if ~exist(outDir,'dir'), mkdir(outDir); end
run(fullfile(thisDir,'ps5_parameters_datasheet.m'));

%% Assumptions and model definition
% Electrical: Lm*di/dt + Rm*i + Km*omega = v
% Rotational: J*domega/dt + b*omega = Kt*i - tau_load
% States: x = [i;omega], input v, output omega (rad/s).
% tau_load = 0 in the voltage-to-speed transfer function.
A_full = [-Rm/Lm, -Km/Lm; Kt/J, -b/J];
B_full = [1/Lm; 0];
B_full_with_load = [1/Lm, 0; 0, -1/J]; % inputs [v; tau_load]
C_full = [0, 1];
D_full = 0;
plant_full_ss = ss(A_full,B_full,C_full,D_full);
plant_full_tf = tf(Kt, [Lm*J, Lm*b + Rm*J, Rm*b + Kt*Km]);

% Lm*di/dt ~ 0 => i ~ (v-Km*omega)/Rm.
% Gred(s) = Kt/[Rm*J*s + (Rm*b+Kt*Km)]
plant_reduced_tf = tf(Kt, [Rm*J, Rm*b + Kt*Km]);
plant_reduced_ss = ss(plant_reduced_tf);
K_dc = dcgain(plant_reduced_tf);             % rad/s per V
tau_reduced = (Rm*J)/(Rm*b+Kt*Km); % s
p_full = pole(plant_full_tf);
p_red = pole(plant_reduced_tf);
T_electrical = Lm/Rm;

% Traceable parameters for use in the report.
Symbol = {'Rm';'Lm';'Kt';'Km';'Jm';'Jh';'md';'rd';'Jd';'J';'b';'N_enc'};
Value = [Rm;Lm;Kt;Km;Jm;Jh;md;rd;Jd;J;b;N_enc];
Unit = {'ohm';'H';'N*m/A';'V/(rad/s)';'kg*m^2';'kg*m^2';'kg';'m'; ...
        'kg*m^2';'kg*m^2';'N*m*s/rad';'counts/rev'};
Origin = {'datasheet';'datasheet';'datasheet';'datasheet'; ...
          'datasheet';'datasheet';'datasheet';'datasheet'; ...
          'calculated: solid disk';'calculated: combined inertia'; ...
          'assumed: not supplied';'datasheet: quadrature'};
parameters_table = table(Symbol,Value,Unit,Origin);
writetable(parameters_table,fullfile(outDir,'01_parameters.csv'));

%% Theoretical 2 V voltage step after t = 0.1 s
% This is a simulation-only excitation, not a hardware instruction.
t = (0:0.0002:1.0)';
u = 2*double(t >= 0.1);
w_full = lsim(plant_full_tf,u,t);
w_reduced = lsim(plant_reduced_tf,u,t);

fig = figure('Color','w','Visible','off');
plot(t,w_full,'LineWidth',1.6); hold on;
plot(t,w_reduced,'--','LineWidth',1.4);
grid on; xlabel('Time (s)'); ylabel('Angular speed (rad/s)');
title('PS5 Step 01: 2 V step at 0.1 s (manufacturer plant, b = 0)');
legend('Full electrical-mechanical model','Reduced first-order model','Location','southeast');
exportgraphics(fig,fullfile(outDir,'02_full_vs_reduced_step.png'),'Resolution',180);
savefig(fig,fullfile(outDir,'02_full_vs_reduced_step.fig')); close(fig);

%% Bode frequency response comparison
w = logspace(-1,5,750)';
Gf = squeeze(freqresp(plant_full_tf,w));
Gr = squeeze(freqresp(plant_reduced_tf,w));
fig = figure('Color','w','Visible','off');
subplot(2,1,1);
semilogx(w,20*log10(abs(Gf)),'LineWidth',1.4); hold on;
semilogx(w,20*log10(abs(Gr)),'--','LineWidth',1.3);
grid on; ylabel('Magnitude (dB)');
title('PS5 Step 01: frequency response - voltage to speed');
legend('Full','Reduced','Location','southwest');
subplot(2,1,2);
semilogx(w,unwrap(angle(Gf))*180/pi,'LineWidth',1.4); hold on;
semilogx(w,unwrap(angle(Gr))*180/pi,'--','LineWidth',1.3);
grid on; xlabel('Frequency (rad/s)'); ylabel('Phase (deg)');
exportgraphics(fig,fullfile(outDir,'03_bode_full_vs_reduced.png'),'Resolution',180);
savefig(fig,fullfile(outDir,'03_bode_full_vs_reduced.fig')); close(fig);

%% Poles and input-output step characteristics
S_full = stepinfo(2*plant_full_tf); % 0->2 V, settles at 2*Kdc
S_red = stepinfo(2*plant_reduced_tf);
Model = {'Full (2-state)';'Reduced (1-state)'};
DC_Gain_radps_per_V = [dcgain(plant_full_tf); dcgain(plant_reduced_tf)];
RiseTime_s = [S_full.RiseTime; S_red.RiseTime];
SettlingTime_s = [S_full.SettlingTime; S_red.SettlingTime];
Overshoot_pct = [S_full.Overshoot; S_red.Overshoot];
SteadySpeedFor2V_radps = 2*DC_Gain_radps_per_V;
model_summary = table(Model,DC_Gain_radps_per_V,RiseTime_s,SettlingTime_s, ...
                      Overshoot_pct,SteadySpeedFor2V_radps);
writetable(model_summary,fullfile(outDir,'04_plant_model_summary.csv'));
writematrix([t,u,w_full,w_reduced],fullfile(outDir,'05_step_time_data.csv'));
writematrix([w,abs(Gf),angle(Gf)*180/pi,abs(Gr),angle(Gr)*180/pi], ...
            fullfile(outDir,'06_frequency_data.csv'));

save(fullfile(outDir,'07_plant_analysis.mat'),'Rm','Lm','Kt','Km','Jm','Jh', ...
     'md','rd','Jd','J','b','N_enc','encoder_rad_per_count', ...
     'A_full','B_full','B_full_with_load','C_full','D_full', ...
     'plant_full_ss','plant_full_tf','plant_reduced_tf','plant_reduced_ss', ...
     'K_dc','tau_reduced','T_electrical','p_full','p_red', ...
     't','u','w_full','w_reduced','w','Gf','Gr','parameters_table','model_summary');

fid = fopen(fullfile(outDir,'08_stage01_run_record.txt'),'w');
fprintf(fid,'Stage: 01 Datasheet Model\nMATLAB release: %s\nDate: %s\n', ...
        version('-release'),datestr(now,31));
fprintf(fid,'b = %.12g (ASSUMED, not given in datasheet)\n',b);
fprintf(fid,'J_disk = %.12g kg*m^2\nJ_total = %.12g kg*m^2\n',Jd,J);
fprintf(fid,'K_dc = %.12g rad/s/V\ntau = %.12g s\n',K_dc,tau_reduced);
fprintf(fid,'Full poles = %.12g, %.12g rad/s\n',p_full(1),p_full(2));
fprintf(fid,'Source: user-uploaded PS5 problem statement and QUBE-Servo 3 Table 2.2 image.\n');
fclose(fid);

fprintf('\nSTAGE 01 COMPLETE (MATLAB analytical simulation)\n');
fprintf('J_disk = %.9g kg*m^2, J_total = %.9g kg*m^2\n',Jd,J);
fprintf('K_dc = %.6f rad/s/V, reduced tau = %.6f s\n',K_dc,tau_reduced);
fprintf('Full plant poles: %.5f, %.5f rad/s\n',p_full(1),p_full(2));
fprintf('Results: %s\n',outDir);
