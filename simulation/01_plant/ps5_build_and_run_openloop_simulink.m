%% PS5 - Step 01: build and simulate the open-loop Simulink plant
% R2024a + Simulink + Control System Toolbox. No hardware required.
% Writes a real .slx when run locally in MATLAB R2024a.
% Run ps5_step01_derive_and_plot.m first (recommended).
clear; clc;
thisDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(thisDir);
outDir = fullfile(projectDir,'results','01_plant');
if ~exist(outDir,'dir'), mkdir(outDir); end
run(fullfile(thisDir,'ps5_parameters_datasheet.m'));

% The full block state x = [i; omega] (ampere, rad/s).
A = [-Rm/Lm, -Km/Lm; Kt/J, -b/J];
B = [1/Lm; 0];
C = [0,1];
D = 0;
red_num = Kt;
red_den = [Rm*J, Rm*b + Kt*Km];
mdl = 'ps5_stage01_openloop_2024a';
modelFile = fullfile(thisDir,[mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist(modelFile,'file'), delete(modelFile); end
new_system(mdl);
set_param(mdl,'Solver','ode15s','StopTime','1.0','MaxStep','0.001', ...
    'ReturnWorkspaceOutputs','on','SaveOutput','off');

add_block('simulink/Sources/Step',[mdl '/Voltage_Step'], ...
    'Time','0.1','Before','0','After','2', 'Position',[35,110,65,140]);
add_block('simulink/Continuous/State-Space',[mdl '/Full_Order_Plant'], ...
    'A',mat2str(A,16),'B',mat2str(B,16), ...
    'C',mat2str(C,16),'D',mat2str(D,16), ...
    'Position',[160,62,260,118]);
add_block('simulink/Continuous/Transfer Fcn',[mdl '/Reduced_Order_Plant'], ...
    'Numerator',mat2str(red_num,16),'Denominator',mat2str(red_den,16), ...
    'Position',[160,162,260,215]);
add_block('simulink/Signal Routing/Mux',[mdl '/Compare_Speeds'], ...
    'Inputs','2','Position',[310,90,315,190]);
add_block('simulink/Sinks/Scope',[mdl '/Speed_Scope'], ...
    'Position',[380,122,415,157]);

% To Workspace is used ONLY for MATLAB Normal simulation, not QUARC/External.
add_block('simulink/Sinks/To Workspace',[mdl '/Log_Input'], ...
    'VariableName','ps5_u_sim','SaveFormat','Timeseries', ...
    'Position',[90,20,185,49]);
add_block('simulink/Sinks/To Workspace',[mdl '/Log_Full'], ...
    'VariableName','ps5_w_full_sim','SaveFormat','Timeseries', ...
    'Position',[310,35,425,65]);
add_block('simulink/Sinks/To Workspace',[mdl '/Log_Reduced'], ...
    'VariableName','ps5_w_reduced_sim','SaveFormat','Timeseries', ...
    'Position',[310,215,425,245]);

add_line(mdl,'Voltage_Step/1','Full_Order_Plant/1','autorouting','on');
add_line(mdl,'Voltage_Step/1','Reduced_Order_Plant/1','autorouting','on');
add_line(mdl,'Voltage_Step/1','Log_Input/1','autorouting','on');
add_line(mdl,'Full_Order_Plant/1','Compare_Speeds/1','autorouting','on');
add_line(mdl,'Reduced_Order_Plant/1','Compare_Speeds/2','autorouting','on');
add_line(mdl,'Compare_Speeds/1','Speed_Scope/1','autorouting','on');
add_line(mdl,'Full_Order_Plant/1','Log_Full/1','autorouting','on');
add_line(mdl,'Reduced_Order_Plant/1','Log_Reduced/1','autorouting','on');

save_system(mdl,modelFile);
simOut = sim(mdl);
input_log = simOut.get('ps5_u_sim');
full_log = simOut.get('ps5_w_full_sim');
reduced_log = simOut.get('ps5_w_reduced_sim');
save(fullfile(outDir,'09_openloop_simulink_logs.mat'), ...
    'input_log','full_log','reduced_log','simOut','A','B','C','D', ...
    'red_num','red_den');

fig=figure('Color','w','Visible','off');
plot(full_log.Time,full_log.Data,'LineWidth',1.6); hold on;
plot(reduced_log.Time,reduced_log.Data,'--','LineWidth',1.4);
grid on; xlabel('Time (s)'); ylabel('Angular speed (rad/s)');
title('Simulink R2024a: full and reduced open-loop motor-disk models');
legend('Full order','Reduced order','Location','southeast');
exportgraphics(fig,fullfile(outDir,'10_openloop_simulink_plot.png'),'Resolution',180);
savefig(fig,fullfile(outDir,'10_openloop_simulink_plot.fig')); close(fig);
fprintf('Generated Simulink model: %s\n',modelFile);
fprintf('Saved Simulink logs/plots to: %s\n',outDir);
open_system(mdl);
