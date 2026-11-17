%% EE302 PS5 - Stage 05: Hardware-matched discrete PI speed-controller model
% MATLAB/Simulink R2024a, normal simulation, no QUARC required.
% EXACT controller order from user hardware PI script:
% reference -> error -> PID block (PI, discrete Forward Euler, internal +/-10V
% clamping anti-windup) -> (+voltage disturbance) -> FINAL +/-10V clamp -> plant
% 4-sample encoder differencing, 2048 counts/rev, 1st order digital filter.
% Original hardware PI_cmd_raw logs AFTER disturbance, BEFORE final clamp.
% Original hardware PI_cmd logs AFTER final clamp (applied plant voltage).
% Motor/angle replacement is full manufacturer 2nd-order state space, ZOH.
clear; clc; close all;
thisDir=fileparts(mfilename('fullpath')); projectDir=fileparts(thisDir);
parfile=fullfile(projectDir,'02_identification','parameters_sim_identified.m');
if exist(parfile,'file')
    run(parfile); parameterSource=parfile;
else
    run(fullfile(thisDir,'parameters_stage05_snapshot.m'));
    parameterSource='Stage-02 user-verified simulation-only parameter snapshot';
    warning('Stage-02 generated parameters missing: using frozen simulated-identification snapshot.');
end
% Original hardware scripts use Ts=.002, quad encoder 2048, 4 samples, +/-10V.
N_avg=4;
assert(abs(Ts-0.002)<1e-12 && N_enc==2048 && N_avg==4,'Unexpected sampling/encoder configuration.');
assert(Vmax==10 && Vmin==-10,'Unexpected voltage limits.');
assert(abs(Kp-0.165965)<0.001 && abs(Ki-5.204048)<0.01, ...
    'Unexpected PI gains; verify simulation identification file and units.');
assert(abs(alpha_f-5/6)<1e-10,'Unexpected speed-filter coefficient.');
A=[-R/L,-Ke/L,0;Kt/Jeq,-beq/Jeq,0;0,1,0];
B=[1/L;0;0]; C=[0,1,0;0,0,1]; D=[0;0];
Pz=c2d(ss(A,B,C,D),Ts,'zoh');
mdl='ps5_stage05_pi_hardware_matched_R2024a';
modelFile=fullfile(thisDir,[mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist(modelFile,'file'), delete(modelFile); end
new_system(mdl); open_system(mdl);
set_param(mdl,'Solver','FixedStepDiscrete','FixedStep',num2str(Ts,17), ...
    'StopTime','10','ReturnWorkspaceOutputs','on');
% Same input source timing and amplitudes as original hardware PI script.
add_block('simulink/Sources/Step',[mdl '/Speed_Ref'], ...
    'Time','1','Before','0','After','50','SampleTime',num2str(Ts,17), ...
    'Position',[40 125 70 155]);
add_block('simulink/Sources/Step',[mdl '/Disturbance'], ...
    'Time','5','Before','0','After','0.5','SampleTime',num2str(Ts,17), ...
    'Position',[515 245 545 275]);
add_block('simulink/Math Operations/Sum',[mdl '/Error_Sum'], ...
    'Inputs','+-','Position',[160 125 185 155]);
% IMPORTANT: preserve the actual PID Controller block, not a custom PI.
% This is also the block/library used by the supplied hardware builder.
add_block('simulink/Continuous/PID Controller',[mdl '/Discrete_PI'], ...
    'Position',[260 115 350 165]);
set_param([mdl '/Discrete_PI'], ...
    'Controller','PI','Form','Parallel', ...
    'TimeDomain','Discrete-time','IntegratorMethod','Forward Euler', ...
    'SampleTime',num2str(Ts,17), ...
    'P',num2str(Kp,17),'I',num2str(Ki,17), ...
    'LimitOutput','on', ...
    'UpperSaturationLimit',num2str(Vmax,17), ...
    'LowerSaturationLimit',num2str(Vmin,17), ...
    'AntiWindupMode','clamping');
add_block('simulink/Math Operations/Sum',[mdl '/Dist_Sum'], ...
    'Inputs','++','Position',[595 125 620 155]);
% DIFFERENT from P: disturbance BEFORE final voltage saturation.
add_block('simulink/Discontinuities/Saturation',[mdl '/Voltage_Sat'], ...
    'UpperLimit',num2str(Vmax,17),'LowerLimit',num2str(Vmin,17), ...
    'Position',[675 125 725 155]);
% Model used in Stage 4: ZOH exact full manufacturer electrical+mechanical
% dynamics with position angle for simulated hardware encoder output.
add_block('simulink/Discrete/Discrete State-Space',[mdl '/Motor_Disk_ZOH'], ...
    'A',mat2str(Pz.A,17),'B',mat2str(Pz.B,17), ...
    'C',mat2str(Pz.C,17),'D',mat2str(Pz.D,17), ...
    'InitialCondition','0','SampleTime',num2str(Ts,17), ...
    'Position',[780 115 905 165]);
add_block('simulink/Signal Routing/Demux',[mdl '/Split_Speed_Angle'], ...
    'Outputs','2','Position',[960 118 965 162]);
add_block('simulink/Math Operations/Gain',[mdl '/Angle_to_Counts'], ...
    'Gain',num2str(N_enc/(2*pi),17),'Position',[1030 245 1135 275]);
add_block('simulink/Math Operations/Rounding Function',[mdl '/Quantize_Encoder'], ...
    'Operator','round','Position',[1185 245 1270 275]);
add_block('simulink/Discrete/Delay',[mdl '/Count_Delay'], ...
    'DelayLength',num2str(N_avg),'InitialCondition','0', ...
    'SampleTime',num2str(Ts,17),'Position',[1330 325 1400 355]);
add_block('simulink/Math Operations/Sum',[mdl '/Count_Diff'], ...
    'Inputs','+-','Position',[1435 245 1460 275]);
add_block('simulink/Math Operations/Gain',[mdl '/Enc_to_RadPerSec'], ...
    'Gain',num2str(2*pi/(N_enc*N_avg*Ts),17), ...
    'Position',[1520 245 1620 275]);
add_block('simulink/Discrete/Discrete Filter',[mdl '/Speed_Filter'], ...
    'Numerator',mat2str(1-alpha_f,17),'Denominator',mat2str([1,-alpha_f],17), ...
    'SampleTime',num2str(Ts,17),'Position',[1680 245 1790 275]);
% MATLAB normal-mode To Workspace logs; hardware uses To File + QUARC.
% s5_pi_cmd_raw matches PI_cmd_raw (after disturbance; pre final clamp).
% s5_pi_cmd matches PI_cmd (post final clamp == actual plant voltage).
logs={
'Log_Ref','s5_pi_ref',[40 35 140 65];
'Log_Error','s5_pi_error',[190 35 290 65];
'Log_PIOutput','s5_pi_internal_cmd',[325 35 430 65];
'Log_Disturbance','s5_pi_disturbance',[515 310 615 340];
'Log_CmdRaw','s5_pi_cmd_raw',[580 35 685 65];
'Log_Cmd','s5_pi_cmd',[695 35 790 65];
'Log_SpeedIdeal','s5_pi_speed_ideal',[985 35 1100 65];
'Log_Angle','s5_pi_angle',[1010 320 1110 350];
'Log_Counts','s5_pi_counts',[1190 385 1290 415];
'Log_Raw','s5_pi_speed_raw',[1520 325 1620 355];
'Log_Filtered','s5_pi_speed_filt',[1690 325 1805 355]};
for k=1:size(logs,1)
    add_block('simulink/Sinks/To Workspace',[mdl '/' logs{k,1}], ...
        'VariableName',logs{k,2},'SaveFormat','Array','Position',logs{k,3});
end
add_block('simulink/Signal Routing/Mux',[mdl '/Scope_Mux'], ...
    'Inputs','3','Position',[1850 85 1855 155]);
add_block('simulink/Sinks/Scope',[mdl '/Speed_Scope'], ...
    'Position',[1905 100 1945 140]);
% Hardware-matched signal routing.
add_line(mdl,'Speed_Ref/1','Error_Sum/1');
add_line(mdl,'Error_Sum/1','Discrete_PI/1');
add_line(mdl,'Discrete_PI/1','Dist_Sum/1');
add_line(mdl,'Disturbance/1','Dist_Sum/2','autorouting','on');
add_line(mdl,'Dist_Sum/1','Voltage_Sat/1');
add_line(mdl,'Voltage_Sat/1','Motor_Disk_ZOH/1');
add_line(mdl,'Motor_Disk_ZOH/1','Split_Speed_Angle/1');
add_line(mdl,'Split_Speed_Angle/2','Angle_to_Counts/1','autorouting','on');
add_line(mdl,'Angle_to_Counts/1','Quantize_Encoder/1');
add_line(mdl,'Quantize_Encoder/1','Count_Diff/1','autorouting','on');
add_line(mdl,'Quantize_Encoder/1','Count_Delay/1','autorouting','on');
add_line(mdl,'Count_Delay/1','Count_Diff/2','autorouting','on');
add_line(mdl,'Count_Diff/1','Enc_to_RadPerSec/1');
add_line(mdl,'Enc_to_RadPerSec/1','Speed_Filter/1');
add_line(mdl,'Speed_Filter/1','Error_Sum/2','autorouting','on');
add_line(mdl,'Speed_Ref/1','Log_Ref/1','autorouting','on');
add_line(mdl,'Error_Sum/1','Log_Error/1','autorouting','on');
add_line(mdl,'Discrete_PI/1','Log_PIOutput/1','autorouting','on');
add_line(mdl,'Disturbance/1','Log_Disturbance/1','autorouting','on');
add_line(mdl,'Dist_Sum/1','Log_CmdRaw/1','autorouting','on');
add_line(mdl,'Voltage_Sat/1','Log_Cmd/1','autorouting','on');
add_line(mdl,'Split_Speed_Angle/1','Log_SpeedIdeal/1','autorouting','on');
add_line(mdl,'Split_Speed_Angle/1','Scope_Mux/1','autorouting','on');
add_line(mdl,'Split_Speed_Angle/2','Log_Angle/1','autorouting','on');
add_line(mdl,'Quantize_Encoder/1','Log_Counts/1','autorouting','on');
add_line(mdl,'Enc_to_RadPerSec/1','Log_Raw/1','autorouting','on');
add_line(mdl,'Enc_to_RadPerSec/1','Scope_Mux/2','autorouting','on');
add_line(mdl,'Speed_Filter/1','Log_Filtered/1','autorouting','on');
add_line(mdl,'Speed_Filter/1','Scope_Mux/3','autorouting','on');
save_system(mdl,modelFile);
outDir=fullfile(projectDir,'results','05_pi_controller');
if ~exist(outDir,'dir'), mkdir(outDir); end
save(fullfile(outDir,'00_pi_stage05_parameters.mat'), ...
    'Pz','A','B','C','D','Kp','Ki','Ts','Vmax','Vmin','N_enc','N_avg', ...
    'alpha_f','Jeq','R','L','Ke','Kt','beq','K_dc','tau','parameterSource');
fid=fopen(fullfile(outDir,'00_build_provenance.txt'),'w');
fprintf(fid,'MATLAB release=%s, built=%s\n',version('-release'),datestr(now,31));
fprintf(fid,'Parameter file=%s\n',parameterSource);
fprintf(fid,'Model=%s\nKp=%.12g Ki=%.12g Ts=%.12g\n',modelFile,Kp,Ki,Ts);
fprintf(fid,'PI block: Parallel, Discrete-time, Forward Euler, SampleTime=Ts, internal clamp +/-10V, antiwindup=clamping.\n');
fprintf(fid,'Hardware PI exact order: PI internal saturation -> disturbance -> final saturation -> motor.\n');
fprintf(fid,'PI_cmd_raw = Dist_Sum (BEFORE final saturation); PI_cmd = final saturation.\n');
fprintf(fid,'Analog amplifier deadband/friction nonlinearity not included in this theoretical model.\n');
fclose(fid);
assert(strcmp(get_param([mdl '/Discrete_PI'],'AntiWindupMode'),'clamping'));
assert(strcmp(get_param([mdl '/Discrete_PI'],'IntegratorMethod'),'Forward Euler'));
assert(strcmp(get_param([mdl '/Discrete_PI'],'TimeDomain'),'Discrete-time'));
fprintf('[OK] Stage 5 hardware-topology PI model saved: %s\n',modelFile);
fprintf('Simulation-only gains: Kp=%.9f, Ki=%.9f; Ts=%.4f s.\n',Kp,Ki,Ts);
fprintf('Run ps5_stage05_run_pi_experiments.m for all trials, plots, metrics and comparison.\n');
