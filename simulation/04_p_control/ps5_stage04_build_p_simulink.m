%% EE302 PS5 - Stage 04: Hardware-topology-matched P speed simulation
% MATLAB / Simulink R2024a. No QUARC and NO hardware-identified parameters.
% Signal chain matches original model3_p_controller_hw.m:
% ref -> error -> Kp_only -> saturation -> (+disturbance) -> plant
% encoder counts -> 4-sample difference -> scale -> discrete filter -> feedback
% CRITICAL: voltage saturation is BEFORE disturbance, exactly as in hardware.
% P_cmd hardware log = output of Voltage_Sat (NOT plant input after disturbance).
clear; clc; close all;
thisDir=fileparts(mfilename('fullpath')); projectDir=fileparts(thisDir);
parfile=fullfile(projectDir,'02_identification','parameters_sim_identified.m');
if exist(parfile,'file')
    run(parfile); parameterSource=parfile;
else
    run(fullfile(thisDir,'parameters_stage04_snapshot.m'));
    parameterSource='User-confirmed Stage 02 identification snapshot';
    warning('Generated Stage-02 parameter file absent. Using recorded simulation-only snapshot.');
end
assert(abs(Ts-.002)<1e-12 && N_enc==2048 && N_avg==4,'Measurement settings must match hardware.');
assert(Vmax==10 && Vmin==-10,'Confirm limits before comparing with hardware.');
assert(abs(Kp_only-0.165965)<0.001,'Unexpected simulation P gain; inspect Stage-02 values.');
% Exact ZOH discretization of motor-current, speed, and position states.
A=[-R/L,-Ke/L,0;Kt/Jeq,-beq/Jeq,0;0,1,0];
B=[1/L;0;0]; C=[0,1,0;0,0,1]; D=[0;0];
Pz=c2d(ss(A,B,C,D),Ts,'zoh');
mdl='ps5_stage04_p_hardware_matched_R2024a'; modelFile=fullfile(thisDir,[mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist(modelFile,'file'), delete(modelFile); end
new_system(mdl); open_system(mdl);
set_param(mdl,'Solver','FixedStepDiscrete','FixedStep',num2str(Ts,17), ...
    'StopTime','10','ReturnWorkspaceOutputs','on');
% Reference and voltage-offset disturbance exactly match the HW default.
add_block('simulink/Sources/Step',[mdl '/Speed_Ref'], ...
    'Time','1','Before','0','After','50','SampleTime',num2str(Ts,17), ...
    'Position',[40 125 70 155]);
add_block('simulink/Sources/Step',[mdl '/Disturbance'], ...
    'Time','5','Before','0','After','0.5','SampleTime',num2str(Ts,17), ...
    'Position',[515 245 545 275]);
add_block('simulink/Math Operations/Sum',[mdl '/Error_Sum'], ...
    'Inputs','+-','Position',[160 125 185 155]);
add_block('simulink/Math Operations/Gain',[mdl '/Kp_Gain'], ...
    'Gain',num2str(Kp_only,17),'Position',[260 125 320 155]);
add_block('simulink/Discontinuities/Saturation',[mdl '/Voltage_Sat'], ...
    'UpperLimit',num2str(Vmax,17),'LowerLimit',num2str(Vmin,17), ...
    'Position',[400 125 450 155]);
add_block('simulink/Math Operations/Sum',[mdl '/Dist_Sum'], ...
    'Inputs','++','Position',[595 125 620 155]);
% Replace the original HIL WRITE/READ with a physical plant and encoder.
add_block('simulink/Discrete/Discrete State-Space',[mdl '/Motor_Disk_ZOH'], ...
    'A',mat2str(Pz.A,17),'B',mat2str(Pz.B,17), ...
    'C',mat2str(Pz.C,17),'D',mat2str(Pz.D,17), ...
    'InitialCondition','0','SampleTime',num2str(Ts,17), ...
    'Position',[720 115 845 165]);
add_block('simulink/Signal Routing/Demux',[mdl '/Split_Speed_Angle'], ...
    'Outputs','2','Position',[900 118 905 162]);
add_block('simulink/Math Operations/Gain',[mdl '/Angle_to_Counts'], ...
    'Gain',num2str(N_enc/(2*pi),17),'Position',[980 245 1085 275]);
add_block('simulink/Math Operations/Rounding Function',[mdl '/Quantize_Encoder'], ...
    'Operator','round','Position',[1135 245 1220 275]);
add_block('simulink/Discrete/Delay',[mdl '/Count_Delay'], ...
    'DelayLength',num2str(N_avg),'InitialCondition','0', ...
    'SampleTime',num2str(Ts,17),'Position',[1280 325 1350 355]);
add_block('simulink/Math Operations/Sum',[mdl '/Count_Diff'], ...
    'Inputs','+-','Position',[1385 245 1410 275]);
add_block('simulink/Math Operations/Gain',[mdl '/Enc_to_RadPerSec'], ...
    'Gain',num2str(2*pi/(N_enc*N_avg*Ts),17), ...
    'Position',[1470 245 1570 275]);
add_block('simulink/Discrete/Discrete Filter',[mdl '/Speed_Filter'], ...
    'Numerator',mat2str(1-alpha_f,17),'Denominator',mat2str([1,-alpha_f],17), ...
    'SampleTime',num2str(Ts,17),'Position',[1630 245 1740 275]);
% To Workspace full-array logs in NORMAL simulation, not External/QUARC.
% s4_p_cmd is the SAME location as P_cmd.mat in the original HW model.
logs={
'Log_Ref','s4_p_ref',[40 35 140 65];
'Log_Error','s4_p_error',[190 35 290 65];
'Log_Unclamped','s4_p_unclamped',[305 35 405 65];
'Log_Cmd','s4_p_cmd',[410 35 510 65];
'Log_Disturbance','s4_p_disturbance',[515 310 615 340];
'Log_PlantVoltage','s4_p_plant_voltage',[610 35 720 65];
'Log_SpeedIdeal','s4_p_speed_ideal',[925 35 1045 65];
'Log_Angle','s4_p_angle',[970 320 1070 350];
'Log_Counts','s4_p_counts',[1140 385 1240 415];
'Log_Raw','s4_p_speed_raw',[1470 325 1570 355];
'Log_Filtered','s4_p_speed_filt',[1640 325 1755 355]};
for k=1:size(logs,1)
    add_block('simulink/Sinks/To Workspace',[mdl '/' logs{k,1}], ...
        'VariableName',logs{k,2},'SaveFormat','Array','Position',logs{k,3});
end
add_block('simulink/Signal Routing/Mux',[mdl '/Scope_Mux'], ...
    'Inputs','3','Position',[1800 85 1805 155]);
add_block('simulink/Sinks/Scope',[mdl '/Speed_Scope'], ...
    'Position',[1855 100 1895 140]);
% Explicit signal wiring. No disturbance clamp AFTER the Dist_Sum.
add_line(mdl,'Speed_Ref/1','Error_Sum/1');
add_line(mdl,'Error_Sum/1','Kp_Gain/1');
add_line(mdl,'Kp_Gain/1','Voltage_Sat/1');
add_line(mdl,'Voltage_Sat/1','Dist_Sum/1');
add_line(mdl,'Disturbance/1','Dist_Sum/2','autorouting','on');
add_line(mdl,'Dist_Sum/1','Motor_Disk_ZOH/1');
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
add_line(mdl,'Kp_Gain/1','Log_Unclamped/1','autorouting','on');
add_line(mdl,'Voltage_Sat/1','Log_Cmd/1','autorouting','on');
add_line(mdl,'Disturbance/1','Log_Disturbance/1','autorouting','on');
add_line(mdl,'Dist_Sum/1','Log_PlantVoltage/1','autorouting','on');
add_line(mdl,'Split_Speed_Angle/1','Log_SpeedIdeal/1','autorouting','on');
add_line(mdl,'Split_Speed_Angle/1','Scope_Mux/1','autorouting','on');
add_line(mdl,'Split_Speed_Angle/2','Log_Angle/1','autorouting','on');
add_line(mdl,'Quantize_Encoder/1','Log_Counts/1','autorouting','on');
add_line(mdl,'Enc_to_RadPerSec/1','Log_Raw/1','autorouting','on');
add_line(mdl,'Enc_to_RadPerSec/1','Scope_Mux/2','autorouting','on');
add_line(mdl,'Speed_Filter/1','Log_Filtered/1','autorouting','on');
add_line(mdl,'Speed_Filter/1','Scope_Mux/3','autorouting','on');
save_system(mdl,modelFile);
outDir=fullfile(projectDir,'results','04_p_controller');
if ~exist(outDir,'dir'), mkdir(outDir); end
save(fullfile(outDir,'00_p_stage04_parameters.mat'), ...
    'Pz','A','B','C','D','Kp_only','Ts','Vmax','Vmin','N_enc','N_avg','alpha_f', ...
    'Jeq','R','L','Ke','Kt','beq','parameterSource');
fid=fopen(fullfile(outDir,'00_build_provenance.txt'),'w');
fprintf(fid,'Release: %s\nDate: %s\n',version('-release'),datestr(now,31));
fprintf(fid,'Parameter source: %s\n',parameterSource);
fprintf(fid,'Model: %s\n',modelFile);
fprintf(fid,'Kp_only=%.12g, Ts=%.12g, N_enc=%d, N_avg=%d, alpha=%.12g\n', ...
    Kp_only,Ts,N_enc,N_avg,alpha_f);
fprintf(fid,'Exact hardware P ordering: error->gain->sat->disturbance->plant.\n');
fprintf(fid,'P_cmd is controller-clamped voltage BEFORE the disturbance.\n');
fprintf(fid,'Hardware amplifier default 0.65V deadband is not included in original HW script or baseline simulation.\n');
fclose(fid);
fprintf('[OK] Stage 4 P model built: %s\n',modelFile);
fprintf('Run ps5_stage04_run_p_experiments to simulate and save every scenario.\n');
