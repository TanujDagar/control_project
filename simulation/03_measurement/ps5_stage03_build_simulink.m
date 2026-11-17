%% EE302 PS5 - Stage 03: Build & run R2024a measurement Simulink plant
% Builds an OPEN-LOOP fixed-step discrete Simulink model matching the supplied
% hardware encoder scaling, 4-sample difference, and discrete speed filter.
% NO QUARC, PID controller or hardware logs. Runs a 2 V input at t=0.1 s.
% Run from any current MATLAB folder. Save model and logged results to project.
clear; clc; close all;
thisDir=fileparts(mfilename('fullpath'));
projectDir=fileparts(thisDir);
outDir=fullfile(projectDir,'results','03_measurement');
if ~exist(outDir,'dir'), mkdir(outDir); end
params=fullfile(projectDir,'02_identification','parameters_sim_identified.m');
if exist(params,'file')
    run(params);
else
    run(fullfile(thisDir,'parameters_stage03_snapshot.m'));
    warning('Using Stage-02 transcript snapshot, not original generated file.');
end

% For exact sample-to-sample angle/speed consistency, discretize the
% augmented 3-state model x = [current; angular speed; angular position].
Ac=[-R/L,-Ke/L,0;Kt/Jeq,-beq/Jeq,0;0,1,0];
Bc=[1/L;0;0];
Cc=[0,1,0;0,0,1];
Dc=[0;0];
Pz=c2d(ss(Ac,Bc,Cc,Dc),Ts,'zoh');

mdl='ps5_stage03_encoder_measurement_R2024a';
modelFile=fullfile(thisDir,[mdl,'.slx']);
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist(modelFile,'file'), delete(modelFile); end
new_system(mdl);
open_system(mdl);
set_param(mdl,'Solver','FixedStepDiscrete','FixedStep',num2str(Ts,16), ...
    'StopTime','1.5','ReturnWorkspaceOutputs','on');

%% Signal source and sampled motor dynamics
add_block('simulink/Sources/Step',[mdl '/Voltage_Step'], ...
    'Time','0.1','Before','0','After','2', ...
    'SampleTime',num2str(Ts,16), ...
    'Position',[35 115 65 145]);
add_block('simulink/Discrete/Discrete State-Space',[mdl '/Motor_Disk_ZOH'], ...
    'A',mat2str(Pz.A,17),'B',mat2str(Pz.B,17), ...
    'C',mat2str(Pz.C,17),'D',mat2str(Pz.D,17), ...
    'InitialCondition','0','SampleTime',num2str(Ts,16), ...
    'Position',[125 102 255 158]);
add_block('simulink/Signal Routing/Demux',[mdl '/Split_Speed_Angle'], ...
    'Outputs','2','Position',[305 103 310 157]);

%% Exact encoder chain from uploaded model3/model4 scripts
add_block('simulink/Math Operations/Gain',[mdl '/Angle_to_Counts'], ...
    'Gain',num2str(N_enc/(2*pi),17), ...
    'Position',[355 182 445 218]);
add_block('simulink/Math Operations/Rounding Function',[mdl '/Quantize_Encoder'], ...
    'Operator','round','Position',[475 182 560 218]);
add_block('simulink/Discrete/Delay',[mdl '/Count_Delay'], ...
    'DelayLength',num2str(N_avg),'InitialCondition','0', ...
    'SampleTime',num2str(Ts,16), ...
    'Position',[600 272 665 308]);
add_block('simulink/Math Operations/Sum',[mdl '/Count_Diff'], ...
    'Inputs','+-','Position',[715 187 745 213]);
add_block('simulink/Math Operations/Gain',[mdl '/Enc_to_RadPerSec'], ...
    'Gain',num2str(2*pi/(N_enc*N_avg*Ts),17), ...
    'Position',[785 177 900 223]);
add_block('simulink/Discrete/Discrete Filter',[mdl '/Speed_Filter'], ...
    'Numerator',mat2str([1-alpha_f],17), ...
    'Denominator',mat2str([1,-alpha_f],17), ...
    'SampleTime',num2str(Ts,16), ...
    'Position',[955 180 1040 220]);

%% Array-format normal-mode logs; NOT External Mode hardware logs
add_ws_log(mdl,'Log_Input','s3_voltage',[35 25 135 55]);
add_ws_log(mdl,'Log_Ideal','s3_speed_ideal',[340 40 440 70]);
add_ws_log(mdl,'Log_Angle','s3_angle_rad',[355 270 455 300]);
add_ws_log(mdl,'Log_Counts','s3_encoder_counts',[475 330 585 360]);
add_ws_log(mdl,'Log_Raw','s3_speed_raw',[785 275 885 305]);
add_ws_log(mdl,'Log_Filtered','s3_speed_filt',[955 300 1070 330]);
add_block('simulink/Signal Routing/Mux',[mdl '/Scope_Mux'], ...
    'Inputs','3','Position',[1115 55 1120 135]);
add_block('simulink/Sinks/Scope',[mdl '/Speed_Scope'], ...
    'Position',[1160 72 1205 118]);

%% Physical signal lines
add_line(mdl,'Voltage_Step/1','Motor_Disk_ZOH/1');
add_line(mdl,'Voltage_Step/1','Log_Input/1','autorouting','on');
add_line(mdl,'Motor_Disk_ZOH/1','Split_Speed_Angle/1');
add_line(mdl,'Split_Speed_Angle/1','Log_Ideal/1','autorouting','on');
add_line(mdl,'Split_Speed_Angle/1','Scope_Mux/1','autorouting','on');
add_line(mdl,'Split_Speed_Angle/2','Angle_to_Counts/1','autorouting','on');
add_line(mdl,'Split_Speed_Angle/2','Log_Angle/1','autorouting','on');
add_line(mdl,'Angle_to_Counts/1','Quantize_Encoder/1');
add_line(mdl,'Quantize_Encoder/1','Count_Diff/1','autorouting','on');
add_line(mdl,'Quantize_Encoder/1','Count_Delay/1','autorouting','on');
add_line(mdl,'Quantize_Encoder/1','Log_Counts/1','autorouting','on');
add_line(mdl,'Count_Delay/1','Count_Diff/2','autorouting','on');
add_line(mdl,'Count_Diff/1','Enc_to_RadPerSec/1');
add_line(mdl,'Enc_to_RadPerSec/1','Speed_Filter/1');
add_line(mdl,'Enc_to_RadPerSec/1','Log_Raw/1','autorouting','on');
add_line(mdl,'Enc_to_RadPerSec/1','Scope_Mux/2','autorouting','on');
add_line(mdl,'Speed_Filter/1','Log_Filtered/1','autorouting','on');
add_line(mdl,'Speed_Filter/1','Scope_Mux/3','autorouting','on');
save_system(mdl,modelFile);

%% Run normal-mode Simulink and preserve full logged traces
simOut=sim(mdl);
try
    v=simOut.get('s3_voltage');
    speed=simOut.get('s3_speed_ideal');
    theta=simOut.get('s3_angle_rad');
    counts=simOut.get('s3_encoder_counts');
    raw=simOut.get('s3_speed_raw');
    filt=simOut.get('s3_speed_filt');
    v=v(:);speed=speed(:);theta=theta(:);counts=counts(:);
    raw=raw(:);filt=filt(:);
    n=numel(speed);
    assert(all([numel(v),numel(theta),numel(counts),numel(raw),numel(filt)]==n), ...
        'Logger arrays have different lengths. Inspect the model.');
    tsig=(0:n-1)'*Ts;
    writematrix([tsig,v,speed,theta,counts,raw,filt], ...
        fullfile(outDir,'20_simulink_discrete_measurement.csv'));
    save(fullfile(outDir,'21_simulink_measurement.mat'), ...
        'tsig','v','speed','theta','counts','raw','filt','Pz','Ts', ...
        'N_enc','N_avg','alpha_f','simOut');
    % Independent algebraic check of the three encoder blocks
    qcalc=round(theta*N_enc/(2*pi));
    rcalc=zeros(n,1);
    for k=N_avg+1:n
       rcalc(k)=(qcalc(k)-qcalc(k-N_avg))*2*pi/(N_enc*N_avg*Ts);
    end
    fcalc=filter(1-alpha_f,[1,-alpha_f],rcalc);
    maxCountError=max(abs(counts-qcalc));
    maxRawError=max(abs(raw-rcalc));
    maxFilterError=max(abs(filt-fcalc));
    assert(maxCountError<1e-7 && maxRawError<1e-7 && maxFilterError<1e-7, ...
       'Simulink encoder/filter outputs mismatch independent discrete equations.');
    fig=figure('Color','w','Visible','off','Position',[100 100 1000 600]);
    plot(tsig,speed,'LineWidth',1.6); hold on;
    plot(tsig,raw,'LineWidth',0.8);
    plot(tsig,filt,'LineWidth',1.3); grid on;
    xlabel('Time (s)'); ylabel('Speed (rad/s)');
    title('Stage 03: actual R2024a Simulink measurement outputs');
    legend('Sampled ideal','Raw encoder speed','Filtered encoder speed', ...
        'Location','southeast');
    savefig(fig,fullfile(outDir,'22_simulink_measurement.fig'));
    exportgraphics(fig,fullfile(outDir,'22_simulink_measurement.png'),'Resolution',180);
    close(fig);
    fprintf('Simulink consistency errors: counts=%.3g, raw=%.3g, filter=%.3g\n', ...
        maxCountError,maxRawError,maxFilterError);
    fprintf('Saved Simulink logs: %s\n',outDir);
catch ME
    warning('Model built/simulated, but trace parsing/check failed: %s',ME.message);
    save(fullfile(outDir,'21_simulink_raw_output_for_debug.mat'),'simOut','Pz');
    rethrow(ME);
end
fprintf('[OK] Saved and ran model: %s\n',modelFile);

function add_ws_log(mdl,name,varname,pos)
    add_block('simulink/Sinks/To Workspace',[mdl '/' name], ...
        'VariableName',varname,'SaveFormat','Array','Position',pos);
end
