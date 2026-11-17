%% EE302 PS5 - Updated open-loop hardware-identification model (R2024a)
% Builds: ps5_openloop_identification_2ms_UPDATED.slx
%
% Reconstructs the supplied open-loop identification diagram, with:
%   (a) Ts = 0.002 s (controller sampling period),
%   (b) ONE-sample angular-speed difference (no 4-sample window), and
%   (c) the controller speed filter H(z)=(1-alpha)/(1-alpha*z^-1).
%
% QUARC is NOT required to build the diagram. The two HIL blocks below are
% INPORT / OUTPORT placeholders, not functional hardware I/O.
%
% IMPORTANT: This is an UPDATED implementation, NOT the acquisition model
% used for the previously recorded 5 ms hardware identification datasets.
% Do not attribute those historical datasets to this updated model.
%
% Run this script directly in MATLAB R2024a. It creates the .slx and attempts
% to export a PNG diagram in the same folder as this script.

clc;
thisDir = fileparts(mfilename('fullpath'));
mdl = 'ps5_openloop_identification_2ms_UPDATED';
modelPath = fullfile(thisDir,[mdl '.slx']);
pngPath = fullfile(thisDir,[mdl '_diagram.png']);

%% Fixed acquisition / measurement settings
Ts = 0.002;                  % 2 ms controller sampling interval
N_enc = 2048;               % quadrature encoder counts per revolution
tau_f = 0.010;              % 100 rad/s filter design (backward Euler)
alpha_f = tau_f/(tau_f+Ts); % = 0.8333333333333
encoderSign = -1;           % matches the negative Counts_to_rad gain in
                            % the supplied reference screenshot. Confirm
                            % actual polarity before physical wiring.
Vmax = 10; Vmin = -10;      % commanded voltage limits
Tstop = 10;

% Recreate the 3 V step and the staircase switching schedule recorded in
% the supplied hardware identification data. These source settings do not
% imply the old measurements were acquired using this UPDATED 2 ms model.
tStep = 0.5;
stairEvents = [1.6 1; 2.4 2; 3.2 3; 4.0 4; 4.8 5; ...
               5.6 0; 7.2 1; 8.0 2; 8.8 3; 9.6 4];
tGrid = (0:Ts:Tstop)';
uStair = zeros(size(tGrid));
for j=1:size(stairEvents,1)
    uStair(tGrid>=stairEvents(j,1)) = stairEvents(j,2);
end
staircase_data = [tGrid uStair];

%% Start a clean model
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist(modelPath,'file'), delete(modelPath); end
new_system(mdl);
open_system(mdl);
set_param(mdl,'Solver','FixedStepDiscrete', ...
    'FixedStep',num2str(Ts,16),'StopTime',num2str(Tstop), ...
    'SaveTime','on','TimeSaveName','t', ...
    'SaveFormat','Array','ReturnWorkspaceOutputs','on');

% Store the staircase INPUT IN THE MODEL ITSELF so the .slx remains
% self-contained if moved to a different MATLAB folder/computer.
modelWs = get_param(mdl,'ModelWorkspace');
assignin(modelWs,'staircase_data',staircase_data);

%% Source blocks (same roles as original screenshot)
add_block('simulink/Sources/Step',[mdl '/Step_Input_3V'], ...
    'Time',num2str(tStep),'Before','0','After','3', ...
    'SampleTime',num2str(Ts,16), ...
    'Position',[65 165 125 215]);
add_block('simulink/Sources/From Workspace',[mdl '/Staircase_Sequence'], ...
    'VariableName','staircase_data','Interpolate','off', ...
    'Position',[55 295 210 355]);
add_block('simulink/Signal Routing/Manual Switch', ...
    [mdl '/Select_Step_or_Staircase'], ...
    'Position',[320 210 360 255]);
set_param([mdl '/Select_Step_or_Staircase'],'sw','1'); % first/top: 3 V step

% Emergency_Drive_Enable is a SOFTWARE COMMAND selector only. Selecting
% Zero_Command_Stop outputs 0 V; it is NOT a proven hardware drive-disable,
% fault handler, or physical emergency-stop circuit.
add_block('simulink/Sources/Constant',[mdl '/Zero_Command_Stop'], ...
    'Value','0','SampleTime',num2str(Ts,16), ...
    'Position',[405 345 475 385]);
add_block('simulink/Signal Routing/Manual Switch', ...
    [mdl '/Emergency_Drive_Enable'], ...
    'Position',[565 210 605 255]);
set_param([mdl '/Emergency_Drive_Enable'],'sw','0'); % second/bottom: zero

add_block('simulink/Discontinuities/Saturation', ...
    [mdl '/Hardware_Command_Saturation'], ...
    'UpperLimit',num2str(Vmax),'LowerLimit',num2str(Vmin), ...
    'Position',[820 212 925 252]);

%% QUARC placeholders: In1 and Out1 (NO device communication)
add_block('simulink/Sinks/Out1',[mdl '/HIL_Write_Analog_PLACEHOLDER'], ...
    'Position',[1285 175 1320 195]);
add_block('simulink/Sources/In1',[mdl '/HIL_Read_Encoder_PLACEHOLDER'], ...
    'Position',[740 500 780 520]);

%% Encoder scaling and SINGLE-sample speed estimation
% H_speed(z) = (1-z^-1)/Ts = 500 - 500 z^-1
% No N_avg=4 block exists in this model.
countsToRad = encoderSign*2*pi/N_enc;
add_block('simulink/Math Operations/Gain',[mdl '/Counts_to_rad'], ...
    'Gain',num2str(countsToRad,16), ...
    'Position',[835 490 940 530]);
add_block('simulink/Discrete/Discrete Filter', ...
    [mdl '/Discrete_Speed_Estimator'], ...
    'Numerator',mat2str([1/Ts -1/Ts],16), ...
    'Denominator','1', ...
    'SampleTime',num2str(Ts,16), ...
    'Position',[1035 485 1180 535]);
add_block('simulink/Discrete/Discrete Filter',[mdl '/Speed_Filter'], ...
    'Numerator',mat2str(1-alpha_f,16), ...
    'Denominator',mat2str([1 -alpha_f],16), ...
    'SampleTime',num2str(Ts,16), ...
    'Position',[1250 485 1390 535]);

%% Logging: u, theta_meas and w_meas (MATLAB normal-mode To Workspace)
add_ws_log(mdl,'ToWorkspace_u','u',[1100 95 1225 130]);
add_ws_log(mdl,'ToWorkspace_theta','theta_meas',[1040 610 1190 645]);
add_ws_log(mdl,'ToWorkspace_w_meas','w_meas',[1460 610 1620 645]);

% Scope with the same three signals visible in the supplied diagram:
% u (command voltage), theta_meas (angle), w_meas (filtered speed).
add_block('simulink/Sinks/Scope',[mdl '/Scope_Hardware_Identification'], ...
    'NumInputPorts','3','Position',[1515 250 1570 350]);

%% Connections
add_line(mdl,'Step_Input_3V/1','Select_Step_or_Staircase/1', ...
    'autorouting','on');
add_line(mdl,'Staircase_Sequence/1','Select_Step_or_Staircase/2', ...
    'autorouting','on');
add_line(mdl,'Select_Step_or_Staircase/1','Emergency_Drive_Enable/1', ...
    'autorouting','on');
add_line(mdl,'Zero_Command_Stop/1','Emergency_Drive_Enable/2', ...
    'autorouting','on');
add_line(mdl,'Emergency_Drive_Enable/1','Hardware_Command_Saturation/1', ...
    'autorouting','on');
add_line(mdl,'Hardware_Command_Saturation/1','HIL_Write_Analog_PLACEHOLDER/1', ...
    'autorouting','on');
add_line(mdl,'Hardware_Command_Saturation/1','ToWorkspace_u/1', ...
    'autorouting','on');
add_line(mdl,'Hardware_Command_Saturation/1','Scope_Hardware_Identification/1', ...
    'autorouting','on');
add_line(mdl,'HIL_Read_Encoder_PLACEHOLDER/1','Counts_to_rad/1', ...
    'autorouting','on');
add_line(mdl,'Counts_to_rad/1','Discrete_Speed_Estimator/1', ...
    'autorouting','on');
add_line(mdl,'Counts_to_rad/1','ToWorkspace_theta/1', ...
    'autorouting','on');
add_line(mdl,'Counts_to_rad/1','Scope_Hardware_Identification/2', ...
    'autorouting','on');
add_line(mdl,'Discrete_Speed_Estimator/1','Speed_Filter/1', ...
    'autorouting','on');
add_line(mdl,'Speed_Filter/1','ToWorkspace_w_meas/1', ...
    'autorouting','on');
add_line(mdl,'Speed_Filter/1','Scope_Hardware_Identification/3', ...
    'autorouting','on');

% The saved manual-stop selection should stay on its zero-command branch.
assert(strcmp(get_param([mdl '/Emergency_Drive_Enable'],'sw'),'0'), ...
    'Unexpected drive selection. The model must start on zero command.');

save_system(mdl,modelPath);
try
    print(['-s' mdl],'-dpng','-r170',pngPath);
    fprintf('Saved diagram preview: %s\n',pngPath);
catch ME
    warning('Model saved. Optional diagram export failed: %s',ME.message);
    fprintf('Use Simulink > File > Export Model To > Image to save a PNG.\n');
end
open_system(mdl);
fprintf('\n[OK] Updated 2 ms model saved: %s\n',modelPath);
fprintf('Speed estimator: %.0f*(1-z^-1), one-sample difference\n',1/Ts);
fprintf('Filter: (%.9f)/(1-%.9f*z^-1)\n',1-alpha_f,alpha_f);
fprintf('Encoder scaling: %.12f rad/count\n',countsToRad);
fprintf('Stop selector defaults to Zero_Command_Stop (0 V).\n');
fprintf('HIL blocks are nonfunctional placeholders.\n');
fprintf('This model was NOT used to record the earlier 5 ms datasets.\n');

function add_ws_log(mdl,blk,varname,pos)
    add_block('simulink/Sinks/To Workspace',[mdl '/' blk], ...
        'VariableName',varname,'SaveFormat','Array','Position',pos);
end
