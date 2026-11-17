%% build_model3_p_controller_hw.m
%  MODEL 3: P Speed Controller (Hardware Implementation - Manual HIL Placeholders)
%  Creates: model3_p_controller_hw.slx (MATLAB R2024b)
%
%  DATA LOGGING: To File blocks (Array format) are used instead of To Workspace.
%  Under QUARC / External mode, To Workspace only returns the LAST sample.
%  To File writes the whole run to disk. Each .mat file holds a 2 x N matrix:
%      row 1 = time [s], row 2 = signal
%  Files are written to the MATLAB current folder when the run stops cleanly
%  (let it reach StopTime = 10 s).
%      P_ref.mat  P_error.mat  P_cmd.mat  P_speed_raw.mat  P_speed_filt.mat
%  Verify with:  check_logged_data('P')

clear; close all; clc;
run('parameters.m');

% Fallback safe gain if Kp_only has not been added to parameters.m yet
if ~exist('Kp_only', 'var')
    Kp_only = 0.05;
    warning('Kp_only not found in parameters.m. Using temporary safe value Kp_only = 0.05.');
end

% Speed-measurement window (samples). 1 = plain difference (jittery), 4 = recommended.
N_avg = 4;

mdl = 'model3_p_controller_hw';
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);

% Fixed-step discrete solver
set_param(mdl, ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(Ts), ...
    'StopTime', '10', ...
    'ReturnWorkspaceOutputs', 'off');

% External Mode settings (only applied once SystemTargetFile = quarc_win64.tlc)
try
    set_param(mdl, ...
        'ExtModeArmWhenConnect', 'on', ...
        'ExtModeTrigType', 'manual', ...
        'ExtModeTrigDuration', num2str(round(10 / Ts)));
catch
    % Ignored until SystemTargetFile is switched to QUARC target
end

%% ===== Signal Sources =====
% Speed reference: 50 rad/s step at t = 1 s
add_block('simulink/Sources/Step', [mdl '/Speed_Ref']);
set_param([mdl '/Speed_Ref'], ...
    'Time', '1', 'Before', '0', 'After', '50', ...
    'Position', [30, 92, 60, 118]);

% Input disturbance: 0.5 V offset at t = 5 s
add_block('simulink/Sources/Step', [mdl '/Disturbance']);
set_param([mdl '/Disturbance'], ...
    'Time', '5', 'Before', '0', 'After', '0.5', ...
    'Position', [380, 162, 410, 188]);

%% ===== Error Computation: e = r - y_filtered =====
add_block('simulink/Math Operations/Sum', [mdl '/Error_Sum']);
set_param([mdl '/Error_Sum'], ...
    'Inputs', '+-', ...
    'Position', [120, 95, 140, 115]);

%% ===== P Controller =====
add_block('simulink/Math Operations/Gain', [mdl '/Kp_Gain']);
set_param([mdl '/Kp_Gain'], ...
    'Gain', num2str(Kp_only), ...
    'Position', [210, 92, 260, 118]);

%% ===== Voltage Saturation =====
add_block('simulink/Discontinuities/Saturation', [mdl '/Voltage_Sat']);
set_param([mdl '/Voltage_Sat'], ...
    'UpperLimit', num2str(Vmax), ...
    'LowerLimit', num2str(Vmin), ...
    'Position', [320, 92, 370, 118]);

%% ===== Disturbance Injection: u_total = u + d =====
add_block('simulink/Math Operations/Sum', [mdl '/Dist_Sum']);
set_param([mdl '/Dist_Sum'], ...
    'Inputs', '++', ...
    'Position', [440, 95, 460, 115]);

%% ===== HARDWARE I/O PLACEHOLDERS =====
add_block('simulink/Sinks/Out1', [mdl '/PLACEHOLDER_HIL_WRITE']);
set_param([mdl '/PLACEHOLDER_HIL_WRITE'], 'Position', [520, 95, 550, 115]);

add_block('simulink/Sources/In1', [mdl '/PLACEHOLDER_HIL_READ']);
set_param([mdl '/PLACEHOLDER_HIL_READ'], 'Position', [520, 240, 550, 260]);

%% ===== Encoder-to-Speed Conversion (N-sample windowed difference) =====
% Speed = (x[k] - x[k-N_avg]) / (N_avg*Ts). A single-sample difference is badly
% affected by USB read-timing jitter (counts alternate ~0.5x / 1.5x of true value).
% Averaging over N_avg samples cancels the jitter (error shrinks ~ 1/N_avg).
add_block('simulink/Discrete/Delay', [mdl '/Count_Delay']);
set_param([mdl '/Count_Delay'], ...
    'DelayLength',      num2str(N_avg), ...
    'InitialCondition', '0', ...
    'SampleTime',       num2str(Ts), ...
    'Position',         [580, 285, 630, 315]);

add_block('simulink/Math Operations/Sum', [mdl '/Count_Diff']);
set_param([mdl '/Count_Diff'], 'Inputs', '+-', 'Position', [650, 240, 670, 260]);

add_block('simulink/Math Operations/Gain', [mdl '/Enc_to_RadPerSec']);
set_param([mdl '/Enc_to_RadPerSec'], ...
    'Gain', num2str(2*pi / (N_enc * N_avg * Ts), 12), ...
    'Position', [700, 235, 760, 265]);

%% ===== Speed Filter (discrete low-pass) =====
add_block('simulink/Discrete/Discrete Filter', [mdl '/Speed_Filter']);
set_param([mdl '/Speed_Filter'], ...
    'Numerator',   mat2str([1 - alpha_f]), ...
    'Denominator', mat2str([1, -alpha_f]), ...
    'SampleTime',  num2str(Ts), ...
    'Position',    [400, 310, 470, 340], ...
    'Orientation', 'left');

%% ===== Data Logging (To File - Array format: [time; signal]) =====
add_file_log(mdl, 'Log_Ref',       'P_ref.mat',        Ts, [30, 25, 110, 50],   'right');
add_file_log(mdl, 'Log_Error',     'P_error.mat',      Ts, [210, 25, 290, 50],  'right');
add_file_log(mdl, 'Log_Cmd',       'P_cmd.mat',        Ts, [320, 25, 400, 50],  'right');
add_file_log(mdl, 'Log_SpeedRaw',  'P_speed_raw.mat',  Ts, [810, 237, 890, 263],'right');
add_file_log(mdl, 'Log_SpeedFilt', 'P_speed_filt.mat', Ts, [250, 312, 330, 338],'left');

%% ===== Scopes =====
add_block('simulink/Sinks/Scope', [mdl '/Speed_Scope']);
set_param([mdl '/Speed_Scope'], 'Position', [810, 295, 850, 335]);

add_block('simulink/Sinks/Scope', [mdl '/Cmd_Scope']);
set_param([mdl '/Cmd_Scope'], 'Position', [520, 155, 560, 195]);

%% ===== Connections =====
add_line(mdl, 'Speed_Ref/1',             'Error_Sum/1');
add_line(mdl, 'Error_Sum/1',             'Kp_Gain/1');
add_line(mdl, 'Kp_Gain/1',               'Voltage_Sat/1');
add_line(mdl, 'Voltage_Sat/1',           'Dist_Sum/1');
add_line(mdl, 'Disturbance/1',           'Dist_Sum/2');
add_line(mdl, 'Dist_Sum/1',              'PLACEHOLDER_HIL_WRITE/1');

add_line(mdl, 'PLACEHOLDER_HIL_READ/1',  'Count_Diff/1');
add_line(mdl, 'PLACEHOLDER_HIL_READ/1',  'Count_Delay/1', 'autorouting', 'on');
add_line(mdl, 'Count_Delay/1',           'Count_Diff/2',  'autorouting', 'on');
add_line(mdl, 'Count_Diff/1',            'Enc_to_RadPerSec/1');
add_line(mdl, 'Enc_to_RadPerSec/1',      'Speed_Filter/1', 'autorouting', 'on');
add_line(mdl, 'Speed_Filter/1',          'Error_Sum/2', 'autorouting', 'on');

add_line(mdl, 'Speed_Ref/1',             'Log_Ref/1');
add_line(mdl, 'Error_Sum/1',             'Log_Error/1');
add_line(mdl, 'Voltage_Sat/1',           'Log_Cmd/1');
add_line(mdl, 'Enc_to_RadPerSec/1',      'Log_SpeedRaw/1');
add_line(mdl, 'Speed_Filter/1',          'Log_SpeedFilt/1', 'autorouting', 'on');

add_line(mdl, 'Speed_Filter/1',          'Speed_Scope/1', 'autorouting', 'on');
add_line(mdl, 'Voltage_Sat/1',           'Cmd_Scope/1');

%% ===== Save =====
save_system(mdl);
fprintf('\n[OK] Model 3 saved: %s.slx (Kp_only = %.4f, N_avg = %d)\n', mdl, Kp_only, N_avg);
fprintf('   1) Replace PLACEHOLDER_HIL_WRITE / PLACEHOLDER_HIL_READ with your QUARC blocks.\n');
fprintf('   2) Run Monitor & Tune and let it STOP BY ITSELF at 10 s.\n');
fprintf('   3) Logged files (current folder): P_ref, P_error, P_cmd, P_speed_raw, P_speed_filt (.mat)\n');
fprintf('   4) Verify with: check_logged_data(''P'')\n\n');

%% ===== Local helper =====
function add_file_log(mdl, blkname, fname, Ts, pos, orient)
    blk = [mdl '/' blkname];
    add_block('simulink/Sinks/To File', blk);
    set_param(blk, ...
        'Filename',   fname, ...
        'MatrixName', erase(fname, '.mat'), ...
        'SaveFormat', 'Array', ...
        'SampleTime', num2str(Ts), ...
        'Position',   pos, ...
        'Orientation', orient);
end
