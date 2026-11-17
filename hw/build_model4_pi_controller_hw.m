%% build_model4_pi_controller_hw.m
%  MODEL 4: Discrete PI Speed Controller with Anti-Windup (Manual HIL Placeholders)
%  Creates: model4_pi_controller_hw.slx (MATLAB R2024b)
%
%  DATA LOGGING: To File blocks (Array format) are used instead of To Workspace.
%  Under QUARC / External mode, To Workspace only returns the LAST sample.
%  To File writes the whole run to disk. Each .mat file holds a 2 x N matrix:
%      row 1 = time [s], row 2 = signal
%  Files are written to the MATLAB current folder when the run stops cleanly
%  (let it reach StopTime = 10 s).
%      PI_ref.mat  PI_error.mat  PI_cmd_raw.mat  PI_cmd.mat
%      PI_speed_raw.mat  PI_speed_filt.mat
%  Verify with:  check_logged_data('PI')

clear; close all; clc;
run('parameters.m');

% Speed-measurement window (samples). 1 = plain difference (jittery), 4 = recommended.
N_avg = 4;

mdl = 'model4_pi_controller_hw';
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);

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
end

%% ===== Signal Sources =====
add_block('simulink/Sources/Step', [mdl '/Speed_Ref']);
set_param([mdl '/Speed_Ref'], 'Time', '1', 'Before', '0', 'After', '50', ...
    'Position', [30, 92, 60, 118]);

add_block('simulink/Sources/Step', [mdl '/Disturbance']);
set_param([mdl '/Disturbance'], 'Time', '5', 'Before', '0', 'After', '0.5', ...
    'Position', [360, 162, 390, 188]);

%% ===== Error & Discrete PI with Clamping Anti-Windup =====
add_block('simulink/Math Operations/Sum', [mdl '/Error_Sum']);
set_param([mdl '/Error_Sum'], 'Inputs', '+-', 'Position', [120, 95, 140, 115]);

add_block('simulink/Continuous/PID Controller', [mdl '/Discrete_PI']);
set_param([mdl '/Discrete_PI'], ...
    'Controller', 'PI', ...
    'TimeDomain', 'Discrete-time', ...
    'IntegratorMethod', 'Forward Euler', ...
    'P', 'Kp', ...
    'I', 'Ki', ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', num2str(Vmax), ...
    'LowerSaturationLimit', num2str(Vmin), ...
    'AntiWindupMode', 'clamping', ...
    'Position', [195, 87, 265, 123]);

%% ===== Disturbance & Final Voltage Clamp =====
add_block('simulink/Math Operations/Sum', [mdl '/Dist_Sum']);
set_param([mdl '/Dist_Sum'], 'Inputs', '++', 'Position', [420, 95, 440, 115]);

add_block('simulink/Discontinuities/Saturation', [mdl '/Voltage_Sat']);
set_param([mdl '/Voltage_Sat'], ...
    'UpperLimit', num2str(Vmax), ...
    'LowerLimit', num2str(Vmin), ...
    'Position', [480, 92, 520, 118]);

%% ===== HARDWARE I/O PLACEHOLDERS =====
add_block('simulink/Sinks/Out1', [mdl '/PLACEHOLDER_HIL_WRITE']);
set_param([mdl '/PLACEHOLDER_HIL_WRITE'], 'Position', [580, 95, 610, 115]);

add_block('simulink/Sources/In1', [mdl '/PLACEHOLDER_HIL_READ']);
set_param([mdl '/PLACEHOLDER_HIL_READ'], 'Position', [580, 240, 610, 260]);

%% ===== Encoder-to-Speed (N-sample windowed difference) & Low-Pass Filter =====
% Speed = (x[k] - x[k-N_avg]) / (N_avg*Ts). A single-sample difference is badly
% affected by USB read-timing jitter (counts alternate ~0.5x / 1.5x of true value).
% Averaging over N_avg samples cancels the jitter (error shrinks ~ 1/N_avg).
add_block('simulink/Discrete/Delay', [mdl '/Count_Delay']);
set_param([mdl '/Count_Delay'], ...
    'DelayLength',      num2str(N_avg), ...
    'InitialCondition', '0', ...
    'SampleTime',       num2str(Ts), ...
    'Position',         [630, 285, 680, 315]);

add_block('simulink/Math Operations/Sum', [mdl '/Count_Diff']);
set_param([mdl '/Count_Diff'], 'Inputs', '+-', 'Position', [700, 240, 720, 260]);

add_block('simulink/Math Operations/Gain', [mdl '/Enc_to_RadPerSec']);
set_param([mdl '/Enc_to_RadPerSec'], ...
    'Gain', num2str(2*pi / (N_enc * N_avg * Ts), 12), ...
    'Position', [745, 235, 805, 265]);

add_block('simulink/Discrete/Discrete Filter', [mdl '/Speed_Filter']);
set_param([mdl '/Speed_Filter'], ...
    'Numerator', mat2str([1 - alpha_f]), ...
    'Denominator', mat2str([1, -alpha_f]), ...
    'SampleTime', num2str(Ts), ...
    'Position', [420, 310, 490, 340], ...
    'Orientation', 'left');

%% ===== Data Logging (To File - Array format: [time; signal]) =====
add_file_log(mdl, 'Log_Ref',       'PI_ref.mat',        Ts, [30, 25, 110, 50],    'right');
add_file_log(mdl, 'Log_Error',     'PI_error.mat',      Ts, [195, 25, 275, 50],   'right');
add_file_log(mdl, 'Log_CmdRaw',    'PI_cmd_raw.mat',    Ts, [330, 25, 415, 50],   'right');
add_file_log(mdl, 'Log_Cmd',       'PI_cmd.mat',        Ts, [480, 25, 560, 50],   'right');
add_file_log(mdl, 'Log_SpeedRaw',  'PI_speed_raw.mat',  Ts, [845, 237, 925, 263], 'right');
add_file_log(mdl, 'Log_SpeedFilt', 'PI_speed_filt.mat', Ts, [260, 312, 345, 338], 'left');

%% ===== Scopes =====
add_block('simulink/Sinks/Scope', [mdl '/Speed_Scope']);
set_param([mdl '/Speed_Scope'], 'Position', [845, 295, 885, 335]);

add_block('simulink/Sinks/Scope', [mdl '/Cmd_Scope']);
set_param([mdl '/Cmd_Scope'], 'Position', [580, 155, 620, 195]);

%% ===== Connections =====
add_line(mdl, 'Speed_Ref/1',            'Error_Sum/1');
add_line(mdl, 'Error_Sum/1',            'Discrete_PI/1');
add_line(mdl, 'Discrete_PI/1',          'Dist_Sum/1');
add_line(mdl, 'Disturbance/1',          'Dist_Sum/2');
add_line(mdl, 'Dist_Sum/1',             'Voltage_Sat/1');
add_line(mdl, 'Voltage_Sat/1',          'PLACEHOLDER_HIL_WRITE/1');

add_line(mdl, 'PLACEHOLDER_HIL_READ/1', 'Count_Diff/1');
add_line(mdl, 'PLACEHOLDER_HIL_READ/1', 'Count_Delay/1', 'autorouting', 'on');
add_line(mdl, 'Count_Delay/1',          'Count_Diff/2',  'autorouting', 'on');
add_line(mdl, 'Count_Diff/1',           'Enc_to_RadPerSec/1');
add_line(mdl, 'Enc_to_RadPerSec/1',     'Speed_Filter/1', 'autorouting', 'on');
add_line(mdl, 'Speed_Filter/1',         'Error_Sum/2', 'autorouting', 'on');

add_line(mdl, 'Speed_Ref/1',            'Log_Ref/1');
add_line(mdl, 'Error_Sum/1',            'Log_Error/1');
add_line(mdl, 'Dist_Sum/1',             'Log_CmdRaw/1', 'autorouting', 'on');
add_line(mdl, 'Voltage_Sat/1',          'Log_Cmd/1');
add_line(mdl, 'Enc_to_RadPerSec/1',     'Log_SpeedRaw/1');
add_line(mdl, 'Speed_Filter/1',         'Log_SpeedFilt/1', 'autorouting', 'on');

add_line(mdl, 'Speed_Filter/1',         'Speed_Scope/1', 'autorouting', 'on');
add_line(mdl, 'Voltage_Sat/1',          'Cmd_Scope/1');

save_system(mdl);
fprintf('\n[OK] Model 4 saved: %s.slx (Kp = %.4f, Ki = %.4f, N_avg = %d)\n', mdl, Kp, Ki, N_avg);
fprintf('   1) Replace PLACEHOLDER_HIL_WRITE / PLACEHOLDER_HIL_READ with your QUARC blocks.\n');
fprintf('   2) Run Monitor & Tune and let it STOP BY ITSELF at 10 s.\n');
fprintf('   3) Logged files (current folder): PI_ref, PI_error, PI_cmd_raw, PI_cmd, PI_speed_raw, PI_speed_filt (.mat)\n');
fprintf('   4) Verify with: check_logged_data(''PI'')\n\n');

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
