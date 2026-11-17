%% identify_qube_hw.m
%  Identifies K_hw and tau_hw from step_3V.mat, step_4V.mat, step_5v.mat,
%  validates on staircase.mat, and computes P / PI gains matching parameters.m.

clear; clc; close all;

mat_files = dir('*.mat');
assert(~isempty(mat_files), 'No .mat files found in current directory.');

step_results = [];
stair_data   = [];

for k = 1:length(mat_files)
    fname = mat_files(k).name;
    raw   = load(fname);

    assert(isfield(raw, 't') && isfield(raw, 'u') && isfield(raw, 'w_meas'), ...
        'File %s is missing t, u, or w_meas.', fname);

    t = double(raw.t(:));
    u = double(raw.u(:));
    w = double(raw.w_meas(:));

    [t, idx_u] = unique(t, 'stable');
    u = u(idx_u);
    w = w(idx_u);

    if contains(lower(fname), 'stair')
        stair_data.name = fname;
        stair_data.t    = t;
        stair_data.u    = u;
        stair_data.w    = w;
    else
        res = fit_step_trial(t, u, w, fname);
        step_results = [step_results; res]; %#ok<AGROW>
    end
end

%% ===== Summary Table =====
K_avg   = mean([step_results.K_ls]);
tau_avg = mean([step_results.tau_ls]);

fprintf('\n====================================================================\n');
fprintf('  QUBE-SERVO 3 IDENTIFICATION SUMMARY (from w_meas & u)\n');
fprintf('====================================================================\n');
fprintf('%-16s | %7s | %10s | %10s | %10s | %9s\n', ...
    'File', 'dV (V)', 'K_63.2%', 'K_LS', 'tau_LS (s)', 'RMSE');
fprintf('--------------------------------------------------------------------\n');
for i = 1:length(step_results)
    r = step_results(i);
    fprintf('%-16s | %7.2f | %10.4f | %10.4f | %10.4f | %9.4f\n', ...
        r.name, r.dV, r.K_63, r.K_ls, r.tau_ls, r.rmse);
end
fprintf('--------------------------------------------------------------------\n');
fprintf('AVERAGE MODEL:   K_hw = %.4f (rad/s)/V,   tau_hw = %.4f s\n', K_avg, tau_avg);

%% ===== Validation on Staircase Input =====
if ~isempty(stair_data)
    t_s = stair_data.t;
    u_s = stair_data.u;
    w_s = stair_data.w;

    w_pred_s = zeros(size(t_s));
    w_pred_s(1) = w_s(1);
    for n = 1:length(t_s)-1
        dt = max(t_s(n+1) - t_s(n), 1e-6);
        a  = exp(-dt / tau_avg);
        b  = K_avg * (1 - a);
        w_pred_s(n+1) = a * w_pred_s(n) + b * u_s(n);
    end
    rmse_val = sqrt(mean((w_s - w_pred_s).^2));
    fprintf('STAIRCASE VALIDATION (%s): RMSE = %.4f rad/s\n', stair_data.name, rmse_val);
end

%% ===== Candidate P & PI Gains (Matching parameters.m: zeta=0.80, wn=40) =====
zeta    = 0.80;
wn      = 40.0;
Kp_only = (2*zeta*wn*tau_avg - 1) / K_avg;
Kp_pi   = (2*zeta*wn*tau_avg - 1) / K_avg;
Ki_pi   = (tau_avg * wn^2) / K_avg;

fprintf('--------------------------------------------------------------------\n');
fprintf('COPY THESE INTO parameters.m:\n');
fprintf('K_dc    = %.4f;\n', K_avg);
fprintf('tau     = %.4f;\n', tau_avg);
fprintf('Kp_only = %.4f;\n', Kp_only);
fprintf('Kp      = %.4f;\n', Kp_pi);
fprintf('Ki      = %.4f;\n', Ki_pi);
fprintf('====================================================================\n\n');

%% ===== Plots (Figure F3) =====
figure('Name', 'Stage 2: Identification & Validation', 'Position', [100, 100, 1050, 420]);

subplot(1, 2, 1); hold on; grid on;
colors = lines(length(step_results));
for i = 1:length(step_results)
    r = step_results(i);
    plot(r.t, r.w, 'Color', colors(i,:), 'LineWidth', 1.1, ...
        'DisplayName', sprintf('%s (Meas)', r.name));
    plot(r.t, r.w_pred, '--', 'Color', colors(i,:), 'LineWidth', 1.7, ...
        'DisplayName', sprintf('%s (Fit: K=%.2f, \\tau=%.4fs)', r.name, r.K_ls, r.tau_ls));
end
xlabel('Time (s)'); ylabel('Speed \omega (rad/s)');
title('Step Responses & 1st-Order Exponential Fits');
legend('Location', 'best');

subplot(1, 2, 2); hold on; grid on;
if ~isempty(stair_data)
    plot(t_s, w_s, 'b', 'LineWidth', 1.2, 'DisplayName', 'Measured Speed');
    plot(t_s, w_pred_s, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Identified Model');
    xlabel('Time (s)'); ylabel('Speed \omega (rad/s)');
    title(sprintf('Staircase Validation (RMSE = %.3f rad/s)', rmse_val));
    legend('Location', 'best');
end

function res = fit_step_trial(t, u, w, fname)
    u_init   = mean(u(1:max(1, floor(0.05*length(u)))));
    idx_step = find(abs(u - u_init) > 0.1, 1, 'first');
    assert(~isempty(idx_step), 'No voltage step detected in %s.', fname);

    t_step = t(idx_step);
    t_ss_start = t_step + 0.70 * (t(end) - t_step);

    u_ss = mean(u(t >= t_ss_start));
    dV   = u_ss - u_init;

    w_init = mean(w(t < t_step));
    w_ss   = mean(w(t >= t_ss_start));
    dw     = w_ss - w_init;

    K_63 = dw / dV;
    w_63 = w_init + 0.632 * dw;
    if dw >= 0
        idx_63 = find(t >= t_step & w >= w_63, 1, 'first');
    else
        idx_63 = find(t >= t_step & w <= w_63, 1, 'first');
    end
    tau_63 = max(t(idx_63) - t_step, 1e-3);

    % Normalized least-squares fit (prevents scale mismatch between K ~ 20 and tau ~ 0.07)
    idx_fit = t >= t_step;
    t_rel   = t(idx_fit) - t_step;
    w_meas  = w(idx_fit);

    K0   = max(abs(K_63), 1.0) * sign(K_63);
    tau0 = max(tau_63, 0.01);

    cost_fn = @(p_norm) mean((w_meas - (w_init + (p_norm(1)*K0)*dV * ...
        (1 - exp(-t_rel / max(p_norm(2)*tau0, 1e-4))))).^2);
    opts   = optimset('TolX', 1e-8, 'TolFun', 1e-8, 'MaxFunEvals', 2000);
    p_best = fminsearch(cost_fn, [1.0, 1.0], opts);

    res.name   = fname;
    res.t      = t;
    res.w      = w;
    res.dV     = dV;
    res.K_63   = K_63;
    res.tau_63 = tau_63;
    res.K_ls   = p_best(1) * K0;
    res.tau_ls = p_best(2) * tau0;
    res.w_pred = w_init * ones(size(t));
    res.w_pred(idx_fit) = w_init + res.K_ls * dV * (1 - exp(-t_rel / res.tau_ls));
    res.rmse   = sqrt(mean((w - res.w_pred).^2));
end