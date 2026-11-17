function ok = check_logged_data(which_ctrl, folder, make_plot)
%CHECK_LOGGED_DATA  Verify that the To File logs of Model 3 (P) / Model 4 (PI)
%  were really written and contain a full, sensible 10 s run.
%
%  Usage (run AFTER a Monitor & Tune run that stopped by itself at 10 s):
%     check_logged_data            % checks both P and PI
%     check_logged_data('P')       % only Model 3
%     check_logged_data('PI')      % only Model 4
%     check_logged_data('P', 'C:\my\folder', false)   % custom folder, no plot
%
%  Returns true if every hard check passed. Warnings do not fail the test.

if nargin < 1 || isempty(which_ctrl), which_ctrl = {'P','PI'}; end
if nargin < 2 || isempty(folder),     folder = pwd;            end
if nargin < 3,                        make_plot = true;        end
if ischar(which_ctrl) || isstring(which_ctrl), which_ctrl = cellstr(which_ctrl); end

cfg.Ts = 0.002;  cfg.Tstop = 10;  cfg.Vmax = 10;
cfg.Kp_only = 0.2231;      % edit if you change Kp_only in parameters.m
cfg.ref_final = 50;  cfg.t_step = 1;  cfg.t_dist = 5;

ok = true;
for c = 1:numel(which_ctrl)
    ok = check_one(upper(which_ctrl{c}), folder, cfg, make_plot) && ok;
end
fprintf('\n=========================================\n');
if ok, fprintf(' OVERALL: PASS - logged data looks valid\n');
else,  fprintf(' OVERALL: FAIL - see [FAIL] lines above\n'); end
fprintf('=========================================\n');
end

% ------------------------------------------------------------------------
function ok = check_one(ctrl, folder, cfg, make_plot)
switch ctrl
    case 'P',  sigs = {'ref','error','cmd','speed_raw','speed_filt'};
    case 'PI', sigs = {'ref','error','cmd_raw','cmd','speed_raw','speed_filt'};
    otherwise, error('Controller must be ''P'' or ''PI''.');
end
fprintf('\n================ %s controller ================\n', ctrl);
fprintf('Folder: %s\n', folder);

nfail = 0;  D = struct();  allLoaded = true;
Nexp = round(cfg.Tstop / cfg.Ts) + 1;

%% ---- 1. file / variable / shape checks ----
for k = 1:numel(sigs)
    s = sigs{k};
    fname = fullfile(folder, sprintf('%s_%s.mat', ctrl, s));
    vname = sprintf('%s_%s', ctrl, s);

    if ~isfile(fname)
        nfail = nfail + fail('%-22s file NOT found', [vname '.mat']);
        allLoaded = false;  continue
    end
    info = dir(fname);
    S = load(fname);
    if isfield(S, vname),  M = S.(vname);
    else, fn = fieldnames(S);  M = S.(fn{1});
          fprintf('  [WARN] %s: expected variable "%s", using "%s"\n', vname, vname, fn{1}); end

    if ~isnumeric(M) || size(M,1) ~= 2 || size(M,2) < 2
        nfail = nfail + fail('%-22s wrong shape %s (need 2 x N, N>1)', [vname '.mat'], mat2str(size(M)));
        allLoaded = false;  continue
    end
    t = M(1,:);  y = M(2,:);
    D.(s).t = t;  D.(s).y = y;

    bad = ~all(isfinite(M(:)));
    full = size(M,2) >= 0.95 * Nexp;
    if bad
        nfail = nfail + fail('%-22s contains NaN/Inf', [vname '.mat']);
    elseif ~full
        nfail = nfail + fail('%-22s only %d samples (expected ~%d) - run stopped early / not clean stop?', ...
                             [vname '.mat'], size(M,2), Nexp);
    else
        pass('%-22s OK  (%d samples, t = %.3f..%.3f s, %.0f KB)', ...
             [vname '.mat'], size(M,2), t(1), t(end), info.bytes/1024);
    end
    age_min = (now - info.datenum) * 24 * 60;
    if age_min > 60
        fprintf('  [WARN] %s is %.0f min old - is it from your latest run?\n', [vname '.mat'], age_min);
    end
end

if ~allLoaded
    fprintf('  -> Skipping content checks (some files missing/invalid).\n');
    ok = false;  return
end

%% ---- 2. time-base checks ----
t = D.ref.t;
if abs(t(1)) > 1e-9,                 nfail = nfail + fail('time does not start at 0 (t0 = %g)', t(1));
else, pass('time starts at 0'); end
dt = diff(t);
if any(dt <= 0),                     nfail = nfail + fail('time not strictly increasing');
elseif max(abs(dt - cfg.Ts)) > 1e-5, nfail = nfail + fail('step not Ts = %g s (max dev %g)', cfg.Ts, max(abs(dt-cfg.Ts)));
else, pass('time step = %.4f s (no dropped samples)', cfg.Ts); end
if t(end) < cfg.Tstop - 0.05,        nfail = nfail + fail('run ended at %.3f s, not %g s', t(end), cfg.Tstop);
else, pass('run length %.3f s', t(end)); end
same = all(cellfun(@(s) numel(D.(s).t) == numel(t), sigs));
if ~same,                            nfail = nfail + fail('signals have different lengths');
else, pass('all %d signals same length (%d samples)', numel(sigs), numel(t)); end

%% ---- 3. content checks ----
ref = D.ref.y;  w = D.speed_filt.y;  wr = D.speed_raw.y;  e = D.error.y;  u = D.cmd.y;
pre = t < cfg.t_step - 0.01;   post = t > cfg.t_step + 0.01;
if max(abs(ref(pre))) < 1e-9 && max(abs(ref(post) - cfg.ref_final)) < 1e-6
    pass('reference: 0 before t=%g s, %g rad/s after', cfg.t_step, cfg.ref_final);
else, nfail = nfail + fail('reference is not the expected 0 -> %g step at t=%g s', cfg.ref_final, cfg.t_step); end

if std(w(t > cfg.t_step + 0.2)) < 0.05 && abs(mean(w(t > cfg.t_step + 0.2))) < 0.5
    nfail = nfail + fail('filtered speed is flat ~0 - encoder not read / motor not moving');
else, pass('filtered speed responds (max %.2f rad/s)', max(w)); end
if all(wr == 0), nfail = nfail + fail('raw speed is all zeros');
else, pass('raw speed non-zero (max %.2f rad/s)', max(wr)); end

if max(abs(u)) > cfg.Vmax + 1e-6, nfail = nfail + fail('command exceeds +/-%g V (max %.3f)', cfg.Vmax, max(abs(u)));
else, pass('command within +/-%g V (peak %.3f V)', cfg.Vmax, max(abs(u))); end

%% ---- 4. consistency checks (warnings only) ----
if max(abs(e - (ref - w))) > 1e-3
    fprintf('  [WARN] error != ref - speed_filt (max dev %.4g) - check logging taps\n', max(abs(e-(ref-w))));
else, pass('error = ref - speed_filt'); end

if strcmp(ctrl, 'P')
    uexp = max(min(cfg.Kp_only * e, cfg.Vmax), -cfg.Vmax);
    if max(abs(u - uexp)) > 1e-3
        fprintf('  [WARN] cmd != sat(Kp*error), max dev %.4g (Kp_only in cfg = %g)\n', max(abs(u-uexp)), cfg.Kp_only);
    else, pass('cmd = sat(Kp_only*error)  (Kp_only = %g)', cfg.Kp_only); end
else
    uexp = max(min(D.cmd_raw.y, cfg.Vmax), -cfg.Vmax);
    if max(abs(u - uexp)) > 1e-3
        fprintf('  [WARN] cmd != sat(cmd_raw), max dev %.4g\n', max(abs(u-uexp)));
    else, pass('cmd = sat(cmd_raw)'); end
end

%% ---- 5. quick performance summary (info only) ----
a = t > cfg.t_dist - 0.5 & t < cfg.t_dist;   b = t > cfg.Tstop - 0.5;
fprintf('  Info: speed before disturbance = %.2f rad/s | at end = %.2f rad/s (ref %g)\n', ...
        mean(w(a)), mean(w(b)), cfg.ref_final);
fprintf('  Info: steady-state error before disturbance = %.2f rad/s, at end = %.2f rad/s\n', ...
        mean(e(a)), mean(e(b)));

ss = t > cfg.t_step + 1;
fprintf('  Info: speed noise (std after t=%g s): raw = %.2f rad/s | filtered = %.2f rad/s | cmd = %.3f V\n', ...
        cfg.t_step + 1, std(wr(ss)), std(w(ss)), std(u(ss)));

ok = (nfail == 0);
if ok, fprintf('  >> %s: PASS\n', ctrl); else, fprintf('  >> %s: FAIL (%d hard check(s))\n', ctrl, nfail); end

%% ---- 6. plot ----
if make_plot
    try
        figure('Name', [ctrl ' logged data'], 'Color', 'w');
        subplot(2,1,1);
        plot(t, ref, 'k--', t, wr, ':', t, w, 'LineWidth', 1.2); grid on
        xlabel('t [s]'); ylabel('\omega [rad/s]'); legend('ref','raw','filtered','Location','best');
        title(sprintf('%s controller - speed', ctrl));
        subplot(2,1,2);
        plot(t, u, 'LineWidth', 1.2); grid on
        xlabel('t [s]'); ylabel('u [V]'); title('command');
    catch ME
        fprintf('  [WARN] Plot skipped: %s\n  (Check "which -all plot" for a file shadowing the built-in.)\n', ME.message);
    end
end
end

% ------------------------------------------------------------------------
function pass(varargin), fprintf('  [PASS] %s\n', sprintf(varargin{:})); end
function n = fail(varargin), fprintf('  [FAIL] %s\n', sprintf(varargin{:})); n = 1; end
