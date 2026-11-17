%% EE302 PS5 - Stage 02: Identify and validate speed model USING SIMULATED DATA
% MATLAB R2024a + Control System Toolbox. NO QUARC, hardware, or hardware
% parameters are used. Run this file from any working directory.
%
% Identification: simulate full electrical-mechanical plant with FOUR
% separate voltage steps, jointly fit K and tau of reduced K/(tau*s+1).
% Validation: a NEW ramp-then-hold input and a NEW multi-step input.
% Export every raw signal, prediction, error metric, figure, and gain file.

clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(thisDir);
outDir = fullfile(projectDir,'results','02_identification');
if ~exist(outDir,'dir'), mkdir(outDir); end
run(fullfile(thisDir,'parameters.m'));  % stage-2 simulation baseline ONLY
plant_full = tf(num_full,den_full);
plant_reduced_theory = tf(K_dc,[tau,1]);

%% 1. SIMULATED TRAINING DATA: four independent voltage steps
% All trials reset to rest. The step occurs at 0.1 s. No hardware results.
t = (0:Ts:1.0)';
t_step = 0.10;
U_train = [1,2,3,4]; % V (design inputs used ONLY in software)
nSteps = numel(U_train);
train_input = zeros(numel(t),nSteps);
train_speed = zeros(numel(t),nSteps);
for j=1:nSteps
    train_input(:,j) = U_train(j)*double(t>=t_step);
    train_speed(:,j) = lsim(plant_full,train_input(:,j),t);
end

%% 2. IDENTIFY first-order K and tau by constrained-positive joint least squares
% FOP1 step response: omega(t)=K*U*(1-exp(-(t-t_step)/tau)), t>=t_step.
% Fitting log(K),log(tau) enforces K>0,tau>0 without extra toolboxes.
x0 = log([K_dc,tau]);
options = optimset('Display','off','TolX',1e-12,'TolFun',1e-12, ...
                   'MaxIter',2000,'MaxFunEvals',4000);
[xOpt,fitSSE,exitflag] = fminsearch( ...
    @(x) step_fit_cost(x,t,t_step,U_train,train_speed),x0,options);
if exitflag<=0
    warning('fminsearch did not report convergence (exitflag=%d). Inspect fit.',exitflag);
end
K_sim_id = exp(xOpt(1));
tau_sim_id = exp(xOpt(2));
plant_identified = tf(K_sim_id,[tau_sim_id,1]);
train_predicted = step_model(xOpt,t,t_step,U_train);
train_error = train_speed-train_predicted;

%% 3. INDEPENDENT VALIDATION 1: linearly increasing ramp then constant hold
% Starts ramp at 0.3 s; rises at 2 V/s and saturates at 3 V at 1.8 s.
tVal = (0:Ts:3.0)';
uRamp = min(3,max(0,2*(tVal-0.3)));
wRampTrue = lsim(plant_full,uRamp,tVal);
wRampPred = lsim(plant_identified,uRamp,tVal);
errRamp = wRampTrue-wRampPred;

%% 4. INDEPENDENT VALIDATION 2: multi-step input NOT used for fitting
uSequence = zeros(size(tVal));
uSequence(tVal>=0.2) = 2.0;
uSequence(tVal>=0.8) = -1.0;
uSequence(tVal>=1.4) = 3.0;
uSequence(tVal>=2.1) = 0.0;
uSequence(tVal>=2.5) = 1.5;
wSeqTrue = lsim(plant_full,uSequence,tVal);
wSeqPred = lsim(plant_identified,uSequence,tVal);
errSequence = wSeqTrue-wSeqPred;

%% 5. Quantitative prediction error on completely unseen voltage inputs
rmseRamp = sqrt(mean(errRamp.^2));
rmseSequence = sqrt(mean(errSequence.^2));
maeRamp = mean(abs(errRamp));
maeSequence = mean(abs(errSequence));
maxErrRamp = max(abs(errRamp));
maxErrSequence = max(abs(errSequence));
nrmseRamp = 100*rmseRamp/max(max(wRampTrue)-min(wRampTrue),eps);
nrmseSequence = 100*rmseSequence/max(max(wSeqTrue)-min(wSeqTrue),eps);
r2Ramp = 1-sum(errRamp.^2)/max(sum((wRampTrue-mean(wRampTrue)).^2),eps);
r2Sequence = 1-sum(errSequence.^2)/max(sum((wSeqTrue-mean(wSeqTrue)).^2),eps);
validation_metrics = table({'Ramp then hold';'Unseen multi-step'}, ...
    [rmseRamp;rmseSequence],[maeRamp;maeSequence], ...
    [maxErrRamp;maxErrSequence],[nrmseRamp;nrmseSequence], ...
    [r2Ramp;r2Sequence], ...
    'VariableNames',{'Input','RMSE_rad_s','MAE_rad_s','MaxAbsError_rad_s', ...
                     'NRMSE_percent_of_range','R_squared'});

%% 6. Recompute controller gains from SIMULATION-IDENTIFIED, NOT HARDWARE data
% Match the existing design targets and compare equal P/PI proportional gains.
Kp_sim_id = (2*zeta*wn*tau_sim_id-1)/K_sim_id;
Ki_sim_id = wn^2*tau_sim_id/K_sim_id;
Kp_only_sim_id = Kp_sim_id;
assert(Kp_sim_id>0 && Ki_sim_id>0,'Computed gains must be positive');
fit_summary = table(K_dc,tau,Kp_only,Kp,Ki, ...
                    K_sim_id,tau_sim_id,Kp_only_sim_id,Kp_sim_id,Ki_sim_id, ...
    'VariableNames',{'K_theory','tau_theory_s','Kp_only_theory', ...
                     'Kp_theory','Ki_theory','K_sim_identified', ...
                     'tau_sim_identified_s','Kp_only_sim_identified', ...
                     'Kp_sim_identified','Ki_sim_identified'});

%% 7. Save MATLAB datasets and portable CSVs
writetable(validation_metrics,fullfile(outDir,'04_validation_metrics.csv'));
writetable(fit_summary,fullfile(outDir,'05_parameter_and_gain_summary.csv'));
for j=1:nSteps
    writematrix([t,train_input(:,j),train_speed(:,j), ...
        train_predicted(:,j),train_error(:,j)], ...
        fullfile(outDir,sprintf('01_training_step_%dV.csv',U_train(j))));
end
writematrix([tVal,uRamp,wRampTrue,wRampPred,errRamp], ...
    fullfile(outDir,'02_validation_ramp.csv'));
writematrix([tVal,uSequence,wSeqTrue,wSeqPred,errSequence], ...
    fullfile(outDir,'03_validation_unseen_multistep.csv'));
save(fullfile(outDir,'06_stage02_identification.mat'), ...
    'plant_full','plant_reduced_theory','plant_identified', ...
    't','t_step','U_train','train_input','train_speed', ...
    'train_predicted','train_error','fitSSE','exitflag', ...
    'tVal','uRamp','wRampTrue','wRampPred','errRamp', ...
    'uSequence','wSeqTrue','wSeqPred','errSequence', ...
    'K_dc','tau','K_sim_id','tau_sim_id', ...
    'Kp','Ki','Kp_only','Kp_sim_id','Ki_sim_id', ...
    'Kp_only_sim_id','Ts','N_enc','alpha_f', ...
    'validation_metrics','fit_summary');

%% 8. Figures (saved as PNG for report and FIG for later editing)
fig = figure('Color','w','Visible','off','Position',[50,50,1080,760]);
for j=1:nSteps
    subplot(2,2,j);
    plot(t,train_speed(:,j),'LineWidth',1.4); hold on;
    plot(t,train_predicted(:,j),'--','LineWidth',1.3);
    grid on; xlabel('Time (s)'); ylabel('Speed (rad/s)');
    title(sprintf('Training: %g V step',U_train(j)));
    if j==1, legend('Full simulated plant','Identified 1st order','Location','southeast'); end
end
save_figure(fig,outDir,'10_training_steps');

fig = figure('Color','w','Visible','off','Position',[50,50,1050,700]);
subplot(3,1,1); plot(tVal,uRamp,'LineWidth',1.5); grid on;
ylabel('Voltage (V)'); title('Independent validation: unseen ramp-then-hold');
subplot(3,1,2); plot(tVal,wRampTrue,'LineWidth',1.5); hold on;
plot(tVal,wRampPred,'--','LineWidth',1.4); grid on;
ylabel('Speed (rad/s)'); legend('Full plant','Identified model','Location','southeast');
subplot(3,1,3); plot(tVal,errRamp,'LineWidth',1.3); grid on;
xlabel('Time (s)'); ylabel('Error (rad/s)');
save_figure(fig,outDir,'11_validation_ramp');

fig = figure('Color','w','Visible','off','Position',[50,50,1050,700]);
subplot(3,1,1); plot(tVal,uSequence,'LineWidth',1.5); grid on;
ylabel('Voltage (V)'); title('Independent validation: unseen multi-step sequence');
subplot(3,1,2); plot(tVal,wSeqTrue,'LineWidth',1.5); hold on;
plot(tVal,wSeqPred,'--','LineWidth',1.4); grid on;
ylabel('Speed (rad/s)'); legend('Full plant','Identified model','Location','southeast');
subplot(3,1,3); plot(tVal,errSequence,'LineWidth',1.3); grid on;
xlabel('Time (s)'); ylabel('Error (rad/s)');
save_figure(fig,outDir,'12_validation_unseen_multistep');

%% 9. Write separately named parameters for FUTURE controller simulation
% Do NOT overwrite parameters.m or the user's hardware identification file.
generatedFile = fullfile(thisDir,'parameters_sim_identified.m');
fid=fopen(generatedFile,'w');
assert(fid>0,'Cannot write identified simulation parameter script');
fprintf(fid,'%%%% PS5 SIMULATION-IDENTIFIED parameters - generated by Stage 02\n');
fprintf(fid,'%% NOT hardware-identified! Data source: simulated full-order plant.\n');
fprintf(fid,'%% Safe as a standalone parameter script (no clear/clc).\n');
fprintf(fid,'R = %.16g; L = %.16g; Kt = %.16g; Ke = %.16g;\n',R,L,Kt,Ke);
fprintf(fid,'Jm = %.16g; Jh = %.16g; md = %.16g; rd = %.16g;\n',Jm,Jh,md,rd);
fprintf(fid,'Jdisk = 0.5*md*rd^2; Jeq = Jm+Jh+Jdisk;\n');
fprintf(fid,'beq = %.16g; %% assumed, not provided by datasheet\n',beq);
fprintf(fid,'num_full = Kt; den_full = [L*Jeq, L*beq+R*Jeq, R*beq+Kt*Ke];\n');
fprintf(fid,'Ts = %.16g; Vmax = %.16g; Vmin = %.16g;\n',Ts,Vmax,Vmin);
fprintf(fid,'N_enc = %d; N_avg = %d;\n',N_enc,N_avg);
fprintf(fid,'tau_f = %.16g; alpha_f = tau_f/(tau_f+Ts);\n',tau_f);
fprintf(fid,'zeta = %.16g; wn = %.16g;\n',zeta,wn);
fprintf(fid,'K_dc = %.16g; %% fitted from simulated full-order voltage steps\n',K_sim_id);
fprintf(fid,'tau = %.16g; %% fitted from simulated full-order voltage steps (s)\n',tau_sim_id);
fprintf(fid,'Kp = (2*zeta*wn*tau-1)/K_dc;\n');
fprintf(fid,'Ki = wn^2*tau/K_dc;\n');
fprintf(fid,'Kp_only = Kp; %% same P vs PI comparison convention as hardware\n');
fclose(fid);

fid=fopen(fullfile(outDir,'07_stage02_run_record.txt'),'w');
assert(fid>0);
fprintf(fid,'MATLAB release: %s\nDate: %s\n',version('-release'),datestr(now,31));
fprintf(fid,'Data source: simulated full 2-state manufacturer model, b=0.\n');
fprintf(fid,'K_theory=%.12g; tau_theory=%.12g s\n',K_dc,tau);
fprintf(fid,'K_sim_identified=%.12g; tau_sim_identified=%.12g s\n',K_sim_id,tau_sim_id);
fprintf(fid,'Kp_sim=%.12g; Ki_sim=%.12g\n',Kp_sim_id,Ki_sim_id);
fprintf(fid,'Validation RMSE ramp=%.12g rad/s, multi-step=%.12g rad/s\n', ...
    rmseRamp,rmseSequence);
fprintf(fid,'No measured hardware data was used.\n');
fclose(fid);

fprintf('\nSTAGE 02 COMPLETE: SIMULATED IDENTIFICATION/VALIDATION\n');
fprintf('Theoretical: K=%.8f rad/s/V, tau=%.8f s\n',K_dc,tau);
fprintf('Sim-identified: K=%.8f rad/s/V, tau=%.8f s\n',K_sim_id,tau_sim_id);
fprintf('Sim-identified gains: Kp_only=%.6f, Kp=%.6f, Ki=%.6f\n', ...
    Kp_only_sim_id,Kp_sim_id,Ki_sim_id);
disp(validation_metrics);
fprintf('Saved all signals/figures: %s\n',outDir);
fprintf('Generated simulation-only parameters: %s\n',generatedFile);

%% Script-local helper functions (supported by MATLAB R2024a)
function Yhat = step_model(x,t,t_step,U)
    K=exp(x(1)); tau=exp(x(2));
    dt=max(t-t_step,0);
    Yhat=(K*(1-exp(-dt/tau)))*U;
end
function J = step_fit_cost(x,t,t_step,U,Y)
    E=Y-step_model(x,t,t_step,U);
    J=sum(E(:).^2);
end
function save_figure(f,outDir,fileStem)
    exportgraphics(f,fullfile(outDir,[fileStem '.png']),'Resolution',180);
    savefig(f,fullfile(outDir,[fileStem '.fig']));
    close(f);
end
