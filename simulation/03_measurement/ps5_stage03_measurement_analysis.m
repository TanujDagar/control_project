%% EE302 PS5 - Stage 03: Discrete plant and encoder measurement analysis
% MATLAB R2024a, Control System Toolbox. No measured data or QUARC needed.
% Uses Stage-2 simulation-identified gains ONLY as metadata for future stages.
% Reproduces EXACT supplied hardware encoder measurement chain:
%   count[k] = round(theta[k] * N_enc/(2*pi))
%   raw[k]   = (count[k] - count[k-N_avg])*2*pi/(N_enc*N_avg*Ts)
%   filt[k]  = (1-alpha_f)*raw[k] + alpha_f*filt[k-1]
% Stage 3 is OPEN LOOP. It does not apply P/PI controller gains.
clear; clc; close all;
thisDir = fileparts(mfilename('fullpath'));
projectDir = fileparts(thisDir);
outDir = fullfile(projectDir,'results','03_measurement');
if ~exist(outDir,'dir'), mkdir(outDir); end
baseParams = fullfile(projectDir,'02_identification','parameters_sim_identified.m');
if exist(baseParams,'file')
    run(baseParams);
    parameterSource = baseParams;
else
    run(fullfile(thisDir,'parameters_stage03_snapshot.m'));
    parameterSource = 'Stage-2 transcript snapshot, see notes';
    warning(['Stage-2 generated parameter file not found. ', ...
             'Using accurately transcribed Stage-2 values; copy full Stage-2 ', ...
             'folder alongside 03_measurement for direct provenance.']);
end
assert(N_enc==2048 && N_avg==4 && abs(Ts-.002)<1e-10, ...
    'Unexpected hardware-parity sample/encoder settings.');

%% 1) Manufacturer 3-state full model: x=[current(A); speed(rad/s); angle(rad)]
% No viscous friction parameter is provided by the datasheet: beq=0 baseline.
A = [-R/L, -Ke/L, 0; Kt/Jeq, -beq/Jeq, 0; 0, 1, 0];
B = [1/L;0;0];
C = [0,1,0; 0,0,1]; % [speed; angle]
D = [0;0];
plant_ct = ss(A,B,C,D);
plant_dt = c2d(plant_ct,Ts,'zoh');
Gspeed_ct = tf(Kt,[L*Jeq,L*beq+R*Jeq,R*beq+Kt*Ke]);
Gspeed_dt = c2d(Gspeed_ct,Ts,'zoh');
poles_ct = pole(Gspeed_ct);
poles_dt = pole(Gspeed_dt);
angle_integrator_pole = 1; % Additional z=1 pole in voltage-to-angle model

%% 2) Exact ZOH sampled response to a 2 V step at 0.1s
Tstop = 1.5;
t = (0:Ts:Tstop)';
u = 2*double(t>=0.1);
y = lsim(plant_dt,u,t);
w_ideal = y(:,1);
theta = y(:,2);
[counts,w_raw,w_filt,w_windowed] = measure_angle(theta,Ts,N_enc,N_avg,alpha_f);
% w_windowed is ideal N-sample average, NOT instantaneous ideal speed.
% The difference w_raw-w_windowed isolates encoder quantization from lag.
valid = t >= 0.1 + N_avg*Ts;
steady = t >= 0.85 & t <= Tstop;
metric_names = {'Quantisation wrt windowed';'Raw wrt instantaneous'; ...
                'Filtered wrt instantaneous'};
metric_rmse = [sqrt(mean((w_raw(valid)-w_windowed(valid)).^2)); ...
               sqrt(mean((w_raw(valid)-w_ideal(valid)).^2)); ...
               sqrt(mean((w_filt(valid)-w_ideal(valid)).^2))];
metric_maxerr = [max(abs(w_raw(valid)-w_windowed(valid))); ...
                 max(abs(w_raw(valid)-w_ideal(valid))); ...
                 max(abs(w_filt(valid)-w_ideal(valid)))];
measurement_metrics = table(metric_names,metric_rmse,metric_maxerr, ...
    'VariableNames',{'Comparison','RMSE_rad_s','MaxAbsError_rad_s'});
steady_ripple_raw = max(w_raw(steady))-min(w_raw(steady));
steady_ripple_filt = max(w_filt(steady))-min(w_filt(steady));
count_to_rad = 2*pi/N_enc;
raw_speed_quantum = count_to_rad/(N_avg*Ts);
fs = 1/Ts;

%% 3) Low-speed encoder diagnostic; ideal constant angular velocity only
% This deliberately isolates quantization from motor/amplifier deadband.
tLow = (0:Ts:1)';
lowSpeeds = [0.5,2,5]; % rad/s; synthetic kinematics, NOT real motor voltage trials
lowRaw = zeros(numel(tLow),numel(lowSpeeds));
lowFilt = lowRaw;
for j=1:numel(lowSpeeds)
    thetaLow = lowSpeeds(j)*tLow;
    [~,lowRaw(:,j),lowFilt(:,j)] = measure_angle( ...
        thetaLow,Ts,N_enc,N_avg,alpha_f);
end

%% 4) Sampling sensitivity (NOT a change to the hardware model)
TsStudy = [0.001;0.002;0.005;0.010];
nCases = numel(TsStudy);
sampleMetrics = zeros(nCases,6);
caseTraces = cell(nCases,1);
for j=1:nCases
    Tsj = TsStudy(j);
    aj = tau_f/(tau_f+Tsj);
    tj = (0:Tsj:Tstop)';
    uj = 2*double(tj>=0.1);
    Pj = c2d(plant_ct,Tsj,'zoh');
    yj = lsim(Pj,uj,tj);
    [qj,rj,fj,wj] = measure_angle(yj(:,2),Tsj,N_enc,N_avg,aj);
    win = tj>=0.1+N_avg*Tsj;
    ssj = tj>=0.85;
    sampleMetrics(j,:) = [Tsj,aj,2*pi/(N_enc*N_avg*Tsj), ...
         sqrt(mean((rj(win)-wj(win)).^2)), ...
         sqrt(mean((fj(win)-yj(win,1)).^2)), ...
         max(fj(ssj))-min(fj(ssj))];
    caseTraces{j} = struct('t',tj,'input',uj,'ideal',yj(:,1), ...
        'counts',qj,'raw',rj,'filtered',fj,'windowed',wj);
end
sample_sensitivity = array2table(sampleMetrics,'VariableNames', ...
    {'Ts_s','alpha','raw_quantum_rad_s','quantisation_RMSE_rad_s', ...
     'filter_total_RMSE_rad_s','filtered_steady_ripple_rad_s'});

%% 5) Save raw signals, matrices, scalar output and figures
writematrix([t,u,w_ideal,theta,counts,w_raw,w_windowed,w_filt, ...
            w_raw-w_windowed,w_filt-w_ideal], ...
            fullfile(outDir,'01_step_measurement_signals.csv'));
writetable(measurement_metrics,fullfile(outDir,'02_measurement_metrics.csv'));
writetable(sample_sensitivity,fullfile(outDir,'03_sampling_sensitivity.csv'));
writematrix([tLow,lowRaw,lowFilt],fullfile(outDir,'04_low_speed_diagnostic.csv'));
save(fullfile(outDir,'05_stage03_results.mat'), ...
    'plant_ct','plant_dt','Gspeed_ct','Gspeed_dt','A','B','C','D', ...
    'poles_ct','poles_dt','angle_integrator_pole', ...
    't','u','theta','w_ideal','counts','w_raw','w_windowed','w_filt', ...
    'measurement_metrics','sample_sensitivity','caseTraces','lowSpeeds', ...
    'tLow','lowRaw','lowFilt','Ts','fs','tau_f','alpha_f','N_enc','N_avg', ...
    'count_to_rad','raw_speed_quantum','steady_ripple_raw', ...
    'steady_ripple_filt','K_dc','tau','Kp_only','Kp','Ki','parameterSource');

fig=figure('Color','w','Visible','off','Position',[80 80 1100 750]);
subplot(3,1,1); plot(t,u,'LineWidth',1.3); grid on;
ylabel('Voltage (V)'); title('Stage 03: sampled motor + hardware-matched encoder chain');
subplot(3,1,2); plot(t,w_ideal,'LineWidth',1.4); hold on;
plot(t,w_raw,'LineWidth',0.85); plot(t,w_filt,'LineWidth',1.1);
grid on; ylabel('Speed (rad/s)');
legend('Ideal sampled speed','4-sample encoder speed','Filtered speed','Location','southeast');
subplot(3,1,3); plot(t,counts,'LineWidth',1.2); grid on;
xlabel('Time (s)'); ylabel('Encoder count');
save_plot(fig,outDir,'10_sampled_encoder_overview');

fig=figure('Color','w','Visible','off','Position',[80 80 1100 700]);
subplot(2,1,1); plot(t,w_ideal,'LineWidth',1.6); hold on;
plot(t,w_windowed,'--','LineWidth',1.3); plot(t,w_raw,'LineWidth',0.8);
plot(t,w_filt,'LineWidth',1.3); grid on; xlim([0.095 0.35]);
ylabel('Speed (rad/s)'); title('Transient detail: measurement delay and quantization');
legend('Instantaneous ideal','Ideal 4-sample average','Quantized 4-sample speed', ...
    'Filtered speed','Location','southeast');
subplot(2,1,2); plot(t,w_raw-w_windowed,'LineWidth',1); hold on;
plot(t,w_filt-w_ideal,'LineWidth',1.1); grid on; xlim([0.095 0.35]);
ylabel('Error (rad/s)'); xlabel('Time (s)');
legend('Quantization-only','Full measurement error incl. lag','Location','southeast');
save_plot(fig,outDir,'11_transient_measurement_error');

fig=figure('Color','w','Visible','off','Position',[80 80 1100 600]);
plot(t,w_raw-w_ideal,'LineWidth',0.8); hold on;
plot(t,w_filt-w_ideal,'LineWidth',1.3); grid on; xlim([0.85 1.5]);
ylabel('Measured minus ideal speed (rad/s)'); xlabel('Time (s)');
title('Steady-region measurement ripple (ideal motor, quantized encoder)');
legend('Raw 4-sample','Filtered','Location','best');
save_plot(fig,outDir,'12_steady_encoder_ripple');

fig=figure('Color','w','Visible','off','Position',[80 80 1100 750]);
for j=1:numel(lowSpeeds)
    subplot(numel(lowSpeeds),1,j);
    plot(tLow,lowRaw(:,j),'LineWidth',0.9); hold on;
    plot(tLow,lowFilt(:,j),'LineWidth',1.4);
    yline(lowSpeeds(j),'k--','LineWidth',1.0);
    grid on; xlim([0.15 0.5]);
    ylabel('rad/s'); title(sprintf('Synthetic constant speed = %.1f rad/s',lowSpeeds(j)));
    if j==1
       legend('4-sample raw','Filtered','Actual constant speed','Location','best');
    end
end
xlabel('Time (s)');
save_plot(fig,outDir,'13_low_speed_quantization');

fig=figure('Color','w','Visible','off','Position',[80 80 1100 650]);
subplot(2,1,1);
plot(1000*TsStudy,sampleMetrics(:,4),'o-','LineWidth',1.4); hold on;
plot(1000*TsStudy,sampleMetrics(:,5),'s-','LineWidth',1.4); grid on;
xlabel('Sampling interval (ms)'); ylabel('RMSE (rad/s)');
legend('Raw quantization vs ideal window','Filtered vs instantaneous','Location','best');
title('Sensitivity analysis: four-sample window retained for all Ts');
subplot(2,1,2);
plot(1000*TsStudy,sampleMetrics(:,3),'o-','LineWidth',1.4); grid on;
xlabel('Sampling interval (ms)'); ylabel('4-sample resolution (rad/s/count)');
save_plot(fig,outDir,'14_sampling_sensitivity');

% Frequency response of continuous plant vs sampled ZOH speed plant.
wgrid = logspace(-1,3.0,500); % rad/s, below 2 ms Nyquist 1570.8 rad/s
Hc = squeeze(freqresp(Gspeed_ct,wgrid));
Hd = squeeze(freqresp(Gspeed_dt,wgrid));
fig=figure('Color','w','Visible','off','Position',[80 80 1100 700]);
subplot(2,1,1);
semilogx(wgrid,20*log10(abs(Hc)),'LineWidth',1.4); hold on;
semilogx(wgrid,20*log10(abs(Hd)),'--','LineWidth',1.4); grid on;
ylabel('Magnitude (dB)'); title('Speed plant: continuous vs 2 ms ZOH discrete');
legend('Continuous','Discrete ZOH','Location','best');
subplot(2,1,2);
semilogx(wgrid,unwrap(angle(Hc))*180/pi,'LineWidth',1.4); hold on;
semilogx(wgrid,unwrap(angle(Hd))*180/pi,'--','LineWidth',1.4); grid on;
xlabel('Frequency (rad/s)'); ylabel('Phase (degrees)');
save_plot(fig,outDir,'15_continuous_vs_discrete_frequency');

fid=fopen(fullfile(outDir,'06_stage03_run_record.txt'),'w');
assert(fid>0,'Could not write run record');
fprintf(fid,'MATLAB release: %s\nRun date: %s\n',version('-release'),datestr(now,31));
fprintf(fid,'Parameter source: %s\n',parameterSource);
fprintf(fid,'Manufacturer full plant with b=0; identified K,tau are metadata only.\n');
fprintf(fid,'Ts = %.8g s; sample frequency = %.8g Hz\n',Ts,fs);
fprintf(fid,'N_enc = %d counts/rev; N_avg = %d; alpha_f = %.12g\n',N_enc,N_avg,alpha_f);
fprintf(fid,'Encoder quantum = %.12g rad/count\n',count_to_rad);
fprintf(fid,'4-sample speed increment = %.12g rad/s/count\n',raw_speed_quantum);
fprintf(fid,'Continuous speed poles: %s\n',mat2str(poles_ct,12));
fprintf(fid,'Discrete speed poles: %s\n',mat2str(poles_dt,12));
fprintf(fid,'Extra position integrator pole in 3-state discrete model: z=1\n');
fprintf(fid,'Steady raw ripple peak-to-peak: %.12g rad/s\n',steady_ripple_raw);
fprintf(fid,'Steady filtered ripple peak-to-peak: %.12g rad/s\n',steady_ripple_filt);
fclose(fid);

fprintf('\n===== STAGE 03: DISCRETE MEASUREMENT ANALYSIS =====\n');
fprintf('Input parameter source: %s\n',parameterSource);
fprintf('Ts = %.4f s; fs = %.0f Hz; N_enc = %d; N_avg = %d\n',Ts,fs,N_enc,N_avg);
fprintf('alpha_f = %.8f; count quantum = %.8f rad\n',alpha_f,count_to_rad);
fprintf('4-sample speed quantum = %.8f rad/s\n',raw_speed_quantum);
fprintf('Speed z-plane poles: %s\n',mat2str(poles_dt,10));
disp(measurement_metrics); disp(sample_sensitivity);
fprintf('Results saved: %s\n',outDir);

%% Local functions - compatible with MATLAB R2024a scripts
function [counts,w_raw,w_filt,w_windowed] = measure_angle(theta,Ts,N_enc,N_avg,alpha)
    theta=theta(:);
    counts=round(theta*N_enc/(2*pi));
    n=numel(counts);
    w_raw=zeros(n,1);
    w_windowed=zeros(n,1);
    scale=2*pi/(N_enc*N_avg*Ts);
    for k=N_avg+1:n
        w_raw(k)=(counts(k)-counts(k-N_avg))*scale;
        w_windowed(k)=(theta(k)-theta(k-N_avg))/(N_avg*Ts);
    end
    w_filt=filter(1-alpha,[1,-alpha],w_raw);
end
function save_plot(fig,outDir,name)
    savefig(fig,fullfile(outDir,[name,'.fig']));
    exportgraphics(fig,fullfile(outDir,[name,'.png']),'Resolution',180);
    close(fig);
end
