%% EE302 PS5 - Stage 05: Verify and archive PI Simulink experiments
% MATLAB/Simulink R2024a. Run after ps5_stage05_build_pi_simulink.m.
% Auto-runs every scenario, compares against Stage 4 P logs if available.
% Hardware-matched PI topology: PI internal clamp -> disturbance -> final clamp.
clear; clc; close all;
thisDir=fileparts(mfilename('fullpath')); projectDir=fileparts(thisDir);
outDir=fullfile(projectDir,'results','05_pi_controller');
if ~exist(outDir,'dir'), mkdir(outDir); end
mdl='ps5_stage05_pi_hardware_matched_R2024a';
modelFile=fullfile(thisDir,[mdl '.slx']);
if ~exist(modelFile,'file')
    fprintf('Stage 5 Simulink model missing; building now...\n');
    run(fullfile(thisDir,'ps5_stage05_build_pi_simulink.m'));
end
load(fullfile(outDir,'00_pi_stage05_parameters.mat'), ...
    'Pz','Kp','Ki','Ts','Vmax','Vmin','N_enc','N_avg','alpha_f', ...
    'Jeq','R','L','Ke','Kt','beq','K_dc','tau','parameterSource');
load_system(modelFile);
set_param(mdl,'StopTime','10','Solver','FixedStepDiscrete', ...
    'FixedStep',num2str(Ts,17),'ReturnWorkspaceOutputs','on');
piBlk=[mdl '/Discrete_PI'];
assert(strcmp(get_param(piBlk,'Controller'),'PI'),'Controller must be PI.');
assert(strcmp(get_param(piBlk,'Form'),'Parallel'),'Controller form must be Parallel.');
assert(strcmp(get_param(piBlk,'TimeDomain'),'Discrete-time'),'Must be discrete-time.');
assert(strcmp(get_param(piBlk,'IntegratorMethod'),'Forward Euler'),'Must use Forward Euler.');
assert(strcmp(get_param(piBlk,'AntiWindupMode'),'clamping'),'Must use clamping anti-windup.');
assert(strcmp(get_param(piBlk,'LimitOutput'),'on'),'Internal PI output saturation missing.');
% Scenarios 1-8 mirror the Stage 4 P trials. Scenario 9 shows two clamps:
% a 250 rad/s target cannot be reached at 10 V with this datasheet plant;
% +0.5 V after t=5 makes the final clamp active, not just the PI block.
scenarios={
    '01_nominal_hw_protocol',1.00,50,+0.5;
    '02_reference_no_disturbance',1.00,50,0;
    '03_high_reference_saturation',1.00,120,0;
    '04_high_reference_plus_disturbance',1.00,120,+0.5;
    '05_reduced_inertia_75pct',0.75,50,+0.5;
    '06_increased_inertia_150pct',1.50,50,+0.5;
    '07_doubled_inertia_200pct',2.00,50,+0.5;
    '08_negative_voltage_disturbance',1.00,50,-0.5;
    '09_unreachable_250rad_s',1.00,250,+0.5
};
logNames={'s5_pi_ref','s5_pi_error','s5_pi_internal_cmd', ...
    's5_pi_disturbance','s5_pi_cmd_raw','s5_pi_cmd', ...
    's5_pi_speed_ideal','s5_pi_angle','s5_pi_counts', ...
    's5_pi_speed_raw','s5_pi_speed_filt'};
fields={'ref','error','u_pi','disturbance','u_cmd_raw','u_cmd', ...
    'speed_ideal','angle','counts','speed_raw','speed_filt'};
metrics=struct([]); traces=struct([]);
for j=1:size(scenarios,1)
    name=scenarios{j,1}; Jfactor=scenarios{j,2};
    refFinal=scenarios{j,3}; dFinal=scenarios{j,4};
    Jtest=Jeq*Jfactor;
    Aj=[-R/L,-Ke/L,0;Kt/Jtest,-beq/Jtest,0;0,1,0];
    Bj=[1/L;0;0]; Cj=[0,1,0;0,0,1]; Dj=[0;0];
    Pj=c2d(ss(Aj,Bj,Cj,Dj),Ts,'zoh');
    set_param([mdl '/Motor_Disk_ZOH'], ...
        'A',mat2str(Pj.A,17),'B',mat2str(Pj.B,17), ...
        'C',mat2str(Pj.C,17),'D',mat2str(Pj.D,17));
    set_param([mdl '/Speed_Ref'],'Time','1','After',num2str(refFinal,17));
    set_param([mdl '/Disturbance'],'Time','5','After',num2str(dFinal,17));
    simOut=sim(mdl);
    s=struct();
    for i=1:numel(logNames)
        x=simOut.get(logNames{i});
        assert(isnumeric(x),'Expected numeric To Workspace (Array format) output');
        s.(fields{i})=x(:);
    end
    n=numel(s.ref);
    for i=1:numel(fields)
        assert(numel(s.(fields{i}))==n,['Length mismatch for ' fields{i}]);
        assert(all(isfinite(s.(fields{i}))),['Nonfinite values in ' fields{i}]);
    end
    s.time=(0:n-1)'*Ts;
    assert(abs(s.time(end)-10)<Ts/10,'Unexpected sample count/StopTime.');
    % Independent, rigorous checks of the controller connections and
    % measurement algebra. PI mask settings are checked above separately.
    controllerCheck=[max(abs(s.error-(s.ref-s.speed_filt))), ...
        max(abs(s.u_cmd_raw-(s.u_pi+s.disturbance))), ...
        max(abs(s.u_cmd-max(Vmin,min(Vmax,s.u_cmd_raw)))), ...
        max(max(s.u_pi-Vmax),0),max(max(Vmin-s.u_pi),0)];
    countsExpected=round(s.angle*N_enc/(2*pi));
    speedExpected=zeros(n,1);
    speedExpected(N_avg+1:end)= ...
        (countsExpected(N_avg+1:end)-countsExpected(1:end-N_avg)) ...
        *2*pi/(N_enc*N_avg*Ts);
    speedFilteredExpected=filter(1-alpha_f,[1,-alpha_f],speedExpected);
    measureCheck=[max(abs(s.counts-countsExpected)), ...
        max(abs(s.speed_raw-speedExpected)), ...
        max(abs(s.speed_filt-speedFilteredExpected))];
    assert(max([controllerCheck measureCheck])<1e-7, ...
        ['Controller wiring or measurement consistency check failed: ' name]);
    m=stage05_metrics(s,name,Jfactor,refFinal,dFinal,Kp,Ki,Vmax,Vmin);
    if isempty(metrics), metrics=m; else, metrics(end+1)=m; end %#ok<AGROW>
    traces(j).name=name; traces(j).data=s; %#ok<SAGROW>
    writetable(struct2table(s),fullfile(outDir,[name '_signals.csv']));
    save(fullfile(outDir,[name '_results.mat']), ...
        's','m','Jfactor','Jtest','refFinal','dFinal','Pj', ...
        'Kp','Ki','Ts','Vmax','Vmin','N_enc','N_avg','alpha_f', ...
        'controllerCheck','measureCheck');
    f=figure('Visible','off','Color','w','Position',[100 100 1200 820]);
    subplot(3,1,1);
    plot(s.time,s.ref,'k--','LineWidth',1.2); hold on;
    plot(s.time,s.speed_ideal,'LineWidth',1.2);
    plot(s.time,s.speed_filt,'LineWidth',1.1); grid on;
    xlabel('Time (s)'); ylabel('Speed (rad/s)');
    title(strrep(name,'_',' '));
    legend('Reference','Physical plant speed','Filtered encoder speed','Location','best');
    subplot(3,1,2);
    plot(s.time,s.u_pi,'LineWidth',1.2); hold on;
    plot(s.time,s.u_cmd_raw,':','LineWidth',1.15);
    plot(s.time,s.u_cmd,'LineWidth',1.15);
    yline(Vmax,'k--'); yline(Vmin,'k--'); grid on;
    xlabel('Time (s)'); ylabel('Voltage (V)');
    legend('Internal saturated PI output','PI_cmd_raw: after disturbance', ...
        'PI_cmd: final-clamped plant input','+/-10 V limits','Location','best');
    subplot(3,1,3);
    plot(s.time,s.error,'LineWidth',1.1); hold on;
    plot(s.time,s.disturbance,'LineWidth',1.1); grid on;
    xlabel('Time (s)'); ylabel('Feedback error (rad/s) / offset (V)');
    legend('Speed error','Disturbance voltage','Location','best');
    savefig(f,fullfile(outDir,[name '_plot.fig']));
    exportgraphics(f,fullfile(outDir,[name '_plot.png']),'Resolution',170);
    close(f);
    fprintf('%s: rise=%.4f s; settle=%.4f s; ',name, ...
        m.Rise10to90_s,m.Settling2pct_s);
    fprintf('pre-error=%.4f; post-error=%.4f; ', ...
        m.PreDistError_rad_s,m.PostDistError_rad_s);
    fprintf('internal sat=%.2f%%; final sat=%.2f%%; peak cmd=%.3fV\n', ...
        m.InternalPISatFraction_pct,m.FinalSatFraction_pct,m.PeakAbsCmd_V);
end
% Report figure: PI robustness to equivalent inertia (same Kp, Ki).
f=figure('Visible','off','Color','w','Position',[100 100 1180 690]);
hold on;
for j=[1 5 6 7]
    plot(traces(j).data.time,traces(j).data.speed_filt,'LineWidth',1.15);
end
plot(traces(1).data.time,traces(1).data.ref,'k--','LineWidth',1);
grid on; xlabel('Time (s)'); ylabel('Filtered speed (rad/s)');
title('Stage 05: inertia sensitivity, SAME PI gains and clamping anti-windup');
legend('100% J','75% J','150% J','200% J','Reference','Location','best');
savefig(f,fullfile(outDir,'92_pi_inertia_sensitivity.fig'));
exportgraphics(f,fullfile(outDir,'92_pi_inertia_sensitivity.png'),'Resolution',180);
close(f);
% Report figure: PI disturbance rejection + enforced final clamp.
f=figure('Visible','off','Color','w','Position',[100 100 1180 790]);
subplot(2,1,1);
plot(traces(1).data.time,traces(1).data.speed_filt,'LineWidth',1.4); hold on;
plot(traces(2).data.time,traces(2).data.speed_filt,'--','LineWidth',1.2);
plot(traces(1).data.time,traces(1).data.ref,'k:','LineWidth',1); grid on;
xlabel('Time (s)'); ylabel('Speed (rad/s)');
title('PI response to +0.5 V offset at 5 s (nominal 50 rad/s reference)');
legend('With disturbance','Without disturbance','Reference','Location','best');
subplot(2,1,2);
s9=traces(9).data;
plot(s9.time,s9.u_pi,'LineWidth',1.2); hold on;
plot(s9.time,s9.u_cmd_raw,':','LineWidth',1.4);
plot(s9.time,s9.u_cmd,'LineWidth',1.2);
yline(Vmax,'k--'); grid on;
xlabel('Time (s)'); ylabel('Voltage (V)');
title('Stress: unreachable 250 rad/s reference, final +/-10 V saturation');
legend('Internally clamped PI','PI_cmd_raw after disturbance', ...
    'Final PI_cmd / plant voltage','10 V limit','Location','best');
savefig(f,fullfile(outDir,'93_pi_disturbance_and_saturation.fig'));
exportgraphics(f,fullfile(outDir,'93_pi_disturbance_and_saturation.png'),'Resolution',180);
close(f);
summary=struct2table(metrics);
writetable(summary,fullfile(outDir,'94_stage05_pi_metrics.csv'));
save(fullfile(outDir,'95_stage05_all_trials.mat'), ...
    'traces','summary','scenarios','Kp','Ki','Ts','Vmax','Vmin','parameterSource');
% Optional stage 4: overlay P and PI, using stored Stage 4 FULL logs,
% never attempting to reconstruct the P response from screenshots.
pFile=fullfile(projectDir,'results','04_p_controller','93_stage04_all_trials.mat');
if exist(pFile,'file')
    par=load(pFile,'traces','summary');
    pNom=par.traces(1).data; piNom=traces(1).data;
    assert(abs(pNom.time(end)-piNom.time(end))<Ts/10,'P and PI durations differ.');
    f=figure('Visible','off','Color','w','Position',[100 100 1150 770]);
    subplot(2,1,1);
    plot(piNom.time,piNom.ref,'k--','LineWidth',1.15); hold on;
    plot(pNom.time,pNom.speed_filt,'LineWidth',1.25);
    plot(piNom.time,piNom.speed_filt,'LineWidth',1.25); grid on;
    xlabel('Time (s)'); ylabel('Filtered speed (rad/s)');
    title('Hardware protocol simulation: P vs PI; 50 rad/s at 1 s; +0.5 V at 5 s');
    legend('Reference','P feedback','PI feedback','Location','best');
    subplot(2,1,2);
    plot(pNom.time,pNom.u_plant,'LineWidth',1.2); hold on;
    plot(piNom.time,piNom.u_cmd,'LineWidth',1.2); grid on;
    yline(Vmax,'k--'); yline(Vmin,'k--');
    xlabel('Time (s)'); ylabel('Applied plant voltage (V)');
    title('Comparable control effort: actual plant voltage, NOT differently located cmd logs');
    legend('P plant-input voltage','PI plant-input voltage','Limits','Location','best');
    savefig(f,fullfile(outDir,'90_p_vs_pi_tracking_and_control.fig'));
    exportgraphics(f,fullfile(outDir,'90_p_vs_pi_tracking_and_control.png'),'Resolution',190);
    close(f);
    pMetrics=par.summary(1,:); piMetrics=summary(1,:);
    comparison=table({'P';'PI'}, ...
        [pMetrics.Rise10to90_s;piMetrics.Rise10to90_s], ...
        [pMetrics.Settling2pct_s;piMetrics.Settling2pct_s], ...
        [pMetrics.PreDistError_rad_s;piMetrics.PreDistError_rad_s], ...
        [pMetrics.PostDistError_rad_s;piMetrics.PostDistError_rad_s], ...
        [pMetrics.PeakAbsPlantVoltage_V;piMetrics.PeakAbsCmd_V], ...
        [pMetrics.RMS_PlantVoltage_V;piMetrics.RMS_Cmd_V], ...
        [pMetrics.IntegralAbsError_rad;piMetrics.IntegralAbsError_rad], ...
        'VariableNames',{'Controller','Rise10to90_s','Settling2pct_s', ...
        'PreDistError_rad_s','PostDistError_rad_s','PeakAppliedVoltage_V', ...
        'RMSAppliedVoltage_V','IAE_rad'});
    writetable(comparison,fullfile(outDir,'91_p_vs_pi_nominal_comparison.csv'));
    disp('===== STAGE 4 P vs STAGE 5 PI: NOMINAL HARDWARE PROTOCOL =====');
    disp(comparison);
else
    fprintf('Stage 4 full logs not found at %s\n',pFile);
    fprintf('PI simulations complete; copy your Stage 4 results folder to generate P-vs-PI overlay.\n');
end
fid=fopen(fullfile(outDir,'96_stage05_run_record.txt'),'w');
fprintf(fid,'Stage 05 MATLAB release=%s, run=%s\n',version('-release'),datestr(now,31));
fprintf(fid,'Model=%s\nParameter source=%s\n',modelFile,parameterSource);
fprintf(fid,'Gains: Kp=%.12g, Ki=%.12g, sample period=%.12g\n',Kp,Ki,Ts);
fprintf(fid,'Exact PI block: Parallel, discrete, Forward Euler, internal +/-10V saturation and clamping anti-windup.\n');
fprintf(fid,'Final saturation AFTER disturbance (different from Stage-4 P).\n');
fprintf(fid,'Raw cmd log is after disturbance / before final clamp; cmd log is final plant voltage.\n');
fprintf(fid,'Feedback 4-sample quadrature encoder speed, filter alpha=%.12g.\n',alpha_f);
fprintf(fid,'No physical hardware data, amplifier deadband or unmeasured friction in model.\n');
fprintf(fid,'All controller and measurement self-checks passed (<1e-7).\n');
fclose(fid);
% Restore base PI scenario/model after batch.
set_param([mdl '/Motor_Disk_ZOH'], ...
    'A',mat2str(Pz.A,17),'B',mat2str(Pz.B,17), ...
    'C',mat2str(Pz.C,17),'D',mat2str(Pz.D,17));
set_param([mdl '/Speed_Ref'],'Time','1','After','50');
set_param([mdl '/Disturbance'],'Time','5','After','0.5');
save_system(mdl,modelFile);
fprintf('\n===== STAGE 05: PI CONTROLLER SIMULATION COMPLETE =====\n');
disp(summary(:,{'Scenario','Rise10to90_s','Settling2pct_s', ...
    'PreDistError_rad_s','PostDistError_rad_s', ...
    'InternalPISatFraction_pct','FinalSatFraction_pct','PeakAbsCmd_V'}));
fprintf('All results saved: %s\n',outDir);

function m=stage05_metrics(s,name,Jfactor,refFinal,dFinal,Kp,Ki,Vmax,Vmin)
    t=s.time; w=s.speed_filt;
    before=mean(w(t>=4.5 & t<4.9));
    after=mean(w(t>=9.5 & t<=10));
    start=mean(w(t>=0.5 & t<0.9));
    idx=find(t>=1 & t<4.9);
    span=before-start;
    i10=[]; i90=[];
    if span>1e-10
        i10=find(w(idx)>=start+0.1*span,1,'first');
        i90=find(w(idx)>=start+0.9*span,1,'first');
    end
    rise=NaN;
    if ~isempty(i10) && ~isempty(i90)
        rise=t(idx(i90))-t(idx(i10));
    end
    tol=max(0.02*abs(span),1e-6);
    bad=idx(abs(w(idx)-before)>tol);
    settle=0;
    if ~isempty(bad)
        next=bad(end)+1;
        if next<=idx(end), settle=t(next)-1; else, settle=NaN; end
    end
    peak=max(w(idx));
    over=max(0,(peak-before)/max(abs(span),eps)*100);
    m=struct('Scenario',name,'InertiaFactor',Jfactor, ...
        'Kp_V_per_rad_s',Kp,'Ki_V_per_rad',Ki, ...
        'Reference_rad_s',refFinal,'Disturbance_V',dFinal, ...
        'Rise10to90_s',rise,'Settling2pct_s',settle, ...
        'Overshoot_pct',over,'PreDistSpeed_rad_s',before, ...
        'PostDistSpeed_rad_s',after, ...
        'PreDistError_rad_s',refFinal-before, ...
        'PostDistError_rad_s',refFinal-after, ...
        'DisturbanceSpeedShift_rad_s',after-before, ...
        'PeakAbsInternalPI_V',max(abs(s.u_pi)), ...
        'PeakAbsCmdRaw_V',max(abs(s.u_cmd_raw)), ...
        'PeakAbsCmd_V',max(abs(s.u_cmd)), ...
        'RMS_Cmd_V',sqrt(mean(s.u_cmd.^2)), ...
        'IntegralAbsError_rad',trapz(t,abs(s.error)), ...
        'ControlVoltageEnergy_V2s',trapz(t,s.u_cmd.^2), ...
        'InternalPISatFraction_pct',100*mean(abs(s.u_pi)>=Vmax-1e-8), ...
        'FinalSatFraction_pct',100*mean(abs(s.u_cmd_raw-s.u_cmd)>1e-8), ...
        'PlantVoltageExceedsCmdLimit',double(any(s.u_cmd>Vmax+1e-8 | s.u_cmd<Vmin-1e-8)));
end
