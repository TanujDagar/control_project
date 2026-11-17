%% EE302 PS5 - Stage 04: Run, validate, archive P-controller Simulink trials
% MATLAB R2024a; run AFTER ps5_stage04_build_p_simulink (or run this alone).
% Original P topology: P gain -> SATURATE -> add voltage disturbance -> plant.
% Saves all traces and plots under ../results/04_p_controller for the report.
clear; clc; close all;
thisDir=fileparts(mfilename('fullpath')); projectDir=fileparts(thisDir);
outDir=fullfile(projectDir,'results','04_p_controller');
if ~exist(outDir,'dir'), mkdir(outDir); end
mdl='ps5_stage04_p_hardware_matched_R2024a';
modelFile=fullfile(thisDir,[mdl '.slx']);
if ~exist(modelFile,'file')
    fprintf('Missing Simulink model; building with simulation-only parameters...\n');
    run(fullfile(thisDir,'ps5_stage04_build_p_simulink.m'));
end
% Load the parameter snapshot saved at model-build time; do not load HW file.
load(fullfile(outDir,'00_p_stage04_parameters.mat'), ...
    'Pz','Kp_only','Ts','Vmax','Vmin','N_enc','N_avg','alpha_f', ...
    'Jeq','R','L','Ke','Kt','beq','parameterSource');
load_system(modelFile);
set_param(mdl,'StopTime','10','Solver','FixedStepDiscrete','FixedStep',num2str(Ts,17), ...
    'ReturnWorkspaceOutputs','on');
% Every scenario retains the SAME block order and feedback measurement chain.
% Inertia factors scale TOTAL equivalent J; Kp is never retuned.
scenarios={
    '01_nominal_hw_protocol', 1.00, 50, +0.5;
    '02_reference_no_disturbance',1.00,50,0;
    '03_high_reference_saturation',1.00,120,0;
    '04_high_reference_plus_disturbance',1.00,120,+0.5;
    '05_reduced_inertia_75pct',0.75,50,+0.5;
    '06_increased_inertia_150pct',1.50,50,+0.5;
    '07_doubled_inertia_200pct',2.00,50,+0.5;
    '08_negative_voltage_disturbance',1.00,50,-0.5
};
logNames={'s4_p_ref','s4_p_error','s4_p_unclamped','s4_p_cmd', ...
    's4_p_disturbance','s4_p_plant_voltage','s4_p_speed_ideal', ...
    's4_p_angle','s4_p_counts','s4_p_speed_raw','s4_p_speed_filt'};
fields={'ref','error','u_unclamped','u_cmd','disturbance','u_plant', ...
    'speed_ideal','angle','counts','speed_raw','speed_filt'};
metrics=struct([]); traces=struct([]);
for j=1:size(scenarios,1)
    name=scenarios{j,1}; Jfactor=scenarios{j,2}; refFinal=scenarios{j,3}; dFinal=scenarios{j,4};
    % Vary only the plant inertia and input amplitudes, never the P feedback topology.
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
        assert(isnumeric(x),'Expected numeric Array-format To Workspace output');
        s.(fields{i})=x(:);
    end
    n=numel(s.ref);
    for i=1:numel(fields)
        assert(numel(s.(fields{i}))==n,['Length mismatch in ' fields{i}]);
        assert(all(isfinite(s.(fields{i}))),['Nonfinite values in ' fields{i}]);
    end
    s.time=(0:n-1)'*Ts;
    assert(abs(s.time(end)-10)<Ts/10,'Unexpected sample count or StopTime');
    % Algebraic verification of matching P topology and independent encoder math.
    check=[max(abs(s.error-(s.ref-s.speed_filt))), ...
           max(abs(s.u_unclamped-Kp_only*s.error)), ...
           max(abs(s.u_cmd-max(Vmin,min(Vmax,s.u_unclamped)))), ...
           max(abs(s.u_plant-(s.u_cmd+s.disturbance)))];
    qcalc=round(s.angle*N_enc/(2*pi));
    rc=zeros(n,1);
    rc(N_avg+1:end)=(qcalc(N_avg+1:end)-qcalc(1:end-N_avg)) ...
        *2*pi/(N_enc*N_avg*Ts);
    fc=filter(1-alpha_f,[1,-alpha_f],rc);
    measureCheck=[max(abs(s.counts-qcalc)),max(abs(s.speed_raw-rc)), ...
        max(abs(s.speed_filt-fc))];
    assert(max([check measureCheck])<1e-7, ...
        ['Signal topology or measurement chain check failed in ' name]);
    m=calculate_metrics(s,refFinal,Jfactor,dFinal,Kp_only,Vmax,Vmin);
    m.Scenario=name;
    if isempty(metrics), metrics=m; else, metrics(end+1)=m; end %#ok<AGROW>
    traces(j).name=name; traces(j).data=s; %#ok<SAGROW>
    writetable(struct2table(s),fullfile(outDir,[name '_signals.csv']));
    save(fullfile(outDir,[name '_results.mat']), ...
        's','m','Jfactor','Jtest','refFinal','dFinal','Pj','Kp_only','Ts', ...
        'Vmax','Vmin','N_enc','N_avg','alpha_f','check','measureCheck');
    f=figure('Visible','off','Color','w','Position',[100 100 1250 800]);
    subplot(3,1,1);
    plot(s.time,s.ref,'k--','LineWidth',1.25); hold on;
    plot(s.time,s.speed_ideal,'LineWidth',1.2);
    plot(s.time,s.speed_filt,'LineWidth',1.15); grid on;
    xlabel('Time (s)'); ylabel('Angular speed (rad/s)');
    title(strrep(name,'_',' '));
    legend('Reference','Physical plant speed','Filtered encoder feedback','Location','best');
    subplot(3,1,2);
    plot(s.time,s.u_unclamped,':','LineWidth',1); hold on;
    plot(s.time,s.u_cmd,'LineWidth',1.2);
    plot(s.time,s.u_plant,'LineWidth',1);
    yline(Vmax,'k--'); yline(Vmin,'k--'); grid on;
    xlabel('Time (s)'); ylabel('Voltage (V)');
    legend('Unclamped P','P_cmd: clamped BEFORE disturbance', ...
        'Plant input: P_cmd + disturbance','Limits','Location','best');
    subplot(3,1,3);
    plot(s.time,s.error,'LineWidth',1.1); hold on;
    plot(s.time,s.disturbance,'LineWidth',1.1); grid on;
    xlabel('Time (s)'); ylabel('Error (rad/s) / offset (V)');
    legend('Feedback error','Disturbance voltage','Location','best');
    savefig(f,fullfile(outDir,[name '_plot.fig']));
    exportgraphics(f,fullfile(outDir,[name '_plot.png']),'Resolution',170);
    close(f);
    fprintf('%s: rise=%.4f s; settle=%.4f s; pre-error=%.3f; post-error=%.3f; ', ...
        name,m.Rise10to90_s,m.Settling2pct_s,m.PreDistError_rad_s,m.PostDistError_rad_s);
    fprintf('sat=%.2f%%; max u_plant=%.3f V\n',m.SaturationFraction_pct, ...
        m.PeakAbsPlantVoltage_V);
end
% Export comparable fixed-gain inertia sensitivity and saturation comparison.
f=figure('Visible','off','Color','w','Position',[100 100 1080 640]);
hold on;
for j=[1 5 6 7]
    plot(traces(j).data.time,traces(j).data.speed_filt,'LineWidth',1.15);
end
plot(traces(1).data.time,traces(1).data.ref,'k--','LineWidth',1); grid on;
xlabel('Time (s)'); ylabel('Filtered angular speed (rad/s)');
title('Stage 04: inertia sensitivity, SAME P controller gain');
legend('100% J','75% J','150% J','200% J','Reference','Location','best');
savefig(f,fullfile(outDir,'90_inertia_sensitivity.fig'));
exportgraphics(f,fullfile(outDir,'90_inertia_sensitivity.png'),'Resolution',180);
close(f);
f=figure('Visible','off','Color','w','Position',[100 100 1080 640]);
subplot(2,1,1);
plot(traces(1).data.time,traces(1).data.speed_filt,'LineWidth',1.4); hold on;
plot(traces(2).data.time,traces(2).data.speed_filt,'--','LineWidth',1.3); grid on;
xlabel('Time (s)'); ylabel('Speed (rad/s)');
title('50 rad/s reference: effect of +0.5 V offset at 5 s');
legend('With voltage offset','Without offset','Location','best');
subplot(2,1,2);
plot(traces(3).data.time,traces(3).data.u_unclamped,':','LineWidth',1.1); hold on;
plot(traces(3).data.time,traces(3).data.u_cmd,'LineWidth',1.3);
plot(traces(4).data.time,traces(4).data.u_plant,'LineWidth',1.1); grid on;
yline(Vmax,'k--'); xlabel('Time (s)'); ylabel('Voltage (V)');
title('120 rad/s reference: command saturation and post-clamp voltage offset');
legend('Raw P','Clamped P','Plant voltage with +0.5V','10V limit','Location','best');
savefig(f,fullfile(outDir,'91_disturbance_and_saturation.fig'));
exportgraphics(f,fullfile(outDir,'91_disturbance_and_saturation.png'),'Resolution',180);
close(f);
summary=struct2table(metrics);
writetable(summary,fullfile(outDir,'92_stage04_p_metrics.csv'));
save(fullfile(outDir,'93_stage04_all_trials.mat'),'traces','summary','scenarios', ...
    'Kp_only','Ts','Vmax','Vmin','parameterSource');
fid=fopen(fullfile(outDir,'94_stage04_run_record.txt'),'w');
fprintf(fid,'Stage 04 completed with MATLAB %s at %s\n',version('-release'),datestr(now,31));
fprintf(fid,'Source: %s\n',parameterSource);
fprintf(fid,'Plant model: manufacturer full second-order electrical/mechanical + angle integrator; beq=%.9g\n',beq);
fprintf(fid,'Kp_only=%.12g; Ts=%.12g; limits=[%.6g, %.6g] V; N_enc=%d; N_avg=%d; alpha=%.12g\n', ...
    Kp_only,Ts,Vmin,Vmax,N_enc,N_avg,alpha_f);
fprintf(fid,'50 rad/s step at 1 s and +0.5 V at 5 s reproduce original P hardware protocol.\n');
fprintf(fid,'Exact original P topology: SATURATION BEFORE disturbance, NO final clamp.\n');
fprintf(fid,'P_cmd log is prior to disturbance; plant voltage is post-disturbance.\n');
fprintf(fid,'Controller gain stays fixed when inertia changes; NO hardware data used.\n');
fprintf(fid,'For physical comparison, amplifier deadband, friction and device nonlinearities remain unmodelled.\n');
fprintf(fid,'All control-equation and measurement-chain self-tests passed (<1e-7).\n');
fclose(fid);
% Reset baseline settings and re-save as the default P hardware-matched model.
set_param([mdl '/Motor_Disk_ZOH'],'A',mat2str(Pz.A,17), ...
    'B',mat2str(Pz.B,17),'C',mat2str(Pz.C,17),'D',mat2str(Pz.D,17));
set_param([mdl '/Speed_Ref'],'Time','1','After','50');
set_param([mdl '/Disturbance'],'Time','5','After','0.5');
save_system(mdl,modelFile);
fprintf('\n===== STAGE 04: P-CONTROLLER SIMULATION COMPLETE =====\n');
disp(summary(:,{'Scenario','Rise10to90_s','Settling2pct_s', ...
    'PreDistError_rad_s','PostDistError_rad_s', ...
    'SaturationFraction_pct','PeakAbsPlantVoltage_V'}));
fprintf('Saved all signals, MATLAB figures, and scenario metrics: %s\n',outDir);

function m=calculate_metrics(s,refFinal,Jfactor,dFinal,Kp,Vmax,Vmin)
    t=s.time; w=s.speed_filt;
    % Evaluate the tracking transient only between reference at 1 s and
    % disturbance at 5 s; distinguish tracking vs disturbance recovery.
    plateau=mean(w(t>=4.5 & t<4.9));
    post=mean(w(t>=9.5 & t<=10));
    start=mean(w(t>=0.5 & t<0.9));
    idx=find(t>=1 & t<4.9);
    riseSpan=plateau-start;
    if riseSpan>1e-10
        i10=find(w(idx)>=start+0.1*riseSpan,1,'first');
        i90=find(w(idx)>=start+0.9*riseSpan,1,'first');
    else
        i10=[]; i90=[];
    end
    riseTime=NaN;
    if ~isempty(i10) && ~isempty(i90), riseTime=t(idx(i90))-t(idx(i10)); end
    tol=max(0.02*abs(riseSpan),1e-6);
    bad=idx(abs(w(idx)-plateau)>tol);
    settle=0;
    if ~isempty(bad)
        next=bad(end)+1;
        if next<=idx(end), settle=t(next)-1; else, settle=NaN; end
    end
    peak=max(w(idx));
    over=max(0,(peak-plateau)/max(abs(riseSpan),eps)*100);
    m=struct('Scenario','', ...
        'InertiaFactor',Jfactor,'Kp_V_per_rad_s',Kp, ...
        'Reference_rad_s',refFinal,'Disturbance_V',dFinal, ...
        'Rise10to90_s',riseTime,'Settling2pct_s',settle, ...
        'Overshoot_pct',over,'PreDistSpeed_rad_s',plateau, ...
        'PostDistSpeed_rad_s',post, ...
        'PreDistError_rad_s',refFinal-plateau, ...
        'PostDistError_rad_s',refFinal-post, ...
        'DisturbanceSpeedShift_rad_s',post-plateau, ...
        'PeakAbsCmd_V',max(abs(s.u_cmd)), ...
        'PeakAbsPlantVoltage_V',max(abs(s.u_plant)), ...
        'RMS_Cmd_V',sqrt(mean(s.u_cmd.^2)), ...
        'RMS_PlantVoltage_V',sqrt(mean(s.u_plant.^2)), ...
        'IntegralAbsError_rad',trapz(t,abs(s.error)), ...
        'ControlVoltageEnergy_V2s',trapz(t,s.u_cmd.^2), ...
        'SaturationFraction_pct',100*mean(abs(s.u_unclamped-s.u_cmd)>1e-8), ...
        'PlantVoltageExceedsCmdLimit',double(any(s.u_plant>Vmax+1e-8 | s.u_plant<Vmin-1e-8)));
end
