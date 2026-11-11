%% ============================================================================
%  l1_mo_pareto_optimization.m   (Stage 2 v2 — BAB IV, optimasi 3-OBJEKTIF)
%  Karakterisasi trade-off surface: TRACKING vs AKURASI MASSA vs ROBUSTNESS/
%  ACTUATION-EFFORT, di atas plant 6-DOF tervalidasi.
%
%  ── APA YANG BERUBAH DARI v1 (2-objektif) ───────────────────────────────────
%  v1 hanya mengukur J1(tracking-ISE) & J2(%error massa) dengan SATU seed noise
%  (deterministik). Hasilnya: parameter menempel di batas atas (mo_ko->1.97,
%  lpf_z->25, kd_z->9.9) -> tanda klasik optimizer mengeksploitasi blind spot:
%  tidak ada yang menghukum noise-amplification & actuation-chatter, padahal
%  gain besar + filter longgar itu costless di simulasi bersih tapi mahal di
%  dunia nyata (noise sensor asli, delay aktuator).
%  v2 menambah:
%    (1) MULTI-SEED evaluation (default 3 seed) -> J1,J2 dirata-ratakan across
%        realisasi noise (bukan overfit ke satu noise draw).
%    (2) OBJEKTIF KE-3 (J3, "robustness/effort cost") = komposit dari:
%          - delta_std : std(%error massa) ANTAR-SEED -> sensitivitas estimator
%                        thd noise draw (semakin besar = semakin rapuh)
%          - sigz_chatter : mean std(sigma_z) pre-drop -> kehalusan aktuasi L1
%          - etc_rate : rata-rata fraksi trigger ETC selama airborne -> beban
%                        komunikasi/aktuasi (relevan ke embeddability)
%        Ketiganya dinormalisasi ke floor tetap (BUKAN ke baseline, supaya
%        tidak meledak kalau baseline kebetulan mendekati nol) lalu dijumlah
%        berbobot sama (W3A=W3B=W3C=1/3, gampang diubah).
%
%  FILOSOFI (tetap sama): MATLAB = generator kandidat + karakterisasi
%  trade-off, BUKAN otoritas final. 0_FINAL_CONFIG.md tetap LOCKED. Hasil
%  optimasi ini bahan diskusi BAB IV — dekat, jauh, atau lebih buruk, semua
%  dilaporkan apa adanya.
%
%  METODE: gamultiobj (NSGA-II, jika Global Optimization Toolbox tersedia &
%  berhasil dipanggil) dengan fallback PSO hand-coded (core MATLAB, sapuan
%  10 kombinasi bobot simplex utk 3 objektif) jika toolbox tak ada / gagal
%  (mis. error komunikasi license 5006) — dibungkus try/catch, tidak crash.
%
%  Output: pareto_candidates_theta.csv (+ breakdown J3), pareto_baseline_locked.csv,
%          Figure 3D Pareto + Figure 2D (J1 vs J2, warna=J3).
% ============================================================================
clear; clc; close all;
SEEDS = [42, 123, 2026];           % multi-seed: seed pertama = sama dgn Stage 1 (kontinuitas)
NSEEDS_SEARCH = numel(SEEDS);      % jumlah realisasi noise per evaluasi kandidat
% bobot komposit J3 (robustness/effort) -- dokumentasikan di tesis kalau diubah
W3A = 1/3;   % bobot: sensitivitas akurasi-massa thd noise draw (delta_std)
W3B = 1/3;   % bobot: chatter aktuasi pre-drop (std sigma_z)
W3C = 1/3;   % bobot: beban trigger ETC (etc_rate)
% floor normalisasi (BUKAN baseline!) -- mencegah ledakan jika baseline ~0
FLOOR_DELTA_STD = 0.05;   % (%) minimum denominator utk delta_std
FLOOR_SIGZ      = 0.01;   % (m/s^2) minimum denominator utk sigz_chatter
FLOOR_ETC       = 0.005;  % (fraksi) minimum denominator utk etc_rate

%% ─────────────────── 1. KONFIGURASI DASAR (locked, identik 6dof) ──────────
P0.g=9.81;
P0.gamma_x=40; P0.gamma_y=40; P0.gamma_z=120;  P0.lpf_xy=10; P0.lpf_z=12;
P0.kobs_xy=15; P0.kobs_z=15;   P0.kp_xy=1.5; P0.kp_z=3.0; P0.kd_xy=1.0; P0.kd_z=5.0;
P0.sig_mod=0.5; P0.sig_lim=4.0;  P0.mo_ko=0.5; P0.m_nom=2.064; P0.m_min=1.5; P0.m_max=5.0;
P0.mo_dth=0.08; P0.thr_kl=17.44; P0.thr_kq=31.72; P0.f_max=40.26;
P0.tilt_max=30; P0.etc_xy=0.3; P0.etc_z=0.5;
P0.KI_XY=0.10; P0.INT_XY_LIM=1.5; P0.VZ_FREEZE=0.30; P0.THR_FREEZE=0.80; P0.THR_CAP=0.85;
P0.Z_AIR_HI=0.12; P0.Z_AIR_LO=0.06;
P0.Jxx=0.021; P0.Jyy=0.021; P0.Jzz=0.042;
P0.att_p=[8.0;8.0;3.5]; P0.rate_p=[0.18;0.18;0.25]; P0.rate_i=[0.06;0.06;0.06];
P0.rate_d=[0.003;0.003;0.000]; P0.tau_max=[4.0;4.0;0.6]; P0.rate_int_lim=[0.5;0.5;0.5];

S.dt=0.01; S.H=5.0; S.t0=1.0; S.Tr=4.0;
S.empty=2.064; S.payload=0.8; S.x_sp=0; S.y_sp=0; S.yaw0=0; S.wind_ax=0.0;
S.tau_mot_up=0.0125; S.tau_mot_dn=0.025; S.tau_tq=0.015; S.tau_est=0.04; S.kd_aero=0.10;
S.Jp_xx=0.002; S.Jp_yy=0.002; S.Jp_zz=0.004;
S.n_pos_z=0.015; S.n_pos_xy=0.03; S.n_vel=0.025;
S.plant_thr_scale=1.0;
MET.recovery_mm=30; MET.settle_mm=80; MET.settle_s=3.0;

FAST_MODE = true;      % true = evaluasi cepat utk pencarian; false = fidelity penuh
if FAST_MODE
    S.T=36.0; S.t_drop=20.0; S.n_sub=4;
else
    S.T=48.0; S.t_drop=25.0; S.n_sub=8;
end

%% ─────────────────── 2. VEKTOR PARAMETER & BATAS PENCARIAN ────────────────
names = {'gamma_z','kobs_z','lpf_z','mo_ko','kp_z','kd_z'};
x0 = [P0.gamma_z, P0.kobs_z, P0.lpf_z, P0.mo_ko, P0.kp_z, P0.kd_z];
%       gamma_z  kobs_z  lpf_z  mo_ko  kp_z  kd_z
LB   = [  30,      5,      3,   0.10,  1.0,  1.0 ];
UB   = [ 300,     40,     30,   2.00,  8.0, 10.0 ];
nvars = numel(x0);

%% ─────────────────── 3. BASELINE (locked config) ──────────────────────────
E0 = evaluate_multi(x0,P0,S,MET,SEEDS,names);
fprintf('[BASELINE locked config] J1=%.1f  J2=%.2f%%  delta_std=%.3f%%  sigz_chatter=%.4f  etc_rate=%.4f\n', ...
    E0.J1,E0.J2,E0.delta_std,E0.sigz_chatter,E0.etc_rate);
if ~isfinite(E0.J1) || ~isfinite(E0.J2)
    error('Baseline locked config gagal dievaluasi (NaN/Inf) -> cek plant sebelum optimasi.');
end
J1_0 = E0.J1; J2_0 = E0.J2;
J3_0 = compose_J3(E0,W3A,W3B,W3C,FLOOR_DELTA_STD,FLOOR_SIGZ,FLOOR_ETC);
fprintf('[BASELINE] J3(robustness/effort composite) = %.3f\n', J3_0);

%% ─────────────────── 4. OPTIMASI: gamultiobj (jika ada) atau PSO fallback ─
has_gads = ~isempty(ver('globaloptim')) && (exist('gamultiobj','file')==2);
gads_ok = false;

if has_gads
    fprintf('[INFO] Global Optimization Toolbox terdeteksi -> mencoba gamultiobj (NSGA-II), 3 objektif.\n');
    try
        opts = optimoptions('gamultiobj', ...
            'PopulationSize',50, 'MaxGenerations',30, ...
            'UseParallel',false, 'Display','iter');
        objfun = @(th) evaluate_pareto_vec3(th,P0,S,MET,SEEDS,names,W3A,W3B,W3C,FLOOR_DELTA_STD,FLOOR_SIGZ,FLOOR_ETC);
        [X,FVAL] = gamultiobj(objfun,nvars,[],[],[],[],LB,UB,opts);
        ThetaSet = X; ObjSet = FVAL;   % kolom: [J1 J2 J3]
        gads_ok = true;
    catch ME
        fprintf('[WARN] gamultiobj gagal dijalankan (%s: %s).\n', ME.identifier, ME.message);
        fprintf('[WARN] Biasanya error komunikasi license/service (mis. 5006), bukan bug kode.\n');
        fprintf('[INFO] Jatuh ke fallback PSO (core MATLAB, tanpa toolbox)...\n');
    end
end

if ~gads_ok
    if ~has_gads
        fprintf('[INFO] Global Optimization Toolbox TIDAK terdeteksi -> fallback PSO (core MATLAB).\n');
    end
    % sapuan bobot simplex (w1+w2+w3=1) representatif utk 3 objektif
    Wlist = [ 1.00 0.00 0.00; 0.00 1.00 0.00; 0.00 0.00 1.00; ...
              0.50 0.50 0.00; 0.50 0.00 0.50; 0.00 0.50 0.50; ...
              0.34 0.33 0.33; 0.60 0.20 0.20; 0.20 0.60 0.20; 0.20 0.20 0.60 ];
    K = size(Wlist,1);
    ThetaSet = zeros(K,nvars); ObjSet = zeros(K,3);
    for i=1:K
        w = Wlist(i,:);
        wsum = @(th) wsum3(th,P0,S,MET,SEEDS,names,w,J1_0,J2_0,W3A,W3B,W3C,FLOOR_DELTA_STD,FLOOR_SIGZ,FLOOR_ETC);
        fprintf('  [PSO] bobot [J1 J2 J3]=[%.2f %.2f %.2f] (%d/%d)...\n',w(1),w(2),w(3),i,K);
        [th_best,~] = pso_min(wsum,LB,UB,20,20);   % dikurangi krn 3x lebih mahal (multi-seed x banyak bobot)
        Ei = evaluate_multi(th_best,P0,S,MET,SEEDS,names);
        j3 = compose_J3(Ei,W3A,W3B,W3C,FLOOR_DELTA_STD,FLOOR_SIGZ,FLOOR_ETC);
        ThetaSet(i,:)=th_best; ObjSet(i,:)=[Ei.J1 Ei.J2 j3];
    end
end

%% ─────────────────── 5. BREAKDOWN DIAGNOSTIK per kandidat ─────────────────
K = size(ThetaSet,1);
Diag = struct('delta_std',zeros(K,1),'sigz_chatter',zeros(K,1),'etc_rate',zeros(K,1));
for i=1:K
    Ei = evaluate_multi(ThetaSet(i,:),P0,S,MET,SEEDS,names);
    Diag.delta_std(i)=Ei.delta_std; Diag.sigz_chatter(i)=Ei.sigz_chatter; Diag.etc_rate(i)=Ei.etc_rate;
end

%% ─────────────────── 6. OUTPUT: tabel, CSV, plot Pareto 3D ────────────────
[~,ord]=sort(ObjSet(:,1)); ThetaSet=ThetaSet(ord,:); ObjSet=ObjSet(ord,:);
fn=fieldnames(Diag); for i=1:numel(fn), Diag.(fn{i})=Diag.(fn{i})(ord); end

fprintf('\n===================== PARETO CANDIDATES (3 objektif) =====================\n');
fprintf('%3s %9s %8s %8s | %7s %7s %7s | %8s %8s %8s %8s %8s %8s\n', ...
    '#','J1_ISE','J2_dm%','J3_rob','dStd%','sigChat','etcRate',names{:});
for i=1:K
    fprintf('%3d %9.1f %8.2f %8.3f | %7.3f %7.4f %7.4f | %8.2f %8.2f %8.2f %8.3f %8.2f %8.2f\n', ...
        i,ObjSet(i,1),ObjSet(i,2),ObjSet(i,3),Diag.delta_std(i),Diag.sigz_chatter(i),Diag.etc_rate(i),ThetaSet(i,:));
end
fprintf('-- Baseline locked -- J1=%.1f J2=%.2f J3=%.3f | dStd=%.3f sigChat=%.4f etcRate=%.4f | theta=[%s]\n', ...
    J1_0,J2_0,J3_0,E0.delta_std,E0.sigz_chatter,E0.etc_rate,sprintf('%.3f ',x0));
fprintf('============================================================================\n');

Tp = array2table([ThetaSet ObjSet Diag.delta_std Diag.sigz_chatter Diag.etc_rate], ...
    'VariableNames',[names,{'J1_trans_ise','J2_dm_err_pct','J3_composite', ...
    'delta_std_pct','sigz_chatter','etc_rate'}]);
writetable(Tp,'pareto_candidates_theta.csv');
Tb = array2table([x0 J1_0 J2_0 J3_0 E0.delta_std E0.sigz_chatter E0.etc_rate], ...
    'VariableNames',[names,{'J1_trans_ise','J2_dm_err_pct','J3_composite', ...
    'delta_std_pct','sigz_chatter','etc_rate'}]);
writetable(Tb,'pareto_baseline_locked.csv');
fprintf('[CSV] tersimpan: pareto_candidates_theta.csv, pareto_baseline_locked.csv\n');

figure('Name','Pareto 3D — Tracking vs Massa vs Robustness','Color','w','Position',[80 80 1150 480]);
subplot(1,2,1);
scatter3(ObjSet(:,1),ObjSet(:,2),ObjSet(:,3),55,'filled','MarkerFaceColor',[0 0.45 0.74]); hold on; grid on;
plot3(J1_0,J2_0,J3_0,'p','MarkerSize',16,'MarkerFaceColor',[0.85 0.10 0.10],'MarkerEdgeColor','k');
xlabel('J_1 tracking ISE (mm^2 s)'); ylabel('J_2 error massa (%)'); zlabel('J_3 robustness/effort');
title('(a) Pareto 3D'); view(-35,20);
subplot(1,2,2); hold on; grid on;
scatter(ObjSet(:,1),ObjSet(:,2),70,ObjSet(:,3),'filled');
plot(J1_0,J2_0,'p','MarkerSize',18,'MarkerFaceColor',[0.85 0.10 0.10],'MarkerEdgeColor','k', ...
    'DisplayName','Locked config');
cb=colorbar; cb.Label.String='J_3 (robustness/effort cost)'; colormap(gca,'parula');
xlabel('J_1 tracking ISE (mm^2 s)'); ylabel('J_2 error massa (%)');
title('(b) Proyeksi 2D, warna = J_3'); legend('Location','best');

%% ─────────────────── 7. LANGKAH SELANJUTNYA (manual, WAJIB) ───────────────
fprintf(['\n[PENTING] Ini hasil FAST_MODE + multi-seed=%d (masih fidelity dikurangi).\n' ...
         '  1) Perhatikan apakah parameter kandidat MASIH menempel batas LB/UB.\n' ...
         '     Kalau ya di semua sumbu -> pertimbangkan lebar batas ATAU cek apakah\n' ...
         '     bobot W3A/W3B/W3C perlu diperbesar (robustness kurang "mahal").\n' ...
         '  2) Pilih 1-3 titik (knee 3D, titik dekat locked config, titik ekstrem\n' ...
         '     tracking-optimal) -> verifikasi fidelity PENUH (FAST_MODE=false,\n' ...
         '     lebih banyak seed) DAN plot waveform time-series (bukan cuma angka\n' ...
         '     skalar) sebelum dipercaya.\n' ...
         '  3) Baru setelah lolos verifikasi, promosikan MAX 1-2 kandidat ke SITL.\n' ...
         '  4) JANGAN overwrite 0_FINAL_CONFIG.md dari hasil ini tanpa cross-check SITL/SIH.\n'], NSEEDS_SEARCH);

% ============================================================================
%           OBJECTIVE WRAPPERS (multi-seed, 3 objektif)
% ============================================================================
function E = evaluate_multi(theta,P0,S,MET,seeds,names)
    P = P0;
    for i=1:numel(names), P.(names{i}) = theta(i); end
    ns = numel(seeds);
    j1s=zeros(ns,1); j2s=zeros(ns,1); sigzs=zeros(ns,1); etcs=zeros(ns,1); bad=false;
    for s=1:ns
        try
            L = run_sim(P,S,'l1_on',seeds(s));
            M = compute_metrics(L,P,S,MET);
            unhealthy = ~isfinite(M.trans_ise) || ~isfinite(M.delta_err_pct) || ...
                        max(abs([L.roll L.pitch]),[],'all') > 60;
            if unhealthy, bad=true; break; end
            j1s(s)=M.trans_ise; j2s(s)=M.delta_err_pct; sigzs(s)=M.pre_sigz_std; etcs(s)=M.etc_rate;
        catch
            bad=true; break;
        end
    end
    if bad
        E.J1=1e9; E.J2=1e9; E.delta_std=1e9; E.sigz_chatter=1e9; E.etc_rate=1e9; return;
    end
    E.J1 = mean(j1s); E.J2 = mean(j2s);
    E.delta_std = std(j2s);          % sensitivitas %error massa ANTAR-SEED
    E.sigz_chatter = mean(sigzs);    % rata-rata chatter sigma_z pre-drop
    E.etc_rate = mean(etcs);         % rata-rata fraksi trigger ETC
end
function j3 = compose_J3(E,W3A,W3B,W3C,FD,FS,FE)
    j3 = W3A*(E.delta_std/max(FD,eps)) + W3B*(E.sigz_chatter/max(FS,eps)) + W3C*(E.etc_rate/max(FE,eps));
end
function F = evaluate_pareto_vec3(theta,P0,S,MET,seeds,names,W3A,W3B,W3C,FD,FS,FE)
    E = evaluate_multi(theta,P0,S,MET,seeds,names);
    j3 = compose_J3(E,W3A,W3B,W3C,FD,FS,FE);
    F = [E.J1 E.J2 j3];
end
function v = wsum3(theta,P0,S,MET,seeds,names,w,J1_0,J2_0,W3A,W3B,W3C,FD,FS,FE)
    E = evaluate_multi(theta,P0,S,MET,seeds,names);
    j1n = E.J1/max(J1_0,eps); j2n = E.J2/max(J2_0,eps);
    j3n = compose_J3(E,W3A,W3B,W3C,FD,FS,FE);   % sudah ternormalisasi via floor
    v = w(1)*j1n + w(2)*j2n + w(3)*j3n;
end

% ============================================================================
%              PSO FALLBACK (murni core MATLAB, tanpa toolbox)
% ============================================================================
function [xbest,fbest] = pso_min(objfun,LB,UB,nP,nIter)
    nD = numel(LB); LB=LB(:)'; UB=UB(:)';
    X = LB + rand(nP,nD).*(UB-LB);
    V = zeros(nP,nD);
    F = arrayfun(@(i) objfun(X(i,:)), 1:nP)';
    Pbest = X; Fbest = F;
    [fbest,ib] = min(F); xbest = X(ib,:);
    w=0.6; c1=1.5; c2=1.5;
    for it=1:nIter
        r1=rand(nP,nD); r2=rand(nP,nD);
        V = w*V + c1*r1.*(Pbest-X) + c2*r2.*(xbest-X);
        X = X + V;
        X = min(max(X,LB),UB);
        F = arrayfun(@(i) objfun(X(i,:)), 1:nP)';
        upd = F<Fbest; Pbest(upd,:)=X(upd,:); Fbest(upd)=F(upd);
        [fmin,ib] = min(Fbest);
        if fmin<fbest, fbest=fmin; xbest=Pbest(ib,:); end
    end
end

% ============================================================================
%          PLANT 6-DOF + CONTROLLER (identik l1_pid_law_6dof.m)
%          — TIDAK diubah dari versi tervalidasi —
% ============================================================================
function L = run_sim(P,S,mode,seed)
    rng(seed);
    g=P.g; dt=S.dt; N=round(S.T/dt)+1; tg=(0:N-1)*dt;
    n_sub=S.n_sub; dti=dt/n_sub;
    pos=[0;0;0]; vel=[0;0;0]; att=[0;0;S.yaw0]; rates=[0;0;0];
    T_appl=P.m_nom*g; tau_appl=[0;0;0];
    IN.rate_int=[0;0;0]; IN.rate_prev=[0;0;0];
    epos=[0;0;0]; evel=[0;0;0];
    is_l1=any(strcmp(mode,{'l1_on','l1_off'})); mo_en=strcmp(mode,'l1_on');
    if is_l1, C=ctrl_init(P); else, C=pid_init(); end
    was_air=false;
    F={'alt','alt_ref','m_est','sigma_z','thr','etc','vel_z','roll','pitch','yaw', ...
       'roll_cmd','pitch_cmd','p_rate','q_rate','r_rate','m_true','err_mm'};
    for f=F, L.(f{1})=zeros(1,N); end
    for k=1:N
        t=tg(k); m_t=true_mass(t,S); [rp,rv,ra,yaw]=ref_traj(t,S);
        loaded=(t<S.t_drop);
        Jv=[P.Jxx;P.Jyy;P.Jzz] + loaded.*[S.Jp_xx;S.Jp_yy;S.Jp_zz];
        ae=dt/(S.tau_est+dt); epos=epos+ae*(pos-epos); evel=evel+ae*(vel-evel);
        zfb=epos+[randn*S.n_pos_xy;randn*S.n_pos_xy;randn*S.n_pos_z];
        vfb=evel+randn(3,1)*S.n_vel;
        h=-zfb(3);
        if was_air, is_air=h>P.Z_AIR_LO; else, is_air=h>P.Z_AIR_HI; end
        etc_any=0;
        if is_l1
            C.is_airborne=is_air;
            if is_air && ~was_air && ~C.first_to, C.sigma=[0 0 0]; C.first_to=true; end
            C.sp_vel_z=rv(3);
            if mo_en && is_air, C=mo_update(C,P,vfb(3),C.u_filt(3),dt); end
            for ax=1:3, C=l1_axis(C,P,ax,zfb(ax),vfb(ax),rp(ax),rv(ax),ra(ax),dt); end
            U1=clamp(C.u_filt(3),0,P.f_max);
            [phi_d,theta_d]=acc_to_att(C.u_filt(1),C.u_filt(2),yaw,P,P.tilt_max);
            [C,phi_cmd,theta_cmd,U1e,etc_any]=etc_throttle(C,P,phi_d,theta_d,U1);
            tn=clamp(invert_thrust(P,U1e),0,P.THR_CAP); C.thr_norm_prev=tn;
            L.m_est(k)=C.m_est; L.sigma_z(k)=C.sigma(3);
        else
            [C,phi_cmd,theta_cmd,tn]=pid_step(C,P,zfb,vfb,rp,rv,ra,yaw,is_air,dt);
            L.m_est(k)=P.m_nom; L.sigma_z(k)=0;
        end
        was_air=is_air; att_sp=[phi_cmd; theta_cmd; yaw]; tn=clamp(tn,0,1.0);
        for s=1:n_sub
            [tau_cmd,IN]=inner_loop(IN,P,att,rates,att_sp,dti);
            tau_appl=tau_appl+dti*(tau_cmd-tau_appl)/S.tau_tq;
            T_ach=(P.thr_kl*tn+P.thr_kq*tn^2)*S.plant_thr_scale;
            if T_ach>T_appl, tm=S.tau_mot_up; else, tm=S.tau_mot_dn; end
            T_appl=T_appl+dti*(T_ach-T_appl)/tm;
            gyro=cross(rates,Jv.*rates); wdot=(tau_appl-gyro)./Jv; rates=rates+dti*wdot;
            phi=att(1); theta=att(2);
            ct=cos(theta); if abs(ct)<1e-3, ct=sign(ct)*1e-3; end
            Te=[1 sin(phi)*tan(theta) cos(phi)*tan(theta); 0 cos(phi) -sin(phi); 0 sin(phi)/ct cos(phi)/ct];
            att=att+dti*(Te*rates);
            phi=att(1); theta=att(2); psi=att(3);
            e3=[cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); cos(phi)*cos(theta)];
            a=[0;0;g]-(T_appl/m_t)*e3-(S.kd_aero/m_t)*vel; a(1)=a(1)+is_air*S.wind_ax;
            vel=vel+dti*a; pos=pos+dti*vel;
            if pos(3)>0, pos(3)=0; if vel(3)>0, vel(3)=0; end; end
        end
        L.alt(k)=-pos(3); L.alt_ref(k)=-rp(3); L.err_mm(k)=(-pos(3)-(-rp(3)))*1e3;
        L.thr(k)=tn; L.etc(k)=etc_any; L.vel_z(k)=vel(3);
        L.roll(k)=rad2deg(att(1)); L.pitch(k)=rad2deg(att(2)); L.yaw(k)=rad2deg(att(3));
        L.roll_cmd(k)=rad2deg(att_sp(1)); L.pitch_cmd(k)=rad2deg(att_sp(2));
        L.p_rate(k)=rad2deg(rates(1)); L.q_rate(k)=rad2deg(rates(2)); L.r_rate(k)=rad2deg(rates(3));
        L.m_true(k)=m_t;
    end
    L.t=tg; L.airborne=(L.alt>P.Z_AIR_LO); L.pos_z_end=pos(3);
end
function [tau,IN]=inner_loop(IN,P,att,rates,att_sp,dti)
    aerr=att_sp-att; aerr(3)=atan2(sin(aerr(3)),cos(aerr(3)));
    omega_sp=P.att_p.*aerr; rerr=omega_sp-rates;
    IN.rate_int=clamp(IN.rate_int+rerr*dti,-P.rate_int_lim,P.rate_int_lim);
    d_rate=(rates-IN.rate_prev)/dti; IN.rate_prev=rates;
    m_norm=P.rate_p.*rerr + P.rate_i.*IN.rate_int - P.rate_d.*d_rate;
    m_norm=clamp(m_norm,-1,1); tau=m_norm.*P.tau_max;
end
function C=ctrl_init(P)
    C.z_hat=[0 0 0]; C.vz_hat=[0 0 0]; C.sigma=[0 0 0]; C.u_filt=[0 0 P.m_nom*P.g];
    C.m_est=P.m_nom; C.mo_m_prev=P.m_nom; C.mo_p_hat=0; C.mo_residual=0;
    C.sig_jump_pending=false; C.sig_jump_dz=0; C.int_xy=[0 0]; C.thr_norm_prev=0;
    C.etc_phi=0; C.etc_theta=0; C.etc_u1=0; C.is_airborne=false; C.first_to=false; C.sp_vel_z=0;
end
function C=mo_update(C,P,vz,U1,dt)
    g=P.g; m0=P.m_nom; Ko=P.mo_ko;
    p=m0*vz; tau=m0*g-U1;
    C.mo_residual=Ko*(p-C.mo_p_hat); C.mo_p_hat=C.mo_p_hat+dt*(tau+C.mo_residual);
    if ~isfinite(C.mo_p_hat), C.mo_p_hat=m0*vz; C.mo_residual=0; end
    C.m_est=clamp(m0+C.mo_residual/g,P.m_min,P.m_max);
    mj=C.m_est-C.mo_m_prev;
    if abs(mj)>P.mo_dth, C.sig_jump_dz=-(mj*g)/max(C.m_est,0.5); C.sig_jump_pending=true; end
    C.mo_m_prev=C.m_est;
end
function C=l1_axis(C,P,axis,pos,vel,rpz,rvz,raz,dt)
    g=P.g; is_xy=(axis<3);
    if is_xy
        if axis==1, gamma=P.gamma_x; else, gamma=P.gamma_y; end
        lpf=P.lpf_xy; kobs=P.kobs_xy; kp=P.kp_xy; kd=P.kd_xy;
    else, gamma=P.gamma_z; lpf=P.lpf_z; kobs=P.kobs_z; kp=P.kp_z; kd=P.kd_z; end
    sl=P.sig_lim; e_z=C.z_hat(axis)-pos; e_vz=C.vz_hat(axis)-vel;
    if is_xy, u_eff=C.u_filt(axis); else, u_eff=g-C.u_filt(3)/max(C.m_est,0.5); end
    C.z_hat(axis)=C.z_hat(axis)+dt*(C.vz_hat(axis)-kobs*e_z);
    C.vz_hat(axis)=C.vz_hat(axis)+dt*(u_eff+C.sigma(axis)-kobs*e_vz);
    a_max=0; sat_hi=false; sat_lo=false;
    if is_xy
        a_max=g*tan(deg2rad(P.tilt_max))*1.4;
        u_pre=kp*(rpz-pos)+kd*(rvz-vel)+raz+C.int_xy(axis)-C.sigma(axis);
        sat_hi=(u_pre>a_max); sat_lo=(u_pre<-a_max);
    end
    if C.is_airborne
        dsig=-gamma*e_vz-gamma*P.sig_mod*C.sigma(axis);
        if is_xy
            ep=rpz-pos; in_tr=(abs(C.sp_vel_z)>P.VZ_FREEZE)||(C.thr_norm_prev>P.THR_FREEZE);
            if ~in_tr
                bh=sat_hi&&(ep>0); bl=sat_lo&&(ep<0);
                if ~bh&&~bl, C.int_xy(axis)=clamp(C.int_xy(axis)+P.KI_XY*ep*dt,-P.INT_XY_LIM,P.INT_XY_LIM); end
            end
        end
        if sat_hi&&dsig<0, dsig=0; end
        if sat_lo&&dsig>0, dsig=0; end
        C.sigma(axis)=C.sigma(axis)+dt*dsig;
        if axis==3&&C.sig_jump_pending, C.sigma(axis)=C.sigma(axis)+C.sig_jump_dz; C.sig_jump_pending=false; end
        C.sigma(axis)=clamp(C.sigma(axis),-sl,sl);
    end
    u_raw=kp*(rpz-pos)+kd*(rvz-vel)+raz;
    if is_xy, u_raw=u_raw+C.int_xy(axis); end
    u_raw=u_raw-C.sigma(axis);
    if is_xy, u_raw=clamp(u_raw,-a_max,a_max);
    else, u_raw=C.m_est*(g-u_raw); fmc=P.thr_kl+P.thr_kq; if fmc>1, fme=fmc; else, fme=P.f_max; end
        u_raw=clamp(u_raw,0,fme); end
    al=(lpf*dt)/(1+lpf*dt); C.u_filt(axis)=(1-al)*C.u_filt(axis)+al*u_raw;
end
function [C,phi,theta,U1,trig]=etc_throttle(C,P,phi,theta,U1)
    dxy=deg2rad(P.etc_xy); dz=P.etc_z; trig=0;
    if abs(phi-C.etc_phi)>=dxy, C.etc_phi=phi; trig=1; else, phi=C.etc_phi; end
    if abs(theta-C.etc_theta)>=dxy, C.etc_theta=theta; trig=1; else, theta=C.etc_theta; end
    if abs(U1-C.etc_u1)>=dz, C.etc_u1=U1; trig=1; else, U1=C.etc_u1; end
end
function C=pid_init(), C.vint=[0;0;0]; C.vdot=[0;0;0]; C.vprev=[0;0;0]; end
function [C,phi_d,theta_d,tn]=pid_step(C,P,pos,vel,rp,rv,ra,yaw,is_air,dt)
    g=P.g;
    Kp=[P.mpc_xy_p;P.mpc_xy_p;P.mpc_z_p]; Pv=[P.mpc_xy_vp;P.mpc_xy_vp;P.mpc_z_vp];
    Iv=[P.mpc_xy_vi;P.mpc_xy_vi;P.mpc_z_vi]; Dv=[P.mpc_xy_vd;P.mpc_xy_vd;P.mpc_z_vd];
    vsp=(rp-pos).*Kp+rv;
    vxy=hypot(vsp(1),vsp(2)); if vxy>P.mpc_xy_vmax, vsp(1:2)=vsp(1:2)*(P.mpc_xy_vmax/vxy); end
    vsp(3)=clamp(vsp(3),-P.mpc_z_vmax_up,P.mpc_z_vmax_dn);
    verr=vsp-vel;
    aD=(P.mpc_vd_lpf*dt)/(1+P.mpc_vd_lpf*dt); C.vdot=C.vdot+aD*((vel-C.vprev)/dt-C.vdot); C.vprev=vel;
    accsp=verr.*Pv+C.vint-C.vdot.*Dv+ra;
    thn=P.mpc_thr_hover*(g-accsp(3))/g;
    [phi_d,theta_d]=acc_to_att(accsp(1),accsp(2),yaw,P,P.mpc_tilt);
    thn=thn/max(cos(phi_d)*cos(theta_d),0.5);
    tn=clamp(thn,P.mpc_thr_min,P.mpc_thr_max);
    if is_air
        ki=Iv;
        if (tn>=P.mpc_thr_max&&verr(3)<0)||(tn<=P.mpc_thr_min&&verr(3)>0), ki(3)=0; end
        C.vint=C.vint+verr.*ki*dt; C.vint(1:2)=clamp(C.vint(1:2),-g,g); C.vint(3)=clamp(C.vint(3),-g,g);
    end
end
function [phi_d,theta_d]=acc_to_att(ax,ay,yaw,P,tilt_deg)
    g=P.g; tl=deg2rad(tilt_deg); c=cos(yaw); s=sin(yaw);
    a_fwd=c*ax+s*ay; a_rgt=-s*ax+c*ay;
    theta_d=clamp(atan2(-a_fwd,g),-tl,tl); phi_d=clamp(atan2(a_rgt,g),-tl,tl);
end
function tn=invert_thrust(P,U1)
    KQ=P.thr_kq; KL=P.thr_kl; U=max(U1,0);
    if KQ>1, if KL>0.01, tn=(-KL+sqrt(max(KL*KL+4*KQ*U,0)))/(2*KQ); else, tn=sqrt(U/KQ); end
    else, tn=U/P.f_max; end
end
function [rp,rv,ra,yaw]=ref_traj(t,S)
    if t<=S.t0, a=0; ad=0; dd=0;
    elseif t<S.t0+S.Tr, u=(t-S.t0)/S.Tr; a=S.H*(3*u^2-2*u^3); ad=S.H*(6*u-6*u^2)/S.Tr; dd=S.H*(6-12*u)/S.Tr^2;
    else, a=S.H; ad=0; dd=0; end
    rp=[S.x_sp;S.y_sp;-a]; rv=[0;0;-ad]; ra=[0;0;-dd]; yaw=S.yaw0;
end
function m=true_mass(t,S), m=S.empty + (t<S.t_drop)*S.payload; end
function y=clamp(x,a,b), y=min(max(x,a),b); end

function M=compute_metrics(L,P,S,MET)
    t=L.t; e=L.err_mm; td=S.t_drop; dt=S.dt; air=L.airborne;
    pre =(t>=td-5)&(t<td-0.5); post=(t>=td)&(t<=td+min(10,S.T-td));
    M.pre_rmse_z=rms_(e(pre)); M.pre_sigz=mean(L.sigma_z(pre));
    M.pre_sigz_std = std(L.sigma_z(pre));
    if any(air), M.etc_rate = mean(L.etc(air)); else, M.etc_rate = NaN; end
    ep=e(post); tp_=t(post);
    if isempty(ep), M.trans_ise=1e9; M.delta_err_pct=1e9; M.rmse_z_air=NaN; return; end
    M.trans_ise=sum(ep.^2)*dt;
    if std(L.m_est)>0.01
        st=(abs(L.vel_z)<0.3);
        mo_before=median(L.m_est((t<td-1)&st)); mo_after=median(L.m_est((t>t(end)-5)&st));
        detected_delta=mo_before-mo_after;
        M.delta_err_pct=abs(detected_delta-S.payload)/S.payload*100;
    else, M.delta_err_pct=1e9; end
    M.rmse_z_air=rms_(e(air));
end
function r=rms_(x), if isempty(x), r=NaN; else, r=sqrt(mean(x.^2)); end, end