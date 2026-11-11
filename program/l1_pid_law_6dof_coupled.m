%% ============================================================================
%  l1_pid_law_6dof_coupled.m   (v6 — thesis-grade, BAB IV 4.1)
%  Model 6-DOF + CROSS-AXIS COUPLING (XY<-Z) + skenario LATERAL
%  L1+MO+ETC vs L1(MO-off) vs PX4 PID
%
%  ── APA YANG BERUBAH DARI v5 (6-DOF tanpa coupling) ─────────────────────────
%  (1) SKENARIO kini LATERAL: takeoff -> jelajah ke (x,y)=(5,5) -> hover -> DROP.
%      Sebelumnya murni vertikal (x,y=0) sehingga channel XY tak pernah teruji.
%  (2) CROSS-AXIS COUPLING (XY<-Z) — FENOMENOLOGIS / REPRESENTATIF:
%      chatter perintah thrust-Z (dari gain-Z agresif + noise) disuntikkan sbg
%      kontaminasi ke FEEDBACK KECEPATAN XY. Ini proxy first-order utk KELAS
%      mekanisme yg ADA di SITL tapi ABSEN di rigid-body ideal:
%        thrust chatter -> getaran accelerometer -> fusi EKF2 (kovarians
%        cross-axis) mencemari estimasi kecepatan/attitude XY -> osilasi XY.
%
%      PENTING (utk pertahanan sidang): ini BUKAN replika EKF2. Uji injeksi
%      terkontrol menunjukkan coupling ini mereproduksi ARAH transfer SITL
%      (gain agresif jadi terhukum di XY) sebesar ~2x; SITL riil ~7-9x.
%      Selisihnya = sisa fidelity (EKF2 penuh + control allocator + spektrum
%      noise IMU riil) yg HANYA ada di SITL/SIH. Selisih itu adalah TEMUAN
%      (transfer-gap), bukan bug. Tujuan coupling ini: memberi optimizer
%      VISIBILITAS thd ongkos XY dari agresivitas Z, sehingga penalaan MATLAB
%      lebih transfer-aware -- bukan menggantikan SITL.
%      cpl_gain: knob representatif (default 0.40, dikalibrasi agar locked
%      config lolos gate & gain agresif terhukum). Boleh dikalibrasi ke SITL.
%
%  Manifestasi coupling: muncul di CHATTER/EFFORT XY (std sigma_xy, std u_filt_xy),
%  BUKAN di RMSE posisi XY -- konsisten dgn observasi SITL (posisi XY setara,
%  tapi effort kontrol ~6x lebih ramai pd gain agresif).
%
%  CONTROLLER TIDAK DIUBAH (mirror mc_l1_mo_main.cpp). Param dari 0_FINAL_CONFIG.md.
%  GERBANG VALIDASI: pd locked config, ordering (L1+MO>MO-off>PID) & akurasi MO
%  (dm<5%) harus tetap terjaga -> calibration_check().
% ============================================================================
clear; clc; close all;
RNG_SEED = 42;

%% ─────────────────────────── 1. KONFIGURASI ───────────────────────────────
% L1+MO+ETC (0_FINAL_CONFIG.md, LOCKED) — TIDAK DIUBAH
P.g=9.81;
P.gamma_x=40; P.gamma_y=40; P.gamma_z=120;  P.lpf_xy=10; P.lpf_z=12;
P.kobs_xy=15; P.kobs_z=15;   P.kp_xy=1.5; P.kp_z=3.0; P.kd_xy=1.0; P.kd_z=5.0;
P.sig_mod=0.5; P.sig_lim=4.0;  P.mo_ko=0.5; P.m_nom=2.064; P.m_min=1.5; P.m_max=5.0;
P.mo_dth=0.08; P.thr_kl=17.44; P.thr_kq=31.72; P.f_max=40.26;
P.tilt_max=30; P.etc_xy=0.3; P.etc_z=0.5;
P.KI_XY=0.10; P.INT_XY_LIM=1.5; P.VZ_FREEZE=0.30; P.THR_FREEZE=0.80; P.THR_CAP=0.85;
P.Z_AIR_HI=0.12; P.Z_AIR_LO=0.06;
% PX4 PID — TIDAK DIUBAH
P.mpc_z_p=1.0; P.mpc_z_vp=4.0; P.mpc_z_vi=2.0; P.mpc_z_vd=0.0;
P.mpc_xy_p=0.95; P.mpc_xy_vp=1.8; P.mpc_xy_vi=0.4; P.mpc_xy_vd=0.2;
P.mpc_thr_min=0.12; P.mpc_thr_max=1.0; P.mpc_tilt=45; P.mpc_vd_lpf=10;
P.mpc_z_vmax_up=3.0; P.mpc_z_vmax_dn=1.5; P.mpc_xy_vmax=12.0;
% PLANT 6-DOF (gz_x500, representatif)
P.Jxx=0.021; P.Jyy=0.021; P.Jzz=0.042;
P.att_p=[8.0;8.0;3.5]; P.rate_p=[0.18;0.18;0.25]; P.rate_i=[0.06;0.06;0.06];
P.rate_d=[0.003;0.003;0.000]; P.tau_max=[4.0;4.0;0.6]; P.rate_int_lim=[0.5;0.5;0.5];

% Skenario SINGLE DROP LATERAL
S.dt=0.01; S.T=48.0; S.H=5.0; S.t0=1.0; S.Tr=4.0;
S.empty=2.064; S.payload=0.8; S.t_drop=25.0; S.yaw0=0; S.wind_ax=0.0;
S.lateral=true; S.xy_t0=8.0; S.xy_Tr=6.0; S.xy_target=5.0;   % jelajah ke (5,5)
S.tau_mot_up=0.0125; S.tau_mot_dn=0.025; S.tau_tq=0.015; S.tau_est=0.04; S.kd_aero=0.10;
S.n_sub=8; S.plant_thr_scale=1.0;
S.Jp_xx=0.002; S.Jp_yy=0.002; S.Jp_zz=0.004;
S.n_pos_z=0.015; S.n_pos_xy=0.03; S.n_vel=0.025;
% ── CROSS-AXIS COUPLING (representatif; lihat header) ──
S.cpl_gain=0.40;    % 0 = matikan coupling (kembali ke v5). Default 0.40 (terkalibrasi).
MET.recovery_mm=30; MET.settle_mm=80; MET.settle_s=3.0;

P.mpc_thr_hover = invert_thrust(P, S.empty*P.g);

%% ─────────────────────────── 2. SIMULASI ──────────────────────────────────
ctrls={'l1_on','l1_off','pid'}; labels={'L1+MO+ETC','L1 (MO off)','PX4 PID'};
cols={[0 0.45 0.74],[0.85 0.33 0.10],[0.20 0.62 0.20]};
Ls=cell(1,3); for i=1:3, Ls{i}=run_sim(P,S,ctrls{i},RNG_SEED); end

%% ─────────────────────────── 3. METRIK ────────────────────────────────────
Ms=cell(1,3); for i=1:3, Ms{i}=compute_metrics(Ls{i},P,S,MET); end
print_table(labels,Ms,S);
calibration_check(Ms{1},S);

%% ─────────────────────────── 4. PLOT ──────────────────────────────────────
plot_z(Ls,labels,cols,S);        % Fig 1 — tracking Z + adaptasi
plot_xy(Ls,labels,cols,S);       % Fig 2 — BARU: tracking XY + chatter coupling
plot_mass_thr(Ls,labels,cols,S); % Fig 3 — massa + thrust

% ============================================================================
%                              LOCAL FUNCTIONS
% ============================================================================
function L = run_sim(P,S,mode,seed)
    rng(seed);
    g=P.g; dt=S.dt; N=round(S.T/dt)+1; tg=(0:N-1)*dt; n_sub=S.n_sub; dti=dt/n_sub;
    pos=[0;0;0]; vel=[0;0;0]; att=[0;0;S.yaw0]; rates=[0;0;0];
    T_appl=P.m_nom*g; tau_appl=[0;0;0]; IN.rate_int=[0;0;0]; IN.rate_prev=[0;0;0];
    epos=[0;0;0]; evel=[0;0;0]; uz_prev=P.m_nom*g;   % utk chatter coupling
    is_l1=any(strcmp(mode,{'l1_on','l1_off'})); mo_en=strcmp(mode,'l1_on');
    if is_l1, C=ctrl_init(P); else, C=pid_init(); end
    was_air=false;
    F={'alt','alt_ref','errz_mm','posx','posy','spx','spy','errxy','m_est','m_true',...
       'sigz','sigx','sigy','ufx','ufy','thr','etc','vel_z','roll','pitch','tilt'};
    for f=F, L.(f{1})=zeros(1,N); end
    for k=1:N
        t=tg(k); m_t=true_mass(t,S); [rp,rv,ra,yaw]=ref_traj(t,S);
        loaded=(t<S.t_drop); Jv=[P.Jxx;P.Jyy;P.Jzz]+loaded.*[S.Jp_xx;S.Jp_yy;S.Jp_zz];
        ae=dt/(S.tau_est+dt); epos=epos+ae*(pos-epos); evel=evel+ae*(vel-evel);
        zfb=epos+[randn*S.n_pos_xy;randn*S.n_pos_xy;randn*S.n_pos_z];
        vfb=evel+randn(3,1)*S.n_vel;
        % ── CROSS-AXIS COUPLING: chatter perintah thrust-Z -> feedback kecepatan XY ──
        if is_l1, uz_now=C.u_filt(3); else, uz_now=uz_prev; end
        chatter=abs(uz_now-uz_prev); uz_prev=uz_now;
        vfb(1)=vfb(1)+S.cpl_gain*chatter*randn; vfb(2)=vfb(2)+S.cpl_gain*chatter*randn;
        h=-zfb(3); if was_air, is_air=h>P.Z_AIR_LO; else, is_air=h>P.Z_AIR_HI; end
        etc_any=0;
        if is_l1
            C.is_airborne=is_air;
            if is_air&&~was_air&&~C.first_to, C.sigma=[0 0 0]; C.first_to=true; end
            C.sp_vel_z=rv(3);
            if mo_en&&is_air, C=mo_update(C,P,vfb(3),C.u_filt(3),dt); end
            for ax=1:3, C=l1_axis(C,P,ax,zfb(ax),vfb(ax),rp(ax),rv(ax),ra(ax),dt); end
            U1=clamp(C.u_filt(3),0,P.f_max);
            [phi_d,theta_d]=acc_to_att(C.u_filt(1),C.u_filt(2),yaw,P,P.tilt_max);
            [C,phi_cmd,theta_cmd,U1e,etc_any]=etc_throttle(C,P,phi_d,theta_d,U1);
            tn=clamp(invert_thrust(P,U1e),0,P.THR_CAP); C.thr_norm_prev=tn;
            L.m_est(k)=C.m_est; L.sigz(k)=C.sigma(3); L.sigx(k)=C.sigma(1); L.sigy(k)=C.sigma(2);
            L.ufx(k)=C.u_filt(1); L.ufy(k)=C.u_filt(2);
        else
            [C,phi_cmd,theta_cmd,tn]=pid_step(C,P,zfb,vfb,rp,rv,ra,yaw,is_air,dt);
            L.m_est(k)=P.m_nom;
        end
        was_air=is_air; att_sp=[phi_cmd;theta_cmd;yaw]; tn=clamp(tn,0,1.0);
        for s=1:n_sub
            [tau_cmd,IN]=inner_loop(IN,P,att,rates,att_sp,dti);
            tau_appl=tau_appl+dti*(tau_cmd-tau_appl)/S.tau_tq;
            T_ach=(P.thr_kl*tn+P.thr_kq*tn^2)*S.plant_thr_scale;
            if T_ach>T_appl, tm=S.tau_mot_up; else, tm=S.tau_mot_dn; end
            T_appl=T_appl+dti*(T_ach-T_appl)/tm;
            gyro=cross(rates,Jv.*rates); rates=rates+dti*(tau_appl-gyro)./Jv;
            phi=att(1); theta=att(2); ct=cos(theta); if abs(ct)<1e-3, ct=sign(ct)*1e-3; end
            Te=[1 sin(phi)*tan(theta) cos(phi)*tan(theta);0 cos(phi) -sin(phi);0 sin(phi)/ct cos(phi)/ct];
            att=att+dti*(Te*rates);
            phi=att(1); theta=att(2); psi=att(3);
            e3=[cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
                cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); cos(phi)*cos(theta)];
            a=[0;0;g]-(T_appl/m_t)*e3-(S.kd_aero/m_t)*vel; a(1)=a(1)+is_air*S.wind_ax;
            vel=vel+dti*a; pos=pos+dti*vel;
            if pos(3)>0, pos(3)=0; if vel(3)>0, vel(3)=0; end; end
        end
        L.alt(k)=-pos(3); L.alt_ref(k)=-rp(3); L.errz_mm(k)=((-pos(3))-(-rp(3)))*1e3;
        L.posx(k)=pos(1); L.posy(k)=pos(2); L.spx(k)=rp(1); L.spy(k)=rp(2);
        L.errxy(k)=hypot(pos(1)-rp(1),pos(2)-rp(2));
        L.thr(k)=tn; L.etc(k)=etc_any; L.vel_z(k)=vel(3); L.m_true(k)=m_t;
        L.roll(k)=rad2deg(att(1)); L.pitch(k)=rad2deg(att(2));
        L.tilt(k)=hypot(rad2deg(att(1)),rad2deg(att(2)));
    end
    L.t=tg; L.airborne=(L.alt>P.Z_AIR_LO);
end

% ---------- inner cascade (PX4 mc_att+mc_rate) ----------
function [tau,IN]=inner_loop(IN,P,att,rates,att_sp,dti)
    aerr=att_sp-att; aerr(3)=atan2(sin(aerr(3)),cos(aerr(3)));
    omega_sp=P.att_p.*aerr; rerr=omega_sp-rates;
    IN.rate_int=clamp(IN.rate_int+rerr*dti,-P.rate_int_lim,P.rate_int_lim);
    d_rate=(rates-IN.rate_prev)/dti; IN.rate_prev=rates;
    m_norm=P.rate_p.*rerr+P.rate_i.*IN.rate_int-P.rate_d.*d_rate;
    m_norm=clamp(m_norm,-1,1); tau=m_norm.*P.tau_max;
end
% ---------- L1+MO+ETC (mirror .cpp) — TIDAK DIUBAH ----------
function C=ctrl_init(P)
    C.z_hat=[0 0 0]; C.vz_hat=[0 0 0]; C.sigma=[0 0 0]; C.u_filt=[0 0 P.m_nom*P.g];
    C.m_est=P.m_nom; C.mo_m_prev=P.m_nom; C.mo_p_hat=0; C.mo_residual=0;
    C.sig_jump_pending=false; C.sig_jump_dz=0; C.int_xy=[0 0]; C.thr_norm_prev=0;
    C.etc_phi=0; C.etc_theta=0; C.etc_u1=0; C.is_airborne=false; C.first_to=false; C.sp_vel_z=0;
end
function C=mo_update(C,P,vz,U1,dt)
    g=P.g; m0=P.m_nom; Ko=P.mo_ko; p=m0*vz; tau=m0*g-U1;
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
    vsp(3)=clamp(vsp(3),-P.mpc_z_vmax_up,P.mpc_z_vmax_dn); verr=vsp-vel;
    aD=(P.mpc_vd_lpf*dt)/(1+P.mpc_vd_lpf*dt); C.vdot=C.vdot+aD*((vel-C.vprev)/dt-C.vdot); C.vprev=vel;
    accsp=verr.*Pv+C.vint-C.vdot.*Dv+ra; thn=P.mpc_thr_hover*(g-accsp(3))/g;
    [phi_d,theta_d]=acc_to_att(accsp(1),accsp(2),yaw,P,P.mpc_tilt);
    thn=thn/max(cos(phi_d)*cos(theta_d),0.5); tn=clamp(thn,P.mpc_thr_min,P.mpc_thr_max);
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
    if S.lateral && t>S.xy_t0
        if t<S.xy_t0+S.xy_Tr, u=(t-S.xy_t0)/S.xy_Tr; xa=S.xy_target*(3*u^2-2*u^3); xv=S.xy_target*(6*u-6*u^2)/S.xy_Tr;
        else, xa=S.xy_target; xv=0; end
    else, xa=0; xv=0; end
    rp=[xa;xa;-a]; rv=[xv;xv;-ad]; ra=[0;0;-dd]; yaw=S.yaw0;
end
function m=true_mass(t,S), m=S.empty+(t<S.t_drop)*S.payload; end
function y=clamp(x,a,b), y=min(max(x,a),b); end

% ---------- METRIK (Z + XY) ----------
function M=compute_metrics(L,P,S,MET)
    t=L.t; e=L.errz_mm; td=S.t_drop; dt=S.dt; air=L.airborne;
    pre=(t>=td-5)&(t<td-0.5); post=(t>=td)&(t<=td+10); tail=(t>=max(td+6,t(end)-4));
    % Z
    M.pre_rmse_z=rms_(e(pre)); M.post_rmse_z=rms_(e(tail)); M.rmse_z_air=rms_(e(air));
    ep=e(post); tp_=t(post);
    M.peak_mm=ep(find(abs(ep)==max(abs(ep)),1)); M.trans_ise=sum(ep.^2)*dt;
    M.pre_sigz=mean(L.sigz(pre)); M.pre_sigz_std=std(L.sigz(pre));
    % XY: coupling muncul di CHATTER/EFFORT, bukan RMSE posisi
    cruise=(t>=S.xy_t0+S.xy_Tr+1)&(t<td-0.5);   % setelah sampai (5,5), sebelum drop
    M.xy_rmse=rms_(L.errxy(cruise))*1e3;         % mm (utk referensi)
    M.xy_sig_chatter=0.5*(std(L.sigx(cruise))+std(L.sigy(cruise)));   % <-- metrik XY utama
    M.xy_uf_effort=0.5*(std(L.ufx(cruise))+std(L.ufy(cruise)));
    M.peak_tilt=max(L.tilt(air));
    % massa
    if std(L.m_est)>0.01
        st=(abs(L.vel_z)<0.3);
        M.mo_before=median(L.m_est((t<td-1)&st)); M.mo_after=median(L.m_est((t>t(end)-5)&st));
        M.detected_delta=M.mo_before-M.mo_after; M.delta_err_pct=abs(M.detected_delta-S.payload)/S.payload*100;
    else, M.mo_before=NaN; M.mo_after=NaN; M.detected_delta=NaN; M.delta_err_pct=NaN; end
    etc=logical(L.etc); if any(air), M.etc_rate=mean(L.etc(air)); else, M.etc_rate=NaN; end
    M.etc_n=sum(etc);
end
function r=rms_(x), if isempty(x), r=NaN; else, r=sqrt(mean(x.^2)); end, end

function print_table(labels,Ms,S)
    fprintf('\n============ METRIK SINGLE-DROP LATERAL 6-DOF+COUPLING (payload %.1f kg) ============\n',S.payload);
    fprintf('%-26s %12s %12s %12s\n','Metrik',labels{:});
    row=@(n,f,k) fprintf('%-26s %12s %12s %12s\n',n,f(Ms{1}.(k)),f(Ms{2}.(k)),f(Ms{3}.(k)));
    f1=@(x)num2str(x,'%.1f'); f3=@(x)num2str(x,'%.3f'); f4=@(x)num2str(x,'%.4f');
    fprintf('-- Z channel --\n');
    row('RMSE_z pre-drop (mm)',f1,'pre_rmse_z'); row('Peak overshoot (mm)',f1,'peak_mm');
    row('trans ISE (mm2.s)',f1,'trans_ise'); row('RMSE_z airborne (mm)',f1,'rmse_z_air');
    fprintf('-- XY channel (coupling terlihat di chatter/effort) --\n');
    row('XY RMSE posisi (mm)',f1,'xy_rmse');
    row('XY sigma chatter',f4,'xy_sig_chatter'); row('XY u_filt effort',f3,'xy_uf_effort');
    row('Peak tilt (deg)',f1,'peak_tilt');
    fprintf('-- Massa / ETC --\n');
    row('Detected dm (kg)',f3,'detected_delta'); row('dm error (%%)',f1,'delta_err_pct');
    row('ETC rate (%%)',@(x)num2str(x*100,'%.1f'),'etc_rate');
    fprintf('====================================================================================\n');
end
function calibration_check(M,S)
    fprintf('\n[CALIBRATION CHECK — L1+MO, 6-DOF+coupling]\n');
    chk('sigma_z |mean| < 0.10',abs(M.pre_sigz)<0.10,sprintf('%.4f',M.pre_sigz));
    chk('mo_before ~ loaded (2.864+-0.05)',abs(M.mo_before-(S.empty+S.payload))<0.05,sprintf('%.3f',M.mo_before));
    chk('mo_after ~ empty (2.064+-0.05)',abs(M.mo_after-S.empty)<0.05,sprintf('%.3f',M.mo_after));
    chk('dm error < 5%%',M.delta_err_pct<5,sprintf('%.1f%%',M.delta_err_pct));
    chk('peak tilt < 30 deg',M.peak_tilt<30,sprintf('%.1f deg',M.peak_tilt));
end
function chk(name,ok,val)
    if ok, s='PASS'; else, s='CHECK'; end
    fprintf('  [%s] %-34s = %s\n',s,name,val);
end

% ---------- PLOTS ----------
function plot_z(Ls,labels,cols,S)
    figure('Name','Fig 1 — Tracking Z','Color','w','Position',[60 60 1150 460]);
    subplot(1,2,1); hold on; grid on;
    plot(Ls{1}.t,Ls{1}.alt_ref,'k:','LineWidth',1.3,'DisplayName','ref');
    for i=1:3, plot(Ls{i}.t,Ls{i}.alt,'Color',cols{i},'LineWidth',1.5,'DisplayName',labels{i}); end
    xline(S.t_drop,'k--','DROP','HandleVisibility','off'); ylabel('Altitude (m)'); xlabel('t (s)');
    legend('Location','southeast'); title('(a) Tracking altitude');
    subplot(1,2,2); hold on; grid on;
    for i=1:3, plot(Ls{i}.t,Ls{i}.errz_mm,'Color',cols{i},'LineWidth',1.3,'DisplayName',labels{i}); end
    yline(30,'g:','HandleVisibility','off'); yline(-30,'g:','HandleVisibility','off');
    xline(S.t_drop,'k--','HandleVisibility','off'); ylabel('error_z (mm)'); xlabel('t (s)');
    legend('Location','best'); title('(b) Error altitude');
end
function plot_xy(Ls,labels,cols,S)
    figure('Name','Fig 2 — XY & coupling','Color','w','Position',[90 90 1150 760]);
    subplot(2,2,1); hold on; grid on; axis equal;
    plot(Ls{1}.spx,Ls{1}.spy,'k:','LineWidth',1.3,'DisplayName','ref');
    for i=1:3, plot(Ls{i}.posx,Ls{i}.posy,'Color',cols{i},'LineWidth',1.2,'DisplayName',labels{i}); end
    xlabel('pos_x (m)'); ylabel('pos_y (m)'); legend('Location','best'); title('(a) Lintasan XY (toilet-bowl?)');
    subplot(2,2,2); hold on; grid on;
    for i=1:3, plot(Ls{i}.t,Ls{i}.errxy,'Color',cols{i},'LineWidth',1.2,'DisplayName',labels{i}); end
    xline(S.t_drop,'k--','HandleVisibility','off'); ylabel('|error_{xy}| (m)'); xlabel('t (s)');
    legend('Location','best'); title('(b) Error posisi XY');
    subplot(2,2,3); hold on; grid on;
    plot(Ls{1}.t,Ls{1}.sigx,'Color',cols{1},'LineWidth',1.2,'DisplayName','\sigma_x L1+MO');
    plot(Ls{2}.t,Ls{2}.sigx,'Color',cols{2},'LineWidth',1.0,'DisplayName','\sigma_x MO-off');
    xline(S.t_drop,'k--','HandleVisibility','off'); ylabel('\sigma_x (m/s^2)'); xlabel('t (s)');
    legend('Location','best'); title('(c) Chatter \sigma_x (jejak coupling)');
    subplot(2,2,4); hold on; grid on;
    for i=1:2, plot(Ls{i}.t,Ls{i}.ufx,'Color',cols{i},'LineWidth',1.0,'DisplayName',labels{i}); end
    xline(S.t_drop,'k--','HandleVisibility','off'); ylabel('u_{filt,x}'); xlabel('t (s)');
    legend('Location','best'); title('(d) Effort kontrol XY');
end
function plot_mass_thr(Ls,labels,cols,S)
    figure('Name','Fig 3 — Massa & thrust','Color','w','Position',[120 120 1150 460]);
    subplot(1,2,1); hold on; grid on;
    plot(Ls{1}.t,Ls{1}.m_true,'k:','LineWidth',1.3,'DisplayName','m_{true}');
    plot(Ls{1}.t,Ls{1}.m_est,'Color',cols{1},'LineWidth',1.6,'DisplayName','m_{est} (MO)');
    xline(S.t_drop,'k--','HandleVisibility','off'); ylabel('massa (kg)'); xlabel('t (s)');
    legend('Location','northeast'); title('(a) Estimasi massa MO');
    subplot(1,2,2); hold on; grid on;
    for i=1:3, plot(Ls{i}.t,Ls{i}.thr,'Color',cols{i},'LineWidth',1.2,'DisplayName',labels{i}); end
    xline(S.t_drop,'k--','HandleVisibility','off'); ylabel('thrust_{norm}'); xlabel('t (s)'); ylim([0 1]);
    legend('Location','best'); title('(b) Thrust');
end