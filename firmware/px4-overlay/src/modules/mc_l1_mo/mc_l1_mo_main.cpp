/****************************************************************************
 *
 * mc_l1_mo_main.cpp
 *
 * L1 Adaptive Control + Momentum Observer + Event-Triggered Control
 *
 * ARCHITECTURE:
 *   Mass estimation: Momentum Observer only (_mo_m_est).
 *   Hover estimator removed — it required waiting 2+ seconds at quasi-hover,
 *   introduced a discrete switch between m_nom / hover / MO that caused
 *   sudden step changes in feedforward thrust, and depended on a valid-flag
 *   gate that blocked MO from feeding the controller during early flight.
 *
 *   MO feeds L1 directly and continuously:
 *     - _mo_m_est used in state predictor (u_eff) and force conversion (u_raw)
 *       from the first airborne cycle (no validity delay)
 *     - Sigma warm-start: when MO detects |Δm| > MO_DTH, sigma_z is adjusted
 *       by Δσ = −Δm·g/m_new in the same cycle, eliminating multi-second
 *       adaptation lag after a payload drop
 *
 * BUG FIXES (vs original uploaded version):
 *   BUG 1 (hpp): _mo_m_est init {5.2f} → {2.064f}
 *   BUG 2 (hpp+cpp): _yaw_target never declared as member; added to hpp
 *   BUG 3 (cpp): _was_airborne_prev updated before second takeoff check,
 *                making yaw lock never activate; fixed execution order
 *   BUG 4 (cpp): parameters_update() reset mass estimates unconditionally;
 *                guarded with if (!_initialized)
 *
 ****************************************************************************/

#include "mc_l1_mo_main.hpp"

ModuleBase::Descriptor McL1MoMain::desc{McL1MoMain::task_spawn, McL1MoMain::custom_command, McL1MoMain::print_usage};

static constexpr float GRAVITY         = 9.81f;
static constexpr float DT_MAX          = 0.05f;
static constexpr float DT_MIN          = 0.001f;
static constexpr float Z_GROUND_THRESH = 0.10f;
static constexpr float Z_AIR_HI = 0.12f;   // naik > 0.12 m → airborne
static constexpr float Z_AIR_LO = 0.06f;   // turun < 0.06 m → ground (hysteresis)
static constexpr float KI_XY      = 0.00f;  // <-- WIRE pakai KI_XY yg sudah kamu pakai
static constexpr float INT_XY_LIM = 1.50f;  // clamp |integrator| XY  [m/s^2]
static constexpr float VZ_FREEZE  = 0.30f;  // |vz_sp| > ini => climb/descent => FREEZE [m/s]
static constexpr float THR_FREEZE = 0.80f;  // collective > ini => dekat saturasi => FREEZE [-]
static constexpr float THR_CAP    = 0.85f;  // sisakan ~0.15 utk roll/pitch [-]


McL1MoMain::McL1MoMain() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
    _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
    parameters_update();
}

McL1MoMain::~McL1MoMain()
{
    perf_free(_loop_perf);
}

bool McL1MoMain::init()
{
    ScheduleOnInterval(10_ms);   // 100 Hz
    PX4_INFO("mc_l1_mo started at 100Hz");
    return true;
}

void McL1MoMain::parameters_update()
{
    ModuleParams::updateParams();
    if (!_initialized) {
        _mo_m_est  = _param_m_nom.get();
        _mo_m_prev = _param_m_nom.get();
    }
}

void McL1MoMain::Run()
{
    if (should_exit()) {
        ScheduleClear();
        return;
    }

    perf_begin(_loop_perf);

    // ── Parameter update check ──
    if (_parameter_update_sub.updated()) {
        parameter_update_s pupdate;
        _parameter_update_sub.copy(&pupdate);
        parameters_update();
    }

    // ── Read vehicle local position ──
    vehicle_local_position_s lpos;
    if (_local_pos_sub.copy(&lpos)) {
        if (PX4_ISFINITE(lpos.x) && PX4_ISFINITE(lpos.y) && PX4_ISFINITE(lpos.z)) {
            _pos[0] = lpos.x; _pos[1] = lpos.y; _pos[2] = lpos.z;
        }
        if (PX4_ISFINITE(lpos.vx) && PX4_ISFINITE(lpos.vy) && PX4_ISFINITE(lpos.vz)) {
            _vel[0] = lpos.vx; _vel[1] = lpos.vy; _vel[2] = lpos.vz;
        }
    }

    // ── Read attitude for yaw extraction ──
    vehicle_attitude_s att_now;
    if (_attitude_sub.copy(&att_now)) {
        const float qw = att_now.q[0], qx = att_now.q[1];
        const float qy = att_now.q[2], qz = att_now.q[3];
        const float yaw_cand = atan2f(2.0f*(qw*qz + qx*qy),
                                       1.0f - 2.0f*(qy*qy + qz*qz));
        if (PX4_ISFINITE(yaw_cand)) _yaw_actual = yaw_cand;

        // roll/pitch — TELEMETRI SAJA (dikirim via l1_mo_debug ke monitor).
        // vehicle_attitude TIDAK di-bridge keluar oleh uxrce_dds_client di
        // build ini (sama seperti vehicle_attitude_setpoint), jadi ROS2 gak
        // pernah dapat data ini langsung. Ini satu-satunya jalan roll_act_deg
        // / pitch_act_deg di CSV monitor bisa terisi.
        const float roll_cand  = atan2f(2.0f*(qw*qx + qy*qz),
                                         1.0f - 2.0f*(qx*qx + qy*qy));
        const float pitch_cand = asinf(fmaxf(-1.0f, fminf(1.0f,
                                         2.0f*(qw*qy - qz*qx))));
        if (PX4_ISFINITE(roll_cand))  _roll_actual  = roll_cand;
        if (PX4_ISFINITE(pitch_cand)) _pitch_actual = pitch_cand;
    }

    // ── Check if offboard or position mode active ──
    vehicle_control_mode_s ctrl_mode;
    _control_mode_sub.copy(&ctrl_mode);
    const bool mode_ok = ctrl_mode.flag_control_offboard_enabled
                      || ctrl_mode.flag_control_position_enabled;
    if (!mode_ok || !ctrl_mode.flag_armed) {
        _initialized = false;
        perf_end(_loop_perf);
        return;
    }

    // ── Read trajectory setpoint ──
    trajectory_setpoint_s traj_sp;
    if (_traj_sp_sub.copy(&traj_sp)) {
        for (int i = 0; i < 3; i++) {
            _sp_pos[i] = PX4_ISFINITE(traj_sp.position[i])     ? traj_sp.position[i]     : _pos[i];
            _sp_vel[i] = PX4_ISFINITE(traj_sp.velocity[i])     ? traj_sp.velocity[i]     : 0.f;
            _sp_acc[i] = PX4_ISFINITE(traj_sp.acceleration[i]) ? traj_sp.acceleration[i] : 0.f;
        }
    }

    // ── Compute dt ──
    const hrt_abstime now = hrt_absolute_time();
    float dt;
    if (_last_run == 0) {
        dt = 0.01f;
    } else {
        dt = math::constrain((float)(now - _last_run) * 1e-6f, DT_MIN, DT_MAX);
    }
    _last_run = now;

    // ── Initialize on first activation ──
    if (!_initialized) {
        for (int i = 0; i < 3; i++) {
            _l1_z_hat[i]  = _pos[i];
            _l1_vz_hat[i] = _vel[i];
            _l1_sigma[i]  = 0.f;
        }
        const float m_init = _param_m_nom.get();
        _l1_u_filt[0] = 0.f;
        _l1_u_filt[1] = 0.f;
        _l1_u_filt[2] = m_init * GRAVITY;
        _mo_m_est      = m_init;
        _mo_m_prev     = m_init;
        _mo_p_hat      = m_init * _vel[2];   // p_hat = m0*vz → residual mulai dari 0
        _mo_residual   = 0.f;
        _sigma_jump_pending = false;
        _sigma_jump_delta_z = 0.f;
        _yaw_target    = _yaw_actual;
        _initialized   = true;
        _was_airborne_prev = false;
        _first_takeoff_done = false;   // re-arm reset σ tiap init/arm baru
        _l1_int_xy[0] = _l1_int_xy[1] = 0.f;
        PX4_INFO("mc_l1_mo init | m_nom=%.3f kg | yaw=%.1f deg",
                 (double)m_init, (double)math::degrees(_yaw_actual));
    }

    // ── Airborne detection (hysteresis: cegah chattering dekat titik ambil) ──
    //   Tanpa hysteresis, ketinggian berosilasi di sekitar ambang bikin
    //   is_airborne berkedip → memicu reset σ dan menghapus kompensasi angin.
    const float h_agl = -_pos[2];                         // altitude (m), positif ke atas
    const bool is_airborne = _was_airborne_prev
                           ? (h_agl > Z_AIR_LO)           // sudah airborne → tahan sampai turun < 6 cm
                           : (h_agl > Z_AIR_HI);          // di tanah      → angkat jika naik > 12 cm
    _is_airborne = is_airborne;

    // Reset σ HANYA pada takeoff pertama misi — JANGAN saat is_airborne
    // berkedip dekat tanah (pick-drop); itu menghapus σ_xy penahan angin.
    if (is_airborne && !_was_airborne_prev && !_first_takeoff_done) {
        _l1_sigma[0] = _l1_sigma[1] = _l1_sigma[2] = 0.f;
        _first_takeoff_done = true;
        _yaw_at_takeoff     = _yaw_actual;
        _yaw_locked         = true;
        PX4_INFO("First takeoff | z=%.2f m | sigma init | yaw locked=%.1f deg",
                 (double)_pos[2], (double)math::degrees(_yaw_at_takeoff));
    }
    if (!is_airborne) {
        _yaw_locked = false;
    }
    _was_airborne_prev = is_airborne;

    // ── Determine yaw target ──
    if (_traj_sp_sub.copy(&traj_sp) && PX4_ISFINITE(traj_sp.yaw)) {
        _yaw_target       = traj_sp.yaw;
        _yaw_sp_from_traj = true;
    } else if (_yaw_locked) {
        _yaw_target       = _yaw_at_takeoff;
        _yaw_sp_from_traj = false;
    } else {
        _yaw_target       = _yaw_actual;
        _yaw_sp_from_traj = false;
    }

    // ── Momentum Observer ──
    // Runs before L1 so _mo_m_est is fresh when l1_update_axis() reads it.
    // Ground: skip — acceleration noise on ground corrupts the estimate.
    if (_param_mo_en.get() && is_airborne) {
        mo_update(_vel[2], _l1_u_filt[2], dt);
    }

    // ── L1 update per axis ──
    // Uses _mo_m_est for state predictor (u_eff) and force conversion (u_raw).
    for (int axis = 0; axis < 3; axis++) {
        l1_update_axis(axis, _pos[axis], _vel[axis],
                       _sp_pos[axis], _sp_vel[axis], _sp_acc[axis], dt);
    }

    // Periodic mass log (2 s interval)
    static uint64_t _last_mass_log = 0;
    if (now - _last_mass_log > 2000000ULL) {
        PX4_INFO("MASS | mo_est=%.3f kg | sigma_z=%.3f",
                 (double)_mo_m_est, (double)_l1_sigma[2]);
        _last_mass_log = now;
    }

    // ── Compose attitude setpoint ──
    vehicle_attitude_setpoint_s att_sp{};
    att_sp.timestamp = now;
    att_sp.yaw_sp_move_rate = 0.f;

    float U1_force = math::constrain(_l1_u_filt[2], 0.f, _param_f_max.get());

    float acc_cmd[3] = {_l1_u_filt[0], _l1_u_filt[1], _l1_u_filt[2]};


    acc_to_attitude_sp(acc_cmd, U1_force, att_sp);

    // ── ETC throttling ──
    float phi_d_out, theta_d_out;
    float U1_d_out = U1_force;
    {
        const float qw = att_sp.q_d[0], qx = att_sp.q_d[1];
        const float qy = att_sp.q_d[2], qz = att_sp.q_d[3];
        phi_d_out = atan2f(2.f*(qw*qx + qy*qz), 1.f - 2.f*(qx*qx + qy*qy));
        float sp_arg = math::constrain(2.f*(qw*qy - qz*qx), -1.f, 1.f);
        theta_d_out = asinf(sp_arg);
    }
    etc_throttle(phi_d_out, theta_d_out, U1_d_out);

    // Rebuild quaternion from ETC-throttled Euler angles + locked yaw
    const float cr = cosf(phi_d_out   * 0.5f);
    const float sr = sinf(phi_d_out   * 0.5f);
    const float cp = cosf(theta_d_out * 0.5f);
    const float sp = sinf(theta_d_out * 0.5f);
    const float cy = cosf(_yaw_target * 0.5f);
    const float sy = sinf(_yaw_target * 0.5f);
    att_sp.q_d[0] = cr*cp*cy + sr*sp*sy;
    att_sp.q_d[1] = sr*cp*cy - cr*sp*sy;
    att_sp.q_d[2] = cr*sp*cy + sr*cp*sy;
    att_sp.q_d[3] = cr*cp*sy - sr*sp*cy;

    // ── Thrust curve inversion ──
    // F = KL·tn + KQ·tn²  →  solve for tn given U1_d_out
    float thrust_norm;
    const float KQ = _param_thr_kq.get();
    const float KL = _param_thr_kl.get();
    const float U  = math::max(U1_d_out, 0.f);

    if (KQ > 1.0f) {
        if (KL > 0.01f) {
            const float disc = KL*KL + 4.0f*KQ*U;
            thrust_norm = (-KL + sqrtf(math::max(disc, 0.f))) / (2.0f*KQ);
        } else {
            thrust_norm = sqrtf(U / KQ);
        }
    } else {
        thrust_norm = U / _param_f_max.get();
    }
    thrust_norm = math::constrain(thrust_norm, 0.f, THR_CAP);
    _thr_norm_prev = thrust_norm;   // dipakai gerbang (B) di cycle berikutnya

    att_sp.thrust_body[0] = 0.f;
    att_sp.thrust_body[1] = 0.f;
    att_sp.thrust_body[2] = -thrust_norm;

    _att_sp_pub.publish(att_sp);

    // ── Publish debug topic ──
    l1_mo_debug_s dbg{};
    dbg.timestamp = hrt_absolute_time();
    for (int i = 0; i < 3; i++) {
        dbg.sigma_hat[i]    = _l1_sigma[i];
        dbg.pos_hat[i]      = _l1_z_hat[i];
        dbg.vel_hat[i]      = _l1_vz_hat[i];
        dbg.u_filt[i]       = _l1_u_filt[i];
        dbg.pred_err_pos[i] = _l1_z_hat[i]  - _pos[i];
        dbg.pred_err_vel[i] = _l1_vz_hat[i] - _vel[i];
    }
    //dbg.mo_residual_or_vzfilt = _mo_residual;  // r [N] ≈ Δm·g — sinyal inti observer
    dbg.mo_m_est   = _mo_m_est;
    dbg.mo_vz_filt = _mo_residual;   // REPURPOSED: residual r [N], bukan lagi vz terfilter
    // sigma_jump_delta_z is internal — visible via step change in sigma_hat[2]
    // and mo_m_est in the same cycle. No separate msg field needed.
    dbg.etc_triggered   = _etc_triggered ? 1u : 0u;
    dbg.etc_delta_phi   = math::degrees(_etc_delta_phi);
    dbg.etc_delta_theta = math::degrees(_etc_delta_theta);
    dbg.etc_delta_u1    = _etc_delta_u1;
    dbg.thrust_norm     = thrust_norm;
    dbg.roll_deg        = math::degrees(_roll_actual);   // TELEMETRI SAJA — vehicle_attitude
    dbg.pitch_deg       = math::degrees(_pitch_actual);  // tidak di-bridge keluar, ini jalannya
    // roll_cmd/pitch_cmd/yaw_cmd: SAMA PERSIS akar masalahnya (vehicle_attitude_setpoint
    // gak di-bridge keluar) — phi_d_out/theta_d_out di sini adalah sudut yang
    // BENERAN dipakai (sudah lewat ETC throttling), lebih akurat drpd baca ulang
    // dari att_sp.q_d. _yaw_target juga sudah dihitung di atas, tinggal pakai.
    dbg.roll_cmd_deg    = math::degrees(phi_d_out);
    dbg.pitch_cmd_deg   = math::degrees(theta_d_out);
    dbg.yaw_cmd_deg     = math::degrees(_yaw_target);
    dbg.is_airborne     = is_airborne ? 1u : 0u;
    dbg.mo_enabled      = (uint8_t)_param_mo_en.get();
    _l1_debug_pub.publish(dbg);

    perf_end(_loop_perf);
}

// ─────────────────────────────────────────────────────────────
//  mo_update: Momentum Observer (De Luca–Mattone, external-force form)
//
//  Mengestimasi gaya vertikal eksternal (berat payload) lewat residual
//  generalized-momentum, lalu memulihkan massa — TANPA mendiferensiasi
//  kecepatan. Inilah ciri pembeda momentum observer: ia mengintegrasikan
//  dinamika momentum dan memakai residual feedback, jadi tidak perlu
//  estimasi akselerasi yang noisy.
//
//  Dinamika vertikal (NED, down +), nominal mass m0:
//      p        = m0 * vz                 (nominal momentum)
//      tau      = m0*g - U1               (modeled generalized force)
//      d/dt p   = tau + f_ext             (f_ext = gaya tak-termodel)
//
//  Observer:
//      r           = Ko * (p - p_hat)     (residual)
//      d/dt p_hat  = tau + r
//  Dinamika residual: d/dt r = Ko*(f_ext - r)
//      => r = LPF orde-1 dari f_ext, bandwidth Ko. Tidak ada d/dt(vz).
//
//  Pemulihan massa (quasi-static, a_z ≈ 0 saat hover/drop):
//      f_ext ≈ Δm*g   =>   m_est = m0 + r/g
//  Saat akselerasi vertikal: f_ext = Δm*(g - a_z); suku a_z adalah error
//  orde-2 yang hanya dominan di transien cepat. Inilah perilaku
//  MO-convergence-limited yang sudah Anda dokumentasikan — bukan defect.
//
//  Sigma warm-start: pada |Δm| > MO_DTH → koreksi sigma_z seketika:
//      Δσ_z = −Δm·g / m_new
// ─────────────────────────────────────────────────────────────
void McL1MoMain::mo_update(float vz_meas, float U1_vert, float dt)
{
    const float m0 = _param_m_nom.get();
    const float Ko = _param_mo_ko.get();          // observer bandwidth [1/s]

    // ── Momentum residual observer (tanpa pengukuran akselerasi) ──
    const float p   = m0 * vz_meas;               // nominal momentum
    const float tau = m0 * GRAVITY - U1_vert;     // modeled generalized force (NED)
    _mo_residual    = Ko * (p - _mo_p_hat);       // r → LPF(f_ext), bandwidth Ko
    _mo_p_hat      += dt * (tau + _mo_residual);  // integrasi observer

    if (!PX4_ISFINITE(_mo_p_hat)) {               // guard NaN / windup
        _mo_p_hat    = m0 * vz_meas;
        _mo_residual = 0.f;
    }

    // ── Pemulihan massa dari residual (quasi-static) ──
    float m_meas = m0 + _mo_residual / GRAVITY;
    m_meas = math::constrain(m_meas, _param_m_min.get(), _param_m_max.get());
    _mo_m_est = m_meas;                            // r sudah ter-filter orde-1
    if (!PX4_ISFINITE(_mo_m_est)) _mo_m_est = m0;

    // ── Sigma warm-start saat transien massa (logika dipertahankan) ──
    const float mass_jump = _mo_m_est - _mo_m_prev;
    if (fabsf(mass_jump) > _param_mo_dth.get()) {
        _sigma_jump_delta_z = -(mass_jump * GRAVITY) / math::max(_mo_m_est, 0.5f);
        _sigma_jump_pending = true;
        PX4_INFO("MO transient: Δm=%.3f kg → sigma warm-start Δσ=%.3f m/s²",
                 (double)mass_jump, (double)_sigma_jump_delta_z);
    }
    _mo_m_prev = _mo_m_est;
}

// ─────────────────────────────────────────────────────────────
//  l1_update_axis: L1 Adaptive Control per axis
//
//  For axis Z, _mo_m_est is used in two places:
//    1. State predictor u_eff: converts thrust (N) to acceleration (m/s²)
//       so the predictor runs in the same space as the real plant
//    2. Force conversion u_raw: converts PD acceleration output back to N
//       so the LPF output _l1_u_filt[2] is always in Newtons
//
//  Sigma warm-start (from mo_update) is applied here on axis Z,
//  consuming the _sigma_jump_pending flag exactly once per transient.
// ─────────────────────────────────────────────────────────────

void McL1MoMain::l1_update_axis(int axis,
                                 float pos, float vel,
                                 float ref_pos, float ref_vel, float ref_acc,
                                 float dt)
{
    float gamma, lpf, kobs, kp, kd, sigma_lim;
    if (axis < 2) {
        gamma = (axis == 0) ? _param_gamma_x.get() : _param_gamma_y.get();
        lpf   = _param_lpf_xy.get();  kobs = _param_kobs_xy.get();
        kp    = _param_kp_xy.get();   kd   = _param_kd_xy.get();
        sigma_lim = _param_sigma_lim.get();
    } else {
        gamma = _param_gamma_z.get(); lpf  = _param_lpf_z.get();
        kobs  = _param_kobs_z.get();  kp   = _param_kp_z.get();
        kd    = _param_kd_z.get();    sigma_lim = _param_sigma_lim.get();
    }

    const float e_z  = _l1_z_hat[axis] - pos;
    const float e_vz = _l1_vz_hat[axis] - vel;

    // ── State predictor ──
    float u_eff;
    if (axis < 2) {
        u_eff = _l1_u_filt[axis];
    } else {
        u_eff = GRAVITY - _l1_u_filt[2] / math::max(_mo_m_est, 0.5f);
    }

    _l1_z_hat[axis]  += dt * (_l1_vz_hat[axis] - kobs * e_z);
    _l1_vz_hat[axis] += dt * (u_eff + _l1_sigma[axis] - kobs * e_vz);
    if (!PX4_ISFINITE(_l1_z_hat[axis]))  _l1_z_hat[axis]  = pos;
    if (!PX4_ISFINITE(_l1_vz_hat[axis])) _l1_vz_hat[axis] = vel;

    // ── [PATCH] Saturation pre-check untuk anti-windup (XY saja) ──
    // Prediksi apakah output kontrol akan jenuh siklus ini dengan sigma saat ini.
    const bool is_xy = (axis < 2);
    float a_max = 0.f;
    bool  sat_hi = false, sat_lo = false;
    if (is_xy) {
        a_max = GRAVITY * tanf(math::radians(_param_tilt_max.get())) * 1.4f;
        const float u_pre = kp*(ref_pos - pos) + kd*(ref_vel - vel) + ref_acc
                            + _l1_int_xy[axis] - _l1_sigma[axis];
        sat_hi = (u_pre >  a_max);
        sat_lo = (u_pre < -a_max);
    }

    // ── [PATCH] Adaptasi dengan σ-modification + conditional integration ──
    if (_is_airborne) {
        const float sig_mod = _param_sig_mod.get();   // leakage (default 0.5)

        // σ-modification: damping −sig_mod·σ mencegah drift & railing.
        // Saat e_vz kecil tapi konsisten (penyebab limit cycle), term ini
        // menahan sigma agar tidak terintegrasi sampai ±SIG_LIM.
        float dsigma = -gamma * e_vz - gamma * sig_mod * _l1_sigma[axis];

        // Conditional integration: blokir adaptasi yang memperdalam saturasi.
        // u = PD + ff − sigma  →  menaikkan sigma menurunkan u.
        // ── [PATCH] Integral error posisi XY: hapus steady-state error angin konstan ──
        // SESUDAH — patch (B) di-wire ke struktur kodemu:
        if (is_xy && _is_airborne) {
            const float e_pos = ref_pos - pos;

            // Phase gate (dari patch B): FREEZE saat climb/descent atau dekat saturasi.
            // _sp_vel[2] = vz_sp (sudah tersedia sebagai member).
            const bool in_transient = (fabsf(_sp_vel[2]) > VZ_FREEZE)
                                || (_thr_norm_prev > THR_FREEZE);

            if (!in_transient) {
                // Anti-windup berbasis saturasi (tetap dipertahankan, tidak dihapus).
                const bool block_hi = sat_hi && (e_pos > 0.f);
                const bool block_lo = sat_lo && (e_pos < 0.f);
                if (!block_hi && !block_lo) {
                    _l1_int_xy[axis] += KI_XY * e_pos * dt;
                    _l1_int_xy[axis]  = math::constrain(_l1_int_xy[axis], -INT_XY_LIM, INT_XY_LIM);
                }
            }
            // Jika in_transient: tidak ada yang dilakukan → _l1_int_xy[axis] FREEZE.

            // Reset hanya sebelum takeoff pertama (latch sudah ada, tidak perlu tambah).
            // Blok init di Run() sudah handle: _l1_int_xy[0] = _l1_int_xy[1] = 0.f;
        }

        if (sat_hi && dsigma < 0.f) dsigma = 0.f;  // sigma turun → u makin tinggi
        if (sat_lo && dsigma > 0.f) dsigma = 0.f;  // sigma naik  → u makin rendah

        _l1_sigma[axis] += dt * dsigma;

        // Sigma warm-start pada transien massa (axis Z) — tidak diubah
        if (axis == 2 && _sigma_jump_pending) {
            _l1_sigma[axis] += _sigma_jump_delta_z;
            _sigma_jump_pending = false;
            PX4_DEBUG("sigma_z warm-start: sigma_z=%.3f m/s²", (double)_l1_sigma[axis]);
        }

        _l1_sigma[axis] = math::constrain(_l1_sigma[axis], -sigma_lim, sigma_lim);
    }

    // ── PD baseline + feedforward − disturbance estimate ──
    float u_raw = kp * (ref_pos - pos) + kd * (ref_vel - vel) + ref_acc;
    if (is_xy) u_raw += _l1_int_xy[axis];
    u_raw -= _l1_sigma[axis];


    if (is_xy) {
        u_raw = math::constrain(u_raw, -a_max, a_max);
    } else {
        u_raw = _mo_m_est * (GRAVITY - u_raw);
        const float f_max_curve = _param_thr_kl.get() + _param_thr_kq.get();
        const float f_max_eff   = (f_max_curve > 1.0f) ? f_max_curve : _param_f_max.get();
        u_raw = math::constrain(u_raw, 0.f, f_max_eff);
    }

    const float alpha = (lpf * dt) / (1.0f + lpf * dt);
    _l1_u_filt[axis] = (1.0f - alpha) * _l1_u_filt[axis] + alpha * u_raw;

    if (!PX4_ISFINITE(_l1_u_filt[axis])) {
        _l1_u_filt[axis] = (axis == 2) ? (_mo_m_est * GRAVITY) : 0.f;
    }
}

void McL1MoMain::acc_to_attitude_sp(const float acc_cmd[3], float thrust_force,
                                     vehicle_attitude_setpoint_s &att_sp)
{
    const float tilt_max = math::radians(_param_tilt_max.get());

    const float c = cosf(_yaw_target);
    const float s = sinf(_yaw_target);
    const float a_fwd =  c * acc_cmd[0] + s * acc_cmd[1];
    const float a_rgt = -s * acc_cmd[0] + c * acc_cmd[1];

    float theta_d = math::constrain(atan2f(-a_fwd, GRAVITY), -tilt_max, tilt_max);
    float phi_d   = math::constrain(atan2f( a_rgt, GRAVITY), -tilt_max, tilt_max);

    const float cr = cosf(phi_d   * 0.5f);
    const float sr = sinf(phi_d   * 0.5f);
    const float cp = cosf(theta_d * 0.5f);
    const float sp = sinf(theta_d * 0.5f);
    const float cy = cosf(_yaw_target * 0.5f);
    const float sy = sinf(_yaw_target * 0.5f);

    att_sp.q_d[0] = cr*cp*cy + sr*sp*sy;
    att_sp.q_d[1] = sr*cp*cy - cr*sp*sy;
    att_sp.q_d[2] = cr*sp*cy + sr*cp*sy;
    att_sp.q_d[3] = cr*cp*sy - sr*sp*cy;

    for (int i = 0; i < 4; i++) {
        if (!PX4_ISFINITE(att_sp.q_d[i])) {
            att_sp.q_d[0] = 1.f; att_sp.q_d[1] = att_sp.q_d[2] = att_sp.q_d[3] = 0.f;
            break;
        }
    }
    (void)thrust_force;
}

void McL1MoMain::etc_throttle(float &phi_d, float &theta_d, float &U1_d)
{
    const float dxy = math::radians(_param_etc_xy.get());
    const float dz  = _param_etc_z.get();
    bool triggered  = false;

    _etc_delta_phi   = fabsf(phi_d   - _etc_phi_last);
    _etc_delta_theta = fabsf(theta_d - _etc_theta_last);
    _etc_delta_u1    = fabsf(U1_d    - _etc_U1_last);

    if (_etc_delta_phi >= dxy) {
        _etc_phi_last = phi_d;
        triggered = true;
    } else {
        phi_d = _etc_phi_last;
    }
    if (_etc_delta_theta >= dxy) {
        _etc_theta_last = theta_d;
        triggered = true;
    } else {
        theta_d = _etc_theta_last;
    }
    if (_etc_delta_u1 >= dz) {
        _etc_U1_last = U1_d;
        triggered = true;
    } else {
        U1_d = _etc_U1_last;
    }

    _etc_triggered = triggered;
}

// ─────────────────────────────────────────────────────────────
//  Module boilerplate
// ─────────────────────────────────────────────────────────────

int McL1MoMain::task_spawn(int argc, char *argv[])
{
    McL1MoMain *instance = new McL1MoMain();
    if (instance) {
        desc.object.store(instance);
        desc.task_id = task_id_is_work_queue;
        if (instance->init()) return PX4_OK;
    } else {
        PX4_ERR("alloc failed");
    }
    delete instance;
    desc.object.store(nullptr);
    desc.task_id = -1;
    return PX4_ERROR;
}

int McL1MoMain::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int McL1MoMain::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s\n", reason);
    PRINT_MODULE_DESCRIPTION(R"DESC_STR(
### Description
L1 Adaptive Position Controller with Momentum Observer mass estimation
and Event-Triggered Control. MO estimate feeds directly into L1 state
predictor and force conversion. Subscribes to trajectory_setpoint,
publishes vehicle_attitude_setpoint.
)DESC_STR");
    PRINT_MODULE_USAGE_NAME("mc_l1_mo", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

extern "C" __EXPORT int mc_l1_mo_main(int argc, char *argv[])
{
    return McL1MoMain::main(McL1MoMain::desc, argc, argv);
}
