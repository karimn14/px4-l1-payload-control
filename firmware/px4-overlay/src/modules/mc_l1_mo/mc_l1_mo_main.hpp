#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/defines.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/mathlib/mathlib.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/l1_mo_debug.h>

using namespace time_literals;

class McL1MoMain : public ModuleBase,
                   public ModuleParams,
                   public px4::ScheduledWorkItem
{
public:
    McL1MoMain();
    ~McL1MoMain() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

    static Descriptor desc;

private:
    void Run() override;
    void parameters_update();

    void l1_update_axis(int axis,
                        float pos, float vel,
                        float ref_pos, float ref_vel, float ref_acc,
                        float dt);

    void mo_update(float vz_meas, float U1_vert, float dt);
    void acc_to_attitude_sp(const float acc_cmd[3], float thrust_force,
                            vehicle_attitude_setpoint_s &att_sp);
    void etc_throttle(float &phi_d, float &theta_d, float &U1_d);

    // ── L1 state per axis [X, Y, Z] ──
    float _l1_z_hat[3]   {0.f, 0.f, 0.f};
    float _l1_vz_hat[3]  {0.f, 0.f, 0.f};
    float _l1_sigma[3]   {0.f, 0.f, 0.f};
    float _l1_u_filt[3]  {0.f, 0.f, 0.f};

    // ── Momentum Observer state ──
    float _mo_m_est      {2.064f};   // current MO mass estimate [kg]
    float _mo_m_prev     {2.064f};   // estimate on previous cycle, for sigma warm-start
    float _mo_p_hat      {0.f};      // p_hat = m*vz, for residual calculation
    float _mo_residual   {0.f};      // residual = p_hat - m*vz, for logging and sigma warm-start

    // ── Mass-to-L1 sigma feedback ──
    // When MO detects a mass jump > MO_DTH, sigma_z is warm-started analytically:
    //   Δσ_z = −Δm·g / m_new
    // This collapses multi-second adaptation lag after a payload drop/pickup into
    // a single-cycle correction.
    bool  _sigma_jump_pending {false};
    float _sigma_jump_delta_z {0.f};
    float _l1_int_xy[2]{0.f, 0.f};   // integrator error posisi XY (anti steady-state wind)
    float _thr_norm_prev{0.0f};

    // ── ETC last-published values ──
    float _etc_phi_last   {0.f};
    float _etc_theta_last {0.f};
    float _etc_U1_last    {0.f};
    bool  _etc_triggered   {false};
    float _etc_delta_phi   {0.f};
    float _etc_delta_theta {0.f};
    float _etc_delta_u1    {0.f};

    // ── Vehicle state cache ──
    float _pos[3]        {0.f, 0.f, 0.f};
    float _vel[3]        {0.f, 0.f, 0.f};
    float _yaw_actual    {0.f};
    float _roll_actual   {0.f};   // rad — TELEMETRI SAJA (l1_mo_debug), gak dipakai kontrol
    float _pitch_actual  {0.f};   // rad — TELEMETRI SAJA (l1_mo_debug), gak dipakai kontrol

    // ── Yaw control state ──
    float _yaw_at_takeoff   {0.f};
    float _yaw_setpoint     {0.f};
    float _yaw_target        {0.f};
    bool  _yaw_locked       {false};
    bool  _yaw_sp_from_traj {false};

    // ── Reference cache ──
    float _sp_pos[3]     {0.f, 0.f, 0.f};
    float _sp_vel[3]     {0.f, 0.f, 0.f};
    float _sp_acc[3]     {0.f, 0.f, 0.f};

    // ── Timing & flags ──
    hrt_abstime _last_run   {0};
    bool        _initialized{false};
    bool        _was_airborne_prev{false};
    bool        _is_airborne{false};
    bool _first_takeoff_done{false};   // latch: σ hanya di-reset pada takeoff pertama

    // ── Performance counter ──
    perf_counter_t _loop_perf;

    // ── Subscriptions ──
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    uORB::Subscription _local_pos_sub     {ORB_ID(vehicle_local_position)};
    uORB::Subscription _attitude_sub      {ORB_ID(vehicle_attitude)};
    uORB::Subscription _traj_sp_sub       {ORB_ID(trajectory_setpoint)};
    uORB::Subscription _control_mode_sub  {ORB_ID(vehicle_control_mode)};
    uORB::Subscription _offboard_mode_sub {ORB_ID(offboard_control_mode)};

    // ── Publications ──
    uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Publication<l1_mo_debug_s> _l1_debug_pub{ORB_ID(l1_mo_debug)};

    // ── Parameters ──
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::L1MO_GAMMA_X>)  _param_gamma_x,
        (ParamFloat<px4::params::L1MO_GAMMA_Y>)  _param_gamma_y,
        (ParamFloat<px4::params::L1MO_GAMMA_Z>)  _param_gamma_z,
        (ParamFloat<px4::params::L1MO_LPF_XY>)   _param_lpf_xy,
        (ParamFloat<px4::params::L1MO_LPF_Z>)    _param_lpf_z,
        (ParamFloat<px4::params::L1MO_KOBS_XY>)  _param_kobs_xy,
        (ParamFloat<px4::params::L1MO_KOBS_Z>)   _param_kobs_z,
        (ParamFloat<px4::params::L1MO_KP_XY>)    _param_kp_xy,
        (ParamFloat<px4::params::L1MO_KP_Z>)     _param_kp_z,
        (ParamFloat<px4::params::L1MO_KD_XY>)    _param_kd_xy,
        (ParamFloat<px4::params::L1MO_KD_Z>)     _param_kd_z,
        (ParamFloat<px4::params::L1MO_SIG_MOD>) _param_sig_mod,
        (ParamFloat<px4::params::L1MO_SIG_LIM>)  _param_sigma_lim,
        (ParamFloat<px4::params::L1MO_MO_KO>)    _param_mo_ko,
        (ParamFloat<px4::params::L1MO_M_NOM>)    _param_m_nom,
        (ParamFloat<px4::params::L1MO_M_MIN>)    _param_m_min,
        (ParamFloat<px4::params::L1MO_M_MAX>)    _param_m_max,
        (ParamFloat<px4::params::L1MO_F_MAX>)    _param_f_max,
        (ParamFloat<px4::params::L1MO_TILT_MAX>) _param_tilt_max,
        (ParamFloat<px4::params::L1MO_ETC_XY>)   _param_etc_xy,
        (ParamFloat<px4::params::L1MO_ETC_Z>)    _param_etc_z,
        (ParamFloat<px4::params::L1MO_MO_DTH>)   _param_mo_dth,
        (ParamFloat<px4::params::L1MO_THR_KQ>)   _param_thr_kq,
        (ParamFloat<px4::params::L1MO_THR_KL>)   _param_thr_kl,
        (ParamInt<px4::params::L1MO_MO_EN>)       _param_mo_en
    )
};
