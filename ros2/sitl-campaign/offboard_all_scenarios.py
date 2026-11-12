#!/usr/bin/env python3
"""
offboard_all_scenarios.py
────────────────────────────────────────────────────────────────────
ROS2 offboard node untuk semua skenario SITL.
Dikontrol oleh environment variable dari run_campaign.sh,
atau bisa dijalankan langsung dengan argumen CLI.

Usage (manual):
  python3 offboard_all_scenarios.py --scenario drop --controller l1 \
          --payload 0.6 --wind --run-id 01

Env vars (dari run_campaign.sh):
  SITL_SCENARIO       : hover | drop | multi_drop | dynamic_drop | motor_fault
  SITL_CONTROLLER     : l1 | pid
  SITL_PAYLOAD_KG     : 0.0 | 0.2 | 0.4 | 0.6
  SITL_NO_WIND        : true | false
  SITL_DYNAMIC_DROP   : true | false  (untuk dynamic_drop via offboard_controller)
  SITL_MOTOR_FACTOR   : 0.0–1.0
  SITL_RUN_ID         : 01–99
  SITL_TAG            : label string untuk log
  SITL_LOG_DIR        : path ke log dir

Metric definitions (SINGLE SOURCE OF TRUTH):
  EPS_MM    = 50      recovery threshold (mm)
  T_HOLD    = 1.0 s   durasi harus stabil setelah recovery
  T_WIN     = 8.0 s   analysis window post-drop
  t_drop    = timestamp DETACH COMMAND (bukan error spike)
"""

import os
import sys
import math
import time
import argparse
import subprocess
import threading
import csv
from datetime import datetime
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# ─── Metric constants (single source of truth) ────────────────────────────────
EPS_MM   = 50.0    # recovery threshold mm
T_HOLD   = 1.0     # s — waktu harus konsisten di bawah EPS setelah recovery
T_WIN    = 8.0     # s — analysis window post-drop
HOVER_SETTLE_TIME = 12.0  # s — tunggu hover stabil sebelum event
HOVER_Z  = -5.0    # m  — target altitude (NED, negatif = naik)
DELIVERY_Z = -5.0  # m  — altitude di waypoint delivery

# ─── Scenarios ────────────────────────────────────────────────────────────────
class State(Enum):
    INIT          = auto()
    ARMING        = auto()
    TAKEOFF       = auto()
    HOVER_SETTLE  = auto()
    TRANSLATE     = auto()   # khusus dynamic_drop: gerak dulu baru drop
    DROP_WAIT     = auto()   # tunggu sebentar setelah drop
    SECOND_DROP   = auto()   # khusus multi_drop
    SECOND_WAIT   = auto()
    POST_DROP     = auto()   # monitoring window T_WIN
    MOTOR_FAULT   = auto()   # trigger fault saat hover
    FAULT_MONITOR = auto()
    LANDING       = auto()
    DONE          = auto()

# ─── QoS PX4 ─────────────────────────────────────────────────────────────────
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ─── Main Node ────────────────────────────────────────────────────────────────
class ScenarioRunner(Node):

    def __init__(self, args):
        super().__init__('scenario_runner')

        self.scenario      = args.scenario
        self.controller    = args.controller
        self.payload_kg    = float(args.payload)
        self.no_wind       = args.no_wind
        self.dynamic_drop  = args.dynamic_drop
        self.motor_factor  = float(args.motor_factor)
        self.run_id        = args.run_id
        self.tag           = args.tag or f"{self.controller}_{self.scenario}_{self.payload_kg}kg_run{self.run_id}"
        self.log_dir       = args.log_dir or "/tmp/sitl_logs"

        os.makedirs(self.log_dir, exist_ok=True)

        # State
        self.state         = State.INIT
        self.nav_state     = 0
        self.arming_state  = 0
        self.pos_x = self.pos_y = self.pos_z = 0.0
        self.vel_x = self.vel_y = self.vel_z = 0.0
        self.offboard_count = 0
        self.state_entry_time = time.time()

        # Metric tracking
        self.t_drop        = None     # timestamp detach command
        self.peak_alt_err  = 0.0      # mm
        self.recovery_s    = None
        self._recovery_start = None
        self._below_eps_start = None
        self.post_drop_errs = []
        self.metrics_ready  = False

        # Target
        self.sp_x = 0.0
        self.sp_y = 0.0
        self.sp_z = HOVER_Z

        # CSV logger
        self._log_csv_path = os.path.join(self.log_dir, f"{self.tag}_offboard.csv")
        self._csv_file = open(self._log_csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            't_s', 'state', 'pos_x', 'pos_y', 'pos_z',
            'sp_x', 'sp_y', 'sp_z',
            'err_z_mm', 'nav_state', 'arming_state'
        ])
        self._t0 = time.time()

        # Publishers
        self._pub_offboard = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self._pub_traj = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self._pub_cmd = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # Subscribers
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._cb_pos, PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._cb_status, PX4_QOS)

        # Main control loop — 10 Hz
        self._timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f"ScenarioRunner ready: {self.tag} | scenario={self.scenario} "
            f"payload={self.payload_kg}kg motor_factor={self.motor_factor}"
        )

    # ─── Callbacks ────────────────────────────────────────────────────────────
    def _cb_pos(self, msg):
        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_z = msg.z
        self.vel_x = msg.vx
        self.vel_y = msg.vy
        self.vel_z = msg.vz

    def _cb_status(self, msg):
        self.nav_state    = msg.nav_state
        self.arming_state = msg.arming_state

    # ─── Publishers ───────────────────────────────────────────────────────────
    def _pub_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_offboard.publish(msg)

    def _pub_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_traj.publish(msg)
        self.sp_x, self.sp_y, self.sp_z = x, y, z

    def _send_cmd(self, cmd, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                  p5=0.0, p6=0.0, p7=0.0):
        msg = VehicleCommand()
        msg.command = cmd
        msg.param1 = float(p1); msg.param2 = float(p2)
        msg.param3 = float(p3); msg.param4 = float(p4)
        msg.param5 = float(p5); msg.param6 = float(p6)
        msg.param7 = float(p7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_cmd.publish(msg)

    def _arm(self):
        self._send_cmd(400, p1=1.0)  # VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM

    def _disarm(self):
        self._send_cmd(400, p1=0.0)

    def _set_offboard_mode(self):
        self._send_cmd(176, p1=1.0, p2=6.0)  # MAV_CMD_DO_SET_MODE OFFBOARD

    def _land(self):
        self._send_cmd(21)  # NAV_CMD_LAND

    # ─── Payload drop via gz topic ────────────────────────────────────────────
    def _trigger_drop(self, topic="/payload_drop"):
        """Trigger payload detach via Gazebo transport. t_drop = waktu command ini."""
        t_now = time.time() - self._t0
        self.get_logger().info(f"DETACH COMMAND at t={t_now:.3f}s → {topic}")
        self.t_drop = t_now  # SINGLE SOURCE: t_drop = detach command timestamp

        try:
            subprocess.run(
                ["gz", "topic", "-t", topic,
                 "-m", "gz.msgs.Empty", "-p", ""],
                timeout=3, check=False, capture_output=True
            )
        except Exception as e:
            self.get_logger().warn(f"gz topic drop failed: {e}")

    def _trigger_motor_fault(self, factor, motor_num=1):
        """Degrade motor via Gazebo MotorFailurePlugin."""
        self.get_logger().info(f"MOTOR FAULT: motor={motor_num}, factor={factor}")
        try:
            subprocess.run([
                "gz", "topic", "-t", "/motor_failure/throttle_factor",
                "-m", "gz.msgs.Float", "-p", f"data: {factor}"
            ], timeout=3, check=False, capture_output=True)
            time.sleep(0.3)
            subprocess.run([
                "gz", "topic", "-t", "/model/x500_0/motor_failure/motor_number",
                "-m", "gz.msgs.Int32", "-p", f"data: {motor_num}"
            ], timeout=3, check=False, capture_output=True)
        except Exception as e:
            self.get_logger().warn(f"Motor fault trigger failed: {e}")

    def _clear_motor_fault(self):
        try:
            subprocess.run([
                "gz", "topic", "-t", "/model/x500_0/motor_failure/motor_number",
                "-m", "gz.msgs.Int32", "-p", "data: 0"
            ], timeout=3, check=False, capture_output=True)
        except Exception:
            pass

    # ─── Helpers ──────────────────────────────────────────────────────────────
    def _at_target(self, x, y, z, tol=0.15):
        return (abs(self.pos_x - x) < tol and
                abs(self.pos_y - y) < tol and
                abs(self.pos_z - z) < tol)

    def _err_z_mm(self):
        return abs(self.pos_z - self.sp_z) * 1000.0

    def _since(self):
        return time.time() - self.state_entry_time

    def _enter_state(self, s):
        self.state = s
        self.state_entry_time = time.time()
        self.get_logger().info(f"→ State: {s.name}")

    def _update_metrics(self):
        """Update recovery tracking setelah drop."""
        if self.t_drop is None:
            return

        t_now = time.time() - self._t0
        elapsed = t_now - self.t_drop
        if elapsed < 0 or elapsed > T_WIN + 2:
            return

        err_mm = self._err_z_mm()
        self.peak_alt_err = max(self.peak_alt_err, err_mm)

        # Recovery: cari kapan err_mm < EPS_MM selama T_HOLD detik berturut-turut
        if self.recovery_s is None:
            if err_mm < EPS_MM:
                if self._below_eps_start is None:
                    self._below_eps_start = t_now
                elif (t_now - self._below_eps_start) >= T_HOLD:
                    self.recovery_s = self._below_eps_start - (self._t0 + self.t_drop)
                    self.get_logger().info(
                        f"RECOVERED at t_drop+{self.recovery_s:.3f}s "
                        f"(peak_err={self.peak_alt_err:.1f}mm)"
                    )
            else:
                self._below_eps_start = None  # reset jika naik lagi

        # Post-drop RMSE window
        if 0 <= elapsed <= T_WIN:
            self.post_drop_errs.append(err_mm)

    def _log_row(self):
        t_now = time.time() - self._t0
        self._csv_writer.writerow([
            f"{t_now:.3f}", self.state.name,
            f"{self.pos_x:.4f}", f"{self.pos_y:.4f}", f"{self.pos_z:.4f}",
            f"{self.sp_x:.4f}", f"{self.sp_y:.4f}", f"{self.sp_z:.4f}",
            f"{self._err_z_mm():.2f}",
            self.nav_state, self.arming_state
        ])

    def _finish(self):
        """Simpan metrics summary dan shutdown."""
        self._csv_file.flush()
        self._csv_file.close()

        post_rmse = 0.0
        if self.post_drop_errs:
            post_rmse = math.sqrt(
                sum(e**2 for e in self.post_drop_errs) / len(self.post_drop_errs)
            )

        # Tulis ringkasan ke file
        summary_path = os.path.join(
            self.log_dir, f"{self.tag}_metrics.csv"
        )
        with open(summary_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'tag', 'controller', 'payload_kg', 'scenario', 'wind',
                't_drop_s', 't_drop_source',
                'peak_alt_err_mm', 'recovery_s', 'post_rmse_mm',
                'motor_factor', 'run_id'
            ])
            wind_tag = "nowind" if self.no_wind else "wind"
            w.writerow([
                self.tag, self.controller, self.payload_kg, self.scenario, wind_tag,
                f"{self.t_drop:.3f}" if self.t_drop else "",
                "detach_cmd",  # t_drop SOURCE: always command, not error spike
                f"{self.peak_alt_err:.2f}",
                f"{self.recovery_s:.4f}" if self.recovery_s else "",
                f"{post_rmse:.4f}",
                self.motor_factor, self.run_id
            ])

        self.get_logger().info(
            f"Metrics saved → {summary_path}\n"
            f"  peak_alt_err={self.peak_alt_err:.1f}mm  "
            f"recovery={self.recovery_s:.3f}s  "
            f"post_rmse={post_rmse:.1f}mm"
            if self.recovery_s else
            f"  peak_alt_err={self.peak_alt_err:.1f}mm  recovery=NOT_RECOVERED"
        )
        self.metrics_ready = True

    # ─── Main control loop ────────────────────────────────────────────────────
    def _control_loop(self):
        # Selalu publish offboard mode (PX4 butuh terus-menerus)
        self._pub_offboard_mode()
        self._pub_setpoint(self.sp_x, self.sp_y, self.sp_z)
        self.offboard_count += 1

        # Update metrics jika post-drop
        self._update_metrics()

        # Log tiap detik
        if self.offboard_count % 10 == 0:
            self._log_row()

        # State machine
        self._fsm()

    def _fsm(self):
        s = self.state

        # ── INIT ──────────────────────────────────────────────────────────────
        if s == State.INIT:
            if self.offboard_count >= 20:   # 2 detik setpoints dulu
                self._set_offboard_mode()
                self._arm()
                self._enter_state(State.ARMING)

        # ── ARMING ────────────────────────────────────────────────────────────
        elif s == State.ARMING:
            if self.arming_state == 2:       # ARMING_STATE_ARMED
                self._enter_state(State.TAKEOFF)
            elif self._since() > 10:
                self.get_logger().warn("Arming timeout — retry")
                self._arm()
                self.state_entry_time = time.time()

        # ── TAKEOFF ───────────────────────────────────────────────────────────
        elif s == State.TAKEOFF:
            self.sp_z = HOVER_Z
            if self._at_target(0, 0, HOVER_Z, tol=0.3):
                self._enter_state(State.HOVER_SETTLE)

        # ── HOVER SETTLE ──────────────────────────────────────────────────────
        elif s == State.HOVER_SETTLE:
            # Tunggu hingga hover benar-benar stabil
            if self._since() >= HOVER_SETTLE_TIME:
                # Dispatch ke scenario berikutnya
                if self.scenario == "hover":
                    # Hover saja — tidak ada event, langsung ke landing
                    # Monitoring 30 detik lagi untuk baseline
                    if self._since() >= HOVER_SETTLE_TIME + 30:
                        self._enter_state(State.LANDING)

                elif self.scenario in ("drop", "motor_fault"):
                    if self.scenario == "drop":
                        self._trigger_drop("/payload_drop")
                        self._enter_state(State.DROP_WAIT)
                    else:
                        self._trigger_motor_fault(self.motor_factor, motor_num=1)
                        self._enter_state(State.MOTOR_FAULT)

                elif self.scenario == "multi_drop":
                    self._trigger_drop("/payload_drop_1")
                    self._enter_state(State.DROP_WAIT)

                elif self.scenario == "dynamic_drop":
                    # Dulu translasi ke waypoint, baru drop
                    self.sp_x = 3.0
                    self.sp_y = 2.0
                    self._enter_state(State.TRANSLATE)

        # ── TRANSLATE (dynamic_drop: gerak dulu) ─────────────────────────────
        elif s == State.TRANSLATE:
            if self._at_target(3.0, 2.0, HOVER_Z, tol=0.5) or self._since() > 20:
                # Drop saat translasi (atau segera setelah waypoint tercapai)
                self._trigger_drop("/payload_drop")
                self._enter_state(State.DROP_WAIT)

        # ── DROP WAIT ─────────────────────────────────────────────────────────
        elif s == State.DROP_WAIT:
            if self._since() >= T_WIN:
                if self.scenario == "multi_drop":
                    # Drop kedua
                    self._trigger_drop("/payload_drop_2")
                    self._enter_state(State.SECOND_DROP)
                else:
                    self._enter_state(State.POST_DROP)

        # ── SECOND DROP (multi_drop) ──────────────────────────────────────────
        elif s == State.SECOND_DROP:
            # Wait sebentar lagi setelah drop kedua
            if self._since() >= T_WIN:
                self._enter_state(State.POST_DROP)

        # ── POST DROP ─────────────────────────────────────────────────────────
        elif s == State.POST_DROP:
            # Monitoring selesai — landing
            self._enter_state(State.LANDING)

        # ── MOTOR FAULT ───────────────────────────────────────────────────────
        elif s == State.MOTOR_FAULT:
            # Monitor respons FTC
            if self._since() >= 30:
                self._clear_motor_fault()
                self._enter_state(State.FAULT_MONITOR)

        # ── FAULT MONITOR ─────────────────────────────────────────────────────
        elif s == State.FAULT_MONITOR:
            if self._since() >= 15:
                self._enter_state(State.LANDING)

        # ── LANDING ───────────────────────────────────────────────────────────
        elif s == State.LANDING:
            self._land()
            if self._since() >= 8 and abs(self.pos_z) < 0.3:
                self._disarm()
                self._finish()
                self._enter_state(State.DONE)
            elif self._since() > 30:
                # Force done — mungkin sudah landed
                self._disarm()
                self._finish()
                self._enter_state(State.DONE)

        # ── DONE ──────────────────────────────────────────────────────────────
        elif s == State.DONE:
            self._timer.cancel()
            rclpy.shutdown()


# ─── Parameter sweep runner ──────────────────────────────────────────────────
class ParamSweepRunner:
    """
    Jalankan sweep parameter L1 controller (gamma, kp, kd, sig_lim).
    Tiap nilai dites N runs menggunakan offboard_all_scenarios.py di atas.
    Dipanggil dari run_campaign.sh via flag --sweep.
    """

    # Parameter sweep matrix untuk thesis
    # Edit sesuai kebutuhan — ini default sweep yang reasonable
    SWEEP_PARAMS = {
        "GAMMA_X": [200, 400, 600, 800],   # L1 adaptive gain
        "GAMMA_Y": [200, 400, 600, 800],   # (biasanya sama X)
        "GAMMA_Z": [0.5, 1.0, 2.0, 5.0],  # vertical adaptive gain
        "SIG_LIM": [3.0, 6.0, 9.0],       # sigma saturation limit
        "KP_Z":    [4.0, 6.0, 8.0],       # vertical P gain
        "KD_Z":    [2.0, 4.0, 6.0],       # vertical D gain
    }

    def __init__(self, param_name, values, n_runs=5):
        self.param_name = param_name
        self.values     = values
        self.n_runs     = n_runs

    def set_l1_param(self, param_name, value):
        """Set parameter L1 via ROS2 parameter server atau PX4 param."""
        # Cara: publish ke /fmu/in/... atau via parameter file
        # Untuk sekarang: print instruction dan return
        print(f"  [PARAM] Set {param_name} = {value}")
        print(f"  Manual: di pxh> param set {param_name} {value}")

    def run(self):
        for val in self.values:
            print(f"\n{'='*60}")
            print(f"  SWEEP: {self.param_name} = {val}")
            print(f"{'='*60}")
            self.set_l1_param(self.param_name, val)
            # ... bisa extended untuk call ScenarioRunner N kali


# ─── Argparse + main ─────────────────────────────────────────────────────────
def parse_args():
    # Ambil dari env vars dulu, lalu CLI override
    env = {
        'scenario':      os.environ.get('SITL_SCENARIO',      'drop'),
        'controller':    os.environ.get('SITL_CONTROLLER',    'l1'),
        'payload':       os.environ.get('SITL_PAYLOAD_KG',    '0.6'),
        'no_wind':       os.environ.get('SITL_NO_WIND',       'false').lower() == 'true',
        'dynamic_drop':  os.environ.get('SITL_DYNAMIC_DROP',  'false').lower() == 'true',
        'motor_factor':  os.environ.get('SITL_MOTOR_FACTOR',  '1.0'),
        'run_id':        os.environ.get('SITL_RUN_ID',        '01'),
        'tag':           os.environ.get('SITL_TAG',           ''),
        'log_dir':       os.environ.get('SITL_LOG_DIR',       '/tmp/sitl_logs'),
    }

    p = argparse.ArgumentParser(description='SITL Scenario Runner')
    p.add_argument('--scenario',     default=env['scenario'],
                   choices=['hover','drop','multi_drop','dynamic_drop','motor_fault'])
    p.add_argument('--controller',   default=env['controller'],  choices=['l1','pid'])
    p.add_argument('--payload',      default=env['payload'],     type=float)
    p.add_argument('--no-wind',      default=env['no_wind'],     action='store_true')
    p.add_argument('--dynamic-drop', default=env['dynamic_drop'],action='store_true')
    p.add_argument('--motor-factor', default=env['motor_factor'],type=float)
    p.add_argument('--run-id',       default=env['run_id'])
    p.add_argument('--tag',          default=env['tag'])
    p.add_argument('--log-dir',      default=env['log_dir'])
    return p.parse_args()


def main():
    args = parse_args()

    rclpy.init(args=None)
    node = ScenarioRunner(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted — cleanup")
        node._clear_motor_fault()
        if not node.metrics_ready:
            node._finish()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
