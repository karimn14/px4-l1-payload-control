#!/usr/bin/env python3
"""
pick_drop_runner.py — Pick & Drop scenario untuk campaign SITL
══════════════════════════════════════════════════════════════════
Dibuat dari pick_drop.py (pick_and_drop_v5_corrected).

PERBEDAAN PENTING dari multi_drop:
  Model    : payload_box   (bukan payload_med)
  Attach   : /payload_attach  (DetachableJoint re-attach)
  Detach   : /payload_drop
  State    : /payload_state
  World    : pickup (PX4_GZ_WORLD=pickup)

Massa payload_box diset di model.sdf — satu file per variant:
  payload_box_0.3kg.sdf  payload_box_0.6kg.sdf  dst.
  (kalau file tidak ada, pakai model.sdf default tanpa modifikasi massa)

Usage:
  python3 pick_drop_runner.py --payload 0.6 --run-id 01 \\
          --log-dir /path/to/logs --tag l1_pick_drop_0.6kg_wind_run01
"""

import argparse
import math
import os
import select
import subprocess
import sys
import time
import csv

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                        DurabilityPolicy, HistoryPolicy)
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleLocalPosition, VehicleStatus,
)

# ─── Constants (dari pick_drop.py) ────────────────────────────────────────────
PICKUP_X   =  2.0
PICKUP_Y   =  0.0
Z_TRANSIT  = -5.0
Z_PICKUP   = -1.2

Z_DELIVERY = -5.0
DELIVERY_X =  5.0
DELIVERY_Y =  5.0

MAX_DESCENT_RATE = 0.4   # m/s
MAX_CLIMB_RATE   = 0.6   # m/s

ATTACH_RADIUS_XY = 0.5
ATTACH_RADIUS_Z  = 0.53

ATTACH_INTERVAL_S = 0.5
DETACH_INTERVAL_S = 0.5
GZ_CMD_TIMEOUT_S  = 3.0

# Topics KHUSUS pick_drop (beda dengan multi_drop!)
TOPIC_ATTACH = "/payload_attach"
TOPIC_DROP   = "/payload_drop"
TOPIC_STATE  = "/payload_state"

# Phase timeline (detik)
PHASES = [
    ("INIT",               4.0),
    ("INITIAL_DETACH",     3.0),
    ("TAKEOFF",           18.0),
    ("STABILIZE_HOVER",    6.0),
    ("GOTO_PICKUP",       16.0),
    ("STABILIZE_ABOVE",   10.0),
    ("DESCEND_TO_PICKUP", 25.0),
    ("STABILIZE_PICKUP",  12.0),
    ("ATTACH",            10.0),
    ("SETTLE_ATTACH",     12.0),
    ("CLIMB_TO_TRANSIT",  20.0),
    ("GOTO_DELIVERY",     16.0),
    ("STABILIZE_DELIVERY", 6.0),
    ("DROP",               6.0),
    ("SETTLE_DROP",       12.0),
    ("HOLD",              20.0),
]
TOTAL_DURATION = sum(d for _, d in PHASES)

# ─── Metric constants ──────────────────────────────────────────────────────────
EPS_MM  = 50.0
T_HOLD  = 1.0
T_WIN   = 8.0

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ─── Gazebo helpers ────────────────────────────────────────────────────────────
def gz_pub(topic: str) -> bool:
    try:
        r = subprocess.run(
            ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""],
            capture_output=True, text=True, timeout=GZ_CMD_TIMEOUT_S)
        return r.returncode == 0
    except Exception:
        return False

def gz_has_subscriber(topic: str) -> bool:
    try:
        r = subprocess.run(
            ["gz", "topic", "-i", "-t", topic],
            capture_output=True, text=True, timeout=3.0)
        return r.returncode == 0 and "Subscriber" in r.stdout
    except Exception:
        return False

def wait_gz_ready(topics: list, timeout: float = 20.0) -> bool:
    print(f"[pick_drop] Menunggu plugin siap (max {timeout:.0f}s)...")
    deadline = time.time() + timeout
    pending  = list(topics)
    while time.time() < deadline and pending:
        still = []
        for t in pending:
            if gz_has_subscriber(t):
                print(f"  [OK] {t}")
            else:
                still.append(t)
        pending = still
        if pending:
            time.sleep(0.5)
    if pending:
        print(f"  [WARN] Timeout — tidak ada subscriber: {pending}")
        print("         Pastikan world=pickup dan SDF memakai DetachableJoint")
        return False
    print("  Plugin siap.\n")
    return True


class PickDropRunner(Node):
    def __init__(self, args):
        super().__init__('pick_drop_runner')
        self.args     = args
        self.log_dir  = args.log_dir
        self.tag      = args.tag
        self.payload_kg = float(args.payload)

        os.makedirs(self.log_dir, exist_ok=True)

        # Publisher / subscriber
        self._pub_ocm  = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self._pub_traj = self.create_publisher(
            TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',  PX4_QOS)
        self._pub_cmd  = self.create_publisher(
            VehicleCommand,      '/fmu/in/vehicle_command',       PX4_QOS)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._cb_pos, PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._cb_status, PX4_QOS)

        # Position state
        self.x = self.y = self.z = 0.0
        self.pos_valid = False
        self.arming_state = 0
        self.sp_z = 0.0

        # Phase
        self.phase_idx  = 0
        self.phase_name = PHASES[0][0]
        self.phase_t0   = self.get_clock().now()
        self.offboard_count = 0

        # Attach/drop tracking
        self.payload_attached     = True
        self.attach_confirmed     = False
        self.drop_confirmed       = False
        self._attach_last_slot    = -1
        self._drop_last_slot      = -1
        self._initial_detach_sent = False
        self._state_poll_proc     = None

        # Metrics
        self._t0 = time.time()
        self.t_attach     = None   # timestamp detach command (t_drop untuk metrics)
        self.t_drop_cmd   = None
        self.peak_alt_err = 0.0
        self._below_eps_start = None
        self.recovery_s   = None
        self.post_errs    = []

        # CSV logger
        self._csv_path = os.path.join(self.log_dir, f"{self.tag}_pickdrop.csv")
        self._csv_f = open(self._csv_path, 'w', newline='')
        self._csv_w = csv.writer(self._csv_f)
        self._csv_w.writerow([
            't_s', 'phase', 'pos_x', 'pos_y', 'pos_z',
            'sp_x', 'sp_y', 'sp_z', 'err_z_mm',
            'arming_state', 'payload_attached',
        ])
        self._sp_x = self._sp_y = 0.0

        # Tunggu plugin siap SEBELUM mulai
        wait_gz_ready([TOPIC_ATTACH, TOPIC_DROP], timeout=25.0)
        self._start_state_poll()

        self._timer = self.create_timer(0.1, self._loop)
        self.get_logger().info(
            f"PickDropRunner siap: payload={self.payload_kg}kg "
            f"tag={self.tag}\n"
            f"  Topics: attach={TOPIC_ATTACH} drop={TOPIC_DROP} state={TOPIC_STATE}"
        )
        self._print_timeline()

    def _cb_pos(self, msg):
        self.x, self.y, self.z = msg.x, msg.y, msg.z
        self.pos_valid = True

    def _cb_status(self, msg):
        self.arming_state = msg.arming_state

    def _start_state_poll(self):
        try:
            self._state_poll_proc = subprocess.Popen(
                ["gz", "topic", "-e", "-t", TOPIC_STATE],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        except FileNotFoundError:
            self.get_logger().warn("gz tidak ditemukan — state polling off")

    def _poll_state(self):
        if not self._state_poll_proc:
            return
        if self._state_poll_proc.poll() is not None:
            return
        ready, _, _ = select.select([self._state_poll_proc.stdout], [], [], 0.0)
        if not ready:
            return
        line = self._state_poll_proc.stdout.readline().lower().strip()
        if not line:
            return
        if "detach" in line or "false" in line or line == "0":
            if self.payload_attached:
                self.payload_attached = False
                self.drop_confirmed   = True
                self.get_logger().info(f"✓ DETACHED ({TOPIC_STATE})")
        elif "attach" in line or "true" in line or line == "1":
            if not self.payload_attached:
                self.payload_attached  = True
                self.attach_confirmed  = True
                self.get_logger().info(f"✓ ATTACHED ({TOPIC_STATE})")

    # ─── PX4 helpers ──────────────────────────────────────────────────────────
    def _ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _pub_offboard(self):
        m = OffboardControlMode()
        m.position = True
        m.timestamp = self._ts()
        self._pub_ocm.publish(m)

    def _pub_sp(self, x, y, z, yaw=0.0):
        sp = TrajectorySetpoint()
        sp.position  = [float(x), float(y), float(z)]
        sp.yaw       = float(yaw)
        sp.timestamp = self._ts()
        self._pub_traj.publish(sp)
        self._sp_x, self._sp_y, self.sp_z = x, y, z

    def _step_z(self, z_target, descend: bool) -> float:
        rate = MAX_DESCENT_RATE if descend else MAX_CLIMB_RATE
        dt   = 0.1
        if descend:
            return min(self.sp_z + rate * dt, z_target)
        else:
            return max(self.sp_z - rate * dt, z_target)

    def _pub_sp_rated(self, x, y, z_target, descend: bool):
        self.sp_z = self._step_z(z_target, descend)
        self._pub_sp(x, y, self.sp_z)

    def _send_cmd(self, cmd, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.command = cmd; c.param1 = float(p1); c.param2 = float(p2)
        c.target_system = c.target_component = 1
        c.source_system = c.source_component = 1
        c.from_external = True; c.timestamp = self._ts()
        self._pub_cmd.publish(c)

    def _arm_offboard(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def _land(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _disarm(self):
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def _near_pickup(self) -> bool:
        if not self.pos_valid:
            return False
        dxy = math.sqrt((self.x - PICKUP_X)**2 + (self.y - PICKUP_Y)**2)
        dz  = abs(self.z - Z_PICKUP)
        return dxy < ATTACH_RADIUS_XY and dz < ATTACH_RADIUS_Z

    def _at_z(self, z_target, tol=0.15) -> bool:
        return abs(self.sp_z - z_target) < tol

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.phase_t0).nanoseconds * 1e-9

    def _phase_done(self) -> bool:
        return self._elapsed() >= PHASES[self.phase_idx][1]

    def _next_phase(self):
        self.phase_idx += 1
        if self.phase_idx >= len(PHASES):
            self.get_logger().info("=== MISI SELESAI ===")
            return
        self.phase_name = PHASES[self.phase_idx][0]
        self.phase_t0   = self.get_clock().now()
        self.get_logger().info(
            f"[{self.phase_idx:02d}/{len(PHASES)-1}] "
            f"{self.phase_name} ({PHASES[self.phase_idx][1]:.0f}s)"
        )

    def _try_interval(self, topic, slot_attr, interval) -> bool:
        slot = int(self._elapsed() / interval)
        if slot > getattr(self, slot_attr):
            setattr(self, slot_attr, slot)
            ok = gz_pub(topic)
            self.get_logger().info(f"  PUB {topic} → {'OK' if ok else 'FAIL'}")
            return ok
        return False

    # ─── Metric update ────────────────────────────────────────────────────────
    def _update_metrics(self):
        if self.t_drop_cmd is None:
            return
        t_now = time.time() - self._t0
        elapsed = t_now - self.t_drop_cmd
        if elapsed < 0 or elapsed > T_WIN + 2:
            return
        err_mm = abs(self.z - self.sp_z) * 1000.0
        self.peak_alt_err = max(self.peak_alt_err, err_mm)
        if self.recovery_s is None:
            if err_mm < EPS_MM:
                if self._below_eps_start is None:
                    self._below_eps_start = t_now
                elif (t_now - self._below_eps_start) >= T_HOLD:
                    self.recovery_s = self._below_eps_start - (self._t0 + self.t_drop_cmd)
            else:
                self._below_eps_start = None
        if 0 <= elapsed <= T_WIN:
            self.post_errs.append(err_mm)

    # ─── Finish ───────────────────────────────────────────────────────────────
    def _finish(self):
        self._csv_f.flush()
        self._csv_f.close()
        import math as _math
        post_rmse = 0.0
        if self.post_errs:
            post_rmse = _math.sqrt(sum(e**2 for e in self.post_errs)/len(self.post_errs))

        summary = os.path.join(self.log_dir, f"{self.tag}_metrics.csv")
        with open(summary, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'tag','controller','payload_kg','scenario','wind',
                't_drop_s','t_drop_source',
                'peak_alt_err_mm','recovery_s','post_rmse_mm',
                'attach_confirmed','drop_confirmed','run_id',
            ])
            wind = "nowind" if os.environ.get('SITL_NO_WIND','false')=='true' else "wind"
            w.writerow([
                self.tag,
                os.environ.get('SITL_CONTROLLER','l1'),
                self.payload_kg, 'pick_drop', wind,
                f"{self.t_drop_cmd:.3f}" if self.t_drop_cmd else "",
                "drop_cmd",
                f"{self.peak_alt_err:.2f}",
                f"{self.recovery_s:.4f}" if self.recovery_s else "",
                f"{post_rmse:.4f}",
                self.attach_confirmed, self.drop_confirmed,
                self.args.run_id,
            ])
        self.get_logger().info(
            f"Metrics → {summary}\n"
            f"  peak={self.peak_alt_err:.1f}mm  "
            f"recovery={'%.3fs'%self.recovery_s if self.recovery_s else 'N/A'}  "
            f"post_rmse={post_rmse:.1f}mm  "
            f"attach={self.attach_confirmed}  drop={self.drop_confirmed}"
        )

    def _cleanup(self):
        if self._state_poll_proc and self._state_poll_proc.poll() is None:
            self._state_poll_proc.terminate()

    def _print_timeline(self):
        print("=" * 60)
        print(f"pick_drop_runner | payload={self.payload_kg}kg")
        print(f"  attach={TOPIC_ATTACH}  drop={TOPIC_DROP}")
        print("=" * 60)
        t = 0.0
        for name, dur in PHASES:
            print(f"  t={t:6.1f}s  {name:<25} ({dur:.0f}s)")
            t += dur
        print(f"  TOTAL: {t:.0f}s (~{t/60:.1f} menit)\n")

    # ─── Main loop ────────────────────────────────────────────────────────────
    def _loop(self):
        self._pub_offboard()
        self._poll_state()
        self._update_metrics()
        self.offboard_count += 1

        if self.offboard_count % 10 == 0:
            t_now = time.time() - self._t0
            err_mm = abs(self.z - self.sp_z) * 1000.0
            self._csv_w.writerow([
                f"{t_now:.3f}", self.phase_name,
                f"{self.x:.4f}", f"{self.y:.4f}", f"{self.z:.4f}",
                f"{self._sp_x:.4f}", f"{self._sp_y:.4f}", f"{self.sp_z:.4f}",
                f"{err_mm:.2f}", self.arming_state, int(self.payload_attached),
            ])

        p = self.phase_name

        if p == "INIT":
            self._pub_sp(0.0, 0.0, 0.0)
            if self.offboard_count >= 30:
                self._arm_offboard()
            if self._phase_done() and self.offboard_count >= 30:
                self._next_phase()

        elif p == "INITIAL_DETACH":
            self._pub_sp(0.0, 0.0, 0.0)
            self._try_interval(TOPIC_DROP, "_drop_last_slot", DETACH_INTERVAL_S)
            if self._phase_done():
                self.drop_confirmed   = False
                self._drop_last_slot  = -1
                self._next_phase()

        elif p == "TAKEOFF":
            self._pub_sp_rated(0.0, 0.0, Z_TRANSIT, descend=False)
            if self._phase_done():
                self._next_phase()

        elif p == "STABILIZE_HOVER":
            self._pub_sp(0.0, 0.0, Z_TRANSIT)
            if self._phase_done():
                self._next_phase()

        elif p == "GOTO_PICKUP":
            self._pub_sp(PICKUP_X, PICKUP_Y, Z_TRANSIT)
            if self._phase_done():
                self._next_phase()

        elif p == "STABILIZE_ABOVE":
            self._pub_sp(PICKUP_X, PICKUP_Y, Z_TRANSIT)
            if self._phase_done():
                self._next_phase()

        elif p == "DESCEND_TO_PICKUP":
            self._pub_sp_rated(PICKUP_X, PICKUP_Y, Z_PICKUP, descend=True)
            if self._at_z(Z_PICKUP):
                self._next_phase()
            elif self._phase_done():
                self.get_logger().warn(f"DESCEND timeout (sp_z={self.sp_z:.2f})")
                self._next_phase()

        elif p == "STABILIZE_PICKUP":
            self._pub_sp(PICKUP_X, PICKUP_Y, Z_PICKUP)
            if self._phase_done():
                self.attach_confirmed    = False
                self._attach_last_slot   = -1
                self._next_phase()

        elif p == "ATTACH":
            self._pub_sp(PICKUP_X, PICKUP_Y, Z_PICKUP)
            if not self.attach_confirmed:
                if self._near_pickup():
                    self._try_interval(TOPIC_ATTACH, "_attach_last_slot",
                                       ATTACH_INTERVAL_S)
                else:
                    slot2 = int(self._elapsed() / 2.0)
                    if slot2 > getattr(self, '_attach_warn_slot', -1):
                        self._attach_warn_slot = slot2
                        if self.pos_valid:
                            dxy = math.sqrt((self.x-PICKUP_X)**2+(self.y-PICKUP_Y)**2)
                            self.get_logger().warn(
                                f"Belum dekat: dXY={dxy:.2f}m "
                                f"dZ={abs(self.z-Z_PICKUP):.2f}m")
                        else:
                            self._try_interval(TOPIC_ATTACH, "_attach_last_slot",
                                               ATTACH_INTERVAL_S)
            if self.attach_confirmed:
                self.t_attach = time.time() - self._t0
                self.get_logger().info(f"ATTACH confirmed t={self.t_attach:.1f}s")
                self._next_phase()
            elif self._phase_done():
                self.get_logger().warn(
                    "ATTACH timeout — cek DetachableJoint SDF dan gz topic")
                self._next_phase()

        elif p == "SETTLE_ATTACH":
            self._pub_sp(PICKUP_X, PICKUP_Y, Z_PICKUP)
            if self._phase_done():
                self._next_phase()

        elif p == "CLIMB_TO_TRANSIT":
            self._pub_sp_rated(PICKUP_X, PICKUP_Y, Z_TRANSIT, descend=False)
            if self._at_z(Z_TRANSIT):
                self._next_phase()
            elif self._phase_done():
                self._next_phase()

        elif p == "GOTO_DELIVERY":
            self._pub_sp(DELIVERY_X, DELIVERY_Y, Z_DELIVERY)
            if self._phase_done():
                self._next_phase()

        elif p == "STABILIZE_DELIVERY":
            self._pub_sp(DELIVERY_X, DELIVERY_Y, Z_DELIVERY)
            if self._phase_done():
                self.drop_confirmed  = False
                self._drop_last_slot = -1
                self._next_phase()

        elif p == "DROP":
            self._pub_sp(DELIVERY_X, DELIVERY_Y, Z_DELIVERY)
            if not self.drop_confirmed:
                self._try_interval(TOPIC_DROP, "_drop_last_slot", DETACH_INTERVAL_S)
            if self.drop_confirmed:
                # t_drop untuk metrics = waktu drop command
                self.t_drop_cmd = time.time() - self._t0
                self.get_logger().info(f"DROP confirmed t={self.t_drop_cmd:.1f}s")
                self._next_phase()
            elif self._phase_done():
                self.get_logger().warn("DROP timeout")
                self._next_phase()

        elif p == "SETTLE_DROP":
            self._pub_sp(DELIVERY_X, DELIVERY_Y, Z_DELIVERY)
            if self._phase_done():
                self._next_phase()

        elif p == "HOLD":
            self._pub_sp(DELIVERY_X, DELIVERY_Y, Z_DELIVERY)
            if self._phase_done():
                self.get_logger().info("HOLD selesai — landing")
                self._land()
                self._finish()
                self._cleanup()
                self._timer.cancel()
                rclpy.shutdown()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--payload',   default=os.environ.get('SITL_PAYLOAD_KG', '0.6'))
    p.add_argument('--run-id',    default=os.environ.get('SITL_RUN_ID',     '01'))
    p.add_argument('--log-dir',   default=os.environ.get('SITL_LOG_DIR',    '/tmp/sitl_logs'))
    p.add_argument('--tag',       default=os.environ.get('SITL_TAG',        'pick_drop'))
    args = p.parse_args()

    rclpy.init()
    node = PickDropRunner(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[pick_drop] Dihentikan.")
        node._finish()
        node._cleanup()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
