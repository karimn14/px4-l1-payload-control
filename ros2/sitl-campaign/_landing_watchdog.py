#!/usr/bin/env python3
"""
_landing_watchdog.py — Deteksi drone sudah landed/disarmed, lalu exit.

Dipakai run_campaign.sh untuk tahu kapan offboard scenario SELESAI,
supaya bisa langsung kill node tanpa nunggu timeout penuh.

Exit code:
  0 = drone landed/disarmed terdeteksi (scenario selesai)
  2 = timeout (tidak terdeteksi dalam batas waktu)
  3 = error (ROS2/px4_msgs tidak tersedia)

Usage:
  python3 _landing_watchdog.py --timeout 90
  python3 _landing_watchdog.py --timeout 90 --min-airborne 8

Test manual (untuk debug):
  source /opt/ros/humble/setup.bash
  source ~/drone_ws/install/setup.bash
  python3 _landing_watchdog.py --timeout 30 --verbose
  # Lihat apakah arming_state & altitude terbaca
"""

import sys
import time
import argparse

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                            DurabilityPolicy, HistoryPolicy)
    from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
except Exception as e:
    print(f"[watchdog] ERROR import: {e}", file=sys.stderr)
    print("[watchdog] Pastikan sudah: source /opt/ros/humble/setup.bash "
          "&& source ~/drone_ws/install/setup.bash", file=sys.stderr)
    sys.exit(3)

# PX4 pakai BEST_EFFORT + TRANSIENT_LOCAL untuk topic /fmu/out/*
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Beberapa versi PX4/px4_msgs pakai nama topic berbeda untuk vehicle_status.
# Kita subscribe ke beberapa kandidat sekaligus.
STATUS_TOPICS = [
    '/fmu/out/vehicle_status',
    '/fmu/out/vehicle_status_v1',
]
POS_TOPICS = [
    '/fmu/out/vehicle_local_position',
    '/fmu/out/vehicle_local_position_v1',
]

ARMING_DISARMED = 1   # VehicleStatus.ARMING_STATE_DISARMED


class LandingWatchdog(Node):
    def __init__(self, args):
        super().__init__('landing_watchdog')
        self.args = args

        self.arming_state = None
        self.z = None            # NED altitude (negatif = naik)
        self.was_airborne = False
        self.airborne_since = None

        self.disarmed_since = None
        self.landed_since = None

        # Subscribe ke semua kandidat topic
        self._subs = []
        for t in STATUS_TOPICS:
            self._subs.append(self.create_subscription(
                VehicleStatus, t, self._cb_status, PX4_QOS))
        for t in POS_TOPICS:
            self._subs.append(self.create_subscription(
                VehicleLocalPosition, t, self._cb_pos, PX4_QOS))

        self._got_any_data = False

    def _cb_status(self, msg):
        self.arming_state = msg.arming_state
        self._got_any_data = True

    def _cb_pos(self, msg):
        self.z = msg.z
        self._got_any_data = True

    def check_done(self, now):
        """Return True kalau drone landed/disarmed cukup lama."""
        # ── Kondisi 1: DISARMED ───────────────────────────────────────────────
        if self.arming_state == ARMING_DISARMED:
            if self.disarmed_since is None:
                self.disarmed_since = now
            if (now - self.disarmed_since) >= self.args.disarm_hold:
                if self.args.verbose:
                    print(f"[watchdog] DISARMED selama "
                          f"{now - self.disarmed_since:.1f}s → selesai",
                          file=sys.stderr)
                return True
        else:
            self.disarmed_since = None

        # ── Kondisi 2: di tanah (altitude ~ 0) setelah pernah airborne ────────
        if self.z is not None:
            alt = abs(self.z)  # |z| dekat 0 = di tanah
            # Track apakah pernah airborne (altitude > 1m)
            if alt > 1.0:
                self.was_airborne = True

            if self.was_airborne and alt < self.args.land_alt:
                if self.landed_since is None:
                    self.landed_since = now
                if (now - self.landed_since) >= self.args.land_hold:
                    if self.args.verbose:
                        print(f"[watchdog] LANDED (alt={alt:.2f}m) selama "
                              f"{now - self.landed_since:.1f}s → selesai",
                              file=sys.stderr)
                    return True
            else:
                self.landed_since = None

        return False


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--timeout',      type=float, default=120.0,
                   help='Batas waktu maksimum (detik)')
    p.add_argument('--disarm-hold',  type=float, default=3.0,
                   help='Durasi DISARMED sebelum dianggap selesai (detik)')
    p.add_argument('--land-alt',     type=float, default=0.3,
                   help='Altitude (m) di bawah ini = di tanah')
    p.add_argument('--land-hold',    type=float, default=4.0,
                   help='Durasi di tanah sebelum dianggap selesai (detik)')
    p.add_argument('--grace',        type=float, default=15.0,
                   help='Jangan deteksi selesai sebelum grace period (detik) '
                        '— cegah false trigger saat masih di ground sebelum takeoff')
    p.add_argument('--verbose',      action='store_true')
    args = p.parse_args()

    rclpy.init()
    node = LandingWatchdog(args)

    t0 = time.time()
    last_print = 0.0
    no_data_warned = False

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time() - t0

            # Timeout
            if now >= args.timeout:
                print(f"[watchdog] TIMEOUT {args.timeout}s", file=sys.stderr)
                node.destroy_node(); rclpy.shutdown()
                sys.exit(2)

            # Warn kalau tidak ada data sama sekali setelah 10 detik
            if not node._got_any_data and now > 10 and not no_data_warned:
                print("[watchdog] WARNING: belum ada data dari /fmu/out/* "
                      "setelah 10s. Cek DDS agent & topic names.",
                      file=sys.stderr)
                no_data_warned = True

            # Progress (verbose)
            if args.verbose and (now - last_print) >= 2.0:
                alt = abs(node.z) if node.z is not None else None
                print(f"[watchdog] t={now:.0f}s arming={node.arming_state} "
                      f"alt={alt if alt is None else round(alt,2)} "
                      f"airborne={node.was_airborne}", file=sys.stderr)
                last_print = now

            # Cek selesai (setelah grace period)
            if now >= args.grace and node.check_done(now):
                print(f"[watchdog] SELESAI terdeteksi di t={now:.1f}s",
                      file=sys.stderr)
                node.destroy_node(); rclpy.shutdown()
                sys.exit(0)

    except KeyboardInterrupt:
        node.destroy_node(); rclpy.shutdown()
        sys.exit(2)


if __name__ == '__main__':
    main()
