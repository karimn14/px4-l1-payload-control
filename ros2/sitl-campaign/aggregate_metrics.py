#!/usr/bin/env python3
"""
aggregate_metrics.py — generic per-run + mean/std aggregator untuk campaign SITL.

Sengaja TIDAK butuh payload / t_drop (beda dengan 1_analyze_flight_logs.py yang
butuh info itu untuk RMSE pre/post-drop, recovery_s, dsb). Script ini hanya
menghitung statistik whole-flight yang bisa dihitung langsung dari CSV
l1_monitor.py apapun skenarionya, supaya bisa langsung dipakai tanpa isi
apa-apa dulu. Untuk metrik drop-specific yang presisi, pakai
1_analyze_flight_logs.py (command-nya sudah dicetak otomatis oleh run_campaign.sh).

Usage:
    python3 aggregate_metrics.py --dir <campaign>/raw --out <campaign>/analysis
"""
import argparse
import glob
import os
import sys
import numpy as np
import pandas as pd


def col(df, name):
    if name in df.columns:
        return pd.to_numeric(df[name], errors="coerce")
    return pd.Series(np.nan, index=df.index)


def per_run_metrics(path):
    df = pd.read_csv(path)
    t = col(df, "t_s")
    duration_s = float(t.max() - t.min()) if len(t) else float("nan")

    err3d = col(df, "err_3d_m")
    errz = col(df, "err_z_m")
    errx = col(df, "err_x_m")
    erry = col(df, "err_y_m")

    rmse_3d_mm = float(np.sqrt(np.nanmean(err3d ** 2)) * 1e3) if len(err3d) else float("nan")
    rmse_z_mm = float(np.sqrt(np.nanmean(errz ** 2)) * 1e3) if len(errz) else float("nan")
    rmse_xy_mm = float(np.sqrt(np.nanmean(errx ** 2 + erry ** 2)) * 1e3) if len(errx) else float("nan")
    max_err_3d_mm = float(np.nanmax(err3d) * 1e3) if len(err3d) else float("nan")

    roll_std_deg = float(col(df, "roll_act_deg").std())
    pitch_std_deg = float(col(df, "pitch_act_deg").std())

    mo_m = col(df, "mo_m_est_kg")
    n_tail = max(1, int(len(mo_m) * 0.2))
    mo_m_est_tail_mean_kg = float(mo_m.tail(n_tail).mean()) if len(mo_m) else float("nan")
    mo_m_est_final_kg = float(mo_m.iloc[-1]) if len(mo_m) else float("nan")

    sigma_mag_mean = float(col(df, "sigma_mag_ms2").mean())
    thrust_norm_mean = float(col(df, "thrust_norm").mean())

    etc_count = col(df, "etc_event_count")
    etc_event_count_total = float(etc_count.iloc[-1]) if len(etc_count) else float("nan")

    n_rows = len(df)

    return {
        "run": os.path.splitext(os.path.basename(path))[0],
        "n_rows": n_rows,
        "duration_s": round(duration_s, 2),
        "rmse_3d_mm": round(rmse_3d_mm, 2),
        "rmse_z_mm": round(rmse_z_mm, 2),
        "rmse_xy_mm": round(rmse_xy_mm, 2),
        "max_err_3d_mm": round(max_err_3d_mm, 2),
        "roll_std_deg": round(roll_std_deg, 3),
        "pitch_std_deg": round(pitch_std_deg, 3),
        "mo_m_est_final_kg": round(mo_m_est_final_kg, 4),
        "mo_m_est_tail_mean_kg": round(mo_m_est_tail_mean_kg, 4),
        "sigma_mag_mean": round(sigma_mag_mean, 4),
        "thrust_norm_mean": round(thrust_norm_mean, 4),
        "etc_event_count_total": etc_event_count_total,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="Folder berisi raw CSV per-run")
    ap.add_argument("--out", required=True, help="Folder output")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.dir, "*.csv")))
    if not files:
        print(f"Tidak ada CSV di {args.dir}", file=sys.stderr)
        sys.exit(1)

    os.makedirs(args.out, exist_ok=True)

    rows = []
    for f in files:
        try:
            rows.append(per_run_metrics(f))
        except Exception as e:
            print(f"  [SKIP] {f}: {e}", file=sys.stderr)

    if not rows:
        print("Semua file gagal diproses.", file=sys.stderr)
        sys.exit(1)

    per_run_df = pd.DataFrame(rows)
    per_run_path = os.path.join(args.out, "metrics_raw_per_run.csv")
    per_run_df.to_csv(per_run_path, index=False)
    print(f"Per-run  -> {per_run_path}  ({len(per_run_df)} runs)")

    numeric_cols = [c for c in per_run_df.columns if c not in ("run",)]
    summary_rows = []
    for c in numeric_cols:
        vals = pd.to_numeric(per_run_df[c], errors="coerce")
        summary_rows.append({
            "metric": c,
            "mean": round(float(vals.mean()), 4),
            "std": round(float(vals.std()), 4),
            "min": round(float(vals.min()), 4),
            "max": round(float(vals.max()), 4),
            "n": int(vals.notna().sum()),
        })

    summary_df = pd.DataFrame(summary_rows)
    summary_path = os.path.join(args.out, "metrics_summary_mean_std.csv")
    summary_df.to_csv(summary_path, index=False)
    print(f"Summary  -> {summary_path}")
    print()
    print(summary_df.to_string(index=False))


if __name__ == "__main__":
    main()
