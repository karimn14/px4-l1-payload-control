#!/usr/bin/env python3
"""
analyze_flight_logs.py  —  L1+MO / PID flight-log analysis & publication plots  (v2)
======================================================================================

USAGE:

  # Drop standar (single payload):
  python3 analyze_flight_logs.py --drop-l1 l1_monitor_215516.csv --payload 1.0

  # L1 vs PID compare:
  python3 analyze_flight_logs.py \
      --drop-l1 drop_l1.csv --drop-pid drop_pid.csv --payload 1.0

  # Multi-drop skenario (beberapa drop events dalam satu log):
  python3 analyze_flight_logs.py \
      --drop-l1 multi_drop_l1.csv --payload 0.4 \
      --scenario multi_drop \
      --drop-times 15.2 45.7 76.3 \
      --drop-payloads 0.4 0.4 0.4

  # Pick-drop skenario (pick up lalu drop):
  python3 analyze_flight_logs.py \
      --drop-l1 pick_drop_l1.csv --payload 0.6 \
      --scenario pick_drop \
      --pick-time 20.5 \
      --drop-times 45.0

  # Hover sweep multi-mass:
  python3 analyze_flight_logs.py \
      --hover hover_empty.csv hover_400g.csv hover_600g.csv hover_1kg.csv

  # Output ke folder custom + settle window berbeda:
  python3 analyze_flight_logs.py --drop-l1 ... --out ./results/ --settle 8

SCENARIO TYPES (--scenario):
  single      : satu drop event, satu payload  (default)
  multi_drop  : beberapa drop events berurutan
  pick_drop   : pick-up event diikuti drop event

OUTPUT (Figure 1 = overview standar, Figure 2 = detail analisis):
  fig1_drop_<stem>.png       — 9-panel overview: altitude, attitude, mass, XY, sigma, velocity, dll
  fig2_drop_<stem>.png       — detail analisis: phase portrait, spektrum, ETC stats, inter-event, dll
  fig1_drop_compare_<stem>.png
  fig2_drop_compare_<stem>.png
  hover_sweep.png / hover_timeseries.png
  metrics_hover.csv / .md
  metrics_drop.csv  / .md
  calibration_check.txt
"""

import argparse, os, sys, math, textwrap, warnings
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection
from pathlib import Path

warnings.filterwarnings("ignore", category=RuntimeWarning)

# ══════════════════════════════════════════════════════════════════════════════
#  KONFIGURASI GLOBAL
# ══════════════════════════════════════════════════════════════════════════════
EMPTY_MASS    = 2.064
F_MAX         = 40.65
KQ            = 32.02
KL            = 17.61
G             = 9.81
SIG_LIM       = 2.0
SETTLE_TOL    = 0.10
STEADY_OFFSET = 8.0
DROP_GUARD    = 3.0
RECOVERY_MM   = 30.0

# ── Warna ──────────────────────────────────────────────────────────────────
L1_COLORS  = ["tab:blue", "tab:green", "tab:red", "tab:purple",
               "tab:brown", "tab:pink", "tab:olive", "tab:cyan"]
PID_COLOR  = "0.40"

# Warna khusus per event type
CMAP_DROP   = "darkorange"
CMAP_PICK   = "steelblue"
CMAP_EVENTS = [
    ("drop",  "darkorange", "⬇",  "solid",   "DROP"),
    ("pick",  "steelblue",  "⬆",  "dashdot", "PICK"),
]

# Marker style per skenario
SCENARIO_MARKERS = {
    "single":     [("drop", CMAP_DROP)],
    "multi_drop": [("drop", CMAP_DROP)],
    "pick_drop":  [("pick", CMAP_PICK), ("drop", CMAP_DROP)],
}

plt.rcParams.update({
    "figure.dpi"    : 130,
    "savefig.dpi"   : 160,
    "font.size"     : 9.5,
    "axes.grid"     : True,
    "grid.alpha"    : 0.28,
    "lines.linewidth": 1.5,
    "axes.titlesize": 10,
    "legend.fontsize": 7.5,
    "axes.spines.top"  : False,
    "axes.spines.right": False,
})

# ══════════════════════════════════════════════════════════════════════════════
#  UTILITY
# ══════════════════════════════════════════════════════════════════════════════
def stem(path):
    return Path(path).stem

def load(path):
    d = pd.read_csv(path)
    d.columns = [c.strip() for c in d.columns]
    return d

def v(s):
    return np.asarray(s, dtype=float)

def col(d, name, default=0.0):
    return d[name] if name in d.columns else pd.Series([default]*len(d), index=d.index)

def _f(val, fmt=".1f"):
    return f"{val:{fmt}}" if (val is not None and math.isfinite(float(val))) else "—"

def _save(fig, path):
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print(f"  → {path}")

def safe_nanmean(arr):
    a = np.asarray(arr, dtype=float)
    return float(np.nanmean(a)) if len(a) > 0 else float("nan")

def safe_nanstd(arr):
    a = np.asarray(arr, dtype=float)
    return float(np.nanstd(a)) if len(a) > 0 else float("nan")

# ── Metrik tracking tambahan (selain RMSE) ──────────────────────────────────
def _mae(arr):
    """Mean Absolute Error — lebih tahan outlier dibanding RMSE, baik untuk
    menilai akurasi tracking 'rata-rata' tanpa terlalu didominasi 1-2 lonjakan."""
    a = np.asarray(arr, dtype=float); a = a[np.isfinite(a)]
    return float(np.mean(np.abs(a))) if len(a) else float("nan")

def _sse(arr):
    """Sum of Squared Error — total error kuadrat mentah (TIDAK dinormalisasi N
    atau dt). Hanya bisa dibandingkan antar run kalau window & sample-rate
    sama; lihat juga _ise() utk versi yang invariant thd panjang/sample-rate."""
    a = np.asarray(arr, dtype=float); a = a[np.isfinite(a)]
    return float(np.sum(a**2)) if len(a) else float("nan")

def _ise(arr, dt):
    """Integral Squared Error ≈ SSE x dt — versi SSE yang dinormalisasi waktu
    (unit² x s), sehingga adil dibandingkan antar log dgn durasi/sample-rate
    berbeda. Pelengkap RMSE/SSE untuk membandingkan L1 vs PID secara adil."""
    s = _sse(arr)
    return s * dt if (math.isfinite(s) and math.isfinite(dt) and dt > 0) else float("nan")

def _itae(err, t_rel):
    """Integral Time-weighted Absolute Error, dihitung dari t_rel=0 (biasanya
    waktu event). Memberi bobot lebih besar ke error yang BERTAHAN LAMA,
    bukan cuma besar sesaat — pelengkap yang baik utk recovery_s (threshold
    biner) saat membandingkan kualitas transien L1 vs PID setelah drop."""
    e = np.asarray(err, dtype=float); t = np.asarray(t_rel, dtype=float)
    m = np.isfinite(e) & np.isfinite(t)
    if m.sum() < 2:
        return float("nan")
    e, t = e[m], t[m]
    w = np.abs(e) * np.clip(t, 0, None)
    _trapz = getattr(np, "trapezoid", None) or np.trapz
    return float(_trapz(w, t))

# ══════════════════════════════════════════════════════════════════════════════
#  DETEKSI & IDENTIFIKASI
# ══════════════════════════════════════════════════════════════════════════════
def settle_time(d, offset=0.0):
    """err_z < 80mm selama 3 detik berturut-turut."""
    err = d["err_z_m"].abs() if "err_z_m" in d.columns else (d["pos_z_m"] - d["sp_z_m"]).abs()
    stable = (err < 0.080)
    dt = float(d["t_s"].iloc[1] - d["t_s"].iloc[0]) if len(d) > 1 else 0.02
    win = max(1, int(3.0 / dt))
    rolling = stable.rolling(win, min_periods=win).sum()
    stable_rows = d[rolling >= win]
    if len(stable_rows) > 0:
        t0 = float(stable_rows["t_s"].iloc[0]) - 3.0
    else:
        t0 = float(d["t_s"].iloc[int(len(d) * 0.7)])
    return t0 + offset

def steady_window(d, offset=None):
    off = STEADY_OFFSET if offset is None else offset
    t0  = settle_time(d, off)
    w   = d[d["t_s"] >= t0]
    return w if len(w) >= 20 else d.tail(300)

def detect_controller(d):
    sigma_active = col(d, "sigma_z_ms2").std() > 0.01
    if sigma_active:
        return "l1"
    l1_columns = {"u_filt_z", "pred_ep_z", "mo_m_est_kg", "hover_m_est_kg",
                  "fused_m_est_kg", "etc_triggered"}
    has_l1_cols = len(l1_columns & set(d.columns)) >= 3
    if has_l1_cols:
        print("  ⚠ WARNING: Kolom L1MO ada tapi sigma_z=0 — gate tertutup atau param=0")
        return "l1_inactive"
    return "pid"


def detect_drop_time(d, delivery_xy=(5.0, 5.0), xy_tol=1.5):
    """Deteksi drop andal: window delivery + lonjakan naik + cross-check massa."""
    t_end = float(d["t_s"].max())
    dx, dy = delivery_xy
    near = (np.hypot(d["pos_x_m"]-dx, d["pos_y_m"]-dy) < xy_tol) & (d["t_s"] > t_end*0.62)
    win = d[near] if near.sum() >= 10 else d[d["t_s"] > t_end*0.62]

    # Lonjakan NAIK (vel_z paling negatif) — drop bikin drone ringan & naik
    i = win["vel_z_ms"].idxmin()
    t_drop = float(d["t_s"].loc[i])

    # Cross-check massa untuk L1
    if d["mo_m_est_kg"].std() > 0.01:
        dm = d["mo_m_est_kg"].diff()
        j = dm.loc[d[d["t_s"] > t_end*0.62].index].idxmin()
        t_mass = float(d["t_s"].loc[j])
        if abs(t_mass - t_drop) < 3.0:
            t_drop = t_mass
    return t_drop, float(d.loc[i, "err_z_m"] * 1e3)

def sample_rate(d):
    """Estimasi sample rate dari median dt."""
    if len(d) < 2:
        return 50.0
    dt = float(np.median(np.diff(v(d["t_s"]))))
    return 1.0 / dt if dt > 0 else 50.0

# ══════════════════════════════════════════════════════════════════════════════
#  EVENT MARKERS — inti baru untuk multi-drop & pick-drop
# ══════════════════════════════════════════════════════════════════════════════
class FlightEvents:
    """
    Kontainer semua event dalam satu flight log.
    events = list of dict:
        { 'type': 'drop'|'pick', 't': float, 'payload_kg': float,
          'label': str, 'color': str, 'ls': str, 'marker': str }
    """
    def __init__(self):
        self.events = []

    def add(self, event_type, t_abs, payload_kg, index=0):
        styles = {
            "drop": (CMAP_DROP, "--",   "⬇", f"DROP#{index+1} {payload_kg:.2f}kg"),
            "pick": (CMAP_PICK, "-.",   "⬆", f"PICK#{index+1} +{payload_kg:.2f}kg"),
        }
        clr, ls, mk, lbl = styles.get(event_type, (CMAP_DROP, "--", "●", f"EVENT#{index+1}"))
        self.events.append(dict(
            type=event_type, t=t_abs, payload_kg=payload_kg,
            label=lbl, color=clr, ls=ls, marker=mk, index=index,
        ))

    def sorted_events(self):
        return sorted(self.events, key=lambda e: e["t"])

    def drops(self):
        return [e for e in self.events if e["type"] == "drop"]

    def picks(self):
        return [e for e in self.events if e["type"] == "pick"]

    def draw_all(self, ax, t_ref=0.0, lw=2.0, alpha=0.85, label_offset=0.97,
                 annotate=True, skip_label=False):
        """
        Gambar semua event sebagai axvline ke ax.
        t_ref: referensi waktu (t_drop utama, biasanya drops()[0]['t'])
        """
        for e in self.sorted_events():
            t_rel = e["t"] - t_ref
            lbl = e["label"] if not skip_label else None
            ax.axvline(t_rel, color=e["color"], ls=e["ls"], lw=lw,
                       label=lbl, alpha=alpha, zorder=5)
            if annotate:
                ylim = ax.get_ylim()
                ax.text(t_rel, ylim[1] * label_offset + ylim[0] * (1 - label_offset),
                        f"  {e['marker']}", fontsize=7.5, color=e["color"],
                        va="top", rotation=0, clip_on=True)

    def draw_all_silent(self, ax, t_ref=0.0, lw=1.6, alpha=0.7):
        """Tanpa label — untuk panel detail agar tidak duplikat legend."""
        for e in self.sorted_events():
            t_rel = e["t"] - t_ref
            ax.axvline(t_rel, color=e["color"], ls=e["ls"], lw=lw, alpha=alpha, zorder=5)


def build_flight_events(scenario, drop_times_abs, pick_times_abs,
                        drop_payloads, pick_payloads):
    """
    Bangun FlightEvents dari argumen CLI.
    drop_times_abs, pick_times_abs: list of float (absolute t_s)
    drop_payloads, pick_payloads  : list of float (kg)
    """
    fe = FlightEvents()
    for i, (t, p) in enumerate(zip(drop_times_abs, drop_payloads)):
        fe.add("drop", t, p, index=i)
    for i, (t, p) in enumerate(zip(pick_times_abs, pick_payloads)):
        fe.add("pick", t, p, index=i)
    return fe


# ══════════════════════════════════════════════════════════════════════════════
#  PHASE SEGMENTATION — generik, adaptif thd skenario (single/multi_drop/pick_drop)
# ══════════════════════════════════════════════════════════════════════════════
def detect_flight_phases(d, fe):
    """
    Segmentasi mission jadi fase yang bisa dibaca manusia, dari sinyal kecepatan
    (vel_x/y/z_ms) + event pick/drop (FlightEvents). Tidak hardcode skenario —
    hasil otomatis menyesuaikan data, misalnya:
      single      : Takeoff/Climb -> Hover -> [DROP] -> Hover after Drop
      multi_drop  : Takeoff/Climb -> Hover -> [DROP] x N -> Hover after Drop
      pick_drop   : Takeoff/Climb -> Hover -> Goto Pickup -> [PICK] -> Hover ->
                    Goto Drop -> (Descend, jika terdeteksi) -> [DROP] -> Hover after Drop

    Return: list of dict {start, end, label, color}  (waktu ABSOLUT t_s, urut naik)
    """
    t_arr = v(d["t_s"])
    if len(t_arr) < 5:
        return [dict(start=float(t_arr.min()), end=float(t_arr.max()),
                      label="Flight", color="0.6")]
    t0, t_end = float(t_arr.min()), float(t_arr.max())
    dt = float(np.median(np.diff(t_arr))) if len(t_arr) > 1 else 0.02
    if not math.isfinite(dt) or dt <= 0:
        dt = 0.02

    vh   = np.hypot(v(col(d, "vel_x_ms")), v(col(d, "vel_y_ms")))
    vz   = v(col(d, "vel_z_ms"))                       # NED: + = turun
    win  = max(3, int(round(0.5 / dt)))
    vh_s = pd.Series(vh).rolling(win, min_periods=1, center=True).median().to_numpy()
    vz_s = pd.Series(vz).rolling(win, min_periods=1, center=True).median().to_numpy()

    TRANSIT_VH = 0.25   # m/s — ambang gerak horizontal signifikan
    VERT_TH    = 0.20   # m/s — ambang gerak vertikal signifikan

    raw = np.where(vh_s > TRANSIT_VH, "transit",
          np.where(vz_s >  VERT_TH,  "descend",
          np.where(vz_s < -VERT_TH,  "ascend",  "hover")))

    # run-length encode jadi segmen kontinu
    segs, s_idx = [], 0
    for i in range(1, len(raw) + 1):
        if i == len(raw) or raw[i] != raw[s_idx]:
            segs.append([float(t_arr[s_idx]), float(t_arr[i - 1]), str(raw[s_idx])])
            s_idx = i

    # gabung segmen pendek (<1.0s) ke segmen sebelumnya — biar tidak berisik
    MIN_DUR = 1.0
    merged = []
    for seg in segs:
        if merged and (seg[1] - seg[0]) < MIN_DUR:
            merged[-1][1] = seg[1]
        else:
            merged.append(list(seg))

    # climb awal pakai settle_time() (lebih akurat dari threshold vz mentah)
    climb_end = min(max(settle_time(d, 0.0), t0), t_end)
    out = [dict(start=t0, end=climb_end, kind="climb")]
    cursor = climb_end
    for s, e, k in merged:
        if e <= cursor:
            continue
        out.append(dict(start=max(s, cursor), end=e, kind=k))
        cursor = e
    if cursor < t_end:
        out.append(dict(start=cursor, end=t_end, kind=(out[-1]["kind"] if out else "hover")))

    # potong segmen yang "menelan" sebuah event pick/drop di tengahnya, supaya
    # bagian sebelum & sesudah event tetap bisa dikasih label terpisah
    # (mis. hover datar yg tidak berubah kecepatan saat drop pada skenario single).
    events_sorted = fe.sorted_events()
    ev_times = sorted(e["t"] for e in events_sorted)
    split_out = []
    for seg in out:
        s, e, k = seg["start"], seg["end"], seg["kind"]
        cuts = [s] + [t for t in ev_times if s < t < e] + [e]
        for a, b in zip(cuts[:-1], cuts[1:]):
            if b > a:
                split_out.append(dict(start=a, end=b, kind=k))
    out = split_out

    # label kontekstual: "transit" -> "Goto Pickup" / "Goto Drop" sesuai event berikutnya;
    # hover terakhir setelah event terakhir -> "Hover after Drop/Pickup"
    def _next_event(t):
        for e in events_sorted:
            if e["t"] >= t - 0.5:
                return e
        return None

    LBL = {"climb": "Takeoff/Climb", "hover": "Hover", "descend": "Descend",
           "ascend": "Climb", "transit": "Transit"}
    CLR = {"climb": "0.55", "hover": "steelblue", "descend": "darkorange",
           "ascend": "0.55", "transit": "mediumpurple"}

    last_ev_t    = events_sorted[-1]["t"]    if events_sorted else None
    last_ev_type = events_sorted[-1]["type"] if events_sorted else None

    final = []
    for seg in out:
        kind  = seg["kind"]
        label = LBL.get(kind, kind)
        color = CLR.get(kind, "gray")
        if kind == "transit":
            nev = _next_event(seg["end"])
            if nev is not None and nev["type"] == "pick":
                label = "Goto Pickup"
            elif nev is not None and nev["type"] == "drop":
                label = "Goto Drop"
            else:
                label = "Transit"
        if kind == "hover" and last_ev_t is not None and seg["start"] >= last_ev_t - 0.5:
            label = "Hover after Drop" if last_ev_type == "drop" else "Hover after Pickup"
        final.append(dict(start=seg["start"], end=seg["end"], label=label, color=color))

    # gabung label identik yang jadi bersebelahan akibat override di atas
    collapsed = []
    for seg in final:
        if collapsed and collapsed[-1]["label"] == seg["label"]:
            collapsed[-1]["end"] = seg["end"]
        else:
            collapsed.append(dict(seg))
    return collapsed


# ══════════════════════════════════════════════════════════════════════════════
#  METRICS
# ══════════════════════════════════════════════════════════════════════════════
def hover_metrics(d, true_mass):
    w = steady_window(d)
    W = true_mass * G
    m = float(col(w, "mo_m_est_kg").mean())
    err_z_mm  = v(col(w,"err_z_m")) * 1e3
    err_3d_mm = v(w["err_3d_m"]) * 1e3
    dt_w = float(np.median(np.diff(v(w["t_s"])))) if len(w) > 1 else float("nan")
    return dict(
        true_mass_kg     = true_mass,
        payload_kg       = true_mass - EMPTY_MASS,
        settle_s         = settle_time(d),
        n_steady         = len(w),
        rmse_3d_mm       = math.sqrt((v(w["err_3d_m"])**2).mean()) * 1e3,
        rmse_z_mm        = math.sqrt((v(col(w,"err_z_m"))**2).mean()) * 1e3,
        rmse_xy_mm       = math.sqrt((v(col(w,"err_x_m"))**2 + v(col(w,"err_y_m"))**2).mean()) * 1e3,
        mae_z_mm         = _mae(err_z_mm),
        mae_3d_mm        = _mae(err_3d_mm),
        sse_z_mm2        = _sse(err_z_mm),
        ise_z_mm2_s      = _ise(err_z_mm, dt_w),
        max_err_mm       = float(v(w["err_3d_m"]).max()) * 1e3,
        roll_std_deg     = float(v(col(w,"roll_act_deg")).std()),
        pitch_std_deg    = float(v(col(w,"pitch_act_deg")).std()),
        roll_p2p_deg     = float(v(col(w,"roll_act_deg")).max() - v(col(w,"roll_act_deg")).min()),
        mo_m_est_kg      = m,
        mo_m_std_kg      = float(col(w,"mo_m_est_kg").std()),
        mo_err_pct       = (m - true_mass) / true_mass * 100 if true_mass > 0 else float("nan"),
        hover_m_est_kg   = float(col(w,"hover_m_est_kg").mean()),
        fused_m_est_kg   = float(col(w,"fused_m_est_kg").mean()),
        hover_valid_frac = float(col(w,"hover_est_valid").mean()),
        sigma_z_mean     = float(col(w,"sigma_z_ms2").mean()),
        sigma_z_std      = float(col(w,"sigma_z_ms2").std()),
        sigma_saturated  = float((np.abs(v(col(w,"sigma_z_ms2"))) >= SIG_LIM).mean()),
        u_filt_z_mean    = float(col(w,"u_filt_z").mean()),
        u_filt_z_err_N   = float(col(w,"u_filt_z").mean()) - W,
        thrust_norm_mean = float(col(w,"thrust_norm").mean()),
        controller       = detect_controller(d),
    )

def drop_metrics_single(d, t_drop, true_payload):
    """Metrics untuk SATU drop event."""
    pre  = d[(d["t_s"] >= t_drop - 5)  & (d["t_s"] < t_drop - 0.5)]
    post = d[(d["t_s"] >= t_drop)      & (d["t_s"] <= t_drop + 10)]
    tail = d[d["t_s"] >= max(t_drop + 6, float(d["t_s"].max()) - 4)]

    if len(post):
        ei      = v(post["err_z_m"])
        pk_idx  = int(np.argmax(np.abs(ei)))
        peak_mm = float(ei[pk_idx] * 1e3)
        tpk     = float(post["t_s"].iloc[pk_idx])
    else:
        peak_mm = float("nan"); tpk = t_drop

    rec_rows = post[(post["t_s"] > tpk) & (post["err_z_m"].abs() < RECOVERY_MM/1e3)]
    rec_s    = float(rec_rows["t_s"].iloc[0] - t_drop) if len(rec_rows) else float("nan")

    pre_rmse_z = math.sqrt((v(col(pre,"err_z_m"))**2).mean())*1e3 if len(pre)>5 else float("nan")
    pre_rmse3d = math.sqrt((v(pre["err_3d_m"])**2).mean())*1e3    if len(pre)>5 else float("nan")
    pre_roll   = float(v(col(pre,"roll_act_deg")).std())            if len(pre)>5 else float("nan")
    pre_thr    = float(col(pre,"thrust_norm").mean())               if len(pre)>5 else float("nan")

    # ── Metrik tracking tambahan: MAE / SSE (pre & post steady-state) ────────
    #   pre  = steady-state SEBELUM drop (window ~4.5s, sama spt pre_rmse_z)
    #   tail = steady-state SETELAH settle pasca-drop (sama spt post_rmse_z)
    pre_mae_z   = _mae(v(col(pre, "err_z_m")) * 1e3)  if len(pre)  > 5 else float("nan")
    pre_sse_z   = _sse(v(col(pre, "err_z_m")) * 1e3)  if len(pre)  > 5 else float("nan")
    post_mae_z  = _mae(v(col(tail,"err_z_m")) * 1e3)  if len(tail) > 5 else float("nan")
    post_sse_z  = _sse(v(col(tail,"err_z_m")) * 1e3)  if len(tail) > 5 else float("nan")

    # ── Metrik transien (window dinamis t_drop..t_drop+10, sama spt "post"):
    #   trans_mae   = akurasi rata-rata selama transien recovery
    #   trans_ise   = energi error kuadrat selama transien (rate-independent)
    #   trans_itae  = makin lama error bertahan, makin besar penalti — pelengkap
    #                 yang baik utk recovery_s krn recovery_s itu biner (threshold)
    if len(post) > 5:
        err_post_mm = v(post["err_z_m"]) * 1e3
        t_rel_post  = v(post["t_s"]) - t_drop
        dt_post     = float(np.median(np.diff(v(post["t_s"])))) if len(post) > 1 else float("nan")
        trans_mae_z  = _mae(err_post_mm)
        trans_ise_z  = _ise(err_post_mm, dt_post)
        trans_itae_z = _itae(err_post_mm, t_rel_post)
    else:
        trans_mae_z = trans_ise_z = trans_itae_z = float("nan")

    # ── Deteksi massa: data-driven, TIDAK bergantung t_drop ──
    #   Pisahkan plateau loaded vs empty dari nilai massa sendiri.
    #   Robust thd: nilai absolut MO bias, lag konvergensi, t_drop salah.
    #   PID / MO-off (mo flat) → N/A (NaN), BUKAN 0 / 100%.
    mo_before = mo_after = float("nan")
    _ctrl  = detect_controller(d)
    ms     = col(d, "mo_m_est_kg").rolling(15, min_periods=1, center=True).median()
    vz     = col(d, "vel_z_ms")
    steady = (vz.abs() < 0.3)                       # hover saja (buang climb & transien drop)
    ms_s   = ms[steady].dropna()
    has_mo = ("mo_m_est_kg" in d.columns) and (_ctrl in ("l1", "l1_inactive")) and (len(ms_s) > 20)

    if has_mo:
        q_hi, q_lo = ms_s.quantile(0.80), ms_s.quantile(0.20)
        if (q_hi - q_lo) >= 0.05:                   # massa benar2 punya 2 level berbeda
            mid = 0.5 * (q_hi + q_lo)
            hi  = ms_s[ms_s >= mid]
            lo  = ms_s[ms_s <  mid]
            if len(hi) >= 5:
                mo_before = float(hi.median())       # loaded plateau
                t_last    = float(d["t_s"].max())
                ms_tail   = ms[(d["t_s"] >= t_last - 5) & steady].dropna()   # empty plateau settled
                if len(ms_tail) >= 5:
                    mo_after = float(ms_tail.median())
                elif len(lo) >= 5:
                    mo_after = float(lo.median())

    if math.isfinite(mo_before) and math.isfinite(mo_after):
        delta     = mo_before - mo_after
        delta_err = abs(delta - true_payload)/true_payload*100 if true_payload > 0 else float("nan")
    else:
        delta     = float("nan")    # PID / MO-off / massa tak terpisah → N/A
        delta_err = float("nan")

    post_rmse = math.sqrt((v(col(tail,"err_z_m"))**2).mean())*1e3 if len(tail)>5 else float("nan")
    post_roll = float(v(col(tail,"roll_act_deg")).std())           if len(tail)>5 else float("nan")
    pre_sz    = float(col(pre,"sigma_z_ms2").mean()) if len(pre)>5 else float("nan")
    pre_sf    = float((np.abs(v(col(pre,"sigma_z_ms2"))) >= SIG_LIM).mean()) if len(pre)>5 else float("nan")

    # ── METRIK TAMBAHAN: XY error, sigma std, ETC (untuk analisis sweep lebih kaya) ──
    def _rmse_xy(seg):
        if len(seg) <= 5 or "err_x_m" not in seg.columns:
            return float("nan")
        ex = v(col(seg, "err_x_m")); ey = v(col(seg, "err_y_m"))
        return float(np.sqrt((ex**2 + ey**2).mean()) * 1e3)
    pre_rmse_xy  = _rmse_xy(pre)
    post_rmse_xy = _rmse_xy(tail)
    if len(post) and "err_x_m" in post.columns:
        exy = np.sqrt(v(col(post, "err_x_m"))**2 + v(col(post, "err_y_m"))**2)
        peak_xy_mm = float(np.max(exy) * 1e3) if len(exy) else float("nan")
    else:
        peak_xy_mm = float("nan")
    pre_sigma_z_std = float(col(pre, "sigma_z_ms2").std()) if len(pre) > 5 else float("nan")
    # ETC statistik untuk seluruh run (None jika controller tanpa ETC, mis. PID)
    _es = etc_stats(d)
    etc_rate     = float(_es["trigger_rate"]) if _es else float("nan")
    etc_iei_mean = float(_es["iei_mean"])     if _es else float("nan")
    etc_n_trig   = int(_es["n_triggered"])    if _es else 0

    return dict(
        t_drop_s=t_drop, true_payload_kg=true_payload,
        pre_rmse3d_mm=pre_rmse3d, pre_rmse_z_mm=pre_rmse_z,
        pre_mae_z_mm=pre_mae_z, pre_sse_z_mm2=pre_sse_z,
        pre_roll_std_deg=pre_roll, pre_sigma_z=pre_sz,
        pre_sigma_sat_frac=pre_sf, pre_thrust_norm=pre_thr,
        peak_alt_err_mm=peak_mm, recovery_s=rec_s,
        trans_mae_z_mm=trans_mae_z, trans_ise_z_mm2_s=trans_ise_z,
        trans_itae_z_mm_s2=trans_itae_z,
        post_rmse_z_mm=post_rmse, post_mae_z_mm=post_mae_z, post_sse_z_mm2=post_sse_z,
        post_roll_std_deg=post_roll,
        mo_before_kg=mo_before, mo_after_kg=mo_after,
        detected_delta_kg=delta, delta_err_pct=delta_err,
        pre_rmse_xy_mm=pre_rmse_xy, post_rmse_xy_mm=post_rmse_xy, peak_xy_err_mm=peak_xy_mm,
        pre_sigma_z_std=pre_sigma_z_std,
        etc_rate=etc_rate, etc_iei_mean=etc_iei_mean, etc_n_trig=etc_n_trig,
        controller=detect_controller(d),
    )

def drop_metrics(d, fe, true_payload):
    """
    Hitung metrics untuk SEMUA drop events di FlightEvents.
    Return dict utama (event pertama) + list semua event metrics.
    """
    drops = fe.drops()
    if not drops:
        t_auto, _ = detect_drop_time(d)
        if t_auto is None:
            t_auto = float(d["t_s"].median())
        dm = drop_metrics_single(d, t_auto, true_payload)
        dm["all_events"] = [dm]
        return dm

    all_dm = []
    for ev in drops:
        dm_ev = drop_metrics_single(d, ev["t"], ev["payload_kg"])
        dm_ev["event_label"] = ev["label"]
        all_dm.append(dm_ev)

    # dm utama = event pertama
    dm_main = all_dm[0].copy()
    dm_main["all_events"] = all_dm

    # statistik agregat (multi-drop)
    if len(all_dm) > 1:
        dm_main["recovery_s_mean"]  = safe_nanmean([m["recovery_s"]      for m in all_dm])
        dm_main["recovery_s_std"]   = safe_nanstd( [m["recovery_s"]      for m in all_dm])
        dm_main["peak_alt_err_mean"]= safe_nanmean([m["peak_alt_err_mm"]  for m in all_dm])
        dm_main["delta_err_mean"]   = safe_nanmean([m["delta_err_pct"]    for m in all_dm])

    return dm_main


# ══════════════════════════════════════════════════════════════════════════════
#  ETC STATISTICS
# ══════════════════════════════════════════════════════════════════════════════
def etc_stats(d):
    """Return dict of ETC trigger statistics, or None if no ETC column."""
    if "etc_triggered" not in d.columns:
        return None
    etc = v(col(d, "etc_triggered")).astype(bool)
    n_total   = len(etc)
    n_trig    = int(etc.sum())
    trig_rate = n_trig / n_total if n_total > 0 else 0.0
    # inter-event intervals
    t_all = v(d["t_s"])
    t_trig = t_all[etc]
    iei = np.diff(t_trig) if len(t_trig) > 1 else np.array([])
    return dict(
        n_total=n_total, n_triggered=n_trig, trigger_rate=trig_rate,
        iei=iei,
        iei_mean=float(np.mean(iei)) if len(iei) else float("nan"),
        iei_std =float(np.std(iei))  if len(iei) else float("nan"),
        iei_min =float(np.min(iei))  if len(iei) else float("nan"),
        iei_max =float(np.max(iei))  if len(iei) else float("nan"),
        t_triggered=t_trig,
    )

# ══════════════════════════════════════════════════════════════════════════════
#  CALIBRATION CHECK
# ══════════════════════════════════════════════════════════════════════════════
def calibration_check(hover_rows, out_path):
    lines = ["CALIBRATION HEALTH CHECK", "="*60, "",
             f"  F_MAX={F_MAX}  KQ={KQ}  KL={KL}  SIG_LIM={SIG_LIM}",
             f"  EMPTY_MASS={EMPTY_MASS} kg\n",
             "  Criteria:", "    sigma_z |mean| < 0.10 m/s²",
             "    u_filt_z error < ±0.5 N", "    mo_m_est ±0.020 kg", "    hover_valid > 0.80\n"]
    all_pass = True
    for hr in hover_rows:
        if abs(hr["true_mass_kg"] - EMPTY_MASS) > 0.05: continue
        checks = [
            ("sigma_z |mean|",   abs(hr["sigma_z_mean"]) < 0.10, f"{hr['sigma_z_mean']:+.4f}", "< 0.10"),
            ("u_filt_z err (N)", abs(hr["u_filt_z_err_N"]) < 0.5, f"{hr['u_filt_z_err_N']:+.3f}", "< ±0.5"),
            ("mo_m_est (kg)",    abs(hr["mo_m_est_kg"]-hr["true_mass_kg"])<0.020,
             f"{hr['mo_m_est_kg']:.4f}", f"{hr['true_mass_kg']:.3f}±0.020"),
            ("hover_valid_frac", hr["hover_valid_frac"] > 0.80, f"{hr['hover_valid_frac']:.2f}", "> 0.80"),
        ]
        lines.append("  --- Empty hover ---")
        for name, ok, val, target in checks:
            lines.append(f"    {'✓' if ok else '✗'} {name:<22} {val:<12} target {target}")
            if not ok: all_pass = False
    lines += ["", "  Overall: " + ("✓ OK" if all_pass else "✗ NEEDS ATTENTION")]
    txt = "\n".join(lines)
    print(txt)
    with open(out_path, "w") as f: f.write(txt + "\n")

# ══════════════════════════════════════════════════════════════════════════════
#  HOVER PLOTS (unchanged from v1)
# ══════════════════════════════════════════════════════════════════════════════
def plot_hover_sweep(hover_rows, outdir):
    rows = sorted(hover_rows, key=lambda r: r["true_mass_kg"])
    pl   = [r["payload_kg"] for r in rows]
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    ax = axes[0]
    ax.plot(pl, [r["rmse_3d_mm"] for r in rows], "o-", color="tab:blue",   label="RMSE 3D")
    ax.plot(pl, [r["rmse_z_mm"]  for r in rows], "s--",color="tab:cyan",   label="RMSE Z")
    ax.plot(pl, [r["rmse_xy_mm"] for r in rows], "^:", color="tab:purple", label="RMSE XY", alpha=0.7)
    ax.set_xlabel("payload (kg)"); ax.set_ylabel("error (mm)"); ax.set_title("Position tracking"); ax.legend()
    ax = axes[1]
    ax.plot(pl, [r["roll_std_deg"]  for r in rows], "o-",  color="tab:red",   label="roll std")
    ax.plot(pl, [r["pitch_std_deg"] for r in rows], "s--", color="tab:orange",label="pitch std")
    ax.plot(pl, [r["roll_p2p_deg"]  for r in rows], "^:",  color="tab:pink",  label="roll p2p", alpha=0.7)
    ax.set_xlabel("payload (kg)"); ax.set_ylabel("attitude (deg)"); ax.set_title("Attitude stability"); ax.legend()
    ax = axes[2]
    ax.plot(pl, [EMPTY_MASS+p for p in pl], "k--", lw=1.2, label="true mass")
    ax.errorbar(pl, [r["mo_m_est_kg"] for r in rows], yerr=[r["mo_m_std_kg"] for r in rows],
                fmt="o-", color="tab:blue", capsize=3, label="MO")
    for p, r in zip(pl, rows):
        ax.annotate(f"{r['mo_err_pct']:+.1f}%", (p, r["mo_m_est_kg"]),
                    textcoords="offset points", xytext=(5, 6), fontsize=7)
    ax.set_xlabel("payload (kg)"); ax.set_ylabel("mass (kg)"); ax.set_title("Mass estimation"); ax.legend()
    fig.suptitle("Hover performance — L1+MO", fontsize=12, y=1.01)
    fig.tight_layout()
    _save(fig, os.path.join(outdir, "hover_sweep.png"))

def plot_hover_timeseries(hover_dfs, hover_rows, outdir):
    rows = sorted(zip(hover_rows, hover_dfs), key=lambda x: x[0]["true_mass_kg"])
    cmap = plt.cm.viridis(np.linspace(0.1, 0.9, len(rows)))
    fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for (hr, d), c in zip(rows, cmap):
        lbl = f"{hr['payload_kg']:.1f}kg"
        t = v(d["t_s"])
        ax[0].plot(t, v(d["err_3d_m"])*1e3, color=c, label=lbl)
        ax[1].plot(t, v(col(d,"mo_m_est_kg")), color=c, label=lbl)
        ax[1].axhline(hr["true_mass_kg"], color=c, ls=":", lw=0.8, alpha=0.6)
        ax[2].plot(t, v(col(d,"sigma_z_ms2")), color=c, label=lbl)
    ax[0].set_ylabel("3D error (mm)"); ax[0].set_title("Position error"); ax[0].legend(ncol=4)
    ax[1].set_ylabel("mass est (kg)"); ax[1].set_title("MO mass estimate")
    ax[2].set_ylabel("σ_z (m/s²)"); ax[2].set_title("L1 disturbance σ_z")
    ax[2].axhline( SIG_LIM, color="r", ls=":", lw=1)
    ax[2].axhline(-SIG_LIM, color="r", ls=":", lw=1, label=f"SIG_LIM={SIG_LIM}")
    ax[2].legend(ncol=4); ax[2].set_xlabel("time (s)")
    fig.tight_layout()
    _save(fig, os.path.join(outdir, "hover_timeseries.png"))

# ══════════════════════════════════════════════════════════════════════════════
#  FIGURE 1 — OVERVIEW STANDAR (9 panel)
# ══════════════════════════════════════════════════════════════════════════════
def plot_fig1_single(d, dm, fe, true_payload, ctrl_label, color, outdir, csv_path, scenario="single"):
    """
    Figure 1: 9-panel overview standar.
    Panel: altitude, altitude error, roll/pitch attitude, mass estimation,
           L1 sigma, thrust+ETC, XY position error, XY ground track, velocity.
    Drop markers disesuaikan dengan scenario (single / multi_drop / pick_drop).
    """
    drops = fe.drops()
    # referensi waktu = drop pertama (atau auto-detect jika tidak ada)
    if drops:
        t_ref = drops[0]["t"]
    else:
        t_ref = dm.get("t_drop_s", float(d["t_s"].median()))

    tr     = v(d["t_s"]) - t_ref
    t0_abs = v(d["t_s"]).min() - t_ref
    t_end  = v(d["t_s"]).max() - t_ref
    m_true_loaded = EMPTY_MASS + true_payload
    ctrl   = detect_controller(d)
    is_l1  = ctrl in ("l1", "l1_inactive")
    l1_inactive = ctrl == "l1_inactive"
    fname  = f"fig1_drop_{'l1' if is_l1 else 'pid'}_{stem(csv_path)}.png"

    fig = plt.figure(figsize=(18, 15))
    gs  = gridspec.GridSpec(3, 3, figure=fig, hspace=0.50, wspace=0.40)

    def draw_ev(ax, annotate=True):
        fe.draw_all(ax, t_ref=t_ref, lw=1.9, annotate=annotate)

    # ── Panel 1 [0,0:2]: Altitude trajectory ─────────────────────────────
    # Catatan: pos_z_m/sp_z_m disimpan dalam frame NED (z-down = positif),
    # di-flip jadi "altitude" (positif = naik) supaya plot intuitif dibaca.
    ax1 = fig.add_subplot(gs[0, :2])
    alt    = -v(d["pos_z_m"])
    alt_sp = -v(d["sp_z_m"])
    ax1.plot(tr, alt,    color=color, lw=2.0, label="actual altitude", zorder=4)
    ax1.plot(tr, alt_sp, color="gray", lw=1.1, ls="--", label="setpoint altitude", alpha=0.8)

    # Beri headroom di atas data utk baris label fase, supaya TIDAK bertumpuk
    # dgn judul panel (title selalu di luar axes, jadi aman selama label fase
    # kita taruh di DALAM ylim, bukan di atas axes via transAxes).
    alt_min  = float(np.nanmin(alt)) if np.isfinite(alt).any() else 0.0
    alt_max  = float(np.nanmax(alt)) if np.isfinite(alt).any() else 1.0
    rng      = max(alt_max - alt_min, 0.5)
    y_bottom = min(0.0, alt_min) - 0.08 * rng
    y_top    = alt_max + 0.32 * rng
    y_label  = alt_max + 0.16 * rng
    ax1.set_ylim(y_bottom, y_top)

    # Shading + label fase mission, otomatis dari detect_flight_phases()
    # (Takeoff/Climb, Hover, Goto Pickup, Descend, Goto Drop, Hover after Drop, dst.)
    phases     = detect_flight_phases(d, fe)
    span_total = (t_end - t0_abs) if (t_end - t0_abs) > 0 else 1.0
    for ph in phases:
        s_rel = max(ph["start"] - t_ref, t0_abs)
        e_rel = min(ph["end"]   - t_ref, t_end)
        if e_rel <= s_rel:
            continue
        ax1.axvspan(s_rel, e_rel, alpha=0.10, color=ph["color"], lw=0, zorder=1)
        if (e_rel - s_rel) / span_total > 0.035:   # skip label kalau segmen terlalu sempit
            ax1.text((s_rel + e_rel) / 2, y_label, ph["label"],
                      ha="center", va="bottom", fontsize=6.3, color=ph["color"],
                      style="italic", clip_on=True)

    draw_ev(ax1)
    ax1.set_xlim(t0_abs, t_end)
    ax1.set_xlabel("time relative to first DROP (s)")
    ax1.set_ylabel("Altitude (m)")
    ax1.legend(fontsize=7, ncol=3, loc="lower left")

    # ── Panel 2 [0,2]: Altitude error ────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(tr, v(d["err_z_m"])*1e3, color=color, lw=1.8)
    ax2.axhline( RECOVERY_MM, color="green", ls=":", lw=1.2, label=f"+{RECOVERY_MM:.0f}mm")
    ax2.axhline(-RECOVERY_MM, color="green", ls=":", lw=1.2, label=f"−{RECOVERY_MM:.0f}mm")
    ax2.axhline(0, color="k", lw=0.4)
    # annotasi recovery per drop event
    all_ev = dm.get("all_events", [dm])
    for ev_dm in all_ev:
        rc = ev_dm.get("recovery_s", float("nan"))
        t_ev = ev_dm.get("t_drop_s", t_ref) - t_ref
        if math.isfinite(rc) and rc > 0:
            ax2.annotate("", xy=(t_ev + rc, 0), xytext=(t_ev, 0),
                         arrowprops=dict(arrowstyle="->", color="green", lw=1.2))
            ax2.text(t_ev + rc/2, RECOVERY_MM*1.3,
                     f"{rc:.2f}s", fontsize=7, color="green", ha="center")
    fe.draw_all_silent(ax2, t_ref=t_ref)
    ax2.set_xlim(t0_abs, t_end)
    ax2.set_xlabel("time relative to first DROP (s)")
    ax2.set_ylabel("altitude error (mm)")
    ax2.set_title("Altitude Error & Recovery")
    ax2.legend(fontsize=6.5, loc="lower right")

    # ── Panel 3 [1,0]: Roll & Pitch attitude ─────────────────────────────
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(tr, v(col(d,"roll_act_deg")),  color="tab:red",   lw=1.4, label="roll act")
    ax3.plot(tr, v(col(d,"pitch_act_deg")), color="tab:blue",  lw=1.4, label="pitch act")
    if "roll_cmd_deg" in d.columns:
        ax3.plot(tr, v(col(d,"roll_cmd_deg")),  color="tab:red",  lw=0.7, ls="--", alpha=0.5, label="roll cmd")
        ax3.plot(tr, v(col(d,"pitch_cmd_deg")), color="tab:blue", lw=0.7, ls="--", alpha=0.5, label="pitch cmd")
    ax3.axhline(0, color="k", lw=0.4)
    fe.draw_all_silent(ax3, t_ref=t_ref)
    ax3.set_xlim(t0_abs, t_end)
    ax3.set_xlabel("time relative to first DROP (s)")
    ax3.set_ylabel("angle (deg)")
    ax3.set_title("Roll & Pitch Attitude")
    ax3.legend(fontsize=6.5)

    # ── Panel 4 [1,1]: Mass estimation ───────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 1])
    if is_l1:
        ax4.plot(tr, v(col(d,"mo_m_est_kg")), color="tab:blue", lw=1.6, label="MO estimate")
        ax4.axhline(m_true_loaded, color="red",   ls="--", lw=1.2, label=f"true loaded {m_true_loaded:.3f}kg")
        ax4.axhline(EMPTY_MASS,   color="black", ls=":",  lw=1.2, label=f"true empty {EMPTY_MASS:.3f}kg")
        # annotasi Δm per drop
        for ev_dm in all_ev:
            dd = ev_dm.get("detected_delta_kg", float("nan"))
            de = ev_dm.get("delta_err_pct",     float("nan"))
            t_ev = ev_dm.get("t_drop_s", t_ref) - t_ref
            mb   = ev_dm.get("mo_before_kg", float("nan"))
            ma   = ev_dm.get("mo_after_kg",  float("nan"))
            if math.isfinite(dd):
                x_ann = t_ev + 2.5
                ax4.annotate(f"Δ={_f(dd,'.3f')}kg\n({_f(de,'.1f')}%)",
                             (x_ann, (mb+ma)/2 if math.isfinite(mb) else m_true_loaded),
                             fontsize=7, color="purple", ha="left")
        ax4.set_ylim(1.0, m_true_loaded + 0.5)
    else:
        ax4.axhline(EMPTY_MASS, color="gray", ls="--", lw=1.5, label="fixed (no estimator)")
    fe.draw_all_silent(ax4, t_ref=t_ref)
    ax4.set_xlim(t0_abs, t_end)
    ax4.set_xlabel("time relative to first DROP (s)")
    ax4.set_ylabel("mass (kg)")
    ax4.legend(fontsize=6, loc="upper right", ncol=2)

    # ── Panel 5 [1,2]: L1 sigma / Attitude (PID fallback) ────────────────
    ax5 = fig.add_subplot(gs[1, 2])
    if is_l1:
        sz = v(col(d,"sigma_z_ms2"))
        ax5.plot(tr, sz,                       color="tab:red",   lw=1.5, label="σ_z")
        ax5.plot(tr, v(col(d,"sigma_x_ms2")), color="tab:blue",  lw=0.9, ls="--", alpha=0.7, label="σ_x")
        ax5.plot(tr, v(col(d,"sigma_y_ms2")), color="tab:green", lw=0.9, ls="--", alpha=0.7, label="σ_y")
        ax5.axhline( SIG_LIM, color="darkred", ls=":", lw=1.2, label=f"±SIG_LIM={SIG_LIM}")
        ax5.axhline(-SIG_LIM, color="darkred", ls=":", lw=1.2)
        ax5.axhline(0, color="k", lw=0.4)
        ax5.fill_between(tr,  SIG_LIM, sz.clip(min=SIG_LIM),  where=(sz >  SIG_LIM), color="red",  alpha=0.20)
        ax5.fill_between(tr, -SIG_LIM, sz.clip(max=-SIG_LIM), where=(sz < -SIG_LIM), color="red",  alpha=0.20)
        ax5.set_title("L1 Adaptive Signal σ (m/s²)")
    else:
        ax5.plot(tr, v(col(d,"roll_act_deg")),  color="tab:red",  lw=1.2, label="roll")
        ax5.plot(tr, v(col(d,"pitch_act_deg")), color="tab:blue", lw=1.2, label="pitch")
        ax5.set_title("Attitude — PID")
    fe.draw_all_silent(ax5, t_ref=t_ref)
    ax5.set_xlim(t0_abs, t_end)
    ax5.set_xlabel("time relative to first DROP (s)")
    ax5.legend(fontsize=6.5)

    # ── Panel 6 [2,0]: Thrust + ETC events ───────────────────────────────
    ax6 = fig.add_subplot(gs[2, 0])
    tn = v(col(d,"thrust_norm"))
    if tn.max() > 1e-4:
        ax6.plot(tr, tn*100, color="tab:purple", lw=1.5, label="thrust (%)")
        ax6b = ax6.twinx()
        ax6b.plot(tr, v(col(d,"u_filt_z")), color="tab:brown", lw=1.0, ls="--",
                  alpha=0.7, label="u_filt_z (N)")
        ax6b.set_ylabel("u_filt_z (N)", color="tab:brown", fontsize=8)
        ax6b.legend(fontsize=6, loc="upper right")
    else:
        ax6.text(0.5, 0.5, "thrust_norm\nnot available", transform=ax6.transAxes,
                 ha="center", va="center", fontsize=9, color="gray")
    if "etc_triggered" in d.columns:
        etc_m = v(col(d,"etc_triggered")).astype(bool)
        if etc_m.any():
            ax6.scatter(tr[etc_m], np.ones(etc_m.sum())*3, marker="|",
                        color="darkorange", s=50, alpha=0.6, label="ETC trigger", zorder=5)
    fe.draw_all_silent(ax6, t_ref=t_ref)
    ax6.set_xlim(t0_abs, t_end)
    ax6.set_xlabel("time relative to first DROP (s)")
    ax6.set_ylabel("thrust (%)")
    ax6.set_title("Thrust & ETC Events")
    ax6.legend(fontsize=6.5)

    # ── Panel 7 [2,1]: XY position error ─────────────────────────────────
    ax7 = fig.add_subplot(gs[2, 1])
    ax7.plot(tr, v(col(d,"err_x_m"))*1e3, color="tab:blue",  lw=1.3, label="err X")
    ax7.plot(tr, v(col(d,"err_y_m"))*1e3, color="tab:green", lw=1.3, label="err Y")
    ax7.plot(tr, v(d["err_3d_m"])*1e3,    color="gray",      lw=0.9, ls=":", alpha=0.7, label="err 3D")
    ax7.axhline(0, color="k", lw=0.4)
    fe.draw_all_silent(ax7, t_ref=t_ref)
    ax7.set_xlim(t0_abs, t_end)
    ax7.set_xlabel("time relative to first DROP (s)")
    ax7.set_ylabel("error (mm)")
    ax7.set_title("XY Position Error")
    ax7.legend(fontsize=7)

    # ── Panel 8 [2,2]: XY Ground Track ───────────────────────────────────
    ax8 = fig.add_subplot(gs[2, 2])
    px = v(col(d,"pos_x_m")) if "pos_x_m" in d.columns else v(col(d,"err_x_m"))
    py = v(col(d,"pos_y_m")) if "pos_y_m" in d.columns else v(col(d,"err_y_m"))
    t_abs = v(d["t_s"])
    # color the track by time
    points = np.array([px, py]).T.reshape(-1, 1, 2)
    segs   = np.concatenate([points[:-1], points[1:]], axis=1)
    norm   = plt.Normalize(t_abs.min(), t_abs.max())
    lc     = LineCollection(segs, cmap="plasma", norm=norm, lw=1.5, alpha=0.85)
    lc.set_array(t_abs[:-1])
    ax8.add_collection(lc)
    fig.colorbar(lc, ax=ax8, label="time (s)", pad=0.02, shrink=0.85)
    # mark start and event positions
    ax8.scatter(px[0], py[0], marker="s", color="green",  s=60, zorder=5, label="start")
    ax8.scatter(px[-1], py[-1], marker="^", color="red",  s=60, zorder=5, label="end")
    # mark XY positions at event times
    for e in fe.sorted_events():
        t_ev_abs = e["t"]
        idx = int(np.argmin(np.abs(t_abs - t_ev_abs)))
        ax8.scatter(px[idx], py[idx], marker="*", color=e["color"], s=120,
                    zorder=6, label=e["label"])
    # setpoint track if available
    if "sp_x_m" in d.columns and "sp_y_m" in d.columns:
        ax8.plot(v(d["sp_x_m"]), v(d["sp_y_m"]), "k--", lw=0.8, alpha=0.4, label="setpoint XY")
    ax8.set_aspect("equal", adjustable="box")
    ax8.set_xlabel("X (m)"); ax8.set_ylabel("Y (m)")
    ax8.set_title("XY Ground Track (color = time)")
    ax8.legend(fontsize=6, loc="upper right", ncol=1)

    # ── Suptitle & Velocity (footer spanning full width via text) ─────────
    # We insert velocity as extra row below — reconfig layout for 4 rows
    # Instead, annotate suptitle with scenario info
    n_drops = len(fe.drops())
    n_picks = len(fe.picks())
    scenario_str = {
        "single": f"Single Drop | {true_payload:.2f} kg payload",
        "multi_drop": f"Multi-Drop ({n_drops} events) | {true_payload:.2f} kg/drop",
        "pick_drop": f"Pick-Drop ({n_picks} picks, {n_drops} drops)",
    }.get(scenario, f"Payload {true_payload:.2f} kg")

    if l1_inactive:
        fig.text(0.5, 0.995,
                 "⚠ mc_l1_mo TIDAK AKTIF: sigma=0. Cek param L1MO_KP_Z / L1MO_GAMMA_Z!",
                 ha="center", fontsize=8.5, color="red",
                 bbox=dict(facecolor="lightyellow", edgecolor="orange", alpha=0.9))

    fig.suptitle(f"Figure 1 — Overview  |  {ctrl_label}  |  {scenario_str}  |  Gazebo SITL x500",
                 fontsize=12, y=1.01, fontweight="bold")
    _save(fig, os.path.join(outdir, fname))
    return fname


# ══════════════════════════════════════════════════════════════════════════════
#  FIGURE 2 — DETAIL ANALISIS
# ══════════════════════════════════════════════════════════════════════════════
def plot_fig2_single(d, dm, fe, true_payload, ctrl_label, color, outdir, csv_path, scenario="single"):
    """
    Figure 2: 12-panel detail analisis.
    Panels: velocity decomposed, sigma heatmap/saturation, err phase portrait,
            thrust PSD, ETC inter-event histogram, ETC time raster,
            mass convergence zoom, recovery multi-threshold, altitude error PSD,
            yaw attitude, 3D error absolute, summary table.
    """
    drops = fe.drops()
    t_ref  = drops[0]["t"] if drops else dm.get("t_drop_s", float(d["t_s"].median()))
    tr     = v(d["t_s"]) - t_ref
    t0_abs = v(d["t_s"]).min() - t_ref
    t_end  = v(d["t_s"]).max() - t_ref
    m_true_loaded = EMPTY_MASS + true_payload
    ctrl   = detect_controller(d)
    is_l1  = ctrl in ("l1", "l1_inactive")
    fname  = f"fig2_drop_{'l1' if is_l1 else 'pid'}_{stem(csv_path)}.png"
    sr     = sample_rate(d)
    all_ev = dm.get("all_events", [dm])

    fig = plt.figure(figsize=(20, 18))
    gs  = gridspec.GridSpec(4, 3, figure=fig, hspace=0.55, wspace=0.42)

    def draw_ev_s(ax):
        fe.draw_all_silent(ax, t_ref=t_ref, lw=1.5)

    # ── Panel A [0,0]: Velocity decomposed ───────────────────────────────
    axA = fig.add_subplot(gs[0, 0])
    axA.plot(tr, v(col(d,"vel_z_ms")),  color="tab:blue",  lw=1.5, label="vz actual")
    axA.plot(tr, v(col(d,"spv_z_ms")), color="tab:blue",  lw=0.8, ls="--", alpha=0.6, label="vz setpoint")
    axA.plot(tr, v(col(d,"vel_x_ms")), color="tab:red",   lw=1.0, alpha=0.8, label="vx")
    axA.plot(tr, v(col(d,"vel_y_ms")), color="tab:green", lw=1.0, alpha=0.8, label="vy")
    axA.axhline(0, color="k", lw=0.4)
    draw_ev_s(axA)
    axA.set_xlim(t0_abs, t_end)
    axA.set_xlabel("time rel. DROP (s)"); axA.set_ylabel("velocity (m/s)")
    axA.set_title("A — Velocity Decomposed")
    axA.legend(fontsize=6, ncol=2)

    # ── Panel B [0,1]: σ_z saturation timeline (heatmap-style) ──────────
    axB = fig.add_subplot(gs[0, 1])
    if is_l1:
        sz = v(col(d,"sigma_z_ms2"))
        axB.fill_between(tr, 0, sz, where=(sz > 0), color="tab:red",  alpha=0.4, label="σ+ (up dist)")
        axB.fill_between(tr, sz, 0, where=(sz < 0), color="tab:blue", alpha=0.4, label="σ- (down dist)")
        # Saturated regions shaded differently
        axB.fill_between(tr, SIG_LIM,  sz.clip(min=SIG_LIM),  where=(sz > SIG_LIM),
                         color="darkred", alpha=0.7, label="SAT+")
        axB.fill_between(tr, -SIG_LIM, sz.clip(max=-SIG_LIM), where=(sz < -SIG_LIM),
                         color="darkblue", alpha=0.7, label="SAT-")
        axB.axhline( SIG_LIM, color="darkred",  ls=":", lw=1.2)
        axB.axhline(-SIG_LIM, color="darkblue", ls=":", lw=1.2)
        axB.axhline(0, color="k", lw=0.4)
        sat_pct = float((np.abs(sz) >= SIG_LIM).mean()) * 100
        axB.text(0.02, 0.97, f"Sat: {sat_pct:.1f}%", transform=axB.transAxes,
                 fontsize=8, va="top", color="darkred")
    else:
        axB.text(0.5, 0.5, "σ_z not available\n(PID)", transform=axB.transAxes,
                 ha="center", va="center", fontsize=10, color="gray")
    draw_ev_s(axB)
    axB.set_xlim(t0_abs, t_end)
    axB.set_xlabel("time rel. DROP (s)"); axB.set_ylabel("σ_z (m/s²)")
    axB.set_title("B — L1 σ_z Saturation Map")
    axB.legend(fontsize=6)

    # ── Panel C [0,2]: err_z phase portrait (err_z vs d(err_z)/dt) ───────
    axC = fig.add_subplot(gs[0, 2])
    ez  = v(d["err_z_m"]) * 1e3  # mm
    dez = np.gradient(ez, v(d["t_s"]))  # mm/s
    # color by time
    points = np.array([ez, dez]).T.reshape(-1, 1, 2)
    segs   = np.concatenate([points[:-1], points[1:]], axis=1)
    norm_c = plt.Normalize(v(d["t_s"]).min(), v(d["t_s"]).max())
    lc2    = LineCollection(segs, cmap="coolwarm", norm=norm_c, lw=1.0, alpha=0.8)
    lc2.set_array(v(d["t_s"])[:-1])
    axC.add_collection(lc2)
    fig.colorbar(lc2, ax=axC, label="time (s)", pad=0.02, shrink=0.85)
    # mark events
    for e in fe.sorted_events():
        idx_e = int(np.argmin(np.abs(v(d["t_s"]) - e["t"])))
        axC.scatter(ez[idx_e], dez[idx_e], marker="*", color=e["color"], s=150, zorder=6,
                    label=e["label"])
    axC.axvline(0, color="k", lw=0.5, ls=":")
    axC.axhline(0, color="k", lw=0.5, ls=":")
    axC.set_xlim(-max(abs(ez.min()), abs(ez.max()))*1.1, max(abs(ez.min()), abs(ez.max()))*1.1)
    axC.set_xlabel("err_z (mm)"); axC.set_ylabel("d(err_z)/dt (mm/s)")
    axC.set_title("C — Error Phase Portrait")
    axC.legend(fontsize=6, loc="upper right")

    # ── Panel D [1,0]: ETC inter-event interval histogram ────────────────
    axD = fig.add_subplot(gs[1, 0])
    es  = etc_stats(d)
    if es and len(es["iei"]) > 2:
        iei = es["iei"]
        axD.hist(iei * 1e3, bins=min(40, max(5, len(iei)//3)),
                 color="darkorange", alpha=0.75, edgecolor="white", lw=0.5)
        axD.axvline(es["iei_mean"]*1e3, color="red", lw=1.5, ls="--",
                    label=f"mean={es['iei_mean']*1e3:.1f}ms")
        axD.axvline(es["iei_min"]*1e3,  color="green", lw=1.0, ls=":",
                    label=f"min={es['iei_min']*1e3:.1f}ms")
        axD.text(0.97, 0.97, f"N={es['n_triggered']:,}\nrate={es['trigger_rate']*100:.1f}%",
                 transform=axD.transAxes, ha="right", va="top", fontsize=8)
        axD.set_xlabel("inter-event interval (ms)"); axD.set_ylabel("count")
        axD.set_title("D — ETC Inter-Event Interval")
        axD.legend(fontsize=7)
    else:
        axD.text(0.5, 0.5, "ETC not available\nor insufficient triggers",
                 transform=axD.transAxes, ha="center", va="center", fontsize=9, color="gray")
        axD.set_title("D — ETC Inter-Event Interval")

    # ── Panel E [1,1]: ETC trigger raster vs err_z ───────────────────────
    axE  = fig.add_subplot(gs[1, 1])
    axEb = axE.twinx()
    axE.plot(tr, v(d["err_z_m"])*1e3, color=color, lw=1.2, alpha=0.7, label="err_z (mm)")
    axE.axhline(0, color="k", lw=0.4)
    if es and es["n_triggered"] > 0:
        t_trig_rel = es["t_triggered"] - t_ref
        axEb.scatter(t_trig_rel, np.zeros(len(t_trig_rel)),
                     marker="|", color="darkorange", s=60, alpha=0.5, label="ETC trigger")
        axEb.set_yticks([]); axEb.set_ylabel("triggers", color="darkorange", fontsize=8)
    draw_ev_s(axE)
    axE.set_xlim(t0_abs, t_end)
    axE.set_xlabel("time rel. DROP (s)"); axE.set_ylabel("err_z (mm)")
    axE.set_title("E — ETC Triggers vs Altitude Error")
    axE.legend(fontsize=7)

    # ── Panel F [1,2]: Altitude error PSD ────────────────────────────────
    axF = fig.add_subplot(gs[1, 2])
    ez_pre  = v(d[(d["t_s"] >= t_ref - 8) & (d["t_s"] < t_ref - 0.5)]["err_z_m"]) * 1e3
    ez_post = v(d[(d["t_s"] >= t_ref + 2) & (d["t_s"] <= t_ref + 12)]["err_z_m"]) * 1e3
    if len(ez_pre) > 16:
        from numpy.fft import rfft, rfftfreq
        def psd(sig):
            n = len(sig)
            sig_w = sig * np.hanning(n)
            f = rfftfreq(n, d=1.0/sr)
            p = (2.0 / (sr * n)) * np.abs(rfft(sig_w))**2
            return f, p
        fp, pp = psd(ez_pre)
        axF.semilogy(fp, pp, color="tab:blue", lw=1.2, alpha=0.8, label="pre-drop")
    if len(ez_post) > 16:
        from numpy.fft import rfft, rfftfreq
        def psd(sig, sr2=sr):
            n = len(sig)
            sig_w = sig * np.hanning(n)
            f = rfftfreq(n, d=1.0/sr2)
            p = (2.0 / (sr2 * n)) * np.abs(rfft(sig_w))**2
            return f, p
        fa, pa = psd(ez_post)
        axF.semilogy(fa, pa, color="tab:red", lw=1.2, alpha=0.8, label="post-drop (2–12s)")
    axF.set_xlim(0, min(sr/2, 10))
    axF.set_xlabel("frequency (Hz)"); axF.set_ylabel("PSD (mm²/Hz)")
    axF.set_title("F — Altitude Error PSD")
    axF.legend(fontsize=7)
    axF.grid(True, which="both", alpha=0.3)

    # ── Panel G [2,0]: Mass convergence zoom (post-drop) ─────────────────
    axG = fig.add_subplot(gs[2, 0])
    if is_l1:
        t_win_end = t_ref + 15
        d_zoom = d[(d["t_s"] >= t_ref - 3) & (d["t_s"] <= t_win_end)]
        tr_z = v(d_zoom["t_s"]) - t_ref
        axG.plot(tr_z, v(col(d_zoom,"mo_m_est_kg")), color="tab:blue", lw=1.6, label="MO estimate")
        axG.axhline(m_true_loaded, color="red",   ls="--", lw=1.2, label=f"loaded {m_true_loaded:.3f}kg")
        axG.axhline(EMPTY_MASS,   color="black", ls=":",  lw=1.2, label=f"empty {EMPTY_MASS:.3f}kg")
        # 5% bands
        axG.axhline(EMPTY_MASS * 1.05, color="gray", ls=":", lw=0.7, alpha=0.5)
        axG.axhline(EMPTY_MASS * 0.95, color="gray", ls=":", lw=0.7, alpha=0.5)
        axG.set_ylim(EMPTY_MASS - 0.3, m_true_loaded + 0.4)
    else:
        axG.text(0.5, 0.5, "No mass estimator\n(PID)", transform=axG.transAxes,
                 ha="center", va="center", fontsize=10, color="gray")
    draw_ev_s(axG)
    axG.set_xlim(-3, 15)
    axG.set_xlabel("time rel. DROP (s)"); axG.set_ylabel("mass (kg)")
    axG.set_title("G — Mass Convergence (zoom)")
    axG.legend(fontsize=6, ncol=2)

    # ── Panel H [2,1]: Recovery by threshold bars ─────────────────────────
    axH = fig.add_subplot(gs[2, 1])
    ths    = [200, 100, 50, 30, 15]
    colors_th = ["#d62728", "#ff7f0e", "#2ca02c", "#1f77b4", "#9467bd"]
    n_ev = len(all_ev)
    bar_w = 0.7 / max(n_ev, 1)
    x_th  = np.arange(len(ths), dtype=float)
    for ei_idx, ev_dm in enumerate(all_ev):
        t_ev = ev_dm.get("t_drop_s", t_ref)
        rl = []
        for th in ths:
            seg = d[(d["t_s"] > t_ev) & (d["err_z_m"].abs() < th/1e3)]
            rl.append(float(seg["t_s"].iloc[0] - t_ev) if len(seg) else float("nan"))
        lbl_bar = ev_dm.get("event_label", f"DROP#{ei_idx+1}")
        offs = (ei_idx - n_ev/2 + 0.5) * bar_w
        bars = axH.barh(x_th + offs, rl, height=bar_w*0.85,
                        color=L1_COLORS[ei_idx % len(L1_COLORS)], label=lbl_bar)
        for b, rv in zip(bars, rl):
            if math.isfinite(rv) and rv < 20:
                axH.text(rv+0.05, b.get_y()+b.get_height()/2,
                         f"{rv:.2f}s", va="center", fontsize=6, color="k")
    axH.set_yticks(x_th)
    axH.set_yticklabels([f"±{t}mm" for t in ths])
    axH.set_xlabel("recovery time (s)")
    axH.set_title("H — Recovery by Threshold (per event)")
    if n_ev > 1: axH.legend(fontsize=6)

    # ── Panel I [2,2]: Yaw + heading ─────────────────────────────────────
    axI = fig.add_subplot(gs[2, 2])
    if "yaw_act_deg" in d.columns:
        axI.plot(tr, v(col(d,"yaw_act_deg")),  color="tab:purple", lw=1.4, label="yaw act")
    if "yaw_cmd_deg" in d.columns:
        axI.plot(tr, v(col(d,"yaw_cmd_deg")), color="tab:purple", lw=0.8, ls="--", alpha=0.6, label="yaw cmd")
    axI.axhline(0, color="k", lw=0.4)
    draw_ev_s(axI)
    axI.set_xlim(t0_abs, t_end)
    axI.set_xlabel("time rel. DROP (s)"); axI.set_ylabel("yaw (deg)")
    axI.set_title("I — Yaw Attitude")
    axI.legend(fontsize=6.5)

    # ── Panel J [3,0]: 3D error absolute ─────────────────────────────────
    axJ = fig.add_subplot(gs[3, 0])
    axJ.plot(tr, v(d["err_3d_m"])*1e3, color=color, lw=1.4, label="err 3D")
    axJ.plot(tr, np.abs(v(col(d,"err_z_m")))*1e3,  color="tab:blue",  lw=1.0, ls="--", alpha=0.7, label="|err Z|")
    axJ.plot(tr, np.sqrt(v(col(d,"err_x_m"))**2 + v(col(d,"err_y_m"))**2)*1e3,
             color="tab:green", lw=1.0, ls=":", alpha=0.7, label="err XY")
    axJ.axhline(RECOVERY_MM, color="gray", ls=":", lw=1, label=f"±{RECOVERY_MM:.0f}mm")
    draw_ev_s(axJ)
    axJ.set_xlim(t0_abs, t_end)
    axJ.set_xlabel("time rel. DROP (s)"); axJ.set_ylabel("error (mm)")
    axJ.set_title("J — Absolute 3D / Z / XY Error")
    axJ.legend(fontsize=6.5, ncol=2)

    # ── Panel K [3,1]: σ_x σ_y detail (if L1) ───────────────────────────
    axK = fig.add_subplot(gs[3, 1])
    if is_l1:
        axK.plot(tr, v(col(d,"sigma_x_ms2")), color="tab:red",   lw=1.3, label="σ_x")
        axK.plot(tr, v(col(d,"sigma_y_ms2")), color="tab:blue",  lw=1.3, label="σ_y")
        axK.axhline(0, color="k", lw=0.4)
        axK.axhline( SIG_LIM, color="darkred", ls=":", lw=0.9, alpha=0.6)
        axK.axhline(-SIG_LIM, color="darkred", ls=":", lw=0.9, alpha=0.6, label=f"±SIG_LIM")
        draw_ev_s(axK)
        axK.set_title("K — L1 σ_x & σ_y (Horizontal Disturbance)")
    else:
        # prediction error for PID placeholder
        axK.text(0.5, 0.5, "σ_x / σ_y not available\n(PID controller)",
                 transform=axK.transAxes, ha="center", va="center", fontsize=9, color="gray")
        axK.set_title("K — Horizontal Disturbance (N/A)")
    axK.set_xlim(t0_abs, t_end)
    axK.set_xlabel("time rel. DROP (s)"); axK.set_ylabel("σ (m/s²)")
    axK.legend(fontsize=6.5)

    # ── Panel L [3,2]: Summary table ─────────────────────────────────────
    axL = fig.add_subplot(gs[3, 2])
    axL.axis("off")
    hdr = ["Event", "Peak\n(mm)", f"Rec ±{RECOVERY_MM:.0f}mm\n(s)",
           "Post RMSE\nZ (mm)", "Δm (kg)", "Δm err"]
    rows_tbl = [hdr]
    for ev_dm in all_ev:
        dd = ev_dm.get("detected_delta_kg", float("nan"))
        de = ev_dm.get("delta_err_pct",     float("nan"))
        rows_tbl.append([
            ev_dm.get("event_label", "DROP"),
            _f(ev_dm.get("peak_alt_err_mm", float("nan")), ".0f"),
            _f(ev_dm.get("recovery_s",      float("nan")), ".3f"),
            _f(ev_dm.get("post_rmse_z_mm",  float("nan")), ".1f"),
            _f(dd, ".3f"),
            f"{_f(de, '.1f')}%" if math.isfinite(de) else "—",
        ])
    # ETC summary row
    if es:
        rows_tbl.append(["— ETC —", f"N={es['n_triggered']:,}",
                          f"rate={es['trigger_rate']*100:.1f}%",
                          f"IEI={es['iei_mean']*1e3:.0f}ms" if math.isfinite(es["iei_mean"]) else "—",
                          "—", "—"])
    c0 = "#d0e8f5"
    tbl_c = [[c0]*len(hdr)] + [["white"]*len(hdr)]*(len(rows_tbl)-1)
    t = axL.table(cellText=rows_tbl, cellColours=tbl_c, cellLoc="center", loc="center")
    t.auto_set_font_size(False); t.set_fontsize(7.5); t.scale(1, 1.65)
    axL.set_title("L — Summary Table", pad=8, fontsize=9)

    n_drops = len(fe.drops()); n_picks = len(fe.picks())
    scenario_str = {
        "single": f"Single Drop | {true_payload:.2f}kg payload",
        "multi_drop": f"Multi-Drop ({n_drops} events) | {true_payload:.2f}kg/drop",
        "pick_drop": f"Pick-Drop ({n_picks} picks, {n_drops} drops)",
    }.get(scenario, f"Payload {true_payload:.2f}kg")

    fig.suptitle(f"Figure 2 — Detail Analysis  |  {ctrl_label}  |  {scenario_str}  |  Gazebo SITL x500",
                 fontsize=12, y=1.01, fontweight="bold")
    _save(fig, os.path.join(outdir, fname))
    return fname


# ══════════════════════════════════════════════════════════════════════════════
#  FIGURE 1 + 2 COMPARE — overlay multi-L1 + optional PID
# ══════════════════════════════════════════════════════════════════════════════
def plot_fig1_compare(l1_runs, pid_run, true_payload, outdir):
    """Figure 1 compare: altitude overlay + error + mass + attitude + sigma + XY + recovery."""
    parts = [stem(r[4]) for r in l1_runs]
    if pid_run: parts.append(stem(pid_run[3]))
    tag = "_".join(parts[:3]) + ("_etc" if len(parts) > 3 else "")
    fname = f"fig1_compare_{tag}.png"

    m_true_loaded = EMPTY_MASS + true_payload

    def get_tref(run):
        return run[1].get("t_drop_s", float(v(run[0]["t_s"]).min()))

    all_t0 = [v(r[0]["t_s"]).min()-get_tref(r) for r in l1_runs]
    all_t1 = [v(r[0]["t_s"]).max()-get_tref(r) for r in l1_runs]
    if pid_run:
        pr_tref = get_tref((pid_run[0], pid_run[1], pid_run[2], pid_run[2], pid_run[3]))
        all_t0.append(v(pid_run[0]["t_s"]).min() - pr_tref)
        all_t1.append(v(pid_run[0]["t_s"]).max() - pr_tref)
    x0 = min(all_t0); x1 = max(all_t1)

    fig = plt.figure(figsize=(18, 14))
    gs  = gridspec.GridSpec(3, 3, figure=fig, hspace=0.52, wspace=0.40)

    # P1 [0,:2]: altitude (di-flip dari NED z-down jadi altitude positif-naik)
    ax1 = fig.add_subplot(gs[0, :2])
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        t_ref_l = get_tref((dl, dm_l, lbl, clr, cpath))
        tr = v(dl["t_s"]) - t_ref_l
        ax1.plot(tr, -v(dl["pos_z_m"]), color=clr, lw=1.8, label=f"{lbl} actual")
        ax1.plot(tr, -v(dl["sp_z_m"]),  color=clr, lw=0.7, ls=":", alpha=0.4)
        fe_l.draw_all_silent(ax1, t_ref=t_ref_l, lw=1.3)
    if pid_run:
        dp, dm_p, lbl_p, _, cpath_p, fe_p = pid_run
        t_ref_p = get_tref((dp, dm_p, lbl_p, PID_COLOR, cpath_p))
        tr_p = v(dp["t_s"]) - t_ref_p
        ax1.plot(tr_p, -v(dp["pos_z_m"]), color=PID_COLOR, lw=1.8, ls="--", label=f"{lbl_p}")
        fe_p.draw_all_silent(ax1, t_ref=t_ref_p)
    ax1.set_xlim(x0, x1); ax1.set_xlabel("time rel. DROP (s)"); ax1.set_ylabel("Altitude (m)")
    ax1.set_title("Altitude Trajectory (all runs)"); ax1.legend(fontsize=7, ncol=2)

    # P2 [0,2]: altitude error
    ax2 = fig.add_subplot(gs[0, 2])
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        t_ref_l = get_tref((dl, dm_l, lbl, clr, cpath))
        tr = v(dl["t_s"]) - t_ref_l
        pk  = dm_l.get("peak_alt_err_mm", 0) or 0
        rc  = dm_l.get("recovery_s", float("nan"))
        ax2.plot(tr, v(dl["err_z_m"])*1e3, color=clr, lw=1.5,
                 label=f"{lbl} pk={_f(pk,'.0f')}mm rc={_f(rc,'.2f')}s")
    if pid_run:
        dp, dm_p, lbl_p, _, cpath_p, fe_p = pid_run
        t_ref_p = get_tref((dp, dm_p, lbl_p, PID_COLOR, cpath_p))
        tr_p = v(dp["t_s"]) - t_ref_p
        pk_p = dm_p.get("peak_alt_err_mm", 0) or 0
        rc_p = dm_p.get("recovery_s", float("nan"))
        ax2.plot(tr_p, v(dp["err_z_m"])*1e3, color=PID_COLOR, lw=1.5, ls="--",
                 label=f"{lbl_p} pk={_f(pk_p,'.0f')}mm rc={_f(rc_p,'.2f')}s")
    ax2.axhline( RECOVERY_MM, color="green", ls=":", lw=1)
    ax2.axhline(-RECOVERY_MM, color="green", ls=":", lw=1, label=f"±{RECOVERY_MM:.0f}mm")
    ax2.set_xlim(x0, x1); ax2.set_xlabel("time rel. DROP (s)")
    ax2.set_ylabel("altitude error (mm)"); ax2.set_title("Altitude Error Overlay")
    ax2.legend(fontsize=6)

    # P3 [1,:2]: mass estimation
    ax3 = fig.add_subplot(gs[1, :2])
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        t_ref_l = get_tref((dl, dm_l, lbl, clr, cpath))
        tr = v(dl["t_s"]) - t_ref_l
        ax3.plot(tr, v(col(dl,"mo_m_est_kg")), color=clr, lw=1.8,
                 label=f"{lbl} Δ={_f(dm_l.get('detected_delta_kg',float('nan')),'.3f')}kg")
        fe_l.draw_all_silent(ax3, t_ref=t_ref_l, lw=1.2)
    ax3.axhline(m_true_loaded, color="red",   ls="--", lw=1.2, label=f"true loaded")
    ax3.axhline(EMPTY_MASS,   color="black", ls=":",  lw=1.2, label=f"true empty")
    ax3.set_xlim(x0, x1); ax3.set_ylim(1.2, m_true_loaded + 0.5)
    ax3.set_xlabel("time rel. DROP (s)"); ax3.set_ylabel("mass (kg)")
    ax3.legend(fontsize=6, ncol=2)

    # P4 [1,2]: sigma_z
    ax4 = fig.add_subplot(gs[1, 2])
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        t_ref_l = get_tref((dl, dm_l, lbl, clr, cpath))
        tr = v(dl["t_s"]) - t_ref_l
        ax4.plot(tr, v(col(dl,"sigma_z_ms2")), color=clr, lw=1.4, label=lbl)
    ax4.axhline( SIG_LIM, color="darkred", ls=":", lw=1.2, label=f"±SIG_LIM={SIG_LIM}")
    ax4.axhline(-SIG_LIM, color="darkred", ls=":", lw=1.2)
    ax4.axhline(0, color="k", lw=0.4)
    ax4.set_xlim(x0, x1); ax4.set_xlabel("time rel. DROP (s)")
    ax4.set_ylabel("σ_z (m/s²)"); ax4.set_title("L1 Disturbance σ_z")
    ax4.legend(fontsize=7)

    # P5 [2,0]: recovery bars
    ax5 = fig.add_subplot(gs[2, 0])
    ths   = [200, 100, 50, 30]
    n_all = len(l1_runs) + (1 if pid_run else 0)
    bar_h = 0.75 / n_all
    x_th  = np.arange(len(ths), dtype=float)
    for idx, (dl, dm_l, lbl, clr, cpath, fe_l) in enumerate(l1_runs):
        t_drop_l = dm_l.get("t_drop_s", float(v(dl["t_s"]).min()))
        rl = []
        for th in ths:
            rec = dl[(dl["t_s"] > t_drop_l) & (dl["err_z_m"].abs() < th/1e3)]
            rl.append(float(rec["t_s"].iloc[0]-t_drop_l) if len(rec) else float("nan"))
        offs = (idx - n_all/2 + 0.5) * bar_h
        bars = ax5.barh(x_th + offs, rl, height=bar_h*0.85, color=clr, label=lbl)
        for b, rv in zip(bars, rl):
            if math.isfinite(rv) and rv < 12:
                ax5.text(rv+0.05, b.get_y()+b.get_height()/2,
                         f"{rv:.2f}s", va="center", fontsize=6, color=clr)
    if pid_run:
        dp, dm_p, lbl_p, _, cpath_p, fe_p = pid_run
        t_drop_p = dm_p.get("t_drop_s", float(v(dp["t_s"]).min()))
        rp = []
        for th in ths:
            rec = dp[(dp["t_s"] > t_drop_p) & (dp["err_z_m"].abs() < th/1e3)]
            rp.append(float(rec["t_s"].iloc[0]-t_drop_p) if len(rec) else float("nan"))
        offs = (len(l1_runs) - n_all/2 + 0.5) * bar_h
        bars = ax5.barh(x_th + offs, rp, height=bar_h*0.85, color=PID_COLOR, hatch="///", label=lbl_p)
        for b, rv in zip(bars, rp):
            if math.isfinite(rv) and rv < 15:
                ax5.text(rv+0.05, b.get_y()+b.get_height()/2,
                         f"{rv:.2f}s", va="center", fontsize=6, color=PID_COLOR)
    ax5.set_yticks(x_th); ax5.set_yticklabels([f"±{t}mm" for t in ths])
    ax5.set_xlabel("recovery time (s)"); ax5.set_title("Recovery by Threshold")
    ax5.legend(fontsize=7)

    # P6 [2,1]: roll attitude compare
    ax6 = fig.add_subplot(gs[2, 1])
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        t_ref_l = get_tref((dl, dm_l, lbl, clr, cpath))
        tr = v(dl["t_s"]) - t_ref_l
        ax6.plot(tr, v(col(dl,"roll_act_deg")), color=clr, lw=1.0, label=f"{lbl} roll")
        ax6.plot(tr, v(col(dl,"pitch_act_deg")), color=clr, lw=0.7, ls=":", alpha=0.6)
    if pid_run:
        dp, dm_p, lbl_p, _, cpath_p, fe_p = pid_run
        t_ref_p = get_tref((dp, dm_p, lbl_p, PID_COLOR, cpath_p))
        tr_p = v(dp["t_s"]) - t_ref_p
        ax6.plot(tr_p, v(col(dp,"roll_act_deg")), color=PID_COLOR, lw=1.0, ls="--", label=f"{lbl_p} roll")
    ax6.axhline(0, color="k", lw=0.4)
    ax6.set_xlim(x0, x1); ax6.set_xlabel("time rel. DROP (s)")
    ax6.set_ylabel("angle (deg)"); ax6.set_title("Roll/Pitch Attitude Compare")
    ax6.legend(fontsize=7)

    # P7 [2,2]: summary table
    ax7 = fig.add_subplot(gs[2, 2])
    ax7.axis("off")
    hdr = ["Run", "PreRMSE Z\n(mm)", "Peak\n(mm)", f"Rec ±{RECOVERY_MM:.0f}mm\n(s)",
           "ITAE\n(transien)", "PostRMSE\nZ (mm)", "Δm (kg)", "Δm err"]
    rows_tbl = [hdr]
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        dd = dm_l.get("detected_delta_kg", float("nan"))
        de = dm_l.get("delta_err_pct",     float("nan"))
        rows_tbl.append([lbl,
            _f(dm_l.get("pre_rmse_z_mm",  float("nan"))),
            _f(dm_l.get("peak_alt_err_mm",float("nan")), ".0f"),
            _f(dm_l.get("recovery_s",     float("nan")), ".2f"),
            _f(dm_l.get("trans_itae_z_mm_s2", float("nan")), ".0f"),
            _f(dm_l.get("post_rmse_z_mm", float("nan"))),
            _f(dd, ".3f"),
            f"{_f(de,'.1f')}%" if math.isfinite(de) else "—",
        ])
    if pid_run:
        dp, dm_p, lbl_p, _, _, _ = pid_run
        dd_p = dm_p.get("detected_delta_kg", float("nan"))
        de_p = dm_p.get("delta_err_pct",     float("nan"))
        rows_tbl.append(["PID",
            _f(dm_p.get("pre_rmse_z_mm", float("nan"))),
            _f(dm_p.get("peak_alt_err_mm",float("nan")), ".0f"),
            _f(dm_p.get("recovery_s",    float("nan")), ".2f"),
            _f(dm_p.get("trans_itae_z_mm_s2", float("nan")), ".0f"),
            _f(dm_p.get("post_rmse_z_mm",float("nan"))),
            "—", "—"])
    c0 = "#d0e8f5"
    tbl_c = [[c0]*len(hdr)] + [["white"]*len(hdr)]*(len(rows_tbl)-1)
    t = ax7.table(cellText=rows_tbl, cellColours=tbl_c, cellLoc="center", loc="center")
    t.auto_set_font_size(False); t.set_fontsize(7); t.scale(1, 1.8)
    ax7.set_title(f"Summary — {true_payload:.1f}kg drop", fontsize=9, pad=8)

    fig.suptitle(f"Figure 1 — Comparison Overview  |  {true_payload:.1f}kg Payload  |  Gazebo SITL x500",
                 fontsize=13, y=1.01, fontweight="bold")
    _save(fig, os.path.join(outdir, fname))


def plot_fig2_compare(l1_runs, pid_run, true_payload, outdir):
    """Figure 2 compare: PSD overlay, phase portraits, ETC compare, mass convergence."""
    parts = [stem(r[4]) for r in l1_runs]
    if pid_run: parts.append(stem(pid_run[3]))
    tag   = "_".join(parts[:3]) + ("_etc" if len(parts) > 3 else "")
    fname = f"fig2_compare_{tag}.png"

    m_true_loaded = EMPTY_MASS + true_payload

    def get_tref(r):
        return r[1].get("t_drop_s", float(v(r[0]["t_s"]).min()))

    all_t0 = [v(r[0]["t_s"]).min()-get_tref(r) for r in l1_runs]
    all_t1 = [v(r[0]["t_s"]).max()-get_tref(r) for r in l1_runs]
    if pid_run:
        all_t0.append(v(pid_run[0]["t_s"]).min()-get_tref((pid_run[0], pid_run[1], None, None, pid_run[3], None)))
        all_t1.append(v(pid_run[0]["t_s"]).max()-get_tref((pid_run[0], pid_run[1], None, None, pid_run[3], None)))
    x0 = min(all_t0); x1 = max(all_t1)

    fig = plt.figure(figsize=(18, 14))
    gs  = gridspec.GridSpec(3, 3, figure=fig, hspace=0.52, wspace=0.42)

    # PA: Phase portrait overlay
    axA = fig.add_subplot(gs[0, 0])
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        ez  = v(dl["err_z_m"]) * 1e3
        dez = np.gradient(ez, v(dl["t_s"]))
        axA.plot(ez, dez, color=clr, lw=0.9, alpha=0.75, label=lbl)
    if pid_run:
        dp, dm_p, lbl_p, _, cpath_p, fe_p = pid_run
        ez_p  = v(dp["err_z_m"]) * 1e3
        dez_p = np.gradient(ez_p, v(dp["t_s"]))
        axA.plot(ez_p, dez_p, color=PID_COLOR, lw=0.9, ls="--", alpha=0.75, label="PID")
    axA.axvline(0, color="k", lw=0.5, ls=":"); axA.axhline(0, color="k", lw=0.5, ls=":")
    axA.set_xlabel("err_z (mm)"); axA.set_ylabel("d(err_z)/dt (mm/s)")
    axA.set_title("A — Phase Portrait Overlay"); axA.legend(fontsize=7)

    # PB: Altitude error PSD compare (L1 vs PID)
    axB = fig.add_subplot(gs[0, 1])
    from numpy.fft import rfft, rfftfreq
    def psd_run(d2, t_drop, sr):
        ez = v(d2[(d2["t_s"] >= t_drop - 8) & (d2["t_s"] < t_drop - 0.5)]["err_z_m"]) * 1e3
        if len(ez) < 16: return None, None
        n = len(ez)
        f = rfftfreq(n, d=1.0/sr)
        p = (2.0/(sr*n)) * np.abs(rfft(ez * np.hanning(n)))**2
        return f, p
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        sr = sample_rate(dl)
        f, p = psd_run(dl, dm_l.get("t_drop_s", 0), sr)
        if f is not None:
            axB.semilogy(f, p, color=clr, lw=1.3, alpha=0.8, label=f"{lbl} pre-drop")
    if pid_run:
        dp, dm_p, lbl_p, _, cpath_p, fe_p = pid_run
        sr_p = sample_rate(dp)
        f_p, p_p = psd_run(dp, dm_p.get("t_drop_s", 0), sr_p)
        if f_p is not None:
            axB.semilogy(f_p, p_p, color=PID_COLOR, lw=1.3, ls="--", alpha=0.8, label="PID pre-drop")
    axB.set_xlim(0, 5); axB.set_xlabel("freq (Hz)"); axB.set_ylabel("PSD (mm²/Hz)")
    axB.set_title("B — Altitude Error PSD (pre-drop)"); axB.legend(fontsize=7)
    axB.grid(True, which="both", alpha=0.3)

    # PC: sigma_z std per run bar chart
    axC = fig.add_subplot(gs[0, 2])
    names_c, sig_pre, sig_post = [], [], []
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        t_drop_l = dm_l.get("t_drop_s", 0)
        pre_s  = v(col(dl[(dl["t_s"] < t_drop_l)], "sigma_z_ms2"))
        post_s = v(col(dl[(dl["t_s"] > t_drop_l)], "sigma_z_ms2"))
        names_c.append(lbl)
        sig_pre.append(float(np.std(pre_s)) if len(pre_s) > 5 else float("nan"))
        sig_post.append(float(np.std(post_s)) if len(post_s) > 5 else float("nan"))
    x_c = np.arange(len(names_c))
    w_c = 0.35
    axC.bar(x_c - w_c/2, sig_pre,  width=w_c, color="tab:blue",  alpha=0.8, label="σ_z std pre-drop")
    axC.bar(x_c + w_c/2, sig_post, width=w_c, color="tab:orange",alpha=0.8, label="σ_z std post-drop")
    axC.set_xticks(x_c); axC.set_xticklabels(names_c, rotation=20, ha="right")
    axC.set_ylabel("σ_z std (m/s²)"); axC.set_title("C — L1 Adaptation Activity (σ_z std)")
    axC.legend(fontsize=7)

    # PD: ETC trigger rate compare
    axD = fig.add_subplot(gs[1, 0])
    names_d, rates = [], []
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        es = etc_stats(dl)
        names_d.append(lbl)
        rates.append(es["trigger_rate"]*100 if es else float("nan"))
    axD.bar(names_d, rates, color=L1_COLORS[:len(names_d)], alpha=0.8)
    if not all(math.isnan(r) for r in rates):
        axD.set_ylabel("trigger rate (%)"); axD.set_title("D — ETC Trigger Rate Compare")
        for i, r in enumerate(rates):
            if math.isfinite(r):
                axD.text(i, r+0.5, f"{r:.1f}%", ha="center", fontsize=8)
    else:
        axD.text(0.5, 0.5, "ETC not logged", transform=axD.transAxes,
                 ha="center", va="center", fontsize=10, color="gray")
        axD.set_title("D — ETC Trigger Rate (N/A)")

    # PE: Velocity vz compare
    axE = fig.add_subplot(gs[1, 1])
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        t_ref_l = get_tref((dl, dm_l, lbl, clr, cpath))
        tr = v(dl["t_s"]) - t_ref_l
        axE.plot(tr, v(col(dl,"vel_z_ms")), color=clr, lw=1.3, label=f"{lbl} vz")
    if pid_run:
        dp, dm_p, lbl_p, _, cpath_p, fe_p = pid_run
        t_ref_p = get_tref((dp, dm_p, lbl_p, PID_COLOR, cpath_p))
        tr_p = v(dp["t_s"]) - t_ref_p
        axE.plot(tr_p, v(col(dp,"vel_z_ms")), color=PID_COLOR, lw=1.3, ls="--", label="PID vz")
    axE.axhline(0, color="k", lw=0.4)
    axE.set_xlim(x0, x1); axE.set_xlabel("time rel. DROP (s)"); axE.set_ylabel("vz (m/s)")
    axE.set_title("E — Vertical Velocity Compare"); axE.legend(fontsize=7)

    # PF: Mass estimation std compare (box-like)
    axF = fig.add_subplot(gs[1, 2])
    names_f, m_vals, m_errs = [], [], []
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        t_drop_l = dm_l.get("t_drop_s", 0)
        post_mo = v(col(dl[dl["t_s"] > t_drop_l+3], "mo_m_est_kg"))
        names_f.append(lbl)
        m_vals.append(float(np.median(post_mo)) if len(post_mo) else float("nan"))
        m_errs.append(float(np.std(post_mo))    if len(post_mo) else float("nan"))
    x_f = np.arange(len(names_f))
    axF.errorbar(x_f, m_vals, yerr=m_errs, fmt="o", capsize=5,
                 color="tab:blue", ms=7, lw=1.5, label="MO estimate post-drop")
    axF.axhline(EMPTY_MASS, color="black", ls=":", lw=1.2, label=f"empty {EMPTY_MASS}kg")
    axF.axhline(EMPTY_MASS + true_payload, color="red", ls="--", lw=1.2, label=f"loaded")
    axF.set_xticks(x_f); axF.set_xticklabels(names_f, rotation=20, ha="right")
    axF.set_ylabel("mass estimate (kg)"); axF.set_title("F — Post-Drop Mass Estimate ± 1σ")
    axF.legend(fontsize=7)

    # PG: recovery time distribution (multi-bar all thresholds)
    axG = fig.add_subplot(gs[2, 0])
    ths_g = [50, 30]; n_run_g = len(l1_runs) + (1 if pid_run else 0)
    x_g = np.arange(len(ths_g)); bw = 0.7/n_run_g
    for idx, (dl, dm_l, lbl, clr, cpath, fe_l) in enumerate(l1_runs):
        t_drop_l = dm_l.get("t_drop_s", 0)
        rl = []
        for th in ths_g:
            rec = dl[(dl["t_s"] > t_drop_l) & (dl["err_z_m"].abs() < th/1e3)]
            rl.append(float(rec["t_s"].iloc[0]-t_drop_l) if len(rec) else float("nan"))
        offs = (idx - n_run_g/2 + 0.5)*bw
        axG.bar(x_g+offs, rl, width=bw*0.85, color=clr, label=lbl)
        for xi, rv in zip(x_g, rl):
            if math.isfinite(rv):
                axG.text(xi+offs, rv+0.05, f"{rv:.2f}s", ha="center", fontsize=6.5)
    if pid_run:
        dp, dm_p, lbl_p, _, cpath_p, fe_p = pid_run
        t_drop_p = dm_p.get("t_drop_s", 0)
        rp = []
        for th in ths_g:
            rec = dp[(dp["t_s"] > t_drop_p) & (dp["err_z_m"].abs() < th/1e3)]
            rp.append(float(rec["t_s"].iloc[0]-t_drop_p) if len(rec) else float("nan"))
        offs = (len(l1_runs) - n_run_g/2 + 0.5)*bw
        axG.bar(x_g+offs, rp, width=bw*0.85, color=PID_COLOR, hatch="///", label="PID")
        for xi, rv in zip(x_g, rp):
            if math.isfinite(rv):
                axG.text(xi+offs, rv+0.05, f"{rv:.2f}s", ha="center", fontsize=6.5)
    axG.set_xticks(x_g); axG.set_xticklabels([f"±{t}mm" for t in ths_g])
    axG.set_ylabel("recovery time (s)"); axG.set_title("G — Recovery Time Compare")
    axG.legend(fontsize=7)

    # PH: Thrust norm compare
    axH = fig.add_subplot(gs[2, 1])
    for dl, dm_l, lbl, clr, cpath, fe_l in l1_runs:
        t_ref_l = get_tref((dl, dm_l, lbl, clr, cpath))
        tr = v(dl["t_s"]) - t_ref_l
        tn = v(col(dl,"thrust_norm"))
        if tn.max() > 1e-4:
            axH.plot(tr, tn*100, color=clr, lw=1.3, alpha=0.8, label=f"{lbl}")
    if pid_run:
        dp, dm_p, lbl_p, _, cpath_p, fe_p = pid_run
        t_ref_p = get_tref((dp, dm_p, lbl_p, PID_COLOR, cpath_p))
        tr_p = v(dp["t_s"]) - t_ref_p
        tn_p = v(col(dp,"thrust_norm"))
        if tn_p.max() > 1e-4:
            axH.plot(tr_p, tn_p*100, color=PID_COLOR, lw=1.3, ls="--", alpha=0.8, label="PID")
    axH.axhline(0, color="k", lw=0.4)
    axH.set_xlim(x0, x1); axH.set_xlabel("time rel. DROP (s)"); axH.set_ylabel("thrust (%)")
    axH.set_title("H — Thrust Norm Compare"); axH.legend(fontsize=7)

    # PI: Sigma activity box per segment (pre / drop / post)
    axI = fig.add_subplot(gs[2, 2])
    seg_labels = ["pre-drop", "0–3s post", "3–10s post"]
    positions_base = np.array([1, 2, 3], dtype=float)
    n_l1_c = len(l1_runs)
    bw_i = 0.6 / n_l1_c
    for idx, (dl, dm_l, lbl, clr, cpath, fe_l) in enumerate(l1_runs):
        t_drop_l = dm_l.get("t_drop_s", 0)
        segs = [
            v(col(dl[(dl["t_s"] < t_drop_l) & (dl["t_s"] > t_drop_l-8)], "sigma_z_ms2")),
            v(col(dl[(dl["t_s"] >= t_drop_l) & (dl["t_s"] < t_drop_l+3)], "sigma_z_ms2")),
            v(col(dl[(dl["t_s"] >= t_drop_l+3) & (dl["t_s"] <= t_drop_l+10)], "sigma_z_ms2")),
        ]
        data_plot = [np.abs(s) for s in segs if len(s) > 3]
        positions = positions_base[:len(data_plot)] + (idx - n_l1_c/2 + 0.5) * bw_i
        bp = axI.boxplot(data_plot, positions=positions, widths=bw_i*0.8,
                         patch_artist=True, notch=False,
                         boxprops=dict(facecolor=clr, alpha=0.5),
                         medianprops=dict(color="black", lw=1.5),
                         whiskerprops=dict(color=clr, alpha=0.7),
                         capprops=dict(color=clr, alpha=0.7),
                         flierprops=dict(marker=".", ms=2, color=clr, alpha=0.4))
        axI.plot([], [], color=clr, lw=5, alpha=0.5, label=lbl)
    axI.set_xticks(positions_base); axI.set_xticklabels(seg_labels, rotation=15, ha="right")
    axI.set_ylabel("|σ_z| (m/s²)"); axI.set_title("I — σ_z Activity by Flight Segment")
    axI.axhline(SIG_LIM, color="darkred", ls=":", lw=1, label=f"SIG_LIM={SIG_LIM}")
    axI.legend(fontsize=6.5)

    fig.suptitle(f"Figure 2 — Detail Comparison  |  {true_payload:.1f}kg Payload  |  Gazebo SITL x500",
                 fontsize=13, y=1.01, fontweight="bold")
    _save(fig, os.path.join(outdir, fname))


# ══════════════════════════════════════════════════════════════════════════════
#  TABLES
# ══════════════════════════════════════════════════════════════════════════════
def _df_to_md(df):
    cols   = list(df.columns)
    widths = [max(len(str(c)), int(df[c].fillna("—").astype(str).str.len().max())) for c in cols]
    sep    = "| " + " | ".join("-"*w for w in widths) + " |"
    hdr    = "| " + " | ".join(str(c).ljust(w) for c, w in zip(cols, widths)) + " |"
    rows   = ["| "+" | ".join(str(vv).ljust(w) for vv, w in zip(r, widths))+" |"
              for _, r in df.iterrows()]
    return "\n".join([hdr, sep]+rows)

def save_tables(hover_rows, all_drop_rows, outdir, scenario="single", tag=""):
    """Simpan tabel metrik ke folder per-skenario, nama unik (tidak ketimpa),
    plus master CSV append untuk perbandingan lintas-run (sweep)."""
    import datetime as _dt
    ts    = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    tag_s = (tag + "_") if tag else ""
    tbl_dir = os.path.join(outdir, scenario, "tables")
    os.makedirs(tbl_dir, exist_ok=True)

    def _emit(df, base):
        # 1) file unik per-run (tidak pernah ketimpa)
        uniq = os.path.join(tbl_dir, f"{base}_{tag_s}{ts}.csv")
        df.to_csv(uniq, index=False)
        with open(uniq.replace(".csv", ".md"), "w") as f:
            f.write(f"# {base}  (tag={tag or '-'}, {ts})\n\n"); f.write(_df_to_md(df))
        # 2) master append (satu file akumulasi semua run untuk sweep)
        master = os.path.join(tbl_dir, f"{base}_MASTER.csv")
        df2 = df.copy()
        df2.insert(0, "run_tag", tag if tag else ts)
        df2.insert(1, "run_time", ts)
        df2.to_csv(master, mode="a", header=not os.path.exists(master), index=False)
        print(f"  → {uniq}")
        print(f"  → {master}  (append — akumulasi sweep)")

    if hover_rows:
        cols_h = ["payload_kg","controller","settle_s","rmse_3d_mm","rmse_z_mm","rmse_xy_mm",
                  "mae_z_mm","mae_3d_mm","sse_z_mm2","ise_z_mm2_s",
                  "max_err_mm","roll_std_deg","roll_p2p_deg","mo_m_est_kg","mo_err_pct",
                  "hover_valid_frac","sigma_z_mean","sigma_saturated","u_filt_z_err_N","thrust_norm_mean"]
        hraw = pd.DataFrame(hover_rows)
        hdf = hraw[[c for c in cols_h if c in hraw.columns]].round(3)
        _emit(hdf, "metrics_hover")

    if all_drop_rows:
        cols_d = ["label","csv_stem","controller","scenario","true_payload_kg","t_drop_s",
                  # tracking pra-drop (steady-state sebelum event)
                  "pre_rmse3d_mm","pre_rmse_z_mm","pre_mae_z_mm","pre_sse_z_mm2",
                  "pre_rmse_xy_mm","pre_roll_std_deg",
                  # sigma / disturbance
                  "pre_sigma_z","pre_sigma_z_std","pre_sigma_sat_frac","pre_thrust_norm",
                  # respons drop (transien)
                  "peak_alt_err_mm","peak_xy_err_mm","recovery_s",
                  "trans_mae_z_mm","trans_ise_z_mm2_s","trans_itae_z_mm_s2",
                  # tracking pasca-drop (steady-state setelah settle)
                  "post_rmse_z_mm","post_mae_z_mm","post_sse_z_mm2",
                  "post_rmse_xy_mm","post_roll_std_deg",
                  # event-triggered control
                  "etc_rate","etc_iei_mean","etc_n_trig",
                  # estimasi massa
                  "detected_delta_kg","delta_err_pct"]
        adf = pd.DataFrame(all_drop_rows)
        ddf = adf[[c for c in cols_d if c in adf.columns]].round(3)
        _emit(ddf, "metrics_drop")


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=textwrap.dedent("""\
        analyze_flight_logs.py v2 — L1+MO / PID flight-log analysis
        Produces Figure 1 (overview) dan Figure 2 (detail analisis) per run.
        Mendukung skenario: single drop, multi_drop, pick_drop.
        """))

    ap.add_argument("--hover",          nargs="+",  metavar="CSV")
    ap.add_argument("--hover-masses",   nargs="+",  type=float, metavar="KG")
    ap.add_argument("--drop-l1",        nargs="+",  metavar="CSV")
    ap.add_argument("--drop-pid",       nargs="+",  metavar="CSV")
    ap.add_argument("--labels",         nargs="+",  metavar="LABEL")
    ap.add_argument("--tag",            default="",
                    help="Label run untuk sweep test (dipakai di nama folder & tabel; "
                         "kalau kosong pakai timestamp). Contoh: --tag gamz120_kobs20")
    ap.add_argument("--payload",        type=float, default=1.0,
                    help="Payload utama dalam kg (per-drop untuk multi-drop)")
    ap.add_argument("--out",            default="./figs")
    ap.add_argument("--settle",         type=float, default=8.0)

    # Skenario flags
    ap.add_argument("--scenario",  default="single",
                    choices=["single", "multi_drop", "pick_drop"],
                    help="Tipe skenario flight: single | multi_drop | pick_drop")

    # Multi-drop: waktu dan payload per event (berlaku untuk SEMUA file --drop-l1)
    ap.add_argument("--drop-times",    nargs="+", type=float, metavar="T_ABS",
                    help="Waktu absolut setiap drop event (t_s dari CSV). "
                         "Jika tidak diisi, auto-detect dari max err_z.")
    ap.add_argument("--drop-payloads", nargs="+", type=float, metavar="KG",
                    help="Payload per drop event (kg). Default = --payload untuk semua event.")

    # Pick-drop: waktu pick-up
    ap.add_argument("--pick-times",    nargs="+", type=float, metavar="T_ABS",
                    help="Waktu absolut setiap pick-up event (t_s).")
    ap.add_argument("--pick-payloads", nargs="+", type=float, metavar="KG",
                    help="Payload per pick-up event (kg). Default = --payload.")

    args = ap.parse_args()

    if not args.hover and not args.drop_l1 and not args.drop_pid:
        ap.print_help(); sys.exit(0)

    os.makedirs(args.out, exist_ok=True)
    global STEADY_OFFSET; STEADY_OFFSET = args.settle

    # ── Organisasi folder: out/<scenario>/<run_id>/{l1,pid,compare} ──
    import datetime as _dt
    run_id = args.tag if args.tag else _dt.datetime.now().strftime("run_%Y%m%d_%H%M%S")
    scen_dir    = os.path.join(args.out, args.scenario)
    l1_dir      = os.path.join(scen_dir, run_id, "l1")
    pid_dir     = os.path.join(scen_dir, run_id, "pid")
    compare_dir = os.path.join(scen_dir, run_id, "compare")
    for _dd in (l1_dir, pid_dir, compare_dir):
        os.makedirs(_dd, exist_ok=True)

    hover_rows = []; hover_dfs = []; all_drop_rows = []

    # ── HOVER ──────────────────────────────────────────────────────────────
    if args.hover:
        print("\n[HOVER]")
        for i, f in enumerate(args.hover):
            d = load(f)
            if args.hover_masses and i < len(args.hover_masses):
                true_mass = args.hover_masses[i]
            else:
                w = steady_window(d)
                true_mass = round(float(col(w,"mo_m_est_kg").mean()) * 10) / 10
            hr = hover_metrics(d, true_mass)
            hover_rows.append(hr); hover_dfs.append(d)
            print(f"  {os.path.basename(f):30s} [{hr['controller'].upper()}]  "
                  f"payload≈{hr['payload_kg']:.2f}kg  RMSE3D={hr['rmse_3d_mm']:.1f}mm  "
                  f"mo_err={hr['mo_err_pct']:+.2f}%  sigma_z={hr['sigma_z_mean']:+.3f}")
        if hover_rows:
            print("\n  Generating hover plots...")
            plot_hover_sweep(hover_rows, args.out)
            plot_hover_timeseries(hover_dfs, hover_rows, args.out)

    # ── RESOLUSI EVENT TIMES ────────────────────────────────────────────────
    def resolve_events(d_ref, scenario, payload):
        """Return (drop_times_abs, drop_payloads, pick_times_abs, pick_payloads)."""
        # drop times
        if args.drop_times:
            dt_abs = args.drop_times
        else:
            # auto-detect satu drop
            t_auto, _ = detect_drop_time(d_ref)
            dt_abs = [t_auto] if t_auto is not None else []

        # drop payloads
        dp_kg = args.drop_payloads if args.drop_payloads else [payload]*len(dt_abs)
        dp_kg = dp_kg[:len(dt_abs)]
        while len(dp_kg) < len(dt_abs): dp_kg.append(payload)

        # pick times
        pt_abs = args.pick_times or []
        # pick payloads
        pp_kg  = args.pick_payloads if args.pick_payloads else [payload]*len(pt_abs)
        pp_kg  = pp_kg[:len(pt_abs)]
        while len(pp_kg) < len(pt_abs): pp_kg.append(payload)

        return dt_abs, dp_kg, pt_abs, pp_kg

    # ── DROP ───────────────────────────────────────────────────────────────
    l1_runs  = []   # list of (df, dm, label, color, csv_path, FlightEvents)
    pid_runs = []   # list of PID runs (mendukung multiple file)

    if args.drop_l1:
        print(f"\n[DROP — L1 | scenario={args.scenario}]")
        for i, f in enumerate(args.drop_l1):
            d = load(f)
            dt_abs, dp_kg, pt_abs, pp_kg = resolve_events(d, args.scenario, args.payload)
            fe = build_flight_events(args.scenario, dt_abs, pt_abs, dp_kg, pp_kg)

            # gunakan drop pertama sebagai referensi utama
            t_main = dt_abs[0] if dt_abs else float(d["t_s"].median())
            dm = drop_metrics(d, fe, args.payload)

            lbl = args.labels[i] if (args.labels and i < len(args.labels)) else stem(f)
            clr = L1_COLORS[i % len(L1_COLORS)]
            l1_runs.append((d, dm, lbl, clr, f, fe))

            dm_row = {**dm, "label": lbl, "csv_stem": stem(f), "scenario": args.scenario}
            all_drop_rows.append(dm_row)

            pk = dm.get("peak_alt_err_mm", float("nan"))
            rc = dm.get("recovery_s",      float("nan"))
            print(f"  {os.path.basename(f):30s} [{lbl}]  "
                  f"n_drops={len(fe.drops())} n_picks={len(fe.picks())}  "
                  f"peak={_f(pk,'.0f')}mm  rec={_f(rc,'.2f')}s")

            print(f"    Generating Fig1 + Fig2...")
            plot_fig1_single(d, dm, fe, args.payload, f"L1+MO [{lbl}]", clr,
                             l1_dir, f, scenario=args.scenario)
            plot_fig2_single(d, dm, fe, args.payload, f"L1+MO [{lbl}]", clr,
                             l1_dir, f, scenario=args.scenario)

    if args.drop_pid:
        print(f"\n[DROP — PID | scenario={args.scenario}]")
        for i, f in enumerate(args.drop_pid):
            dp = load(f)
            dt_abs_p, dp_kg_p, pt_abs_p, pp_kg_p = resolve_events(dp, args.scenario, args.payload)
            fe_p = build_flight_events(args.scenario, dt_abs_p, pt_abs_p, dp_kg_p, pp_kg_p)
            dm_p = drop_metrics(dp, fe_p, args.payload)
            lbl_p = (f"PID-{i+1}" if len(args.drop_pid) > 1 else "PID")
            pid_runs.append((dp, dm_p, lbl_p, PID_COLOR, f, fe_p))
            dm_row_p = {**dm_p, "label": lbl_p, "csv_stem": stem(f), "scenario": args.scenario}
            all_drop_rows.append(dm_row_p)
            pk_p = dm_p.get("peak_alt_err_mm", float("nan"))
            rc_p = dm_p.get("recovery_s",      float("nan"))
            print(f"  {os.path.basename(f):30s} [{lbl_p}]  "
                  f"peak={_f(pk_p,'.0f')}mm  rec={_f(rc_p,'.2f')}s")
            print(f"    Generating PID Fig1 + Fig2...")
            plot_fig1_single(dp, dm_p, fe_p, args.payload, f"PID [{lbl_p}]", PID_COLOR,
                             pid_dir, f, scenario=args.scenario)
            plot_fig2_single(dp, dm_p, fe_p, args.payload, f"PID [{lbl_p}]", PID_COLOR,
                             pid_dir, f, scenario=args.scenario)

    if l1_runs or pid_runs:
        print("\n  Generating comparison Fig1 + Fig2...")
        pid_run_for_cmp = pid_runs[0] if pid_runs else None
        if len(pid_runs) > 1:
            print(f"  (Catatan: {len(pid_runs)} file PID — figur perbandingan pakai file pertama; "
                  f"semua file tetap masuk tabel metrik)")
        if l1_runs:
            plot_fig1_compare(l1_runs, pid_run_for_cmp, args.payload, compare_dir)
            plot_fig2_compare(l1_runs, pid_run_for_cmp, args.payload, compare_dir)

    # ── TABLES ─────────────────────────────────────────────────────────────
    print("\n[TABLES]")
    save_tables(hover_rows, all_drop_rows, args.out, scenario=args.scenario, tag=args.tag)

    # ── CALIBRATION CHECK ──────────────────────────────────────────────────
    if hover_rows:
        print("\n[CALIBRATION CHECK]")
        calibration_check(hover_rows, os.path.join(args.out, "calibration_check.txt"))

    print(f"\nDone. Output → {scen_dir}/\n  Tabel → {scen_dir}/tables/  (file unik + _MASTER.csv akumulasi sweep)\n")


if __name__ == "__main__":
    main()
