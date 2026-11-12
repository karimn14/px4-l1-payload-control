#!/usr/bin/env bash
# =============================================================================
# sweep_runner.sh — L1MO Parameter Sweep
#
# Jalankan secara sekuensial: untuk setiap nilai parameter,
# switch ke L1, set param, hover N detik, catat metrik.
#
# Usage:
#   ./sweep_runner.sh                         # menu interaktif
#   ./sweep_runner.sh -p KP_Z -v "3.0 4.0 6.0 8.0" -n 3
#   ./sweep_runner.sh -p GAMMA_Z -v "60 100 150 200 300" -n 3
#   ./sweep_runner.sh --full                  # semua parameter penting
#
# Hasil: logs/sweep_PARAM_YYYYMMDD/sweep_results.csv
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/run_campaign.sh" --dry-run -s hover -c l1 -p 0.6 -n 1 \
  2>/dev/null || true   # load fungsi-fungsi dari run_campaign.sh

# Override beberapa variable
SESSION_ID="sweep_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${SCRIPT_DIR}/logs/${SESSION_ID}"
RESULTS_CSV="${LOG_DIR}/sweep_results.csv"

# ─── Colors ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'
BLU='\033[0;34m'; CYN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'
log()  { echo -e "${CYN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GRN}[OK]${NC} $*"; }
warn() { echo -e "${YLW}[WARN]${NC} $*"; }
die()  { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }
sep()  { echo -e "${BOLD}─────────────────────────────────────────────────${NC}"; }

# ─── Parameter definitions ────────────────────────────────────────────────────
# Format: "PARAM_SUFFIX|default|unit|description|values_to_sweep"
# PARAM_SUFFIX dipakai sebagai L1MO_<SUFFIX>
declare -A SWEEP_PARAMS
declare -A SWEEP_DEFAULTS
declare -A SWEEP_UNITS
declare -A SWEEP_DESC

# Position/velocity gains
SWEEP_PARAMS[KP_XY]="0.8 1.0 1.5 2.0"
SWEEP_DEFAULTS[KP_XY]="1.0"; SWEEP_UNITS[KP_XY]=""
SWEEP_DESC[KP_XY]="XY position P gain (tinggi=respons cepat, terlalu tinggi=osilasi)"

SWEEP_PARAMS[KD_XY]="0.3 0.5 0.8 1.2"
SWEEP_DEFAULTS[KD_XY]="0.5"; SWEEP_UNITS[KD_XY]=""
SWEEP_DESC[KD_XY]="XY velocity D gain (limit-cycle risk jika terlalu tinggi)"

SWEEP_PARAMS[KP_Z]="2.0 3.0 4.0 6.0 8.0"
SWEEP_DEFAULTS[KP_Z]="3.0"; SWEEP_UNITS[KP_Z]=""
SWEEP_DESC[KP_Z]="Z position P gain"

SWEEP_PARAMS[KD_Z]="1.0 1.5 2.0 3.0 4.0"
SWEEP_DEFAULTS[KD_Z]="1.5"; SWEEP_UNITS[KD_Z]=""
SWEEP_DESC[KD_Z]="Z velocity D gain (paper: KD>2.1 → 0.66Hz limit-cycle)"

# L1 adaptive rates
SWEEP_PARAMS[GAMMA_X]="30 60 100 150 200"
SWEEP_DEFAULTS[GAMMA_X]="60"; SWEEP_UNITS[GAMMA_X]=""
SWEEP_DESC[GAMMA_X]="X adaptation rate (tinggi=adaptasi cepat, bisa amplify noise)"

SWEEP_PARAMS[GAMMA_Y]="30 60 100 150 200"
SWEEP_DEFAULTS[GAMMA_Y]="60"; SWEEP_UNITS[GAMMA_Y]=""
SWEEP_DESC[GAMMA_Y]="Y adaptation rate"

SWEEP_PARAMS[GAMMA_Z]="80 100 150 200 300"
SWEEP_DEFAULTS[GAMMA_Z]="150"; SWEEP_UNITS[GAMMA_Z]=""
SWEEP_DESC[GAMMA_Z]="Z adaptation rate (lebih kritis — pengaruhi payload drop recovery)"

# Low-pass filter
SWEEP_PARAMS[LPF_Z]="5.0 6.0 8.0 10.0 15.0"
SWEEP_DEFAULTS[LPF_Z]="8.0"; SWEEP_UNITS[LPF_Z]="rad/s"
SWEEP_DESC[LPF_Z]="Z LPF cutoff (rendah=smooth, tinggi=responsif+noisy)"

# Observer gain
SWEEP_PARAMS[KOBS_Z]="8.0 12.0 15.0 20.0"
SWEEP_DEFAULTS[KOBS_Z]="15.0"; SWEEP_UNITS[KOBS_Z]=""
SWEEP_DESC[KOBS_Z]="Z state predictor observer gain"

# Sigma limit
SWEEP_PARAMS[SIG_LIM]="1.0 1.5 2.0 3.0 4.0"
SWEEP_DEFAULTS[SIG_LIM]="2.0"; SWEEP_UNITS[SIG_LIM]="m/s²"
SWEEP_DESC[SIG_LIM]="Disturbance estimate saturation (terlalu kecil=clamp, terlalu besar=windup)"

# Mass estimator
SWEEP_PARAMS[HE_TAU]="1.0 2.0 3.0 5.0"
SWEEP_DEFAULTS[HE_TAU]="2.0"; SWEEP_UNITS[HE_TAU]="s"
SWEEP_DESC[HE_TAU]="Hover estimator filter tau (rendah=cepat, terlalu rendah=noise)"

SWEEP_PARAMS[MO_DTH]="0.03 0.05 0.08 0.10"
SWEEP_DEFAULTS[MO_DTH]="0.05"; SWEEP_UNITS[MO_DTH]="kg"
SWEEP_DESC[MO_DTH]="MO transient detection threshold"

# ─── CLI args ─────────────────────────────────────────────────────────────────
ARG_PARAM=""
ARG_VALUES=""
ARG_RUNS=3
ARG_PAYLOAD="0.6"
ARG_FULL=false
ARG_DRY_RUN=false
PX4_DIR="${PX4_DIR:-$HOME/PX4-main}"
DRONE_WS="${DRONE_WS:-$HOME/drone_ws}"
ARG_SKIP_QGC=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--param)    ARG_PARAM="$2";      shift 2 ;;
    -v|--values)   ARG_VALUES="$2";     shift 2 ;;
    -n|--runs)     ARG_RUNS="$2";       shift 2 ;;
    --payload)     ARG_PAYLOAD="$2";    shift 2 ;;
    --full)        ARG_FULL=true;       shift   ;;
    --dry-run)     ARG_DRY_RUN=true;    shift   ;;
    --skip-qgc)    ARG_SKIP_QGC=true;   shift   ;;
    -h|--help)
      echo "Usage: $0 -p KP_Z -v '3.0 4.0 6.0' -n 3"
      echo "       $0 --full  (sweep semua parameter)"
      echo ""
      echo "Parameters yang tersedia:"
      for k in "${!SWEEP_PARAMS[@]}"; do
        printf "  L1MO_%-12s default=%-6s  %s\n" \
          "$k" "${SWEEP_DEFAULTS[$k]}" "${SWEEP_DESC[$k]}"
      done | sort
      exit 0 ;;
    *) die "Argumen tidak dikenal: $1" ;;
  esac
done

# ─── Interactive menu ─────────────────────────────────────────────────────────
sweep_menu() {
  echo ""
  echo -e "${BOLD}╔══════════════════════════════════════════════════╗${NC}"
  echo -e "${BOLD}║         L1MO Parameter Sweep                     ║${NC}"
  echo -e "${BOLD}╚══════════════════════════════════════════════════╝${NC}"
  echo ""
  echo -e "${BLU}Parameter yang tersedia:${NC}"
  local i=1
  local param_list=()
  for k in $(echo "${!SWEEP_PARAMS[@]}" | tr ' ' '\n' | sort); do
    printf "  %2d) L1MO_%-12s [default=%-5s] %s\n" \
      "$i" "$k" "${SWEEP_DEFAULTS[$k]}" "${SWEEP_DESC[$k]}"
    param_list+=("$k")
    (( i++ ))
  done
  echo "   0) ALL — sweep semua parameter (penuh)"
  echo ""
  read -rp "Pilih parameter [0-$((i-1))]: " choice

  if [[ "$choice" == "0" ]]; then
    ARG_FULL=true
  else
    ARG_PARAM="${param_list[$((choice-1))]}"
    echo ""
    echo -e "  Default values: ${YLW}${SWEEP_PARAMS[$ARG_PARAM]}${NC}"
    read -rp "  Nilai sweep (space-separated, Enter=default): " custom_vals
    [[ -n "$custom_vals" ]] && ARG_VALUES="$custom_vals" || \
      ARG_VALUES="${SWEEP_PARAMS[$ARG_PARAM]}"
  fi

  read -rp "Runs per nilai [default 3]: " rn
  ARG_RUNS="${rn:-3}"

  echo ""
  echo -e "${BLU}Payload (kg):${NC} 0.3  0.6  0.8  1.0  1.2"
  read -rp "Payload [default 0.6]: " pl
  ARG_PAYLOAD="${pl:-0.6}"
}

# ─── Fungsi kirim command ke PX4 ──────────────────────────────────────────────
_px4_cmd_sweep() {
  local cmd="$1"
  local px4_pid
  px4_pid=$(pgrep -x "px4" 2>/dev/null | head -1)

  if [[ -n "$px4_pid" ]] && [[ -w "/proc/${px4_pid}/fd/0" ]]; then
    printf "%s\n" "$cmd" > "/proc/${px4_pid}/fd/0"
    return 0
  fi
  warn "Tidak bisa kirim '$cmd' — PID $px4_pid tidak writable"
  return 1
}

# ─── Switch ke L1 dengan VERIFIKASI ───────────────────────────────────────────
switch_to_l1_verified() {
  local param_name="$1"
  local param_val="$2"
  local px4_log="${LOG_DIR}/px4_sitl.log"

  log "Switch L1: mc_pos_control stop → set ${param_name}=${param_val} → mc_l1_mo start"

  # Stop PID
  _px4_cmd_sweep "mc_pos_control stop"
  sleep 1

  # Set parameter
  _px4_cmd_sweep "param set L1MO_${param_name} ${param_val}"
  sleep 0.5

  # Start L1
  _px4_cmd_sweep "mc_l1_mo start"
  sleep 3

  # Verifikasi: cek L1 modul jalan via param show
  local verified=false
  for attempt in 1 2 3 4 5; do
    _px4_cmd_sweep "param show L1MO_${param_name}"
    sleep 1
    local log_tail
    log_tail=$(tail -5 "$px4_log" 2>/dev/null || echo "")
    if echo "$log_tail" | grep -q "L1MO_${param_name}"; then
      local actual_val
      actual_val=$(echo "$log_tail" | grep "L1MO_${param_name}" | \
        grep -oE '[0-9]+\.?[0-9]*' | tail -1)
      if [[ "$actual_val" == "$param_val" ]] || \
         [[ "$(printf '%.4f' "$actual_val" 2>/dev/null)" == \
            "$(printf '%.4f' "$param_val" 2>/dev/null)" ]]; then
        ok "  VERIFIED: L1MO_${param_name} = ${actual_val}"
        verified=true
        break
      fi
    fi
    warn "  Attempt ${attempt}/5: belum terverifikasi..."
    sleep 1
  done

  if ! $verified; then
    warn "  TIDAK TERVERIFIKASI — kemungkinan mc_l1_mo belum start"
    warn "  Manual: di pxh> jalankan: px4 ps | grep l1"
  fi

  return 0
}

# ─── Reset ke default ─────────────────────────────────────────────────────────
reset_param() {
  local param_name="$1"
  local default_val="${SWEEP_DEFAULTS[$param_name]}"
  log "Reset L1MO_${param_name} → ${default_val} (default)"
  _px4_cmd_sweep "param set L1MO_${param_name} ${default_val}"
  sleep 0.5
}

# ─── Single sweep run ──────────────────────────────────────────────────────────
run_sweep_single() {
  local param_name="$1"
  local param_val="$2"
  local payload="$3"
  local run_num="$4"

  local run_id; run_id="$(printf '%02d' "$run_num")"
  local tag="sweep_L1MO_${param_name}_${param_val}_${payload}kg_run${run_id}"
  tag="${tag//\./_}"   # ganti titik dengan underscore untuk nama file

  echo ""; sep
  log "SWEEP: L1MO_${param_name}=${param_val} | payload=${payload}kg | run ${run_id}"
  sep

  if $ARG_DRY_RUN; then
    log "[DRY] switch L1 → set L1MO_${param_name}=${param_val} → hover → extract"
    return
  fi

  mkdir -p "$LOG_DIR"

  # Kill + clean
  pkill -9 -f "px4" 2>/dev/null || true
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
  pkill -9 -f "l1_monitor" 2>/dev/null || true
  sleep 2
  rm -rf ~/.gz/sim/ ~/.gz/rendering/ /tmp/gz* 2>/dev/null || true

  # DDS Agent
  MicroXRCEAgent udp4 -p 8888 > "${LOG_DIR}/${tag}_dds.log" 2>&1 &
  local dds_pid=$!
  sleep 2

  # QGC
  if ! $ARG_SKIP_QGC; then
    for qgc_p in "$HOME/Downloads/QGroundControl.AppImage" \
                 "$HOME/QGroundControl.AppImage"; do
      [[ -f "$qgc_p" ]] && { "$qgc_p" > /dev/null 2>&1 & break; }
    done
    sleep 2
  fi

  # PX4 SITL
  local px4_log="${LOG_DIR}/px4_sitl.log"
  cd "$PX4_DIR"
  make px4_sitl gz_x500 > "$px4_log" 2>&1 &
  local px4_pid=$!

  # Tunggu pxh>
  local waited=0
  while [[ $waited -lt 90 ]]; do
    grep -q "pxh>" "$px4_log" 2>/dev/null && break
    sleep 2; waited=$((waited+2))
    printf "  Menunggu PX4... %ds\r" "$waited"
  done
  echo ""
  sleep 5

  # Switch ke L1 + set parameter
  switch_to_l1_verified "$param_name" "$param_val"

  # Monitor
  local monitor_err="${LOG_DIR}/${tag}_monitor_stderr.log"
  : > "$monitor_err"
  if [[ -f "${PX4_DIR}/l1_monitor.py" ]]; then
    (
      source /opt/ros/humble/setup.bash 2>/dev/null || true
      source "${DRONE_WS}/install/setup.bash" 2>/dev/null || true
      cd "${LOG_DIR}"
      if python3 "${PX4_DIR}/l1_monitor.py" --help 2>/dev/null | grep -q -- "--out"; then
        exec python3 "${PX4_DIR}/l1_monitor.py" --out "${LOG_DIR}/${tag}_l1monitor.csv"
      else
        exec python3 "${PX4_DIR}/l1_monitor.py"
      fi
    ) >> "$monitor_err" 2>&1 &
    local monitor_pid=$!
    sleep 3
  fi

  # Offboard hover (30 detik settle + 60 detik data collection)
  local hover_log="${LOG_DIR}/${tag}_hover.log"
  (
    source /opt/ros/humble/setup.bash
    source "${DRONE_WS}/install/setup.bash"
    export SITL_SCENARIO=hover
    export SITL_CONTROLLER=l1
    export SITL_PAYLOAD_KG="$payload"
    export SITL_LOG_DIR="$LOG_DIR"
    export SITL_TAG="$tag"
    exec ros2 run mpc_leso_controller test_hover_mass
  ) > "$hover_log" 2>&1 &
  local hover_pid=$!

  # Tunggu hover selesai via watchdog
  if [[ -f "${SCRIPT_DIR}/_landing_watchdog.py" ]]; then
    (
      source /opt/ros/humble/setup.bash 2>/dev/null || true
      source "${DRONE_WS}/install/setup.bash" 2>/dev/null || true
      python3 "${SCRIPT_DIR}/_landing_watchdog.py" --timeout 90 --grace 15
    ) > "${LOG_DIR}/${tag}_watchdog.log" 2>&1
    local wd_rc=$?
    [[ $wd_rc -eq 0 ]] && ok "Hover selesai via watchdog" || warn "Watchdog timeout"
  else
    sleep 90
  fi

  kill "$hover_pid" 2>/dev/null || true
  sleep 2
  kill -9 "$hover_pid" 2>/dev/null || true

  # Stop monitor
  sleep 3
  kill "${monitor_pid:-0}" 2>/dev/null || true
  sleep 2

  # Cari CSV monitor
  local monitor_csv=""
  for csv_path in \
    "${LOG_DIR}/${tag}_l1monitor.csv" \
    "${LOG_DIR}"/l1_monitor_*.csv \
    "${HOME}"/l1_monitor_*.csv; do
    [[ -s "$csv_path" ]] && { monitor_csv="$csv_path"; break; }
  done

  # Extract metrik hover + catat param sweep
  local line_count=0
  [[ -n "$monitor_csv" ]] && line_count=$(wc -l < "$monitor_csv" 2>/dev/null || echo 0)

  if [[ "$line_count" -gt 1 ]]; then
    local hover_metrics
    hover_metrics=$(python3 - << PYEOF 2>/dev/null
import sys, csv, math, numpy as np
sys.path.insert(0, '${SCRIPT_DIR}')
import extract_metrics as em
data = em.load_csv('${monitor_csv}')
hm   = em.compute_hover_metrics(data)
print(f"{hm.get('rmse_z_mm',''):.2f}|{hm.get('rmse_3d_mm',''):.2f}|{hm.get('roll_std_deg',''):.3f}|{hm.get('m_est_kg',''):.4f}|{hm.get('etc_rate_hz',''):.3f}")
PYEOF
)
    IFS='|' read -r rmse_z rmse_3d roll_std m_est etc_rate <<< "$hover_metrics"

    # Append ke sweep CSV
    echo "${tag},L1MO_${param_name},${param_val},${payload},${run_id},${rmse_z},${rmse_3d},${roll_std},${m_est},${etc_rate}" \
      >> "$RESULTS_CSV"
    ok "Result: rmse_z=${rmse_z}mm rmse_3d=${rmse_3d}mm roll=${roll_std}deg m_est=${m_est}kg"
  else
    warn "Tidak ada data monitor untuk run ini — catat N/A"
    echo "${tag},L1MO_${param_name},${param_val},${payload},${run_id},,,,,"\
      >> "$RESULTS_CSV"
  fi

  # Kill all
  pkill -TERM -f "gz sim" 2>/dev/null || true
  pkill -TERM -f "px4"    2>/dev/null || true
  pkill -TERM -f "MicroXRCEAgent" 2>/dev/null || true
  sleep 3
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "px4"    2>/dev/null || true
  sleep 1
}

# ─── Sweep loop satu parameter ─────────────────────────────────────────────────
sweep_one_param() {
  local param_name="$1"
  local values_str="$2"
  local payload="$3"
  local n_runs="$4"

  read -ra values <<< "$values_str"
  local total=$(( ${#values[@]} * n_runs ))
  local done_count=0

  log "${BOLD}SWEEP L1MO_${param_name}: ${values[*]}${NC}"
  log "  ${#values[@]} values × ${n_runs} runs = ${total} total runs"
  sep

  for val in "${values[@]}"; do
    for (( i=1; i<=n_runs; i++ )); do
      run_sweep_single "$param_name" "$val" "$payload" "$i"
      (( done_count++ ))
      log "Progress sweep: ${done_count}/${total}"
    done
    # Reset ke default setelah selesai sweep satu nilai
    # (tidak perlu karena tiap run fresh PX4)
  done

  # Agregasi hasil sweep parameter ini
  log "Agregasi hasil sweep L1MO_${param_name}..."
  python3 - << PYEOF 2>/dev/null
import csv, sys
from collections import defaultdict
import numpy as np

rows = []
try:
    with open('${RESULTS_CSV}', newline='') as f:
        rows = list(csv.DictReader(f))
except: pass

# Filter untuk param ini saja
rows = [r for r in rows if r.get('param_name','') == 'L1MO_${param_name}']
groups = defaultdict(list)
for r in rows:
    groups[r.get('param_val','')].append(r)

print(f"\n{'L1MO_${param_name}':<10} {'RMSE_Z(mm)':>12} {'RMSE_3D(mm)':>13} {'Roll_std(°)':>12} {'M_est(kg)':>10}")
print("─" * 60)
for val, grp in sorted(groups.items(), key=lambda x: float(x[0]) if x[0] else 0):
    def ms(col):
        vs = []
        for r in grp:
            try: vs.append(float(r[col]))
            except: pass
        return (np.mean(vs), np.std(vs,ddof=1) if len(vs)>1 else 0.) if vs else (float('nan'),float('nan'))
    rz_m,  rz_s  = ms('rmse_z_mm')
    r3_m,  r3_s  = ms('rmse_3d_mm')
    rl_m,  rl_s  = ms('roll_std_deg')
    me_m,  _     = ms('m_est_kg')
    print(f"  {val:<8} {rz_m:>8.2f}±{rz_s:<4.2f} "
          f"{r3_m:>8.2f}±{r3_s:<4.2f} "
          f"{rl_m:>8.3f}±{rl_s:<4.3f} "
          f"{me_m:>8.4f}")
print()
PYEOF

  ok "Sweep L1MO_${param_name} selesai"
}

# ─── Full sweep semua parameter ───────────────────────────────────────────────
sweep_full() {
  local n_runs="$1"
  local payload="$2"

  # Urutan: dari yang paling penting/basic ke kompleks
  local priority_order=(KP_Z KD_Z GAMMA_Z LPF_Z KOBS_Z SIG_LIM KP_XY KD_XY GAMMA_X GAMMA_Y HE_TAU MO_DTH)

  local total_params=${#priority_order[@]}
  local param_num=0
  log "${BOLD}FULL SWEEP: ${total_params} parameters × ${n_runs} runs each${NC}"
  sep

  for param_name in "${priority_order[@]}"; do
    (( param_num++ ))
    log "${BOLD}[${param_num}/${total_params}] Sweeping L1MO_${param_name}${NC}"
    sweep_one_param "$param_name" "${SWEEP_PARAMS[$param_name]}" "$payload" "$n_runs"
  done

  ok "FULL SWEEP SELESAI → ${RESULTS_CSV}"
}

# ─── Main ─────────────────────────────────────────────────────────────────────
main_sweep() {
  mkdir -p "$LOG_DIR"

  # Init CSV
  if [[ ! -f "$RESULTS_CSV" ]]; then
    echo "tag,param_name,param_val,payload_kg,run_id,rmse_z_mm,rmse_3d_mm,roll_std_deg,m_est_kg,etc_rate_hz" \
      > "$RESULTS_CSV"
    ok "Sweep CSV: ${RESULTS_CSV}"
  fi

  # Interactive jika tidak ada args
  [[ -z "$ARG_PARAM" ]] && ! $ARG_FULL && sweep_menu

  log "Session: ${SESSION_ID}"
  log "Log dir: ${LOG_DIR}"

  if $ARG_FULL; then
    sweep_full "$ARG_RUNS" "$ARG_PAYLOAD"
  else
    [[ -z "$ARG_VALUES" ]] && ARG_VALUES="${SWEEP_PARAMS[$ARG_PARAM]}"
    sweep_one_param "$ARG_PARAM" "$ARG_VALUES" "$ARG_PAYLOAD" "$ARG_RUNS"
  fi

  sep
  log "${BOLD}SELESAI${NC}"
  log "  Results : ${RESULTS_CSV}"
  log "  Plot    : python3 ${SCRIPT_DIR}/extract_metrics.py --plot ${RESULTS_CSV}"
  sep
}

main_sweep "$@"
