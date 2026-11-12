#!/usr/bin/env bash
# run_campaign.sh — SITL Campaign Launcher (SIMPLIFIED)
#
# Tugas script ini HANYA:
#   1. Nyalain DDS agent + QGC sekali di awal
#   2. Loop N kali: PX4 SITL (world sesuai skenario) -> ros2 offboard skenario
#      -> l1_monitor.py -> tunggu sampai misi BENERAN selesai -> matiin semua
#      -> bersihin cache gazebo -> ulang
#   3. Simpan semua CSV per-run dalam satu folder campaign
#   4. Aggregate metrik dasar (mean/std) di akhir
#
# TIDAK diurus di sini (sengaja): ganti controller, ganti parameter L1MO,
# ganti wind, spawn/attach payload (itu sudah di-handle sendiri oleh masing-
# masing node ros2 scenario, mis. pick_drop.py manggil gz topic sendiri).
#
# PENTING: PX4 SITL di-restart dari nol setiap run. Kalau kamu butuh param
# atau controller (mc_l1_mo vs mc_pos_control) tertentu konsisten di semua
# run, itu HARUS sudah dibakar ke extras.txt / rc.mc_apps SEBELUM jalanin
# script ini. Apapun yang kamu ketik manual di pxh> tidak akan bertahan
# lewat restart.

set -uo pipefail

# ═══════════════════════════════════════════════════════════════════════════
# CONFIG — edit sesuai environment kamu
# ═══════════════════════════════════════════════════════════════════════════
PX4_DIR="${PX4_DIR:-$HOME/PX4-main}"
DRONE_WS="${DRONE_WS:-$HOME/drone_ws}"
MONITOR_PY="${MONITOR_PY:-$PX4_DIR/l1_monitor.py}"
ROS2_PKG="${ROS2_PKG:-mpc_leso_controller}"
QGC_PATHS=("$HOME/Downloads/QGroundControl.AppImage" "$HOME/QGroundControl.AppImage")
CAMPAIGNS_ROOT="${CAMPAIGNS_ROOT:-$HOME/sitl_campaigns}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# World Gazebo per skenario. CATATAN: 0_FINAL_COMMAND.md kamu sedikit
# kontradiktif soal world "drop" (tabel bilang default, bagian manual
# command bilang single_drop) — aku ikut bagian manual command (lebih
# spesifik). Edit di sini kalau ternyata salah.
declare -A WORLD_FOR=(
  [hover]="default"
  [single_drop]="single_drop"
  [multi_drop]="multi_drop"
  [pick_drop]="pickup"
)

# Executable ros2 (mpc_leso_controller <exec>) per skenario
declare -A EXEC_FOR=(
  [hover]="test_hover_mass"
  [single_drop]="single_drop"
  [multi_drop]="multi_drop"
  [pick_drop]="pick_drop"
)

# Hard safety timeout per skenario (detik) — jaring pengaman KALAU marker
# selesai tidak pernah kedetect. Nilainya generous di atas total durasi
# PHASES asli (dihitung dari source pick_drop.py=193s, multi_drop.py=124s).
declare -A MAXWAIT_FOR=(
  [hover]=150
  [single_drop]=200
  [multi_drop]=240
  [pick_drop]=320
)

# Regex marker "misi selesai" — dicocokkan case-insensitive ke log stdout
# node ros2. pick_drop.py & multi_drop.py both print "...selesai — Ctrl+C
# untuk berhenti". Kalau test_hover_mass.py / single_drop.py beda kalimat,
# ganti di sini.
COMPLETION_REGEX="${COMPLETION_REGEX:-selesai|Ctrl\+C}"

GRACE_S="${GRACE_S:-6}"          # detik nunggu setelah marker sebelum kill (rekam ekor data)
BOOT_TIMEOUT_S="${BOOT_TIMEOUT_S:-120}"  # nunggu pxh> muncul
DDS_PORT="${DDS_PORT:-8888}"

# ═══════════════════════════════════════════════════════════════════════════
# UTIL
# ═══════════════════════════════════════════════════════════════════════════
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'; CYN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'
log()  { echo -e "${CYN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GRN}[OK]${NC} $*"; }
warn() { echo -e "${YLW}[WARN]${NC} $*" >&2; }
die()  { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
sep()  { echo -e "${BOLD}──────────────────────────────────────────────────────${NC}"; }

DDS_PID=""; QGC_PID=""; PX4_PID=""; OFFBOARD_PID=""; MONITOR_PID=""

check_deps() {
  [[ -d "$PX4_DIR"  ]] || die "PX4_DIR tidak ada: $PX4_DIR"
  [[ -d "$DRONE_WS" ]] || die "DRONE_WS tidak ada: $DRONE_WS"
  [[ -f "$MONITOR_PY" ]] || die "l1_monitor.py tidak ketemu di: $MONITOR_PY (set MONITOR_PY=... kalau lokasinya beda)"
  command -v MicroXRCEAgent &>/dev/null || die "MicroXRCEAgent tidak ditemukan di PATH"
  command -v ros2 &>/dev/null || die "ros2 tidak ditemukan (sudah source setup.bash?)"
  command -v python3 &>/dev/null || die "python3 tidak ditemukan"
  ok "Dependencies OK"
}

clean_gz_cache() {
  rm -rf "$HOME/.gz/sim/" "$HOME/.gz/rendering/" /tmp/gz* 2>/dev/null || true
}

# ═══════════════════════════════════════════════════════════════════════════
# LAYANAN PERSISTEN (sekali untuk seluruh campaign): DDS agent + QGC
# ═══════════════════════════════════════════════════════════════════════════
launch_dds_agent() {
  log "Nyalain MicroXRCEAgent (port ${DDS_PORT})..."
  MicroXRCEAgent udp4 -p "$DDS_PORT" > "${CAMPAIGN_DIR}/logs/dds_agent.log" 2>&1 &
  DDS_PID=$!; disown "$DDS_PID" 2>/dev/null || true
  sleep 2
  kill -0 "$DDS_PID" 2>/dev/null && ok "DDS Agent PID: $DDS_PID" || die "DDS Agent gagal jalan"
}

launch_qgc() {
  local p
  for p in "${QGC_PATHS[@]}"; do
    if [[ -f "$p" ]]; then
      "$p" > "${CAMPAIGN_DIR}/logs/qgc.log" 2>&1 &
      QGC_PID=$!; disown "$QGC_PID" 2>/dev/null || true
      sleep 2; ok "QGC PID: $QGC_PID"
      return
    fi
  done
  warn "QGroundControl.AppImage tidak ketemu — skip (opsional, monitoring saja)"
}

kill_persistent() {
  [[ -n "$DDS_PID" ]] && kill -9 "$DDS_PID" 2>/dev/null || true
  [[ -n "$QGC_PID" ]] && kill -9 "$QGC_PID" 2>/dev/null || true
  pkill -9 -f "MicroXRCEAgent" 2>/dev/null || true
}

# ═══════════════════════════════════════════════════════════════════════════
# PER-RUN: PX4 SITL, offboard node, monitor
# ═══════════════════════════════════════════════════════════════════════════
launch_px4_sitl() {
  local world="$1" px4_log="$2"
  cd "$PX4_DIR" || die "cd gagal ke $PX4_DIR"
  if [[ "$world" == "default" ]]; then
    make px4_sitl gz_x500 > "$px4_log" 2>&1 &
  else
    PX4_GZ_WORLD="$world" make px4_sitl gz_x500 > "$px4_log" 2>&1 &
  fi
  PX4_PID=$!
  log "  PX4+Gazebo booting (world=${world}, PID=${PX4_PID}), nunggu pxh>..."
  local t=0
  while (( t < BOOT_TIMEOUT_S )); do
    grep -q "pxh>" "$px4_log" 2>/dev/null && break
    sleep 2; t=$((t+2)); printf "  ...%ds\r" "$t"
  done
  echo ""
  if (( t >= BOOT_TIMEOUT_S )); then
    warn "pxh> belum muncul setelah ${BOOT_TIMEOUT_S}s — lanjut paksa (cek ${px4_log} kalau run ini gagal)"
  fi
  sleep 5   # kasih waktu topik2 mulai publish stabil
  ok "  PX4 SITL ready"
}

launch_offboard() {
  local exec_name="$1" log_file="$2"
  (
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source "${DRONE_WS}/install/setup.bash" 2>/dev/null || true
    exec ros2 run "$ROS2_PKG" "$exec_name"
  ) > "$log_file" 2>&1 &
  OFFBOARD_PID=$!
  log "  Offboard node '${exec_name}' PID: ${OFFBOARD_PID}"
}

launch_monitor() {
  local out_csv="$1" err_log="$2"
  (
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source "${DRONE_WS}/install/setup.bash" 2>/dev/null || true
    exec python3 "$MONITOR_PY" --out "$out_csv"
  ) > "$err_log" 2>&1 &
  MONITOR_PID=$!
  sleep 2
  if kill -0 "$MONITOR_PID" 2>/dev/null; then
    ok "  l1_monitor.py PID: ${MONITOR_PID} -> $(basename "$out_csv")"
  else
    warn "  l1_monitor.py exit langsung, cek: ${err_log}"
  fi
}

# Kalau --out tidak didukung / gagal ditulis, cari file l1_monitor_*.csv
# terbaru yang muncul setelah run dimulai, lalu copy ke path yang benar.
finalize_monitor_csv() {
  local out_csv="$1" run_start_epoch="$2"
  if [[ -s "$out_csv" ]] && (( $(wc -l < "$out_csv" 2>/dev/null || echo 0) > 1 )); then
    return 0
  fi
  warn "  ${out_csv} kosong/tidak ada — cari fallback l1_monitor_*.csv..."
  local newest=""
  for d in "$PX4_DIR" "$HOME" "$SCRIPT_DIR"; do
    local cand
    cand=$(find "$d" -maxdepth 2 -name "l1_monitor_*.csv" -newermt "@${run_start_epoch}" 2>/dev/null | sort | tail -1)
    [[ -n "$cand" ]] && { newest="$cand"; break; }
  done
  if [[ -n "$newest" && -s "$newest" ]]; then
    cp "$newest" "$out_csv"
    ok "  Fallback ketemu: ${newest} -> disalin ke ${out_csv}"
  else
    warn "  Tidak ketemu fallback CSV sama sekali untuk run ini — data run ini HILANG"
  fi
}

wait_for_completion() {
  local log_file="$1" max_wait="$2"
  local waited=0
  while (( waited < max_wait )); do
    if grep -Eiq "$COMPLETION_REGEX" "$log_file" 2>/dev/null; then
      ok "  Marker 'selesai' terdeteksi (${waited}s) — grace ${GRACE_S}s buat ekor data..."
      sleep "$GRACE_S"
      return 0
    fi
    if ! kill -0 "$OFFBOARD_PID" 2>/dev/null; then
      warn "  Node offboard exit sendiri tanpa marker (${waited}s) — lanjut"
      return 0
    fi
    sleep 1; waited=$((waited+1))
    printf "  nunggu misi selesai... %ds/%ds\r" "$waited" "$max_wait"
  done
  echo ""
  warn "  Timeout ${max_wait}s tanpa marker — paksa matikan (cek ${log_file} kalau ini sering kejadian)"
  return 1
}

kill_iteration() {
  log "  Matiin run ini..."
  [[ -n "$OFFBOARD_PID" ]] && kill -INT "$OFFBOARD_PID" 2>/dev/null || true
  sleep 2
  [[ -n "$OFFBOARD_PID" ]] && kill -9 "$OFFBOARD_PID" 2>/dev/null || true
  [[ -n "$MONITOR_PID"  ]] && kill -TERM "$MONITOR_PID" 2>/dev/null || true
  sleep 1
  [[ -n "$MONITOR_PID"  ]] && kill -9 "$MONITOR_PID" 2>/dev/null || true
  pkill -9 -f "px4" 2>/dev/null || true
  pkill -9 -f "gz sim" 2>/dev/null || true
  pkill -9 -f "ruby"   2>/dev/null || true
  sleep 2
}

# ═══════════════════════════════════════════════════════════════════════════
# SATU RUN LENGKAP
# ═══════════════════════════════════════════════════════════════════════════
run_one() {
  local scenario="$1" run_idx="$2" n_runs="$3"
  local run_tag; run_tag=$(printf "%s_run%02d" "$scenario" "$run_idx")
  local raw_csv="${CAMPAIGN_DIR}/raw/${run_tag}.csv"
  local px4_log="${CAMPAIGN_DIR}/logs/${run_tag}_px4.log"
  local offboard_log="${CAMPAIGN_DIR}/logs/${run_tag}_offboard.log"
  local monitor_err="${CAMPAIGN_DIR}/logs/${run_tag}_monitor_stderr.log"
  local run_start_epoch; run_start_epoch=$(date +%s)

  echo ""; sep; log "${BOLD}RUN ${run_idx}/${n_runs} — ${scenario}${NC}"; sep

  clean_gz_cache
  launch_px4_sitl "${WORLD_FOR[$scenario]}" "$px4_log"
  launch_offboard "${EXEC_FOR[$scenario]}" "$offboard_log"
  sleep 1
  launch_monitor "$raw_csv" "$monitor_err"

  wait_for_completion "$offboard_log" "${MAXWAIT_FOR[$scenario]}"
  kill_iteration
  finalize_monitor_csv "$raw_csv" "$run_start_epoch"
  clean_gz_cache

  ok "Run ${run_idx}/${n_runs} selesai -> ${raw_csv}"
}

# ═══════════════════════════════════════════════════════════════════════════
# MENU
# ═══════════════════════════════════════════════════════════════════════════
SCENARIO=""; N_RUNS=""; TAG=""

show_help() {
cat <<EOF
run_campaign.sh — SITL campaign loop (hover|single_drop|multi_drop|pick_drop)

  -s SCENARIO   hover | single_drop | multi_drop | pick_drop
  -n RUNS       jumlah run
  -t TAG        nama tag folder campaign
  -h            help

Contoh:
  ./run_campaign.sh -s pick_drop -n 10 -t l1_ko05_wind
  ./run_campaign.sh          # interaktif
EOF
}

while getopts "s:n:t:h" opt; do
  case "$opt" in
    s) SCENARIO="$OPTARG" ;;
    n) N_RUNS="$OPTARG" ;;
    t) TAG="$OPTARG" ;;
    h) show_help; exit 0 ;;
    *) show_help; exit 1 ;;
  esac
done

interactive_menu() {
  echo -e "\n${BOLD}╔══════════════════════════════════════╗"
  echo -e "║   SITL Campaign Launcher (simple)    ║"
  echo -e "╚══════════════════════════════════════╝${NC}\n"
  echo -e "${CYN}[1] Skenario:${NC}"
  echo "    1) hover"
  echo "    2) single_drop"
  echo "    3) multi_drop"
  echo "    4) pick_drop"
  read -rp "Pilih [1-4]: " sc
  case "$sc" in
    1) SCENARIO=hover ;; 2) SCENARIO=single_drop ;;
    3) SCENARIO=multi_drop ;; 4) SCENARIO=pick_drop ;;
    *) die "Pilihan tidak valid" ;;
  esac
  echo -e "\n${CYN}[2] Jumlah run:${NC}"
  read -rp "Runs [default 10]: " rn; N_RUNS="${rn:-10}"
  echo -e "\n${CYN}[3] Tag folder campaign:${NC}"
  read -rp "Tag (mis. l1_ko05_wind): " tg
  TAG="${tg:-run_$(date +%Y%m%d_%H%M%S)}"

  sep
  echo -e "  Skenario : ${YLW}${SCENARIO}${NC}"
  echo -e "  Runs     : ${YLW}${N_RUNS}${NC}"
  echo -e "  Tag      : ${YLW}${TAG}${NC}"
  echo -e "  World    : ${YLW}${WORLD_FOR[$SCENARIO]}${NC}"
  echo -e "  Exec     : ${YLW}ros2 run ${ROS2_PKG} ${EXEC_FOR[$SCENARIO]}${NC}"
  sep
  read -rp $'\nLanjut? [y/N]: ' confirm
  [[ "${confirm,,}" == "y" ]] || { echo "Dibatalkan."; exit 0; }
}

# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════
main() {
  [[ -z "$SCENARIO" ]] && interactive_menu
  [[ -z "${WORLD_FOR[$SCENARIO]:-}" ]] && die "Skenario tidak dikenal: ${SCENARIO} (pakai: hover|single_drop|multi_drop|pick_drop)"
  N_RUNS="${N_RUNS:-10}"
  TAG="${TAG:-run_$(date +%Y%m%d_%H%M%S)}"

  CAMPAIGN_DIR="${CAMPAIGNS_ROOT}/${TAG}_${SCENARIO}_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "${CAMPAIGN_DIR}/raw" "${CAMPAIGN_DIR}/logs" "${CAMPAIGN_DIR}/analysis"

  sep; log "${BOLD}CAMPAIGN: ${SCENARIO} x${N_RUNS} runs${NC}"
  log "  Folder: ${CAMPAIGN_DIR}"; sep

  check_deps

  trap 'echo ""; warn "Interrupted — cleanup..."; kill_iteration; kill_persistent; clean_gz_cache; exit 130' INT TERM

  launch_dds_agent
  launch_qgc

  for (( i=1; i<=N_RUNS; i++ )); do
    run_one "$SCENARIO" "$i" "$N_RUNS"
  done

  sep; log "Semua run selesai. Cleanup layanan persisten..."
  kill_persistent

  log "Aggregating metrics..."
  python3 "${SCRIPT_DIR}/aggregate_metrics.py" \
    --dir "${CAMPAIGN_DIR}/raw" \
    --out "${CAMPAIGN_DIR}/analysis" \
    && ok "Summary -> ${CAMPAIGN_DIR}/analysis/metrics_summary_mean_std.csv" \
    || warn "Aggregate gagal — cek manual CSV di ${CAMPAIGN_DIR}/raw/"

  sep
  echo -e "${BOLD}SELESAI${NC}"
  log "  Raw CSV     : ${CAMPAIGN_DIR}/raw/"
  log "  Per-run     : ${CAMPAIGN_DIR}/analysis/metrics_raw_per_run.csv"
  log "  Mean/std    : ${CAMPAIGN_DIR}/analysis/metrics_summary_mean_std.csv"
  sep
  echo ""
  echo "Untuk analisis lengkap (RMSE pre/post drop, recovery_s, error massa dsb)"
  echo "pakai 1_analyze_flight_logs.py yang sudah divalidasi. Contoh siap-pakai:"
  echo ""
  local file_list; file_list=$(ls "${CAMPAIGN_DIR}"/raw/*.csv 2>/dev/null | tr '\n' ' ')
  if [[ "$SCENARIO" == "hover" ]]; then
    echo "  python3 1_analyze_flight_logs.py --hover ${file_list}--tag ${TAG} --out ${CAMPAIGN_DIR}/analysis"
  else
    local scflag="single"; [[ "$SCENARIO" == "multi_drop" || "$SCENARIO" == "pick_drop" ]] && scflag="$SCENARIO"
    echo "  python3 1_analyze_flight_logs.py --drop-l1 ${file_list}--scenario ${scflag} \\"
    echo "      --payload <ISI_KG_DI_SINI> --tag ${TAG} --out ${CAMPAIGN_DIR}/analysis"
  fi
  echo ""
}

main "$@"
