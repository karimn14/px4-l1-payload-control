#!/usr/bin/env bash
# sweep_test.sh — SITL campaign: hover | single_drop | multi_drop | pickup
#                 mode: sweep (1 parameter, banyak nilai) | normal (N run, baseline lock)
#
# DIBANGUN ULANG dari run_campaign.sh lama — fungsi dipersempit sesuai permintaan:
#   - tidak ada QGC launcher, motor_fault, dynamic_drop, payload-matrix, "all"
#   - param injection lewat extras.txt (FEXTRAS), TIDAK edit rc.mc_apps
#     -> tidak perlu rebuild ROMFS tiap run, aman dari risiko ASCII-encoding
#   - baseline param = dump param-lock yang kamu kirim (lihat BASELINE_PARAMS)
#
# ╔══════════════════════════════════════════════════════════════════════╗
# ║ ARAHAN — WAJIB DICEK SEBELUM DIPAKAI                                  ║
# ║ 1. Verifikasi FEXTRAS masih dipakai rcS SITL kamu:                    ║
# ║      grep -rn "FEXTRAS\|extras.txt" \                                ║
# ║        "$PX4_DIR"/ROMFS/px4fmu_common/init.d-posix/rcS                ║
# ║    Kalau TIDAK ada -> mekanisme extras.txt di skrip ini tidak akan    ║
# ║    terbaca PX4. Beri tahu saya, saya alihkan ke edit rc.mc_apps.      ║
# ║ 2. ROS2_PKG / nama executable tiap skenario (lihat blok di bawah) —   ║
# ║    saya isi dengan asumsi nama lama (mpc_leso_controller). Cek/ganti. ║
# ║ 3. Pemanggilan 1_analyze_flight_logs.py di run_analysis() ditandai    ║
# ║    [VERIFIKASI] — saya tidak punya CLI persisnya. Cek argumennya.    ║
# ║ 4. Topic posisi: pick_drop.py pakai vehicle_local_position (TANPA     ║
# ║    _v1), tapi test_hover_mass.py & offboard_controller.py pakai      ║
# ║    vehicle_local_position_v1. SAMAKAN dulu ke nama yang benar-benar  ║
# ║    ada di `ros2 topic list` sebelum run sweep semalam.               ║
# ╚══════════════════════════════════════════════════════════════════════╝

set -uo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-main}"
DRONE_WS="${DRONE_WS:-$HOME/drone_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${SCRIPT_DIR}/raw_logs/${SESSION_ID}"
OUT_ROOT="${SCRIPT_DIR}/out"     # dipakai 1_analyze_flight_logs.py (out/<scenario>/<tag>/)

# ── ROS2 package / executable per skenario — CEK ulang (lihat ARAHAN #2) ──
ROS2_PKG="${ROS2_PKG:-mpc_leso_controller}"
declare -A SCENARIO_EXE=(
  [hover]="test_hover_mass"
  [single_drop]="offboard_controller"
  [multi_drop]="multi_drop"
  [pickup]="pick_drop"          # [VERIFIKASI] nama entry point belum pasti
)
declare -A SCENARIO_WORLD=(
  [hover]="default"
  [single_drop]="default"
  [multi_drop]="multi_drop"
  [pickup]="pickup"
)
declare -A SCENARIO_TIMEOUT=(
  [hover]=90
  [single_drop]=80
  [multi_drop]=180
  [pickup]=300
)
# Payload (kg) untuk single_drop — multi_drop & pickup payloadnya fixed di world
declare -A PAYLOAD_MODEL=(
  [0.3]="payload_box_03kg" [0.6]="payload_box_06kg" [0.8]="payload_box_08kg"
  [1.0]="payload_box_1kg"  [1.2]="payload_box_1_2kg"
)

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'
BLU='\033[0;34m'; CYN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'
log()  { echo -e "${CYN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GRN}[OK]${NC} $*"; }
warn() { echo -e "${YLW}[WARN]${NC} $*" >&2; }
die()  { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
sep()  { echo -e "${BOLD}──────────────────────────────────────────────${NC}"; }

# ═══════════════════════════════════════════════════════════════════════
# BASELINE PARAMETER LOCK — dari dump kamu (cek ARAHAN soal diskrepansi
# vs nilai BAB III di tabel chat sebelum percaya angka ini buta-buta)
# ═══════════════════════════════════════════════════════════════════════
declare -A BASELINE_PARAMS=(
  [L1MO_ETC_XY]=0.30   [L1MO_ETC_Z]=0.50
  [L1MO_F_MAX]=40.26
  [L1MO_GAMMA_X]=40.0  [L1MO_GAMMA_Y]=40.0  [L1MO_GAMMA_Z]=120.0
  [L1MO_KD_XY]=2.0     [L1MO_KD_Z]=2.0
  [L1MO_KOBS_XY]=15.0  [L1MO_KOBS_Z]=15.0
  [L1MO_KP_XY]=1.5     [L1MO_KP_Z]=3.0
  [L1MO_LPF_XY]=10.0   [L1MO_LPF_Z]=12.0
  [L1MO_MO_DTH]=0.08
  [L1MO_MO_EN]=1
  [L1MO_MO_KO]=0.5
  [L1MO_M_MAX]=5.0     [L1MO_M_MIN]=1.5     [L1MO_M_NOM]=2.064
  [L1MO_SIG_LIM]=4.0   [L1MO_SIG_MOD]=0.5
  [L1MO_THR_KL]=17.44  [L1MO_THR_KQ]=31.72
  [L1MO_TILT_MAX]=30.0
)

# Param yang bisa di-sweep (sesuai permintaanmu + 2 tambahan saran: SIG_LIM,
# SIG_MOD, MO_KO — MO_KO beda dari KOBS_XY/Z: MO_KO = gain observer momentum
# itu sendiri (di mo_update), KOBS_XY/Z = gain observer state predictor L1.
# Jangan tertukar saat menulis BAB IV.
SWEEPABLE=(L1MO_KP_XY L1MO_KD_XY L1MO_KP_Z L1MO_KD_Z
           L1MO_KOBS_XY L1MO_KOBS_Z L1MO_LPF_XY L1MO_LPF_Z
           L1MO_GAMMA_X L1MO_GAMMA_Y L1MO_GAMMA_Z
           L1MO_SIG_LIM L1MO_SIG_MOD L1MO_MO_KO)

# ═══════════════════════════════════════════════════════════════════════
# KILL / CLEAN
# ═══════════════════════════════════════════════════════════════════════
kill_all() {
  log "Kill semua proses SITL lama..."
  pkill -9 -f "px4"             2>/dev/null || true
  pkill -9 -f "gz sim"          2>/dev/null || true
  pkill -9 -f "MicroXRCEAgent"  2>/dev/null || true
  pkill -9 -f "test_hover_mass" 2>/dev/null || true
  pkill -9 -f "offboard_controller" 2>/dev/null || true
  pkill -9 -f "multi_drop"      2>/dev/null || true
  pkill -9 -f "pick_drop"       2>/dev/null || true
  sleep 2
}
clean_gz_cache() { rm -rf ~/.gz/sim/ ~/.gz/rendering/ /tmp/gz* 2>/dev/null || true; }

# ═══════════════════════════════════════════════════════════════════════
# EXTRAS.TXT — controller swap + baseline + override sweep, TANPA rc.mc_apps
# ═══════════════════════════════════════════════════════════════════════
_find_extras_path() {
  local build="${PX4_DIR}/build/px4_sitl_default"
  local extras_dir="${build}/tmp/rootfs/fs/microsd/etc"
  mkdir -p "$extras_dir" 2>/dev/null || true
  echo "${extras_dir}/extras.txt"
}

# write_extras <mo_en:0|1> [override_param] [override_val]
write_extras() {
  local mo_en="$1" ov_param="${2:-}" ov_val="${3:-}"
  local extras; extras=$(_find_extras_path)
  {
    echo "# sweep_test.sh — auto-generated $(date)"
    echo "mc_pos_control stop"
    echo ""
    for p in "${!BASELINE_PARAMS[@]}"; do
      echo "param set ${p} ${BASELINE_PARAMS[$p]}"
    done
    echo "param set L1MO_MO_EN ${mo_en}"
    if [[ -n "$ov_param" ]]; then
      echo ""
      echo "# ─── SWEEP OVERRIDE ───"
      echo "param set ${ov_param} ${ov_val}"
    fi
    echo ""
    echo "mc_l1_mo start"
  } > "$extras"
  ok "extras.txt ditulis: ${extras}"
}

# ═══════════════════════════════════════════════════════════════════════
# LAUNCH
# ═══════════════════════════════════════════════════════════════════════
launch_dds_agent() {
  MicroXRCEAgent udp4 -p 8888 > "${LOG_DIR}/dds_agent.log" 2>&1 &
  DDS_PID=$!; disown "$DDS_PID" 2>/dev/null || true; sleep 2
  kill -0 "$DDS_PID" 2>/dev/null && ok "DDS Agent PID: $DDS_PID" || die "DDS Agent gagal start"
}

launch_px4_sitl() {
  local world="$1" px4_log="${LOG_DIR}/px4_sitl.log"
  cd "$PX4_DIR" || die "PX4_DIR tidak ada: $PX4_DIR"
  if [[ "$world" == "default" ]]; then
    make px4_sitl gz_x500 > "$px4_log" 2>&1 &
  else
    PX4_GZ_WORLD="$world" make px4_sitl gz_x500 > "$px4_log" 2>&1 &
  fi
  PX4_MAKE_PID=$!
  log "  Menunggu pxh>..."
  local t=0
  while [[ $t -lt 90 ]]; do
    grep -q "pxh>" "$px4_log" 2>/dev/null && break
    sleep 2; t=$((t+2)); printf "  %ds...\r" "$t"
  done; echo ""
  [[ $t -ge 90 ]] && warn "pxh> belum muncul setelah 90s — cek ${px4_log}"
  sleep 5
}

verify_controller() {
  local px4_log="${LOG_DIR}/px4_sitl.log"
  sleep 3
  if grep -qiE "mc_l1_mo started" "$px4_log" 2>/dev/null; then
    ok "mc_l1_mo VERIFIED jalan"
  else
    warn "mc_l1_mo TIDAK terdeteksi di log — cek ${px4_log} (extras.txt mungkin tidak terbaca, lihat ARAHAN #1)"
  fi
}

prepare_payload_symlink() {
  # Hanya untuk single_drop. multi_drop & pickup payloadnya fixed di world.
  local kg="$1" model_name="${PAYLOAD_MODEL[$kg]:-}"
  [[ -z "$model_name" ]] && { warn "payload ${kg}kg tidak dikenal, skip symlink"; return; }
  local model_base="${PX4_DIR}/Tools/simulation/gz/models"
  [[ -d "${model_base}/${model_name}" ]] || { warn "model ${model_name} tidak ditemukan di ${model_base}"; return; }
  local dst="${model_base}/payload_box"
  [[ -L "$dst" ]] && rm "$dst"
  ln -s "${model_base}/${model_name}" "$dst"
  ok "payload symlink -> ${model_name}"
}

launch_scenario() {
  local scenario="$1" tag="$2"
  local exe="${SCENARIO_EXE[$scenario]}"
  local ros2_log="${LOG_DIR}/${tag}_ros2.log"
  (
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source "${DRONE_WS}/install/setup.bash" 2>/dev/null || true
    exec ros2 run "$ROS2_PKG" "$exe"
  ) > "$ros2_log" 2>&1 &
  OFFBOARD_PID=$!
  ok "Scenario node (${exe}) PID: $OFFBOARD_PID"
}

wait_offboard() {
  local scenario="$1" timeout_sec="${SCENARIO_TIMEOUT[$scenario]:-120}"
  log "Wait offboard selesai (maks ${timeout_sec}s)..."
  local w=0
  while [[ $w -lt $timeout_sec ]]; do
    kill -0 "${OFFBOARD_PID:-0}" 2>/dev/null || { ok "Node exit normal (${w}s)"; break; }
    sleep 2; w=$((w+2))
  done
  kill -TERM "${OFFBOARD_PID:-0}" 2>/dev/null || true; sleep 2
  kill -9    "${OFFBOARD_PID:-0}" 2>/dev/null || true
}

# ═══════════════════════════════════════════════════════════════════════
# ULOG: cari file terbaru, pindahkan ke out/<scenario>/<tag>/
# ═══════════════════════════════════════════════════════════════════════
collect_ulog() {
  local scenario="$1" tag="$2"
  local dest_dir="${OUT_ROOT}/${scenario}/${tag}"
  mkdir -p "$dest_dir"
  local newest
  newest=$(find "${PX4_DIR}/build/px4_sitl_default" -name "*.ulg" -printf '%T@ %p\n' 2>/dev/null \
            | sort -rn | head -1 | cut -d' ' -f2-)
  if [[ -z "$newest" ]]; then
    warn "Tidak ada .ulg ditemukan untuk ${tag} — cek apakah SDLOG_MODE aktif"
    return 1
  fi
  cp "$newest" "${dest_dir}/${tag}.ulg"
  ok "ulog -> ${dest_dir}/${tag}.ulg"
  echo "${dest_dir}/${tag}.ulg"
}

# [VERIFIKASI ARAHAN #3] — sesuaikan argumen dengan 1_analyze_flight_logs.py
# yang sebenarnya. Bentuk di bawah ini ASUMSI berdasar catatan: dia terima
# --tag untuk label sweep, dan mengorganisir sendiri ke out/<scenario>/<tag>/
# serta append ke MASTER.csv.
run_analysis() {
  local scenario="$1" tag="$2" ulog_path="$3"
  local analyze_script="${SCRIPT_DIR}/1_analyze_flight_logs.py"
  if [[ ! -f "$analyze_script" ]]; then
    warn "1_analyze_flight_logs.py tidak ditemukan di ${SCRIPT_DIR} — skip analysis, ulog disimpan saja"
    return
  fi
  python3 "$analyze_script" \
    --ulog "$ulog_path" \
    --scenario "$scenario" \
    --tag "$tag" \
    --out "$OUT_ROOT" \
    && ok "Analysis selesai: ${tag}" \
    || warn "Analysis gagal untuk ${tag} — cek argumen CLI (ARAHAN #3)"
}

# ═══════════════════════════════════════════════════════════════════════
# SATU RUN PENUH
# ═══════════════════════════════════════════════════════════════════════
run_one() {
  local scenario="$1" mo_en="$2" tag="$3"
  local ov_param="${4:-}" ov_val="${5:-}" payload_kg="${6:-}"

  echo ""; sep; log "RUN: ${BOLD}${tag}${NC}"; sep
  mkdir -p "$LOG_DIR"

  kill_all; clean_gz_cache
  write_extras "$mo_en" "$ov_param" "$ov_val"

  [[ "$scenario" == "single_drop" && -n "$payload_kg" ]] && prepare_payload_symlink "$payload_kg"

  launch_dds_agent
  launch_px4_sitl "${SCENARIO_WORLD[$scenario]}"
  verify_controller

  launch_scenario "$scenario" "$tag"
  wait_offboard "$scenario"

  local ulog_path
  ulog_path=$(collect_ulog "$scenario" "$tag") || true
  [[ -n "$ulog_path" ]] && run_analysis "$scenario" "$tag" "$ulog_path"

  pkill -TERM -f "gz sim" 2>/dev/null || true
  pkill -TERM -f "px4"    2>/dev/null || true
  pkill -TERM -f "MicroXRCEAgent" 2>/dev/null || true
  sleep 3
  ok "Run selesai: ${tag}"
}

# ═══════════════════════════════════════════════════════════════════════
# MENU INTERAKTIF
# ═══════════════════════════════════════════════════════════════════════
echo -e "\n${BOLD}╔════════════════════════════════════════╗"
echo -e "║   SITL Campaign — hover/drop/pickup     ║"
echo -e "╚════════════════════════════════════════╝${NC}\n"

echo -e "${BLU}[1] Skenario:${NC}"
echo "    1) hover         — baseline tuning, tanpa payload"
echo "    2) single_drop   — pre-attached payload, drop saat hover"
echo "    3) multi_drop    — 2x0.5kg sequential delivery"
echo "    4) pickup        — pickup dari station + delivery"
read -rp "Pilih [1-4]: " sc_in
case "$sc_in" in
  1) SCENARIO=hover ;; 2) SCENARIO=single_drop ;;
  3) SCENARIO=multi_drop ;; 4) SCENARIO=pickup ;;
  *) die "Pilihan tidak valid" ;;
esac

PAYLOAD_KG=""
if [[ "$SCENARIO" == "single_drop" ]]; then
  echo -e "\n${BLU}[1b] Payload (kg):${NC}  1) 0.3  2) 0.6  3) 0.8  4) 1.0  5) 1.2"
  read -rp "Pilih [1-5]: " pl
  case "$pl" in
    1) PAYLOAD_KG=0.3 ;; 2) PAYLOAD_KG=0.6 ;; 3) PAYLOAD_KG=0.8 ;;
    4) PAYLOAD_KG=1.0 ;; 5) PAYLOAD_KG=1.2 ;; *) die "Pilihan tidak valid" ;;
  esac
fi

echo -e "\n${BLU}[2] Momentum Observer:${NC}  1) OFF (L1-only baseline)   2) ON"
read -rp "Pilih [1-2]: " mo_in
case "$mo_in" in
  1) MO_EN=0 ;; 2) MO_EN=1 ;; *) die "Pilihan tidak valid" ;;
esac

echo -e "\n${BLU}[3] Mode:${NC}  1) sweep   2) normal"
read -rp "Pilih [1-2]: " mode_in

if [[ "$mode_in" == "1" ]]; then
  MODE="sweep"
  echo -e "\n${BLU}[3a] Parameter yang di-sweep:${NC}"
  for i in "${!SWEEPABLE[@]}"; do
    printf "    %2d) %-14s (baseline=%s)\n" "$((i+1))" "${SWEEPABLE[$i]}" "${BASELINE_PARAMS[${SWEEPABLE[$i]}]}"
  done
  read -rp "Pilih nomor [1-${#SWEEPABLE[@]}]: " pidx
  SWEEP_PARAM="${SWEEPABLE[$((pidx-1))]}"
  [[ -z "$SWEEP_PARAM" ]] && die "Pilihan parameter tidak valid"
  read -rp "Nilai yang dicoba (pisah spasi, contoh: 1.0 1.5 2.0 2.5): " -a SWEEP_VALUES
  [[ ${#SWEEP_VALUES[@]} -eq 0 ]] && die "Tidak ada nilai sweep"
else
  MODE="normal"
  read -rp "Jumlah run [default 10]: " N_RUNS
  N_RUNS="${N_RUNS:-10}"
fi

sep
log "Ringkasan: scenario=${SCENARIO} MO_EN=${MO_EN} mode=${MODE}"
[[ "$MODE" == "sweep" ]] && log "  sweep param=${SWEEP_PARAM} values=(${SWEEP_VALUES[*]})"
[[ "$MODE" == "normal" ]] && log "  N runs=${N_RUNS} (baseline lock, tanpa override)"
read -rp "Lanjut? [y/N]: " confirm
[[ "$confirm" =~ ^[Yy]$ ]] || die "Dibatalkan"

mkdir -p "$LOG_DIR" "$OUT_ROOT"

if [[ "$MODE" == "sweep" ]]; then
  for val in "${SWEEP_VALUES[@]}"; do
    tag="${SCENARIO}_MO${MO_EN}_${SWEEP_PARAM}_${val//./_}"
    run_one "$SCENARIO" "$MO_EN" "$tag" "$SWEEP_PARAM" "$val" "$PAYLOAD_KG"
  done
else
  for ((r=1; r<=N_RUNS; r++)); do
    run_id=$(printf '%02d' "$r")
    tag="${SCENARIO}_MO${MO_EN}_run${run_id}"
    run_one "$SCENARIO" "$MO_EN" "$tag" "" "" "$PAYLOAD_KG"
  done
fi

sep; ok "Campaign selesai. Log mentah: ${LOG_DIR}"
ok "Hasil terorganisir: ${OUT_ROOT}/${SCENARIO}/"
