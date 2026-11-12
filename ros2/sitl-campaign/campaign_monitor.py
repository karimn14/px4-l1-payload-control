#!/usr/bin/env bash
# run_campaign.sh — SITL Campaign Launcher
# PX4 commands via screen/tmux session (PTY proper, tidak ada deadlock)
# Tidak perlu terminal terpisah untuk pxh>

PX4_DIR="${PX4_DIR:-$HOME/PX4-main}"
DRONE_WS="${DRONE_WS:-$HOME/drone_ws}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${SCRIPT_DIR}/logs/${SESSION_ID}"
RESULTS_CSV="${LOG_DIR}/metrics_summary.csv"

ARG_SCENARIO=""; ARG_CONTROLLER=""; ARG_PAYLOAD=""
ARG_RUNS=10; ARG_NO_WIND=false; ARG_DRY_RUN=false
ARG_MOTOR_FACTOR="1.0"; ARG_SKIP_QGC=false
ARG_SWEEP_PARAM=""; ARG_SWEEP_VALUES=""

RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[0;33m'
BLU='\033[0;34m'; CYN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'
log()  { echo -e "${CYN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GRN}[OK]${NC} $*"; }
warn() { echo -e "${YLW}[WARN]${NC} $*" >&2; }
die()  { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
sep()  { echo -e "${BOLD}──────────────────────────────────────────────${NC}"; }

show_help() {
cat <<'EOF'
run_campaign.sh — SITL Campaign Launcher
Scenarios : hover|drop|dynamic_drop|multi_drop|pick_drop|motor_fault|sweep|all
Controller: l1|pid|both   Payload: 0.3|0.6|0.8|1.0|1.2 kg

  -s scenario  -c controller  -p payload  -n runs
  --no-wind  --motor-fault FACTOR  --skip-qgc  --dry-run
  --sweep-param PARAM  --sweep-values "v1 v2 v3"

Contoh:
  ./run_campaign.sh -s drop -c l1 -p 0.6 -n 10
  ./run_campaign.sh -s sweep -c l1 -p 0.6 --sweep-param GAMMA_XY --sweep-values "60 100 150 200" -n 3
  ./run_campaign.sh -s all -c both -n 10
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -s|--scenario)   ARG_SCENARIO="$2";    shift 2 ;;
    -c|--controller) ARG_CONTROLLER="$2";  shift 2 ;;
    -p|--payload)    ARG_PAYLOAD="$2";     shift 2 ;;
    -n|--runs)       ARG_RUNS="$2";        shift 2 ;;
    --no-wind)       ARG_NO_WIND=true;     shift ;;
    --motor-fault)   ARG_MOTOR_FACTOR="$2"; shift 2 ;;
    --skip-qgc)      ARG_SKIP_QGC=true;    shift ;;
    --dry-run)       ARG_DRY_RUN=true;     shift ;;
    --sweep-param)   ARG_SWEEP_PARAM="$2"; shift 2 ;;
    --sweep-values)  ARG_SWEEP_VALUES="$2"; shift 2 ;;
    -h|--help)       show_help; exit 0 ;;
    *) die "Argumen tidak dikenal: $1" ;;
  esac
done

check_deps() {
  [[ -d "$PX4_DIR"  ]] || die "PX4_DIR tidak ada: $PX4_DIR"
  [[ -d "$DRONE_WS" ]] || die "DRONE_WS tidak ada: $DRONE_WS"
  command -v MicroXRCEAgent &>/dev/null || die "MicroXRCEAgent tidak ditemukan"
  command -v ros2   &>/dev/null || die "ros2 tidak ditemukan"
  command -v gz     &>/dev/null || die "gz tidak ditemukan"
  command -v python3 &>/dev/null || die "python3 tidak ditemukan"
  [[ -f "${PX4_DIR}/l1_monitor.py" ]] || warn "l1_monitor.py tidak ada di ${PX4_DIR}"
  ok "Dependencies OK"
}

# ═══════════════════════════════════════════════════════════════════════════════
# PX4 COMMAND VIA SCREEN/TMUX SESSION
# screen -X stuff → kirim keystrokes ke PTY → pxh> terima command
# Screen/tmux buat PTY proper: Gazebo dapat environment yang benar,
# tidak ada deadlock (mkfifo exec 99> memblokir sampai ada reader).
# ═══════════════════════════════════════════════════════════════════════════════
_pipe_close() { true; }  # no-op, ada untuk kompatibilitas

_px4_cmd() {
  local cmd="$1"
  if command -v screen &>/dev/null && screen -list 2>/dev/null | grep -q "px4_sitl"; then
    screen -S px4_sitl -X stuff "$(printf '%s\r' "$cmd")"
    return 0
  fi
  if command -v tmux &>/dev/null && tmux list-sessions 2>/dev/null | grep -q "px4_sitl"; then
    tmux send-keys -t px4_sitl "$cmd" Enter
    return 0
  fi
  warn "_px4_cmd: session px4_sitl tidak ada untuk: ${cmd}"
  return 1
}

# ═══════════════════════════════════════════════════════════════════════════════
# KILL / CLEAN
# ═══════════════════════════════════════════════════════════════════════════════
kill_all() {
  log "Kill semua proses SITL..."
  pkill -9 -f "px4"                 2>/dev/null || true
  pkill -9 -f "gz sim"              2>/dev/null || true
  pkill -9 -f "MicroXRCEAgent"      2>/dev/null || true
  pkill -9 -f "l1_monitor"          2>/dev/null || true
  pkill -9 -f "offboard_controller" 2>/dev/null || true
  pkill -9 -f "test_hover_mass"     2>/dev/null || true
  pkill -9 -f "multi_drop"          2>/dev/null || true
  pkill -9 -f "fault"               2>/dev/null || true
  pkill -9 -f "pick_drop_runner"    2>/dev/null || true
  pkill -9 -f "ruby"                2>/dev/null || true
  sleep 2; ok "Kill done"
}

clean_gz_cache() {
  rm -rf ~/.gz/sim/ ~/.gz/rendering/ /tmp/gz* 2>/dev/null || true
  log "GZ cache cleaned"
}

# ═══════════════════════════════════════════════════════════════════════════════
# PAYLOAD MODEL MANAGEMENT
# ═══════════════════════════════════════════════════════════════════════════════
payload_model_name() {
  case "$1" in
    0.3) echo "payload_box_03kg"  ;; 0.6) echo "payload_box_06kg" ;;
    0.8) echo "payload_box_08kg"  ;; 1.0) echo "payload_box_1kg"  ;;
    1.2) echo "payload_box_1_2kg" ;;
    *) warn "payload_model_name: ${1}kg tidak dikenal"; echo "payload_box_06kg" ;;
  esac
}

_find_model_base() {
  local model_name="$1"; local _paths=()
  if [[ -n "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
    local IFS_O="$IFS"; IFS=":"; read -ra _paths <<< "$GZ_SIM_RESOURCE_PATH"; IFS="$IFS_O"
  fi
  _paths+=("${PX4_DIR}/Tools/simulation/gz/models" "$HOME/px4_mpc_leso_project/models" "$HOME/.gz/sim/models")
  local seen=""
  for p in "${_paths[@]}"; do
    [[ -z "$p" || "$seen" == *"|${p}|"* ]] && continue
    seen="${seen}|${p}|"
    [[ -d "${p}/${model_name}" ]] && { echo "$p"; return; }
  done
  echo ""
}

# pick_drop: buat symlink payload_box → payload_box_XXkg SEBELUM PX4 launch
prepare_payload_symlink() {
  local kg="$1"; local model_name; model_name=$(payload_model_name "$kg")
  local model_base; model_base=$(_find_model_base "$model_name")
  [[ -z "$model_base" ]] && die "Model ${model_name} tidak ditemukan"
  local src="${model_base}/${model_name}" dst="${model_base}/payload_box"
  log "Symlink: payload_box → ${model_name}"
  [[ -L "$dst" ]] && rm "$dst"
  [[ -d "$dst" ]] && mv "$dst" "${dst}.bak_$(date +%H%M%S)"
  ln -s "$src" "$dst"
  ok "Symlink dibuat"
  export CURRENT_PAYLOAD_MODEL="$model_name"
}

cleanup_payload_symlink() {
  local _paths=()
  if [[ -n "${GZ_SIM_RESOURCE_PATH:-}" ]]; then
    local IFS_O="$IFS"; IFS=":"; read -ra _paths <<< "$GZ_SIM_RESOURCE_PATH"; IFS="$IFS_O"
  fi
  _paths+=("${PX4_DIR}/Tools/simulation/gz/models" "$HOME/px4_mpc_leso_project/models")
  for p in "${_paths[@]}"; do
    [[ -n "$p" && -L "${p}/payload_box" ]] && { rm "${p}/payload_box"; log "Symlink dibersihkan"; return; }
  done
}

# Spawn payload di world + attach ke drone (semua scenario kecuali multi_drop, pick_drop)
spawn_and_attach_payload() {
  local kg="$1" scenario="$2"
  [[ "$scenario" == "multi_drop" || "$scenario" == "pick_drop" ]] && return

  local model_name; model_name=$(payload_model_name "$kg")
  local model_base; model_base=$(_find_model_base "$model_name")
  if [[ -z "$model_base" ]]; then
    warn "Model ${model_name} tidak ditemukan — skip spawn"; return 1
  fi
  local model_sdf="${model_base}/${model_name}/model.sdf"

  log "Spawn + attach ${model_name} (${kg}kg)..."
  $ARG_DRY_RUN && { log "  [DRY] spawn + attach"; return; }

  # Tunggu gz service
  local t=0
  until gz service -l 2>/dev/null | grep -q "create" || [[ $t -gt 30 ]]; do
    sleep 2; t=$((t+2)); printf "  gz service... %ds\r" "$t"
  done; echo ""

  # Spawn via sdf_filename
  local result
  result=$(gz service -s /world/default/create \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
    --req "sdf_filename: '${model_sdf}', name: 'payload_box', \
pose: {position: {x:0, y:0, z:0.2}}" 2>&1)

  if ! echo "$result" | grep -q "data: true"; then
    # Fallback: SDF string
    local sdf_str
    sdf_str=$(python3 -c "
with open('${model_sdf}') as f:
    print(f.read().replace(chr(10),' ').strip())" 2>/dev/null)
    [[ -n "$sdf_str" ]] && result=$(gz service -s /world/default/create \
      --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
      --req "sdf: '${sdf_str}', name: 'payload_box', \
pose: {position: {x:0, y:0, z:0.2}}" 2>&1)
  fi

  if echo "$result" | grep -q "data: true"; then
    ok "Payload spawned: ${model_name}"
    sleep 2
    gz topic -t /payload_attach -m gz.msgs.Empty -p "" 2>/dev/null \
      && ok "Payload attached" || warn "Attach topic gagal"
    sleep 1
  else
    warn "Spawn gagal: $(echo "$result" | head -1)"
    warn "  SDF: ${model_sdf}"
  fi
}

# ═══════════════════════════════════════════════════════════════════════════════
# LAUNCH FUNCTIONS
# ═══════════════════════════════════════════════════════════════════════════════
launch_dds_agent() {
  log "STEP 1: DDS Agent..."
  $ARG_DRY_RUN && return
  MicroXRCEAgent udp4 -p 8888 > "${LOG_DIR}/dds_agent.log" 2>&1 &
  DDS_PID=$!; disown "$DDS_PID" 2>/dev/null || true; sleep 2
  kill -0 "$DDS_PID" 2>/dev/null && ok "DDS Agent PID: $DDS_PID" \
    || die "DDS Agent gagal"
}

launch_qgc() {
  $ARG_SKIP_QGC && return
  local p
  for p in "$HOME/Downloads/QGroundControl.AppImage" "$HOME/QGroundControl.AppImage"; do
    if [[ -f "$p" ]]; then $ARG_DRY_RUN || { "$p" > "${LOG_DIR}/qgc.log" 2>&1 & disown $!; sleep 2; }; return; fi
  done
  warn "QGC tidak ditemukan"
}

# ─── PREPARE EXTRAS.TXT ───────────────────────────────────────────────────────
# PX4 SITL membaca extras.txt dari SD card path saat boot (via rcS).
# Tulis controller switch + sweep params ke sini SEBELUM make px4_sitl.
# Tidak perlu stdin tricks, screen, atau pipe — PALING RELIABLE.
_find_extras_path() {
  # PX4 rcS: set FEXTRAS /fs/microsd/etc/extras.txt
  # → path absolut: build/px4_sitl_default/tmp/rootfs/fs/microsd/etc/extras.txt
  local build="${PX4_DIR}/build/px4_sitl_default"
  local extras_dir="${build}/tmp/rootfs/fs/microsd/etc"
  mkdir -p "$extras_dir" 2>/dev/null || true
  echo "${extras_dir}/extras.txt"
}

prepare_extras() {
  local controller="$1" sweep_param="${2:-}" sweep_val="${3:-}"
  local extras; extras=$(_find_extras_path)
  log "extras.txt: ${extras}"

  if [[ "$controller" == "l1" ]]; then
    {
      echo "# run_campaign.sh — $(date)"
      echo "mc_pos_control stop"
      if [[ -n "$sweep_param" && -n "$sweep_val" ]]; then
        case "$sweep_param" in
          GAMMA_XY)
            echo "param set L1MO_GAMMA_X ${sweep_val}"
            echo "param set L1MO_GAMMA_Y ${sweep_val}" ;;
          *)
            echo "param set L1MO_${sweep_param} ${sweep_val}" ;;
        esac
      fi
      echo "mc_l1_mo start"
    } > "$extras"
    ok "extras.txt → mc_l1_mo akan auto-start:"
    grep -v "^#" "$extras" | sed 's/^/  /'
  else
    rm -f "$extras" 2>/dev/null || true
    ok "extras.txt removed → PID default"
  fi
  export CURRENT_EXTRAS_PATH="$extras"
}

launch_px4_sitl() {
  local world="$1" px4_log="${LOG_DIR}/px4_sitl.log"
  log "STEP 3: PX4 SITL (world=${world})..."
  $ARG_DRY_RUN && return

  cd "$PX4_DIR"
  # Launch PERSIS seperti manual — tidak ada stdin redirect, screen, atau pipe
  # extras.txt sudah ditulis di prepare_extras → mc_l1_mo auto-start saat boot
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
  [[ $t -ge 90 ]] && warn "pxh> belum muncul setelah 90s"
  sleep 5; ok "PX4 + Gazebo ready"
}


switch_controller() {
  # Switching sudah terjadi via extras.txt saat PX4 boot.
  # Fungsi ini hanya verifikasi: grep log untuk konfirmasi.
  local controller="$1"
  local px4_log="${LOG_DIR}/px4_sitl.log"
  log "STEP 4: Verifikasi controller (${controller})..."
  $ARG_DRY_RUN && return
  sleep 3

  if [[ "$controller" == "l1" ]]; then
    if grep -qiE "mc_l1_mo|l1_mo" "$px4_log" 2>/dev/null; then
      ok "mc_l1_mo VERIFIED di log"
    else
      warn "mc_l1_mo belum terdeteksi di log."
      warn "  Cek: grep -i l1 ${px4_log}"
      warn "  extras.txt: ${CURRENT_EXTRAS_PATH:-tidak diketahui}"
      warn "  Kemungkinan: extras.txt path tidak diload PX4, atau module tidak ter-build"
    fi
  else
    ok "PID (mc_pos_control default)"
  fi
}

launch_monitor() {
  local tag="$1"
  local monitor_out="${LOG_DIR}/${tag}_l1monitor.csv"
  local monitor_err="${LOG_DIR}/${tag}_monitor_stderr.log"
  MONITOR_PID=""; MONITOR_TAG="$tag"
  MONITOR_CSV="$monitor_out"  # path pasti — tidak perlu guess atau scan

  log "STEP 5: campaign_monitor.py → $(basename ${monitor_out})"
  $ARG_DRY_RUN && return
  : > "$monitor_err"

  (
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source "${DRONE_WS}/install/setup.bash" 2>/dev/null || true
    exec python3 "${SCRIPT_DIR}/campaign_monitor.py" --out "${monitor_out}"
  ) >> "$monitor_err" 2>&1 &
  MONITOR_PID=$!

  sleep 3
  if kill -0 "$MONITOR_PID" 2>/dev/null; then
    ok "Monitor PID: $MONITOR_PID"
  else
    warn "Monitor exit langsung — stderr:"
    sed 's/^/    /' "$monitor_err" | head -15 >&2
    warn "  Cek: ${monitor_err}"
  fi
}


# HANYA path atau kosong ke stdout — warn ke stderr
find_monitor_csv() {
  local tag="$1" explicit="${LOG_DIR}/${tag}_l1monitor.csv"
  [[ -s "$explicit" ]] && { echo "$explicit"; return; }
  local newest
  newest=$(ls -t "${LOG_DIR}"/l1_monitor_*.csv 2>/dev/null | head -1)
  [[ -n "$newest" && -s "$newest" ]] && { cp "$newest" "$explicit"; echo "$explicit"; return; }
  for d in "$HOME" "$PX4_DIR" "$SCRIPT_DIR"; do
    newest=$(ls -t "${d}"/l1_monitor_*.csv 2>/dev/null | head -1)
    [[ -n "$newest" && -s "$newest" ]] && {
      warn "Monitor CSV di ${d}, copy ke log dir" >&2
      cp "$newest" "$explicit"; echo "$explicit"; return
    }
  done
  echo ""
}

launch_offboard() {
  local scenario="$1" controller="$2" payload="$3" run_id="$4" wind_tag="$5"
  local motor_factor="$6" sweep_param="${7:-}" sweep_val="${8:-}"
  local tag="${controller}_${scenario}_${payload}kg_${wind_tag}_run${run_id}"
  [[ -n "$sweep_param" ]] && tag="${tag}_L1MO_${sweep_param}_${sweep_val//./_}"
  local ros2_log="${LOG_DIR}/${tag}_ros2.log"; OFFBOARD_LOG="$ros2_log"
  log "STEP 6: Offboard: ${tag}"
  $ARG_DRY_RUN && return
  (
    source /opt/ros/humble/setup.bash
    source "${DRONE_WS}/install/setup.bash"
    export SITL_SCENARIO="$scenario" SITL_CONTROLLER="$controller"
    export SITL_PAYLOAD_KG="$payload" SITL_NO_WIND="$ARG_NO_WIND"
    export SITL_MOTOR_FACTOR="$motor_factor" SITL_RUN_ID="$run_id"
    export SITL_TAG="$tag" SITL_LOG_DIR="$LOG_DIR"
    export SITL_DYNAMIC_DROP="false"
    [[ "$scenario" == "dynamic_drop" ]] && export SITL_DYNAMIC_DROP="true"
    case "$scenario" in
      hover|sweep)   exec ros2 run mpc_leso_controller test_hover_mass ;;
      drop|dynamic_drop) exec ros2 run mpc_leso_controller offboard_controller ;;
      multi_drop)    exec ros2 run mpc_leso_controller multi_drop ;;
      motor_fault)   exec ros2 run mpc_leso_controller fault ;;
      pick_drop)     exec python3 "${SCRIPT_DIR}/pick_drop_runner.py" \
                       --payload "$payload" --run-id "$run_id" \
                       --log-dir "$LOG_DIR" --tag "$tag" ;;
      *)             exec python3 "${SCRIPT_DIR}/offboard_all_scenarios.py" \
                       --scenario "$scenario" --payload "$payload" \
                       --run-id "$run_id" --log-dir "$LOG_DIR" --tag "$tag" ;;
    esac
  ) > "$ros2_log" 2>&1 &
  OFFBOARD_PID=$!; ok "Offboard PID: $OFFBOARD_PID"
}

wait_offboard() {
  local scenario="$1" timeout_sec
  case "$scenario" in
    hover|sweep) timeout_sec=90  ;; drop)         timeout_sec=120 ;;
    multi_drop)  timeout_sec=180 ;; dynamic_drop) timeout_sec=150 ;;
    motor_fault) timeout_sec=120 ;; pick_drop)    timeout_sec=300 ;;
    *)           timeout_sec=120 ;;
  esac
  log "Wait offboard max ${timeout_sec}s..."
  if [[ -f "${SCRIPT_DIR}/_landing_watchdog.py" ]]; then
    ( source /opt/ros/humble/setup.bash 2>/dev/null || true
      source "${DRONE_WS}/install/setup.bash" 2>/dev/null || true
      python3 "${SCRIPT_DIR}/_landing_watchdog.py" --timeout "$timeout_sec" --grace 15
    ) > "${LOG_DIR}/${MONITOR_TAG:-run}_watchdog.log" 2>&1
    local rc=$?
    case $rc in
      0) ok "Watchdog: landed" ;;
      2) warn "Watchdog: timeout" ;;
      3) warn "Watchdog ROS2 error — tunggu sederhana"
         local w=0; while [[ $w -lt $timeout_sec ]]; do
           kill -0 "${OFFBOARD_PID:-0}" 2>/dev/null || break; sleep 2; w=$((w+2)); done ;;
    esac
  else
    local w=0; while [[ $w -lt $timeout_sec ]]; do
      kill -0 "${OFFBOARD_PID:-0}" 2>/dev/null || { ok "Offboard exit (${w}s)"; break; }
      sleep 2; w=$((w+2)); done
  fi
  kill -TERM "${OFFBOARD_PID:-0}" 2>/dev/null || true; sleep 2
  kill -9    "${OFFBOARD_PID:-0}" 2>/dev/null || true
  pkill -TERM -f "test_hover_mass"     2>/dev/null || true
  pkill -TERM -f "offboard_controller" 2>/dev/null || true
  pkill -TERM -f "multi_drop"          2>/dev/null || true; sleep 1
}

trigger_motor_fault() {
  gz topic -t /motor_failure/throttle_factor -m gz.msgs.Float -p "data: $1" 2>/dev/null || true; sleep 0.5
  gz topic -t /model/x500_0/motor_failure/motor_number -m gz.msgs.Int32 -p "data: ${2:-1}" 2>/dev/null || true
  log "Motor fault: motor=${2:-1} factor=$1"
}
clear_motor_fault() {
  gz topic -t /model/x500_0/motor_failure/motor_number -m gz.msgs.Int32 -p "data: 0" 2>/dev/null || true
}

extract_run_metrics() {
  local tag="$1" scenario="$2" controller="$3" payload="$4" wind_tag="$5"
  local monitor_csv; monitor_csv=$(find_monitor_csv "$tag")
  if [[ -z "$monitor_csv" ]]; then
    warn "Monitor CSV tidak ditemukan — ${tag}"; warn "  Cek: ${LOG_DIR}/${tag}_monitor_stderr.log"; return
  fi
  local lc; lc=$(wc -l < "$monitor_csv" 2>/dev/null || echo 0)
  if [[ "$lc" -le 1 ]]; then
    warn "Monitor CSV kosong (${lc} baris): ${monitor_csv}"; return
  fi
  log "Extract metrics: ${monitor_csv} (${lc} baris)"
  python3 "${SCRIPT_DIR}/extract_metrics.py" \
    --csv "$monitor_csv" --tag "$tag" --scenario "$scenario" \
    --controller "$controller" --payload "$payload" --wind "$wind_tag" \
    --out "$RESULTS_CSV" && ok "Metrics → ${RESULTS_CSV}" || warn "extract_metrics gagal"
}

# ═══════════════════════════════════════════════════════════════════════════════
# SINGLE RUN
# ═══════════════════════════════════════════════════════════════════════════════
run_single() {
  local scenario="$1" controller="$2" payload="$3"
  local motor_factor="$4" run_num="$5" sweep_param="${6:-}" sweep_val="${7:-}"
  local wind_tag; $ARG_NO_WIND && wind_tag="nowind" || wind_tag="wind"
  local run_id; run_id="$(printf '%02d' "$run_num")"
  local tag="${controller}_${scenario}_${payload}kg_${wind_tag}_run${run_id}"
  [[ -n "$sweep_param" ]] && tag="${tag}_L1MO_${sweep_param}_${sweep_val//./_}"

  echo ""; sep; log "RUN ${run_id}/${ARG_RUNS}: ${BOLD}${tag}${NC}"; sep
  mkdir -p "$LOG_DIR"

  kill_all; clean_gz_cache

  # STEP 0: Tulis extras.txt SEBELUM PX4 launch
  prepare_extras "$controller" "$sweep_param" "$sweep_val"

  # Symlink payload_box untuk pick_drop (SEBELUM PX4)
  [[ "$scenario" == "pick_drop" ]] && prepare_payload_symlink "$payload" || true

  # DDS → QGC → PX4 SITL
  launch_dds_agent; launch_qgc
  local world="default"
  [[ "$scenario" == "multi_drop" ]] && world="multi_drop"
  [[ "$scenario" == "pick_drop"  ]] && world="pickup"
  launch_px4_sitl "$world"

  # Spawn + attach payload (semua kecuali multi_drop, pick_drop)
  spawn_and_attach_payload "$payload" "$scenario"

  # Switch controller (+ set sweep param jika ada)
  switch_controller "$controller" "$sweep_param" "$sweep_val"

  # Monitor → Offboard
  launch_monitor "$tag"
  launch_offboard "$scenario" "$controller" "$payload" \
    "$run_id" "$wind_tag" "$motor_factor" "$sweep_param" "$sweep_val"

  # Motor fault
  if [[ "$scenario" == "motor_fault" && "$motor_factor" != "1.0" ]]; then
    log "Tunggu 22s hover stabil..."; sleep 22
    trigger_motor_fault "$motor_factor" 1
  fi

  wait_offboard "$scenario"

  # Cleanup
  [[ "$scenario" == "motor_fault" ]] && clear_motor_fault   || true
  [[ "$scenario" == "pick_drop"   ]] && cleanup_payload_symlink || true
  log "Stop monitor..."; sleep 3
  [[ -n "${MONITOR_PID:-}" ]] && kill "$MONITOR_PID" 2>/dev/null || true
  sleep 2

  extract_run_metrics "$tag" "$scenario" "$controller" "$payload" "$wind_tag"

  log "Cleanup end of run..."
  pkill -TERM -f "gz sim" 2>/dev/null || true
  pkill -TERM -f "px4"    2>/dev/null || true
  pkill -TERM -f "MicroXRCEAgent" 2>/dev/null || true
  sleep 3; pkill -9 -f "gz sim" 2>/dev/null || true; pkill -9 -f "px4" 2>/dev/null || true
  sleep 1
  ok "Run ${run_id} selesai: ${tag}"
}

# ═══════════════════════════════════════════════════════════════════════════════
# INTERACTIVE MENU
# ═══════════════════════════════════════════════════════════════════════════════
interactive_menu() {
  echo -e "\n${BOLD}╔══════════════════════════════════════════╗"
  echo -e "║      SITL Campaign Launcher              ║"
  echo -e "╚══════════════════════════════════════════╝${NC}\n"

  echo -e "${BLU}[1] Scenario:${NC}"
  echo "    1) hover         — hover + payload attached"
  echo "    2) drop          — single drop (payload start attached → drop)"
  echo "    3) multi_drop    — dual drop 2×0.6kg payload_med (fixed)"
  echo "    4) dynamic_drop  — drop saat translasi"
  echo "    5) motor_fault   — degradasi motor"
  echo "    6) pick_drop     — pickup dari station + delivery"
  echo "    7) sweep         — parameter sweep L1MO (hover-based)"
  echo "    8) all           — full campaign matrix"
  read -rp "Pilih [1-8]: " sc
  case "$sc" in
    1) ARG_SCENARIO=hover ;;       2) ARG_SCENARIO=drop ;;
    3) ARG_SCENARIO=multi_drop ;;  4) ARG_SCENARIO=dynamic_drop ;;
    5) ARG_SCENARIO=motor_fault ;; 6) ARG_SCENARIO=pick_drop ;;
    7) ARG_SCENARIO=sweep ;;       8) ARG_SCENARIO=all ;;
    *) die "Pilihan tidak valid" ;;
  esac

  echo -e "\n${BLU}[2] Controller:${NC}"
  echo "    1) l1   2) pid   3) both"
  read -rp "Pilih [1-3]: " ct
  case "$ct" in
    1) ARG_CONTROLLER=l1 ;; 2) ARG_CONTROLLER=pid ;; 3) ARG_CONTROLLER=both ;;
    *) die "Pilihan tidak valid" ;;
  esac

  echo -e "\n${BLU}[3] Payload (kg):${NC}"
  if [[ "$ARG_SCENARIO" == "multi_drop" ]]; then
    echo "    → multi_drop: fixed 0.6+0.6 payload_med"; ARG_PAYLOAD="0.6"
  elif [[ "$ARG_SCENARIO" == "all" ]]; then
    echo "    → full matrix otomatis"; ARG_PAYLOAD="matrix"
  else
    echo "    1) 0.3   2) 0.6   3) 0.8   4) 1.0   5) 1.2"
    read -rp "Pilih [1-5]: " pl
    case "$pl" in
      1) ARG_PAYLOAD=0.3 ;; 2) ARG_PAYLOAD=0.6 ;; 3) ARG_PAYLOAD=0.8 ;;
      4) ARG_PAYLOAD=1.0 ;; 5) ARG_PAYLOAD=1.2 ;; *) die "Pilihan tidak valid" ;;
    esac
  fi

  # Sweep sub-menu
  if [[ "$ARG_SCENARIO" == "sweep" ]]; then
    echo -e "\n${BLU}[3b] Parameter L1MO yang di-sweep:${NC}"
    echo "     1) KP_Z      [def 3.0]  Z pos P gain"
    echo "     2) KD_Z      [def 1.5]  Z vel D gain"
    echo "     3) GAMMA_Z   [def 150]  Z adaptation rate"
    echo "     4) KP_XY     [def 1.0]  XY pos P gain"
    echo "     5) KD_XY     [def 0.5]  XY vel D gain"
    echo "     6) GAMMA_XY  [def 60]   XY adaptation rate (set GAMMA_X=GAMMA_Y)"
    echo "     7) LPF_Z     [def 8.0]  Z low-pass filter (rad/s)"
    echo "     8) LPF_XY    [def 10.0] XY low-pass filter (rad/s)"
    echo "     9) KOBS_Z    [def 15.0] Z observer gain"
    echo "    10) KOBS_XY   [def 15.0] XY observer gain"
    echo "    11) SIG_LIM   [def 2.0]  disturbance saturation (m/s²)"
    echo "    12) HE_TAU    [def 2.0]  hover est filter tau (s)"
    echo "    13) MO_DTH    [def 0.05] MO transient threshold (kg)"
    read -rp "Pilih [1-13]: " sp
    local _def=""
    case "$sp" in
      1)  ARG_SWEEP_PARAM=KP_Z;    _def="2.0 3.0 4.0 6.0 8.0" ;;
      2)  ARG_SWEEP_PARAM=KD_Z;    _def="1.0 1.5 2.0 3.0" ;;
      3)  ARG_SWEEP_PARAM=GAMMA_Z; _def="80 100 150 200 300" ;;
      4)  ARG_SWEEP_PARAM=KP_XY;   _def="0.8 1.0 1.5 2.0" ;;
      5)  ARG_SWEEP_PARAM=KD_XY;   _def="0.3 0.5 0.8 1.2" ;;
      6)  ARG_SWEEP_PARAM=GAMMA_XY; _def="30 60 100 150 200" ;;
      7)  ARG_SWEEP_PARAM=LPF_Z;   _def="5.0 6.0 8.0 10.0 15.0" ;;
      8)  ARG_SWEEP_PARAM=LPF_XY;  _def="5.0 8.0 10.0 15.0 20.0" ;;
      9)  ARG_SWEEP_PARAM=KOBS_Z;  _def="8.0 12.0 15.0 20.0" ;;
      10) ARG_SWEEP_PARAM=KOBS_XY; _def="8.0 12.0 15.0 20.0" ;;
      11) ARG_SWEEP_PARAM=SIG_LIM; _def="1.0 1.5 2.0 3.0 4.0" ;;
      12) ARG_SWEEP_PARAM=HE_TAU;  _def="1.0 2.0 3.0 5.0" ;;
      13) ARG_SWEEP_PARAM=MO_DTH;  _def="0.03 0.05 0.08 0.10" ;;
      *) die "Pilihan tidak valid" ;;
    esac
    echo -e "  Default: ${YLW}${_def}${NC}"
    read -rp "  Nilai sweep (Enter=default): " sv
    ARG_SWEEP_VALUES="${sv:-$_def}"
    echo -e "  → Sweep ${YLW}L1MO_${ARG_SWEEP_PARAM}${NC} = ${YLW}${ARG_SWEEP_VALUES}${NC}"
  fi

  echo -e "\n${BLU}[4] Wind:${NC}  1) with_wind   2) no_wind"
  read -rp "Pilih [1-2]: " wd; [[ "$wd" == "2" ]] && ARG_NO_WIND=true

  if [[ "$ARG_SCENARIO" == "motor_fault" ]]; then
    echo -e "\n${BLU}[5] Motor factor [1.0=normal 0.8=20% 0.6=40%]:${NC}"
    read -rp "Factor [default 0.6]: " mf; ARG_MOTOR_FACTOR="${mf:-0.6}"
  fi

  echo -e "\n${BLU}[6] Runs [default 10]:${NC}"; read -rp "Runs: " rn; ARG_RUNS="${rn:-10}"
  echo -e "\n${BLU}[7] Launch QGC? [y/N]:${NC}"; read -rp "" qgc
  [[ "${qgc,,}" == "y" ]] || ARG_SKIP_QGC=true

  local wind_str="with_wind"; $ARG_NO_WIND && wind_str="no_wind"
  echo ""; sep
  echo -e "  Scenario   : ${YLW}${ARG_SCENARIO}${NC}"
  echo -e "  Controller : ${YLW}${ARG_CONTROLLER}${NC}"
  echo -e "  Payload    : ${YLW}${ARG_PAYLOAD} kg${NC}"
  echo -e "  Wind       : ${YLW}${wind_str}${NC}"
  echo -e "  Runs       : ${YLW}${ARG_RUNS}${NC}"
  [[ "$ARG_SCENARIO" == "sweep" ]] && {
    echo -e "  Sweep      : ${YLW}L1MO_${ARG_SWEEP_PARAM}${NC}"
    echo -e "  Values     : ${YLW}${ARG_SWEEP_VALUES}${NC}"
  }
  [[ "$ARG_SCENARIO" == "motor_fault" ]] && echo -e "  Motor Fct  : ${YLW}${ARG_MOTOR_FACTOR}${NC}"
  sep
  read -rp $'\nLanjut? [y/N]: ' confirm
  [[ "${confirm,,}" == "y" ]] || { echo "Dibatalkan."; exit 0; }
}

# ═══════════════════════════════════════════════════════════════════════════════
# LOOPS & CAMPAIGN
# ═══════════════════════════════════════════════════════════════════════════════
run_scenario_loop() {
  local scenario="$1" controller="$2" payload="$3"
  local motor_factor="${4:-1.0}" n_runs="$5" sweep_param="${6:-}" sweep_val="${7:-}"
  for (( i=1; i<=n_runs; i++ )); do
    run_single "$scenario" "$controller" "$payload" "$motor_factor" "$i" "$sweep_param" "$sweep_val"
  done
  ok "Loop selesai: ${scenario}/${controller}/${payload}kg"
}

run_sweep_loop() {
  local controller="$1" payload="$2" n_runs="$3"
  [[ -z "$ARG_SWEEP_PARAM"  ]] && die "ARG_SWEEP_PARAM kosong"
  [[ -z "$ARG_SWEEP_VALUES" ]] && die "ARG_SWEEP_VALUES kosong"

  local values=()
  read -ra values <<< "$ARG_SWEEP_VALUES"
  local total_runs=$(( ${#values[@]} * n_runs ))

  sep
  log "${BOLD}PARAMETER SWEEP${NC}"
  log "  Parameter : L1MO_${ARG_SWEEP_PARAM}"
  log "  Nilai     : ${values[*]}"
  log "  Jumlah    : ${#values[@]} nilai x ${n_runs} runs = ${total_runs} total runs"
  log "  Payload   : ${payload} kg"
  sep

  local done_val=0
  for val in "${values[@]}"; do
    (( done_val++ ))
    log "${BOLD}[${done_val}/${#values[@]}] L1MO_${ARG_SWEEP_PARAM} = ${val}${NC}"
    run_scenario_loop "sweep" "l1" "$payload" "1.0" "$n_runs" "$ARG_SWEEP_PARAM" "$val"
    log "  Nilai ${val} selesai"
  done

  log "Aggregating sweep results..."
  python3 "${SCRIPT_DIR}/extract_metrics.py" \
    --aggregate "$RESULTS_CSV" \
    --out "${LOG_DIR}/sweep_${ARG_SWEEP_PARAM}_summary.csv" 2>/dev/null \
    && ok "Sweep summary → ${LOG_DIR}/sweep_${ARG_SWEEP_PARAM}_summary.csv" \
    || warn "Aggregate skip — pastikan metrics_summary.csv punya data"
}

run_full_campaign() {
  local n_runs="$1"
  local controllers=()
  [[ "$ARG_CONTROLLER" == "both" ]] && controllers=(l1 pid) || controllers=("$ARG_CONTROLLER")
  local matrix=(
    "hover|0.3|1.0"        "hover|0.6|1.0"   "hover|0.8|1.0"  "hover|1.0|1.0"
    "drop|0.6|1.0"         "drop|1.0|1.0"    "drop|1.2|1.0"
    "multi_drop|0.6|1.0"
    "dynamic_drop|0.6|1.0" "dynamic_drop|1.0|1.0"
    "pick_drop|0.6|1.0"    "pick_drop|1.0|1.0"
    "motor_fault|0.6|0.6"
  )
  local total=$(( ${#matrix[@]} * ${#controllers[@]} * n_runs )) done_count=0
  log "${BOLD}FULL CAMPAIGN: ${total} total runs${NC}"; sep
  for ctrl in "${controllers[@]}"; do
    for entry in "${matrix[@]}"; do
      IFS='|' read -r sc pl mf <<< "$entry"
      run_scenario_loop "$sc" "$ctrl" "$pl" "$mf" "$n_runs"
      done_count=$(( done_count + n_runs )); log "Progress: ${done_count}/${total}"
    done
  done
  ok "FULL CAMPAIGN SELESAI"
}

# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════
main() {
  mkdir -p "$LOG_DIR"
  log "Session: ${SESSION_ID} | Log: ${LOG_DIR}"
  check_deps
  [[ -z "$ARG_SCENARIO" ]] && interactive_menu
  [[ -z "$ARG_CONTROLLER" ]] && ARG_CONTROLLER="l1"
  [[ -z "$ARG_PAYLOAD" ]] && ARG_PAYLOAD="0.6"

  if [[ ! -f "$RESULTS_CSV" ]]; then
    python3 - << 'PYEOF' > "$RESULTS_CSV"
import csv, sys
fields = ['run_id','tag','controller','payload_kg','true_loaded_kg','scenario','wind',
    'motor_factor','drop_method','t_drop_s','t_drop_source','peak_alt_err_mm',
    'recovery_s','post_rmse_mm','m_before_kg','m_after_kg','detected_delta_kg',
    'mass_channel','n_steady','rmse_3d_mm','rmse_z_mm','mae_3d_mm','max_err_mm',
    'roll_std_deg','pitch_std_deg','roll_p2p_deg','m_est_kg','m_est_std',
    'etc_count','etc_rate_hz','sweep_param','sweep_val']
csv.DictWriter(sys.stdout, fieldnames=fields).writeheader()
PYEOF
    ok "Metrics CSV: ${RESULTS_CSV}"
  fi

  if [[ "$ARG_SCENARIO" == "all" ]]; then
    run_full_campaign "$ARG_RUNS"
  elif [[ "$ARG_SCENARIO" == "sweep" ]]; then
    [[ -z "$ARG_SWEEP_PARAM" ]] && die "Pilih sweep param dulu"
    run_sweep_loop "l1" "$ARG_PAYLOAD" "$ARG_RUNS"
  else
    local controllers=()
    [[ "$ARG_CONTROLLER" == "both" ]] && controllers=(l1 pid) || controllers=("$ARG_CONTROLLER")
    for ctrl in "${controllers[@]}"; do
      run_scenario_loop "$ARG_SCENARIO" "$ctrl" "$ARG_PAYLOAD" "$ARG_MOTOR_FACTOR" "$ARG_RUNS"
    done
  fi

  sep
  python3 "${SCRIPT_DIR}/extract_metrics.py" \
    --aggregate "$RESULTS_CSV" --out "${LOG_DIR}/campaign_summary.csv" 2>/dev/null \
    && ok "Summary → ${LOG_DIR}/campaign_summary.csv" || true
  echo ""; sep; log "${BOLD}SELESAI${NC}"; log "  Logs: ${LOG_DIR}"; log "  CSV: ${RESULTS_CSV}"; sep
}

main "$@"
