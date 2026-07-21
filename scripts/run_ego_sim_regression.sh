#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/home/p/px4_ros2_ws}"
PROFILE="${PROFILE:-chain}"
REPEAT_COUNT="${REPEAT_COUNT:-3}"
INJECT_HELPER_TIMEOUT="${INJECT_HELPER_TIMEOUT:-false}"
TAKEOVER_COMMAND="${TAKEOVER_COMMAND:-all}"
PLANNER_LOG="${PLANNER_LOG:-${WS_DIR}/.runtime/logs/ego_planner_offboard.log}"
RESULT_DIR="${RESULT_DIR:-${WS_DIR}/.runtime/regression}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

case "${PROFILE}" in
  chain|small|avoidance|takeover) ;;
  *) echo "[ERROR] PROFILE must be chain, small, avoidance, or takeover" >&2; exit 2 ;;
esac

if ! [[ "${REPEAT_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "[ERROR] REPEAT_COUNT must be a positive integer" >&2
  exit 2
fi

case "${TAKEOVER_COMMAND}" in
  all|land|rtl|disarm) ;;
  *) echo "[ERROR] TAKEOVER_COMMAND must be all, land, rtl, or disarm" >&2; exit 2 ;;
esac

if [[ "${PROFILE}" != "chain" && "${CONFIRM_SIMULATION_FLIGHT:-}" != "YES" ]]; then
  echo "[ERROR] ${PROFILE} arms PX4 SITL. Export CONFIRM_SIMULATION_FLIGHT=YES to confirm simulation-only flight." >&2
  exit 2
fi

mkdir -p "${RESULT_DIR}"
timestamp="$(date +%Y%m%d_%H%M%S)"
summary="${RESULT_DIR}/${timestamp}_${PROFILE}_summary.log"
planner_start_line=1
if [[ -f "${PLANNER_LOG}" ]]; then
  planner_start_line=$(( $(wc -l < "${PLANNER_LOG}") + 1 ))
fi

restore_nounset=0
if [[ $- == *u* ]]; then restore_nounset=1; set +u; fi
source /opt/ros/humble/setup.bash
source "${WS_DIR}/install/setup.bash"
if [[ "${restore_nounset}" == "1" ]]; then set -u; fi

if [[ "${PROFILE}" != "chain" ]]; then
  required_topics=(
    /fmu/out/vehicle_status_v4
    /fmu/out/vehicle_local_position_v1
    /odom
  )
  available_topics="$(timeout 5 ros2 topic list 2>/dev/null || true)"
  missing_topics=()
  for topic in "${required_topics[@]}"; do
    if ! grep -Fxq "${topic}" <<< "${available_topics}"; then
      missing_topics+=("${topic}")
    fi
  done
  if (( ${#missing_topics[@]} > 0 )); then
    echo "[ERROR] Autonomy runtime is not ready; missing topics: ${missing_topics[*]}" >&2
    echo "[ERROR] Start PX4 SITL, MicroXRCEAgent, FAST-LIO/odom bridge, Ego-Planner, and trajectory_interface first." >&2
    exit 2
  fi
fi

echo "REGRESSION profile=${PROFILE} repeats=${REPEAT_COUNT} started=${timestamp}" | tee "${summary}"
for ((run=1; run<=REPEAT_COUNT; run++)); do
  run_log="${RESULT_DIR}/${timestamp}_${PROFILE}_${run}.log"
  echo "[INFO] ${PROFILE} run ${run}/${REPEAT_COUNT}" | tee -a "${summary}"
  case "${PROFILE}" in
    chain)
      # EGOReplanFSM intentionally de-duplicates goals closer than 5 cm. Vary the
      # no-arm target so every regression iteration must generate a fresh spline.
      chain_goal_distance="$(awk -v run="${run}" 'BEGIN { printf "%.2f", 0.60 + (run * 0.10) }')"
      "${PYTHON_BIN}" -u "${WS_DIR}/scripts/validate_fastlio_ego_avoidance.py" --self-test | tee "${run_log}"
      "${PYTHON_BIN}" -u "${WS_DIR}/scripts/validate_fastlio_ego_avoidance.py" \
        --require-chain --no-arm --chain-goal-distance "${chain_goal_distance}" | tee -a "${run_log}"
      grep -q "chain_ok: True\|chain_ok True" "${run_log}"
      ;;
    small)
      small_extra_args=()
      if [[ "${INJECT_HELPER_TIMEOUT}" == "true" ]]; then
        small_extra_args+=(--inject-helper-timeout-after-arm)
      fi
      set +e
      "${PYTHON_BIN}" -u "${WS_DIR}/scripts/validate_ego_small_goal_handover.py" \
        --goal-forward-m 0.6 --max-move-pass-m 1.2 "${small_extra_args[@]}" | tee "${run_log}"
      validation_rc=${PIPESTATUS[0]}
      set -e
      if [[ "${INJECT_HELPER_TIMEOUT}" == "true" ]]; then
        grep -q "SAFETY_CLEANUP reason=injected_helper_timeout" "${run_log}"
        grep -q "LAND_COMPLETE" "${run_log}"
        if [[ "${validation_rc}" -ne 2 ]]; then
          echo "[ERROR] Injected helper timeout returned unexpected status ${validation_rc}" >&2
          exit 1
        fi
        echo "PASS injected_helper_timeout_cleanup run=${run}" | tee -a "${summary}"
        continue
      fi
      if [[ "${validation_rc}" -ne 0 ]]; then
        echo "[ERROR] Small-goal validator failed with status ${validation_rc}" >&2
        exit "${validation_rc}"
      fi
      grep -q "SMALL_GOAL_RESULT passed=True" "${run_log}"
      grep -q "bad=none" "${run_log}"
      grep -q "offboard_seen=True" "${run_log}"
      grep -q "failsafe=False" "${run_log}"
      grep -q "LAND_COMPLETE" "${run_log}"
      ;;
    avoidance)
      "${PYTHON_BIN}" -u "${WS_DIR}/scripts/validate_fastlio_ego_avoidance.py" \
        --require-chain --required-clearance 0.45 --landing-zone-clearance 0.80 --timeout 120 | tee "${run_log}"
      grep -q "PASS_CLEARANCE_TEST True" "${run_log}"
      grep -q "LAND_COMPLETE" "${run_log}"
      ;;
    takeover)
      if [[ "${TAKEOVER_COMMAND}" == "all" ]]; then
        takeover_commands=(land rtl disarm)
      else
        takeover_commands=("${TAKEOVER_COMMAND}")
      fi
      : > "${run_log}"
      for takeover_command in "${takeover_commands[@]}"; do
        command_log="${RESULT_DIR}/${timestamp}_${PROFILE}_${run}_${takeover_command}.log"
        echo "[INFO] takeover command=${takeover_command}" | tee -a "${run_log}"
        "${PYTHON_BIN}" -u "${WS_DIR}/scripts/validate_fastlio_ego_avoidance.py" \
          --require-chain --takeover-after-s 8 --takeover-command "${takeover_command}" --timeout 45 \
          | tee "${command_log}"
        cat "${command_log}" >> "${run_log}"
        grep -q "TAKEOVER_TEST True" "${command_log}"
        if [[ "${takeover_command}" != "disarm" ]]; then
          grep -q "LAND_COMPLETE\|TAKEOVER_DISARMED" "${command_log}"
        else
          grep -q "TAKEOVER_DISARMED" "${command_log}"
        fi
        # Prevent a previous takeover epoch from contaminating the next one.
        sleep 2
      done
      ;;
  esac
  echo "PASS run=${run}" | tee -a "${summary}"
done

if [[ -f "${PLANNER_LOG}" ]]; then
  "${PYTHON_BIN}" "${WS_DIR}/scripts/check_ego_planner_log.py" \
    --start-line "${planner_start_line}" "${PLANNER_LOG}" | tee -a "${summary}"
else
  echo "[ERROR] Planner log not found: ${PLANNER_LOG}" | tee -a "${summary}" >&2
  exit 2
fi

echo "SIM_REGRESSION_RESULT passed=True profile=${PROFILE} repeats=${REPEAT_COUNT} summary=${summary}" | tee -a "${summary}"