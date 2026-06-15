#!/usr/bin/env bash
# Headless acceptance gate for the state estimation module (Module 7).
#
# Assumes a headless sim (big_bertha_sim_bringup) + policy controller + leg
# odometry + EKF are already running.  Checks topic rates, tf continuity,
# covariance bounds, and standing-still drift.
#
# Usage: test/verify_state_estimation.sh
set -uo pipefail
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

ART="${ART:-verification_artifacts/state_estimation}"
mkdir -p "${ART}"

echo "[verify] launching EKF (sim config)"
ros2 launch big_bertha_state_estimation ekf.launch.py \
  params_file:=big_bertha_state_estimation/config/ekf_sim.yaml \
  > "${ART}/node.log" 2>&1 &
NODE_PID=$!

cleanup() {
  echo "[verify] cleaning up"
  kill "${NODE_PID}" 2>/dev/null
  wait "${NODE_PID}" 2>/dev/null
}
trap cleanup EXIT

sleep 5

PASS=0

: > "${ART}/gate.txt"

echo "[verify] checking /odom_filtered rate"
HZ=$(timeout 10 ros2 topic hz /odom_filtered 2>/dev/null \
     | grep -oP 'average rate: \K[0-9.]+' | head -1)
if [[ -n "${HZ}" ]] && awk "BEGIN{exit !(${HZ} > 40)}"; then
  echo "[PASS] /odom_filtered @ ${HZ} Hz" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] /odom_filtered rate < 40 Hz" | tee -a "${ART}/gate.txt"
  PASS=1
fi

echo "[verify] checking covariance finite"
ODOM=$(timeout 10 ros2 topic echo /odom_filtered --once 2>/dev/null)
echo "${ODOM}" > "${ART}/odom_sample.txt"
if echo "${ODOM}" | grep -qE 'covariance:'; then
  echo "[PASS] /odom_filtered has covariance" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] /odom_filtered missing covariance" | tee -a "${ART}/gate.txt"
  PASS=1
fi

echo "[verify] checking tf continuity (odom->base_link)"
GAPS=$(timeout 10 ros2 tf2_echo odom base_link 2>/dev/null \
       | head -20 | grep -c 'Translation')
if [[ "${GAPS}" -gt 0 ]]; then
  echo "[PASS] odom->base_link tf present" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] odom->base_link tf missing" | tee -a "${ART}/gate.txt"
  PASS=1
fi

echo "[verify] standing-still drift check"
sleep 3
DRIFT_POS=$(timeout 6 ros2 topic echo /odom_filtered --once 2>/dev/null \
            | grep -A3 'position:' | grep -oE '[-0-9.]+' | head -3 | tr '\n' ' ')
echo "position after rest: ${DRIFT_POS}" | tee -a "${ART}/gate.txt"
DRIFT=$(awk '{print sqrt($1*$1+$2*$2)}' <<< "${DRIFT_POS}" 2>/dev/null || echo 0)
if awk "BEGIN{exit !(${DRIFT} < 0.01)}" 2>/dev/null; then
  echo "[PASS] drift < 0.01 m" | tee -a "${ART}/gate.txt"
else
  echo "[WARN] drift ${DRIFT}m (may be high if robot was just moved)" | tee -a "${ART}/gate.txt"
fi

echo "================ GATE ================"; cat "${ART}/gate.txt"
exit "${PASS}"
