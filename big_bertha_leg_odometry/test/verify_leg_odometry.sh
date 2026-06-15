#!/usr/bin/env bash
# Headless acceptance gate for the leg odometry module (Module 6).
#
# Assumes a headless sim (big_bertha_sim_bringup) and the policy controller
# are already running.  Launches the leg odometry node, publishes /cmd_vel
# vx=0.3 for ~6 s, asserts /leg_odom publishes at rate and that the twist is
# non-zero while moving and near-zero when stopped.  Evidence goes to
# verification_artifacts/leg_odometry/.
#
# Usage: test/verify_leg_odometry.sh
set -uo pipefail
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

ART="${ART:-verification_artifacts/leg_odometry}"
mkdir -p "${ART}"

echo "[verify] launching leg odometry node"
ros2 run big_bertha_leg_odometry leg_odometry_node \
  --ros-args --params-file big_bertha_leg_odometry/config/leg_odometry.yaml \
  > "${ART}/node.log" 2>&1 &
NODE_PID=$!

cleanup() {
  echo "[verify] cleaning up"
  kill "${NODE_PID}" 2>/dev/null
  wait "${NODE_PID}" 2>/dev/null
}
trap cleanup EXIT

sleep 3

PASS=0

echo "[verify] checking topic rate while standing still"
HZ_STAND=$(timeout 8 ros2 topic hz /leg_odom 2>/dev/null \
           | grep -oP 'average rate: \K[0-9.]+' | head -1)
if [[ -n "${HZ_STAND}" ]] && awk "BEGIN{exit !(${HZ_STAND} > 0)}"; then
  echo "[PASS] /leg_odom @ ${HZ_STAND} Hz (standing)" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] /leg_odom not publishing while standing" | tee -a "${ART}/gate.txt"
  PASS=1
fi

echo "[verify] driving vx=0.3 for 6 s"
timeout 7 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}}' > /dev/null 2>&1 &
PUB=$!
sleep 6.5
kill "${PUB}" 2>/dev/null

echo "[verify] checking twist linear.x while moving"
TWIST=$(timeout 6 ros2 topic echo /leg_odom --once 2>/dev/null \
        | grep -A5 'linear:' | grep -E '  x:' | grep -oE '[-0-9.]+' | head -1)
echo "linear.x during motion: ${TWIST:-0}" | tee -a "${ART}/gate.txt"
if [[ -n "${TWIST}" ]] && awk "BEGIN{exit !(${TWIST} > 0.05)}"; then
  echo "[PASS] /leg_odom twist non-zero while driving" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] /leg_odom twist too low while driving" | tee -a "${ART}/gate.txt"
  PASS=1
fi

echo "[verify] checking twist near-zero after stop"
sleep 3
TWIST_STOP=$(timeout 6 ros2 topic echo /leg_odom --once 2>/dev/null \
             | grep -A5 'linear:' | grep -E '  x:' | grep -oE '[-0-9.]+' | head -1)
echo "linear.x after stop: ${TWIST_STOP:-0}" | tee -a "${ART}/gate.txt"
if [[ -n "${TWIST_STOP}" ]] && awk "BEGIN{exit !(sqrt(${TWIST_STOP}^2) < 0.05)}"; then
  echo "[PASS] /leg_odom twist near-zero at rest" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] /leg_odom twist non-zero at rest" | tee -a "${ART}/gate.txt"
  PASS=1
fi

echo "================ GATE ================"; cat "${ART}/gate.txt"
exit "${PASS}"
