#!/usr/bin/env bash
# Copyright 2026, lbgos
# SPDX-License-Identifier: BSL-1.0
set -euo pipefail
# Run with SteamVR stopped and the headset ready. Needs the xr-probe OpenXR pose logger; set XR_PROBE to its path.
# Usage: rift-s-controller-capture.sh OUTPUT [SECONDS=80] [ENV=VALUE ...]
# RIFTS_LOCK_OWNER names the hardware lock holder (default rift-s-capture).
# Keep unworn runs to SECONDS <= 20, for a total service lifetime <= 30 s.
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
out=$(realpath -m "${1:?output}")
seconds=${2:-80}
shift "$(( $# > 1 ? 2 : 1 ))"
if pgrep -x monado-service >/dev/null || pgrep -x vrserver >/dev/null; then
  echo 'Hardware busy: monado-service or vrserver is running' >&2
  exit 1
fi
owner=${RIFTS_LOCK_OWNER:-rift-s-capture}
# A lock already written under our own name is accepted, so a caller can queue for it.
if [[ $(cat /tmp/rifts-hw.lock 2>/dev/null) != "$owner" ]]; then
  (set -o noclobber; printf '%s\n' "$owner" > /tmp/rifts-hw.lock) || exit 1
fi
pid=''
cleanup() {
  if [[ -n "$pid" ]]; then
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
  rm -f /tmp/rifts-hw.lock
}
trap cleanup EXIT
mkdir -p "$out/led"
start=$(date --iso-8601=seconds)
printf "%s\n" "$start" > "$out/started-at.txt"
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
export DBUS_SESSION_BUS_ADDRESS=unix:path=$XDG_RUNTIME_DIR/bus
export XRT_COMPOSITOR_FORCE_WAYLAND_DIRECT=1
export SLAM_SUBMIT_FROM_START=true
export VIT_SYSTEM_LIBRARY_PATH=/usr/local/lib/libbasalt.so
export XRT_NO_STDIN=true
export RIFT_S_LOG=trace CONSTELLATION_LOG=trace SLAM_LOG=info
export RIFT_S_CONTROLLER_DUMP_DIR="$out/led"
export XR_RUNTIME_JSON="$root/build/openxr_monado-dev.json"
env "$@" timeout -s INT "$((seconds+10))" "$root/build/src/xrt/targets/service/monado-service" > "$out/monado.log" 2>&1 &
pid=$!
sleep 8
if ! grep -q 'Controller IMU device=2 ' "$out/monado.log" ||
   ! grep -q 'Controller IMU device=3 ' "$out/monado.log"; then
  echo 'Both controllers must stream IMU. Stopping; wake them before the next capture.' >&2
  exit 1
fi
printf "Pose capture starts now: %s seconds\n" "$seconds"
"${XR_PROBE:?set XR_PROBE to the xr-probe binary}" --seconds "$seconds" > "$out/poses.jsonl" 2> "$out/probe.log"
set +e
wait "$pid"
rc=$?
set -e
pid=''
printf '%s\n' "$rc" > "$out/monado.exit"
[[ $rc == 124 || $rc == 0 ]]
sudo -n journalctl -k --since "$start" --no-pager -o short-monotonic > "$out/kernel.log" 2> "$out/kernel.error" || true
printf "%s\n" "$out"
