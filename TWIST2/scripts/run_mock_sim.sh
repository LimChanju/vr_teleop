#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON="$ROOT/.conda-vr/bin/python"
REDIS_CLI="$ROOT/.conda-vr/bin/redis-cli"
REDIS_SERVER="$ROOT/.conda-vr/bin/redis-server"
mkdir -p "$ROOT/runtime/redis"

redis_started=0
publisher_pid=""
cleanup() {
  if [[ -n "$publisher_pid" ]]; then
    kill -TERM "$publisher_pid" 2>/dev/null || true
    wait "$publisher_pid" 2>/dev/null || true
  fi
  if [[ $redis_started -eq 1 ]]; then
    "$REDIS_CLI" -h 127.0.0.1 -p 6379 shutdown nosave >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if ! "$REDIS_CLI" -h 127.0.0.1 -p 6379 ping >/dev/null 2>&1; then
  "$REDIS_SERVER" --port 6379 --bind 127.0.0.1 --protected-mode yes \
    --save '' --appendonly no --dir "$ROOT/runtime/redis" \
    >"$ROOT/runtime/redis.log" 2>&1 &
  redis_started=1
  for _ in $(seq 1 50); do
    "$REDIS_CLI" -h 127.0.0.1 -p 6379 ping >/dev/null 2>&1 && break
    sleep 0.1
  done
fi

TWIST2_ALLOW_SIM_REDIS=1 "$PYTHON" -m twist2_vr mock --frames 0 \
  --mock-layout full --retargeter gmr --publish-sim --status-every 250 \
  >"$ROOT/runtime/mock_publisher.log" 2>&1 &
publisher_pid=$!
sleep 1

cd "$ROOT/deploy_real"
"$PYTHON" server_low_level_g1_sim.py \
  --xml ../assets/g1/g1_sim2sim_29dof.xml \
  --policy ../assets/ckpts/twist2_1017_20k.onnx \
  --device cpu --measure_fps 1 --policy_frequency 50 --limit_fps 1 "$@"
