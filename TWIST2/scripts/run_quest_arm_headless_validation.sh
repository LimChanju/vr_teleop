#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON="$ROOT/.conda-vr/bin/python"
REDIS_CLI="$ROOT/.conda-vr/bin/redis-cli"
REDIS_SERVER="$ROOT/.conda-vr/bin/redis-server"
REDIS_PORT=${TWIST2_VALIDATION_REDIS_PORT:-16378}
mkdir -p "$ROOT/runtime/redis-validation"

publisher_pid=""
redis_pid=""
cleanup() {
  if [[ -n "$publisher_pid" ]]; then
    kill -TERM "$publisher_pid" 2>/dev/null || true
    wait "$publisher_pid" 2>/dev/null || true
  fi
  if [[ -n "$redis_pid" ]]; then
    "$REDIS_CLI" -h 127.0.0.1 -p "$REDIS_PORT" shutdown nosave >/dev/null 2>&1 || true
    wait "$redis_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if "$REDIS_CLI" -h 127.0.0.1 -p "$REDIS_PORT" ping >/dev/null 2>&1; then
  echo "Validation Redis port $REDIS_PORT is already in use." >&2
  exit 1
fi
"$REDIS_SERVER" --port "$REDIS_PORT" --bind 127.0.0.1 --protected-mode yes \
  --save '' --appendonly no --dir "$ROOT/runtime/redis-validation" \
  >"$ROOT/runtime/redis-validation.log" 2>&1 &
redis_pid=$!
for _ in $(seq 1 50); do
  "$REDIS_CLI" -h 127.0.0.1 -p "$REDIS_PORT" ping >/dev/null 2>&1 && break
  sleep 0.1
done

TWIST2_ALLOW_SIM_REDIS=1 "$PYTHON" "$ROOT/scripts/publish_quest_arm_validation.py" \
  --redis-port "$REDIS_PORT" --seconds 16 \
  >"$ROOT/runtime/quest_arm_validation_publisher.log" 2>&1 &
publisher_pid=$!
sleep 0.5

TWIST2_ALLOW_SIM_REDIS=1 "$PYTHON" "$ROOT/scripts/headless_sim_smoke.py" \
  --redis-port "$REDIS_PORT" --action-timeout-ms 300 \
  --seconds 13 --policy-frequency 50 --realtime \
  --output "$ROOT/runtime/sim_quest_arm_ik_smoke.json"
wait "$publisher_pid"
publisher_pid=""
cat "$ROOT/runtime/quest_arm_reference_validation.json"
