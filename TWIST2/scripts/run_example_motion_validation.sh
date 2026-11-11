#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON="$ROOT/.conda-vr/bin/python"
REDIS_CLI="$ROOT/.conda-vr/bin/redis-cli"
REDIS_SERVER="$ROOT/.conda-vr/bin/redis-server"
MOTION="${1:-$ROOT/assets/example_motions/0807_yanjie_walk_003.pkl}"
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

"$REDIS_CLI" -h 127.0.0.1 -p 6379 del \
  state_body_unitree_g1_with_hands >/dev/null

TWIST2_ALLOW_SIM_REDIS=1 "$PYTHON" "$ROOT/scripts/publish_example_motion.py" \
  --motion "$MOTION" >"$ROOT/runtime/example_motion_publisher.log" 2>&1 &
publisher_pid=$!

TWIST2_ALLOW_SIM_REDIS=1 "$PYTHON" "$ROOT/scripts/headless_sim_smoke.py" \
  --seconds 5 --policy-frequency 50 --realtime \
  --output "$ROOT/runtime/sim_example_motion_smoke.json"

kill -TERM "$publisher_pid" 2>/dev/null || true
wait "$publisher_pid"
publisher_pid=""
grep -q 'step=1/' "$ROOT/runtime/example_motion_publisher.log"
