#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON="$ROOT/.conda-vr/bin/python"
REDIS_CLI="$ROOT/.conda-vr/bin/redis-cli"
REDIS_SERVER="$ROOT/.conda-vr/bin/redis-server"
REDIS_PORT=${TWIST2_QUEST_REDIS_PORT:-16379}
mkdir -p "$ROOT/runtime/redis-quest"

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

if [[ ! -x "$PYTHON" || ! -x "$REDIS_SERVER" ]]; then
  echo "TWIST2 environment is missing. Run: bash $ROOT/scripts/setup_vr_env.sh" >&2
  exit 1
fi
if "$REDIS_CLI" -h 127.0.0.1 -p "$REDIS_PORT" ping >/dev/null 2>&1; then
  echo "Dedicated simulation Redis port $REDIS_PORT is already in use; stop that process first." >&2
  exit 1
fi

"$REDIS_SERVER" --port "$REDIS_PORT" --bind 127.0.0.1 --protected-mode yes \
  --save '' --appendonly no --dir "$ROOT/runtime/redis-quest" \
  >"$ROOT/runtime/redis-quest.log" 2>&1 &
redis_pid=$!
for _ in $(seq 1 50); do
  "$REDIS_CLI" -h 127.0.0.1 -p "$REDIS_PORT" ping >/dev/null 2>&1 && break
  sleep 0.1
done
if ! "$REDIS_CLI" -h 127.0.0.1 -p "$REDIS_PORT" ping >/dev/null 2>&1; then
  echo "Could not start the isolated Redis server; see runtime/redis-quest.log." >&2
  exit 1
fi

echo "============================================================"
echo " QUEST 3 ARM TELEOP — LOCAL MUJOCO SIMULATION ONLY"
echo " 1) 양쪽 그립을 놓고 컨트롤러를 편한 중립 위치에 고정"
echo " 2) [READY]가 보이면 양쪽 그립을 계속 누른 채 천천히 손 이동"
echo " 3) 한쪽 그립을 놓으면 현재 팔 목표를 유지"
echo " 4) 오른쪽 조이스틱 클릭: VR 시점 재정렬"
echo " 5) 양쪽 조이스틱 동시 클릭: 팔 입력 비상정지"
echo " 6) 입력 1:1: smoothing/gain/cap/slew/IK projection 없음"
echo "    TRACKING_LOST: 추적 손실"
echo "    IK: 요청 손목과 실제 IK 결과 사이의 오차(mm); G1 실제 관절 범위는 유지"
echo " 종료: Ctrl+C"
echo "============================================================"

(
  set -o pipefail
  TWIST2_ALLOW_SIM_REDIS=1 "$PYTHON" -u -m twist2_vr quest-arms \
    --frames 0 --frequency 50 --publish-sim --redis-port "$REDIS_PORT" \
    --status-every 250 2>&1 | tee "$ROOT/runtime/quest_arms_publisher.log"
) &
publisher_pid=$!
sleep 1
if ! kill -0 "$publisher_pid" 2>/dev/null; then
  echo "Quest input publisher failed; see runtime/quest_arms_publisher.log." >&2
  wait "$publisher_pid" || true
  exit 1
fi

cd "$ROOT/deploy_real"
"$PYTHON" server_low_level_g1_sim.py \
  --xml ../assets/g1/g1_sim2sim_29dof.xml \
  --policy ../assets/ckpts/twist2_1017_20k.onnx \
  --device cpu --measure_fps 1 --policy_frequency 50 --limit_fps 1 \
  --redis-host 127.0.0.1 --redis-port "$REDIS_PORT" --action-timeout-ms 300 \
  --viewer vr --vr-view front --vr-distance 2.2 "$@"
