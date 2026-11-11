#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

# Safe visual validation: deterministic full mock trackers drive only the local
# MuJoCo simulator.  The Quest receives a stereo front view through
# SteamVR/ALVR; no Unitree SDK or physical-robot output is used.
exec bash "$ROOT/scripts/run_mock_sim.sh" \
  --viewer vr --vr-view front --vr-distance 2.2 "$@"
