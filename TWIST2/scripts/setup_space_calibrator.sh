#!/usr/bin/env bash
# Build upstream unchanged; install only project-local files and user OpenVR registrations.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
SC="$ROOT/external/openvr-space-calibrator-linux"
# SteamVR 2.12.14 exposes IVRSystem_023, not the latest SDK's IVRSystem_026.
# Keep both upstream SDK checkouts intact and select the matching release here.
OVR="$ROOT/external/openvr-2.12.14"
DEPS="$ROOT/.spacecal-build-env"
BUILD="$ROOT/runtime/spacecal/build-openvr-2.12.14"
DRIVERS="$ROOT/runtime/spacecal/drivers"
MODE=${1:---build}
[[ "$MODE" == --build || "$MODE" == --install ]] || { echo 'Usage: setup_space_calibrator.sh [--build|--install]'; exit 2; }
"$ROOT/.conda-vr/bin/python" - <<'PY'
from twist2_vr.spacecal_session import loaded_runtime_drivers
if loaded_runtime_drivers():
    raise SystemExit('Stop SteamVR before rebuilding/replacing loaded driver binaries.')
PY
if [[ ! -d "$SC/.git" ]]; then
  git clone --recursive https://github.com/xi-ve/openvr-space-calibrator-linux.git "$SC"
fi
if [[ ! -d "$OVR/.git" ]]; then
  git clone --depth 1 --branch v2.12.14 https://github.com/ValveSoftware/openvr.git "$OVR"
fi
[[ $(git -C "$SC" rev-parse HEAD) == 28e3f83f7bc9808f145fac03ff8a8e455ab211fe ]] || { echo 'xi-ve revision changed; review before building.'; exit 1; }
[[ $(git -C "$OVR" rev-parse HEAD) == 91825305130f446f82054c1ec3d416321ace0072 ]] || { echo 'OpenVR revision changed; review before building.'; exit 1; }
git -C "$SC" diff --exit-code
git -C "$OVR" diff --exit-code
if [[ ! -x "$DEPS/bin/cmake" ]]; then
  "${CONDA_EXE:-/home/railab/miniconda3/bin/conda}" create --yes --prefix "$DEPS" \
    --override-channels -c conda-forge 'cmake<4' 'eigen=3.4' glfw ninja pkg-config \
    libgl-devel xorg-libx11 xorg-libxrandr xorg-libxinerama xorg-libxcursor xorg-libxi
fi
# Prefer the host GLVND libraries at runtime; mixing conda GLX with host OpenGL
# fails ImGui's loader on this NVIDIA machine. GLFW/build tools remain isolated.
env PKG_CONFIG_PATH="$DEPS/lib/pkgconfig" "$DEPS/bin/cmake" -S "$SC" -B "$BUILD" -G Ninja \
  -DCMAKE_MAKE_PROGRAM="$DEPS/bin/ninja" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$DEPS" -DOPENVR_INCLUDE_DIR="$OVR/headers" \
  -DOPENVR_LIB_DIR="$OVR/lib/linux64" \
  -DCMAKE_CXX_FLAGS="-I$DEPS/include -I$SC/src/common" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$DEPS/lib -Wl,-rpath,/usr/lib/x86_64-linux-gnu -Wl,-rpath,$DEPS/lib"
"$DEPS/bin/cmake" --build "$BUILD" --parallel 2
mkdir -p "$DRIVERS/twist2_vut/bin/linux64"
g++ -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -I"$OVR/headers" \
  -fPIC -shared tools/vut_openvr/driver.cpp -pthread \
  -o "$DRIVERS/twist2_vut/bin/linux64/driver_twist2_vut.so"
g++ -std=c++17 -O2 -I"$OVR/headers" tools/vut_openvr/test_protocol.cpp -o "$BUILD/test_vut_protocol"
"$BUILD/test_vut_protocol"
cp tools/vut_openvr/driver.vrdrivermanifest "$DRIVERS/twist2_vut/"
mkdir -p "$DRIVERS/01spacecalibrator/bin/linux64"
cp -r "$SC/driver_01spacecalibrator/resources" "$DRIVERS/01spacecalibrator/"
cp "$SC/driver_01spacecalibrator/driver.vrdrivermanifest" "$DRIVERS/01spacecalibrator/"
cp "$BUILD/lib/driver_01spacecalibrator.so" "$BUILD/bin/space-calibrator" \
  "$SC/manifest.vrmanifest" "$SC/src/overlay/actions.json" "$DRIVERS/01spacecalibrator/bin/linux64/"
echo 'Built xi-ve unchanged and the separate VUT driver. No calibration applied.'
if [[ "$MODE" == --install ]]; then
  STEAMVR=$("$ROOT/.conda-vr/bin/python" -c 'import json,pathlib; print(json.loads((pathlib.Path.home()/".config/openvr/openvrpaths.vrpath").read_text())["runtime"][0])')
  BACKUP=$(mktemp -d "$ROOT/runtime/spacecal/backup.XXXXXXXX")
  for CONFIG in "$HOME/.config/openvr/openvrpaths.vrpath" \
      "$HOME/.steam/debian-installation/config/steamvr.vrsettings" \
      "$HOME/.steam/debian-installation/config/appconfig.json"; do
    [[ ! -f "$CONFIG" ]] || cp -p "$CONFIG" "$BACKUP/"
  done
  for DRIVER in "$DRIVERS/01spacecalibrator" "$DRIVERS/twist2_vut"; do
    env LD_LIBRARY_PATH="$STEAMVR/bin/linux64:${LD_LIBRARY_PATH:-}" \
      "$STEAMVR/bin/linux64/vrpathreg" adddriver "$DRIVER"
  done
  echo "User OpenVR registrations added. Backup: $BACKUP"
  echo 'SteamVR must be restarted once. This script does not restart it.'
fi
