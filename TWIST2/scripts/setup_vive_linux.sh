#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
VERSION=${VIVE_HUB_VERSION:-v1.0.3}
REPOSITORY=https://github.com/ViveRoboticsSoftware/vivehub-linux

case "$(uname -m)" in
  x86_64) ARCH=x86_64 ;;
  aarch64|arm64) ARCH=arm64 ;;
  *) echo "Unsupported architecture: $(uname -m)" >&2; exit 1 ;;
esac

ASSET="VIVEHub-Linux-${VERSION}-${ARCH}.tar.gz"
BASE_URL="${REPOSITORY}/releases/download/${VERSION}"
DOWNLOAD_DIR="$ROOT/runtime/downloads/vivehub-linux-${VERSION}"
DESTINATION="$ROOT/external/VIVEHub-Linux-${VERSION}"

if [[ ${1:-} != "--accept-vive-eula" ]]; then
  echo "This downloads HTC's proprietary VIVE Hub Linux Beta." >&2
  echo "Review HTC's release and VIVE Product EULA first:" >&2
  echo "  $REPOSITORY/releases/tag/$VERSION" >&2
  echo "  bash scripts/setup_vive_linux.sh --accept-vive-eula" >&2
  exit 2
fi

for command_name in curl sha256sum tar; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "Missing required command: $command_name" >&2
    exit 1
  fi
done

mkdir -p "$DOWNLOAD_DIR" "$ROOT/external" "$ROOT/runtime"

if [[ ! -f "$DOWNLOAD_DIR/$ASSET" ]]; then
  curl -fL "$BASE_URL/$ASSET" -o "$DOWNLOAD_DIR/$ASSET"
fi
curl -fL "$BASE_URL/VIVEHub-Linux-${VERSION}.sha256" \
  -o "$DOWNLOAD_DIR/VIVEHub-Linux-${VERSION}.sha256"

EXPECTED=$(awk -v asset="$ASSET" '$2 == asset || $2 == "*" asset {print $1}' \
  "$DOWNLOAD_DIR/VIVEHub-Linux-${VERSION}.sha256")
ACTUAL=$(sha256sum "$DOWNLOAD_DIR/$ASSET" | awk '{print $1}')
if [[ -z "$EXPECTED" || "$ACTUAL" != "$EXPECTED" ]]; then
  echo "SHA256 verification failed for $ASSET" >&2
  exit 1
fi

if [[ -e "$DESTINATION" && ! -x "$DESTINATION/run-vivehub.sh" ]]; then
  echo "Existing VIVE Hub path is incomplete; move it aside and retry: $DESTINATION" >&2
  exit 1
fi

if [[ ! -x "$DESTINATION/run-vivehub.sh" ]]; then
  STAGING=$(mktemp -d "$ROOT/runtime/vivehub-extract.XXXXXX")
  trap 'rm -rf -- "$STAGING"' EXIT
  tar -xzf "$DOWNLOAD_DIR/$ASSET" -C "$STAGING"
  if [[ ! -x "$STAGING/VIVEHub-Linux/run-vivehub.sh" ]]; then
    echo "Unexpected VIVE Hub archive layout" >&2
    exit 1
  fi
  mv "$STAGING/VIVEHub-Linux" "$DESTINATION"
fi

mkdir -p "$ROOT/config/local"
if [[ ! -f "$ROOT/config/local/tracker_mapping.vive_linux.json" ]]; then
  cp "$ROOT/config/tracker_mapping.vive_linux.example.json" \
    "$ROOT/config/local/tracker_mapping.vive_linux.json"
fi
if [[ ! -f "$ROOT/config/local/coordinates.vive_linux_fullbody.json" ]]; then
  cp "$ROOT/config/coordinates.vive_linux_fullbody.example.json" \
    "$ROOT/config/local/coordinates.vive_linux_fullbody.json"
fi

echo "VIVE Hub installed without modifying its files: $DESTINATION"
echo
echo "One-time system step (requires your sudo password):"
echo "  sudo apt install libgl1 libx11-6 libgl1-mesa-dri"
echo "  sudo $DESTINATION/install-udev.sh"
if [[ $ARCH == arm64 ]]; then
  echo "  sudo $DESTINATION/install-box64.sh"
fi
echo "Then unplug/replug the VIVE Wireless Dongle."
echo
echo "Start the Hub with:"
echo "  bash $ROOT/scripts/run_vive_linux_hub.sh --background"
