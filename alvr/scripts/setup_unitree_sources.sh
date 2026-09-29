#!/usr/bin/env bash
set -euo pipefail

alvr_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
repo_root="$(cd -- "$alvr_root/.." && pwd)"
vendor_root="${VENDOR_ROOT:-$repo_root/vendor}"

xr_revision="${XR_TELEOPERATE_REVISION:-817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b}"
sim_revision="${UNITREE_SIM_REVISION:-e30c25b1dffdf92ada1d6c8c1fe9a47bdde0fecc}"
sdk_revision="${UNITREE_SDK_REVISION:-814556d15970dd2ecf1c9984e845ca02ab07e206}"

command -v git >/dev/null || { echo "git is required" >&2; exit 2; }
mkdir -p "$vendor_root"

clone_pinned() {
    local url="$1"
    local destination="$2"
    local revision="$3"
    local label="$4"

    if [[ -e "$destination/.git" ]]; then
        local current
        current="$(git -C "$destination" rev-parse HEAD)"
        if [[ "$current" != "$revision" ]]; then
            echo "$label revision mismatch; refusing to continue." >&2
            echo "  expected: $revision" >&2
            echo "  current:  $current" >&2
            echo "Move the existing checkout aside or check out the expected revision explicitly:" >&2
            echo "  $destination" >&2
            exit 2
        fi
        echo "$label already exists at the pinned revision: $current"
        return
    fi
    if [[ -e "$destination" ]]; then
        echo "refusing to overwrite non-git path: $destination" >&2
        exit 2
    fi

    echo "Cloning $label..."
    GIT_LFS_SKIP_SMUDGE=1 git clone --filter=blob:none --no-checkout "$url" "$destination"
    git -C "$destination" fetch --depth 1 origin "$revision"
    git -C "$destination" checkout --detach "$revision"
}

clone_pinned \
    "https://github.com/unitreerobotics/xr_teleoperate.git" \
    "$vendor_root/xr_teleoperate" \
    "$xr_revision" \
    "xr_teleoperate"
clone_pinned \
    "https://github.com/unitreerobotics/unitree_sim_isaaclab.git" \
    "$vendor_root/unitree_sim_isaaclab" \
    "$sim_revision" \
    "unitree_sim_isaaclab"
clone_pinned \
    "https://github.com/unitreerobotics/unitree_sdk2_python.git" \
    "$vendor_root/unitree_sdk2_python" \
    "$sdk_revision" \
    "unitree_sdk2_python"

echo "Initializing pinned submodules..."
GIT_LFS_SKIP_SMUDGE=1 git -C "$vendor_root/xr_teleoperate" submodule update --init --depth 1
GIT_LFS_SKIP_SMUDGE=1 git -C "$vendor_root/unitree_sim_isaaclab" submodule update --init --depth 1

echo
echo "Sources are ready under: $vendor_root"
echo "Large Isaac assets are intentionally not downloaded by this script."
echo "Follow unitree_sim_isaaclab/README.md, or run its auto_setup_env.sh, before teleoperation."
