#!/usr/bin/env bash
set -euo pipefail

SDK_DIR="${SDK_DIR:-$HOME/Livox-SDK2}"
SDK_REPO="${SDK_REPO:-https://github.com/Livox-SDK/Livox-SDK2.git}"
BUILD_DIR="${BUILD_DIR:-${SDK_DIR}/build}"
LIVOX_SDK2_PREFIX="${LIVOX_SDK2_PREFIX:-$HOME/.local/livox_sdk2}"

echo "[INFO] Installing Livox-SDK2 from: ${SDK_REPO}"
echo "[INFO] Target source directory: ${SDK_DIR}"
echo "[INFO] Install prefix: ${LIVOX_SDK2_PREFIX}"

if [[ -d "${SDK_DIR}/.git" ]]; then
  echo "[INFO] Existing Livox-SDK2 checkout found, updating..."
  git -C "${SDK_DIR}" pull --ff-only
else
  rm -rf "${SDK_DIR}"
  git clone "${SDK_REPO}" "${SDK_DIR}"
fi

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"
cmake .. -DCMAKE_INSTALL_PREFIX="${LIVOX_SDK2_PREFIX}"
make -j"$(nproc)"
make install

if [[ -f "${LIVOX_SDK2_PREFIX}/lib/liblivox_lidar_sdk_shared.so" ]]; then
  echo "[ OK ] Livox-SDK2 installed under ${LIVOX_SDK2_PREFIX}"
else
  echo "[FAIL] Livox-SDK2 install finished, but liblivox_lidar_sdk_shared.so was not found under ${LIVOX_SDK2_PREFIX}/lib"
  exit 1
fi

echo "[INFO] Next step:"
echo "  cd /home/p/px4_ros2_ws"
echo "  export LIVOX_SDK2_PREFIX=${LIVOX_SDK2_PREFIX}"
echo "  ./scripts/check_planning_prereqs.sh"
