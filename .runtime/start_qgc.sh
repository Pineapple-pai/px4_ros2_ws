#!/usr/bin/env bash
set -euo pipefail
if pgrep -f "QGroundControl|/home/p/下载/PX4/qgc-daily-root/squashfs-root/AppRun" >/dev/null 2>&1; then
  echo "QGroundControl is already running; not starting a second instance."
  exit 0
fi
cd "/home/p/下载/PX4/qgc-daily-root/squashfs-root"
exec env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY ./AppRun
