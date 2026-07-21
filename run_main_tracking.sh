#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

# Do not start Python until all RID-version serial devices are present.  The
# Python process creates a timestamped run directory immediately, so letting
# systemd restart it while hardware is absent would generate one empty log
# directory every RestartSec interval.
GIMBAL_PORT="${GIMBAL_PORT:-/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.2:1.0-port0}"
GPS_PORT="${GPS_PORT:-/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.1:1.0-port0}"
RID_PORT="${RID_PORT:-/dev/serial/by-path/platform-xhci-hcd.4.auto-usb-0:1.2.3:1.0-port0}"
HARDWARE_WAIT_INTERVAL="${HARDWARE_WAIT_INTERVAL:-2}"
HARDWARE_WAIT_LOG_INTERVAL="${HARDWARE_WAIT_LOG_INTERVAL:-30}"

last_missing=""
last_log_ts=0
while true; do
    missing=()
    for device in \
        "gimbal:${GIMBAL_PORT}" \
        "gps:${GPS_PORT}" \
        "rid:${RID_PORT}"; do
        role="${device%%:*}"
        path="${device#*:}"
        if [[ -z "$path" || ! -c "$path" || ! -r "$path" || ! -w "$path" ]]; then
            missing+=("${role}=${path:-unset}")
        fi
    done

    if (( ${#missing[@]} == 0 )); then
        echo "[HardwareWait] ready: gimbal=${GIMBAL_PORT}, gps=${GPS_PORT}, rid=${RID_PORT}"
        break
    fi

    missing_text="${missing[*]}"
    now_ts="$(date +%s)"
    if [[ "$missing_text" != "$last_missing" ]] \
        || (( now_ts - last_log_ts >= HARDWARE_WAIT_LOG_INTERVAL )); then
        echo "[HardwareWait] waiting; missing/unusable: ${missing_text}"
        last_missing="$missing_text"
        last_log_ts="$now_ts"
    fi
    sleep "$HARDWARE_WAIT_INTERVAL"
done

export GIMBAL_PORT GPS_PORT RID_PORT
exec python3 "$SCRIPT_DIR/main_tracking_v9.py"
