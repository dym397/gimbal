#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

exec python3 "$SCRIPT_DIR/main_tracking_v9.py"
