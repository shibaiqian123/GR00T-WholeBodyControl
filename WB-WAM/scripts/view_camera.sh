#!/usr/bin/env bash
set -euo pipefail
GROOT_ROOT="${GROOT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)}"
exec "$GROOT_ROOT/.venv_teleop/bin/python" "$GROOT_ROOT/gear_sonic/scripts/run_camera_viewer.py" \
  --camera-host "${CAMERA_HOST:-localhost}" \
  --camera-port "${CAMERA_PORT:-5555}"
