#!/usr/bin/env bash
set -euo pipefail
GROOT_ROOT="${GROOT_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)}"
exec "$GROOT_ROOT/.venv_teleop/bin/python" -u -m gear_sonic.scripts.run_model_observers \
  --interface "${ROBOT_INTERFACE:-enx6c1ff7b8d821}"
