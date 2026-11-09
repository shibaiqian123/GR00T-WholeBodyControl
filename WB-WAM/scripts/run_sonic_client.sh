#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GROOT_ROOT="${GROOT_ROOT:-$(cd -- "$SCRIPT_DIR/../.." && pwd)}"
SONIC_CONDA_PREFIX="${SONIC_CONDA_PREFIX:-$GROOT_ROOT/.venv_teleop}"
CLIENT="$GROOT_ROOT/dit4dit/src/deploy/run_dit4dit_rtc_sonic_dex1.py"

[[ -x "$SONIC_CONDA_PREFIX/bin/python" ]] || { echo "missing SONIC client Python: $SONIC_CONDA_PREFIX/bin/python" >&2; exit 2; }
[[ -f "$CLIENT" ]] || { echo "missing GROOT RTC client: $CLIENT" >&2; exit 2; }

HOST=127.0.0.1
PORT=8014
args=("$@")
for ((i=0; i<${#args[@]}; i++)); do
  case "${args[$i]}" in
    --host) HOST="${args[$((i+1))]}" ;;
    --port) PORT="${args[$((i+1))]}" ;;
  esac
done
PYTHONPATH="$GROOT_ROOT/WB-WAM/deploy${PYTHONPATH:+:$PYTHONPATH}" \
  "$SONIC_CONDA_PREFIX/bin/python" "$GROOT_ROOT/WB-WAM/deploy/check_server.py" \
  --host "$HOST" --port "$PORT"

exec "$SONIC_CONDA_PREFIX/bin/python" -u "$CLIENT" "$@"
