#!/usr/bin/env bash
# Reference: Psi0 real/scripts/deploy_psi0-sonic-rtc-client.sh
# Forward all options to the existing DiT4DiT RTC client unchanged.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/sonic_conda_env.sh"

exec "$SONIC_PYTHON" -u \
    "$DIT4DIT_DEPLOY_DIR/run_dit4dit_rtc_sonic_dex1.py" \
    "$@"
