#!/usr/bin/env bash
set -euo pipefail

WBWAM_CONDA_PYTHON="${WBWAM_CONDA_PYTHON:-/share/2025slx/miniconda3/envs/wbwam/bin/python}"
DEPLOY_ENV="${WBWAM_DEPLOY_ENV:-/share/2025slx/GR00T-WholeBodyControl/WB-WAM/.venv_deploy}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

[[ -x "$WBWAM_CONDA_PYTHON" ]] || { echo "missing WB-WAM Python: $WBWAM_CONDA_PYTHON" >&2; exit 2; }
"$WBWAM_CONDA_PYTHON" -m venv --system-site-packages "$DEPLOY_ENV"
"$DEPLOY_ENV/bin/python" -m pip install \
  --index-url "${PIP_INDEX_URL:-https://pypi.org/simple}" \
  -r "$SCRIPT_DIR/../deploy/requirements-server.txt"
"$DEPLOY_ENV/bin/python" - <<'PY'
import fastapi, torch, uvicorn, websockets
print("WBWAM_DEPLOY_ENV_OK", torch.__version__, fastapi.__version__)
PY
