#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_ROOT="$(cd -- "$SCRIPT_DIR/../deploy" && pwd)"
WBWAM_ROOT="${WBWAM_ROOT:-/share/2025slx/WAM}"
RUN_DIR="${WBWAM_RUN_DIR:?Set WBWAM_RUN_DIR to a completed WB-WAM run directory}"
TEXT_CACHE="${WBWAM_TEXT_CACHE:?Set WBWAM_TEXT_CACHE to the exact 192x4096 prompt cache .pt}"
CHECKPOINT_STEP="${WBWAM_CHECKPOINT_STEP:-5000}"
PYTHON_BIN="${PYTHON_BIN:-python}"

if [[ -f "$WBWAM_ROOT/training/scripts/load_env.sh" ]]; then
  # shellcheck source=/dev/null
  source "$WBWAM_ROOT/training/scripts/load_env.sh"
fi

export PYTHONPATH="$DEPLOY_ROOT:$WBWAM_ROOT/training/src:$WBWAM_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export DIFFSYNTH_SKIP_DOWNLOAD=true

extra_args=()
if [[ -n "${WBWAM_ENABLE_PREVIEW_SOURCE:-}" ]]; then
  extra_args+=(--enable-preview-source)
fi

exec "$PYTHON_BIN" -u "$DEPLOY_ROOT/serve.py" \
  --host "${WBWAM_SERVER_HOST:-127.0.0.1}" \
  --port "${WBWAM_SERVER_PORT:-8014}" \
  --action-exec-horizon "${WBWAM_ACTION_EXEC_HORIZON:-10}" \
  --run-dir "$RUN_DIR" \
  --ckpt-step "$CHECKPOINT_STEP" \
  --wbwam-root "$WBWAM_ROOT" \
  --text-cache "$TEXT_CACHE" \
  --num-inference-steps "${WBWAM_INFERENCE_STEPS:-10}" \
  --seed "${WBWAM_SEED:-42}" \
  "${extra_args[@]}"
