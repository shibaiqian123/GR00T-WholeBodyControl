#!/usr/bin/env bash
set -euo pipefail
: "${DIT4DIT_ROOT:?Set DIT4DIT_ROOT to the DiT4DiT model repository}"
: "${RTC_PYTHON:?Set RTC_PYTHON to the isolated RTC environment python}"
: "${POLICY_GPU_UUID:?Select an available GPU UUID explicitly}"
export PYTHONPATH="$DIT4DIT_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export CUDA_VISIBLE_DEVICES="$POLICY_GPU_UUID"
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
exec "$RTC_PYTHON" -u "$(dirname "$0")/serve.py" "$@"
