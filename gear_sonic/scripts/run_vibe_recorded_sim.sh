#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ORCS_ROOT="${ORCS_ROOT:-$HOME/ViBe-SONIC}"
export VIBE_PYTHON="${VIBE_PYTHON:-$ORCS_ROOT/.venv/bin/python}"
export PYTHONPATH="$ORCS_ROOT/vibe/src:$ORCS_ROOT/src:${VIBE_ORT_ROOT:-/tmp/orcs-split-validation-20261020/runtime}${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="/opt/onnxruntime/lib:/usr/local/cuda-12.1/lib64:/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$VIBE_PYTHON" -c 'import onnxruntime, mujoco' || {
    echo 'Missing inference dependencies. Set VIBE_ORT_ROOT to the existing ONNX Runtime installation.' >&2
    exit 2
}
RUN_ROOT="${VIBE_RUN_ROOT:-$HOME/sonic-diagnostics/groot-vibe-v1}"
BUNDLE="${VIBE_BUNDLE:-$HOME/orcs-exports/stairs_split_v1/focus/split_sonic}"
VISION="${VIBE_VISION:-$HOME/orcs-exports/stairs_split_v1/vision/theia-tiny.onnx}"
CONFIG="${VIBE_OBS_CONFIG:-$RUN_ROOT/observation_config.json}"
LIBRARY="${VIBE_TRT_LIBRARY:-$HOME/sonic-diagnostics/vibe-deploy-build-v1/libvibe_trt.so}"
OUT="${VIBE_OUT:-$RUN_ROOT/run_$(date +%Y%m%d_%H%M%S)_$$}"
cd "$ROOT/gear_sonic_deploy"
echo "Recorded SMPL visual simulation only; no PICO, DDS, planner or hardware output."
echo "Output: $OUT"
exec ./deploy.sh --policy-type vibe --bundle "$BUNDLE" --vision-encoder "$VISION" \
    --obs-config "$CONFIG" --input-type recorded_smpl --backend trt --trt-library "$LIBRARY" \
    --engine-cache "$RUN_ROOT/check/engine_cache" --out "$OUT" --realtime "$@" sim
