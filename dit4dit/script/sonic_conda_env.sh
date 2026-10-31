#!/usr/bin/env bash
# Shared Python environment for this checkout's DiT4DiT -> SONIC wrappers.
# Reference: https://github.com/physical-superintelligence-lab/Psi0/blob/main/real/SONIC/scripts/sonic_conda_env.sh
# Usage: source /home/cqu023/GR00T-WholeBodyControl/dit4dit/script/sonic_conda_env.sh
# Reuses conda sonic; does not install packages or start any robot process.
# Do not change the caller's shell options when sourced.

SONIC_ENV_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || { return 1 2>/dev/null || exit 1; }
export SONIC_DIR="${SONIC_DIR:-$(cd -- "$SONIC_ENV_SCRIPT_DIR/../.." && pwd)}"
export SONIC_DEPLOY_DIR="${SONIC_DEPLOY_DIR:-$SONIC_DIR/gear_sonic_deploy}"
export DIT4DIT_DEPLOY_DIR="${DIT4DIT_DEPLOY_DIR:-$SONIC_DIR/dit4dit/src/deploy}"
# DIT4DIT_ROOT belongs to the separate training/server checkout; leave it alone.

if [[ -z "${SONIC_CONDA_PREFIX:-}" ]]; then
    if [[ "${CONDA_DEFAULT_ENV:-}" == "sonic" && -n "${CONDA_PREFIX:-}" ]]; then
        SONIC_CONDA_PREFIX="$CONDA_PREFIX"
    else
        SONIC_CONDA_PREFIX="$HOME/miniconda3/envs/sonic"
    fi
fi
export SONIC_CONDA_PREFIX
export SONIC_PYTHON="$SONIC_CONDA_PREFIX/bin/python"
if [[ ! -x "$SONIC_PYTHON" ]]; then
    printf 'Missing conda sonic Python: %s\nSet SONIC_CONDA_PREFIX to an existing sonic environment.\n' "$SONIC_PYTHON" >&2
    return 1 2>/dev/null || exit 1
fi
if [[ ! -d "$SONIC_DIR/gear_sonic" || ! -d "$SONIC_DEPLOY_DIR" || ! -d "$DIT4DIT_DEPLOY_DIR" ]]; then
    printf 'Invalid SONIC/DiT4DiT checkout: %s\n' "$SONIC_DIR" >&2
    return 1 2>/dev/null || exit 1
fi

# Replace inherited IsaacLab/other-env paths, as in the reference script.
# Select Python explicitly; this is not a full `conda activate` operation.
case "$PATH" in
    "$SONIC_CONDA_PREFIX/bin"|"$SONIC_CONDA_PREFIX/bin:"*) ;;
    *) export PATH="$SONIC_CONDA_PREFIX/bin:$PATH" ;;
esac
export PYTHONPATH="$SONIC_DIR:$DIT4DIT_DEPLOY_DIR:$SONIC_DIR/external_dependencies/unitree_sdk2_python"
export LD_LIBRARY_PATH="$SONIC_CONDA_PREFIX/lib:$SONIC_DIR/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib"

# Absolute paths work regardless of the caller's current directory.
# SONIC_CHECKPOINT is a prefix, without _encoder.onnx / _decoder.onnx.
export SONIC_CHECKPOINT="${SONIC_CHECKPOINT:-$SONIC_DEPLOY_DIR/policy/sonic_v1_1/model}"
export SONIC_OBS_CONFIG="${SONIC_OBS_CONFIG:-$SONIC_DEPLOY_DIR/policy/sonic_v1_1/observation_config.yaml}"
# Match deploy_dit4dit_sonic_robot.sh; override explicitly if wiring changes.
export G1_NETWORK_INTERFACE="${G1_NETWORK_INTERFACE:-enx6c1ff7b8d821}"
# No DEX1_VIRTUAL_STATS default: this client uses physical Dex1 motor-radian2,
# not Psi0's virtual hand14 mapping. Existing explicit settings are untouched.
