#!/usr/bin/env bash
# Domain-38 simulation bottom only. No automatic planner start or token send.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
"$ROOT/.venv_teleop/bin/python" - <<'PY_CHECK'
import socket
for port in (15557,15558):
    with socket.socket() as s:
        s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try:s.bind(("127.0.0.1",port))
        except OSError:raise SystemExit(f"Simulation port {port} occupied; nothing started. Stop only the old simulation bottom first.")
PY_CHECK
RUN="$(mktemp -d "$ROOT/outputs/sim_single_step_enabled_XXXXXXXX")"
"$ROOT/.venv_teleop/bin/python" -m gear_sonic.scripts.prepare_sim_single_step --output "$RUN/config.json"
echo "SIM_RUN=$RUN"
cd "$ROOT/gear_sonic_deploy"
exec env -u CYCLONEDDS_URI -u VIBE_CONFIG -u VIBE_HYBRID_ENDPOINT \
  SONIC_STARTUP_AUDIT="$RUN/bottom.jsonl" \
  SONIC_READONLY_DECODE_SNAPSHOT=1 \
  SONIC_SINGLE_STEP_CONFIG="$RUN/config.json" \
  ./target/release/g1_deploy_onnx_ref lo \
  policy/sonic_v1_1/model_decoder.onnx reference/example/ \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --encoder-file policy/sonic_v1_1/model_encoder.onnx \
  --planner-file planner/target_vel/V2/planner_sonic.onnx \
  --input-type zmq_manager --output-type zmq \
  --zmq-host 127.0.0.1 --zmq-port 15556 \
  --zmq-out-host 127.0.0.1 --zmq-out-port 15557 \
  --dds-domain 38 --disable-crc-check --enable-dex1-heartbeat \
  --motor-kp-scale "4,10=1.5" --motor-kd-scale "4,10=1.5"
