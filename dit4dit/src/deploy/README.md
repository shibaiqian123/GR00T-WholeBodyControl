# DiT4DiT test-time RTC deployment (read-only validation stage)

Canonical source on229: `/home/cqu023/GR00T-WholeBodyControl/dit4dit/src/deploy`.
32 runtime copy: `/share/2025slx/DIT4DIT/rtc_deploy`.
This directory is independent of the old five-terminal launcher. No file here creates a DDS publisher, ZMQ robot command socket, or motor controller. `readonly_client.py` consumes actions into a JSONL log only. It does **not** implement SONIC execution or authorization to move a robot.

## Sampling contract

`RTCActionHead.predict_action_rtc` wraps the existing frozen DiT4DiT action head without changing its state_dict or trained architecture. It reuses its state encoder, action encoder, positional embedding, cross-attention model and decoder. Original model code / ordinary predict_action remain unchanged. The deployment framework adapter selects this independent sampler; it does not monkey-patch the old server.

Checkpoint resources must be explicit. Current contract is raw RGB image + original task text + raw `state[31]` -> normalized `50x66`. Use training state order and stats. The model's four Euler steps (noise time1 to0) remain unchanged. RTC guides **all66 normalized channels**. Training denormalization clips normalized values to[-1,1], applies min/max, and splits `motion_tokens[50,64]` and `gripper_actions[50,2]`. The clipping/out-of-range fraction is reported, not hidden. The 64 channels are checkpoint-specific latent actions, not joint angles.

Psi0's exponential mask and VJP are ported in rtc_sampler.py. For `x0=x-t*v`, `e=(previous-x0.detach())*mask`, compute `J(x0,x)^T e`; subtract the scaled correction from v, then take the original negative-time Euler step. Correction is zero at t=1 or near-zero VJP norm. Weight-one prefix is **soft guidance, not exact freezing**. No token interpolation or pseudo current-pose token is used. First chunk is ordinary inference.

Weights are frozen. Backbone and state features are detached. Only the per-step action sample carries gradients, and each step discards its graph. Do not wrap RTC in `torch.inference_mode()`. The original ordinary path is retained for baseline comparisons.

## Wire and timing

HTTP `/health`, `/info`; WebSocket `/ws`; server binds loopback8000. One active owner and one in-flight inference. An independent `observe` message replaces the single latest-observation slot. An `infer` request freezes its own observation and consumption position together; observations arriving mid-forward cannot change that request.

An observation has `image_png` (base64 PNG), `lang`, raw `state[31]`, `observation_id`, and `observed_at` (opaque client timestamp). An inference request adds `op=infer`, `session`, `request_id`, `chunk_id` (null initially), `consumed` (next unread index), `delay` (estimated consumed ticks during round trip). A reply includes the same request/session identity, new chunk_id, normalized actions, split denormalized outputs, source observation identity/time, resource hashes, inference duration, estimated delay and window_offset. No cross-host monotonic timestamp subtraction.

229's ConsumptionClock triggers after `action_exec_horizon` (default25) consumed rows. At request, `s` is actual consumed count; server shifts old normalized actions by s and pads the s-row tail with zero. That tail has **zero guidance**. Validate `0<=d<=s<=H-d` and s<H; no silent clamps. Delay window is six samples, initially10. On response, skip exactly the number of rows the client consumed while the request was pending; those rows belong to the committed old block. No burst catch-up. Client late by a full20ms tick fails instead of speeding up. Old-block exhaustion, expired response, invalid overlap or nonfinite output is terminal for that session. No infinite last-token hold.

Consumer progress is **not motor execution acknowledgement**. Future SONIC integration must preserve this distinction and its own stop/freshness checks. Saved-observation replay has neither live observation freshness nor closed-loop validity. RTC does not solve the initial physical takeover mismatch.

## Server32

Install only in an isolated environment layered on the existing model environment:

```bash
/share/2025slx/miniconda3/envs/dit4dit/bin/python -m venv --system-site-packages /share/2025slx/DIT4DIT/.venv_rtc_deploy
/share/2025slx/DIT4DIT/.venv_rtc_deploy/bin/python -m pip install --index-url https://pypi.org/simple -r /share/2025slx/DIT4DIT/rtc_deploy/requirements-server.txt
export DIT4DIT_ROOT=/share/2025slx/DIT4DIT
export RTC_PYTHON="$DIT4DIT_ROOT/.venv_rtc_deploy/bin/python"
# Choose a currently available A100 UUID from nvidia-smi, then set POLICY_GPU_UUID.
bash "$DIT4DIT_ROOT/rtc_deploy/start_server.sh" \
  --run "$DIT4DIT_ROOT/remote_policy/run19000" \
  --checkpoint steps_19000_pytorch_model.pt \
  --cosmos /share/2025slx/models/Cosmos-Predict2.5-2B
```

Port collision fails before model loading. No existing process is stopped. The service reports RTC_READY after resource loading; `/info` describes actual resources. Never infer a checkpoint from directory ordering. Source modifications to serve.py require restarting only this new service.

## Read-only client229

Tunnel in a separate terminal (don't create a duplicate):

```bash
ssh -N -T -o ExitOnForwardFailure=yes -L 127.0.0.1:8000:127.0.0.1:8000 2025slx@10.246.1.32
```

Use the isolated `.venv_rtc_client` environment with numpy,Pillow,websockets (no torch needed for the client). Install via `.venv_rtc_client/bin/python -m pip install -r dit4dit/src/deploy/requirements-client.txt`. Set `CAPTURE` to an existing saved capture containing manifest.json and observation.npz, and `RTC_OUTPUT` to a new directory:

```bash
cd /home/cqu023/GR00T-WholeBodyControl
.venv_rtc_client/bin/python dit4dit/src/deploy/readonly_client.py \
  --capture "$CAPTURE" --output "$RTC_OUTPUT" --blocks 3
```

Success marker: `READONLY_RTC_COMPLETE; NO MOTOR SEND`. Failure is recorded in events.jsonl; no automatic retry or threshold widening. The first response deadline is10seconds, matching the older offline request deadline; model loading happens before listening. Saved input is explicitly labeled, not presented as live control data.

## Tests and evidence

```bash
cd /share/2025slx/DIT4DIT/rtc_deploy
../.venv_rtc_deploy/bin/python -m unittest test_rtc test_service -v
```

`validate_checkpoint.py` accepts explicit --run, --checkpoint, --cosmos, --capture, --output. It fixes the zero-guidance parity tolerance at atol=rtol=0 before running, uses identical random noise, compares plain/RTC overlap RMSE and timing across four paired requests, records allocated CUDA memory and parameter gradients. Its repeated saved observation is an offline sampler comparison, **not** an executed robot trajectory. Allocated memory stability covers that short test only, not an indefinite leak proof.

`extract_validation_sample.py` can extract episode0/frame0 from the existing converted dataset using its recorded observation.sonic_state and first video frame. It records hashes, task, episode/frame and dataset-relative timestamp. It never substitutes zeros for missing data. Yesterday's missing manual capture is not reconstructed by this fixture.

See VALIDATION.md for measured results. Old workflows, checkpoints, data and SONIC protection settings are unchanged. Retirement of old entrypoints requires a later integration acceptance step.
