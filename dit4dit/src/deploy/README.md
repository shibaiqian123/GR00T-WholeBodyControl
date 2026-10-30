# DiT4DiT test-time RTC deployment

Canonical source on229: `/home/cqu023/GR00T-WholeBodyControl/dit4dit/src/deploy`.
32 runtime copy: `/share/2025slx/DIT4DIT/rtc_deploy`.
This directory is independent of the old five-terminal launcher. The saved-observation client is read-only. The separate live client creates command publishers only with explicit --execute; see the live startup section below. `readonly_client.py` consumes actions into a JSONL log only. It does **not** implement SONIC execution or authorization to move a robot.

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

## Psi0-style launch (added)

The existing `start_server.sh` now also accepts environment-only startup: explicitly set CHECKPOINT_DIR, CHECKPOINT_STEP and POLICY_GPU_UUID, then run `bash start_server.sh`. Its environment-only defaults are port8014, client execution horizon30, loopback host, and policydit4dit. The prior explicit-argument invocation remains compatible (serve.py defaults remain8000/25).

`serve.py --host 127.0.0.1 --port 8014 --action_exec_horizon 30 --policy dit4dit --rtc --run-dir "$CHECKPOINT_DIR" --ckpt-step "$CHECKPOINT_STEP" --cosmos "$COSMOS_PATH"` is supported. Step19000 resolves strictly to `checkpoints/steps_19000_pytorch_model.pt`; there is no latest/fallback checkpoint. `--rtc` explicitly selects the already-default test-time path, not train-time RTC.

Set the229 readonly client to `--url ws://127.0.0.1:8014/ws --action-exec-horizon 30` and tunnel8014 to8014. `/info` advertises the configured horizon and the server rejects premature replans. A window30 has20 frames remaining; previous validation used25, so it does not certify the30-frame real-network latency budget. Invalid delay/overlap still fails; no timeout is widened.

`--host 0.0.0.0` is supported if deliberately exposing the server directly. This unauthenticated diagnostic endpoint defaults to loopback + SSH forwarding.

## 实时客户端：Psi0 顺序（229）

本节使用 `run_dit4dit_rtc_sonic_dex1.py`；上面的 `readonly_client.py` 仍是保存文件回放入口。
不同时运行旧 gateway、独立 Dex1 命令服务或另一个 RTC 客户端。

1. 启动相机/本体观测和32推理服务，再建立8014 SSH隧道。229执行 `curl --fail http://127.0.0.1:8014/info` 核对 state31、action50x66、test_time及资源哈希。
2. 启动 SONIC V1.1：`bash dit4dit/script/deploy_dit4dit_sonic_robot.sh --interface enx6c1ff7b8d821`。初始化后按 `]`，等待默认参考控制稳定；先不要按 Enter。CONTROL并不意味着默认参考的跟踪稳定，仍需现场确认。更新后须使用新编译的底层，旧心跳缺少流状态将被拒绝。
3. 在229启动客户端（每个进程保持独立终端）：

```bash
cd /home/cqu023/GR00T-WholeBodyControl
SONIC_CONDA_PREFIX="$PWD/.venv_teleop" \
bash dit4dit/script/deploy_dit4dit-sonic-rtc-client.sh \
  --host 127.0.0.1 --port 8014 \
  --camera-port 5555 --body-port 5564 \
  --network enx6c1ff7b8d821 \
  --instruction 'Pick up the cup on the left with your left gripper. Then pick up the red block on the right with your right gripper and place it into the cup held in your left gripper. Finally, place the cup, with the red block inside, back on the table using your left gripper.' \
  --output "$PWD/outputs/rtc_execute_$(date +%Y%m%d_%H%M%S)" \
  --execute
```

默认不传 `--execute` 是只读实时推理，不创建动作发布者。`--check` 只检查依赖/标定，不打开相机、DDS或网络连接。启动脚本是本机已有包装器；亦可直接用 `.venv_teleop/bin/python dit4dit/src/deploy/run_dit4dit_rtc_sonic_dex1.py` 配相同参数。

4. 客户端自动检查 CONTROL + external_stream=0 并初始化夹爪保持，无需 `arm/run`。等待 `GRIPPERS_READY`、`Received action` 和 `TOKEN_PUBLISHING`，且没有故障，再在C++终端按 Enter。确认C++打印 `ZMQ STREAMING MODE: ENABLED`、客户端报告 `EXTERNAL_STREAM: 1`。等待期间动作帧持续消费/重规划，不固定第一帧；开启流后夹爪采用当前消费帧，不补发旧夹爪动作。

`external_stream` 心跳：-1未知、0默认参考、1外部流。仅普通zmq输入报告确定值；模式状态不是首个token采用/执行确认。事件记录心跳时间、观察时间与心跳年龄，不将心跳年龄当成完整切换延迟，也不宣称身体和夹爪硬件同步。

首次预测若超过原0.5秒新鲜度门限，且尚未发布动作/外部流关闭，允许仅一次作为预热结果丢弃；等待预热结束后拍摄的新观测，关闭旧连接并建立新会话普通采样，丢弃所有历史。第二次仍过期、网络失败或正式执行后过期均锁定；不扩大0.5/1秒门限或10秒请求超时。`warmup_discarded.json` 保留丢弃结果及年龄。

正常结束：在C++终端按 Enter返回默认参考；客户端看到流关闭后停止新token和推理请求，进入既有受保护夹爪保持并锁定。确认现场状态后Ctrl+C客户端，再按O退出底层。紧急情况使用既有O/硬件急停流程。默认参考切换、FAULT_HOLD及停止后的支撑并未验证为安全状态。

客户端 `stop` 停止发布并锁定，`quit`/Ctrl+C关闭资源；关闭后夹爪软件保持结束。自动断网停控尚未闭环，客户端停止发布不等于SONIC停止控制。首目标连续性、实际采用帧确认仍未解决；本轮只有静态/内存与编译验证，没有实机验收。
