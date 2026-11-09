# WB-WAM 31->66 SONIC 部署

这个目录复用 GROOT 的相机、身体 observer、SONIC V1.1 和 Dex1 控制客户端，只新增 WB-WAM 的 32 端模型服务。

服务加载的是完整 WB-WAM SONIC checkpoint：原生模型输出 `[32,66]`，对应 20 Hz 数据合同。为了兼容现有 SONIC 客户端的 `[50,66]`、50 Hz 协议，服务按执行时间轴对每一行做最近邻保持（zero-order hold），不对 64 维 latent 线性插值。50 个 50 Hz tick 覆盖 0.98 秒，对应原生行 0～19；不会把 1.6 秒的 32 行强行拉伸到 1 秒。`/info` 会报告 `time_adapter`；这只是部署适配，不等价于重新训练 50 Hz 模型，正式真机验收前必须单独验证速度和动作连续性。

## 32：启动 WB-WAM server

首次在 32 安装隔离服务环境（继承现有 `wbwam` 环境的 PyTorch/Wan 依赖）：

```bash
cd /share/2025slx/GR00T-WholeBodyControl/WB-WAM
bash scripts/setup_server_env.sh
```

```bash
cd /share/2025slx/GR00T-WholeBodyControl/WB-WAM

export WBWAM_ROOT=/share/2025slx/WAM
export WBWAM_RUN_DIR=/share/2025slx/WAM/training/runs/sonic_dex1_20hz/sonic66_100k_20261108_191251
export WBWAM_CHECKPOINT_STEP=5000
export WBWAM_TEXT_CACHE=/share/2025slx/WAM/data/sonic_datasets/block_into_cup_v1_1_dex1_wbwam_20hz_s31_a66/text_embeds_cache/ddda90a3cf143c1a9a43600df63834647de463969b8c98b1af33977389a23aa0.t5_len192.sonic31_latent66_20hz_v1.pt
export CUDA_VISIBLE_DEVICES=GPU-0a4d91c9-014c-c83e-d8d0-b7c7fdc2738a
export WBWAM_ENABLE_PREVIEW_SOURCE=1
export PYTHON_BIN="$PWD/.venv_deploy/bin/python"

bash scripts/start_server.sh
```

启动后，32 本机检查：

```bash
curl --fail --silent http://127.0.0.1:8014/info | python -m json.tool
```

必须看到 `state_dim=31`、`native_action_shape=[32,66]`、`action_shape=[50,66]`、`rtc_mode=disabled` 和 `time_adapter=zero_order_hold_latest_20hz_row_on_50hz_ticks`。

## 229：相机、SONIC 和客户端

相机服务：

```bash
cd /home/cqu023/GR00T-WholeBodyControl/WB-WAM
bash scripts/start_camera.sh
```

查看画面：

```bash
bash scripts/view_camera.sh
```

建立 32 服务通道：

```bash
ssh -N -T -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8014:127.0.0.1:8014 \
  2025slx@10.246.1.32
```

启动 SONIC：

```bash
cd /home/cqu023/GR00T-WholeBodyControl
bash dit4dit/script/deploy_dit4dit_sonic_robot.sh \
  --interface enx6c1ff7b8d821
```

机器人端 Dex1 服务仍按已有 GROOT 流程启动。最后在 229 的交互终端运行：

```bash
cd /home/cqu023/GR00T-WholeBodyControl/WB-WAM
bash scripts/run_sonic_client.sh \
  --host 127.0.0.1 \
  --port 8014 \
  --camera-port 5555 \
  --body-port 5564 \
  --network enx6c1ff7b8d821 \
  --instruction 'Pick up the cup on the left with your left gripper. Then pick up the red block on the right with your right gripper and place it into the cup held in your left gripper. Finally, place the cup, with the red block inside, back on the table using your left gripper.' \
  --output "$PWD/outputs/wbwam_execute_$(date +%Y%m%d_%H%M%S)" \
  --execute
```

复用的客户端会先检查 `CONTROL`、Dex1 READY、观测新鲜度、`external_stream=0` 和 server 资源信息；只有 `--execute` 才发布动作。它仍然保留 GROOT 的故障锁、Dex1 限位和手动 Enter 切换流程。

## 只读检查

去掉 `--execute` 即为只读实时推理，不创建动作发布器：

```bash
bash scripts/run_sonic_client.sh ...
```

当前服务的 RTC 是 `disabled`。不能把 DIT4DIT 的 RTC guidance 直接套在 Wan2.2 flow-matching 模型上。

服务不截断或量化 latent；如果输出 token 绝对值超过 1.25，则请求失败并由客户端锁定，不把异常 token 交给 SONIC。

## 训练 checkpoint 和预览

`WBWAM_CHECKPOINT_STEP=5000` 要求运行目录存在 `checkpoints/weights/step_005000.pt`、`config.yaml` 和 `dataset_stats.json`。服务只接受带 `sonic_contract` 的完整 SONIC checkpoint。

当前 WB-WAM 动作服务使用 action-only 路径。训练运行中的 `eval/*.mp4` 是训练评估视频，不是在线预测视频。独立未来视频预览使用第二张空闲 GPU：

```bash
cd /share/2025slx/GR00T-WholeBodyControl/WB-WAM
export CUDA_VISIBLE_DEVICES=GPU-ea98a495-2d03-62eb-779f-4a0cef804025
"$PWD/.venv_deploy/bin/python" deploy/serve_preview.py \
  --run-dir "$WBWAM_RUN_DIR" \
  --ckpt-step "$WBWAM_CHECKPOINT_STEP" \
  --wbwam-root "$WBWAM_ROOT" \
  --text-cache "$WBWAM_TEXT_CACHE" \
  --gpu-uuid "$CUDA_VISIBLE_DEVICES" \
  --port 8015
```

它调用 `infer_joint` 生成 9 帧视频，只用于诊断，不创建动作发布器。在 32 本机打开 `http://127.0.0.1:8015`；从本机查看时另行转发 8015。

延迟事件继续兼容现有 `tools/g1_latency_trace/latency_trace.py`。设置 `LATENCY_TRACE_UDP=127.0.0.1:19555` 和对应收集/转发进程后，可沿用原报告命令；未启用 UDP 收集时不会生成完整延迟报告。
