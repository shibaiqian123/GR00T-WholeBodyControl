# 人工分步：实测观测 → DiT第一帧 → SONIC解码后的31维结果

本轮仅编写脚本。每条命令独立执行，不串行自动调用下一步，不启动机器人，不切模式。31维结果布局是 **SONIC硬件顺序29关节目标（rad）+左Dex1目标+右Dex1目标（rad）**。它不是统一DDS电机包；解码命令不发送身体或夹爪动作。

## 1. 保存实测观测

先由现场人员安排机器人站稳。需要现有相机5555、身体只读5564、夹爪反馈5559。此步骤无须模型服务或底层运动控制。

```bash
cd /home/cqu023/GR00T-WholeBodyControl
export STEP_RUN="$PWD/outputs/manual_step_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$STEP_RUN"
.venv_data_collection/bin/python -m gear_sonic.scripts.dit_sonic_steps capture \
  --output "$STEP_RUN/capture" \
  --task-prompt "Pick up the cup on the left with your left gripper. Then pick up the red block on the right with your right gripper and place it into the cup held in your left gripper. Finally, place the cup, with the red block inside, back on the table using your left gripper."
```

成功标志 `CAPTURE_SAVED`；保存 observation.npz（原图、31维实测状态、原时间）、manifest.json。缺反馈、反馈过期或不同观测相差超过150ms时拒绝保存，不补零。若已有底层诊断日志，可加 `--bottom-log /absolute/path/bottom.jsonl`，同时保存完整SONIC快照；否则此时只保存VLA观测。

## 2. 请求一次DiT推理并保存第一帧

32上的推理服务和229本机5560 SSH隧道需已运行。不自动启动、重启服务。

```bash
.venv_data_collection/bin/python -m gear_sonic.scripts.dit_sonic_steps infer \
  --capture "$STEP_RUN/capture" --output "$STEP_RUN/prediction"
```

成功标志 `PREDICTION_SAVED`。保存 normalized.npy、actions.npy（50×66）、token_step0.npy（64）、gripper_step0.npy（2）、原始请求与响应。使用保存的观测，不伪造新的采样时间；这是离线诊断预测，不通过放宽实机新鲜度门槛下发。超时不自动重试，已有输出目录不覆盖。

## 3. 单独保存完整SONIC本体输入

仅“机器人没动”不提供历史last_action；不能仅用LowState的位置拼出真实完整994维。底层现有快照需显式启用，且记录正常运行时已构造的完整输入。

在**底层进程所属终端、人工启动底层之前**设置以下环境变量（不影响当前已运行进程，tmux其他窗口也不会自动继承）：

```bash
export SONIC_READONLY_DECODE_SNAPSHOT=1
export SONIC_STARTUP_AUDIT=/absolute/path/new-bottom.jsonl
```

这里不自动启动底层：底层INIT自身会产生电机命令，必须单独安排。WAIT快照保留并标明原生历史填充值；PLANNER快照复制实际994维输入，二者证据含义不同。

在采集终端指定真实日志路径后：

```bash
.venv_teleop/bin/python -m gear_sonic.scripts.dit_sonic_steps snapshot \
  --bottom-log /absolute/path/new-bottom.jsonl \
  --output "$STEP_RUN/sonic_snapshot.json"
```

成功标志 `SONIC_SNAPSHOT_SAVED`；快照必须不超过250ms，不填补缺失历史。可在模型推理完成后单独采当前本体状态；因此模型采样与解码采样时间会不同，结果明确保留两个来源，不能称为同一时刻。

## 4. 保存的第一帧 + 保存的本体信息 → 离线31维结果

```bash
.venv_teleop/bin/python -m gear_sonic.scripts.dit_sonic_steps decode \
  --capture "$STEP_RUN/capture" \
  --prediction "$STEP_RUN/prediction" \
  --sonic-snapshot "$STEP_RUN/sonic_snapshot.json" \
  --output "$STEP_RUN/decoded"
```

成功标志 `DECODE_SAVED`。使用无DDS的 replay_sonic_decoder；仅替换994维输入中的前64维，剩余930维保持快照原值。校验捕获文件哈希、预测来源、第一帧一致性、模型映射/缩放/默认角一致性及输出有限性。

输出 action31.npy、body_target29.npy、result.json、原始decoder输出及资源哈希。result包含与该快照实际位置的差值，`executed=false`、`motor_written=false`。未自动确认位置或过渡约束，不将“能解码”当作允许实机执行。

## 实机执行边界

当前 `sonic_single_step.py` 是独立的**身体**单帧入口：发送64维token，在底层用新的当前输入解码，之后恢复PLANNER；它不直接执行上面的静态31维数组，也不发送Dex1目标。该入口仍需经核对的位置/变化限值及闭环验证。不能把31维结果整体发到5556或直接映射成LowCmd。

因此本流程交付到“可逐步保存和检查31维结果”；身体与Dex1联合执行尚未通过验证。本脚本不包含自动实机发送步骤。
