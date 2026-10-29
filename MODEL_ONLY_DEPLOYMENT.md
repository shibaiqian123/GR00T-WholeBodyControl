# DiT4DiT + SONIC + Dex1 纯模型入口

本入口不启动 PICO，不订阅 VR，不提供遥操接管。原采集入口保持原行为，不可与本入口同时占用动作端口。

## 启动（229）

```bash
cd /home/cqu023/GR00T-WholeBodyControl
python3 gear_sonic/scripts/launch_dit4dit_field.py
```

默认使用 sonic_v1_1 底层、19000步 checkpoint、本地 Cosmos、enx6c1ff7b8d821 网口及原任务文字。可先加 `--dry-run` 查看全部命令。已有同名会话时使用 `tmux attach -t dit4dit_field`，不要重复启动。端口冲突时先确认原服务用途，不要强行杀掉未知进程。

五个窗口：0 bottom、1 observations、2 dex1、3 gateway、4 model。鼠标点击选择输入窗口，Ctrl+B 后按 z 放大/还原。不要用 Ctrl+S 暂停输出。底层确认提示由现场人员处理；入口不会自动启用运动。

## 手动启用

1. 确认底层处于 WAIT_FOR_CONTROL，模型加载完成，相机、身体与左右夹爪反馈持续有效。
2. 在 gateway 输入 `arm` 回车：启用夹爪当前实测位置保持，不执行原遥操的自动张开流程。
3. 等待反馈就绪，在 gateway 输入 `model` 回车。入口先发送真实预测 token，等待底层 token_ready 确认，随后才发启动请求。
4. 本步骤会驱动机器人。首次测试需现场看护、防跌保护，先验证底层窗口 O 停止。尚未完成本版本真实运动验收。

## 停止与恢复

底层窗口按 O 或 gateway 输入 `stop` 回车，入口锁定 stop，清空模型动作；model/arm 请求不能解除锁定。O 是正常运行下的停止操作，不是独立硬件急停。

收到底层 STOP 确认后，入口停止重复向底层发送 STOP，夹爪继续走停止保护。若底层已经退出而 STOP 确认丢失，可在确认机器人停止后输入 `recover`：仅停止 STOP 重试，仍保持锁定，不恢复运动。

重新启动底层并获得新鲜 WAIT_FOR_CONTROL 心跳后，手动输入 `reset`，然后重新 `arm`、`model`。不会自动恢复。异常夹爪按原设备恢复流程处理，不通过重复命令绕过保护。

## 观测与保护

独立 DDS LowState 订阅器只读身体状态，按原 robot_model 映射组成29维身体状态，再拼接左右实测夹爪为31维。身体观测使用5564；底层5557心跳仍是控制授权与停止依据。

相机5555存在时复用；否则启动原相机源的 USBSource 和 RecordingPublisher，保留采集图像处理，不连接头显。动作输出5556、夹爪目标5563由 gateway 唯一发布。Dex1遥测5559，模型服务5560。

预测应为50×66，拆分64维token和两维夹爪绝对目标。按观测时间选50Hz动作索引；整段过期、反馈过期、模型错误或夹爪预测越界均停止，不无限重复末帧。停止后的旧代预测不能恢复运动。

## 日志与回退

日志位于 outputs/model_only_时间戳/，包括五个窗口日志、gateway事件JSONL及输入图像/状态NPZ。原始预测记录在gateway事件中；非有限预测单独记prediction_error并停止，不写非法JSON。

先正常停止机器人，再关闭本入口相关进程，确认端口释放后可使用原 launch_data_collection.py 恢复遥操采集。不能让两个入口同时发布命令。

软件验证与现场验收分开：隔离模拟接收覆盖停止锁定、反馈中断、非有限预测、模型超时及延迟token确认；真实运动表现仍须现场验证。

## 远程推理版本（当前默认）

229不再加载DiT4DiT，第4窗口改为SSH tunnel；32只在127.0.0.1:5560监听。先在32启动服务并看到READY，再在229启动现场入口。所有运动启用仍需现场人工arm/model。

32部署位置 /share/2025slx/DIT4DIT/remote_policy/，匹配的19000步权重、配置及统计放run19000/，不会修改原训练目录。

32启动示例（选择空闲GPU）：

```bash
export POLICY_GPU_UUID=GPU-ea98a495-2d03-62eb-779f-4a0cef804025
bash /share/2025slx/DIT4DIT/remote_policy/start_policy_32.sh
```

229启动：

```bash
cd /home/cqu023/GR00T-WholeBodyControl
python3 gear_sonic/scripts/launch_dit4dit_field.py
```

默认SSH目标2025slx@10.246.1.32，可用--policy-ssh修改。若旧本地模型仍运行，应正常退出旧会话后再启动，修改文件不会卸载已运行模型。隧道断开时不会自动恢复模型控制。

时间戳约定：observed_at属于229的单调时钟，32只原样回传、不比较自己的时钟；客户端按请求编号和generation匹配响应，并使用本地保留的观测时间覆盖响应时间戳。观测总年龄达到0.95秒拒绝结果；50帧动作窗口过期仍停止。日志增加roundtrip_seconds、observation_age_seconds；不能用服务器单次推理时间替代端到端延迟。

### 2026-10-28 远程验证记录

32服务已显示 All keys matched successfully 和 READY，运行于tmux会话 dit4dit_policy。229的outputs/remote_policy_replay.json保存10个已记录观测的真实SSH网络推理结果：全部50×66且有限；首次预热3.359秒，客户端拒绝过期结果，随后9次往返0.284–0.316秒。该时间从已编码请求发送开始计算，不包含现场相机采集等待，不是完整实机延迟验收。

9项控制/跨时钟单元测试通过；5类模拟接收测试（O锁定、身体反馈中断、夹爪反馈中断、非有限预测、模型超时）通过。尚未自动启动底层或发送实机动作。每次重启服务后，应先等readonly状态获得稳定新鲜预测，再人工arm/model。

### 流式token启动传递修复

ZMQManager在STREAMED_MOTION模式处理完pose输入后，将网络start传入operator_state.start；要求token已就绪且没有stop。网络stop、键盘O、已有stop及pose产生的stop均优先。未就绪时拒绝请求，不保存为未来自动启动。启动超时仍由原gateway控制，未延长。

底层日志链应依次出现：
- Network start received: stop=0, planner=0
- Streamed token start applied: operator_state.start=true
- 原有 transitioning to CONTROL state

若停止优先或token缺失，会记录Start blocked或Start rejected。接收日志只能证明解析到请求，不能替代CONTROL心跳确认。

测试覆盖启动/停止/token/既有启动状态的16种组合、token晚到不得自动启动、同帧停止优先。全程序编译用于检查集成，实机启动仍待现场验证。本次没有自动重启底层或发送model命令。此前bottom_stale是独立问题，不因这次启动补丁而认定解决。

## 2026-10-28: asynchronous input preparation

The model-only gateway copies observations into its bounded one-request queue.
PNG encoding and input saving run in the inference worker, not the control loop.
The worker applies the server's RGB/BILINEAR 224x224 preprocessing before lossless
PNG transport; full-resolution inputs and original timestamps remain in NPZ.
NPZ files now use uncompressed storage: lower CPU cost, greater disk usage.
Preparation errors return inference errors; original freshness thresholds,
generation checks, stop priority, and manual activation remain unchanged.

Validation: 16 unit tests and five isolated transport scenarios passed.
A 60-second parallel read-only run at outputs/async_readonly_20261028_203256
returned 142 valid predictions with no motion frames. Preparation median 37.9ms;
health-sampled loop gap median 20.07ms, P95 20.09ms (not a full 50Hz timing audit).
Observation-to-response median 425ms, P95 524ms; 122/142 arrivals met 500ms.
The existing gateway was still sharing the inference server. This verifies the
control-loop improvement, not stable startup freshness or hardware acceptance.
An exclusive read-only measurement is still needed after the operator safely
stops the existing control session. Do not loosen freshness thresholds or
activate motion automatically. Running processes must restart to load changes.
