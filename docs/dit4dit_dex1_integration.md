# 229：DiT4DiT + SONIC + Dex1

代码提供真实观测推理、模型/遥操仲裁和可选动作下发。**目前通过软件及模拟接收测试，未通过实机运动验收。** 所有进程均在 229 上运行，共享 Linux monotonic 时钟。不得把模型服务移到其他机器而不改时间同步。

## 已实现的接口

- 推理服务在 `dit4dit` 环境运行，只绑定 `127.0.0.1:5560`，没有机器人命令接口。协议为 ZMQ REQ/REP JSON：输入包含 request_id、generation、observed_at（本机单调时钟）、RGB PNG/base64、31维实测状态、任务文字。输出包含相同标识、反归一化的50×66动作、原始归一化动作、越界比例、推理耗时；失败返回 ok=false。
- 关节排列沿用数据采集的 robot_model 映射后，再取 whole_q[0:22] + whole_q[29:36]，追加左右 Dex1 实测位置。不能直接把 DDS body_q 的顺序当作训练顺序。
- 当前图像按训练使用的 RGB/BILINEAR 224×224 处理；只给当前图像，不给未来帧。任务文字沿用采集任务。
- 前64维使用 SONIC protocol v4 token_state；后2维是 motor_output_radians 绝对夹爪目标。通过标定逆映射传入原 RampController，保留反馈失效、限位、端点减速、堵转保护和故障保持；不使用旧7+7手部字段。
- 相机/身体/Dex1观测必须在0.25秒内，组装时间差不超过0.15秒。动作从图像时间计算索引，50Hz、50帧，满1秒即过期。身体状态新鲜度当前以本地收到 g1_debug 为依据，不能证明 DDS 生产端未冻结。
- 请求最多一个在途；接管/停止递增 generation，旧回复不能恢复控制。模型错误、动作过期、观测失效、Dex1离开READY或底层心跳失效会停止模型。
- 唯一机器人发布者为 gateway：5556发送身体消息，5563发送Dex1目标。PICO改用5562；状态5557、相机5555、Dex1实测遥测5559不变。

## 首版接管范围与停止

PICO 接管支持原 PLANNER 系列模式，**不支持 POSE/POSE_PAUSE 与模型 token 模式直接混用**。不要按 A+X 切入 POSE；遇到该切换会停止。首版不自动发送预设初始姿态 token。软件测试未验证 C++ planner/token 切换时的实际姿态连续性。

部署终端 `O` 保持不变。gateway 收到 STOP 后锁定停止、清空缓存、持续发送停止请求；不允许模型回复或PICO启动消息重新启用。此保证依赖正常运行的进程及状态通道，不代表硬件急停。

模型模式下 PICO 的 stop 请求，以及四键产生的 start 请求，均按停止处理，不能启动模型。原 PICO 程序进入 OFF 后会退出，需要手动重启。重新启动底层并收到正常心跳后，才可在 gateway 输入 `reset` 回到只读；之后仍需手动选模式。

## A. 先做真实观测只读检查

在不同终端运行；已有同一服务时不要重复启动。不要同时运行原采集 launcher 和新执行入口，它们会争用5556。

### 终端1：模型服务

```bash
cd /home/cqu023/GR00T-WholeBodyControl
export PYTHONPATH=/home/cqu023/DiT4DiT:/home/cqu023/GR00T-WholeBodyControl
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 \
/home/cqu023/miniconda3/envs/dit4dit/bin/python -u \
  -m gear_sonic.scripts.dit4dit_policy_server \
  --run /home/cqu023/DiT4DiT/results/sonic_dex1_52eps_joint_bs4_20261027_161659 \
  --cosmos /home/cqu023/models/Cosmos-Predict2.5-2B
```

等待 `READY: local inference only`。已在 dit4dit 环境补装 pyzmq==26.4.0；无需重装 SONIC 或 Torch。

### 终端2：保留采集时的相机桥

已有该进程且图像正常则复用。需要重启时：

```bash
cd /home/cqu023/GR00T-WholeBodyControl
.venv_data_collection/bin/python -m gear_sonic.scripts.run_pico_usb_stereo \
  --camera-host 192.168.123.164 --camera-port 15556 \
  --listen-host 192.168.11.224 --headset 192.168.11.228 \
  --view right --publish-camera-port 5555 --preview
```

### 终端3：原底层服务

需要身体状态及控制状态输出，使用原先已验证的启动方式。不要为了只读测试按启动运动的键。是否在等待控制阶段发布身体状态须以 gateway 的 health 为准；缺失时不能绕过检查。

```bash
cd /home/cqu023/GR00T-WholeBodyControl/gear_sonic_deploy
./deploy.sh \
  --cp policy/sonic_v1_1/model \
  --obs-config policy/sonic_v1_1/observation_config.yaml \
  --input-type zmq_manager \
  --motor-kp-scale 4,10=1.5 \
  --motor-kd-scale 4,10=1.5 \
  --enable-dex1-heartbeat \
  enx6c1ff7b8d821
```

### 终端4：仅接收夹爪反馈

```bash
cd /home/cqu023/GR00T-WholeBodyControl
.venv_teleop/bin/python -m gear_sonic.scripts.run_dex1_trigger_control \
  --interface enx6c1ff7b8d821 \
  --limits gear_sonic/configs/dex1_limits.g1_20261013.json \
  --feedback-only --telemetry-port 5559
```

该模式不创建 DDS 命令发布者、不自动张开夹爪。`measured` 必须有左右位置；空字典代表没有反馈。

### 终端5：60秒只读推理

```bash
cd /home/cqu023/GR00T-WholeBodyControl
.venv_data_collection/bin/python -u -m gear_sonic.scripts.run_dit4dit_gateway \
  --task-prompt "Pick up the cup on the left with your left gripper. Then pick up the red block on the right with your right gripper and place it into the cup held in your left gripper. Finally, place the cup, with the red block inside, back on the table using your left gripper." \
  --limits gear_sonic/configs/dex1_limits.g1_20261013.json \
  --log-dir "outputs/dit4dit_readonly_$(date +%Y%m%d_%H%M%S)" \
  --duration 60
```

无 `--execute` 时不创建5556/5563发布者，输入 model/teleop 也会拒绝。日志须出现 sensors_ok=true 和 prediction 的 ok=true。feedback-only 下 dex_ready=false 属正常现象，不用于允许运动。

每次推理保存输入NPZ以及 events.jsonl 中的预测、越界比例、时延和状态。日志较大，长时间运行请关注磁盘空间；退出后手动保留/清理，不自动删除。

## B. 现场配合的短时运动验收

先完成A阶段。现场有人看护、防跌保护到位，在可操作的底层终端验证 O。此阶段尚未自动执行。

1. 停止只读 gateway 和 feedback-only 进程；保留相机与模型服务。
2. PICO改到独立上游端口：

```bash
cd /home/cqu023/GR00T-WholeBodyControl
.venv_teleop/bin/python gear_sonic/scripts/pico_manager_thread_server.py \
  --manager --port 5562 --dex1-triggers
```

3. 夹爪改读统一入口目标（此命令显式启用DDS输出，仍受原保护状态机约束）：

```bash
cd /home/cqu023/GR00T-WholeBodyControl
.venv_teleop/bin/python -m gear_sonic.scripts.run_dex1_trigger_control \
  --interface enx6c1ff7b8d821 \
  --limits gear_sonic/configs/dex1_limits.g1_20261013.json \
  --target-port 5563 --telemetry-port 5559 \
  --travel-time 2 --obstruction-hold --obstruction-timeout 0.5 --execute
```

4. 将A阶段终端5命令的 `--duration 60` 改成 `--execute`，改用新的日志目录。gateway 初始仍为 readonly。
5. 按原PICO操作进入 PLANNER，使上游有新鲜 planner 和有效扳机数据。在 gateway 输入 `teleop` + Enter 才将控制权交给PICO并请求底层启动。首次夹爪仍执行原有张开/松开扳机解锁流程，确认READY。
6. 先验证底层 `O`：gateway 必须进入 stop，后续输入 model 必须被拒绝。重启底层/PICO，待心跳恢复后手动 reset，重新进入teleop。故障保持的夹爪需按原规则在输入新鲜且双扳机松开时，手动 R 恢复。
7. 有 fresh prediction、sensors_ok=true、dex_ready=true 且底层CONTROL后，在 gateway 输入 `model` + Enter。模型实际输出开始下发。
8. 输入 `teleop` + Enter可请求PICO接管：先撤销模型、丢弃在途回复；无新鲜PICO数据则停止。接管不是姿态平滑保证，首次需短时验证。
9. `stop` + Enter请求停止，`quit`退出时请求停止。Dex1控制程序保留运行以执行其故障保持；终止该进程不保证继续保持。

## 回退

先在底层终端 O 停止并确认，再关闭 gateway 和新的夹爪目标控制进程。PICO恢复 `--port 5556`，夹爪恢复不带 `--target-port` 的原命令；原 `launch_data_collection.py` 未修改。不要同时启动两套控制入口。

## 自动化验证

```bash
cd /home/cqu023/GR00T-WholeBodyControl
.venv_data_collection/bin/python -m unittest gear_sonic.utils.inference.test_dit4dit_control -v
.venv_data_collection/bin/python -m gear_sonic.utils.inference.test_dit4dit_transport
```

后者只使用随机16000以上的localhost端口、模拟相机/身体/Dex1/PICO/模型，不创建DDS连接。验证停止、相机断流、推理超时与PICO接管后不再出现模型token。模拟通过不等于实机 O 和控制模式切换已验收。
