# Dex1-1 食指扳机深度比例控制

此功能独立于身体策略和 Dex3，不修改身体增益、模型或观测配置。默认关闭。
左食指扳机控制左夹爪，右食指扳机控制右夹爪；握把键不参与。
扳机深度连续映射到标定电机位置：0 全开，0.5 半行程，1 全闭。保持半按就保持对应位置。
公式：q_des = command_open_rad + trigger * (closed_rad - command_open_rad)。
command_open_rad 从标定 open_rad 向闭合方向内缩 open_margin_rad；标定反馈边界不变。
中间行程直接发送比例位置目标；向两端最后 5% 的行程运动时执行端点制动。启动自动张开单独保留约 2 秒的渐进过程。

## 前提与行程

使用 Unitree 官方 dex1_1_service 的 `unitree_go::MotorCmds_` / `MotorStates_`，话题为
`rt/dex1/left/cmd`、`rt/dex1/right/cmd` 及对应 `state`。每条命令只含一个电机。
参考协议：https://github.com/unitreerobotics/dex1_1_service
电机输出侧 q 使用弧度；不是夹爪开口毫米数。

必须先确认机器人侧夹爪服务已运行且标定有效。程序不会安装、启动或标定机器人侧服务。
官方正弦演示范围不等于你的夹爪行程端点，不可直接套用。

在项目根目录复制 `gear_sonic/configs/dex1_limits.example.json` 到自己的运行目录。
填写左右 `closed_rad`、`open_rad`，确认标定后设置 `calibrated: true`。
方向允许相反，程序按开闭端点计算。null、相同端点、非有限值、未标定配置均被拒绝。
本次开发没有收到实际夹爪状态，也没有取得确认过的端点，因此示例有意不能直接用于硬件。

## 不连接硬件的演示

```bash
cd /home/cqu023/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python -m gear_sonic.scripts.run_dex1_trigger_control --demo
```

此演示不初始化 DDS/ZMQ，使用归一化虚拟行程。显示启动张开、左右闭合、松开张开和停止保持。

## 只读检查夹爪服务

```bash
python -m gear_sonic.scripts.run_dex1_trigger_control \
  --interface enx6c1ff7b8d821 --observe
```

只订阅状态，持续十秒，不创建命令发布器。`{}` 表示没有收到状态，不能据此断定夹爪硬件故障；
检查机器人端服务、DDS 域和网络。必须看到左右两侧有效位置，才进入硬件开合测试。

## 三个进程

都在同一台 Linux 电脑运行：时间戳使用共享单调时钟。本实现不支持跨电脑时钟直接对齐。
先退出旧控制器，保留身体支撑保护，避免重复运行。启动身体程序本身就会发送初始化命令。

### 1. 身体控制器（新编译程序）

```bash
cd /home/cqu023/GR00T-WholeBodyControl/gear_sonic_deploy
cmake --build build -j4
./deploy.sh \
  --cp policy/low_latency/model \
  --obs-config policy/low_latency/observation_config.yaml \
  --input-type zmq_manager \
  --enable-dex1-heartbeat \
  enx6c1ff7b8d821
```

`--enable-dex1-heartbeat` 只开启反馈端口 5557 上的 `dex1_control_status ` 状态话题；
不发布 Dex1 电机命令。初始化、等待接管和策略运行都会发布；O/停止时发布 STOP，退出后心跳消失。

### 2. PICO 管理器

```bash
cd /home/cqu023/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager --port 5556 --dex1-triggers
```

`--dex1-triggers` 在默认 OFF/debug 状态也发送食指扳机数据，不需要 A+B+X+Y 启动身体。
话题 `dex1_trigger ` 使用 msgpack，包含采样时间、发布时间、序号、有效标志和左右扳机值。
身体数据停止更新时，扳机数据判为无效。新增通路不使用握把键。

### 3. 夹爪进程（先 dry-run）

```bash
cd /home/cqu023/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python -m gear_sonic.scripts.run_dex1_trigger_control \
  --interface enx6c1ff7b8d821 \
  --limits /absolute/path/to/confirmed-dex1-limits.json \
  --startup-open-time 2
```

默认 DRY-RUN，只接收实际反馈、打印目标，不创建电机命令发布器。
真实夹爪不会跟随 dry-run 目标，所以未实际张到最大时，会停在 OPENING；完整状态机请用 `--demo` 验证。

端点、服务与空夹爪环境已确认后，在同一条命令末尾添加 `--execute`，才开启真实输出。
开启后收到有效输入、控制器心跳和左右状态时会立即开始缓慢张开，无须启动身体策略。初始化完成、松开扳机后，才进入比例控制。

参数：`--domain` 默认 0；`--trigger-port` 默认 5556；`--status-port` 默认 5557；
`--feedback-timeout` 默认 0.25 秒，只允许收紧；`--kp 5 --kd 0.05` 采用官方夹爪测试示例的增益，
不影响身体增益。真实夹持力仍需实测，不把位置保持等同于力控。

## 状态及停止语义

- WAIT_INPUTS：等待所有输入有效，不发布命令。
- OPENING：从实测位置开始，缓慢打开。暂不接受闭合；两侧实测开口达到端点附近才进入下一阶段。
- RELEASE_TO_ARM：要求左右扳机都松开一次。
- READY：扳机 0～1 连续对应全开到全闭的电机行程；保持部分按压则停止在对应目标。日志 requested_target 是比例目标，target 是实际发送目标，中间行程与 requested_target 相同；端点制动时 target 暂缓推进；到位时 direction 为 holding。
- FAULT_HOLD：停止推进目标，以最近有效实测位置为保持目标。恢复数据不会自动恢复动作。

头显数据/扳机、控制器心跳超过 250 ms 未更新，O、PICO 停止组合键、无效数据都会锁存保持。
有一侧夹爪状态无效，该侧不再发布命令；另一侧可保持。状态恢复后仍停在保持阶段。
夹爪进程中输入 `R` 再回车：仅当控制器恢复正常、所有数据新鲜且左右扳机松开时，才恢复。
恢复从当前实测位置继续，不重新执行启动全开。

O 停止身体后，不要关闭夹爪进程：它继续发送保持目标。
夹爪进程被 Ctrl+C/Q 关闭、服务退出、断电、网络中断时无法保证保持物体；软件不能替代机械支撑。
正常松开时比例目标为全开；异常停止仍保持，不按扳机值继续运动。

## 验证状态

自动测试覆盖独立左右开合、两秒行程、反向、阈值滞回、限位、卡顿、输入与反馈失联、O 停止语义、
恢复门控、旧包重放、无效数据、SDK 消息与 dry-run 不创建发布器。
真实验收尚未执行：需确认服务与端点，空夹爪分别完成左右和双侧测试各三次，再与身体遥操联动。

### 本次开发验证结果

- 21 项 Python 自动测试通过；C++ 完整构建、Python 编译检查、Shell 语法检查和 diff 检查通过。
- 仅 loopback、DDS 域 83 的虚拟夹爪端到端测试通过：dry-run 收到 0 条电机命令；
  execute 模式收到 628 条虚拟命令，STOP 后左夹爪保持位置约 0.50023，保持段变化为 0。
- 测试脚本和日志保存在 `/home/cqu023/dex1-trigger-validation/`；其中 VIRTUAL-ONLY-limits.json
  仅供虚拟服务测试，禁止用于真实夹爪。
- 仓库已有的 FK 测试依赖缺失的 `reference/bones_072925_test/`，无法完成；不是新增功能测试通过的组成部分。
- 对真实机器人网口的十秒只读监听未收到 Dex1 状态；未执行真实夹爪开合或行程标定。

### 端点反馈容差修正

命令仍严格限制在标定的闭合/张开端点内。实测反馈单独允许
`min(0.02 rad, 全行程的 1%)` 的端点偏差，避免编码器波动或轻微跟踪超调导致端点锁死。
启动、恢复和故障保持目标也会夹紧到命令行程内；超过反馈容差仍锁存保护，250 ms 超时不变。
新增全开/全闭附近带噪声反向、启动和保持目标限幅、大幅越界保护测试；26 项测试通过。
此修正尚需现场重复全开/全闭循环验收，不能据此断言所有实机保持事件已经解决。

### SDK 输入帧新鲜度修正

一次实机 first_fault 显示：两侧反馈年龄约 1.3 ms、身体心跳约 19 ms，
但扳机发布时引用的全身姿态缓存已超过 259 ms，因此误将该通路关联到姿态处理线程的新鲜度。
扳机发布改用 reader.get_timestamp_ns() 的 SDK 输入帧推进来计时；不再依赖身体姿态缓存。
首次看到缓存时间戳不会直接认定为新数据，必须观察到时间戳向前推进；
重复读取相同帧不会刷新年龄，SDK 真正停止更新超过 250 ms 仍触发锁存保持。
诊断包增加 source_timestamp_ns 和 freshness_source 字段。
31 项测试通过，包括姿态线程不更新时的独立输入帧推进、时间戳冻结/重置和真实失联仍锁存。
这份日志不能证明当时 SDK 输入帧本身是否也停顿；仍须现场验证，而非取消或延长超时保护。


### 比例控制验收

按用户更新要求，已将二值持续开合改为绝对比例位置目标。33 项自动测试通过，包含
保持半按多秒不继续闭合、25%/75%/50%/100%/0% 往返、左右独立、小于原阈值的轻按也按比例响应。
仅需重启夹爪进程使新控制逻辑生效；仍须用空夹爪验证实际位置跟随。


### 去除正常跟随限速

按用户要求，READY 阶段直接采用连续比例目标，不再经过两秒斜坡。
`--startup-open-time`（兼容别名 `--travel-time`）仅影响启动自动张开，不影响正常扳机跟随。
故障保持、手动恢复、端点命令限幅和超时仍保留。目标无软件斜坡不代表机械响应或网络延迟为零。


### 全开端点内缩余量

本次 first_fault 明确记录左侧 q=5.385349，而标定全开为 5.35725，偏差约 0.0281 rad；
扳机 valid=true、stop=false，身体心跳正常。因此本次是端点反馈越界，不是输入断流。
增加可配置 open_margin_rad，仅将正常全开目标向内缩小，反馈越界保护和 250 ms 超时不变。
这台机器两侧设为 0.05 rad：左侧正常全开目标 5.30725，右侧 5.33774；闭合目标仍为 0。
最大开口略减约 1% 电机行程，连续比例映射和无软件跟随限速保持不变。
启动从当前实测位置缓慢过渡；异常保持仍以实测位置为准，不强行退到内缩目标。
37 项自动测试通过；软件余量不能保证所有动态超调都被消除，须实机重复开合验证。


### 仅端点制动（用户已确认）

中间 90% 行程保留直接连续位置映射；朝全开/全闭方向进入最后 5% 时制动：

1. 从中间突然要求端点，先把目标设在端点区入口，避免一步把目标送到尽头。
2. 实测位置与当前目标相差不超过 2% 行程、实测速率不超过 15% 行程/秒后，才继续推进。
3. 端点区内目标速度不超过 15% 行程/秒，并随剩余距离减小，最低 2% 行程/秒，最终到达比例目标。
4. 从端点向中间撤回时直接跟随；左右两侧独立。初始化、故障保持、人工恢复和反馈保护不变。

SDK 状态采集增加实测 dq，日志 feedback_status 显示 dq，endpoint_braking 标记是否在端点制动。
--observe 输出每侧为 [q, 接收单调时间, dq]。首次实机使用空夹爪，从中间位置测试松开到全开、
再按深到全闭，重复三轮。若仍有 FAULT_HOLD，保存 first_fault，不直接放宽越界和通信保护。
44 项自动测试通过；只验证目标生成与通信行为，不等于已经证明实际夹爪没有超调。
