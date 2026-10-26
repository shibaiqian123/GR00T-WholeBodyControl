# SONIC 原入口 + 训练 Dex1 / ViBe 仿真

本入口只允许本机 MuJoCo。PLANNER/VR3PT 仍是原 low-latency；POSE 才是三模型 ViBe。模型、数据、原实时包装器和真机路径没有改写。本轮没有提交或推送。

## 本次验收状态（2026-10-21）

- 原 run_sim_realtime.py 保留；不传 --vibe-config 时，原入口的语法树与 HEAD 相同，原 --help 正常。没有重跑完整 43DOF 遥操，不能称为动态回归通过。
- 训练 G1 + 固定 Dex1、原训练执行器、楼梯和相机由已有校验过的 case 加载。相机 112×63、垂直 FOV 42.5°；无新缩放、映射或接触补偿。
- 平地站立检查：root=(-0.271346,1.157173,0.759022)，wxyz≈(0.698871,0,0,-0.715247)，保留原样本水平朝向 -1.593956rad，只去掉倾斜，地面 z=0，足底预留 2mm，初始无负接触距离。原 SONIC default_angles 与训练默认角逐项一致。
- 21 项测试通过：人体输入、配置拒绝、站立初始化不改资产、吊带撤力、seed 诊断恢复；脚本语法和 diff 检查通过。
- 吊带 OFF 策略初始化：700 个控制步/14 秒仿真，Init Done；实际步进约 0.999× 实时。
- 吊带支撑下，原 manager 的静止 PLANNER 启动消息进入 C++，486 次 SONIC 策略 RPC，14 秒测试结束，实际约 1× 实时。没有解除吊带，不是独立平衡验收。
- 该 PLANNER 测试中最小接触距离约 -0.01272m，末帧最大目标/实际关节差约 1.414rad，root_up 最小约 0.9808。需继续检查实际站立表现，不能忽略这些数值宣布通过。
- 三模型数值对照报告：外部 final-parity/visual/validation.json。此测试比较同输入 ORT 与 TensorRT，不代表 RGB 与灰图行为相同。
- 未通过/未执行：人工解除吊带后的站立、真实 PICO 校准和 POSE、返回 PLANNER、VR3PT、无吊带楼梯以及实时灰图闭环。按验收顺序停在这里，没有自动执行后续遥操。

## Terminal 1：沿用原仿真入口

在服务器桌面终端执行，保留桌面的 DISPLAY/XAUTHORITY。不要从无 DISPLAY 的 SSH 终端直接打开窗口。

```bash
cd /home/cqu023/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
unset PYTHONPATH
export MUJOCO_GL=glfw
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
: "${DISPLAY:?请在服务器桌面终端执行}"

python /home/cqu023/sonic-diagnostics/sonic-realtime-v1/run_sim_realtime.py \
  --vibe-config gear_sonic_deploy/policy/vibe/observation_config_vibe_sonic.yaml \
  --viewer \
  --out "/home/cqu023/sonic-diagnostics/vibe-live-sim-$(date +%Y%m%d_%H%M%S)"
```

只想检查场景时用 --check-only 替换 --viewer，使用新的输出目录；无需 Terminal 2/3。
seed 专用诊断额外传 --initialization seed，默认关闭吊带，不作为日常初始化。

原脚本仍先使用原 teleop 环境；仅显式 ViBe 模式切到现有 ORCS Python 运行训练编译场景。DDS 使用原 .venv_teleop/SDK 子进程，避免 CycloneDDS 版本混用。仿真端不执行策略推理。

## Terminal 2：混合控制器与视觉模型

在服务器另一个桌面终端：

```bash
cd /home/cqu023/GR00T-WholeBodyControl/gear_sonic_deploy
source scripts/setup_env.sh
export VIBE_PYTHON=/home/cqu023/ViBe-SONIC/.venv/bin/python

./deploy.sh \
  --policy-type vibe-hybrid \
  --cp policy/low_latency/model \
  --obs-config policy/low_latency/observation_config.yaml \
  --vibe-obs-config policy/vibe/observation_config_vibe_sonic.yaml \
  --bundle /home/cqu023/orcs-exports/stairs_split_v1/focus/split_sonic/bundle.json \
  --vision-encoder /home/cqu023/orcs-exports/stairs_split_v1/vision/theia-tiny.onnx \
  --input-type zmq_manager \
  --visual-input pico \
  --rgb-mode normal \
  --rgb-viewer \
  --out "/home/cqu023/sonic-diagnostics/vibe-live-control-$(date +%Y%m%d_%H%M%S)" \
  sim
```

每次在该输出目录编译独立控制二进制，不替换原 target/release 二进制。模型加载/编译可能较久；Terminal 1 会等待，不让机器人在等待期间自由落体。
先显示 HYBRID_VISUAL_READY，通过模型对照后才启动 C++；等待 Init Done。
仅做模型检查可去掉 --rgb-viewer，增加 --check-only，无需其他终端。

Actual Theia RGB 窗口在实际 POSE 推理后才更新；PLANNER 中等待是正常现象，不表示已经使用视觉。正常或 --rgb-mode gray 必须通过重启 Terminal 2 从同一初始状态对照。
visual/ 中保存首次及每 50 次视觉前向实际使用的图像和 epoch/tick/模式元数据。parity_normal.png 与 parity_gray.png 是数值对照输入，不是实时遥操画面。Terminal 1 的 camera_initial.png 只是相机原始画面。

## Terminal 3：原 PICO manager（后续人工验收）

先完成无吊带站立门槛，再进行下面的实际遥操验收：

```bash
cd /home/cqu023/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
unset PYTHONPATH

python gear_sonic/scripts/pico_manager_thread_server.py \
  --manager \
  --vibe-hybrid \
  --port 5556 \
  --vis_vr3pt \
  --vis_smpl
```

保留 XRoboToolkit full-body、tracker 校准和原按键；IP 填当前电脑的实际局域网地址。
A+B+X+Y 启动 PLANNER；A+X 请求切换 POSE；PLANNER 中左摇杆请求 VR3PT。当前仅通信/数值门槛通过，不能将这些命令视作真实 PICO 验收完成。
--vibe-hybrid 额外发送真实人体时间与根平移，50Hz；不替换原 pose/command/planner 消息。POSE 需要 11 个真实采样点，不补造未来帧。
本 SOP 不启动任何真机网口或夹爪服务。

## 窗口操作和下一道门槛

- 7 降低吊带锚点，8 升高，9 开关吊带，沿用原 ElasticBand 算法/增益。默认启用。
- 空格暂停/继续物理。暂停时不推进仿真/策略控制步，真实 PICO 时间仍会继续。
- R 恢复本次初始化、开启站立吊带、更新 epoch 并暂停。停止并重启 Terminal 2，回到 Terminal 1 按空格，重新等待 Init Done，再显式接管。不会自动复用旧历史或恢复接管。
- 断流、时钟不匹配、非有限值和严重倾倒会停止推进。故障后关闭并重启 Terminal 1/2；不能靠继续按启动键清除故障。
- 默认最大 10000 控制步，可用 --steps 指定；这不是训练迭代。

下一步人工步骤：吊带下确认姿态 → 原 manager 启动静止 PLANNER → 逐步降低吊带 → 人工解除吊带，短时观察平衡。若持续振荡/倾倒则停止，不进入 ViBe POSE。不得通过降低 PD 或修改模型参数掩盖差异。
录制参考仅供对照：Terminal 2 的 --visual-input recorded 从 POSE 接管时起计参考帧；仍需原 manager 的启动/模式操作。录制路径不代表实时 PICO 已验证。

## 复用来源与证据

- 吊带直接加载 GR00T 的 gear_sonic/utils/mujoco_sim/unitree_sdk2py_bridge.py 中 ElasticBand 类，避免导入另一版本 DDS；记录该源文件 SHA256。角误差先变换到初始水平朝向参考系，再把力矩旋回世界系；保持原增益和弹簧公式，防止吊带把机器人扭回世界零朝向。
- 复用原 SONIC manager 消息构造、C++ 控制与 TensorRT 包装；训练模型、NativeCase、归一化、相机来自现有 ViBe。
- 独立仿真使用累计截止时间、5ms×4 子步，每个 20ms 控制边界严格等待匹配命令；不跳步追赶。原基类循环和外部 realtime 包装器未编辑。
- policy/vibe/observation_config_vibe_sonic.yaml 保持原校验格式，仍含 recorded_sim 标记；它只是此处的资产契约，实际输入模式以启动日志/--visual-input 为准，未绕过配置校验。
- 日志根目录：/home/cqu023/sonic-diagnostics/vibe-sonic-entry-v1/
  - planner-band-heading/sim/initialization.json（最终水平朝向）
  - original-regression.json
  - standing-dds/{sim,controller,visual}.log
  - planner-band-heading/sim/{report.json,steps.jsonl,camera_initial.png}
  - planner-band-heading/visual/control.jsonl
  - final-parity/visual/{validation.json,parity_normal.png,parity_gray.png}
- 本轮诊断进程结束后关闭，仅留下上述报告和产物；既有训练和原 SONIC 真机程序未启动/停止。


## A+X 预热握手
仅 --vibe-hybrid 模式：A+X 请求 POSE 后继续当前 PLANNER，开始发送真实人体数据。
接收端确认连续 11 帧、新鲜度、RGB/状态配对及视觉动作切换差值后才切到 POSE。
再次 A+X 取消请求；10 秒未通过则保留 PLANNER，显示原因。不要反复切换绕过保护。
接收端在本机 5583 发布就绪状态；不需要新增终端。preparation.jsonl 记录逐次检查。
修改后需要重新启动控制器和 PICO manager，旧进程不会自动加载修改。
这不代表实际 PICO 遥操或动作质量已经通过；进入 POSE 后的真实断流仍停止控制。
