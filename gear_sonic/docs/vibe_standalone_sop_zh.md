# 不依赖训练框架的 SONIC＋ViBe 原生 MuJoCo 仿真

只用于仿真。原 SONIC 真机入口和权重未改。PLANNER/VR3PT 使用 low-latency，POSE 使用三模型视觉策略。
运行环境为 GR00T 下的 `.venv_vibe_deploy`，不安装 mjlab、ORCS、mocke 或 ViBe 训练包。
人体预处理的纯数学函数从既有实现提取，来源、版本和许可证见 `gear_sonic/utils/standalone/SOURCES.md`。

## 环境重建
推荐通过 uv 创建独立 Python 3.11 环境，然后安装 requirements_vibe_deploy.txt。
本机网络下载大型包缓慢，实际采用同版本已安装依赖的校验复制。复制为独立文件，不添加原环境的 site-packages 路径；不复制 .pth 或训练包。
`seed_vibe_deploy_environment.py` 递归核对依赖版本、复制并校验每个文件。源环境仅作为安装时输入，部署启动不再需要它。
精确版本和复制记录位于 `/home/cqu023/sonic-diagnostics/vibe-standalone-v1/environment.json`。

在线安装：
```bash
cd /home/cqu023/GR00T-WholeBodyControl
/home/cqu023/.local/bin/uv venv --python 3.11 .venv_vibe_deploy
/home/cqu023/.local/bin/uv pip install --python .venv_vibe_deploy/bin/python -r gear_sonic/scripts/requirements_vibe_deploy.lock
```
已有环境不要重复创建。Linux 桌面窗口需要有效 DISPLAY/XAUTHORITY；请在服务器桌面终端运行。

## 一次性准备
当前已完成，不要用相同输出目录重复准备：
```bash
cd /home/cqu023/GR00T-WholeBodyControl
unset PYTHONPATH
.venv_vibe_deploy/bin/python gear_sonic/scripts/prepare_vibe_standalone.py \
  --source-config gear_sonic_deploy/policy/vibe/observation_config_vibe_sonic.yaml \
  --out /home/cqu023/sonic-diagnostics/vibe-standalone-v1/artifacts
```
生成独立 observation_config.yaml 和派生 Theia FP32 像素入口。
原始三模型和数据保持不变；派生模型在准备时对随机/全灰图像做精确特征对照。
运行时只读取、校验准备好的产物，不调用导出器。配置不再引用训练辅助源码。

## 终端 1：原 SONIC 入口启动独立 MuJoCo
先退出之前的仿真、混合控制器和 manager，避免重复端口；不要停止训练。
```bash
cd /home/cqu023/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
unset PYTHONPATH
export VIBE_DEPLOY_PYTHON=/home/cqu023/GR00T-WholeBodyControl/.venv_vibe_deploy/bin/python
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
unset MUJOCO_GL

python /home/cqu023/sonic-diagnostics/sonic-realtime-v1/run_sim_realtime.py \
  --vibe-config /home/cqu023/sonic-diagnostics/vibe-standalone-v1/artifacts/observation_config.yaml \
  --viewer --steps 1000000 \
  --out "/home/cqu023/sonic-diagnostics/standalone-sim-$(date +%Y%m%d_%H%M%S)"
```
该入口转入新环境的原生 MuJoCo 仿真。DDS 桥仍使用原 SONIC teleop 环境。
7/8 调整吊带、9 切换吊带、空格暂停、R 重置；重置后按现有流程重新启动控制器。
吊带支撑下稳定不能视为独立平衡通过。

## 终端 2：混合控制器
```bash
cd /home/cqu023/GR00T-WholeBodyControl/gear_sonic_deploy
source scripts/setup_env.sh
unset PYTHONPATH
export VIBE_DEPLOY_PYTHON=/home/cqu023/GR00T-WholeBodyControl/.venv_vibe_deploy/bin/python

./deploy.sh --policy-type vibe-hybrid \
  --cp policy/low_latency/model \
  --obs-config policy/low_latency/observation_config.yaml \
  --vibe-obs-config /home/cqu023/sonic-diagnostics/vibe-standalone-v1/artifacts/observation_config.yaml \
  --bundle /home/cqu023/orcs-exports/stairs_split_v1/focus/split_sonic/bundle.json \
  --vision-encoder /home/cqu023/orcs-exports/stairs_split_v1/vision/theia-tiny.onnx \
  --input-type zmq_manager --visual-input pico --rgb-viewer \
  --out "/home/cqu023/sonic-diagnostics/standalone-control-$(date +%Y%m%d_%H%M%S)" sim
```
等待 Init Done。独立 C++ 混合二进制编译至日志目录，不覆盖原 SONIC 二进制。
VIBE_DEPLOY_PYTHON 是混合入口的新变量；不要再用 VIBE_PYTHON 指向 ORCS 环境。
实际图像窗口需要桌面会话。若只需无窗口运行，省略 --rgb-viewer。

## 终端 3：原 PICO manager
```bash
cd /home/cqu023/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
unset PYTHONPATH
python gear_sonic/scripts/pico_manager_thread_server.py --manager --vibe-hybrid --port 5556
```
先不启用 SMPL/VR3PT 调试可视化，避免同步渲染拖慢采样。
A+B+X+Y 校准/启动进入 PLANNER；A+X 请求 POSE。
预热达到连续 11 帧、新鲜度检查通过后进入 POSE。
动作跳变按用户选择仅记录，不阻止切换；断流、NaN、模型/状态不匹配仍停止。
实际 PICO 与无吊带动作质量必须单独验收，不以离线测试代替。

## 验证
```bash
cd /home/cqu023/GR00T-WholeBodyControl
unset PYTHONPATH
.venv_vibe_deploy/bin/python -m unittest gear_sonic.tests.test_vibe_standalone gear_sonic.tests.test_vibe_pose_gate gear_sonic.tests.test_vibe_config
```
报告根目录：/home/cqu023/sonic-diagnostics/vibe-standalone-v1。
preprocessing_parity.json 为原环境/独立实现对照；
handshake/service/validation.json 为正常/全灰 TensorRT 对照；
imports-*.json 记录带导入拦截器的进程模块，forbidden 应为空。
闭环短段结果仅用于加载与执行验证，不等于完整上楼或真实遥操通过。

离线重建：创建空部署环境后，在已有依赖环境中执行 seed_vibe_deploy_environment.py --target <新环境路径> --report <独立报告路径>。必要时通过安装时 PYTHONPATH 提供已下载的 ONNX Runtime 等依赖；该路径不写入新环境，不用于运行。

## 查看器状态与按键
窗口标题中的 mjlab scene 是已导出 MJB 的模型名称，不表示运行训练环境。窗口覆盖层 Native MuJoCo (standalone) 显示运行状态。WAITING FOR TERMINAL 2 时物理保持不动，7/8/9 会更改吊带设置但不会让机器人移动。点击三维画面后按键，终端应输出 SIM_KEY；数字键盘也支持。故障暂停仍处理按键，空格不能越过故障，R 重置后重新启动终端2，再空格继续。
