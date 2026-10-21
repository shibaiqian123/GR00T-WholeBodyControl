# GR00T 内运行已训练的视觉策略（录制 SMPL 基准）

当前入口运行 motion encoder、Theia 和 visual decoder，使用训练同款固定 Dex1 G1、相机及楼梯。推理默认复用 SONIC C++ TensorRT FP32 适配库；物理循环复用已验证的 ViBe NativeCase。它是单进程录制回放，**尚未接入 DDS、PICO 或 planner**，不能称为三终端遥操已经完成。原 SONIC 命令不受影响。

## 启动窗口

在服务器桌面终端执行，使用现有 ORCS Python 环境，不重装或覆盖 `.venv_teleop`：

```bash
cd /home/cqu023/GR00T-WholeBodyControl
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
unset MUJOCO_GL
bash gear_sonic/scripts/run_vibe_recorded_sim.sh --viewer --hold-final
```

MuJoCo 窗口启动后暂停，空格开始/暂停；运行循环中 R 重置并暂停，关闭窗口退出。参考结束后 `--hold-final` 保留末帧；该状态不处理重播，需要重新启动。Actual Theia RGB 窗口在首次推理后显示实际输入。这里无吊带，不使用 `]`、`9` 或 PICO 接管。

全灰输入对照：在同一命令末尾增加 `--rgb-mode gray`。默认每次新建输出目录，不覆盖旧报告。报告记录跟踪、穿透、动作和实际输入图像；不把参考结束自动当成动作质量通过。

## 配置和命令的含义

`gear_sonic_deploy/deploy.sh` 仅在显式 `--policy-type vibe` 时转入新入口，`sim` 是唯一允许的目标。`--input-type zmq_manager` 明确拒绝，防止误把此入口当作实时遥操。SONIC 默认分支未改。

观测配置是 JSON（也是 YAML 兼容格式），由 bundle、manifest 与 native case 自动生成。保持 930 维本体历史、840 维 tokenizer、64 维本体查询、6 维参考速度查询和 Theia patch/CLS 特征。根速度查询来自录制 reference，不使用实时机器人速度冒充目标速度。配置与模型、相机、动作参数、场景哈希不一致时拒绝运行。

生成新版本配置（目标文件必须不存在）：

```bash
cd /home/cqu023/GR00T-WholeBodyControl/gear_sonic_deploy
export VIBE_PYTHON=/home/cqu023/ViBe-SONIC/.venv/bin/python
./deploy.sh --policy-type vibe \
  --bundle /home/cqu023/orcs-exports/stairs_split_v1/focus/split_sonic \
  --vision-encoder /home/cqu023/orcs-exports/stairs_split_v1/vision/theia-tiny.onnx \
  --obs-config /home/cqu023/sonic-diagnostics/groot-vibe-v1/observation_config.json \
  --prepare-config \
  --case /home/cqu023/sonic-diagnostics/vibe-native-v1/case_stair_p2_6768415cac75882b \
  --vibe-root /home/cqu023/ViBe-SONIC/vibe sim
```

现有配置已经生成，不要重复覆盖。便捷启动器支持 `VIBE_OBS_CONFIG`、`VIBE_BUNDLE`、`VIBE_VISION`、`VIBE_TRT_LIBRARY`、`VIBE_OUT` 覆盖路径。ONNX Runtime 当前从已有临时安装 `/tmp/orcs-split-validation-20261020/runtime` 加载；清理临时目录后必须将 `VIBE_ORT_ROOT` 指向有效安装，启动器不会自动安装依赖。

## 验证边界

- 启动前强制正常/灰图的 TensorRT 对 ORT 同状态对照；latent 必须完全一致，action 最大误差不超过 1e-5。
- 配置测试涵盖模型/场景被改、相机不匹配、配置篡改、缺失文件摘要、真机和 manager 入口拒绝。
- 当前状态：数值对照通过；闭环结果见外部输出目录的 validation.json、trajectory.jsonl。RGB 为 MuJoCo 生成，转换为 BGR 后送入导出 Theia；归一化只在模型内执行。
- 后续仍需：训练场景 DDS 状态/命令接入、RGB 与 DDS 时钟配对、实时 PICO 的窗口与参考根速度契约、接管状态机与断流闭环。planner、VR3PT、实机未验收。

源码复用：ViBe checkout 的 `src/vibe/sim2sim/runtime.py`（NativeCase/History/Deadline）、`deploy/controller/{trt_session.py,prepare_vision_trt.py}`，后者复用 SONIC TensorRT C++ 引擎。保留外部 checkout，不复制网络实现。配置固定 native helper 哈希；报告记录模型与 TRT 库哈希。

## 本轮服务器验证结果

输出目录：`/home/cqu023/sonic-diagnostics/groot-vibe-v1`。

- `check/validation.json`：正常/灰图 latent 完全一致；TensorRT 对 ORT action 最大误差分别 6.5565e-7、7.1526e-7。
- `normal/validation.json`：关注样本完整执行 498 个控制步（9.96 s 仿真），未触发 NaN/倾倒停止。
- `normal/trajectory.jsonl`：逐帧平均跟踪误差的均值约 0.1207 m，最小接触距离 -0.01719 m。不是接触质量验收通过。
- `gray/validation.json`：第 367 步停止失稳；平均跟踪误差约 0.4266 m，最小接触距离 -0.01878 m。此单次对照支持视觉有影响，但不是统计成功率评估。
- 灰图控制循环实时倍率约 1.00，排除了模型加载和暂停；整次进程计时仍另行保留，不可混用。首轮正常图报告的 0.772 包含初始化，不能当作控制循环倍率。
- 桌面已确认出现 `MuJoCo : mjlab scene` 和 `Actual Theia RGB - native MuJoCo` 窗口。初始暂停，尚未自动按键接管。
- 7 项配置/入口测试通过。默认 SONIC 分支在新增 dispatch 区块之外保持原样；本轮没有重新启动原 SONIC 实机或遥操进程。
