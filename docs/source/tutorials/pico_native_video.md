# G1 原厂摄像头 → PICO XRoboToolkit 原生视频

独立于 SONIC、PICO 动作和 Dex1-1 控制的第四个进程。它不打开 `/dev/video4`，不停止
`videohub_pc4` / `master_service`，不调用身体或夹爪命令。通过 SDK 的 videohub 取图接口
（GetImageSample / API 1001）取得 JPEG，在电脑上解码并用 NVENC 编码 H.264。
SDK 的 VideoClient 位于 go2.video 包中，但该图像 API 已在本台 G1 验证成功；不使用 Go2 运动接口。

## 已确认环境

- G1 开发机：192.168.123.164；D435i 被原厂 videohub_pc4 正常使用。
- 电脑机器人网卡：enx6c1ff7b8d821；电脑与 PICO 相通的 IP：192.168.1.103。
- PICO：192.168.1.199；现有 XRoboToolkit App 的视频配置为 PICO4U / ZEDMINI。
- 原厂接口取得的图像为 1920×1080 JPEG。
- 实际 ZEDMINI 请求：2560×720、60 FPS、4 Mbps、renderMode=2、H.264、接收端口 12345。
- 电脑已验证 FFmpeg h264_nvenc 可用；使用现有 .venv_teleop 的 numpy、opencv、SDK。

## 启动与停止

先测试画面布局：

```bash
cd /home/cqu023/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python -m gear_sonic.scripts.run_pico_video_bridge --source test --layout mono
```

PICO：Remote Vision → ZEDMINI → Listen → 输入 **192.168.1.103**。
这里输入电脑 IP，不是机器人 IP，也不是头显自己的 IP。
画面打开后短按右手柄 B 切到平面视图。应看到一幅完整 TOP / LEFT / RIGHT、彩色区域、移动黄点。
使用现有 App 的平面窗口，两眼自然观看同一个屏幕，不生成立体深度。当前默认 --layout mono。

只有测试图双眼显示正确后，再退出测试程序并启动真实摄像头：

```bash
python -m gear_sonic.scripts.run_pico_video_bridge --source robot --layout mono
```

在视频终端按 Ctrl+C 停止。只停止本进程及它创建的 FFmpeg，不操作其他服务。
一次只启动一个占用 13579 的桥接进程；关闭后等待日志 bridge_stopped，再启动另一份。
PICO 关闭画面后可重新 Listen；取流失败或发送阻塞时关闭视频连接，重新 Listen 建立新编码会话和 IDR。

## 旧 APK 的双眼偏移

已检查上游旧版 SampleRT 着色器：左右眼采样有 +0.08/-0.08 的水平纹理偏移。
本机头显的首次测试表现为单眼正常、双眼无法合成，因此提供发送端可撤销补偿：

```bash
python -m gear_sonic.scripts.run_pico_video_bridge --source test --layout stereo --eye-shift 0.08
```

在头显重新关闭/Listen后比较效果。--eye-shift 是图像宽度比例，**不是瞳距毫米数**。
头显瞳距保持使用者实际设置，不为修正视频软件偏移随意改变。
补偿需要头显人工确认；若安装版本不需要补偿，省略此参数（默认 0）。
真实摄像头使用测试通过的同一个 --eye-shift 值。
本轮不替换 APK；若现有显示仍无法正确合成或满足画面大小要求，必须报告限制后另行决定 APK 改动。

## 画面、带宽与延迟

- 默认 mono：一幅完整图等比例放入整个视频帧，供 App 平面视图显示；因旧 ZED 宽画幅可能两侧有黑边。
- 可选 stereo：同一幅图放入两个眼区域，用于诊断；当前旧 APK 双眼舒适性未通过，不作为默认方案。
- 默认采集上限 10 FPS；编码输出取请求 FPS 与 30 FPS 的较小值。编码可重复最新相机帧。
- 默认码率取请求码率与 4 Mbps 的较小值；禁用 B 帧及前瞻，每秒关键帧。
- 取图缓存仅保留最新一帧，编码输出最多缓存 3 个访问单元。
- 编码帧不能随意丢 P 帧；发送堵塞时结束该会话，重连重新生成 IDR，避免越积越慢或花屏。
- Annex-B 分帧使用 AUD，有一个编码帧的分界等待；30 FPS 时约 33 ms。
- RX age 是电脑收到并解码 JPEG 后经过的时间，不是摄像头曝光时间，也不是端到端延迟。
- 0.75 秒无新解码图像时输出明显的 VIDEO STALE 提示，恢复取图后自动恢复画面。
- 网络完全断开或编码器故障时无法送达提示图；必须实测头显如何显示断线，不能把旧画面当实时图像。
- 不声称零延迟；实际端到端延迟需在真实头显下另行测量。

## 网络、参数与日志

只在指定电脑地址监听 TCP 13579，仅接受 --headset 指定的头显 IP。
头显发 OPEN_CAMERA 后，电脑回连该头显请求的视频 TCP 端口（当前 12345）。
保留 --interface、--listen-host、--headset、--source-fps、--max-bitrate、--eye-shift 可配置。
若 IP 变化，显式修改命令；不自动改默认路由或防火墙，不向公网暴露服务。

每次运行生成独立目录 `~/pico-video-runs/日期时间-PID/`：

- events.jsonl：真实头显请求、连接、取图帧率、编码/发送统计、错误和关闭状态。
- encoder-N.log：每个编码会话的 FFmpeg 日志。

视频图像默认不录制到磁盘。关闭机器人动作程序不影响视频；关闭视频不应影响动作程序。
运行视频与遥操时仍需验证头显输入频率及控制周期没有明显退化。

## 开发测试

```bash
python -m unittest discover -s gear_sonic/tests -p test_pico_video_bridge.py -v
PICO_NVENC_TESTS=1 python -m unittest discover -s gear_sonic/tests -p test_pico_video_runtime.py -v
```

第一组测试覆盖协议分包/合包、尺寸/目标检查、H.264 分帧、双眼布局和补偿。
第二组仅连接 loopback，使用虚拟图像验证实际 NVENC 解码、视频失效画面、接收端断线和慢接收端背压。
10分钟原厂取图 → NVENC → 本机模拟头显验证脚本与结果位于 `/home/cqu023/pico-video-validation/`。
本机验证不能代替头显双眼融合、断线显示、端到端延迟及与遥操同时运行的验收。

协议参考：
- https://github.com/XR-Robotics/XRoboToolkit-Orin-Video-Sender/blob/main/main_zed_tcp.cpp
- https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/blob/main/Assets/Scripts/Li/SampleRT.shader


## 本轮验证结论

- 11 项协议/图像单元测试通过，3 项仅 loopback 的真实 NVENC 运行测试通过。
- 原厂相机经电脑编码向本机模拟头显连续发送 600 秒，接收 17988 个访问单元，
  平均约 30 FPS，最大帧间隔约 50.6 ms、P99 约 44.3 ms，编码队列高水位 1。
- 原厂取图约 10 FPS；期间发生 1 次取图错误并恢复。编码帧率包含重复画面，不等于相机更新率。
- 真实 PICO 已成功请求并显示彩色测试图，但用户反馈单眼正常、双眼不舒适。
  发送端 --eye-shift 0.08 补偿后仍未通过舒适性验收。测试视频已停止。
- 所以视频传输代码已实现，但现有 APK 的双眼布局尚未通过验收；
  尚未完成真实相机在头显中的显示、端到端延迟、断网表现和与全身遥操并行验证。
  不应把当前 ZEDMINI 测试显示直接用于操作机器人。
- 下一步先确定采用现有 App 平面视图，还是另行适配单目显示；不擅自替换 APK。


### 平面单目路线

用户选择保留 APK，先采用原 App 的平面大屏。新增 --layout mono（默认），发送端不做左右眼拼接或偏移。
仍选择 ZEDMINI 建立兼容连接，然后在视频打开后用右手柄 B 切换平面视图。
受旧配置 2560×720 宽画幅限制，完整 16:9 视频居中显示，左右留黑边；不通过拉伸来填满画面。
需由用户确认平面视图是否舒适，以及窗口大小能否满足需求；否则报告旧 APK 限制，不暗中替换应用。


## 平面显示实机确认

用户已先后确认：平面单目测试图完整、双眼舒适；切换真实机器人摄像头后，真实画面正常、双眼舒适。
因此当前交付默认使用 --layout mono，并在头显上用 B 切到平面视图，不采用 stereo 偏移补偿。
最终软件测试为 12 项协议/图像单元测试及 4 项本机 NVENC 运行测试通过。
用户确认关闭 Remote Vision 或摘下头显后，接收端背压结束了视频会话；这条记录不能作为观看中自行断流的证据。
同一轮原厂取图也有暂时失败后恢复的记录。不能把本机十分钟通过当作头显长时间运行已验证。
尚未测得 glass-to-glass 延迟，也未在身体/夹爪实际控制过程中做性能对照。

临时验证进程已停止。正常使用请按本文前台启动命令运行，退出时 Ctrl+C；本轮未执行 Git 暂存、提交或推送。
