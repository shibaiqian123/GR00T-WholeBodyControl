# USB 双目相机到 PICO

机器人新 USB 相机 -> 有线网络 MJPEG -> 工作站 NVENC H.264 -> PICO ZEDMINI。
原来的 run_pico_video_bridge 不变，仍用于原厂相机。新入口是真实左右双目，非单目复制。

## 启动

机器人 unitree@192.168.123.164：
```bash
/usr/bin/python3 ~/pico_usb_stereo/robot_usb_stereo.py
```
稳定设备路径对应 USB2.0 Camera RGB 的 video-index0，配置为 2560x720 MJPEG 30 fps。
仅监听机器人 192.168.123.164:15556，允许电脑 192.168.123.99 访问。

电脑 cqu023@10.242.3.229：
```bash
cd /home/cqu023/GR00T-WholeBodyControl
source .venv_teleop/bin/activate
python -m gear_sonic.scripts.run_pico_usb_stereo \
  --camera-host 192.168.123.164 \
  --listen-host 192.168.11.224 \
  --headset 192.168.11.228
```
PICO: Remote Vision -> ZEDMINI -> Listen -> 192.168.11.224。
按右手柄 B 切换至立体模式，分别遮挡相机镜头确认眼序。反向时重启电脑入口加 --swap-eyes。
仅在确认旧 APK shader 存在偏移时测试 --eye-shift 0.08；默认不补偿。
新入口与旧入口都用 13579，不能同时启动。前台运行 Ctrl+C 停止。

## 实现和验证边界

机器人仅采集 UVC MJPEG，不访问机器人运动接口。缓存只保存最新 JPEG，电脑按需请求。
电脑解码后严格要求 2560x720，拆分两张 1280x720 并分别适配头显左右眼。
保留旧桥的帧过期提示、连接目标校验、编码背压终止机制。
USB 原始取流 10 秒得到 299 帧、0 错误；真实相机到本机模拟 PICO 编码测试接收 150 个 H.264 访问单元，解码成功，无过期帧，队列最高 1。
尚需实际 PICO 左右眼融合、长时间稳定性、端到端延迟验证。左右眼镜头清晰度差异也需要现场检查。

## 本次临时后台运行

机器人 PID 见 /tmp/pico-usb-stereo.log 启动对应进程；电脑 PID 保存到 /tmp/pico-usb-bridge.pid。
电脑后台日志 /tmp/pico-usb-bridge.log；详细日志 ~/pico-video-runs/usb-*。
不设置开机自启，重启后按上面顺序启动。停止后台进程时核实命令后只给对应 PID 发送 TERM。

## 独立画面大小选项
新参数 --image-scale 0.85 将每眼图像以中心为基准缩小到 85%，周围补黑边。
范围 0.5 到 1.0，默认 1.0 保持原效果。--eye-shift 仍以输出单眼宽度为基准，
缩放不会把该补偿一起缩小。两个参数可以组合。
这只调整图像的视觉大小，不等于调整 Unity 虚拟屏幕距离，也不是毫米单位。
缩放同时改变图像内部视差，舒适性仍需头显实测。重启桥并重新 Listen 生效。

## 上下移动
--vertical-shift 0.04 将两眼画面一起向下移动输出高度的 4%；负数向上，0 保持原位置。范围 -0.25 到 0.25。位移在缩放后应用，不改变左右眼相对高度。超出视频帧的内容会裁掉，原位置补黑。与 --image-scale 0.85 合用时，可先试 0.04。它移动视频内容，不移动 PICO 的虚拟显示面板。重启并重新 Listen 生效。

## 电脑实时预览
在电脑图形桌面终端运行原命令并加 --preview，默认显示原始左右并排图像。
可选 --preview-eye left 或 right 只看一路。预览不套用头显的偏移、缩放。
复用同一个相机连接，不另开机器人取流。即使 PICO 未连接也可预览。
按 Q/Esc 或关闭窗口只关闭预览，PICO 传输继续；终端 Ctrl+C 停止整个桥。
无新帧超过 0.75 秒显示 NO LIVE CAMERA IMAGE，而不是保留旧图冒充实时。
普通 SSH 无图形桌面时不能直接打开预览，应在电脑本机桌面终端运行。

## 右眼单目 2D 模式
启动入口增加 --view right --preview，仅发送原始右半幅相机画面，电脑预览默认也只看右眼。
去掉 --eye-shift、--swap-eyes、--image-scale、--vertical-shift，避免混用立体补偿。
PICO 仍选 ZEDMINI 建立连接，再按 B 切换平面模式。两眼看同一块屏幕，不再产生双目深度。
现有 APK 请求 2560x720，右眼 1280x720 等比例居中，左右黑边是预期行为。
--view left 可换左镜头，--view stereo 保留原立体路线。关闭当前进程后重启才生效。

## PICO 1.1.1 右眼 16:9 配置
头显 ZEDMINI 配置已改 CamWidth=1280、CamHeight=720、CamFPS=30、RawImageRectSize=600x337.5。PICO4U 配置不变。备份：/home/cqu023/pico-config-backups/20261023-111245-right-mono/video_source.original.yml。重启头显应用再 Listen，按 B 使用平面视图。发送端必须使用 --view right（或 left），当前配置不适合 stereo；恢复双目前先恢复备份配置。配置写入已回读验证，实际头显显示仍待确认。


## Record the USB right eye while viewing it in PICO

The robot USB service on port 15556 supports one active client. Keep the PICO
bridge as that client; do not connect the ZMQ data exporter directly to 15556.

Add `--publish-camera-port 5555` to `run_pico_usb_stereo --view right`.
The bridge publishes `ego_view` over ZMQ at `127.0.0.1:5555`.
Run the data exporter with `--camera-host 127.0.0.1 --camera-port 5555`.
The bridge must remain running throughout recording. After a bridge restart,
press Listen again in the PICO ZEDMINI panel if the video does not reconnect.

The complete 1280x720 right eye is resized to 640x360 and padded vertically to
the existing 640x480 dataset schema; this does not change the PICO image.
Camera rate is approximately 30 FPS; the exporter remains at its existing
50 Hz, so adjacent dataset samples can share a camera frame. Image timestamps
are desktop receive times, not hardware exposure timestamps. The publisher
stops publishing stale source images; the exporter skips samples whose images
are older than 0.5 seconds.

Local zero-frame datasets can be resumed without downloading Hub metadata.
Missing indices are initialized only when metadata reports zero frames and
episodes and no episode parquet files are present. Nonempty incomplete
datasets fail locally rather than attempting to download tmp/tmp_dataset.

Validation: synthetic right-eye/color/shape and stale-stream tests passed;
live reception 149 frames in 5 seconds; temporary dataset resume plus
30-frame MP4 encode/decode passed. Full robot-state/action episode recording
has not been validated by these camera-only checks.
