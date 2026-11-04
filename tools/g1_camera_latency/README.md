# G1相机兼容计时版本

此目录在229准备，尚未安装到G1。不要直接覆盖运行中的程序。
robot_usb_stereo.original.py 是从G1只读取回的原文件。
原文件SHA256：23924bd67041c747e41ad286206123ca93dea0831a1d313dab76cc1c1804ff9c

候选文件 robot_usb_stereo.py 默认仍使用原来的N请求和四字节长度+JPEG响应。
仅显式添加 --latency-requests 才加载同目录 latency_trace.py，接受T+16字节请求UUID，并继续接受旧N请求。
保留原相机参数、监听地址、allow-peer和新鲜度校验。
相机硬件输出MJPEG，时间戳是完整JPEG在用户态可用，不是曝光或编码时间。

需要部署时：先备份G1原文件，核对原SHA，在维护窗口把候选程序与SDK放在同一目录；
保留原启动参数，并添加 --latency-requests，设置 LATENCY_TRACE_HOST=g1 和 LATENCY_TRACE_UDP=127.0.0.1:19555。
先启动G1本地collect，确认旧N客户端依然工作，再在229相机接收进程启动前设置 LATENCY_CAMERA_PROTOCOL=1。
绝不能对尚未升级的G1服务启用T请求。
未启用该环境变量的229客户端仍发原N请求；相机查看器接口不变。

G1本地记录示例：
    LATENCY_TRACE_HOST=g1 python3 latency_trace.py collect --output /home/unitree/outputs/latency/exp_002/g1.jsonl
在229将G1记录汇入本地收集器：
    ssh -T unitree@192.168.123.164 'tail -n +1 -F /home/unitree/outputs/latency/exp_002/g1.jsonl' | python3 /home/cqu023/DiT4DiT/tools/g1_latency_trace/latency_trace.py relay

同机选帧/发送调用可直接计算；G1→229单程需要两台机器各自连续有效的chrony证据。
不自动安装服务、不自动重启、不更改帧率或控制进程。
