# 隔离仿真启动诊断

实机模型切换保持暂停。本入口只复用实时 MuJoCo、SONIC 底层和历史 token；不启动相机、Dex1 服务或32推理服务。默认仅预检查，运行需要 `--execute`。

## 运行

在229仓库根目录：

```bash
source .venv_teleop/bin/activate
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
python -m gear_sonic.scripts.run_sim_startup_audit \
  --output /home/cqu023/GR00T-WholeBodyControl/outputs/sim_startup_audit_NEW \
  --execute --manual-ready
```

使用新的输出目录。每轮独立启动：
`/home/cqu023/sonic-diagnostics/sonic-realtime-v1/run_sim_realtime.py --interface lo --dds-domain 38`。
用户提供的实时入口未改动；物理步长、原有增益、超时及停止保护不变。底层直调已编译二进制，不调用实机部署启动脚本。DDS选择域38、网卡lo；ZMQ动作/状态为回环15556/15557。端口冲突即退出，不杀其他进程。

必须等待终端显示 `WAITING FOR OPERATOR`，在仿真窗口调绳子。原模拟器按键7降低绳索长度、8升高、9切换启用；不要用9代替降低。操作完成并确认姿态稳定后，在另一终端创建提示中的该轮 `operator.ready` 文件。只放行当前轮，不预先创建后三轮文件。绳索运行时开关、长度、锚点和实际施力会记录；改变长度是实验条件，不代表与实机初态一致。

每次按原事件单调时间顺序播放原token、帧计数和start请求，不额外发送中立token。发送落后时不突发补发；累计调度偏移超过一个50Hz周期即停止该轮。此限值是重放质量门槛，不是改变底层控制超时。独立后续样本仍可运行，但失败样本不会自动重试。

## 输出与验证

- manifest.json：资源哈希、版本、域/接口和仿真配置；source.diff：已跟踪源码改动。
- commands.json：每轮完整启动命令；sim.log、bottom.log：进程输出。
- replay.jsonl：人工放行、支撑状态、原帧、计划/实际发送时间、底层心跳和停止。
- sim.jsonl：实际关节位置/速度、完整qpos/qvel和绳索状态；sim.xml/mjb：实际模型快照及哈希。
- bottom.jsonl：接收token、等待状态、参考/航向事件、完整994输入、解码结果、29关节目标、DDS发送尝试。
- results.json：各轮通过/失败。日志有丢记录、调度失败、无CONTROL、原数据不足启动后2秒时均不通过。

```bash
python -m gear_sonic.scripts.analyze_sim_startup_audit --output /path/to/run
```

analysis.json 保存首目标差值、后续逐关节轨迹、token归属候选、历史输入分段及已捕获完整输入的独立TensorRT复算结果。部分输入数值一致不修复丢记录，不等价于整段重放通过。严格 `replay_sonic_startup.py` 仍拒绝任何缺失尾标或dropped非零的完整性不合格日志。

注意：SDK可能在同进程重复初始化时输出domain初始化提示；必须结合后续有效状态判断，不能据此单独宣布通信失败。DDS套接字可能显示0.0.0.0监听；指定lo和domain38是配置证据，不等价于抓包证明所有物理接口均无流量。

## 当前限制

C++诊断写入采用try-lock，多线程同时记录时会丢条目并在audit_end计数。丢记录时禁止宣称完整首周期链路验收通过。需要改进记录器后单独重测，不能把dropped改成0或删除失败尾标。

首目标与当前姿态存在差值，不自动证明延迟是根因。还需核对首token的原窗口索引、首周期历史输入补零、默认参考/航向初始化，以及实际DDS目标；禁止插值64维token或放宽超时掩盖跳变。
