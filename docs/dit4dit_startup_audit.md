# DiT4DiT 启动跳变离线审计（2026-10-29）

本轮未启动、重启或切换实机控制。未调整增益、超时、动作过期门槛，也未添加姿态过渡。三次启动均按现场报告视为异常样本。

## 已证实

按单调时钟排序：

- 203518：start=2354871.820911629；预填充为请求259的窗口索引23；首个gateway model帧为索引31，观测年龄0.627594秒。
- 203841：start=2355002.74701752；预填充为请求62的窗口索引19；首个gateway model帧为索引27，观测年龄0.545477秒。
- 203831：start=2355104.892231097；预填充为请求59的窗口索引23；首个gateway model帧为索引31，观测年龄0.627718秒。

所有索引从0开始。启动前后窗口内286条发送token均唯一匹配到已记录预测，最大逐项误差0。gateway的frame.index是发送计数，不是窗口索引。gateway model帧标志反映CONTROL心跳确认，不能当作底层首次解码周期。

28份窗口内输入均保留31维状态。最近一次启动前身体采样分别早于start约219.7、114.2、122.3毫秒。其硬件顺序29维绝对关节角和两维夹爪实测位置已恢复到audit.json。它们不是启动瞬间的精确反馈。

三次窗口内预测按实际sonic_normalization.py重新反归一化，与记录动作逐项一致，float32最大误差0。夹爪保留连续电机输出弧度，不使用7+7字段。不同值关节及非零偏置测试通过；当前robot_model还原的训练身体29维排列为硬件顺序。

32推理资源与229副本匹配：config SHA256 e6fe46f709bc801df9f0e413c90ebb196d68ae0d5bbb2245724a8db7c1e98a26；统计 b6a675604088c80298375e0b43fb9f2dfaa36203f8af4e94c34431f1d04e0463；19000 checkpoint 83271d85fc3b541b0437b960a0268cd51cb8e143e068e7f11a4f3d8de2bf7198。decoder/encoder/配置/关节参数哈希见audit.json。

## 已排除的范围

排除审计窗口内“记录的归一化预测被错误反归一化”和“32、229这三份资源副本不同”。静态映射及合成关节测试未发现重复加偏置。上述结论不等于原训练标签来源与SONIC编码器版本已独立验真：历史标签未提供可验证的编码器哈希，仍需追溯标签生成材料。

## 候选原因与证据不足

源码显示启动外部token不是从窗口0开始，预填充至CONTROL确认期间还继续按观测年龄前进。此事实不能单独证明跳变原因。

SONIC v1.1 decoder输入994维，含64维token和10帧身体/角速度/速度/历史动作/重力方向。StateLogger历史不足时补零，last_action初始化为零。GatherRobotStateToLogger在CONTROL路径内执行；必须核对首次历史内容，不能把零历史导致跳变写成已证实。

源码将raw_action按isaaclab_to_mujoco重排，乘g1_action_scale再加default_angles，随后写入电机命令缓冲；LowCommandWriter逐项复制q到DDS。增益缩放不改变q。当前正常SONIC路径未看到首目标连续性检查或目标过渡。

历史日志没有首个CONTROL周期完整994维输入、实际消费token、原始decoder输出、29维目标和DDS写出记录。因此每关节首目标差值、最大突变关节、确切参考/航向重置时刻和最早突变层均为“证据不足”，不能靠静态截图补算。正常退出、token已收到均不构成动作正确验收。

## 新增工具

- `gear_sonic/scripts/audit_model_startup.py`：启动事件、token候选匹配、NPZ状态、资源哈希审计。重复token保持歧义。
- `gear_sonic/scripts/replay_sonic_startup.py`：只接受完整decoder记录；复用独立C++ `replay_sonic_decoder`，实际调用原PolicyEngine/TensorRT。输入不完整或记录丢失即拒绝，不填零。
- `SONIC_STARTUP_AUDIT=/absolute/new_trace.jsonl`：底层诊断环境变量，默认未设置则关闭。本轮未启用实机采集。记录等待状态、token接收/发送编号、参考重置、实际使用token、完整输入、原动作、目标映射和DDS写出尝试。后台队列容量1024；丢失计数非零或缺少完整结束记录，重放拒绝验收。文件应每次使用新路径；这是诊断开关，不是动作许可。

诊断只新增记录，不改变启动条件。DDS记录表示发布尝试，不证明电机实际响应；相同token或目标导致关联歧义时保留全部候选。现场身份必须另附资源哈希、原启动命令和运行版本。

## 离线运行

在229仓库中运行：

```bash
.venv_data_collection/bin/python -m unittest gear_sonic.utils.inference.test_startup_audit gear_sonic.utils.inference.test_readonly_body
.venv_data_collection/bin/python -m gear_sonic.scripts.audit_model_startup \
  --runs outputs/model_only_20261028_203518 outputs/model_only_20261028_203841 outputs/model_only_20261028_203831 \
  --statistics /home/cqu023/DiT4DiT/results/sonic_dex1_52eps_joint_bs4_20261027_161659/dataset_statistics.json \
  --normalization-source /home/cqu023/DiT4DiT/DiT4DiT/dataloader/gr00t_lerobot/sonic_normalization.py \
  --output outputs/startup_audit_rerun.json
# 仅有未来完整诊断文件时使用；历史events.jsonl会被拒绝。
.venv_data_collection/bin/python -m gear_sonic.scripts.replay_sonic_startup \
  --trace /absolute/complete_trace.jsonl \
  --decoder gear_sonic_deploy/policy/sonic_v1_1/model_decoder.onnx \
  --output /absolute/replay_report.json
```

C++目标：`cmake --build gear_sonic_deploy/build --target replay_sonic_decoder -j 2`。该程序不构造G1Deploy或DDS通道。必须使用匹配FP32部署引擎；若现场使用FP16，当前重放入口不作为其等价验收。

重放容差预设为decoder rtol=1e-3、atol=1e-4；目标映射rtol=atol=1e-6；运行时编译映射与捕获映射比较atol=1e-12。禁止看到结果后放宽。

## 验证边界与下一步

诊断底层及独立解码器已编译通过。映射、唯一/重复token与缺失输入测试已通过。合成完整994维输入只验证重放工具，不代替现场历史输入，也不用于认定身体动作正确。

继续保留停止边界。下一步需要完整decoder诊断输入，优先在离线或仿真路径采集；若只能现场获得，另行安排有人看护的记录流程，本轮不执行。证据齐全且映射、资源、参考切换核对通过后，再另行设计关节目标层过渡及限值，禁止直接插值64维token。

实际验证结果：26项Python回归测试通过；原PolicyEngine对3组明确标注的合成994维输入重复推理，最大输出差值为0.0。历史events缺少完整输入时，在启动推理前被拒绝。异步诊断器默认关闭、启用后完整刷出及零丢失结束标志的独立测试通过。合成结果保存在outputs/startup_audit_20261029/synthetic_parity.json。
