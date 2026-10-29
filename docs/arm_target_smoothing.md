# 肩肘最终目标平滑（实验性、默认关闭）

用户选择真机连续异步控制入口。参数来自用户给出的调试起点，不是通用实机安全限值；已实现并做代码测试，不代表完成物理稳定性验证。不得据此宣称解决了此前首目标跳变根因。

## 启动入口

在仓库根目录预览（不启动进程）：

```bash
python3 gear_sonic/scripts/launch_dit4dit_field.py --inference-mode async --arm-target-smoothing gear_sonic/configs/arm_target_smoothing.experimental.json --dry-run
```

去掉`--dry-run`才会启动五终端。已有同名会话会退出提示，不自动杀进程或重启。启动器复制配置到本轮日志目录，记录其哈希，并显式传入`SONIC_ARM_TARGET_SMOOTHING`和`SONIC_STARTUP_AUDIT`。bottom应输出`[ArmTargetSmoother] shoulders/elbows only (experimental)`及参数；仅看到启动器打印不证明运行中的旧bottom已启用。

新功能与全身目标限速、single-step、hybrid入口互斥。未指定参数/调用端环境变量时显式清除tmux遗留设置，保持默认关闭。底层仍可直接读取该环境变量；simulation_only=true的配置只能在DDS38、lo且没有CYCLONEDDS_URI覆盖时启用。随附实验配置simulation_only=false用于用户请求的真机程序接入，不能被解释成已经验证安全。

## 数值与范围

- T_blend=0.5秒，tau=0.10秒，v_max=0.5 rad/s。
- 选定硬件关节索引15,16,17,18,22,23,24,25：左右肩pitch/roll/yaw和肘。
- 腰、腿、腕、夹爪不滤波；不插值token，不改观测历史中的原始last_action语义。
- 原始decoder输出完成排列/缩放/默认角度变换后，才进行平滑。
- 原始目标非有限，或选定关节原始目标超URDF位置范围即拒绝，不能通过滤波掩盖异常。URDF仅提供位置范围来源，不是平滑参数或稳定性证明。

## 时序

首次外部token解码有效后才启动过渡，之前PLANNER原样控制。初值取DDS Write返回成功后缓存的目标，不使用零值、当前原始decoder目标或仅入队未发送目标。Write成功只说明发布成功，不是机器人执行确认。

模式边界按decoder的using_encoder变化识别；切入外部token和返回encoder参考时都从最近成功发布目标重新开始。正常token更新/动作chunk不重置滤波器。进入INIT清空状态。

每周期：u=clip(elapsed/T_blend,0,1)，w=3u²−2u³；q_blend=(1−w)q_start+w*q_raw；alpha=dt/(tau+dt)；dq=clip(alpha*(q_blend−q_prev),±v_max*dt)；q_cmd=q_prev+dq。

50Hz正常dt=.02秒时alpha=1/6，每步上限.01rad。实际周期较短时采用较短dt；迟到时dt上限为原控制周期，不累积补步，过渡elapsed也只积累该dt。0.5秒是混合权重时间，不是保证到达时间。0.498rad目标差在0.5rad/s限速下单靠速度约束就至少需0.996秒。

停止、阻尼、原超时及增益不变；STOP不经过滤波，仅允许原阻尼命令通过新增发送保护。DDS发布失败、缺少新鲜的成功发布目标或本体反馈沿用停止处理。原停止处理可能进入阻尼，不是站立保持。此功能不新增未经定义的跟踪误差阈值，也不提供实际速度/加速度/碰撞/稳定性保证。

## 诊断与验证范围

arm_target_smoothed记录raw_target、last_published_target、smoothed_target、dt、alpha、blend_weight、elapsed、reseeded、using_encoder和control_tick。decoder_command保留原始目标；dds_write_attempt保留发送尝试。结合这些记录区分原始模型跳变和平滑结果。

纯算法测试覆盖首帧使用成功发布种子、smoothstep/低通公式、逐周期上限、长延迟不追赶、跨chunk状态保留、PLANNER返回重置、非选定关节原样通过、非法输入拒绝与INIT重置。启动器dry-run测试覆盖参数传递、映射和仿真配置拒绝。尚未进行动态仿真或实机验证，不自动发送arm/model。
