# 全身统一接管与逐关节滤波（schema 2）

这是连续模型控制代码修改，不是实机验收。没有启动机器人、修改增益、延长超时或放宽位置限位。之前的启动日志保留在outputs/preserved_arm_startup_20261029_204016，不参与覆盖或迁移。

## 范围与顺序

在decoder输出经过实际关节排列、缩放和默认角度转换后，对29维身体q_target统一计算接管过渡。左右Dex1另走原控制服务，不包含在29维中。不插值64维token。

等第一条有效外部token解码目标到达后，使用最后一次DDS Write成功发布的全身目标q_start。Write成功不是机器人执行确认；另检查当前实测q及反馈新鲜度。每个关节的abs(last_sent_q−actual_q)必须不超过必填max_start_tracking_error；否则按既有故障停止处理。此门槛只在接管/返回参考时检查，不是新增持续跟踪误差保护。

统一w=3u²−2u³，u=clip(elapsed/T_blend,0,1)。所有29关节先q_blend=q_start+w*(q_raw−q_start)，再逐关节执行alpha=dt/(tau[i]+dt)，delta=alpha*(q_blend−last_sent_q)，可选限速±v_max[i]*dt。下一周期基于最后成功发布的目标，不使用仅放进buffer尚未发送的值。

初始PLANNER保持原样；切入外部token后开始统一接管，返回encoder参考时重新检查跟踪偏差并统一接管。正常chunk和token更新不重置，INIT清空状态。STOP/阻尼绕过所有滤波并保持优先级；单帧和hybrid与此入口互斥。普通CONTROL motor buffer写入前的统一处理覆盖全部29个身体目标。

## 配置

`gear_sonic/configs/joint_target_smoothing.draft.json`是不能执行的草案：

- T_blend=0.5秒，29个关节共用一个权重。
- 肩肘tau=.10秒、v_max=.5rad/s沿用用户的实验起点。
- 腰腿和手腕tau=0、v_max=null：只有接管过渡，不增加持续低通和限速；这些组以后可以独立设置。
- `max_start_tracking_error` 29项均为null、`tracking_error_source`为空。用户已确认暂未提供经过核对的阈值。启动器和底层均拒绝此配置，不会替用户生成任意默认阈值。
- 位置限位和关节顺序沿用原matched URDF配置，不扩大范围。29个关节全量验证顺序、有限数值和位置范围；原始目标越界就拒绝，不因混合权重小而掩盖。

手腕是有限位关节，数值按合法区间直接处理；不对目标取模，不自动使用最短角度差。未来如果更换连续旋转关节，需要单独核对编码器/接口语义，此实现不默认为连续关节。

## 启动接口

新的参数为`--target-smoothing`，对应底层`SONIC_TARGET_SMOOTHING`。旧`--arm-target-smoothing`/SONIC_ARM_TARGET_SMOOTHING明确报错，避免误以为旧肩肘配置已覆盖全身。两种平滑都默认关闭。

下面只检查草案，预期报错，绝不启动进程：

```bash
python3 gear_sonic/scripts/launch_dit4dit_field.py --inference-mode async --target-smoothing gear_sonic/configs/joint_target_smoothing.draft.json --dry-run
```

补齐有来源的跟踪阈值后，另存新配置，再dry-run验证。不能把合成测试值当成实机参数。已有运行进程不会被代码修改自动更新。

## 诊断与局限

joint_target_smoothed记录统一blend_weight、dt、elapsed、逐关节alpha、raw_target、last_published_target、actual_q及smoothed_target。decoder_command保留原目标，dds_write_attempt记录发送尝试。第一帧w=0，全部29维等于最后发布目标。

dt取实际间隔与原控制周期的较小值，不累积补发；调度迟到不会突然推进整个接管过渡。过渡时间是权重变化时间，不保证所有有持续限速的关节同时到达目标。w接近1后未滤波关节的持续突变仍会传出。该方法改变全身闭环时序，不能证明平衡或解决原始解码发散；原始last_action语义保持不变。

测试覆盖：29个不同值的统一首帧与共同权重，选定关节独立低通限速，跨chunk状态，长延迟不追赶，最后成功发布值而非待发值，跟踪误差拒绝，PLANNER返回、有限位手腕、不完整配置拒绝。仅测试与离线公式检查，不运行实机。
