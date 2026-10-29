# 连续 SONIC 关节目标限速（默认关闭）

用途：对映射、缩放、默认角度转换完成后的29维身体关节目标进行逐周期速度限制。不是64维token插值，不处理Dex1两维，不保证机器人保持平衡。尚未进行仿真或实机动态验证。

启用变量：`SONIC_JOINT_TARGET_LIMITS=/absolute/path/limits.json`，在底层进程启动前设置。未设置时原路径不变。与单帧执行、hybrid入口互斥，不改变现有超时、增益、停止与阻尼路径。

JSON必需字段：`units="rad"`、`joint_order="hardware_29"`、非空`constraint_source`以及29项数组`q_min`、`q_max`、`max_velocity`（rad/s）、`max_tracking_error`（rad）。各项按policy_parameters.hpp的g1_action_scale硬件关节顺序配置。必须来自经过核对的控制约束；本改动不提供任意实机默认限值。仿真配置设置`simulation_only=true`，要求DDS38、lo且无CYCLONEDDS_URI覆盖。

算法：首次CONTROL从上一条已提交电机目标起步，之后从上一限速目标推进。每关节步长≤max_velocity × min(实际周期, control_dt)。延迟不累积补步。所有常规CONTROL目标（包括PLANNER及返回PLANNER）连续限速；INIT不受影响并清空限速历史。STOP/阻尼绕过限速。dq、tau、kp、kd不变。它限制的是目标位移速度，不是电机实际速度或加速度。

异常：原始目标、实际状态或前一目标非有限/超位置范围、前一/下一目标相对实测位置跟踪误差超限、时钟不推进或低层反馈过期，返回控制错误并沿用停止处理。不会将异常原始目标夹紧后当成正常值执行。

历史动作仍保留原decoder输出语义，未改为限速后的反归一化动作；这是需要仿真评估的分布变化。目标限速可能妨碍平衡控制，尤其腿部，不能视为实机切换问题已解决。

诊断：开启SONIC_STARTUP_AUDIT后，原decoder_command保持原始目标；joint_target_limited记录raw_target、limited_target、actual_q、control_tick、using_encoder；dds_write_attempt记录实际发送尝试。审计不是硬实时执行确认。

验证：test_joint_target_limiter.cpp使用每关节不同速度值，覆盖方向、启动种子、迟到不追赶、短周期、重置、NaN、越界、跟踪误差和失败不修改历史。测试参数为合成值，不能用于实机。随后应在隔离仿真验证稳定性、停止优先级及原始/限速/DDS目标链路，再考虑部署。

## 五终端入口

`launch_dit4dit_field.py --inference-mode async --joint-target-limits /实际配置路径.json --dry-run`检查配置结构并打印bottom命令，不创建tmux、不发送动作。去掉dry-run才启动进程；这不是实机验证许可。现有会话不会被自动重启。

启动器优先读取显式参数，其次读取调用终端SONIC_JOINT_TARGET_LIMITS；两者都没有时在bottom命令中显式清除该变量，避免tmux遗留环境悄悄启用限速。启用时将校验后的配置复制到本轮日志目录并记录SHA256，同时开启bottom_target_audit.jsonl。实际启用要检查bottom日志中的[JointTargetLimiter] enabled，不应仅根据dry-run判断正在运行的进程。

此入口是实机入口，拒绝simulation_only配置。结构校验不能证明限值合理。仿真仍使用隔离DDS38/lo入口。
