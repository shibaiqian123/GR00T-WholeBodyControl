# 单 token 解码、单次电机发送、回到 PLANNER

## 当前交付边界

实现位于 `single_step_endpoint.hpp`、底层 `CreatePolicyCommand/LowCommandWriter` 和 `gear_sonic/scripts/sonic_single_step.py`。默认关闭。已做协议/模拟发送门控测试及编译；没有进行 MuJoCo 整机闭环测试或实机测试，不能据此认定切换连续性、安全站立或抓取通过。

该入口不会启动底层、不会从 INIT/WAIT 启动控制、不会启用 PLANNER，也不连接32推理服务、不控制Dex1。用户指定的64维 token 独立于完整66维动作中的夹爪目标；此次不发送夹爪动作。

## 执行语义

1. 底层已处于 CONTROL，PLANNER enabled/initialized，当前参考确为 planner_motion，encoder有效，LowState符合原50ms新鲜度要求，入口才报告 ready。ready是模式与反馈条件，不是机器人已经站稳的证明。
2. 请求包含当前底层会话ID、唯一request_id、64维有限token和显式execute=true。接收线程不发送电机命令。
3. 下一正常控制周期使用正常构造的完整994维观测，只在decoder浮点输入的前64维替换指定token。只调用一次decoder；不清空历史、不重置航向、不插值token、不改增益和超时。
4. 检查最终29维目标的位置范围，以及它与该次实际关节位置的偏差；越界则进入原停止流程，不发送该目标。
5. 电机写线程对这份目标只尝试一次DDS Write。后续500Hz回调不会重复发送它；不自动重试失败发送。
6. 下一正常控制周期恢复PLANNER生成的token及原历史（包括刚才的last_action）。同时检查PLANNER首目标相对实测位置、以及相对上一单帧目标的偏差。通过后发送PLANNER目标，返回complete和followup_confirmed=true。其后继续原PLANNER控制。
7. 停止、反馈异常、PLANNER条件丢失、decoder失败、目标越界、发送失败，或到下一控制周期仍未确认前次发送，都结束事务并沿原停止流程处理。停止不会因新请求解除。

**单次DDS发送不是“一次物理动作”或固定时长脉冲。** 电机控制器可能维持该目标，直至收到下一PLANNER或停止指令；本入口不保证已到达目标。`dds_write_succeeded`仅是SDK Write返回成功，`reached_target`始终为false。请求窗口内可能已经发生运动，失败也不能称为“未执行”。

## 开启前必需配置

只在另行安排的验证运行中设置：

- `SONIC_STARTUP_AUDIT`：独立审计文件；不开日志就拒绝启用。
- `SONIC_SINGLE_STEP_CONFIG`：配置JSON绝对路径。

配置字段：`port:15558`、`followup:"planner"`、`units:"rad"`、`joint_order:"hardware_29"`；`q_min`、`q_max`、`max_delta`必须各含29个有限数值；`constraint_source`记录限值来源。没有默认限值，也不能用测试里的合成数值代替机器人约束。必须根据匹配的机器人关节约束及现场允许的单次目标变化核对配置。缺少配置时15558不会监听。

只绑定127.0.0.1:15558，端口占用即报错。不会使用5556或修改已有gateway。若现有gateway仍在发布模型动作，不能将它与此次PLANNER单帧测试混用。

本次没有创建实机限值配置，也没有重启底层或启用此入口。

## 客户端

默认只查询（不会启动底层）：

```bash
cd /home/cqu023/GR00T-WholeBodyControl
.venv_teleop/bin/python -m gear_sonic.scripts.sonic_single_step
```

仅在限值核对及仿真验证完成、现场人工准备好后，显式请求：

```bash
.venv_teleop/bin/python -m gear_sonic.scripts.sonic_single_step \
  --execute --token /absolute/path/token_step0.npy \
  --output /absolute/path/new-single-step-directory
```

不确定是否成功、连接超时或客户端中断时，不重新发送新请求；查询原目录：

```bash
.venv_teleop/bin/python -m gear_sonic.scripts.sonic_single_step \
  --output /absolute/path/existing-single-step-directory
```

保存request.json、receipt.json和receipt_history.jsonl。回执含994维输入、原始decoder动作、最终目标、该周期实际位置、DDS返回结果和PLANNER恢复目标。审计文件另记single_step_phase、single_step_decoded、decoder_command、dds_write_attempt。检查审计footer确认是否丢记录。

同一会话同一ID重复请求返回原回执，不重解码、不重发。相同ID不同token拒绝。会话重启后旧session拒绝；内存回执不能跨重启查询，应查已保存日志，不自动重试。

## 尚待验证

- 填写并核对实机约束来源；不能任意给max_delta。
- 隔离MuJoCo中验证真实控制线程与DDS发送路径、O停止、恢复PLANNER时序、目标连续性及日志无丢失。
- 确认是否存在首目标跳变；本功能不修复原跳变，也不是姿态过渡方案。
- 在上述完成前保持实机模型切换暂停。


## 隔离仿真配置与启动（2026-10-29）
prepare_sim_single_step.py 从匹配 G1 URDF 读取硬件顺序29关节的位置和速度范围，
生成 simulation_only 配置；max_delta=URDF velocity*0.02秒，仅为仿真测试规则，
不是经实机验证的安全目标跳变量，也不是允许放宽既有限制的依据。
记录来源文件哈希和关节名称。仿真配置在创建DDS实体之前拒绝非域38、非lo及
CYCLONEDDS_URI覆盖。没有仿真入口自动退回实机配置的行为。

保持域38/lo的MuJoCo运行；停止旧的仿真底层后，在仓库根执行
bash gear_sonic/scripts/launch_sim_single_step_bottom.sh。该命令创建独立日志及配置，
启用15558；不会自动进入PLANNER或发送token。
仍需通过15556发送一次 start=true,stop=false,planner=true，再查询单帧接口ready。
ready=false或超限必须停下排查；不自动重试、不改限值。
单帧之后仍按原实现返回PLANNER；不是INIT直接下发路径，不控制Dex1。
初次status超时的文案现在明确说明本次调用没有发送执行请求。
