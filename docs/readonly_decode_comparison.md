# WAIT 与 PLANNER 同 token 只读解码对照

此功能不切换模式，不启动实机，不向机器人发送外部 token。底层仍是原控制程序，启动底层及进入 PLANNER 本身有电机输出，不能把底层启动称为只读。实机模型切换继续暂停。

## 采样开关

默认关闭。在另行批准的底层运行中，同时设置 `SONIC_STARTUP_AUDIT=/absolute/path/bottom.jsonl` 和 `SONIC_READONLY_DECODE_SNAPSHOT=1` 才采样。此文不提供实机启动或模式切换命令。

- WAIT_FOR_CONTROL：安全检查通过后，以当前实测位置/速度/IMU、现有 last_action 和历史构造预期首周期 994 维输入。前64维是占位，离线替换。原生历史填充行记录在 padding_rows，不能称为实测历史。
- CONTROL_PLANNER：仅 planner enabled/initialized、当前参考确为 planner_motion 且使用 encoder 时，复制已经构造完成的实际 obs_buffer；采样发生在原正常 decoder 之前，不追加历史、不运行第二次推理、不写电机缓存。
- `motor_written=0` 只描述采样回调；原 PLANNER 继续原有控制。
- PLANNER 模式不代表站稳。日志记录 q/dq、IMU、采样时间和控制周期；稳定性标记 not_assessed。选样前必须检查连续窗口与现场观察，不能凭一帧状态自动判定稳定。
- 采样会增加日志量；必须检查原审计 footer 的丢记录/写入错误，不能将缺失历史当连续证据。

## 离线输入

从日志选择并单独保存一个 WAIT `readonly_decode_snapshot` JSON 对象和一个 PLANNER JSON 对象（保留完整字段）；使用同一64维 `.npy` token。工具读取保存快照，不要求它们仍然新鲜，也绝不将它们重新发送实机。

在仓库根目录：

```bash
.venv_teleop/bin/python gear_sonic/scripts/compare_decode_snapshots.py \
  --wait-snapshot /absolute/path/wait.json \
  --planner-snapshot /absolute/path/planner.json \
  --token /absolute/path/token_step0.npy \
  --decoder gear_sonic_deploy/target/release/replay_sonic_decoder \
  --model gear_sonic_deploy/policy/sonic_v1_1/model_decoder.onnx \
  --output /absolute/path/new-comparison-directory
```

输出目录必须不存在。输出包含资源SHA256、两个完整输入、原始动作、解码日志、目标及与实测位置的逐关节差值。校验994/64/29维、有限值、排列合法性及独立decoder映射/缩放/默认角一致性；不补缺失数据。

结果为“相同token在两种捕获状态下的反事实解码”，不等于真的执行PLANNER→模型切换，不覆盖切换时可能重置的参考、航向和历史。WAIT输入是预测构造，PLANNER输入是实际捕获，必须保持该证据区别。两者姿态不同也会影响结果。不能仅据目标差减小就恢复实机。

当前完成源代码与编译/纯函数测试；未采集新实机PLANNER样本，未完成同token实测对照，也未完成单帧执行确认协议。
