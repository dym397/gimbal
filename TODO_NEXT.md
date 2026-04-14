# TODO_NEXT.md

## 2026-04-14 状态更新

### 已完成
- [x] 将真实激光 `sddm_laser.py` 接入 `main_tracking_v9.py` 主流程
- [x] 新增 `USE_MOCK_LASER`，使真实/模拟激光切换方式与 `USE_MOCK_GIMBAL` 保持一致
- [x] 明确当前激光模式应使用“连续测量”，而不是“单次测量”
- [x] 按手册检查并补强 SDDM 激光驱动的数据帧校验
- [x] 完成真实硬件联调：云台 `COM3`，激光 `COM8`
- [x] 完成 15 秒 `linear` 长测：真实激光成功触发 19 次，`[Laser] No valid laser distance` 为 0

### 当前结论
- 主程序已经真正使用真实激光，而不是继续只用 mono distance。
- 当激光回包有效时，`[Laser] Triggered ... dist=...m` 会在 `GimbalSettled` 后打印。
- 真实激光返回的是当前物理环境中的反射目标距离，最近一轮长测约为 `4.7m ~ 6.5m`；这与发送脚本中模拟的 `320m` 不矛盾。

### 下一步优先级
1. 固定一个明确的真实反射目标，验证激光距离是否与该实物距离一致，而不是仅确认“有值”。
2. 若激光偶发回到无效帧，补抓原始激光回包，确认是盲区、反射条件，还是串口噪声。
3. 继续复测 `15FPS linear` 场景，记录 `GimbalSettled / GimbalTimeout / Laser Triggered` 的长期稳定性。
4. 若需要让激光距离更严格地对应当前追踪目标，再评估是否增加更强的“云台稳定窗口”或多次激光采样确认逻辑。

## 本文件用途
给下一位接手者快速了解“已经做了什么、下一步要先验证什么、哪些任务还没开始”。如果需要理解为什么做这些改动，请看 `DECISIONS.md`；如果需要理解系统整体架构，请看 `PROJECT_CONTEXT.md`。

## 当前阶段 (Current phase)
基于真实动态场景（如 320m 距离，5.1m/s 速度）的卡尔曼速度估算与云台闭环控制专项调优。

## 下一步优先级
1. 先完成目标运行平台的串口配置复核：Windows 下确认 `COMx` 端口，Linux 下确认 `/dev/ttyUSB*` / `/dev/ttyACM*` 设备节点，并同步到 `GIMBAL_PORT` / `LASER_PORT` / `IMU_PORT`。
2. 再复测已改参数：15FPS `linear` 场景，确认速度估计、抢占次数、到位/超时比例、俯仰首捕获时间。
3. 如果速度估计正常但跟随仍滞后，再做 `PREDICT_DELAY` 双轴拆分。
4. 如果低速/高速目标需求冲突明显，再实现 Dual-Phase Tracking，把首次捕获和稳定跟踪参数分离。
5. 激光测距集成排在控制闭环稳定之后，除非当前测试必须依赖真实激光距离。

## 已完成 (Done)
- [x] 完成主运行程序及多模块集成的基础搭建。
- [x] 通过 `udp_sender_tracking_scenarios.py` 完成了 `linear` 场景的高速测试 (320m, 5.1m/s)。
- [x] 生成并分析了专项测试日志 `main_tracking_v9_20260409_154819.log`。
- [x] 确诊了卡尔曼速度低估的根因：`MIN_DT=0.08` 与实际 15 FPS 不匹配。
- [x] 确诊了云台震荡的根因：`PREEMPT` (0.7) 与 `SETTLE` (0.8) 阈值过于接近。
- [x] 已在 `main_tracking_v9.py` 将 `MIN_DT` 调整为 `0.001`，仅保留异常极小 `dt` 保护。
- [x] 已将迟滞参数调整为 `GIMBAL_PREEMPT_DEG=1.5`、`GIMBAL_SETTLE_THRESHOLD=0.3`。
- [x] 已新增启动位姿初始化：系统启动后先通过云台控制队列下发 `Az=GIMBAL_AZ_BASE`、`El=0.0°`，保持单一控制线程所有权。
- [x] 文档和代码运行背景已调整为双平台；串口默认值和校验逻辑已支持 Windows `COMx` 与 Linux `/dev/...`。

## 当前关注的痛点/问题 (Current concerns)
1. **俯仰初始化慢**：已通过启动位姿初始化部分缓解，但需要真实硬件复测确认首次锁定时间是否明显缩短。
2. **速度估算保守**：代码已移除 `MIN_DT=0.08` 的实际帧率抬高问题，需用 15FPS `linear` 场景复测估算速度是否接近理论 `~0.91°/s`。
3. **参数一刀切**：“快速拉近”和“抑制抖动”仍未完全分离，后续仍需要 Dual-Phase Tracking 参数化。
4. **边缘横跳 (Thrashing)**：迟滞参数已初步调整，需复测 `preempt` 是否不再在 `0.71~0.74°` 附近频繁触发。
5. **动力学未解耦**：单一的 `PREDICT_DELAY=0.25` 仍未拆分为方位/俯仰独立提前量。

## 推荐的后续核心技术任务 (Recommended next technical tasks)
- [x] **自适应时间步 (Adaptive Kalman DT)**：已将固定 `MIN_DT=0.08` 改为异常保护值 `0.001`，让 Kalman 使用实际收包间隔；待复测确认速度估算。
- [ ] **双轴独立前馈 (Decoupled Feedforward)**：将 `PREDICT_DELAY` 拆分为 `AZ_PREDICT_DELAY` 和 `EL_PREDICT_DELAY`，为较慢的俯仰轴设定更大的预测提前量。
- [ ] **追踪状态机 (Dual-Phase Tracking)**：区分“首次跟踪 (Acquisition)”与“稳定跟踪 (Stable)”，两阶段应用不同的预测权重和死区参数。
- [x] **扩大控制迟滞 (Widen Hysteresis)**：已设为 `SETTLE_THRESHOLD=0.3`、`PREEMPT_DEG=1.5`；待硬件复测观察 `GimbalSettled` 与 `GimbalTimeout` 比例。
- [x] **初始化位姿优化 (Init Posture)**：已在启动时预设云台到 `Az=GIMBAL_AZ_BASE`、`El=0.0°`；待硬件复测确认首捕获延迟。
- [ ] **集成激光测距 (`sddm_laser.py`)**：确保后台线程稳定及 Checksum 校验。

## 下一次启动检查清单 (Next startup checklist)
1. 阅读 `AGENTS`, `PROJECT_CONTEXT`, `TODO_NEXT`, `DECISIONS`。
2. 打开 `main_tracking_v9.py`。
3. 在目标平台确认 GT06Z、激光、IMU 的串口设备名，并同步到 `GIMBAL_PORT` / `LASER_PORT` / `IMU_PORT` 配置。
4. 检查当前基线：`MIN_DT=0.001`、`GIMBAL_PREEMPT_DEG=1.5`、`GIMBAL_SETTLE_THRESHOLD=0.3`、启动归位 `Az=GIMBAL_AZ_BASE` / `El=0.0°`。
5. 运行 `linear` 测试 (320m, 5.1m/s, 15FPS, `cx=700 -> 3686.67`, `cy=1080`)。
6. 重点观察日志中的：预估速度是否接近 `~0.91°/s`、是否消除了 `0.71~0.74°` 附近的边缘抢占、`GimbalSettled` 是否稳定触发、是否出现更多 `GimbalTimeout`、俯仰角首捕获是否更快。
