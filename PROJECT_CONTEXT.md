# PROJECT_CONTEXT.md

## 2026-04-16 最新进展
- 云台抢占控制已进一步解耦为“同目标按轴更新、切目标整条替换”。
- 当前方位/俯仰抢占基线为：
  - `AZ_PREEMPT_DEG = 0.5`
  - `EL_PREEMPT_DEG = 0.8`
- 当前控制线程行为：
  1. 若 `new_track_id != active_track_id`，视为目标切换，`Az/El` 整条命令一起替换。
  2. 若仍是同一 `track_id`，则分别比较 `dAz` 与 `dEl`：
     - `dAz > AZ_PREEMPT_DEG` 时只更新方位目标
     - `dEl > EL_PREEMPT_DEG` 时只更新俯仰目标
     - 两轴都满足则同时更新
- 这样做的目的，是避免在同一目标连续跟踪时，低方位阈值把俯仰目标也一起频繁刷新；同时又避免在切换到新目标时出现“新方位 + 旧俯仰”的混合指向。
- 当前这次代码变更后，尚未写入新的实机回归结果；后续应优先复测 `linear` 场景，验证方位误差是否不再呈现明显的周期性放大。

## 2026-04-14 最新进展
- 真实激光已正式接入主流程，不再只是保留 `LASER_PORT`、`raw_laser_dist` 和驱动文件但没有数据源。
- 当前激光数据路径：
  1. `main_tracking_v9.py` 在 `USE_MOCK_LASER=False` 时实例化 `SDDMLaser`
  2. 启动后台线程发送连续测量命令
  3. 后台线程循环读取真实串口数据并更新 `SharedHardwareState.raw_laser_dist`
  4. 云台 `GimbalSettled` 后读取最新激光缓存，并绑定到当前 `laser_track_id`
- 当前保留两种激光模式：
  - 真实激光：`USE_MOCK_LASER=False`
  - 模拟激光：`USE_MOCK_LASER=True`，使用当前主目标的新鲜 mono 距离模拟激光输入
- `sddm_laser.py` 已根据手册确认协议正确，当前实现使用：
  - 串口 `115200 / 8N1`
  - 连续测量命令 `MeaType=1, MeaTimes=0`
  - 数据帧头 `0xFB`
  - 距离单位 `dm -> m`
  - 附加校验：`MsgCode=0x03`、`PayloadLen=0x04`
- 最新实机验证结果：
  - 真实云台 `COM3` 与真实激光 `COM8` 已可同时启动
  - 15 秒 `linear` 场景下，云台 `GimbalSettled=19`、`GimbalTimeout=0`
  - 同场景下，真实激光 `Laser Triggered=19`、`Laser No valid=0`
  - 激光测到的是现实场景距离 `4.7m ~ 6.5m`，说明代码已使用真实激光；发送脚本中的 `320m` 只代表视觉输入，不代表激光会返回 320m
- 2026-04-14 Linux 追加联调结果：
  - 发送端对 `10.72.2.28:8888` 反复执行 `15FPS / 15s / linear / cx=700 -> 3686.67 / cy=1080 / mono=320m`
  - `logs/main_tracking_v9_20260414_161730.log` 中，激光线程和连续测量均正常启动，但所有 settled 均退回 mono distance，说明“主流程未拿到有效激光值”可以在室内近距离/弱反射目标条件下出现
  - `logs/main_tracking_v9_20260414_162745.log` 中，仅调整激光指向更远目标后即恢复 `Laser Triggered=22`、`Laser No valid=0`，实测距离约 `6.2m ~ 7.9m`
  - 当前可确认：激光链路与主流程绑定逻辑是通的，剩余问题集中在“什么目标条件下会返回有效值”的边界验证

## 本文件用途
给第一次接触项目的人快速建立工程全貌：系统做什么、数据如何流动、哪些模块与硬件耦合、当前控制基线是什么。实现细节和待办请继续阅读 `TODO_NEXT.md` 与 `DECISIONS.md`。

## 项目名称
支持 Windows 调试与 Linux 部署的云台目标追踪与测距系统

## 主入口
- `main_tracking_v9.py`

## 项目目标
接收 UDP 目标检测，进行连续追踪，驱动 GT06Z 云台进行实时跟随，并将数据回传。针对真实物理延迟进行闭环调优。

## 当前进度摘要
- 主流程已在 `main_tracking_v9.py` 集成：UDP 输入、目标归一化、Kalman 追踪、目标选择、云台控制、UI 回传。
- 当前代码已支持双平台串口默认值切换：Windows 默认 `COM3` / `COM4` / `COM5`，Linux 默认 `/dev/ttyUSB0` / `/dev/ttyUSB1` / `/dev/ttyUSB2`；实际运行时仍可通过 `GIMBAL_PORT` / `LASER_PORT` / `IMU_PORT` 覆盖。
- 当前调优依据来自 `logs/main_tracking_v9_20260409_154819.log`：15FPS、320m、5.1m/s、水平匀速直线运动，`cx=700 -> 3686.67`，`cy=1080`。
- 已完成第一轮控制修正并在 2026-04-16 继续演进：Kalman `MIN_DT=0.001`、抢占阈值改为 `AZ_PREEMPT_DEG=0.5` / `EL_PREEMPT_DEG=0.8`、`GIMBAL_SETTLE_THRESHOLD=0.3`、启动时云台归位到 `Az=GIMBAL_AZ_BASE` / `El=0.0°`。
- 尚未完成：`PREDICT_DELAY` 双轴拆分、首次捕获/稳定跟踪的独立参数状态机、激光有效工作距离/反射条件边界验证。

---

## 端到端管线 (End-to-end pipeline)

### 1. 上游输入与检测归一化
接收板卡/相机/BBox/距离数据，并通过 `parse_udp_objects` 进行归一化。

### 2. 目标追踪 (Kalman Tracking with Dynamic DT)
系统包含基于卡尔曼滤波器的追踪器 (`StandardKalmanTrack`)。
- **状态量**：方位角、俯仰角、方位角速度、俯仰角速度。
- **自适应时间步长**：追踪器的预测步长 `dt` 必须基于实际的视频帧率（如 15FPS 约等于 0.066s）动态计算，避免系统性低估目标速度。

### 3. 双阶段追踪控制模型 (Dual-Phase Tracking Model)
为了平衡响应速度与防抖，系统区分了两种控制阶段：
- **首次捕获模式 (Initial Acquisition)**：目标刚出现或刚切换时，采用激进的参数实现“快速拉近”。
- **稳定跟踪模式 (Stable Tracking)**：目标处于视野中心区域后，采用平滑的参数来“抑制抖动”。

### 4. 解耦的云台控制与前馈补偿 (Decoupled Gimbal Control & Feedforward)
- **多轴独立预测**：由于俯仰轴 (Pitch) 的机械响应往往慢于方位轴 (Pan)，`PREDICT_DELAY` 必须双轴解耦（例如 `AZ_PREDICT_DELAY` 和 `EL_PREDICT_DELAY`）。
- **抗震荡迟滞 (Anti-thrashing Hysteresis)**：指令抢占阈值 (`PREEMPT_DEG`) 与到位阈值 (`SETTLE_THRESHOLD`) 必须保持合理的间距，避免云台在“快到位但又被新命令刷新”的边缘状态下反复横跳。
- **初始位姿修正**：系统需处理云台初始位姿过远（如俯仰 -9° 到 28.5° 需要 >2s）带来的初次锁定迟缓问题。

### 4.1 当前控制基线 (2026-04-10)
- `main_tracking_v9.py` 已将 Kalman `MIN_DT` 从固定 `0.08` 调整为 `0.001`，仅作为异常极小 `dt` 保护，避免 15FPS (`~0.066s`) 场景被强行抬高。
- 抢占阈值已在 2026-04-16 更新为 `AZ_PREEMPT_DEG=0.5`、`EL_PREEMPT_DEG=0.8`；`GIMBAL_SETTLE_THRESHOLD=0.3` 保持不变。当前策略是同目标连续跟踪时按轴更新、切换目标时整条替换，并继续利用云台约 `0.1°` 精度收紧到位判定。
- 系统启动后会先通过现有 `gimbal_cmd_queue` 投递初始化姿态命令：方位角 `GIMBAL_AZ_BASE`，俯仰角 `0°`。串口下发仍由 `gimbal_control_thread` 单一控制线程执行。
- 尚未完成 `PREDICT_DELAY` 的双轴拆分；当前仍是单一 `PREDICT_DELAY=0.25`，俯仰轴慢的问题先通过启动位姿初始化进行部分缓解，后续仍需垂直/斜向运动测试后再调 `AZ_PREDICT_DELAY` / `EL_PREDICT_DELAY`。

### 5. 共享状态与传感器融合
通过 `SharedHardwareState`（使用线程锁保护）融合最新有效的单目、激光、IMU 数据，放弃严格时间戳对齐，改为基于 TTL 的最新有效值融合。

---

## 硬件相关模块 (Hardware-related modules)
- `gimbal_interface.py` & `GT06Z_gimbal.py`: 抽象接口与底层驱动。
- `sddm_laser.py` & `hwt905_driver.py`: 独立串口传感器的后台读取线程。

## 运行环境与架构风格
双平台运行环境。Windows 调试时串口设备使用 `COMx` 命名；Linux 部署时串口设备使用 `/dev/ttyUSB*` / `/dev/ttyACM*` 命名。默认端口已按平台切换，实际运行前仍需按设备管理器或 Linux 设备节点确认。硬件调试导向，强调日志覆盖率、显式线程控制与物理真实性。
