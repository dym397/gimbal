# PROJECT_CONTEXT.md

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
- 已完成第一轮小范围控制修正：Kalman `MIN_DT=0.001`、`GIMBAL_PREEMPT_DEG=1.5`、`GIMBAL_SETTLE_THRESHOLD=0.3`、启动时云台归位到 `Az=GIMBAL_AZ_BASE` / `El=0.0°`。
- 尚未完成：`PREDICT_DELAY` 双轴拆分、首次捕获/稳定跟踪的独立参数状态机、激光测距完整集成验证。

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
- `GIMBAL_PREEMPT_DEG=1.5`，`GIMBAL_SETTLE_THRESHOLD=0.3`。当前策略是让抢占阈值显著大于到位阈值，利用云台约 `0.1°` 精度收紧到位判定，同时降低小幅新命令反复抢占。
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
