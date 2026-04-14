# AGENTS.md

## 本文件用途
这是给 AI/新接手工程师的项目守则。先读本文件了解硬约束和安全边界，再读 `PROJECT_CONTEXT.md` 建立架构全貌，读 `TODO_NEXT.md` 看当前进度，读 `DECISIONS.md` 理解决策原因。

## 项目目标 (Purpose)
本仓库是一个支持 Windows 调试与 Linux 部署的实时云台追踪与测距系统。

主要职责：
- 接收来自 RK3588 或发送端脚本的 UDP 目标检测数据
- 解析检测数据并将其转换为归一化的目标观测值
- 使用基于卡尔曼滤波（Kalman-based）的逻辑对目标进行连续追踪，并根据帧率自适应时间步长
- 选择一个活动目标进行云台追踪
- 将 UI/世界坐标系下的角度转换为云台控制角度
- 通过串口控制 GT06Z 云台，应用前馈预测以补偿机械延迟
- 读取激光/IMU 相关的硬件状态（若启用）
- 通过 UDP 将目标状态发送回 UI

主入口点：
- `main_tracking_v9.py`

测试发送脚本：
- `udp_sender_tracking_scenarios.py`

---

## 开发优先级 (Development priorities)
1. 运行稳定性与真实硬件的安全性
2. 串口通信的正确性
3. 动态帧率下的追踪连续性与速度估算准确性
4. 初始捕获速度 (Acquisition Speed) 与稳定追踪精度 (Tracking Smoothness) 的平衡
5. 低延迟表现与向后兼容性

---

## 强制规则 (Hard rules)

### 1. 严禁随意重写整个项目
这是一个与硬件深度耦合的项目。优先选择局部、最小化且可控的代码修改。

### 2. 除非明确要求，否则不要更改外部协议格式
UI 输出格式、GT06Z 串口协议与坐标系约定必须保持一致。

### 3. 尊重云台多轴动力学的不对称性
水平方位轴 (Pan) 与俯仰轴 (Pitch) 的响应速度不同。在修改预测提前量 (Predictive Lead) 时，必须支持两轴解耦。

### 4. 保留双平台串口假设与控制线程所有权
默认串口设备名按平台切换：Windows 使用 `COMx`，Linux 使用 `/dev/ttyUSB*` / `/dev/ttyACM*`；也可通过 `GIMBAL_PORT` / `LASER_PORT` / `IMU_PORT` 显式指定。云台指令由单一控制线程统一下发。

### 5. 任何影响控制行为的更改都必须说明：
- 更改了什么 (例如：调整了 `PREEMPT` 或 `SETTLE_THRESHOLD`)
- 为什么更改 (例如：拉大差距以防止指令频繁重置/反复横跳)
- 可能的硬件风险及验证方法

---

## 代码修改策略 (Code modification strategy)
优先理解现有逻辑 -> 最小必要范围修改 -> 保持现有接口 -> 必要时添加注释 -> 保留调试打印风格。

---

## 重要文件 (Important files)
- `main_tracking_v9.py`: 主运行程序，包含自适应帧率的卡尔曼滤波与双阶段(捕获/稳定)状态机。
- `gimbal_interface.py`: 云台适配接口。
- `GT06Z_gimbal.py`: 底层串口驱动。
- `sddm_laser.py` / `hwt905_driver.py`: 传感器底层驱动。
- `udp_sender_tracking_scenarios.py`: 测试发送器。

---

## 当前控制基线 (Current control baseline)
截至 2026-04-10，`main_tracking_v9.py` 的当前控制基线为：
- Kalman `MIN_DT=0.001`，仅保留异常极小 `dt` 保护，避免 15FPS 场景被固定下限抬高。
- `GIMBAL_PREEMPT_DEG=1.5`，`GIMBAL_SETTLE_THRESHOLD=0.3`，用于拉开抢占/到位迟滞区间。
- 启动时通过 `gimbal_cmd_queue` 投递初始化姿态命令，目标为 `Az=GIMBAL_AZ_BASE`、`El=0.0°`，仍由单一云台控制线程下发。
- 串口默认值按平台切换：Windows 为 `COM3` / `COM4` / `COM5`，Linux 为 `/dev/ttyUSB0` / `/dev/ttyUSB1` / `/dev/ttyUSB2`。
- `PREDICT_DELAY` 尚未拆分为 `AZ_PREDICT_DELAY` / `EL_PREDICT_DELAY`；后续如修改预测提前量，仍必须保持 Pan/Pitch 解耦。

---

## 未来编辑的预期行为 (Expected behavior for future edits)
- **安全修复模式**: 处理异常时必须优雅降级，严禁导致主循环崩溃。
- **控制调优模式**: 调整控制逻辑时，必须区分“首次跟踪 (Initial Acquisition)”和“稳定跟踪 (Stable Tracking)”的不同需求；必须确保控制死区和抢占阈值之间有足够的“迟滞 (Hysteresis)”。
- **重构模式**: 仅在明确要求时才执行此操作。

---

## AI 不应“好心”更改的事项
**严禁**自动执行以下操作：
- 将 UDP 替换为 TCP 或将所有多线程替换为异步。
- 将卡尔曼滤波器时间步长 `dt` 硬编码为固定常数（必须依赖实际帧率计算）。
- 将方位角和俯仰角的预测延迟强行统一合并。
- 移除云台控制中的抢占和到位迟滞保护逻辑。
