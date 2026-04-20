# AGENTS.md

## 2026-04-16 增量说明
- 云台抢占逻辑已从“单一总阈值 + 整条命令替换”升级为“分轴阈值 + 同目标按轴更新”。
- 当前抢占基线：
  - `AZ_PREEMPT_DEG = 0.5`
  - `EL_PREEMPT_DEG = 0.8`
- 当前执行规则：
  - **同一 `track_id` 连续跟踪**：哪个轴超过各自阈值，就只更新哪个轴的目标值。
  - **`track_id` 发生切换**：必须整条命令替换，`Az/El` 一起更新，避免出现“新方位 + 旧俯仰”的混合指向。
- 当前抢占日志已补充 `mode=track_switch/axis_update` 与 `axes=Az/El/Az+El`，便于从日志直接判断是目标切换，还是同目标下的单轴更新。
- 本次更新只涉及控制线程内的抢占更新逻辑，没有改动 `PREDICT_DELAY`、`SETTLE_THRESHOLD`、激光流程或外部协议。

## 2026-04-14 增量说明
- 激光链路已从“只有接口、未接主流程”升级为“真实可运行”状态：`main_tracking_v9.py` 现会在 `USE_MOCK_LASER=False` 时启动 `SDDMLaser` 后台线程，持续读取真实激光并写入 `SharedHardwareState.raw_laser_dist`。
- 已新增 `USE_MOCK_LASER`，语义与 `USE_MOCK_GIMBAL` 一致：
  - `True`: 激光通道使用当前主目标的 mono 距离做模拟输入
  - `False`: 激光通道使用真实 SDDM 激光串口
- 当前真实硬件基线端口：云台 `COM3`，激光 `COM8`。部署到 Linux 时仍优先通过 `GIMBAL_PORT` / `LASER_PORT` / `IMU_PORT` 覆盖。
- SDDM 激光当前采用“连续测量”而不是“单次测量”。原因是主流程是在 `GimbalSettled` 后读取一份最新缓存值，连续测量更符合现有非阻塞控制结构。
- `sddm_laser.py` 已按手册补强帧校验：除 CRC 外，还校验了数据帧 `MsgCode=0x03` 与 `PayloadLen=0x04`。
- 最新实测结论：
  - 短时测试里曾出现只有无效激光帧、主流程退回 mono 距离
  - 15 秒长测中，真实激光已稳定触发 19 次，`[Laser] No valid laser distance` 为 0
  - 真实激光返回的是实物环境距离（约 `4.7m ~ 6.5m`），不是发送脚本里的模拟 `320m`
- 2026-04-14 Linux 联调补充：
  - 接收端运行在 `10.72.2.28:8888`，串口为云台 `/dev/ttyUSB0`、激光 `/dev/ttyUSB1`
  - `logs/main_tracking_v9_20260414_161730.log` 显示：激光线程已启动、连续测量命令已发，但整场测试都只有 `[Laser] No valid laser distance`；该现象当前优先解释为室内目标距离/反射条件不满足，而不是主流程未触发
  - `logs/main_tracking_v9_20260414_162745.log` 显示：将激光指向更远目标后恢复稳定真实测距，`GimbalSettled=22`、`GimbalTimeout=0`、`Laser Triggered=22`、`Laser No valid=0`，实测距离约 `6.2m ~ 7.9m`

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
- 抢占阈值已在 2026-04-16 更新为分轴基线：`AZ_PREEMPT_DEG=0.5`、`EL_PREEMPT_DEG=0.8`；`GIMBAL_SETTLE_THRESHOLD=0.3` 保持不变，用于维持到位迟滞区间。
- 启动时通过 `gimbal_cmd_queue` 投递初始化姿态命令，目标为 `Az=GIMBAL_AZ_BASE`、`El=0.0°`，仍由单一云台控制线程下发。
- 串口默认值按平台切换：Windows 为 `COM3` / `COM4` / `COM5`，Linux 为 `/dev/ttyUSB0` / `/dev/ttyUSB1` / `/dev/ttyUSB2`。
- `PREDICT_DELAY` 尚未拆分为 `AZ_PREDICT_DELAY` / `EL_PREDICT_DELAY`；后续如修改预测提前量，仍必须保持 Pan/Pitch 解耦。

---

## 未来编辑的预期行为 (Expected behavior for future edits)
- **安全修复模式**: 处理异常时必须优雅降级，严禁导致主循环崩溃。
- **控制调优模式**: 调整控制逻辑时，必须区分“首次跟踪 (Initial Acquisition)”和“稳定跟踪 (Stable Tracking)”的不同需求；必须确保控制死区和抢占阈值之间有足够的“迟滞 (Hysteresis)”。
- **重构模式**: 仅在明确要求时才执行此操作。
- **激光联调模式**: 如果日志里已出现 `[Init] Real laser enabled ...`、`[LaserThread]`、`[Laser] start_measurement mode=continuous`，但持续没有有效距离，先检查目标距离、反射条件与瞄准方向，再怀疑主流程代码回归。

---

## AI 不应“好心”更改的事项
**严禁**自动执行以下操作：
- 将 UDP 替换为 TCP 或将所有多线程替换为异步。
- 将卡尔曼滤波器时间步长 `dt` 硬编码为固定常数（必须依赖实际帧率计算）。
- 将方位角和俯仰角的预测延迟强行统一合并。
- 移除云台控制中的抢占和到位迟滞保护逻辑。
