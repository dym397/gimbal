# SFL0603 系统驱动接入说明

`sfl0603_driver.py` 是供主程序导入的驱动。`sfl0603_cli.py` 仍只用于人工串口测试，两者互不依赖。

## 安装依赖

```powershell
python -m pip install pyserial
```

## 安全启动

驱动默认执行以下安全策略：

1. 创建驱动时不会启动单次或连续测距。
2. 打开串口后立即发送 `0x00` 停止命令，并要求收到正确应答。
3. 默认锁住所有出光接口。只有主程序确认目标距离和现场安全后，调用 `arm_ranging()` 才能测距。
4. `close()` 和上下文管理器退出时会再次发送停止命令。

距离选通值只是数据处理参数，不能代替安全距离，也不能保护 APD 免受近距离强反射。

## 推荐调用方式

```python
from sfl0603_driver import SFL0603, TargetMode


with SFL0603("COM15") as laser:
    # 此时已确认进入停止状态，以下接口不出光。
    status = laser.self_test()
    shot_count = laser.query_shot_count()
    laser.set_nearest_distance(100)
    laser.set_target_mode(TargetMode.SINGLE)

    # 仅在主程序已经确认现场满足产品安全距离要求后执行。
    laser.arm_ranging()
    result = laser.measure_once()
    if result.valid:
        print(result.target_1_m)
```

## 连续测距

```python
from sfl0603_driver import SFL0603


laser = SFL0603("COM15")
try:
    laser.arm_ranging()
    laser.start_continuous(period_ms=1000)  # 1 Hz
    while running:
        result = laser.read_measurement(timeout=1.5)
        if result.valid:
            use_distance(result.target_1_m)
finally:
    laser.close()  # 自动发送停止命令
```

## 替换旧 SDDMLaser

驱动提供兼容方法：

```python
laser = SFL0603(port)
laser.arm_ranging()
laser.start_measurement(continuous=True)
distance_m = laser.read_distance()
laser.stop_measurement()
laser.close()
```

与旧驱动不同，`arm_ranging()` 是必须的安全步骤。不要在程序初始化或激光读取线程启动时自动调用它；应由系统状态机在确认云台指向、目标距离和现场条件后调用。

## 协议接口对应关系

| 协议命令              | 驱动方法                                        | 是否出光 |
| --------------------- | ----------------------------------------------- | -------- |
| `0x00` 待机/停止    | `stop_measurement()`                          | 否       |
| `0x01` 单次测距     | `measure_once()`                              | 是       |
| `0x02` 连续测距     | `start_continuous()` + `read_measurement()` | 是       |
| `0x03` 自检         | `self_test()`                                 | 否       |
| `0x04` 最近距离设置 | `set_nearest_distance()`                      | 否       |
| `0x06` 累计出光次数 | `query_shot_count()`                          | 否       |
| `0x22` 目标模式设置 | `set_target_mode()`                           | 否       |
| `0x26` 波特率设置   | `set_baudrate()`                              | 否       |

## APD 增益接口缺失

V0.005 首页的修改记录写有“增加 APD 增益模式设置和查询指令”，但正文的发送表和接收表都没有这两项，也没有命令字、参数值或响应定义。驱动中的 `set_apd_gain_mode()` 与 `query_apd_gain_mode()` 会抛出 `ProtocolDefinitionMissingError`，防止主程序误以为已经成功设置。需要向厂家索取补全后的协议页后才能实现。

## 运行离线测试

测试使用假串口，不会连接设备，也不会出光：

```powershell
python -m unittest -v test_sfl0603_driver.py
```
