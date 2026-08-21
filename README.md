# RTOS-CAN

STM32H750 + FreeRTOS + FDCAN 与 ESP32-S3 + TWAI 的双向 Classic CAN 通信工程。

当前阶段已经完成 STM32 CAN 应用层工程化整理、FDCAN 硬件过滤、ESP32 Heartbeat、节点状态机、CAN 错误状态监测，以及 Bus-Off 后的 FDCAN 外设重初始化恢复路径。CANopen 尚未引入。

## 1. Project Overview

- STM32H750 运行 FreeRTOS，使用 FDCAN1 Normal Mode。
- ESP32-S3 使用 TWAI，作为对端 CAN 节点。
- STM32 周期发送业务帧 `0x120`。
- ESP32 周期发送业务帧 `0x321` 和 Heartbeat `0x701`。
- STM32 RX 路径保持为：FDCAN IRQ -> RX FIFO0 Callback -> CMSIS-RTOS Message Queue -> CAN RX Task。
- STM32 CAN 应用逻辑集中在 `can_app.c` / `can_app.h`，`freertos.c` 仅保留 Queue/Task 创建和 `CAN_App_Init()` glue code。

## 2. Hardware

STM32 侧：

- MCU：STM32H750VBT6
- FDCAN：FDCAN1
- FDCAN RX：PB8
- FDCAN TX：PB9
- CAN Mode：Normal

ESP32 侧：

- Board：Waveshare ESP32-S3-Touch-LCD-2.8
- TWAI TX：GPIO18
- TWAI RX：GPIO15
- 外部 CAN Transceiver：3.3 V TXD/RXD 接口

## 3. Architecture

```text
STM32H750 / FreeRTOS / FDCAN1

can_tx_task
  -> HAL_FDCAN_AddMessageToTxFifoQ()
  -> CAN ID 0x120
  -> CAN Bus

CAN Bus
  -> Hardware Filter: 0x321 / 0x701 only
  -> FDCAN RX FIFO0
  -> FDCAN1_IT0_IRQHandler
  -> HAL_FDCAN_RxFifo0Callback()
  -> osMessageQueuePut(canRxQueue)
  -> can_rx_task
  -> business frame / heartbeat processing

can_tx_task + can_rx_task
  -> CAN_App_UpdateBusStatus()
  -> bus_state / recovery
```

## 4. CAN Protocol

| Direction | CAN ID | Type | DLC | Period | Purpose |
| --- | --- | --- | --- | --- | --- |
| STM32 -> ESP32 | `0x120` | Standard Data Frame | 8 | 1000 ms | STM32 business frame |
| ESP32 -> STM32 | `0x321` | Standard Data Frame | 8 | 1000 ms | ESP32 business frame |
| ESP32 -> STM32 | `0x701` | Standard Data Frame | 8 | 500 ms | ESP32 Heartbeat |

Formal bitrate：500 kbit/s。

STM32 FDCAN bit timing 保持 CubeMX 当前配置：

- NominalPrescaler = 10
- NominalSyncJumpWidth = 3
- NominalTimeSeg1 = 12
- NominalTimeSeg2 = 3
- AutoRetransmission = ENABLE
- FrameFormat = Classic CAN

## 5. FreeRTOS RX Pipeline

STM32 接收链路：

```text
FDCAN IRQ
  -> HAL_FDCAN_RxFifo0Callback()
  -> osMessageQueuePut()
  -> canRxQueue
  -> can_rx_task
```

`can_rx_task` 只处理来自 Queue 的帧。ISR/Callback 不直接执行复杂业务逻辑。

## 6. Hardware Filter

STM32 FDCAN 使用 Standard ID Dual Filter：

- `FilterID1 = 0x321`
- `FilterID2 = 0x701`
- 匹配帧进入 RX FIFO0

Global Filter：

- 未匹配 Standard ID：Reject
- 未匹配 Extended ID：Reject
- Standard Remote Frame：Reject
- Extended Remote Frame：Reject

因此只有 ESP32 业务帧 `0x321` 和 Heartbeat `0x701` 能进入 STM32 RX FIFO0。

## 7. Node State Machine

ESP32 节点状态由 Heartbeat 驱动，和 CAN 总线状态独立。

```text
CAN_NODE_INIT
  -- first valid heartbeat --> CAN_NODE_ONLINE

CAN_NODE_ONLINE
  -- heartbeat age > 1500 ms --> CAN_NODE_OFFLINE

CAN_NODE_OFFLINE
  -- valid heartbeat --> CAN_NODE_RECOVERING

CAN_NODE_RECOVERING
  -- 3 consecutive heartbeats --> CAN_NODE_ONLINE
  -- timeout again --> CAN_NODE_OFFLINE
```

保留统计：

- `heartbeat_rx_count`
- `last_heartbeat_tick`
- `heartbeat_age_ms`
- `heartbeat_timeout_count`
- `esp32_online`
- `node_state`
- `node_state_change_count`
- `recovery_heartbeat_count`

## 8. CAN Bus State

CAN 总线状态来自 FDCAN Protocol Status 和 Error Counters，严重程度：

```text
CAN_BUS_OK
CAN_BUS_WARNING
CAN_BUS_ERROR_PASSIVE
CAN_BUS_OFF
CAN_BUS_RECOVERING
```

判断优先级：

```text
BUS_OFF > ERROR_PASSIVE > WARNING > OK
```

Bus state 与 node state 是两个独立状态源：

- ESP32 断电时，可能是 `CAN_BUS_OK` + `CAN_NODE_OFFLINE`。
- CAN 总线 Bus-Off 时，可能是 `CAN_BUS_OFF`，同时 node state 按 Heartbeat 自行变化。

## 9. Bus-Off Fault Injection

Bus-Off fault injection 仅用于诊断测试，正式默认关闭。

默认值：

- STM32：`BUS_OFF_TEST = 0`
- ESP32 LCD sketch：`BUS_OFF_TEST = 0`
- ESP32 TX test sketch：`BUS_OFF_TEST = 0`

开启测试后可通过 ESP32 串口命令控制：

- `FAULT ON`：停止 TWAI，将 TXD GPIO18 切为 GPIO 输出，产生短 dominant pulse。
- `FAULT OFF`：停止 pulse，GPIO18 输出 HIGH，等待稳定，再恢复 TWAI 500 kbit/s。
- `FAULT STATUS`：查看 pulse 统计。

正式模式下不需要 FAULT 命令，也不会改变 500 kbit/s、`0x321` 业务帧、`0x701` Heartbeat。

## 10. Bus-Off Recovery

当前正式恢复路径使用 Level 2：FDCAN Peripheral Reinitialization Recovery。

```text
CAN_BUS_OFF
  -> backoff
  -> CAN_BUS_RECOVERING
  -> stop normal TX
  -> HAL_FDCAN_Stop()
  -> HAL_FDCAN_DeInit()
  -> __HAL_RCC_FDCAN_FORCE_RESET()
  -> __HAL_RCC_FDCAN_RELEASE_RESET()
  -> MX_FDCAN1_Init()
  -> CAN_App_Start()
  -> controller ready check
  -> CAN_BUS_OK
```

恢复期间不重建：

- FreeRTOS Scheduler
- `canTxTask`
- `canRxTask`
- `canRxQueue`

`CAN_App_Start()` 会重新配置：

- Hardware Filter
- Global Filter
- Interrupt Line
- RX/TX/Error Notification
- `HAL_FDCAN_Start()`

Controller ready check 至少确认：

- HAL FDCAN state 为 BUSY
- `CCCR.INIT == 0`
- `PSR.BusOff == 0`
- Interrupt Line 0 enabled
- required IE notification bits enabled
- interrupt mapping 正确
- TXBRP 无异常 pending
- TX FIFO/Queue 未满

注意：STM32H7 的 FDCAN peripheral reset 会影响 FDCAN 共享外设域；当前工程只正式使用 FDCAN1，因此本阶段允许使用该恢复方式。

## 11. Verified Result

已验证的正式行为：

- STM32 <-> ESP32 双向 CAN 通信稳定。
- STM32 `0x120` 周期发送，周期 1000 ms。
- ESP32 `0x321` 周期发送，周期 1000 ms。
- ESP32 `0x701` Heartbeat 周期 500 ms。
- STM32 Heartbeat timeout 为 1500 ms。
- FDCAN Hardware Filter 只允许 `0x321` 和 `0x701`。
- 非目标 Standard ID、Extended ID、Remote Frame 在硬件层 Reject。
- RX 软件链路保持 ISR -> Queue -> RX Task。
- Bus-Off fault injection 默认关闭，诊断时可手动开启。

## 12. Repository Structure

```text
Core/
  Inc/
    can_app.h          STM32 CAN application API, state and statistics
    fdcan.h            CubeMX generated FDCAN declarations
  Src/
    can_app.c          STM32 CAN application, callbacks, node/bus state, recovery
    fdcan.c            CubeMX generated FDCAN init and MSP init
    freertos.c         Queue/task creation and CAN_App_Init glue code

ESP32_CAN_HMI/
  Arduino/
    ESP32_CAN_HMI_LCD/
      ESP32_CAN_HMI_LCD.ino    ESP32 LCD HMI + formal TWAI node + optional fault injection
    ESP32_CAN_TX_TEST/
      ESP32_CAN_TX_TEST.ino    Minimal ESP32 TWAI test sketch
```

## 13. Build / Run

STM32：

- Open `MDK-ARM/FreeRTOS+FDCAN.uvprojx` with Keil MDK.
- Build and flash the STM32H750 target.
- Keep `BUS_OFF_TEST = 0` for formal operation.

ESP32：

- Open `ESP32_CAN_HMI/Arduino/ESP32_CAN_HMI_LCD/ESP32_CAN_HMI_LCD.ino`.
- Build with Arduino ESP32 support.
- Keep `BUS_OFF_TEST = 0` for formal operation.
- Connect TWAI TX/RX to the CAN transceiver TXD/RXD pins.

## 14. Roadmap

Next stage:

- CANopen feasibility/design analysis
- NMT / PDO / SDO / Object Dictionary planning

Not implemented yet:

- CANopen
- NMT
- PDO
- SDO
- Object Dictionary
