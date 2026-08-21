# RTOS-CAN CANopenNode Port Plan

本文件是 CANopenNode 移植阶段的分析与计划文档。本轮不修改现有 STM32/ESP32 代码，不添加 CANopen 源码，不修改 CAN ID、FDCAN bit timing、FreeRTOS 任务或 ESP32 业务逻辑。

参考源码/文档：

- CANopenNode core: https://github.com/CANopenNode/CANopenNode
- CANopenNode driver API: https://canopennode.github.io/CANopenNode/group__CO__driver.html
- CANopenNode default CAN IDs: https://canopennode.github.io/CANopenNode/group__CO__Default__CAN__ID__t.html
- CANopenNode STM32 port: https://github.com/CANopenNode/CanOpenSTM32
- CANopenNode STM32 port README: https://github.com/CANopenNode/CanOpenSTM32/blob/master/README.md

## 1. CANopenNode 架构

CANopenNode 可以分成五层看待：

1. CANopenNode core

   通用协议栈代码，位于 CANopenNode 仓库中，例如 `CANopen.c/h`、`301/CO_NMT_Heartbeat.*`、`301/CO_HBconsumer.*`、`301/CO_SDOserver.*`、`301/CO_SDOclient.*`、`301/CO_PDO.*`、`301/CO_Emergency.*` 等。

   这一层实现 CiA 301 通信对象、NMT、Heartbeat、SDO、PDO、Emergency、SYNC 等协议逻辑，原则上不应针对 STM32H750 修改。

2. Object Dictionary

   Object Dictionary 是 CANopen 节点的对象表。它用 16-bit index + 8-bit subindex 描述节点参数、状态、通信配置和应用数据。

   CANopenNode 中典型文件是 `OD.c` / `OD.h`，通常由 EDS/XDD 工具或 CANopenEditor 生成。`OD_t` 是 OD 总表，`OD_entry_t` 是其中单个 index 的入口。

3. CAN driver / hardware abstraction

   这一层把 CANopenNode 的抽象接口接到具体 CAN 控制器。核心接口包括：

   - `CO_CANmodule_init()`
   - `CO_CANrxBufferInit()`
   - `CO_CANtxBufferInit()`
   - `CO_CANsend()`
   - `CO_CANmodule_process()`
   - CAN RX interrupt dispatch
   - CAN TX complete dispatch
   - CAN error/status update

   对 STM32H750 + FDCAN 来说，这层必须适配 `HAL_FDCAN_*`、FDCAN Message RAM、FDCAN RX FIFO、TX FIFO/Queue、Error Status 和当前工程已有的 Bus-Off Recovery。

4. Application layer

   应用层负责把真实业务状态映射到 OD 或 PDO。例如把当前 `CanStatistics_t` 中的 `node_state`、`bus_state`、TX/RX 计数等映射成 OD 变量。

   应用层还决定什么时候请求 TPDO 发送、如何处理 RPDO 写入，以及自定义业务 ID 是否继续保留或迁移到 PDO。

5. FreeRTOS task / timer integration

   CANopenNode 需要周期性调用：

   - `CO_process()`
   - `CO_process_SYNC()`
   - `CO_process_RPDO()`
   - `CO_process_TPDO()`

   CANopenNode_STM32 的示例使用一个高优先级 `canopen_task` 调用 `canopen_app_process()`，并用 1 ms timer interrupt 调用 `canopen_app_interrupt()` 处理 SYNC/RPDO/TPDO。

通用代码与必须适配代码的边界：

| 层 | 是否通用 | 当前工程需要做什么 |
| --- | --- | --- |
| CANopenNode core | 通用 | 作为第三方源码加入工程，不改协议栈 |
| OD.c / OD.h | 半通用 | 为 RTOS-CAN 生成/维护自己的 OD |
| CO_driver_target.h | 平台相关 | 需要适配 STM32H750、FreeRTOS 临界区、FDCAN HAL |
| CO_driver_STM32.c | 平台相关 | 可参考，但建议针对当前 FDCAN filter、Queue、Bus-Off Recovery 做定制 |
| CO_app_STM32.c/h | 示例应用层 | 可参考初始化顺序和 1 ms 处理模型，不建议原样接管当前工程 |
| can_app.c/h | 当前项目应用层 | 需要逐步让 CANopen 与现有业务并存，再迁移自定义 Heartbeat/业务帧 |

## 2. 核心 CANopen 对象

### 2.1 CO_t

`CO_t` 是 CANopenNode 的顶层对象，定义在 `CANopen.h`。它聚合整套 CANopen 对象：

- `CO_CANmodule_t *CANmodule`
- `CO_CANrx_t *CANrx`
- `CO_CANtx_t *CANtx`
- `CO_NMT_t *NMT`
- `CO_HBconsumer_t *HBcons`
- `CO_SDOserver_t *SDOserver`
- `CO_SDOclient_t *SDOclient`
- `CO_RPDO_t *RPDO`
- `CO_TPDO_t *TPDO`
- Emergency、SYNC、TIME、LED、LSS 等可选对象

初始化时通常先 `CO_new()` 分配/绑定对象，再 `CO_CANinit()` 初始化 CANmodule，之后 `CO_CANopenInit()` 初始化 NMT、Heartbeat、SDO 等，最后 `CO_CANopenInitPDO()` 初始化 PDO。

### 2.2 CO_CANmodule_t

`CO_CANmodule_t` 是 CAN 控制器抽象对象。它保存：

- CAN 控制器指针 `CANptr`
- RX 匹配表 `rxArray`
- TX 缓冲表 `txArray`
- CAN normal 状态
- TX pending 计数
- CAN error status
- 平台临界区/锁相关状态

在 STM32 port 中，`CANptr` 通常指向 `CANopenNodeSTM32`，其中再保存 `FDCAN_HandleTypeDef *CANHandle`。

### 2.3 CO_CANrx_t

`CO_CANrx_t` 是 CANopenNode 的软件接收过滤项。每个协议对象会通过 `CO_CANrxBufferInit()` 注册：

- CAN ID
- mask
- RTR 期望值
- object 指针
- RX callback

收到 CAN 帧后，driver 遍历 `rxArray`，找到匹配项后调用对应 CANopen 对象的回调。

### 2.4 CO_CANtx_t

`CO_CANtx_t` 是 CANopenNode 的发送缓冲项。每个 CANopen 发送对象通过 `CO_CANtxBufferInit()` 注册：

- CAN ID
- DLC
- data[8]
- bufferFull
- syncFlag

发送时协议对象填充 data，然后调用 `CO_CANsend()`。如果 FDCAN TX FIFO 有空位，driver 直接提交；否则标记 `bufferFull`，等 TX complete/empty 回调继续发送。

### 2.5 OD_t

`OD_t` 是 Object Dictionary 总表。CANopenNode 通过 `OD_find()`、`OD_getSub()`、`OD_get_value()`、`OD_set_value()` 等接口访问 OD。

OD 是 SDO/PDO 和应用数据之间的中心桥梁。

### 2.6 CO_NMT_t

NMT 和 Heartbeat producer 对象。它负责：

- NMT slave 状态机
- boot-up message
- producer heartbeat
- reset communication / reset node 请求

初始化来自 `CO_NMT_init()`，通常由 `CO_CANopenInit()` 内部调用。

### 2.7 CO_HBconsumer_t

Heartbeat consumer 对象。它根据 OD `0x1016 Consumer Heartbeat Time` 监控其他节点的 `0x700 + nodeId` heartbeat。

这会替代当前自定义 `0x701` heartbeat timeout 逻辑中的“节点在线监控”角色，但不替代底层 FDCAN bus state。

### 2.8 CO_SDOserver_t

SDO server 让外部 CANopen master/client 通过 `0x600 + nodeId` 请求访问本节点 OD，本节点通过 `0x580 + nodeId` 响应。

当前项目后续若要被 PC 工具或 ESP32 CANopen master 配置/读取，就需要 SDO server。

### 2.9 CO_SDOclient_t

SDO client 让本节点主动访问其他节点 OD。最小 STM32 CANopen slave 节点阶段可以先不启用 SDO client。

### 2.10 CO_RPDO_t / CO_TPDO_t

RPDO 是接收过程数据对象，TPDO 是发送过程数据对象。

- RPDO：CAN 帧到 OD/application 变量。
- TPDO：OD/application 变量到 CAN 帧。

PDO 初始化依赖 OD 中的通信参数和映射参数：

- RPDO communication：`0x1400...`
- RPDO mapping：`0x1600...`
- TPDO communication：`0x1800...`
- TPDO mapping：`0x1A00...`

### 2.11 初始化大致顺序

建议顺序：

1. CubeMX/HAL 完成基础时钟、GPIO、FDCAN init。
2. 创建 CANopen application object，例如 `CANopenNodeSTM32` 或本项目自定义 `CanOpenPort_t`。
3. `CO_new()`
4. `CO_CANsetConfigurationMode()`
5. `CO_CANinit()`
6. `CO_CANopenInit()`
7. `CO_CANopenInitPDO()`
8. 配置 1 ms process tick
9. `CO_CANsetNormalMode()`
10. 周期调用 `CO_process()`，1 ms 调用 SYNC/RPDO/TPDO process。

## 3. 当前 RTOS-CAN 与 CANopenNode 接口映射

### 3.1 当前工程中的 CAN 结构

当前 STM32 工程已经具备：

- `MX_FDCAN1_Init()`：CubeMX 生成 FDCAN1 初始化，Normal mode，500 kbit/s，Classic CAN。
- `CAN_App_Start()`：配置硬件 filter、global filter、interrupt line、notification、`HAL_FDCAN_Start()`。
- `HAL_FDCAN_RxFifo0Callback()`：读取 RX FIFO0，组装 `CanFrame_t`，投递 `canRxQueue`。
- `can_rx_task()`：从 Queue 取帧，处理 `0x321` 业务帧和 `0x701` heartbeat。
- `can_tx_task()`：周期发送 `0x120`。
- `CAN_App_UpdateBusStatus()`：周期读取 FDCAN protocol status / error counters，并驱动 bus_state。
- Level 2 Bus-Off Recovery：Stop -> DeInit -> RCC FDCAN reset -> MX_FDCAN1_Init -> CAN_App_Start。

### 3.2 CANopenNode_STM32 port 的实际做法

当前 CANopenNode_STM32 port 中：

- `CO_CANmodule_init()` 会调用 `HWInitFunction()`。
- FDCAN 下默认 `CANmodule->useCANrxFilters = false`。
- FDCAN Global Filter 默认接收所有 non-matching standard ID 到 RX FIFO0，extended reject，remote filter。
- `HAL_FDCAN_RxFifo0Callback()` 在 port 内部直接读取 FIFO0，然后遍历 `CO_CANrx_t` 软件匹配。
- `CO_CANsend()` 使用 `HAL_FDCAN_GetTxFifoFreeLevel()` 和 `HAL_FDCAN_AddMessageToTxFifoQ()`。
- `CO_CANmodule_process()` 读取 FDCAN `PSR` 中的 Bus-Off、Warning、Error Passive。
- `canopen_app_interrupt()` 以 1 ms 周期调用 `CO_process_SYNC()`、`CO_process_RPDO()`、`CO_process_TPDO()`。

### 3.3 最大接入冲突点

当前 RTOS-CAN 的 hardware filter 只允许：

- `0x321`
- `0x701`

而 CANopenNode 至少需要接收：

- NMT：`0x000`
- SDO request：`0x600 + nodeId`
- RPDO：`0x200 + nodeId`、`0x300 + nodeId`、...
- Heartbeat consumer：`0x700 + remoteNodeId`
- SYNC：`0x080`，如果启用
- LSS：`0x7E4/0x7E5`，如果启用

所以 CANopen 接入后必须重新设计硬件过滤策略。不能继续只放行 `0x321/0x701`，否则 SDO/NMT/PDO 不可能进入 CANopenNode。

### 3.4 HAL_FDCAN_RxFifo0Callback 的归属

不能让 `can_app.c` 和 CANopenNode_STM32 port 同时定义 `HAL_FDCAN_RxFifo0Callback()`，否则会出现重复符号或接收链路分叉。

推荐方向：

1. 当前阶段不要直接原样接入 CANopenNode_STM32 的 callback。
2. 后续实现本项目自己的 `canopen_port_fdcan.c`。
3. 统一保留一个 STM32 HAL FDCAN RX FIFO0 callback。
4. callback 内做轻量分发：
   - CANopen COB-ID -> CANopen RX dispatch
   - 当前 legacy ID `0x321/0x701` -> 现有 Queue 或迁移路径
5. 等 CANopen Heartbeat/PDO 完成后，再逐步移除 legacy 自定义 Heartbeat/业务帧。

### 3.5 Message Queue / can_rx_task 是否保留

短期建议保留。

- 当前 ISR -> Queue -> RX Task 已真机验证稳定。
- legacy `0x321` / `0x701` 仍可继续走 Queue。
- CANopen SDO/NMT/Heartbeat/PDO 不建议长期只靠 Queue 延后处理，尤其 SYNC/RPDO/TPDO 对时序更敏感。

建议分阶段：

| 当前模块 | CANopen 接入初期 | CANopen 稳定后 |
| --- | --- | --- |
| `HAL_FDCAN_RxFifo0Callback()` | 改成统一分发点 | 仍保留唯一入口 |
| `canRxQueue` | 保留 legacy 帧 | 可只保留非 CANopen 调试/兼容帧 |
| `can_rx_task()` | 继续处理 `0x321/0x701` | 逐步转为应用层/OD 同步任务 |
| `can_tx_task()` | 继续发送 `0x120`，但避开 CANopen ID | 迁移为 TPDO 或应用触发 |
| `CAN_App_UpdateBusStatus()` | 保留 | 保留，属于底层 bus 可靠性 |
| Bus-Off Recovery | 保留 | 必须保留，并让 CANopen port 感知恢复事件 |

### 3.6 can_tx_task 是否由 CANopen 接管

短期不接管，长期建议迁移。

- Phase 1-3：`can_tx_task` 可以继续发送 `0x120`，用于保持当前已验证业务。
- Phase 4-6：当 TPDO 建立后，`0x120` 业务应迁移为 TPDO 或重新分配为非冲突厂家自定义帧。
- 正式 CANopen 网络中，不建议长期保留无 OD/EDS 描述的裸 CAN 业务帧。

## 4. FreeRTOS 任务模型

### 4.1 是否需要独立 canopen_task

建议需要。

理由：

- CANopenNode 有自己的周期处理入口 `CO_process()`。
- NMT reset communication / reset node 需要统一处理。
- SDO server/client、Heartbeat consumer、Emergency、LSS 等都属于协议栈上下文。
- 让 CANopen 逻辑塞进现有 `can_rx_task()` 会让 legacy RX 和协议栈生命周期纠缠。

### 4.2 canopen_task 优先级

建议优先级高于普通业务 task，低于真正硬实时中断。

当前：

- `canRxTask`：`osPriorityAboveNormal`
- `canTxTask`：`osPriorityNormal`

建议初期：

- `canopen_task`：`osPriorityAboveNormal` 或略高于 legacy `canTxTask`
- 不建议高过所有系统关键任务太多，避免 SDO/日志处理影响系统调度。

如果 SYNC/PDO 有强实时要求，应由 1 ms timer 或高优先级周期机制驱动，不依赖普通 `canopen_task` 的阻塞延迟。

### 4.3 CO_process() 周期

`CO_process()` 建议在 `canopen_task` 中约 1 ms 调用一次，传入真实 elapsed time，单位为 us。

`CO_process()` 主要处理：

- NMT
- Heartbeat producer
- Heartbeat consumer
- SDO server/client
- Emergency
- LED/status
- 其他非硬实时协议对象

### 4.4 SYNC / RPDO / TPDO 放在哪里

推荐方案：

- 1 ms hardware timer 或高精度 RTOS timer 触发轻量 `canopen_1ms_process()`。
- 其中调用：
  - `CO_process_SYNC()`
  - `CO_process_RPDO()`
  - `CO_process_TPDO()`

不建议在 FDCAN RX ISR 里直接做大量 PDO 应用处理。RX ISR 只应做 CANopen RX dispatch 和必要标志置位。

### 4.5 为什么通常需要 1 ms 周期

CANopen 中多个时间相关对象以 ms 为典型粒度：

- Producer Heartbeat
- Consumer Heartbeat timeout
- SDO timeout
- TPDO event timer / inhibit time
- SYNC window
- NMT timing

1 ms 周期不是协议唯一允许值，但它是 CANopenNode_STM32 示例采用的稳妥基础粒度，便于把 us elapsed time 传给协议栈，同时满足大部分 heartbeat/PDO/SDO 超时处理。

### 4.6 Task / Software Timer / Hardware Timer 对比

| 方案 | 优点 | 缺点 | 建议用途 |
| --- | --- | --- | --- |
| FreeRTOS Task `osDelay(1)` | 简单，容易调试 | 受调度抖动影响 | `CO_process()`、SDO、NMT、Heartbeat |
| FreeRTOS Software Timer | 和 RTOS 集成好 | callback 不宜做重活，依赖 timer service task | 可触发轻量 flag，让 task 处理 |
| Hardware Timer interrupt | 抖动最小，接近 CANopenNode_STM32 示例 | ISR 中必须非常克制，注意锁和优先级 | SYNC/RPDO/TPDO 的 1 ms process |

本项目建议：

- Phase 1-3：先用 `canopen_task` 1 ms 循环跑通。
- Phase 4-6：如果使用 SYNC/PDO，再引入 1 ms hardware timer 或 RTOS timer + high priority task。

## 5. Object Dictionary 设计示例

Object Dictionary 是 CANopen 节点对外公开的参数和数据模型。外部工具通过 SDO 读写 OD，PDO 则把 OD 中的实时变量映射到 CAN 帧。

当前项目的最小厂家自定义 OD 示例：

| Index | Name | Type | Access | PDO mapping | 来源 |
| --- | --- | --- | --- | --- | --- |
| `0x2000` | STM32 status | UNSIGNED32 | ro | TPDO optional | `CanStatistics_t` / 应用状态汇总 |
| `0x2001` | ESP32 online | UNSIGNED8 | ro | TPDO optional | `esp32_online` 或 CANopen HB consumer 状态 |
| `0x2002` | CAN bus state | UNSIGNED8 | ro | TPDO optional | `bus_state` |
| `0x2003` | TX count | UNSIGNED32 | ro | TPDO optional | `tx_ok_count` |
| `0x2004` | RX count | UNSIGNED32 | ro | TPDO optional | `rx_task_count` 或 CANopen RX 计数 |

SDO 访问方式：

- PC 工具或 CANopen master 通过 `0x600 + nodeId` 发送 SDO request。
- STM32 SDO server 通过 `0x580 + nodeId` 返回 OD 值。
- 适合调试、配置、低频读取。

PDO 访问方式：

- 把 `0x2000...0x2004` 映射到 TPDO mapping object，例如 `0x1A00`。
- 配置 TPDO communication parameter，例如 `0x1800`。
- 运行时由 TPDO 周期、事件或 SYNC 触发发送。
- 适合实时状态上报。

注意：正式 OD 不能只写 C 变量表，还需要对应 EDS/XDD，方便 PC CANopen 工具识别。

## 6. COB-ID 冲突分析

当前项目使用：

- `0x120`：STM32 -> ESP32 业务帧
- `0x321`：ESP32 -> STM32 业务帧
- `0x701`：ESP32 Heartbeat，自定义 DLC=8

CANopen 默认 COB-ID：

| 对象 | COB-ID |
| --- | --- |
| NMT control | `0x000` |
| SYNC | `0x080` |
| EMCY | `0x080 + nodeId` |
| TIME | `0x100` |
| TPDO1 | `0x180 + nodeId` |
| RPDO1 | `0x200 + nodeId` |
| TPDO2 | `0x280 + nodeId` |
| RPDO2 | `0x300 + nodeId` |
| TPDO3 | `0x380 + nodeId` |
| RPDO3 | `0x400 + nodeId` |
| TPDO4 | `0x480 + nodeId` |
| RPDO4 | `0x500 + nodeId` |
| SDO response/server TX | `0x580 + nodeId` |
| SDO request/server RX | `0x600 + nodeId` |
| Heartbeat / boot-up | `0x700 + nodeId` |
| LSS slave/master | `0x7E4` / `0x7E5` |

### 6.1 `0x120`

`0x120` 不属于最常用 CiA 301 NMT/SDO/PDO/Heartbeat 默认 ID。

但它位于 `0x101...0x180` 区间附近，可能与 SRDO/safety-related data object 默认分配产生冲突。若本项目不启用 SRDO/CiA 304，短期冲突风险较低。

长期建议：不要把 `0x120` 作为裸业务帧长期保留在 CANopen 网络中，应迁移为 TPDO 或重新分配为明确的厂家自定义/非冲突 ID，并在 EDS 中描述。

### 6.2 `0x321`

`0x321` 明确落入 `RPDO2 = 0x300 + nodeId` 范围。

如果 STM32 CANopen nodeId 选择 `0x21`，则 `0x321` 正好是该节点默认 RPDO2。当前 ESP32 -> STM32 业务帧可以被设计成 RPDO2，但前提是 payload 和 OD mapping 按 CANopen PDO 规则定义。

如果不打算把它作为 RPDO2，则 `0x321` 与某个节点的 CANopen RPDO2 默认 COB-ID 冲突，不应继续作为裸业务帧。

### 6.3 `0x701`

`0x701` 明确是 CANopen heartbeat / boot-up for nodeId `1`。

当前自定义 heartbeat 使用 `0x701` 且 DLC=8；CANopen heartbeat 标准帧是 `0x700 + nodeId`，通常 DLC=1，data[0] 为 NMT state。

冲突结论：

- 如果 ESP32 未来作为 CANopen nodeId `1`，可以把当前 `0x701` 迁移为标准 CANopen Heartbeat，但 payload 必须改成 CANopen NMT state。
- 如果 STM32 自己 nodeId 选择 `1`，STM32 的 producer heartbeat 也会使用 `0x701`，会与当前 ESP32 自定义 heartbeat 冲突。
- 因此 CANopen 阶段必须规划 nodeId，并废止或迁移当前自定义 `0x701`。

### 6.4 Hardware Filter 影响

加入 CANopen 后，当前 filter `0x321/0x701 only` 不再满足需求。

后续可选策略：

1. 使用 CANopenNode_STM32 风格：接收所有 Standard ID 到 FIFO0，CANopen 软件过滤。
2. 使用多组 FDCAN hardware filters，只放行本节点 CANopen 所需 COB-ID 和保留 legacy ID。
3. 初期为了移植简单，接收所有 Standard ID；稳定后再收紧 hardware filter。

当前工程 FDCAN `StdFiltersNbr = 1`，只支持一个 standard filter。若要硬件层同时精确过滤 NMT、SDO、PDO、Heartbeat 等多个 ID，需要后续调整 FDCAN filter 数量和配置。这会涉及 CubeMX/HAL 初始化策略，不能在本轮修改。

## 7. 现有功能保留/替换关系

| 当前机制 | CANopen 后处理 | 原因 |
| --- | --- | --- |
| FDCAN bit timing / 500 kbit/s | 保留 | 底层物理通信已验证 |
| Hardware Filter `0x321/0x701` | 后续必须重新设计 | CANopen 需要更多 COB-ID |
| ISR -> Queue -> RX Task | 短期保留 legacy，CANopen 需增加直接 dispatch | CANopen SDO/PDO/NMT 需要协议栈接收分发 |
| `can_tx_task` 发送 `0x120` | 短期保留，长期迁移 TPDO | 裸 CAN 不适合长期混在 CANopen 网络 |
| 自定义 `0x701` Heartbeat | 长期替换为 CANopen Heartbeat/HBconsumer | 两套 Heartbeat 不能长期并存 |
| `esp32_online` / Node State | 迁移为 CANopen HBconsumer 派生 | CANopen NMT/Heartbeat 已定义节点在线语义 |
| CAN Bus State | 保留 | 它描述本地 FDCAN 控制器/bus 错误，不等同于 CANopen node state |
| Bus-Off fault injection | 保留为诊断功能，默认关闭 | 用于验证 driver/recovery |
| FDCAN peripheral-level Bus-Off Recovery | 必须保留 | CANopenNode_STM32 默认错误处理不足以替代当前已验证恢复 |
| `CanStatistics_t` | 保留并扩展 CANopen 统计 | 调试统一出口 |

### CANopen Heartbeat 与当前自定义 Heartbeat 的区别

当前自定义 Heartbeat：

- CAN ID：`0x701`
- DLC：8
- 用途：STM32 判断 ESP32 是否在线
- 逻辑：自定义 timeout + node_state

CANopen Heartbeat：

- CAN ID：`0x700 + nodeId`
- DLC：1
- data[0]：NMT state
- 配置入口：OD `0x1017 Producer Heartbeat Time`、`0x1016 Consumer Heartbeat Time`
- 逻辑：由 `CO_NMT_t` 和 `CO_HBconsumer_t` 处理

长期方向：用 CANopen Heartbeat 替代自定义 `0x701`，避免同一 ID 不同 payload 语义长期并存。

## 8. 分阶段移植计划

### Phase 1：CANopenNode 源码加入工程并成功编译

要改什么：

- 添加 CANopenNode core 源码到工程目录。
- 添加 STM32 port 源码或本项目 port 骨架。
- 添加最小 `OD.c` / `OD.h`。
- 配置 Keil include path/source group。
- 暂不接管 FDCAN callback，不改现有 CAN 行为。

怎么验证：

- Keil/ARM GCC 能编译 CANopenNode 源码。
- 不生成重复 `HAL_FDCAN_*Callback` 符号。
- 现有 STM32/ESP32 通信行为不变。

完成标准：

- 工程编译通过。
- `BUS_OFF_TEST=0`，现有 `0x120/0x321/0x701` 真机行为不变。

### Phase 2：完成 STM32H750 FDCAN Port

要改什么：

- 实现或定制 `canopen_port_fdcan.c/h`。
- 明确 `CO_CANmodule_init()` 不破坏当前 FDCAN bit timing。
- 明确 RX dispatch 与现有 `HAL_FDCAN_RxFifo0Callback()` 的唯一入口关系。
- 明确 `CO_CANsend()` 与 bus_state / Bus-Off Recovery 的关系。

怎么验证：

- CANopen RX software filter 可以收到 NMT/SDO/PDO 目标 ID。
- CO_CANsend 能通过 FDCAN TX FIFO 发送 boot-up 或测试帧。
- Bus-Off 时 CANopen 不继续提交正常 TX。

完成标准：

- 无重复 HAL callback。
- FDCAN reset recovery 后 CANopen notification/filter/dispatch 可恢复。

### Phase 3：CANopen 节点完成初始化

要改什么：

- 新增 `canopen_task` 设计实现。
- 初始化 `CO_t`、`CO_CANmodule_t`、`OD_t`。
- 调用 `CO_CANinit()`、`CO_CANopenInit()`、`CO_CANopenInitPDO()`。

怎么验证：

- 节点启动后能进入 Pre-operational 或 Operational。
- 能发出 boot-up message。
- 无 hard fault、无 queue drop、无 Bus-Off。

完成标准：

- CANopen nodeId 固定且可观测。
- CO_process 周期运行稳定。

### Phase 4：NMT + Producer Heartbeat

要改什么：

- 启用 NMT slave。
- 配置 OD `0x1017 Producer Heartbeat Time`。
- 发出 `0x700 + nodeId` heartbeat。

怎么验证：

- PC CANopen 工具能看到 boot-up 和 heartbeat。
- NMT start/stop/pre-operational 命令能改变节点状态。

完成标准：

- CANopen heartbeat 稳定。
- 当前自定义 heartbeat 替换策略明确，但不必在本阶段全部删除。

### Phase 5：Object Dictionary + SDO

要改什么：

- 建立 RTOS-CAN 最小 OD。
- 映射 `0x2000...0x2004` 示例对象。
- 启用 SDO server。

怎么验证：

- PC CANopen 工具通过 SDO 读取 `0x2000...0x2004`。
- 写只读对象会返回正确 SDO abort。
- OD 数据与 `CanStatistics_t` 一致。

完成标准：

- EDS/OD 与固件一致。
- SDO server 稳定无超时。

### Phase 6：TPDO / RPDO

要改什么：

- 设计 TPDO/RPDO mapping。
- 决定是否把 `0x120` 迁移为 TPDO。
- 决定是否把 `0x321` 迁移为 RPDO2 或其他 PDO。

怎么验证：

- TPDO 触发周期/事件正确。
- RPDO 写入后 OD/application 变量更新正确。
- SYNC 模式下时序符合预期。

完成标准：

- 裸业务帧与 CANopen PDO 不再冲突。
- PC 工具可解析 PDO mapping。

### Phase 7：与 ESP32/PC 工具联调

要改什么：

- 决定 ESP32 是继续裸 CAN 节点，还是迁移为 CANopen node。
- 用 PC CANopen 工具验证 NMT、SDO、PDO、Heartbeat。
- 若 ESP32 迁移 CANopen，替换自定义 `0x701` heartbeat。

怎么验证：

- STM32 与 PC 工具互通。
- STM32 与 ESP32 互通。
- Bus-Off fault injection 后恢复，CANopen communication 也恢复。

完成标准：

- CANopen 状态机、OD、SDO、PDO、Heartbeat 全链路真机通过。
- legacy ID 冲突已处理或明确废止。

## 9. 第一阶段实际要修改的文件列表

Phase 1 预计只做“加入源码并编译通过”，不改变现有 CAN 行为。

建议新增：

- `Middlewares/Third_Party/CANopenNode/`
- `Middlewares/Third_Party/CANopenNode_STM32/` 或 `Core/CANopen/Port/`
- `Core/CANopen/OD.c`
- `Core/CANopen/OD.h`
- `Core/CANopen/canopen_port_config.h` 或等价配置头
- `Core/CANopen/canopen_app_stub.c/h`，仅用于编译期初始化骨架

建议修改：

- `MDK-ARM/FreeRTOS+FDCAN.uvprojx`：加入 CANopenNode source/include path。
- `MDK-ARM/FreeRTOS+FDCAN.uvoptx`：如 Keil 自动更新，可接受但需审查。

Phase 1 暂不修改：

- `Core/Src/can_app.c`
- `Core/Inc/can_app.h`
- `Core/Src/freertos.c`
- `Core/Src/fdcan.c`
- `FreeRTOS+FDCAN.ioc`
- ESP32 工程

如果 Phase 1 编译因 callback 重复或 driver target 配置无法隔离，则下一轮应先只加源码不启用 `CO_driver_STM32.c` 的 HAL callback，改用本项目专属 port 骨架。

## 10. 当前结论

1. CANopenNode core 可以作为第三方协议栈加入，不应修改其通用协议代码。
2. 当前工程最需要定制的是 STM32H750 FDCAN port，而不是 CANopen core。
3. 现有 `0x321/0x701 only` hardware filter 与 CANopen 接收需求冲突，后续必须重新设计。
4. `HAL_FDCAN_RxFifo0Callback()` 必须保持唯一入口，不能和 CANopenNode_STM32 port 原样 callback 并存。
5. 当前 Bus State 和 Bus-Off Recovery 是底层 FDCAN 可靠性机制，CANopen 不能替代，必须保留。
6. 当前自定义 `0x701` heartbeat 与 CANopen heartbeat ID 直接重叠，长期必须迁移或替换。
7. `0x321` 与 CANopen RPDO2 默认范围冲突，若选择 nodeId `0x21` 可设计性迁移为 RPDO2，否则需要换 ID 或禁用对应默认 PDO。
8. 最小安全路线是先编译 CANopenNode，再做本项目专属 FDCAN port，最后逐步启用 NMT、Heartbeat、SDO、PDO。
