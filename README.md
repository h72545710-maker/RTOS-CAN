# STM32H750 FreeRTOS FDCAN Demo

This repository contains a CubeMX/Keil project for STM32H750VBT6.

## Current Milestone

- FreeRTOS tasks are running.
- FDCAN1 internal loopback is enabled.
- `can_tx_task` sends Classic CAN frames with standard ID `0x123`.
- FDCAN RX FIFO0 interrupt receives loopback frames.
- `HAL_FDCAN_RxFifo0Callback` forwards received frames into a CMSIS-RTOS message queue.
- `can_rx_task` consumes frames from the queue.

The verified path is:

```text
can_tx_task
  -> HAL_FDCAN_AddMessageToTxFifoQ
  -> FDCAN internal loopback
  -> HAL_FDCAN_RxFifo0Callback
  -> osMessageQueuePut
  -> can_rx_task
```

## Hardware

- MCU board: STM32H750VBT6
- Current test: no external CAN transceiver required
- Next stage: external CAN transceiver plus USB-CAN analyzer

## Next Stage

After the CAN transceiver and USB-CAN analyzer arrive:

1. Change FDCAN mode from `FDCAN_MODE_INTERNAL_LOOPBACK` to `FDCAN_MODE_NORMAL`.
2. Connect `PA12/FDCAN_TX` to transceiver `TXD`.
3. Connect `PA11/FDCAN_RX` to transceiver `RXD`.
4. Connect `CANH/CANL` to the USB-CAN analyzer.
5. Set both sides to the same bitrate and verify frames with ID `0x123`.

