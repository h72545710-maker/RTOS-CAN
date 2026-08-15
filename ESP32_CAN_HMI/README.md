# ESP32 CAN HMI

This folder is the ESP32-S3 companion side for the STM32H750 FreeRTOS + FDCAN demo.

Current target:

- Run a small CAN dashboard data model on the ESP32-S3.
- Support a no-hardware demo mode first.
- Switch to ESP32 TWAI/CAN after an external CAN transceiver is connected.
- Keep the display adapter isolated so the Waveshare LVGL/LCD code can be added later without changing the CAN logic.

## Board

Waveshare ESP32-S3-Touch-LCD-2.8.

According to Waveshare's documentation, this 2.8-inch board exposes UART, I2C, USB, IO15, and IO18, but it does not provide an onboard CAN connector/transceiver. ESP32-S3 has a TWAI controller, so CAN still needs an external transceiver module.

## Files

- `Arduino/ESP32_CAN_HMI/ESP32_CAN_HMI.ino`: Arduino sketch with demo mode and TWAI receive mode.
- `docs/wiring.md`: wiring notes for the future CAN transceiver connection.

## First Run Without CAN Hardware

Keep this macro as `0`:

```c
#define USE_REAL_CAN 0
```

Upload the sketch and open Serial Monitor at `115200`. You should see a simulated frame every second:

```text
bus=OK rx=1 tx=0 err=0 last_id=0x123 counter=0 age=...
```

## Later With CAN Hardware

After the CAN transceiver arrives, change:

```c
#define USE_REAL_CAN 1
```

Then connect the transceiver as described in `docs/wiring.md`.

The STM32 side currently sends:

- Standard ID: `0x123`
- DLC: `8`
- Data: `counter, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77`

The ESP32 side stores the first byte as `last_counter`.
