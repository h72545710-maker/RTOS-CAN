# ESP32-S3 CAN Wiring Notes

## Why A Transceiver Is Needed

ESP32-S3 provides a TWAI controller, which is compatible with classic CAN controller logic, but it cannot connect directly to CANH/CANL.

You need a 3.3 V CAN transceiver module, for example an SN65HVD230-style module.

## Suggested Pins

The Waveshare ESP32-S3-Touch-LCD-2.8 12-pin interface exposes spare IO pins:

- `IO18` -> use as ESP32 TWAI TX
- `IO15` -> use as ESP32 TWAI RX

Do not use the board's USB differential pins `D+`/`D-` GPIO20/GPIO19 for this first version.

## Connection

ESP32-S3 board to CAN transceiver:

```text
ESP32 3V3  -> Transceiver VCC
ESP32 GND  -> Transceiver GND
ESP32 IO18 -> Transceiver TXD
ESP32 IO15 <- Transceiver RXD
```

CAN transceiver to bus:

```text
Transceiver CANH -> CAN_H on USB-CAN analyzer / STM32 transceiver
Transceiver CANL -> CAN_L on USB-CAN analyzer / STM32 transceiver
GND common       -> Connect grounds together
```

## Termination

For a short lab setup, use 120 ohm termination at both ends of the CAN bus.

If there are only two CAN nodes, that usually means:

- one 120 ohm resistor at the USB-CAN/analyzer side, if its switch is enabled
- one 120 ohm resistor at the other end, usually the transceiver side

If communication is unstable, measure between CANH and CANL with power off. A correctly terminated two-end bus is usually around 60 ohm.

## Matching The STM32 Demo

Set both sides to classic CAN at 500 kbps.

STM32 sends:

```text
ID  : 0x123
DLC : 8
DATA: counter 11 22 33 44 55 66 77
```

The ESP32 sketch expects this frame and displays the first byte as the changing counter.
