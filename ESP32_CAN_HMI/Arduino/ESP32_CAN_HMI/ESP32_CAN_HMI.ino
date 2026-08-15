/*
 * ESP32-S3 CAN HMI companion demo
 *
 * Board: Waveshare ESP32-S3-Touch-LCD-2.8
 * Purpose:
 *   - Receive CAN frames through ESP32 TWAI when a CAN transceiver is connected.
 *   - Run without CAN hardware in demo mode so the HMI/data logic can be tested first.
 *
 * Hardware notes:
 *   - This 2.8-inch board does not expose an onboard CAN connector.
 *   - ESP32-S3 has a TWAI controller, but it still needs an external CAN transceiver.
 *   - Suggested pins from the board's spare 12-pin interface:
 *       GPIO18 -> CAN transceiver TXD
 *       GPIO15 <- CAN transceiver RXD
 */

#include <Arduino.h>
#include "driver/twai.h"

#define USE_REAL_CAN 0

static constexpr gpio_num_t CAN_TX_GPIO = GPIO_NUM_18;
static constexpr gpio_num_t CAN_RX_GPIO = GPIO_NUM_15;
static constexpr uint32_t CAN_BITRATE = 500000;

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} CanFrame;

typedef struct {
  uint32_t tx_count;
  uint32_t rx_count;
  uint32_t err_count;
  uint32_t last_id;
  uint8_t last_counter;
  uint32_t last_update_ms;
  bool bus_ok;
} CanDashboardState;

static CanDashboardState dashboard = {0};
static uint32_t last_print_ms = 0;

static void display_init(void);
static void display_update(const CanDashboardState *state);
static bool can_init(void);
static bool can_receive(CanFrame *frame);
static void handle_frame(const CanFrame *frame);
static void make_demo_frame(CanFrame *frame);

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("ESP32-S3 CAN HMI starting...");
  Serial.printf("Mode: %s\r\n", USE_REAL_CAN ? "TWAI/CAN" : "demo frames");

  display_init();
  dashboard.bus_ok = can_init();
}

void loop() {
  CanFrame frame = {0};

  if (USE_REAL_CAN) {
    if (can_receive(&frame)) {
      handle_frame(&frame);
    }
  } else {
    static uint32_t last_demo_ms = 0;
    if (millis() - last_demo_ms >= 1000) {
      last_demo_ms = millis();
      make_demo_frame(&frame);
      handle_frame(&frame);
    }
  }

  if (millis() - last_print_ms >= 500) {
    last_print_ms = millis();
    display_update(&dashboard);
  }

  delay(10);
}

static void display_init(void) {
  /*
   * Keep this independent from a specific LCD library for now.
   * Later we will replace Serial output with the Waveshare/LVGL screen port.
   */
  Serial.println("Display adapter: Serial console for now.");
}

static void display_update(const CanDashboardState *state) {
  Serial.printf(
      "bus=%s rx=%lu tx=%lu err=%lu last_id=0x%03lX counter=%u age=%lums\r\n",
      state->bus_ok ? "OK" : "WAIT",
      static_cast<unsigned long>(state->rx_count),
      static_cast<unsigned long>(state->tx_count),
      static_cast<unsigned long>(state->err_count),
      static_cast<unsigned long>(state->last_id),
      state->last_counter,
      static_cast<unsigned long>(millis() - state->last_update_ms));
}

static bool can_init(void) {
  if (!USE_REAL_CAN) {
    return true;
  }

  twai_general_config_t general_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&general_config, &timing_config, &filter_config);
  if (err != ESP_OK) {
    Serial.printf("twai_driver_install failed: %d\r\n", err);
    dashboard.err_count++;
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("twai_start failed: %d\r\n", err);
    dashboard.err_count++;
    return false;
  }

  Serial.printf("TWAI started at %lu bps, TX=%d RX=%d\r\n",
                static_cast<unsigned long>(CAN_BITRATE),
                static_cast<int>(CAN_TX_GPIO),
                static_cast<int>(CAN_RX_GPIO));
  return true;
}

static bool can_receive(CanFrame *frame) {
  twai_message_t message = {0};
  esp_err_t err = twai_receive(&message, pdMS_TO_TICKS(20));

  if (err == ESP_ERR_TIMEOUT) {
    return false;
  }

  if (err != ESP_OK) {
    dashboard.err_count++;
    return false;
  }

  if (message.rtr) {
    return false;
  }

  frame->id = message.identifier;
  frame->dlc = message.data_length_code;
  for (uint8_t i = 0; i < frame->dlc && i < sizeof(frame->data); i++) {
    frame->data[i] = message.data[i];
  }

  return true;
}

static void handle_frame(const CanFrame *frame) {
  dashboard.rx_count++;
  dashboard.last_id = frame->id;
  dashboard.last_counter = frame->dlc > 0 ? frame->data[0] : 0;
  dashboard.last_update_ms = millis();
}

static void make_demo_frame(CanFrame *frame) {
  static uint8_t counter = 0;

  frame->id = 0x123;
  frame->dlc = 8;
  frame->data[0] = counter++;
  frame->data[1] = 0x11;
  frame->data[2] = 0x22;
  frame->data[3] = 0x33;
  frame->data[4] = 0x44;
  frame->data[5] = 0x55;
  frame->data[6] = 0x66;
  frame->data[7] = 0x77;
}
