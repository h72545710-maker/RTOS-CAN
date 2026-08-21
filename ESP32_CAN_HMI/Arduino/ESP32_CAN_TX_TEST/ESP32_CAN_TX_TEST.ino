#include <Arduino.h>
#include "driver/twai.h"

#define BUS_OFF_TEST 0

static constexpr gpio_num_t CAN_TX_GPIO = GPIO_NUM_18;
static constexpr gpio_num_t CAN_RX_GPIO = GPIO_NUM_15;
static constexpr uint32_t CAN_TX_ID = 0x321;
static constexpr uint32_t STM32_TX_ID = 0x120;

static constexpr uint32_t TX_PERIOD_MS = 1000;
static constexpr uint32_t STATUS_PERIOD_MS = 500;
static constexpr uint32_t LOOP_DELAY_MS = 5;
static constexpr uint32_t CAN_BITRATE = 500000;

static uint32_t tx_ok_count = 0;
static uint32_t tx_err_count = 0;
static uint32_t rx_ok_count = 0;
static uint32_t rx_err_count = 0;
static uint32_t rx_stm32_count = 0;
static uint32_t last_rx_id = 0;
static uint8_t last_rx_dlc = 0;
static uint8_t last_rx_data[8] = {0};
static uint8_t counter = 0;

static bool can_start(void) {
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("twai_driver_install failed: %d\r\n", err);
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("twai_start failed: %d\r\n", err);
    return false;
  }

  Serial.printf("TWAI started: %lu bit/s, TX GPIO=%d, RX GPIO=%d, BUS_OFF_TEST=%d\r\n",
                (unsigned long)CAN_BITRATE,
                (int)CAN_TX_GPIO,
                (int)CAN_RX_GPIO,
                BUS_OFF_TEST);
  return true;
}

static void send_demo_frame(void) {
  twai_message_t msg = {};
  msg.identifier = CAN_TX_ID;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;
  msg.data[0] = counter++;
  msg.data[1] = 0x11;
  msg.data[2] = 0x22;
  msg.data[3] = 0x33;
  msg.data[4] = 0x44;
  msg.data[5] = 0x55;
  msg.data[6] = 0x66;
  msg.data[7] = 0x77;

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
  if (err == ESP_OK) {
    tx_ok_count++;
  } else {
    tx_err_count++;
  }

}

static void receive_pending_frames(void) {
  twai_message_t msg = {};

  while (twai_receive(&msg, 0) == ESP_OK) {
    last_rx_id = msg.identifier;
    last_rx_dlc = msg.data_length_code;
    for (uint8_t i = 0; i < sizeof(last_rx_data); i++) {
      last_rx_data[i] = (i < msg.data_length_code) ? msg.data[i] : 0;
    }

    rx_ok_count++;
    if (!msg.extd && !msg.rtr && msg.identifier == STM32_TX_ID) {
      rx_stm32_count++;
    }

    Serial.printf("RX count=%lu stm32=%lu id=0x%03lX dlc=%u data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                  (unsigned long)rx_ok_count,
                  (unsigned long)rx_stm32_count,
                  (unsigned long)last_rx_id,
                  (unsigned int)last_rx_dlc,
                  last_rx_data[0],
                  last_rx_data[1],
                  last_rx_data[2],
                  last_rx_data[3],
                  last_rx_data[4],
                  last_rx_data[5],
                  last_rx_data[6],
                  last_rx_data[7]);
  }
}

static void print_status(void) {
  twai_status_info_t status = {};
  esp_err_t err = twai_get_status_info(&status);
  if (err != ESP_OK) {
    rx_err_count++;
    return;
  }

  Serial.printf("STAT tx=%lu tx_err=%lu rx=%lu stm32=%lu state=%d tx_failed=%lu rx_missed=%lu arb_lost=%lu bus_err=%lu last_id=0x%03lX data0=0x%02X\r\n",
                (unsigned long)tx_ok_count,
                (unsigned long)tx_err_count,
                (unsigned long)rx_ok_count,
                (unsigned long)rx_stm32_count,
                (int)status.state,
                (unsigned long)status.tx_failed_count,
                (unsigned long)status.rx_missed_count,
                (unsigned long)status.arb_lost_count,
                (unsigned long)status.bus_error_count,
                (unsigned long)last_rx_id,
                last_rx_data[0]);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("ESP32 CAN TX/RX TEST");
  if (!can_start()) {
    Serial.println("CAN start failed");
  }
}

void loop() {
  static uint32_t last_tx_ms = 0;
  static uint32_t last_status_ms = 0;
  uint32_t now = millis();

  receive_pending_frames();

  if ((now - last_tx_ms) >= TX_PERIOD_MS) {
    last_tx_ms = now;
    send_demo_frame();
  }

  if ((now - last_status_ms) >= STATUS_PERIOD_MS) {
    last_status_ms = now;
    print_status();
  }

  delay(LOOP_DELAY_MS);
}
