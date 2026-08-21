/*
 * ESP32-S3 CAN HMI LCD demo
 *
 * Board: Waveshare ESP32-S3-Touch-LCD-2.8
 * Screen: ST7789 240x320 with LVGL
 *
 * This sketch uses demo frames by default. Change USE_REAL_CAN to 1 after
 * connecting an external 3.3 V CAN transceiver to GPIO18/GPIO15.
 */

#include <Arduino.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_rom_sys.h"
#include "Display_ST7789.h"
#include "LVGL_Driver.h"

#define USE_REAL_CAN 1
#define BUS_OFF_TEST 0

static constexpr gpio_num_t CAN_TX_GPIO = GPIO_NUM_18;
static constexpr gpio_num_t CAN_RX_GPIO = GPIO_NUM_15;
static constexpr uint32_t CAN_STM32_TX_ID = 0x120;
static constexpr uint32_t CAN_ESP32_TX_ID = 0x321;
static constexpr uint32_t CAN_ESP32_HEARTBEAT_ID = 0x701;
static constexpr uint32_t CAN_TX_PERIOD_MS = 1000;
static constexpr uint32_t CAN_HEARTBEAT_PERIOD_MS = 500;
#if BUS_OFF_TEST
static constexpr uint32_t FAULT_PULSE_LOW_MIN_US = 4;
static constexpr uint32_t FAULT_PULSE_LOW_MAX_US = 8;
static constexpr uint32_t FAULT_PULSE_GAP_MIN_US = 200;
static constexpr uint32_t FAULT_PULSE_GAP_MAX_US = 700;
static constexpr uint32_t FAULT_OFF_SETTLE_MS = 20;
#endif

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
} CanFrame;

typedef struct {
  uint32_t rx_count;
  uint32_t stm32_rx_count;
  uint32_t other_rx_count;
  uint32_t tx_count;
  uint32_t tx_fail_count;
  uint32_t heartbeat_tx_count;
  uint32_t heartbeat_tx_fail_count;
  uint32_t err_count;
  uint32_t last_id;
  uint8_t last_counter;
  uint8_t last_data[8];
  uint32_t last_update_ms;
  uint32_t last_tx_ms;
  uint8_t last_tx_counter;
  uint32_t last_heartbeat_ms;
  uint8_t last_heartbeat_counter;
  bool bus_ok;
#if BUS_OFF_TEST
  uint32_t fault_on_count;
  uint32_t fault_off_count;
  uint32_t fault_pulse_count;
  uint32_t current_low_us;
  uint32_t current_gap_us;
#endif
} CanDashboardState;

static CanDashboardState dashboard = {0};
static bool twai_running = false;
#if BUS_OFF_TEST
static bool fault_injection_active = false;
static uint32_t last_fault_pulse_us = 0;
static uint32_t fault_next_gap_us = FAULT_PULSE_GAP_MIN_US;
static uint32_t fault_prng_state = 0xA5C3321U;
static char serial_command[32] = {0};
static uint8_t serial_command_len = 0;
#endif

static lv_obj_t *bus_value;
static lv_obj_t *mode_value;
static lv_obj_t *rx_value;
static lv_obj_t *tx_value;
static lv_obj_t *err_value;
static lv_obj_t *id_value;
static lv_obj_t *counter_value;
static lv_obj_t *data_value;
static lv_obj_t *age_value;

static bool can_init(void);
static void can_stop(void);
static bool can_receive(CanFrame *frame);
static bool can_send_demo(void);
static bool can_send_heartbeat(void);
static void make_demo_frame(CanFrame *frame);
static void handle_frame(const CanFrame *frame);
#if BUS_OFF_TEST
static void serial_service(void);
static void process_serial_command(const char *command);
static bool fault_injection_start(void);
static bool fault_injection_stop(void);
static void fault_injection_service(void);
static uint32_t fault_random_range(uint32_t min_value, uint32_t max_value);
#endif
static void ui_create(void);
static void ui_update(void);
static lv_obj_t *ui_make_value(lv_obj_t *parent, const char *name, int y);

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("ESP32-S3 CAN HMI LCD starting...");

  Backlight_Init();
  LCD_Init();
  Lvgl_Init();
  ui_create();

  dashboard.bus_ok = can_init();
}

void loop() {
  CanFrame frame = {0};

#if BUS_OFF_TEST
  serial_service();
  fault_injection_service();

  if (fault_injection_active) {
    static uint32_t last_fault_ui_ms = 0;
    uint32_t now_ms = millis();

    if (now_ms - last_fault_ui_ms >= 20) {
      last_fault_ui_ms = now_ms;
      ui_update();
      Lvgl_Loop();
    }
    return;
  }
#endif

  if (USE_REAL_CAN) {
    if (can_receive(&frame)) {
      handle_frame(&frame);
    }

    static uint32_t last_tx_ms = 0;
    if (millis() - last_tx_ms >= CAN_TX_PERIOD_MS) {
      last_tx_ms = millis();
      can_send_demo();
    }

    static uint32_t last_heartbeat_ms = 0;
    if (millis() - last_heartbeat_ms >= CAN_HEARTBEAT_PERIOD_MS) {
      last_heartbeat_ms = millis();
      can_send_heartbeat();
    }
  } else {
    static uint32_t last_demo_ms = 0;
    if (millis() - last_demo_ms >= 1000) {
      last_demo_ms = millis();
      make_demo_frame(&frame);
      handle_frame(&frame);
    }
  }

  static uint32_t last_ui_ms = 0;
  if (millis() - last_ui_ms >= 200) {
    last_ui_ms = millis();
    ui_update();
  }

  Lvgl_Loop();
  delay(
#if BUS_OFF_TEST
      fault_injection_active ? 1 :
#endif
      5);
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
    dashboard.err_count++;
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    dashboard.err_count++;
    (void)twai_driver_uninstall();
    return false;
  }

  twai_running = true;
  Serial.printf("TWAI started: 500 kbit/s, TX GPIO=%d, RX GPIO=%d, BUS_OFF_TEST=%d\r\n",
                static_cast<int>(CAN_TX_GPIO),
                static_cast<int>(CAN_RX_GPIO),
                BUS_OFF_TEST);
#if BUS_OFF_TEST
  Serial.println("Commands: FAULT ON, FAULT OFF, FAULT STATUS");
#endif
  return true;
}

static void can_stop(void) {
  if (!twai_running) {
    return;
  }

  esp_err_t stop_err = twai_stop();
  if (stop_err != ESP_OK && stop_err != ESP_ERR_INVALID_STATE) {
    dashboard.err_count++;
    Serial.printf("twai_stop failed: %d\r\n", stop_err);
  }

  esp_err_t uninstall_err = twai_driver_uninstall();
  if (uninstall_err != ESP_OK && uninstall_err != ESP_ERR_INVALID_STATE) {
    dashboard.err_count++;
    Serial.printf("twai_driver_uninstall failed: %d\r\n", uninstall_err);
  }

  twai_running = false;
}

static bool can_receive(CanFrame *frame) {
  if (!twai_running) {
    return false;
  }

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

static bool can_send_demo(void) {
  if (!twai_running) {
    return false;
  }

  static uint8_t counter = 0;
  twai_message_t message = {0};

  message.identifier = CAN_ESP32_TX_ID;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 8;
  message.data[0] = counter++;
  message.data[1] = 0xA1;
  message.data[2] = 0xA2;
  message.data[3] = 0xA3;
  message.data[4] = 0xA4;
  message.data[5] = 0xA5;
  message.data[6] = 0xA6;
  message.data[7] = 0xA7;

  esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(10));
  if (err == ESP_OK) {
    dashboard.tx_count++;
    dashboard.last_tx_ms = millis();
    dashboard.last_tx_counter = message.data[0];
    return true;
  }

  dashboard.tx_fail_count++;
  dashboard.err_count++;
  return false;
}

static bool can_send_heartbeat(void) {
  if (!twai_running) {
    return false;
  }

  static uint8_t counter = 0;
  twai_message_t message = {0};

  message.identifier = CAN_ESP32_HEARTBEAT_ID;
  message.extd = 0;
  message.rtr = 0;
  message.data_length_code = 1;
  message.data[0] = counter++;

  esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(10));
  if (err == ESP_OK) {
    dashboard.heartbeat_tx_count++;
    dashboard.last_heartbeat_ms = millis();
    dashboard.last_heartbeat_counter = message.data[0];
    return true;
  }

  dashboard.heartbeat_tx_fail_count++;
  dashboard.err_count++;
  return false;
}

#if BUS_OFF_TEST
static void serial_service(void) {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\r' || ch == '\n') {
      if (serial_command_len > 0) {
        serial_command[serial_command_len] = '\0';
        process_serial_command(serial_command);
        serial_command_len = 0;
      }
      continue;
    }

    if (serial_command_len < (sizeof(serial_command) - 1U)) {
      serial_command[serial_command_len++] = ch;
    }
  }
}

static void process_serial_command(const char *command) {
  if (strcmp(command, "FAULT ON") == 0) {
    if (!fault_injection_start()) {
      Serial.println("FAULT ON failed");
    }
    return;
  }

  if (strcmp(command, "FAULT OFF") == 0) {
    if (!fault_injection_stop()) {
      Serial.println("FAULT OFF failed");
    }
    return;
  }

  if (strcmp(command, "FAULT STATUS") == 0) {
    Serial.printf("FAULT active=%d twai_running=%d pulses=%lu low=%luus gap=%luus on=%lu off=%lu\r\n",
                  fault_injection_active,
                  twai_running,
                  static_cast<unsigned long>(dashboard.fault_pulse_count),
                  static_cast<unsigned long>(dashboard.current_low_us),
                  static_cast<unsigned long>(dashboard.current_gap_us),
                  static_cast<unsigned long>(dashboard.fault_on_count),
                  static_cast<unsigned long>(dashboard.fault_off_count));
    return;
  }

  Serial.printf("Unknown command: %s\r\n", command);
}

static bool fault_injection_start(void) {
  if (!USE_REAL_CAN) {
    Serial.println("FAULT ON ignored: USE_REAL_CAN is 0");
    return false;
  }

  if (fault_injection_active) {
    Serial.println("FAULT already active");
    return true;
  }

  can_stop();

  gpio_set_level(CAN_TX_GPIO, 1);
  gpio_set_direction(CAN_TX_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(CAN_TX_GPIO, GPIO_PULLUP_ONLY);
  gpio_set_level(CAN_TX_GPIO, 1);

  last_fault_pulse_us = micros();
  fault_next_gap_us = fault_random_range(FAULT_PULSE_GAP_MIN_US, FAULT_PULSE_GAP_MAX_US);
  dashboard.current_low_us = FAULT_PULSE_LOW_MIN_US;
  dashboard.current_gap_us = fault_next_gap_us;
  fault_injection_active = true;
  dashboard.bus_ok = false;
  dashboard.fault_on_count++;

  Serial.printf("FAULT ON: TXD GPIO%d LOW=%lu..%luus gap=%lu..%luus, TWAI stopped\r\n",
                static_cast<int>(CAN_TX_GPIO),
                static_cast<unsigned long>(FAULT_PULSE_LOW_MIN_US),
                static_cast<unsigned long>(FAULT_PULSE_LOW_MAX_US),
                static_cast<unsigned long>(FAULT_PULSE_GAP_MIN_US),
                static_cast<unsigned long>(FAULT_PULSE_GAP_MAX_US));
  return true;
}

static bool fault_injection_stop(void) {
  if (!fault_injection_active && twai_running) {
    Serial.println("FAULT already off");
    return true;
  }

  fault_injection_active = false;
  gpio_set_level(CAN_TX_GPIO, 1);
  delay(FAULT_OFF_SETTLE_MS);

  bool ok = can_init();
  dashboard.bus_ok = ok;
  dashboard.fault_off_count++;

  Serial.printf("FAULT OFF: TXD GPIO%d HIGH, TWAI 500 kbit/s %s\r\n",
                static_cast<int>(CAN_TX_GPIO),
                ok ? "restored" : "restore failed");
  return ok;
}

static void fault_injection_service(void) {
  uint32_t low_us;
  uint32_t gap_us;

  if (!fault_injection_active) {
    return;
  }

  uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_fault_pulse_us) < fault_next_gap_us) {
    return;
  }

  gap_us = fault_next_gap_us;
  low_us = fault_random_range(FAULT_PULSE_LOW_MIN_US, FAULT_PULSE_LOW_MAX_US);
  dashboard.current_low_us = low_us;
  dashboard.current_gap_us = gap_us;

  gpio_set_level(CAN_TX_GPIO, 0);
  esp_rom_delay_us(low_us);
  gpio_set_level(CAN_TX_GPIO, 1);

  last_fault_pulse_us = micros();
  fault_next_gap_us = fault_random_range(FAULT_PULSE_GAP_MIN_US, FAULT_PULSE_GAP_MAX_US);
  dashboard.fault_pulse_count++;
}

static uint32_t fault_random_range(uint32_t min_value, uint32_t max_value) {
  uint32_t span = max_value - min_value + 1U;

  fault_prng_state = fault_prng_state * 1664525UL + 1013904223UL;
  return min_value + ((fault_prng_state >> 8) % span);
}
#endif

static void make_demo_frame(CanFrame *frame) {
  static uint8_t counter = 0;

  frame->id = CAN_STM32_TX_ID;
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

static void handle_frame(const CanFrame *frame) {
  dashboard.rx_count++;
  dashboard.bus_ok = true;
  if (frame->id == CAN_STM32_TX_ID) {
    dashboard.stm32_rx_count++;
  } else {
    dashboard.other_rx_count++;
  }

  dashboard.last_id = frame->id;
  dashboard.last_counter = frame->dlc > 0 ? frame->data[0] : 0;
  dashboard.last_update_ms = millis();

  for (uint8_t i = 0; i < sizeof(dashboard.last_data); i++) {
    dashboard.last_data[i] = i < frame->dlc ? frame->data[i] : 0;
  }
}

static void ui_create(void) {
  lv_obj_t *screen = lv_scr_act();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), 0);
  lv_obj_set_style_text_color(screen, lv_color_hex(0xEAF2F8), 0);
  lv_obj_set_style_text_font(screen, &lv_font_montserrat_14, 0);

  lv_obj_t *title = lv_label_create(screen);
  lv_label_set_text(title, "CAN Monitor");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 14, 12);

  bus_value = ui_make_value(screen, "Bus", 42);
  mode_value = ui_make_value(screen, "Mode", 66);
  rx_value = ui_make_value(screen, "RX Count", 90);
  tx_value = ui_make_value(screen, "TX Count", 114);
  err_value = ui_make_value(screen, "Errors", 138);
  id_value = ui_make_value(screen, "Last ID", 162);
  counter_value = ui_make_value(screen, "Counter", 186);

  lv_obj_t *data_title = lv_label_create(screen);
  lv_label_set_text(data_title, "Data");
  lv_obj_set_style_text_color(data_title, lv_color_hex(0x9FB3C8), 0);
  lv_obj_align(data_title, LV_ALIGN_TOP_LEFT, 14, 214);

  data_value = lv_label_create(screen);
  lv_obj_set_style_text_font(data_value, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(data_value, lv_color_hex(0x5EEAD4), 0);
  lv_obj_align(data_value, LV_ALIGN_TOP_LEFT, 14, 238);

  age_value = lv_label_create(screen);
  lv_obj_set_style_text_color(age_value, lv_color_hex(0x9FB3C8), 0);
  lv_obj_align(age_value, LV_ALIGN_BOTTOM_LEFT, 14, -10);
}

static lv_obj_t *ui_make_value(lv_obj_t *parent, const char *name, int y) {
  lv_obj_t *label = lv_label_create(parent);
  lv_label_set_text(label, name);
  lv_obj_set_style_text_color(label, lv_color_hex(0x9FB3C8), 0);
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 14, y);

  lv_obj_t *value = lv_label_create(parent);
  lv_obj_set_style_text_font(value, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(value, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(value, LV_ALIGN_TOP_RIGHT, -14, y - 2);
  return value;
}

static void ui_update(void) {
  char buf[96];

#if BUS_OFF_TEST
  lv_label_set_text(bus_value, fault_injection_active ? "FAULT" : (dashboard.bus_ok ? "OK" : "WAIT"));
  lv_label_set_text(mode_value, fault_injection_active ? "PULSE" : (USE_REAL_CAN ? "CAN" : "DEMO"));
#else
  lv_label_set_text(bus_value, dashboard.bus_ok ? "OK" : "WAIT");
  lv_label_set_text(mode_value, USE_REAL_CAN ? "CAN" : "DEMO");
#endif

  snprintf(buf, sizeof(buf), "%lu/%lu",
           static_cast<unsigned long>(dashboard.rx_count),
           static_cast<unsigned long>(dashboard.stm32_rx_count));
  lv_label_set_text(rx_value, buf);

  snprintf(buf, sizeof(buf), "%lu", static_cast<unsigned long>(dashboard.tx_count));
  lv_label_set_text(tx_value, buf);

  snprintf(buf, sizeof(buf), "%lu/%lu",
           static_cast<unsigned long>(dashboard.err_count),
           static_cast<unsigned long>(dashboard.tx_fail_count));
  lv_label_set_text(err_value, buf);

  snprintf(buf, sizeof(buf), "0x%03lX", static_cast<unsigned long>(dashboard.last_id));
  lv_label_set_text(id_value, buf);

  snprintf(buf, sizeof(buf), "%u", dashboard.last_counter);
  lv_label_set_text(counter_value, buf);

  snprintf(buf, sizeof(buf), "%02X %02X %02X %02X\n%02X %02X %02X %02X",
           dashboard.last_data[0], dashboard.last_data[1],
           dashboard.last_data[2], dashboard.last_data[3],
           dashboard.last_data[4], dashboard.last_data[5],
           dashboard.last_data[6], dashboard.last_data[7]);
  lv_label_set_text(data_value, buf);

  uint32_t age = dashboard.last_update_ms == 0 ? 0 : millis() - dashboard.last_update_ms;
  snprintf(buf, sizeof(buf), "Updated %lums ago", static_cast<unsigned long>(age));
  lv_label_set_text(age_value, buf);
}
